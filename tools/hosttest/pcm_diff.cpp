// Differential host test: every scalar PCM_Update variant vs the baseline
// PCM_Update on identical, randomized pcm_t states.
//
// For each seed: randomize a pcm_t (registers, config, wave ROMs, eram,
// masks), copy it, run `ticks` ticks through the baseline and through the
// variant, and compare: every posted sample, every interrupt request, and
// the full pcm_t afterwards. Variants that use exact register accumulators
// (scalacc, romacc) legitimately differ in ram1[31][1], ram1[31][3] and in
// the raw accum_l/accum_r/rcsum values; for those the accumulators are
// compared mod 2^20 (the only way the engine ever reads them) and the two
// bus words are skipped. Everything else must be byte-identical.
//
// Also runs PCM_RomFastSelfTest (exhaustive PCM_RomFetch vs PCM_ReadROM).
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "mcu.h"
#include "mcu_interrupt.h"
#include "nuked_profile.h"
#include "pcm.h"
#include "split_stub.h"

uint64_t PCM_RomFastSelfTest(pcm_t& pcm);

ThreadSplitStateCircle* g_splitT = nullptr;
NukedProfileClockFn g_nukedProfileClockFn = nullptr;
static NukedProfileData g_prof;
NukedProfileData& NukedProfile() { return g_prof; }

static std::vector<int32_t> g_samples;
static std::vector<int32_t> g_irqs;

void MCU_PostSample(mcu_t&, const AudioFrame<int32_t>& f)
{
    g_samples.push_back(f.left);
    g_samples.push_back(f.right);
}
void MCU_Interrupt_SetRequest(mcu_t&, MCU_Interrupt_Source s, bool v)
{
    g_irqs.push_back((int32_t)s * 2 + (v ? 1 : 0));
}
void MCU_GA_SetGAInt(mcu_t&, uint8_t, bool) {}
void MCU_DefaultSampleCallback(void*, const AudioFrame<int32_t>&) {}

struct Rng
{
    uint64_t s;
    uint32_t Next() { s = s * 6364136223846793005ULL + 1442695040888963407ULL; return (uint32_t)(s >> 33); }
};

static inline int32_t sx20h(int32_t v) { return (int32_t)((uint32_t)v << 12) >> 12; }

static int g_maxslots = 32;
static void Randomize(pcm_t& p, mcu_t& mcu, Rng& r, bool mk1)
{
    mcu.is_mk1 = mk1;
    mcu.is_jv880 = false;
    p.mcu = &mcu;
    for (auto& row : p.ram1)
        for (auto& v : row)
            v = r.Next() & 0xfffff;
    for (auto& row : p.ram2)
        for (auto& v : row)
            v = (uint16_t)r.Next();
    for (auto& v : p.eram)
        v = (uint16_t)r.Next();
    // Wave ROMs: random bytes (fill in 32-bit chunks for speed).
    auto fill = [&](uint8_t* b, size_t n) { for (size_t i = 0; i + 3 < n; i += 4) { uint32_t x = r.Next(); memcpy(b + i, &x, 4); } };
    fill(p.waverom1, WAVEROM1_SIZE);
    fill(p.waverom2, WAVEROM2_SIZE);
    fill(p.waverom3, WAVEROM3_SIZE);
    fill(p.waverom_card, WAVEROM_CARD_SIZE);
    fill(p.waverom_exp, WAVEROM_EXP_SIZE);
    p.cycles = (uint64_t)(r.Next() & 0xffff) * 800;
    p.voice_mask = r.Next();
    p.voice_mask_pending = (r.Next() & 1) ? p.voice_mask : r.Next();
    p.write_latch = r.Next() & 0xfffff;
    p.read_latch = r.Next() & 0xfffff;
    p.wave_read_address = r.Next() & 0xffffff;
    p.tv_counter = (uint16_t)(r.Next() & 0x3fff);
    p.wave_byte_latch = (uint8_t)r.Next();
    p.select_channel = (uint8_t)(r.Next() & 31);
    p.config_reg_3c = (uint8_t)r.Next();
    PCM_GetConfig(p.config, p.config_reg_3c);
    p.config_reg_3d = (uint8_t)r.Next();
    p.config.reg_slots = (uint8_t)((p.config_reg_3d & 31) + 1);
    if (p.config.reg_slots > g_maxslots)
        p.config.reg_slots = (uint8_t)g_maxslots;
    p.irq_channel = (uint8_t)(r.Next() & 31);
    p.irq_assert = (r.Next() & 3) == 0;
    p.voice_mask_updating = (r.Next() & 1) != 0;
    p.nfs = (r.Next() & 7) != 0;
    p.accum_l = (int32_t)(r.Next() & 0xfffff);
    p.accum_r = (int32_t)(r.Next() & 0xfffff);
    p.rcsum[0] = (int32_t)(r.Next() & 0xfffff);
    p.rcsum[1] = (int32_t)(r.Next() & 0xfffff);
    p.enable_oversampling = (r.Next() & 1) != 0;
}

// Compare two states; `acc` relaxes the accumulator rules for the register-
// accumulator variants. Returns a description of the first difference or "".
static const char* Compare(const pcm_t& a, const pcm_t& b, bool acc, char* buf, size_t n)
{
    for (int i = 0; i < 32; ++i)
        for (int j = 0; j < 8; ++j)
        {
            if (acc && i == 31 && (j == 1 || j == 3))
                continue;
            if (a.ram1[i][j] != b.ram1[i][j]) { snprintf(buf, n, "ram1[%d][%d] %08x vs %08x", i, j, a.ram1[i][j], b.ram1[i][j]); return buf; }
        }
    for (int i = 0; i < 32; ++i)
        for (int j = 0; j < 16; ++j)
            if (a.ram2[i][j] != b.ram2[i][j]) { snprintf(buf, n, "ram2[%d][%d] %04x vs %04x", i, j, a.ram2[i][j], b.ram2[i][j]); return buf; }
    if (memcmp(a.eram, b.eram, sizeof(a.eram)) != 0) return "eram";
    if (a.cycles != b.cycles) return "cycles";
    if (a.voice_mask != b.voice_mask || a.voice_mask_pending != b.voice_mask_pending) return "voice_mask";
    if (a.tv_counter != b.tv_counter) return "tv_counter";
    if (a.irq_assert != b.irq_assert) return "irq_assert";
    if (a.irq_channel != b.irq_channel) return "irq_channel";
    if (a.nfs != b.nfs) return "nfs";
    if (acc)
    {
        if (sx20h(a.accum_l) != sx20h(b.accum_l)) { snprintf(buf, n, "accum_l mod 2^20 %d vs %d", sx20h(a.accum_l), sx20h(b.accum_l)); return buf; }
        if (sx20h(a.accum_r) != sx20h(b.accum_r)) return "accum_r mod 2^20";
        if (sx20h(a.rcsum[0]) != sx20h(b.rcsum[0])) return "rcsum[0] mod 2^20";
        if (sx20h(a.rcsum[1]) != sx20h(b.rcsum[1])) return "rcsum[1] mod 2^20";
    }
    else
    {
        if (a.accum_l != b.accum_l || a.accum_r != b.accum_r) return "accum";
        if (a.rcsum[0] != b.rcsum[0] || a.rcsum[1] != b.rcsum[1]) return "rcsum";
    }
    return "";
}

int main(int argc, char** argv)
{
    const int seeds = argc > 1 ? atoi(argv[1]) : 150;
    const int ticks = argc > 2 ? atoi(argv[2]) : 400;
    const int maxslots = argc > 3 ? atoi(argv[3]) : 32;
    g_maxslots = maxslots;
    printf("max reg_slots in test states: %d\n", maxslots);
    pcm_t* base = new pcm_t;
    pcm_t* var = new pcm_t;
    mcu_t* mcuA = new mcu_t;
    mcu_t* mcuB = new mcu_t;
    int rc = 0;

    // Exhaustive ROM fetch equivalence, MK1 and mkII, random ROM contents.
    {
        Rng r{12345};
        for (int mk1 = 1; mk1 >= 0; --mk1)
        {
            Randomize(*base, *mcuA, r, mk1 != 0);
            const uint64_t mm = PCM_RomFastSelfTest(*base);
            printf("romfast selftest %s: mismatches=%llu -> %s\n", mk1 ? "MK1" : "mkII", (unsigned long long)mm, mm == 0 ? "PASS" : "FAIL");
            if (mm) rc = 1;
        }
    }

    for (int vi = 0; vi < g_pcmVariantCount; ++vi)
    {
        const PCMVariantDesc& d = g_pcmVariants[vi];
        if (d.number == 0) continue;
        const bool acc = (d.number == 7 || d.number == 8 || d.number == 4 || d.number == 5 || d.number == 9);
        uint64_t fails = 0;
        char buf[128];
        const char* first = nullptr;
        for (int seed = 0; seed < seeds; ++seed)
        {
            Rng r{0x1234567ULL + (uint64_t)seed * 7919ULL};
            const bool mk1 = (seed % 3) != 2; // 2/3 MK1 (the kernel's target), 1/3 mkII
            Randomize(*base, *mcuA, r, mk1);
            memcpy(var, base, sizeof(pcm_t));
            mcuB->is_mk1 = mcuA->is_mk1;
            mcuB->is_jv880 = mcuA->is_jv880;
            var->mcu = mcuB;
            const uint64_t tick = (uint64_t)(base->config.reg_slots + 1) * 25;
            const uint64_t target = base->cycles + tick * (uint64_t)ticks;

            g_samples.clear(); g_irqs.clear();
            PCM_Update(*base, target);
            std::vector<int32_t> sA = g_samples, iA = g_irqs;
            g_samples.clear(); g_irqs.clear();
            d.update(*var, target);
            const char* diff = Compare(*base, *var, acc, buf, sizeof(buf));
            if (sA != g_samples) diff = "samples";
            else if (iA != g_irqs) diff = "irq requests";
            if (diff[0])
            {
                ++fails;
                if (!first) { static char keep[160]; snprintf(keep, sizeof(keep), "seed %d (%s): %s", seed, mk1 ? "MK1" : "mkII", diff); first = keep; }
            }
        }
        printf("variant %d %-9s seeds=%d ticks=%d mismatching_seeds=%llu -> %s%s%s\n", d.number, d.short_name, seeds, ticks,
               (unsigned long long)fails, fails ? "FAIL" : "PASS", first ? "  first: " : "", first ? first : "");
        if (fails) rc = 1;
    }
    return rc;
}
