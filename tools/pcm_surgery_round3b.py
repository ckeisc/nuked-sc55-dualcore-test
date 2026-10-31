"""Round 3b surgery on the patched pcm.cpp:
 1. #if defined(__aarch64__) guards around the NEON-only variants + table entries
 2. PCMRomBanks / PCM_RomFetch helpers + PCM_RomFastSelfTest after PCM_ReadROM
 3. four generated variants: RomFast, ScalarAcc, RomAcc (scalar, from the baseline
    text) and LayoutRom (aarch64, from the Layout text)
 4. table entries 6..9
All transforms are assertion-checked textual surgery, the same method rounds 1-3 used.
"""
import re
import sys

P = sys.argv[1]
s = open(P, encoding='utf-8').read()


def func(src, name):
    i = src.index('void ' + name + '(pcm_t& pcm, uint64_t cycles)\n{')
    j = src.index('\n}\n', i) + 3
    return i, j


def rep1(src, old, new, label):
    n = src.count(old)
    assert n == 1, f'{label}: expected 1 got {n}'
    return src.replace(old, new)


# ---------------------------------------------------------------- 1. guards
i_neon, _ = func(s, 'PCM_Update_Neon2')
blk = s.rfind('\n\n', 0, i_neon) + 2
_, j_layout = func(s, 'PCM_Update_Layout')
guard_open = '''// ---------------------------------------------------------------------------
// NEON variants (3 neon2, 4 splitacc, 5 layout) carry AArch64 inline asm.
// They exist only on AArch64 builds (the QEMU harness and the kernel); a
// hosted build of the backend on another ISA skips them and their table
// entries, so the scalar variants stay testable everywhere.
// ---------------------------------------------------------------------------
#if defined(__aarch64__)
'''
s = s[:blk] + guard_open + s[blk:j_layout] + '#endif // defined(__aarch64__)\n' + s[j_layout:]

old_tbl = '''    {3, "neon2", "round-2 NEON mix-tail v2 (no lane inserts, full-width loads)", PCM_Update_Neon2},
    {4, "splitacc", "round-3 NEON v2 block + even/odd split accumulators", PCM_Update_Splitacc},
    {5, "layout", "round-3 cross-slot NEON, no per-slot packing", PCM_Update_Layout},
};
'''
new_tbl = '''#if defined(__aarch64__)
    {3, "neon2", "round-2 NEON mix-tail v2 (no lane inserts, full-width loads)", PCM_Update_Neon2},
    {4, "splitacc", "round-3 NEON v2 block + even/odd split accumulators", PCM_Update_Splitacc},
    {5, "layout", "round-3 cross-slot NEON, no per-slot packing", PCM_Update_Layout},
#endif
    // Round 3b (scalar, host-verified against variant 0 by tools/hosttest):
    {6, "romfast", "round-3b per-slot ROM bank pointers: PCM_ReadROM's bank switch resolved once per slot, 5 fetches become indexed loads", PCM_Update_RomFast},
    {7, "scalacc", "round-3b exact register accumulators for the mix bus / reverb / chorus sums (scalar form of B2; no per-slot bus load/store)", PCM_Update_ScalarAcc},
    {8, "romacc", "round-3b romfast + scalacc combined", PCM_Update_RomAcc},
#if defined(__aarch64__)
    {9, "layoutrom", "round-3b layout (cross-slot NEON mix) + romfast fetch path (NEON part unchanged; gate pending)", PCM_Update_LayoutRom},
#endif
};
'''
s = rep1(s, old_tbl, new_tbl, 'table')

# ---------------------------------------------------------------- 2. helpers
i_rr = s.index('uint8_t PCM_ReadROM(pcm_t& pcm, uint32_t address)')
j_rr = s.index('\n    return 0;\n}\n', i_rr) + len('\n    return 0;\n}\n')
helpers = r'''
// ---------------------------------------------------------------------------
// Round 3b (RomFast): PCM_ReadROM resolved once per slot.
//
// PCM_ReadROM maps a 24-bit ROM address ((hiaddr << 20) | a, a < 2^20) to a
// byte through a bank switch that depends only on hiaddr and on bit 19 of
// a (bank = (address >> 19) & 7, or (address >> 21) & 7 when config bit
// 0x20 is set), then indexes one of the wave ROM arrays with a mask that,
// for a < 2^20, reduces to a itself (plus (hiaddr & 1) << 20 for the mkII
// 2 MB bank 0). The voice loop calls it 5 times per slot per tick with the
// same hiaddr (~90 instructions per slot including the call, 11% of the
// PCM core's instructions in the round-1 attribution). PCM_RomBanks
// resolves the two possible banks (a bit 19 clear / set) once per slot;
// PCM_RomFetch is then a select + indexed load. Any case the fast path
// does not cover (JV880 mappings, a >= 2^20) falls back to PCM_ReadROM, so
// the result is PCM_ReadROM's by construction; PCM_RomFastSelfTest proves
// it exhaustively (every hiaddr, every a, both config modes) against
// whatever the wave ROM arrays hold.
// ---------------------------------------------------------------------------
struct PCMRomBanks
{
    const uint8_t* lo;   // bank for 20-bit addresses with bit 19 clear (nullptr: reads 0)
    const uint8_t* hi;   // bank for bit 19 set
    bool           fast; // false: use PCM_ReadROM for every fetch
};

static inline const uint8_t* PCM_RomBankPtr(const pcm_t& pcm, int bank, int hiaddr)
{
    switch (bank)
    {
        case 0:
            // MK1: waverom1[address & 0xfffff] = waverom1[a];
            // mkII: waverom1[address & 0x1fffff] = waverom1[((hiaddr & 1) << 20) | a].
            return pcm.mcu->is_mk1 ? pcm.waverom1 : pcm.waverom1 + ((uint32_t)(hiaddr & 1) << 20);
        case 1:
            return pcm.waverom2; // !jv880: waverom2[address & 0xfffff]
        case 2:
            return pcm.waverom3; // !jv880: waverom3[address & 0xfffff]
        default:
            return nullptr;      // banks 3-7: 0 on non-JV880
    }
}

static inline PCMRomBanks PCM_RomBanks(const pcm_t& pcm, int hiaddr)
{
    PCMRomBanks b{nullptr, nullptr, false};
    if (pcm.mcu->is_jv880)
        return b;
    b.fast = true;
    if (pcm.config_reg_3d & 0x20)
    {
        const int bank = ((hiaddr << 20) >> 21) & 7;
        b.lo = PCM_RomBankPtr(pcm, bank, hiaddr);
        b.hi = b.lo;
    }
    else
    {
        b.lo = PCM_RomBankPtr(pcm, ((hiaddr << 20) >> 19) & 7, hiaddr);
        b.hi = PCM_RomBankPtr(pcm, (((hiaddr << 20) | 0x80000) >> 19) & 7, hiaddr);
    }
    return b;
}

static inline uint8_t PCM_RomFetch(pcm_t& pcm, const PCMRomBanks& b, int hiaddr, uint32_t a)
{
    if (!b.fast || (a >> 20) != 0)
        return PCM_ReadROM(pcm, (uint32_t)((hiaddr << 20) | a));
    const uint8_t* p = (a & 0x80000u) ? b.hi : b.lo;
    return p ? p[a] : (uint8_t)0;
}

// Exhaustive equivalence check of the fast fetch against PCM_ReadROM for
// the current mcu flags: both config modes, every hiaddr, every 20-bit
// address, against whatever the wave ROM arrays hold (random contents in
// the host test; the real ROMs on the device). ~64M fetches. Returns the
// number of mismatches (0 = pass). Restores config_reg_3d.
uint64_t PCM_RomFastSelfTest(pcm_t& pcm)
{
    uint64_t mismatches = 0;
    const uint8_t saved = pcm.config_reg_3d;
    for (int mode = 0; mode < 2; ++mode)
    {
        pcm.config_reg_3d = (uint8_t)((saved & ~0x20) | (mode ? 0x20 : 0));
        for (int hiaddr = 0; hiaddr < 16; ++hiaddr)
        {
            const PCMRomBanks b = PCM_RomBanks(pcm, hiaddr);
            for (uint32_t a = 0; a < 0x100000u; ++a)
            {
                if (PCM_RomFetch(pcm, b, hiaddr, a) != PCM_ReadROM(pcm, (uint32_t)((hiaddr << 20) | a)))
                    ++mismatches;
            }
        }
    }
    pcm.config_reg_3d = saved;
    return mismatches;
}
'''
s = s[:j_rr] + helpers + s[j_rr:]

# ---------------------------------------------------------------- 3. variants
i_b, j_b = func(s, 'PCM_Update')
base = s[i_b:j_b]
i_l, j_l = func(s, 'PCM_Update_Layout')
layout = s[i_l:j_l]

FETCH_RE = re.compile(r'PCM_ReadROM\(pcm, \(uint32_t\)\(\(hiaddr << 20\) \| (\w+)\)\)')


def rename(body, old, new):
    return rep1(body, 'void ' + old + '(pcm_t& pcm, uint64_t cycles)', 'void ' + new + '(pcm_t& pcm, uint64_t cycles)', 'rename ' + new)


def romfast(body):
    old = '            int hiaddr = (ram2[7] >> 8) & 15; // 1\n'
    body = rep1(body, old, old + '''            // RomFast (round 3b): resolve PCM_ReadROM's bank switch once per
            // slot; the 5 fetches below become select + indexed load.
            // PCM_RomFetch falls back to PCM_ReadROM for anything the fast
            // path does not cover, so the bytes are PCM_ReadROM's exactly.
            const PCMRomBanks rombanks = PCM_RomBanks(pcm, hiaddr);
''', 'hiaddr anchor')
    assert len(FETCH_RE.findall(body)) == 5, len(FETCH_RE.findall(body))
    return FETCH_RE.sub(r'PCM_RomFetch(pcm, rombanks, hiaddr, (uint32_t)(\1))', body)


MIX_TAIL_NEW = '''            // ScalarAcc (round 3b): the scalar form of the chain-break idea
            // (catalog B2, never tried as a scalar change). The original
            // accumulates through MEMORY: per slot it loads and stores
            // pcm.ram1[31][1], pcm.ram1[31][3], pcm.rcsum[0], pcm.rcsum[1]
            // (8 AGU slots on a single-AGU core) and the stores alias the
            // slot's own uint32 ram1 pointer, forcing reloads. Here each
            // slot's exact terms (sx20(term) + cin, plus its slot2
            // injection terms under the original two switches' mapping,
            // merged) go to four register accumulators, combined once per
            // tick. Exactness: addclip20(a, b, cin) = sx20(a) + sx20(b) +
            // cin and sx20 is reduction mod 2^20 to [-2^19, 2^19), so the
            // serial chain's value at any sx20 read equals the mod-2^20 sum
            // of all raw terms and cin bits regardless of grouping; every
            // downstream reader of accum_l/r and rcsum observes them through
            // sx20 (addclip20 / multi) or through write_mask (low bits).
            // Terms are < 2^19 + 1 in magnitude, <= 32 + 6 of them: no int32
            // overflow. Same argument and same combine as splitacc/layout;
            // ram1[31][1]/[3] stay at their pre-loop zero (valid only while
            // slot 31 is outside the loop -- see the reg_slots guard).
            const int32_t tL0 = sx20(sampl >> 6) + ((sampl >> 5) & 1);
            const int32_t tR0 = sx20(sampr >> 6) + ((sampr >> 5) & 1);
            const int32_t tC00 = sx20(rc0 >> 1) + (rc0 & 1);
            const int32_t tC10 = sx20(rc1 >> 1) + (rc1 & 1);
            int32_t tL = tL0, tR = tR0, tC0 = tC00, tC1 = tC10;
            const int slot2 = (slot == pcm.config.reg_slots - 1) ? 31 : slot + 1;
            switch (slot2)
            {
                case 17:
                    tL += sx20(rcadd[0] >> 1) + (rcadd[0] & 1);
                    tC1 += sx20(rcadd2[0] >> 1) + (rcadd2[0] & 1);
                    break;
                case 18:
                    tR += sx20(rcadd[1] >> 1) + (rcadd[1] & 1);
                    tC1 += sx20(rcadd2[1] >> 1) + (rcadd2[1] & 1);
                    break;
                case 21:
                    tL += sx20(rcadd[2] >> 1) + (rcadd[2] & 1);
                    tC0 += sx20(rcadd2[2] >> 1) + (rcadd2[2] & 1);
                    break;
                case 22:
                    tR += sx20(rcadd[3] >> 1) + (rcadd[3] & 1);
                    tC1 += sx20(rcadd2[3] >> 1) + (rcadd2[3] & 1);
                    break;
                case 23:
                    tL += sx20(rcadd[4] >> 1) + (rcadd[4] & 1);
                    tC0 += sx20(rcadd2[4] >> 1) + (rcadd2[4] & 1);
                    break;
                case 31:
                    tR += sx20(rcadd[5] >> 1) + (rcadd[5] & 1);
                    tC1 += sx20(rcadd2[5] >> 1) + (rcadd2[5] & 1);
                    break;
                default:
                    break;
            }
            accL += tL;
            accR += tR;
            accC0 += tC0;
            accC1 += tC1;
'''

MIX_TAIL_END = '''            else
            {
                pcm.accum_l = suml;
                pcm.accum_r = sumr;
            }
'''


def scalaracc(body):
    loop_start = body.index('        for (int slot = 0; slot < pcm.config.reg_slots; slot++)\n        {\n')
    pmu_idx = body.index('        // PMU (shaped-var): voice-mix cycles for this batch,\n')
    loop_end = body.rindex('        }\n', loop_start, pmu_idx) + len('        }\n')
    loop_text = body[loop_start:loop_end]
    start = loop_text.index('            // mix reverb/chorus?\n')
    end = loop_text.index(MIX_TAIL_END, start) + len(MIX_TAIL_END)
    fast_loop = loop_text[:start] + MIX_TAIL_NEW + loop_text[end:]
    hybrid = '''        // ScalarAcc applies only while slot 31 is outside the loop
        // (reg_slots < 32: always true for the SC-55 firmware, and assumed
        // by the split's 800-cycle tick cap). With 32 slots, slot 31's
        // filter registers ram1[31][1]/[3] ARE the mix bus words (the
        // chip's register file aliases them), so the bus must flow through
        // slot 31's filter in serial order: that tick runs the original
        // loop. The random-state host test (tools/hosttest) found this
        // aliasing; pipe/splitacc/layout have the same exposure.
        if (pcm.config.reg_slots < 32)
        {
        // ScalarAcc: exact register accumulators (see the mix tail below).
        int32_t accL = 0, accR = 0, accC0 = 0, accC1 = 0;
''' + fast_loop + '''        // ScalarAcc combine: congruent mod 2^20 to the original serial
        // sums; every reader observes them through sx20 or write_mask.
        pcm.accum_l = accL;
        pcm.accum_r = accR;
        pcm.rcsum[0] = accC0;
        pcm.rcsum[1] = accC1;
        }
        else
        {
''' + loop_text + '''        }
'''
    return body[:loop_start] + hybrid + body[loop_end:]


v_romfast = rename(romfast(base), 'PCM_Update', 'PCM_Update_RomFast')
v_scalacc = rename(scalaracc(base), 'PCM_Update', 'PCM_Update_ScalarAcc')
v_romacc = rename(scalaracc(romfast(base)), 'PCM_Update', 'PCM_Update_RomAcc')
v_layoutrom = rename(romfast(layout), 'PCM_Update_Layout', 'PCM_Update_LayoutRom')

new_funcs = '''
// ---------------------------------------------------------------------------
// Round 3b variants (generated from the baseline / layout text by
// assertion-checked textual surgery; see the RomFast and ScalarAcc comments
// inside the bodies for the transforms and their exactness arguments).
// ---------------------------------------------------------------------------

''' + v_romfast + '\n' + v_scalacc + '\n' + v_romacc + '''
#if defined(__aarch64__)
''' + v_layoutrom + '''#endif // defined(__aarch64__)
'''
tbl_anchor = '// Variant descriptor table (data-driven dispatch'
k = s.index(tbl_anchor)
s = s[:k] + new_funcs + '\n' + s[k:]

open(P, 'w', encoding='utf-8', newline='\n').write(s)
print('surgery ok; variants:', re.findall(r'void (PCM_Update\w*)\(pcm_t& pcm, uint64_t cycles\)\n\{', s))
