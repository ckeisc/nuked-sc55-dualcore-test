// Host stand-in for split_thread_circle.h (no Circle, no AArch64 asm):
// exactly the symbols pcm.cpp needs, as no-ops. g_splitT stays null, so
// every split hook in the variants is skipped, as in the single-threaded
// QEMU harness.
#pragma once
#include <cstdint>
#include "mcu.h"
#include "mcu_interrupt.h"
#include "pcm.h"

static inline uint64_t PMU_ReadCycles() { return 0; }

struct PCMVariantDesc {
    int number;
    const char* short_name;
    const char* full_name;
    void (*update)(pcm_t& pcm, uint64_t cycles);
};
extern const PCMVariantDesc g_pcmVariants[];
extern const int g_pcmVariantCount;
void PCM_Update_Variant(pcm_t& pcm, uint64_t cycles, int variant);

struct ThreadSplitStateCircle {
    volatile uint64_t mix_cyc{0};
    volatile uint64_t nop_cyc3{0};
    void DrainBeforeTick(pcm_t&) {}
    void PublishRenderedStep(pcm_t&, uint64_t) {}
};
extern ThreadSplitStateCircle* g_splitT;

inline void SplitAssertIrq0(pcm_t& pcm)
{
    MCU_Interrupt_SetRequest(*pcm.mcu, INTERRUPT_SOURCE_IRQ0, 1);
}
