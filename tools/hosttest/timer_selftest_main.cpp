// Host harness for the round-3 lazy timer: runs TIMER_SelfTest (legacy vs
// lazy, with and without fault injection) on the build machine, for both
// step tables, with long sequences. Exit 0 = plain runs 0 mismatches AND
// injected runs > 0 mismatches for every table.
#include <cstdint>
#include <cstdio>

#include "mcu_timer.h"
#include "mcu.h"

// mcu_timer.cpp calls this only when no interrupt sink is installed; the
// self-test always installs sinks, so a stub is enough to link.
void MCU_Interrupt_SetRequest(mcu_t&, MCU_Interrupt_Source, bool)
{
}

int main()
{
    int rc = 0;
    const uint32_t steps = 3000000;
    for (int mk1 = 1; mk1 >= 0; --mk1)
    {
        const uint64_t plain  = TIMER_SelfTest(mk1 != 0, steps, false);
        const uint64_t inject = TIMER_SelfTest(mk1 != 0, steps, true);
        std::printf("table=%s steps=%u plain_mismatches=%llu inject_mismatches=%llu -> %s\n",
                    mk1 ? "MK1" : "GENERIC", steps,
                    (unsigned long long)plain, (unsigned long long)inject,
                    (plain == 0 && inject > 0) ? "PASS" : "FAIL");
        if (!(plain == 0 && inject > 0))
            rc = 1;
    }
    return rc;
}
