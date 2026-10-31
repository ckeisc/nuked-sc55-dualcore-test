// Forced include for the MSVC host build of mcu_timer.cpp only.
#pragma once
#include <intrin.h>
#define __builtin_ctz(x) ((unsigned)_tzcnt_u32((unsigned)(x)))
