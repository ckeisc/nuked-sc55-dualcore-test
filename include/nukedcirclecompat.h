//
// nukedcirclecompat.h
//
// mt32-pi - A baremetal MIDI synthesizer for Raspberry Pi
// Copyright (C) 2020-2023 Dale Whinham <daleyo@gmail.com>
//
// This file is part of mt32-pi.
//
// mt32-pi is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// mt32-pi is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
// details.
//
// You should have received a copy of the GNU General Public License along with
// mt32-pi. If not, see <http://www.gnu.org/licenses/>.
//

#ifndef _nukedcirclecompat_h
#define _nukedcirclecompat_h

// Compatibility shims for building the Nuked-SC55 backend (which targets
// C++23) with the Circle toolchain's GCC 11 libstdc++.
//
// This header is force-included into every backend translation unit (see
// the "nuked" target in the Makefile). It supplies what this libstdc++
// lacks:
//
// - std::mutex: the aarch64-none-elf libstdc++ is built without gthread
//   support, so <mutex> declares std::scoped_lock but no std::mutex.
//   The backend's only mutex guards the LCD framebuffer hand-off to an
//   LCD thread; the mt32-pi integration never starts one (the LCD
//   backend is null), and CNukedSC55Synth serializes every emulator
//   entry point with its CSynthBase spinlock, so a no-op mutex is
//   sound here (same reasoning as the libEmuSC shim).
// - std::unreachable() (C++23, added in GCC 12) and std::byteswap()
//   (C++23, added in GCC 13): both map directly onto GCC builtins with
//   identical semantics, version-guarded so a future toolchain with
//   the real library functions uses those instead.

#include <cstdint>
#include <type_traits>

namespace std
{
	class mutex
	{
	public:
		constexpr mutex() noexcept = default;
		mutex(const mutex&) = delete;
		mutex& operator=(const mutex&) = delete;

		void lock() noexcept { }
		bool try_lock() noexcept { return true; }
		void unlock() noexcept { }
	};

}

#if defined(__GNUC__) && !defined(__clang__)

#if __GNUC__ < 12
namespace std
{
	[[noreturn]] inline void unreachable()
	{
		__builtin_unreachable();
	}
}
#endif

#if __GNUC__ < 13
namespace std
{
	template <typename T>
	constexpr std::enable_if_t<std::is_integral_v<T>, T> byteswap(T nValue) noexcept
	{
		if constexpr (sizeof(T) == 2)
			return static_cast<T>(__builtin_bswap16(static_cast<uint16_t>(nValue)));
		else if constexpr (sizeof(T) == 4)
			return static_cast<T>(__builtin_bswap32(static_cast<uint32_t>(nValue)));
		else if constexpr (sizeof(T) == 8)
			return static_cast<T>(__builtin_bswap64(static_cast<uint64_t>(nValue)));
		else
			return nValue;
	}
}
#endif

#endif

#endif
