// Riftweaver - Copyright (C) 2026 Ph3nixxSTRise
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>

// Copy n bytes from a possibly invalid address; false instead of crashing.
inline bool ReadSafe(const void* a_src, void* a_dst, std::size_t a_n)
{
	__try {
		std::memcpy(a_dst, a_src, a_n);
		return true;
	} __except (EXCEPTION_EXECUTE_HANDLER) {
		return false;
	}
}

template <class T>
inline bool ReadSafeT(const void* a_src, T& a_out)
{
	return ReadSafe(a_src, &a_out, sizeof(T));
}
