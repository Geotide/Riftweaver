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

// Function-entry detours with a prologue-byte check. A hook is only written when the bytes at the resolved
// address equal what was read from the 1.7.104.0 exe offline; otherwise it is logged and skipped.
namespace hooks
{
	struct Spec
	{
		const char*                  name;
		std::uint64_t                id;        // AE column of the Address Library
		std::array<std::uint8_t, 16> prologue;  // first 16 bytes of the function in SkyrimSE.exe 1.7.104.0
	};

	bool          Init();
	bool          Install(const Spec& a_spec, void* a_detour, void** a_original);
	std::uint64_t Address(std::uint64_t a_id);  // absolute address of an Address Library id
}
