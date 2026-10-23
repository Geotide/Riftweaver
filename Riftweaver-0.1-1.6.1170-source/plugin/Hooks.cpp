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

#include "PCH.h"

#include "Hooks.h"

#include "MinHook.h"

namespace hooks
{
	bool Init()
	{
		const auto st = MH_Initialize();
		if (st != MH_OK && st != MH_ERROR_ALREADY_INITIALIZED) {
			logger::error("MH_Initialize failed: {}", MH_StatusToString(st));
			return false;
		}
		return true;
	}

	std::uint64_t Address(std::uint64_t a_id)
	{
		return REL::ID(a_id).address();
	}

	bool Install(const Spec& a_spec, void* a_detour, void** a_original)
	{
		const auto addr = Address(a_spec.id);
		const auto* bytes = reinterpret_cast<const std::uint8_t*>(addr);
		if (std::memcmp(bytes, a_spec.prologue.data(), a_spec.prologue.size()) != 0) {
			std::string got, want;
			for (std::size_t i = 0; i < a_spec.prologue.size(); ++i) {
				got += std::format("{:02X} ", bytes[i]);
				want += std::format("{:02X} ", a_spec.prologue[i]);
			}
			logger::error("hook {} (id {}) at {:#x}: prologue mismatch, NOT hooked. want {}got  {}", a_spec.name, a_spec.id, addr, want, got);
			return false;
		}
		auto st = MH_CreateHook(reinterpret_cast<void*>(addr), a_detour, a_original);
		if (st != MH_OK) {
			logger::error("hook {} (id {}): MH_CreateHook failed: {}", a_spec.name, a_spec.id, MH_StatusToString(st));
			return false;
		}
		st = MH_EnableHook(reinterpret_cast<void*>(addr));
		if (st != MH_OK) {
			logger::error("hook {} (id {}): MH_EnableHook failed: {}", a_spec.name, a_spec.id, MH_StatusToString(st));
			return false;
		}
		logger::info("hooked {} (id {}) at {:#x} (exe rva {:#x}), prologue verified", a_spec.name, a_spec.id, addr, addr - REL::Module::get().base());
		return true;
	}
}
