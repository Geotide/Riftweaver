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

#include "Api.h"

#include <dbghelp.h>

// Logs fatal-looking exceptions (access violations, illegal instructions, heap corruption, stack overflow) with the faulting module and offset, the registers and a short stack walk
// into Riftweaver.log, then lets the exception continue to whoever handles it. The game has no crash handler of its own here; this exists so a crash during a test leaves a trace.
// First-chance exceptions that the engine handles itself (some of its code uses SEH) are logged too, marked as such, up to a small limit.
namespace crashlog
{
	namespace
	{
		std::atomic<int> g_count{ 0 };
		PVOID            g_handler{ nullptr };

		std::string Where(const void* a_addr)
		{
			HMODULE mod = nullptr;
			if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, static_cast<LPCSTR>(a_addr), &mod) || !mod) {
				return std::format("{:p} (no module)", a_addr);
			}
			char path[MAX_PATH]{};
			GetModuleFileNameA(mod, path, MAX_PATH);
			const char* name = std::strrchr(path, '\\');
			return std::format("{}+{:#x}", name ? name + 1 : path, reinterpret_cast<std::uintptr_t>(a_addr) - reinterpret_cast<std::uintptr_t>(mod));
		}

		bool ReadWord(const std::uintptr_t* a_p, std::uintptr_t& a_out)  // no C++ objects here: __try cannot share a function with unwinding
		{
			__try {
				a_out = *a_p;
				return true;
			} __except (EXCEPTION_EXECUTE_HANDLER) {
				return false;
			}
		}

		LONG CALLBACK Handler(EXCEPTION_POINTERS* a_ep)
		{
			const auto code = a_ep->ExceptionRecord->ExceptionCode;
			const bool fatal = code == EXCEPTION_ACCESS_VIOLATION || code == EXCEPTION_ILLEGAL_INSTRUCTION || code == EXCEPTION_STACK_OVERFLOW || code == EXCEPTION_INT_DIVIDE_BY_ZERO ||
			                   code == EXCEPTION_PRIV_INSTRUCTION || code == 0xC0000374 /* heap corruption */ || code == EXCEPTION_ARRAY_BOUNDS_EXCEEDED || code == 0xC0000409 /* fail fast */;
			if (!fatal || g_count.fetch_add(1) >= 12) {
				return EXCEPTION_CONTINUE_SEARCH;
			}
			auto* rec = a_ep->ExceptionRecord;
			auto* c = a_ep->ContextRecord;
			logger::error("crash: exception {:#x} at {} (first chance, may be handled by the engine); access type {} address {:#x}", code, Where(rec->ExceptionAddress),
				rec->NumberParameters > 0 ? static_cast<std::uint64_t>(rec->ExceptionInformation[0]) : 0, rec->NumberParameters > 1 ? static_cast<std::uint64_t>(rec->ExceptionInformation[1]) : 0);
			logger::error("crash: rip {:#x} rsp {:#x} rax {:#x} rbx {:#x} rcx {:#x} rdx {:#x} rsi {:#x} rdi {:#x} r8 {:#x} r9 {:#x}", c->Rip, c->Rsp, c->Rax, c->Rbx, c->Rcx, c->Rdx, c->Rsi, c->Rdi, c->R8, c->R9);
			void*      frames[32]{};
			const auto n = RtlCaptureStackBackTrace(0, 32, frames, nullptr);
			for (USHORT i = 0; i < n; ++i) {
				logger::error("crash:   #{} {}", i, Where(frames[i]));
			}
			// the stack words around rsp that point into modules: a cheap stand-in for the frames the walk above cannot see through the faulting frame
			const auto* sp = reinterpret_cast<const std::uintptr_t*>(c->Rsp);
			int         shown = 0;
			for (int i = 0; i < 96 && shown < 12; ++i) {
				std::uintptr_t v = 0;
				if (!ReadWord(sp + i, v)) {
					break;
				}
				HMODULE m = nullptr;
				if (v > 0x10000 && GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCSTR>(v), &m) && m) {
					logger::error("crash:   stack[{}] {}", i, Where(reinterpret_cast<const void*>(v)));
					++shown;
				}
			}
			if (auto l = spdlog::default_logger()) {
				l->flush();
			}
			return EXCEPTION_CONTINUE_SEARCH;
		}
	}

	void Install()
	{
		if (!g_handler) {
			g_handler = AddVectoredExceptionHandler(1, Handler);
			logger::info("crashlog: exception logger installed");
		}
	}
}
