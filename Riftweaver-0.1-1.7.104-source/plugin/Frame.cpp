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
#include "Hooks.h"

#include <TlHelp32.h>
#include <thread>

namespace frame
{
	namespace
	{
		// Prologue read from SkyrimSE.exe 1.7.104.0 (scripts/prologue.py). Called once per rendered frame while in game, not at the main menu.
		const hooks::Spec kFrameFn{ "frame fn (id 36559)", 36559, { 0x48, 0x8B, 0xC4, 0x89, 0x50, 0x10, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41 } };

		// Main::RenderWorld: the main world pass (opaque, sky, transparent, particles); post-processing and UI come after it returns.
		const hooks::Spec kRenderWorld{ "Main::RenderWorld", 107142, { 0x48, 0x8B, 0xC4, 0x56, 0x57, 0x41, 0x54, 0x41, 0x56, 0x41, 0x57, 0x48, 0x81, 0xEC, 0x40, 0x01 } };

		// The main world draw (cull + SubmitAccumulator with flags 3), called once per frame by the frame function after the engine's per-frame setup (camera data,
		// shadows, lights, weather) and before Main::RenderWorld. Two bool arguments. Read from SkyrimSE.exe 1.7.104.0 (scripts/calls.py 107139, prologue.py).
		const hooks::Spec kWorldDraw{ "main world draw (id 107139)", 107139, { 0x40, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0xAC, 0x24 } };
		// ---- G234: trace of the engine's sun shadow calls ----
		std::atomic<bool> g_shFence{ false };
		std::atomic<int> g_shLeft{ 0 };  // frames still to trace (counted down at the frame function)
		bool             g_shInstalled{ false };
		std::uint64_t    g_shOrig[0x20]{};
		double ShNow() { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
		std::uintptr_t ShRva(void* a_ra) { return reinterpret_cast<std::uintptr_t>(a_ra) - REL::Module::get().base(); }
		template <int N>
		std::uint64_t ShWrap(void* a, void* b, void* c, void* d, void* e)
		{
			const bool log = g_shLeft.load() > 0;
			const double t0 = log ? ShNow() : 0.0;
			if (log) {
				logger::info("shtrace: frame {} -> light slot {:#x} this {:p} a {:p} b {:p} c {:p} d {:p} from rva {:#x}", frame::Count(), N, a, b, c, d, e, ShRva(_ReturnAddress()));
			}
			const auto r = reinterpret_cast<std::uint64_t (*)(void*, void*, void*, void*, void*)>(g_shOrig[N])(a, b, c, d, e);
			if (g_shFence.load() && N == 0xA) {
				view::GpuFence("ENGINE sun light Render (the real camera's shadow maps)");
			}
			if (N == 0xA) {
				view::LogSunShadow("ENGINE: after the sun light's Render (slot 0xA)");
			}
			if (log) {
				logger::info("shtrace: frame {} <- light slot {:#x} took {:.3f} ms", frame::Count(), N, ShNow() - t0);
			}
			return r;
		}
		const hooks::Spec kShadowStage{ "shadow stage (id 107132)", 107132, { 0x40, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8D, 0x6C, 0x24 } };
		using ShadowStage_t = std::uint64_t (*)(void*, void*, void*, void*);
		ShadowStage_t g_origShadowStage{ nullptr };
		std::uint64_t Hook_ShadowStage(void* a, void* b, void* c, void* d)
		{
			const bool log = g_shLeft.load() > 0;
			const double t0 = log ? ShNow() : 0.0;
			if (log) {
				logger::info("shtrace: frame {} -> SHADOW STAGE (id 107132) a {:p} b {:p} from rva {:#x}", frame::Count(), a, b, ShRva(_ReturnAddress()));
			}
			const auto r = g_origShadowStage(a, b, c, d);
			view::LogSunShadow("ENGINE: after its shadow stage 107132");
			if (g_shFence.load()) {
				view::GpuFence("ENGINE shadow stage 107132 done");
			}
			if (log) {
				logger::info("shtrace: frame {} <- SHADOW STAGE took {:.3f} ms", frame::Count(), ShNow() - t0);
			}
			return r;
		}
		// The shadow map render of Main::Render's setup (id 108492, called by the frame function's shadow stage id 107132 after the lights are updated and before the world cull jobs start).
		const hooks::Spec kShadowMaps{ "shadow maps (id 108492)", 108492, { 0x40, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x8B, 0xEC, 0x48 } };
		using ShadowMaps_t = std::uint64_t (*)(void*, void*, void*, void*, void*, void*, void*, void*);
		ShadowMaps_t g_origShadowMaps{ nullptr };
		bool         g_inShadowHook{ false };
		std::uint64_t Hook_ShadowMaps(void* a1, void* a2, void* a3, void* a4, void* a5, void* a6, void* a7, void* a8)
		{
			const auto ra = reinterpret_cast<std::uintptr_t>(_ReturnAddress()) - REL::Module::get().base();
			const bool shLog = g_shLeft.load() > 0;
			const double shT0 = shLog ? ShNow() : 0.0;
			if (shLog) {
				logger::info("shtrace: frame {} -> shadow-map cull (id 108492) a1 {:p} a2 {:p} a3 {:p} a4 {:p} from rva {:#x}", frame::Count(), a1, a2, a3, a4, ra);
			}
			const auto r = g_origShadowMaps(a1, a2, a3, a4, a5, a6, a7, a8);
			if (shLog) {
				logger::info("shtrace: frame {} <- shadow-map cull took {:.3f} ms", frame::Count(), ShNow() - shT0);
			}
			if (view::P().when == 3 && !g_inShadowHook && ra >= 0x1537fd0 && ra < 0x1537fd0 + 1111) {
				g_inShadowHook = true;
				view::RenderViews();
				g_inShadowHook = false;
			}
			return r;
		}
		using WorldDraw_t = void (*)(bool, bool);
		WorldDraw_t g_origWorldDraw{ nullptr };

		void Hook_WorldDraw(bool a_a, bool a_b)
		{
			crossing::ReapplyRoll("the main world draw");
			crossing::RollCheck("start of the main world draw");
			if (g_shFence.load()) {
				view::GpuFence("before the ENGINE main world draw 107139");
			}
			g_origWorldDraw(a_a, a_b);
		}

		using FrameFn_t = std::uint64_t (*)(std::uint64_t, std::uint64_t, std::uint64_t, std::uint64_t);
		using RenderWorld_t = void (*)(bool);
		RenderWorld_t g_origRenderWorld{ nullptr };

		void Hook_RenderWorld(bool a_flag)
		{
			crossing::ReapplyRoll("Main::RenderWorld");
			crossing::RollCheck("start of Main::RenderWorld");
			g_origRenderWorld(a_flag);
			diag::SampleParity();
			crossing::RollCheck("end of Main::RenderWorld");
			crossing::ReapplyRoll("the overlay");  // the engine rebuilds the camera inside Main::RenderWorld; the overlay and everything after must see the rolled one
			diag::SampleMv();
			view::ReportActual();
			overlay::Draw();
			diag::SamplePost();
			view::PostLogBegin();
		}
		FrameFn_t                g_orig{ nullptr };
		std::atomic<std::uint64_t> g_count{ 0 };

		// ---- G236: hang watchdog (view watchdog=<seconds>, 0 = off, default off) ----
		// Armed only by test scripts: when the frame counter has not moved for that long, the stacks of every thread are captured WITHOUT logging while they are suspended (RtlVirtualUnwind, guarded by SEH),
		// written to the log, and the process terminates itself, so a frozen game never has to be shut off by hand and the log says where it hangs.
		std::atomic<double>    g_wdSeconds{ 0.0 };
		std::atomic<DWORD>     g_mainTid{ 0 };
		std::atomic<bool>      g_wdStarted{ false };
		struct WdFrame
		{
			std::uint64_t rip;
		};
		struct WdThread
		{
			DWORD   tid{ 0 };
			WdFrame fr[24];
			int     n{ 0 };
		};
		// no C++ objects with destructors in here (SEH): fills a thread's frames from its suspended context
		int WdWalk(CONTEXT* a_ctx, WdFrame* a_out, int a_max)
		{
			int n = 0;
			__try {
				while (n < a_max && a_ctx->Rip) {
					a_out[n++].rip = a_ctx->Rip;
					DWORD64 base = 0;
					auto*   fe = RtlLookupFunctionEntry(a_ctx->Rip, &base, nullptr);
					if (!fe) {
						a_ctx->Rip = *reinterpret_cast<DWORD64*>(a_ctx->Rsp);
						a_ctx->Rsp += 8;
					} else {
						PVOID   hd = nullptr;
						DWORD64 ef = 0;
						RtlVirtualUnwind(UNW_FLAG_NHANDLER, base, a_ctx->Rip, fe, a_ctx, &hd, &ef, nullptr);
					}
				}
			} __except (EXCEPTION_EXECUTE_HANDLER) {
			}
			return n;
		}
		void WdCapture(DWORD a_tid, WdThread& a_t)
		{
			a_t.tid = a_tid;
			a_t.n = 0;
			HANDLE h = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_QUERY_INFORMATION, FALSE, a_tid);
			if (!h) {
				return;
			}
			if (SuspendThread(h) != static_cast<DWORD>(-1)) {
				CONTEXT ctx{};
				ctx.ContextFlags = CONTEXT_FULL;
				if (GetThreadContext(h, &ctx)) {
					a_t.n = WdWalk(&ctx, a_t.fr, 24);
				}
				ResumeThread(h);
			}
			CloseHandle(h);
		}
		std::string WdWhere(std::uint64_t a_rip)
		{
			HMODULE hm = nullptr;
			if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCSTR>(a_rip), &hm) && hm) {
				char path[MAX_PATH]{};
				GetModuleFileNameA(hm, path, MAX_PATH);
				const char* name = std::strrchr(path, '\\');
				return std::format("{}+{:#x}", name ? name + 1 : path, a_rip - reinterpret_cast<std::uint64_t>(hm));
			}
			return std::format("{:#x}", a_rip);
		}
		void WdFire(double a_seconds)
		{
			std::vector<WdThread> ts;
			const DWORD           me = GetCurrentThreadId();
			HANDLE                snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
			if (snap != INVALID_HANDLE_VALUE) {
				THREADENTRY32 te{};
				te.dwSize = sizeof(te);
				const DWORD pid = GetCurrentProcessId();
				if (Thread32First(snap, &te)) {
					do {
						if (te.th32OwnerProcessID == pid && te.th32ThreadID != me) {
							ts.emplace_back();
							WdCapture(te.th32ThreadID, ts.back());
						}
					} while (Thread32Next(snap, &te));
				}
				CloseHandle(snap);
			}
			logger::error("watchdog: the frame counter has not moved for {:.1f} s: HANG. Main (frame function) thread id {}; {} threads captured.", a_seconds, g_mainTid.load(), ts.size());
			for (const auto& t : ts) {
				if (t.n <= 0) {
					continue;
				}
				std::string line;
				for (int i = 0; i < t.n; ++i) {
					line += (i ? " <- " : "") + WdWhere(t.fr[i].rip);
				}
				logger::error("watchdog: thread {}{}: {}", t.tid, t.tid == g_mainTid.load() ? " (MAIN)" : "", line);
			}
			logger::error("watchdog: terminating the game process now");
			spdlog::default_logger()->flush();
			Sleep(300);
			TerminateProcess(GetCurrentProcess(), 0xDEAD0001);
		}
		void WdLoop()
		{
			std::uint64_t last = 0;
			auto          since = std::chrono::steady_clock::now();
			for (;;) {
				Sleep(200);
				const double lim = g_wdSeconds.load();
				const auto   now = std::chrono::steady_clock::now();
				const auto   cnt = g_count.load();
				if (lim <= 0.0 || cnt != last) {
					last = cnt;
					since = now;
					continue;
				}
				const double dt = std::chrono::duration<double>(now - since).count();
				if (dt > lim) {
					WdFire(dt);
					return;
				}
			}
		}

		std::mutex                  g_pfMutex;
		std::function<bool(double)> g_perFrame;
		std::chrono::steady_clock::time_point g_lastFrame{};

		std::uint64_t Hook_Frame(std::uint64_t a1, std::uint64_t a2, std::uint64_t a3, std::uint64_t a4)
		{
			g_count.fetch_add(1, std::memory_order_relaxed);
			g_mainTid = GetCurrentThreadId();
			if (g_shLeft.load() > 0) {
				logger::info("shtrace: ===== frame {} start (frames left {})", g_count.load(), g_shLeft.load());
			}
			hole::Tick();
			{
				const auto now = std::chrono::steady_clock::now();
				const double dt = g_lastFrame.time_since_epoch().count() ? std::chrono::duration<double>(now - g_lastFrame).count() : 0.0;
				g_lastFrame = now;
				crossing::Tick(dt);
				crossing::ApplyRoll(dt);
				spells::Tick();
				travel::Tick(dt);
				input::Tick();
				killcam::Tick();
				std::function<bool(double)> job;
				{
					std::lock_guard lk(g_pfMutex);
					job = g_perFrame;
				}
				if (job && !job(dt)) {
					std::lock_guard lk(g_pfMutex);
					g_perFrame = nullptr;
				}
			}
			world::ProbeFlags("1 frame start (before any write)");
			if (view::P().animFix & 1) {
				world::KeepActorsAnimated(3500.0f);  // before the engine's own update of this frame reads the visibility flags
			}
			if (view::P().when == 0) {
				world::ProbeFlags("2 after the start write");
				view::RenderViews();  // before the engine's own per-frame setup, so nothing of the virtual views leaks into the main view
			} else {
				view::ResetFrame();  // when=3: RenderViews runs later in the frame (after the shadow maps); a frame that never gets there must not show last frame's views
			}
			if (g_shLeft.load() > 0) {
				logger::info("shtrace: frame {} views done, entering the engine's frame function", g_count.load());
			}
			if (g_shFence.load()) {
				view::GpuFence("after all virtual views (RenderViews returned)");
			}
			world::ProbeFlags("3 after the virtual views");
			const auto fr = g_orig(a1, a2, a3, a4);
			world::ProbeFlags("4 after the engine's frame");
			if (view::P().animFix & 2) {
				world::KeepActorsAnimated(3500.0f);  // after the engine's cull of this frame: the next update sees the actors as visible
				world::ProbeFlags("5 after the end write");
			}
			if (g_shLeft.load() > 0) {
				logger::info("shtrace: ===== frame {} end", g_count.load());
				--g_shLeft;
			}
			return fr;
		}
	}

	void SetWatchdog(double a_seconds)
	{
		g_wdSeconds = a_seconds;
		if (a_seconds > 0.0 && !g_wdStarted.exchange(true)) {
			std::thread(WdLoop).detach();
		}
		logger::info("watchdog: {}", a_seconds > 0.0 ? std::format("armed, fires after {:.1f} s without a frame", a_seconds) : std::string("off"));
	}

	void ShadowFence(bool a_on)
	{
		ShadowTrace(0);  // installs the hooks (silent while the trace counter is 0)
		g_shFence = a_on;
	}

	void ShadowTrace(int a_frames)
	{
		if (!g_shInstalled) {
			g_shInstalled = true;
			REL::Relocation<std::uintptr_t*> vt{ RE::VTABLE_BSShadowDirectionalLight[0] };
			std::uintptr_t* tab = vt.get();
			DWORD old = 0;
			VirtualProtect(tab, 0x20 * sizeof(void*), PAGE_READWRITE, &old);
			g_shOrig[0x9] = tab[0x9];
			g_shOrig[0xA] = tab[0xA];
			g_shOrig[0xC] = tab[0xC];
			g_shOrig[0x10] = tab[0x10];
			tab[0x9] = reinterpret_cast<std::uintptr_t>(&ShWrap<0x9>);
			tab[0xA] = reinterpret_cast<std::uintptr_t>(&ShWrap<0xA>);
			tab[0xC] = reinterpret_cast<std::uintptr_t>(&ShWrap<0xC>);
			tab[0x10] = reinterpret_cast<std::uintptr_t>(&ShWrap<0x10>);
			VirtualProtect(tab, 0x20 * sizeof(void*), old, &old);
			const bool ok = hooks::Install(kShadowStage, reinterpret_cast<void*>(&Hook_ShadowStage), reinterpret_cast<void**>(&g_origShadowStage));
			logger::info("shtrace: sun shadow light vtable {:p} patched (slots 9, 0xA, 0xC, 0x10), shadow stage hook {}", static_cast<void*>(tab), ok);
		}
		g_shLeft = a_frames;
	}

	bool Install()

	{
		bool ok = hooks::Install(kFrameFn, reinterpret_cast<void*>(&Hook_Frame), reinterpret_cast<void**>(&g_orig));
		ok &= hooks::Install(kRenderWorld, reinterpret_cast<void*>(&Hook_RenderWorld), reinterpret_cast<void**>(&g_origRenderWorld));
		ok &= hooks::Install(kWorldDraw, reinterpret_cast<void*>(&Hook_WorldDraw), reinterpret_cast<void**>(&g_origWorldDraw));
		ok &= hooks::Install(kShadowMaps, reinterpret_cast<void*>(&Hook_ShadowMaps), reinterpret_cast<void**>(&g_origShadowMaps));
		return ok;
	}

	void SetPerFrame(std::function<bool(double)> a_job)
	{
		std::lock_guard lk(g_pfMutex);
		g_perFrame = std::move(a_job);
	}

	std::uint64_t Count()
	{
		return g_count.load(std::memory_order_relaxed);
	}
}
