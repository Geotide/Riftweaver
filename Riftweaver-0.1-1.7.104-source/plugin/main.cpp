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

#ifndef RW_VERSION_STRING
#	define RW_VERSION_STRING "dev"
#endif

namespace
{
	void InitLog()
	{
		auto path = logger::log_directory();
		if (!path) {
			return;
		}
		*path /= "Riftweaver.log"sv;
		auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path->string(), true);
		auto log = std::make_shared<spdlog::logger>("riftweaver", std::move(sink));
		log->set_level(spdlog::level::info);
		log->flush_on(spdlog::level::info);
		spdlog::set_default_logger(std::move(log));
		spdlog::set_pattern("[%H:%M:%S.%e] [%l] %v");
	}

	void OnMessage(SKSE::MessagingInterface::Message* a_msg)
	{
		if (a_msg->type == SKSE::MessagingInterface::kPreLoadGame || a_msg->type == SKSE::MessagingInterface::kNewGame) {
			portals::Clear();  // portals are not saved: they are cleared whenever a save is loaded
			if (a_msg->type == SKSE::MessagingInterface::kNewGame) {
				spells::OnLoad(true);  // the gift: the two portal spells
			}
			return;
		}
		if (a_msg->type == SKSE::MessagingInterface::kPostLoadGame) {
			spells::OnLoad(false);  // the gift (once per save: only when missing) and the retired gun's removal
			return;
		}
		if (a_msg->type != SKSE::MessagingInterface::kDataLoaded) {
			return;
		}
		logger::info("kDataLoaded");
		const bool hooksOk = hooks::Init();
		const bool frameOk = frame::Install();
#if !defined(RW_RELEASE) || defined(RW_TEST_HOOKS)
		const bool shotOk = shot::Init();  // Present hook: test screenshots, recorder, label
#else
		const bool shotOk = false;
#endif
		const bool viewOk = view::Install();
		const bool holeOk = hole::Install();
		const bool crossOk = crossing::Install();
		travel::SetPassage(true);  // S8: gradual passage for everything that can touch a portal (props, actors, ragdolls, projectiles, streams, runes, summons) is on by default; `passage off` in the test harness turns it off
		crashlog::Install();
		const bool spellsOk = spells::Install();
#if !defined(RW_RELEASE) || defined(RW_TEST_HOOKS)
		if (auto dir = logger::log_directory(); dir && std::filesystem::exists(*dir / "Riftweaver_mute.flag")) {
			harness::Mute(true);  // a silent test run (scripts/run_test.ps1 -Mute): muted before the menu music starts
			std::error_code ec;
			std::filesystem::remove(*dir / "Riftweaver_mute.flag", ec);  // one launch only: a cancelled run must not leave his own next start silent
		}
#endif
		logger::info("init: hooks ok={} frame hook ok={} present hook ok={} view hook ok={} hole hook ok={} camera hook ok={} spell events ok={}", hooksOk, frameOk, shotOk, viewOk, holeOk, crossOk, spellsOk);
#if !defined(RW_RELEASE) || defined(RW_TEST_HOOKS)
		// The game pauses when it loses focus; tests run unattended, so keep it running (runtime only, no ini change).
		if (auto* st = RE::GetINISetting("bAlwaysActive:General")) {
			logger::info("bAlwaysActive was {}; setting it to true at runtime", st->GetBool());
			st->data.b = true;
		} else {
			logger::warn("INI setting bAlwaysActive:General not found; the game will pause when it loses focus");
		}
		harness::Start();
#else
		spdlog::default_logger()->set_level(spdlog::level::warn);  // the release logs its start-up, then only warnings and errors
#endif
	}
}

SKSEPluginLoad(const SKSE::LoadInterface* a_skse)
{
	InitLog();
	SKSE::Init(a_skse);
	const auto ver = a_skse->RuntimeVersion();
	logger::info("Riftweaver {} loaded", RW_VERSION_STRING);
	logger::info("runtime version: {}.{}.{}.{}", ver.major(), ver.minor(), ver.patch(), ver.build());
#ifdef RW_RELEASE
	// the see-through view reads a few engine addresses and structure layouts directly (shader vtables, renderer fields): they are only known for this runtime
	if (ver != REL::Version(1, 7, 104, 0)) {
		logger::critical("Riftweaver {} supports Skyrim Special Edition 1.7.104 only (this is {}.{}.{}.{}): not loaded", RW_VERSION_STRING, ver.major(), ver.minor(), ver.patch(), ver.build());
		return false;
	}
#endif
	logger::info("module base {:#x}", REL::Module::get().base());
	view::InstallEarlyDeviceHook();
	SKSE::GetMessagingInterface()->RegisterListener(OnMessage);
	return true;
}
