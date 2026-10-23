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

// The two portal spells (Riftweaver.esp; scripts/build_esp.py): "Rift: Tidal Weave" places the TEAL portal (index 0, the old blue), "Rift: Umbral Weave" the VIOLET one (index 1, the old orange).
// Both are fire-and-forget spells with a do-nothing "Script" effect (delivery self); everything they do is done here. A cast is recognised by three independent triggers, whichever fires first
// places the portal (a short debounce per colour drops the others of the same cast):
//   1. TESSpellCastEvent for the player and one of our spells,
//   2. TESMagicEffectApplyEvent: one of our effects applied to the player,
//   3. a per-frame poll (Tick): the player's magicka dropped by about the spell's cost while the spell was being readied or charged in a hand.
// The portal is placed by portals::PlaceFromCamera: exactly the old gun's ray, placement rules and refusals.
// Also here: the gift (both spells are added when a save loads or a new game starts, Riftweaver.ini GiveSpells=1), the removal of the retired gun stub (RemoveGun=1) and the hand state
// that the particle effects use for the wisp at the casting hand.
namespace spells
{
	namespace
	{
		constexpr const char*   kPlugin = "Riftweaver.esp";
		constexpr std::uint32_t kSpellLocal[2] = { 0x805, 0x804 };   // [colour]: teal = Tidal Weave, violet = Umbral Weave
		constexpr std::uint32_t kEffectLocal[2] = { 0x803, 0x802 };
		constexpr std::uint32_t kStubLocal = 0x800;                  // the retired gun's record (stub: no model)
		const char* const       kColourName[2] = { "teal", "violet" };
		const char* const       kSpellName[2] = { "Rift: Tidal Weave", "Rift: Umbral Weave" };

		struct Settings
		{
			bool giveSpells{ true };
			bool removeGun{ true };
			bool particles{ true };
		};
		Settings           g_set;
		bool               g_settingsRead{ false };
		std::filesystem::path g_iniPath;
		std::filesystem::file_time_type g_iniTime{};
		std::mutex         g_styleMutex;
		Style              g_style;
		RE::SpellItem*     g_spell[2]{ nullptr, nullptr };
		RE::EffectSetting* g_effect[2]{ nullptr, nullptr };
		RE::TESObjectWEAP* g_stub{ nullptr };
		bool               g_resolved{ false };
		bool               g_warned{ false };

		std::mutex g_mutex;           // guards g_lastPlaced, g_hand
		double     g_lastPlaced[2]{ -100.0, -100.0 };
		HandFx     g_hand[2];
		std::atomic<int> g_placed[2]{ 0, 0 };
		std::atomic<int> g_viaCount[3]{ 0, 0, 0 };
		std::atomic<int> g_dups{ 0 };

		double Now()
		{
			return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
		}

		float IniFloat(const wchar_t* a_key, float a_def, const wchar_t* a_path)
		{
			wchar_t buf[64]{};
			GetPrivateProfileStringW(L"General", a_key, L"", buf, 64, a_path);
			if (!buf[0]) {
				return a_def;
			}
			try {
				return std::stof(std::wstring(buf));
			} catch (...) {
				return a_def;
			}
		}

		void ReadIni(bool a_first)
		{
			const auto path = g_iniPath.wstring();
			Settings   s;
			Style      st;
			s.giveSpells = GetPrivateProfileIntW(L"General", L"GiveSpells", 1, path.c_str()) != 0;
#ifdef RW_RELEASE
			s.removeGun = false;  // the release has no retired-gun record
#else
			s.removeGun = GetPrivateProfileIntW(L"General", L"RemoveGun", 1, path.c_str()) != 0;
#endif
			s.particles = GetPrivateProfileIntW(L"General", L"Particles", 1, path.c_str()) != 0;
			st.particles = s.particles;
			st.density = std::clamp(IniFloat(L"ParticleDensity", 1.0f, path.c_str()), 0.0f, 4.0f);
			st.alpha = std::clamp(IniFloat(L"ParticleAlpha", 1.0f, path.c_str()), 0.0f, 3.0f);
			st.halo = std::clamp(IniFloat(L"Halo", 0.7f, path.c_str()), 0.0f, 2.0f);
			st.rim = std::clamp(IniFloat(L"Rim", 1.0f, path.c_str()), 0.0f, 2.0f);
			st.rimFlow = std::clamp(IniFloat(L"RimFlow", 1.0f, path.c_str()), 0.0f, 2.0f);
			st.rimGlints = std::clamp(IniFloat(L"RimGlints", 1.0f, path.c_str()), 0.0f, 2.0f);
			st.ripple = std::clamp(IniFloat(L"Ripple", 1.0f, path.c_str()), 0.0f, 2.0f);
			st.sheen = std::clamp(IniFloat(L"Sheen", 1.0f, path.c_str()), 0.0f, 2.0f);
			st.mistSwirl = std::clamp(IniFloat(L"MistSwirl", 1.0f, path.c_str()), 0.0f, 2.0f);
			st.embers = std::clamp(IniFloat(L"Embers", 1.0f, path.c_str()), 0.0f, 3.0f);
			st.light = GetPrivateProfileIntW(L"General", L"Light", 1, path.c_str()) != 0;
			st.lightRadius = std::clamp(IniFloat(L"LightRadius", 250.0f, path.c_str()), 50.0f, 1000.0f);
			st.lightIntensity = std::clamp(IniFloat(L"LightIntensity", 1.0f, path.c_str()), 0.0f, 4.0f);
			st.openAnim = std::clamp(IniFloat(L"OpenAnim", 1.0f, path.c_str()), 0.0f, 2.0f);
			st.closeAnim = std::clamp(IniFloat(L"CloseAnim", 1.0f, path.c_str()), 0.0f, 2.0f);
			st.handRunes = std::clamp(IniFloat(L"HandRunes", 0.0f, path.c_str()), 0.0f, 2.0f);  // the plugin-drawn ring and glow: off by default since the spells' Casting Art carries them (it drifted while moving)
			st.releaseFlash = std::clamp(IniFloat(L"ReleaseFlash", 1.0f, path.c_str()), 0.0f, 2.0f);
			st.walkRipple = std::clamp(IniFloat(L"WalkRipple", 1.0f, path.c_str()), 0.0f, 2.0f);
			st.exitPuff = std::clamp(IniFloat(L"ExitPuff", 1.0f, path.c_str()), 0.0f, 2.0f);
			st.sounds = std::clamp(IniFloat(L"Sounds", 1.0f, path.c_str()), 0.0f, 2.0f);
			st.humVolume = std::clamp(IniFloat(L"HumVolume", 1.0f, path.c_str()), 0.0f, 2.0f);
			travel::SetTrapdoor(GetPrivateProfileIntW(L"General", L"Trapdoor", 1, path.c_str()) != 0);
			killcam::SetEnabled(GetPrivateProfileIntW(L"General", L"Killcam", 1, path.c_str()) != 0);
			fx::SetHandFxLog(GetPrivateProfileIntW(L"General", L"HandFxLog", 0, path.c_str()) != 0);
			fx::SetHandWisps(GetPrivateProfileIntW(L"General", L"HandWisps", 0, path.c_str()) != 0);
			{
				std::lock_guard lk(g_styleMutex);
				g_set = s;
				g_style = st;
			}
			logger::info("spells: {} {}: GiveSpells={} Particles={} ParticleDensity={:.2f} ParticleAlpha={:.2f} Halo={:.2f} Rim={:.2f}", a_first ? "read" : "re-read", g_iniPath.filename().string(), s.giveSpells, s.particles, st.density, st.alpha, st.halo, st.rim);
		}

		void LoadSettings()
		{
			if (g_settingsRead) {
				return;
			}
			g_settingsRead = true;
			HMODULE self = nullptr;
			if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCWSTR>(&LoadSettings), &self)) {
				logger::warn("spells: cannot find the plugin's own path; default settings");
				return;
			}
			wchar_t buf[MAX_PATH]{};
			const DWORD n = GetModuleFileNameW(self, buf, MAX_PATH);
			g_iniPath = std::filesystem::path(std::wstring(buf, n));
			g_iniPath.replace_extension(L".ini");
			std::error_code ec;
			if (!std::filesystem::exists(g_iniPath, ec)) {
				logger::info("spells: no {} next to the plugin: default settings (GiveSpells=1 Particles=1 Halo=0.7 Rim=1)", g_iniPath.filename().string());
				return;
			}
			g_iniTime = std::filesystem::last_write_time(g_iniPath, ec);
			ReadIni(true);
		}

		// the ini is watched while the game runs: edit and save it and the look changes within half a second
		void PollIni()
		{
			static double next = 0.0;
			const double  now = Now();
			if (now < next || g_iniPath.empty()) {
				return;
			}
			next = now + 0.5;
			std::error_code ec;
			const auto      t = std::filesystem::last_write_time(g_iniPath, ec);
			if (!ec && t != g_iniTime) {
				g_iniTime = t;
				ReadIni(false);
			}
		}

		void Resolve()
		{
			if (g_resolved) {
				return;
			}
			auto* dh = RE::TESDataHandler::GetSingleton();
			if (!dh) {
				return;
			}
			bool all = true;
			for (int c = 0; c < 2; ++c) {
				if (!g_spell[c]) {
					g_spell[c] = dh->LookupForm<RE::SpellItem>(kSpellLocal[c], kPlugin);
				}
				if (!g_effect[c]) {
					g_effect[c] = dh->LookupForm<RE::EffectSetting>(kEffectLocal[c], kPlugin);
				}
				all = all && g_spell[c] && g_effect[c];
			}
#ifndef RW_RELEASE
			if (!g_stub) {
				g_stub = dh->LookupForm<RE::TESObjectWEAP>(kStubLocal, kPlugin);
			}
#endif
			g_resolved = all;
			if (!all && !g_warned) {
				g_warned = true;
				logger::warn("spells: {} is not loaded or lacks the spell records (teal {:p}, violet {:p}); the portal spells are unavailable", kPlugin, static_cast<void*>(g_spell[0]), static_cast<void*>(g_spell[1]));
			}
			if (all) {
				logger::info("spells: resolved {} ({:#x}) and {} ({:#x})", kSpellName[0], g_spell[0]->GetFormID(), kSpellName[1], g_spell[1]->GetFormID());
			}
		}

		// Runs on the game's main thread (events are queued there): one cast places one portal.
		void Trigger(int a_colour, int a_via, const char* a_what)
		{
			if (a_colour < 0 || a_colour > 1) {
				return;
			}
			const double now = Now();
			{
				std::lock_guard lk(g_mutex);
				if (now - g_lastPlaced[a_colour] < 0.7) {
					++g_dups;
					logger::info("spells: {} cast seen again by '{}' ({:.2f} s after the first): ignored", kColourName[a_colour], a_what, now - g_lastPlaced[a_colour]);
					return;
				}
				g_lastPlaced[a_colour] = now;
			}
			++g_viaCount[a_via];
			std::string why;
			const bool  ok = portals::PlaceFromCamera(a_colour, why);
			if (ok) {
				++g_placed[a_colour];
			}
			logger::info("spells: {} cast detected by '{}': {}{}", kColourName[a_colour], a_what, ok ? "portal placed" : "refused: ", ok ? "" : why);
			if (!ok) {
				// feedback on screen (2026-10-21: casts that open nothing looked like a bug)
				const std::string msg = why.rfind("the ray hit nothing", 0) == 0 ? std::string{ "The rift finds nothing to hold on to there." }
				                      : why.rfind("an actor is in the way", 0) == 0 ? std::string{ "The rift cannot open: someone is in the way." }
				                      : why.rfind("surface not allowed", 0) == 0 ? std::string{ "The rift cannot hold on that surface." }
				                      : why.rfind("overlaps the other portal", 0) == 0 ? std::string{ "Too close to the other rift." }
				                                                                     : std::string{ "The rift cannot open there." };
				RE::SendHUDMessage::ShowHUDMessage(msg.c_str(), nullptr, true);
			}
		}

		void QueueTrigger(int a_colour, int a_via, const char* a_what)
		{
			// the events may arrive on another thread than the one that may raycast and edit portal state: hand the placement to the main thread
			if (auto* ti = SKSE::GetTaskInterface()) {
				ti->AddTask([a_colour, a_via, a_what]() { Trigger(a_colour, a_via, a_what); });
			} else {
				Trigger(a_colour, a_via, a_what);
			}
		}

		class CastSink : public RE::BSTEventSink<RE::TESSpellCastEvent>
		{
		public:
			RE::BSEventNotifyControl ProcessEvent(const RE::TESSpellCastEvent* a_event, RE::BSTEventSource<RE::TESSpellCastEvent>*) override
			{
				if (a_event && a_event->object && a_event->object.get() == RE::PlayerCharacter::GetSingleton()) {
					const int c = ColourOfSpell(a_event->spell);
					if (c >= 0) {
						QueueTrigger(c, 0, "spell cast event");
					}
				}
				return RE::BSEventNotifyControl::kContinue;
			}
		};

		class ApplySink : public RE::BSTEventSink<RE::TESMagicEffectApplyEvent>
		{
		public:
			RE::BSEventNotifyControl ProcessEvent(const RE::TESMagicEffectApplyEvent* a_event, RE::BSTEventSource<RE::TESMagicEffectApplyEvent>*) override
			{
				if (a_event && a_event->caster && a_event->caster.get() == RE::PlayerCharacter::GetSingleton()) {
					const int c = ColourOfSpell(a_event->magicEffect);
					if (c >= 0) {
						QueueTrigger(c, 1, "effect apply event");
					}
				}
				return RE::BSEventNotifyControl::kContinue;
			}
		};

		CastSink  g_castSink;
		ApplySink g_applySink;

		// the magic node of a hand: the first-person skeleton's while the camera is in first person, the third-person one otherwise
		RE::NiAVObject* HandNode(RE::PlayerCharacter* a_p, int a_hand, RE::ActorMagicCaster* a_caster)
		{
			static const char* const kMagic[2] = { "NPC L MagicNode [LMag]", "NPC R MagicNode [RMag]" };
			static const char* const kHand[2] = { "NPC L Hand [LHnd]", "NPC R Hand [RHnd]" };
			auto*                    cam = RE::PlayerCamera::GetSingleton();
			RE::NiAVObject*          root = a_p->Get3D(cam && cam->IsInFirstPerson());
			if (root) {
				if (auto* n = root->GetObjectByName(kMagic[a_hand])) {
					return n;
				}
				if (auto* n = root->GetObjectByName(kHand[a_hand])) {
					return n;
				}
			}
			return a_caster ? static_cast<RE::NiAVObject*>(a_caster->magicNode) : nullptr;
		}

		// The casting-hand effect is each spell's Casting Art (meshes/Riftweaver/HandWisp{Teal,Violet}.nif): the engine attaches it to the hand's magic node and shows it whenever
		// the spell is readied. (2026-10-22): only while casting. While a hand holds our spell in a casting state the effect's billboard grows in over 0.15 s, otherwise
		// it shrinks out and is hidden; the ring and the swirl turn slowly (their nodes' local rotation about the billboard's axis).
		void UpdateHandArt(RE::PlayerCharacter* a_p, const HandFx a_hand[2], double a_now)
		{
			// 0.1 test (18:10Z): the effect still showed outside a cast and did not turn, so the lookup by the art's root name failed in his game; every node is looked up by
			// its own (per-colour) name from the skeleton's root, and what is found or not is logged (warning level, a few lines)
			static const char* const kBillboard[2] = { "RiftBillboardTeal", "RiftBillboardViolet" };
			static const char* const kRing[2] = { "RiftRingSpinTeal", "RiftRingSpinViolet" };
			static const char* const kSwirl[2] = { "RiftSwirlSpinTeal", "RiftSwirlSpinViolet" };
			static const char* const kOrbit[2] = { "RiftOrbitTeal", "RiftOrbitViolet" };
			static float  s_level[2]{ 0.0f, 0.0f };
			static double s_last{ 0.0 };
			static int    s_found[2][2]{};   // [colour][first person]: logged once found
			static int    s_missLogged{ 0 };
			const float   dt = s_last > 0.0 ? static_cast<float>(std::clamp(a_now - s_last, 0.0, 0.1)) : 0.0f;
			s_last = a_now;
			for (int c = 0; c < 2; ++c) {
				const bool want = (a_hand[0].active && a_hand[0].colour == c) || (a_hand[1].active && a_hand[1].colour == c);
				s_level[c] = std::clamp(s_level[c] + (want ? dt : -dt) / 0.15f, 0.0f, 1.0f);
				const float ring = static_cast<float>(std::fmod(a_now * 0.9, 6.283185307));
				const float swirl = static_cast<float>(std::fmod(-a_now * 3.0, 6.283185307));
				const float orbit = static_cast<float>(std::fmod(a_now * 4.0, 6.283185307));
				bool        any = false;
				for (bool first : { true, false }) {
					auto* root3d = a_p->Get3D(first);
					auto* bb = root3d ? root3d->GetObjectByName(kBillboard[c]) : nullptr;
					if (!bb) {
						continue;
					}
					any = true;
					if (!s_found[c][first]) {
						s_found[c][first] = 1;
						logger::warn("handart: '{}' found in the {} skeleton (parent '{}')", kBillboard[c], first ? "first-person" : "third-person", bb->parent && bb->parent->name.c_str() ? bb->parent->name.c_str() : "");
					}
					const float k = s_level[c];
					bb->local.scale = 0.25f + 0.75f * k * k * (3.0f - 2.0f * k);
					if (k <= 0.001f) {
						bb->GetFlags().set(RE::NiAVObject::Flag::kHidden);
					} else {
						bb->GetFlags().reset(RE::NiAVObject::Flag::kHidden);
					}
					if (auto* n = root3d->GetObjectByName(kRing[c])) n->local.rotate.SetEulerAnglesXYZ(0.0f, 0.0f, ring);
					if (auto* n = root3d->GetObjectByName(kSwirl[c])) n->local.rotate.SetEulerAnglesXYZ(0.0f, 0.0f, swirl);
					if (auto* n = root3d->GetObjectByName(kOrbit[c])) n->local.rotate.SetEulerAnglesXYZ(0.0f, 0.0f, orbit);
				}
				if (want && !any && s_missLogged < 4) {
					++s_missLogged;
					auto* pcam = RE::PlayerCamera::GetSingleton();
					logger::warn("handart: casting {} but '{}' is not in the player's skeletons (first person {})", c == 0 ? "teal" : "violet", kBillboard[c], pcam && pcam->IsInFirstPerson());
				}
			}
		}

		// how many of the retired gun the player carries
#ifndef RW_RELEASE
		int CountRetiredGun(RE::PlayerCharacter* a_p)
		{
			const auto counts = a_p->GetInventoryCounts([&](RE::TESBoundObject& a_o) { return &a_o == g_stub; });
			const auto it = counts.find(g_stub);
			return it == counts.end() ? 0 : it->second;
		}

		void RemoveRetiredGunImpl(RE::PlayerCharacter* a_p, int a_count)
		{
			if (auto* em = RE::ActorEquipManager::GetSingleton()) {
				em->UnequipObject(a_p, g_stub, nullptr, 1, nullptr, false, true, false, true);
			}
			a_p->RemoveItem(g_stub, a_count, RE::ITEM_REMOVE_REASON::kRemove, nullptr, nullptr);
		}

		bool g_removeFailed{ false };

		void LogRemoveFailure()
		{
			logger::error("spells: an exception was raised while removing the retired portal gun; it stays in the inventory (set RemoveGun=0 in Riftweaver.ini to stop trying)");
		}

		// SEH wrapper: this function has no objects with destructors
		bool RemoveRetiredGunGuarded(RE::PlayerCharacter* a_p, int a_count)
		{
			__try {
				RemoveRetiredGunImpl(a_p, a_count);
				return true;
			} __except (EXCEPTION_EXECUTE_HANDLER) {
				g_removeFailed = true;
				LogRemoveFailure();
				return false;
			}
		}

		void RemoveRetiredGun(RE::PlayerCharacter* a_p)
		{
			if (!g_set.removeGun || !g_stub || !a_p || g_removeFailed) {
				return;
			}
			const int n = CountRetiredGun(a_p);
			if (n <= 0) {
				return;
			}
			if (RemoveRetiredGunGuarded(a_p, n)) {
				logger::info("spells: removed the retired portal gun ({}) from the player's inventory", n);
			}
		}

#endif

		// the gift (and in test builds the retired gun's removal) runs a little after a load, from Tick, when the player's 3D is up: not inside the load message
		bool   g_pending{ false };
		double g_pendingAt{ 0.0 };
		bool   g_pendingNew{ false };
	}

	int ColourOfSpell(std::uint32_t a_formId)
	{
		for (int c = 0; c < 2; ++c) {
			if ((g_spell[c] && g_spell[c]->GetFormID() == a_formId) || (g_effect[c] && g_effect[c]->GetFormID() == a_formId)) {
				return c;
			}
		}
		return -1;
	}

	bool Install()
	{
		LoadSettings();
		Resolve();
		auto* holder = RE::ScriptEventSourceHolder::GetSingleton();
		if (!holder) {
			logger::error("spells: no script event source holder; only the per-frame poll can detect a cast");
			return false;
		}
		holder->AddEventSink<RE::TESSpellCastEvent>(&g_castSink);
		holder->AddEventSink<RE::TESMagicEffectApplyEvent>(&g_applySink);
		logger::info("spells: spell-cast and effect-apply event sinks installed; per-frame poll armed");
		return true;
	}

	bool GiveAll()
	{
		Resolve();
		auto* p = RE::PlayerCharacter::GetSingleton();
		if (!p || !g_resolved) {
			return false;
		}
		for (int c : { 1, 0 }) {  // Umbral Weave first, then Tidal Weave
			if (!p->HasSpell(g_spell[c])) {
				p->AddSpell(g_spell[c]);
				logger::info("spells: added {} to the player", kSpellName[c]);
				// the game's own notification, one line per spell actually added (2026-10-22, 1.0.1); nothing is shown when the player already has both
				const std::string msg = std::format("You have learned: {}", kSpellName[c]);
				RE::SendHUDMessage::ShowHUDMessage(msg.c_str(), nullptr, false);
			}
		}
		return true;
	}

	void RemoveAllForTest()
	{
		Resolve();
		auto* p = RE::PlayerCharacter::GetSingleton();
		if (!p || !g_resolved) {
			return;
		}
		for (int c = 0; c < 2; ++c) {
			p->RemoveSpell(g_spell[c]);
		}
		logger::info("spells: test: both spells removed from the player (teal {} violet {})", p->HasSpell(g_spell[0]), p->HasSpell(g_spell[1]));
	}

	void OnLoad(bool a_newGame)
	{
		LoadSettings();
		Resolve();
		g_pending = true;
		g_pendingNew = a_newGame;
		g_pendingAt = Now() + 2.0;
		logger::info("spells: {} - the gift runs in about 2 s (GiveSpells={})", a_newGame ? "new game" : "save loaded", g_set.giveSpells);
	}

	bool Equip(int a_colour, bool a_rightHand)
	{
		Resolve();
		auto* p = RE::PlayerCharacter::GetSingleton();
		auto* em = RE::ActorEquipManager::GetSingleton();
		if (!p || !em || !g_resolved || a_colour < 0 || a_colour > 1) {
			return false;
		}
		GiveAll();
		auto* slot = RE::TESForm::LookupByID<RE::BGSEquipSlot>(a_rightHand ? 0x00013F42u : 0x00013F43u);
		em->EquipSpell(p, g_spell[a_colour], slot);
		if (auto* st = p->AsActorState(); st && !st->IsWeaponDrawn()) {
			p->DrawWeaponMagicHands(true);
		}
		p->AsActorValueOwner()->RestoreActorValue(RE::ActorValue::kMagicka, 1000.0f);  // test only: so a run can cast as often as it likes
		logger::info("spells: {} equipped in the {} hand", kSpellName[a_colour], a_rightHand ? "right" : "left");
		return true;
	}

	void Tick()
	{
		PollIni();
		if (!g_resolved) {
			Resolve();
			if (!g_resolved && !g_pending) {
				return;
			}
		}
		auto* p = RE::PlayerCharacter::GetSingleton();
		if (!p || !p->Is3DLoaded()) {
			return;
		}
		if (g_pending && Now() >= g_pendingAt) {
			g_pending = false;
#ifndef RW_RELEASE
			RemoveRetiredGun(p);
#endif
			if (g_set.giveSpells && g_resolved) {
				GiveAll();
			}
			logger::info("spells: {} handled; the player has teal {} violet {}", g_pendingNew ? "new game" : "loaded save", g_resolved && p->HasSpell(g_spell[0]), g_resolved && p->HasSpell(g_spell[1]));
		}
		if (!g_resolved) {
			return;
		}
		static float  lastMag = -1.0f;
		static double activeUntil[2]{ 0, 0 };
		static int    lastColour[2]{ -1, -1 };
		static unsigned lastState[2]{ 0, 0 };
		static int    stateLog = 0;
		const double  now = Now();
		const float   mag = p->AsActorValueOwner()->GetActorValue(RE::ActorValue::kMagicka);
		HandFx        hand[2];
		for (int h = 0; h < 2; ++h) {
			auto*      c = p->magicCasters[h];
			auto*      item = c ? c->currentSpell : nullptr;
			const int  col = item ? ColourOfSpell(item->GetFormID()) : -1;
			const unsigned st = c ? static_cast<unsigned>(c->state.get()) : 0u;
			if ((col >= 0 || lastColour[h] >= 0) && st != lastState[h] && stateLog < 60) {
				++stateLog;
				logger::info("spells: {} hand caster state {} -> {} (spell {}), magicka {:.1f}", h == 0 ? "left" : "right", lastState[h], st, col >= 0 ? kSpellName[col] : "none", mag);
			}
			const bool casting = col >= 0 && st >= 4 && st <= 7;  // PreStart, Charging, Casting, (7): the spell is being cast from this hand
			const bool active = col >= 0 && st >= 1 && st <= 7;   // plus the start/charge states (1, 2) and Ready (3): the hand is up with the spell, the wisp shows
			if (casting) {
				activeUntil[h] = now + 0.6;
			}
			// trigger 3: the magicka cost is paid while our spell is (or has just been) charging
			const int trigColour = col >= 0 ? col : lastColour[h];
			if (lastMag >= 0.0f && mag < lastMag - 15.0f && trigColour >= 0 && now < activeUntil[h]) {
				QueueTrigger(trigColour, 2, "per-frame poll (magicka paid)");
				activeUntil[h] = 0.0;
			}
			lastColour[h] = col;
			lastState[h] = st;
			if (active || now < activeUntil[h] - 0.3) {
				hand[h].active = true;
				hand[h].colour = col >= 0 ? col : trigColour;
				if (auto* n = HandNode(p, h, c)) {
					hand[h].pos = n->world.translate;
					// in first person the arms move with the camera: keep the hand relative to the camera read at the same moment, so the drawing (later, with that frame's camera)
					// puts the runes on the hand while the player turns (2026-10-21: the ring jumped away from the hand when moving)
					auto*          pcam = RE::PlayerCamera::GetSingleton();
					world::CamInfo ci;
					hand[h].firstPerson = pcam && pcam->IsInFirstPerson() && world::CameraFull(ci);
					if (hand[h].firstPerson) {
						const RE::NiPoint3 d = hand[h].pos - ci.pos;
						hand[h].camRel = RE::NiPoint3{ d.x * ci.right.x + d.y * ci.right.y + d.z * ci.right.z, d.x * ci.up.x + d.y * ci.up.y + d.z * ci.up.z, d.x * ci.fwd.x + d.y * ci.fwd.y + d.z * ci.fwd.z };
					}
				} else {
					hand[h].active = false;
				}
				hand[h].at = now;
			}
		}
		lastMag = mag;
		UpdateHandArt(p, hand, now);
		std::lock_guard lk(g_mutex);
		g_hand[0] = hand[0];
		g_hand[1] = hand[1];
	}

	bool HandWorldNow(int a_hand, RE::NiPoint3& a_out)
	{
		// read when the moments are drawn (render thread, read only): by then the engine has updated the first-person arms for this frame, so the node and the frame's camera agree
		// (2026-10-21: the ring drifted off the hand while turning; the position taken at the start of the frame belonged to the previous frame's arms)
		auto* p = RE::PlayerCharacter::GetSingleton();
		if (!p || a_hand < 0 || a_hand > 1) {
			return false;
		}
		if (auto* n = HandNode(p, a_hand, p->magicCasters[a_hand])) {
			a_out = n->world.translate;
			return true;
		}
		return false;
	}

	void GetHandFx(HandFx a_out[2])
	{
		std::lock_guard lk(g_mutex);
		a_out[0] = g_hand[0];
		a_out[1] = g_hand[1];
	}

	bool ParticlesEnabled()
	{
		LoadSettings();
		std::lock_guard lk(g_styleMutex);
		return g_style.particles;
	}

	Style GetStyle()
	{
		std::lock_guard lk(g_styleMutex);
		return g_style;
	}

	void SetStyle(const Style& a_style)
	{
		std::lock_guard lk(g_styleMutex);
		g_style = a_style;
		logger::info("spells: style set: particles {} density {:.2f} alpha {:.2f} halo {:.2f} rim {:.2f}", a_style.particles, a_style.density, a_style.alpha, a_style.halo, a_style.rim);
	}

	void LogStatus()
	{
		auto* p = RE::PlayerCharacter::GetSingleton();
		logger::info("spells: resolved {} teal {:p} violet {:p}; player has teal {} violet {}; placed teal {} violet {}; triggers: cast event {} effect event {} poll {}; duplicates dropped {}", g_resolved, static_cast<void*>(g_spell[0]),
			static_cast<void*>(g_spell[1]), g_resolved && p && p->HasSpell(g_spell[0]), g_resolved && p && p->HasSpell(g_spell[1]), g_placed[0].load(), g_placed[1].load(), g_viaCount[0].load(), g_viaCount[1].load(), g_viaCount[2].load(), g_dups.load());
	}
}
