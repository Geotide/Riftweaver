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
#include "Util.h"

// The player's body through a portal (stage S6).
//
// While the player stands inside an entry portal's oval, the half of the body that has passed the plane is hidden by the wall in the real view. What a viewer should see through
// the portal is that half emerging from the exit. The see-through pass (PortalView.cpp, RenderBody) draws exactly that: the player's own, unmoved skeleton, with the real
// camera, into the virtual picture, keeping only what lies behind the entry portal's plane. Carrying the body to the exit instead (moving every node and its skin) does not work:
// the engine caches the skin's bone matrices per frame and builds them relative to the camera of the first pass that draws the body (tried in G39-G41).
// Why the real camera gives the right picture: the virtual camera is the real camera carried through the pair by a rigid map, so the body as seen from the real camera is the
// carried body as seen from the virtual camera, and the exit plane that clips the virtual pass corresponds to the entry plane with the opposite normal.
namespace ghost
{
	namespace
	{
		std::atomic<bool> g_enabled{ true };
		std::atomic<bool> g_front{ false };
		int               g_bodyPasses{ 0 };
		float             g_lastDn{ 0 };

		using RE::NiPoint3;
		float Dot(const NiPoint3& a, const NiPoint3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
	}

	void SetEnabled(bool a_on)
	{
		g_enabled = a_on;
		logger::info("ghost: the player's body in the see-through picture {}", a_on ? "ON" : "off");
	}

	bool Straddling(const portals::Portal& a_entry)
	{
		if (!g_enabled.load() || !a_entry.valid) {
			return false;
		}
		auto* p = RE::PlayerCharacter::GetSingleton();
		auto* pc = RE::PlayerCamera::GetSingleton();
		if (!p || !pc || pc->IsInFirstPerson() || !p->Get3D(false)) {
			return false;
		}
		const RE::NiPoint3 pos = p->GetPosition();
		const NiPoint3     centre{ pos.x, pos.y, pos.z + 64.0f };  // capsule centre of the 128-unit tall player
		const NiPoint3     d = centre - a_entry.c;
		const float        dn = Dot(d, a_entry.n), lx = Dot(d, a_entry.r), ly = Dot(d, a_entry.u);
		g_lastDn = dn;
		constexpr float kRadius = 22.0f;
		if (std::fabs(dn) > kRadius + 10.0f) {
			return false;
		}
		const float ex = (lx / (a_entry.hw + kRadius)), ey = (ly / (a_entry.hh + 64.0f));
		return ex * ex + ey * ey <= 1.0f;
	}

	void SetDebugFront(bool a_on)
	{
		g_front = a_on;
		logger::info("ghost: debug: the body pass keeps the part {} the entry plane", a_on ? "IN FRONT of" : "behind");
	}

	bool DebugFront()
	{
		return g_front.load();
	}

	namespace
	{
		void CollectInner(RE::NiAVObject* a_o, std::vector<RE::NiAVObject*>& a_out, int a_depth)
		{
			if (!a_o || a_depth > 48) {
				return;
			}
			if (a_o->AsGeometry() && a_o->name.c_str() && !a_o->flags.any(RE::NiAVObject::Flag::kHidden)) {
				std::string n = a_o->name.c_str();
				std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
				if (n.find("eyes") != std::string::npos || n.find("mouth") != std::string::npos || n.find("teeth") != std::string::npos || n.find("tongue") != std::string::npos) {
					a_out.push_back(a_o);
				}
			}
			if (auto* node = a_o->AsNode()) {
				for (auto& c : node->GetChildren()) {
					CollectInner(c.get(), a_out, a_depth + 1);
				}
			}
		}
	}

	std::vector<RE::NiAVObject*> HideInnerParts(RE::NiAVObject* a_root)
	{
		std::vector<RE::NiAVObject*> v;
		CollectInner(a_root, v, 0);
		for (auto* o : v) {
			o->flags.set(RE::NiAVObject::Flag::kHidden);
		}
		return v;
	}

	void ShowInnerParts(const std::vector<RE::NiAVObject*>& a_hidden)
	{
		for (auto* o : a_hidden) {
			o->flags.reset(RE::NiAVObject::Flag::kHidden);
		}
	}

	void NoteBodyPass()
	{
		++g_bodyPasses;
	}

	void LogStats()
	{
		logger::info("ghost: {} body pass(es); enabled {}; last player-centre distance from the entry plane {:+.1f}", g_bodyPasses, g_enabled.load(), g_lastDn);
	}
}
