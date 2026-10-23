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
#include "Util.h"

#include <xmmintrin.h>

// The narrow collision exception for walking through a portal.
//
// The player's character controller is a Havok character proxy (a phantom with a capsule). Each physics step it asks the world for contact points
// twice: closest points at its position (penetrations) and a linear cast along its intended motion; both are delivered to a point collector,
// hkpCdPointCollector::AddCdPoint (slot 1; hkpAllCdPointCollector and the game's bhkCharacterPointCollector override it). Removing the contact
// constraints is not enough (tested: the cast's time of impact still stops the capsule at the wall), so this hook drops the contact point itself:
// a point is dropped only if
//   - one of its two bodies is the player's own collision object,
//   - the other one is a collision object the portal was placed on (the surface it sits on, nothing else),
//   - the point lies inside that portal's oval (plus the capsule radius) and within the surface's depth,
//   - and the player's centre is inside the portal's oval prism right now.
// Everything else stays solid, for the player and for everyone else.
namespace hole
{
	namespace
	{
		using RE::NiPoint3;

		struct Slot
		{
			bool                        valid{ false };
			NiPoint3                    c{}, n{}, u{}, r{};
			float                       hw{ 44 }, hh{ 96 };
			std::array<const void*, 12> surf{};
			int                         nsurf{ 0 };
		};

		std::mutex               g_m;
		Slot                     g_slot[2];
		NiPoint3                 g_playerCentre{};
		std::atomic<const void*> g_playerColl{ nullptr };
		std::vector<Body>        g_extra;  // other actors near a portal (set by travel every frame), under g_m
		std::atomic<int>         g_extraCount{ 0 };
		std::atomic<void*>       g_driveCC{ nullptr };  // test: the controller whose horizontal velocity is overwritten inside the physics step
		std::atomic<float>       g_driveVx{ 0 }, g_driveVy{ 0 };
		std::atomic<bool>        g_enabled{ true };  // on by default: it was off, and only the test scripts switched it on, so in a normal session the player got stuck at the wall
		std::atomic<long>        g_seen{ 0 }, g_dropped{ 0 };
		std::atomic<int>         g_logged{ 0 };
		std::atomic<int>         g_keptLogged{ 0 };  // contacts kept while the player is over a floor portal (what may still hold the player up)

		// Flight after a fast exit: the horizontal velocity the solver is asked to achieve is overwritten inside the physics step (the game's own movement code
		// rewrites the controller's velocity during its update, before the step, so setting it from the frame hook never reaches the solver).
		std::atomic<bool>  g_flightOn{ false };
		std::atomic<float> g_flightVx{ 0 }, g_flightVy{ 0 };
		using Proc_t = void (*)(void*, const void*, const RE::hkArray<RE::hkpRootCdPoint>*, RE::hkpSimplexSolverInput*);
		Proc_t g_origProc{ nullptr };

		void ProcHook(void* a_this, const void* a_proxy, const RE::hkArray<RE::hkpRootCdPoint>* a_manifold, RE::hkpSimplexSolverInput* a_input)
		{
			g_origProc(a_this, a_proxy, a_manifold, a_input);
			if (a_input) {
				if (void* dcc = g_driveCC.load(std::memory_order_relaxed); dcc && reinterpret_cast<std::uint8_t*>(a_this) + 0x10 == reinterpret_cast<std::uint8_t*>(dcc)) {
					alignas(16) float dv[4];
					_mm_store_ps(dv, a_input->velocity.quad);
					const float ds = RE::bhkWorld::GetWorldScale();
					a_input->velocity = RE::hkVector4(g_driveVx.load() * ds, g_driveVy.load() * ds, dv[2], dv[3]);
					return;
				}
			}
			if (!g_flightOn.load(std::memory_order_relaxed) || !a_input) {
				return;
			}
			auto* player = RE::PlayerCharacter::GetSingleton();
			auto* cc = player ? player->GetCharController() : nullptr;
			if (!cc || reinterpret_cast<std::uint8_t*>(a_this) + 0x10 != reinterpret_cast<std::uint8_t*>(cc)) {
				return;
			}
			alignas(16) float v[4];
			_mm_store_ps(v, a_input->velocity.quad);
			const float s = RE::bhkWorld::GetWorldScale();
			a_input->velocity = RE::hkVector4(g_flightVx.load() * s, g_flightVy.load() * s, v[2], v[3]);
		}

		using AddCd_t = void (*)(RE::hkpCdPointCollector*, const RE::hkpCdPoint&);
		AddCd_t g_origAll{ nullptr };
		AddCd_t g_origChar{ nullptr };

		float Dot(const NiPoint3& a, const NiPoint3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

		bool InSet(const Slot& s, const void* a_c)
		{
			for (int i = 0; i < s.nsurf; ++i) {
				if (s.surf[i] == a_c) return true;
			}
			return false;
		}

		const void* Root(const RE::hkpCdBody* a_b)
		{
			for (int i = 0; a_b && i < 16; ++i) {
				if (!a_b->parent) return a_b;
				a_b = a_b->parent;
			}
			return a_b;
		}

		bool ShouldDrop(const RE::hkpCdPoint& a_pt)
		{
			if (!g_enabled.load(std::memory_order_relaxed)) {
				return false;
			}
			const void* pc = g_playerColl.load(std::memory_order_relaxed);
			const bool  extras = g_extraCount.load(std::memory_order_relaxed) > 0;
			if (!pc && !extras) {
				return false;
			}
			const void* ra = Root(a_pt.cdBodyA);
			const void* rb = Root(a_pt.cdBodyB);
			const void* other = pc ? (ra == pc ? rb : (rb == pc ? ra : nullptr)) : nullptr;
			if (!other && !extras) {
				return false;  // not a query of the player's capsule
			}
			Slot     s[2];
			NiPoint3 centre;
			{
				std::lock_guard lk(g_m);
				s[0] = g_slot[0];
				s[1] = g_slot[1];
				centre = g_playerCentre;
				if (!other) {
					// another actor that is near a portal: its own capsule centre is the one that must be inside the oval
					for (const auto& b : g_extra) {
						if (b.coll == ra) {
							other = rb;
							centre = b.centre;
							break;
						}
						if (b.coll == rb) {
							other = ra;
							centre = b.centre;
							break;
						}
					}
				}
			}
			if (!other) {
				return false;  // not a query of any capsule that may pass through a portal
			}
			g_seen.fetch_add(1, std::memory_order_relaxed);
			alignas(16) float q[4];
			_mm_store_ps(q, a_pt.contact.position.quad);
			const float    inv = 1.0f / RE::bhkWorld::GetWorldScale();
			const NiPoint3 cp{ q[0] * inv, q[1] * inv, q[2] * inv };
			for (int k = 0; k < 2; ++k) {
				// a floor portal: every surface under the oval counts, not only the ones its footprint rays hit when it was placed (G350: on uneven ground the capsule
				// rested on a neighbouring terrain piece or rock outside that set and was pushed back up for a second before it fell in); a wall keeps the set
				const bool floorPortal = s[k].n.z > 0.7f;
				if (!s[k].valid || (!floorPortal && !InSet(s[k], other))) continue;
				const NiPoint3 dp = centre - s[k].c;
				const float    px = Dot(dp, s[k].r), py = Dot(dp, s[k].u), pz = Dot(dp, s[k].n);
				// the player's centre must be inside the same oval (+8) that triggers the crossing, and near the plane (a fast fall covers 17 units a frame and the
				// centre used here is a frame old, hence the generous depth)
				const float    ex = px / (s[k].hw + 8.0f), ey = py / (s[k].hh + 8.0f);
				if (std::fabs(pz) >= 140.0f || ex * ex + ey * ey > 1.0f) continue;
				const NiPoint3 d = cp - s[k].c;
				const float    x = Dot(d, s[k].r), y = Dot(d, s[k].u), z = Dot(d, s[k].n);
				const float    cx = x / (s[k].hw + 32.0f), cy = y / (s[k].hh + 32.0f);  // 8 + the capsule radius 18 + slack
				if (cx * cx + cy * cy <= 1.0f && std::fabs(z) <= 80.0f) {
					g_dropped.fetch_add(1, std::memory_order_relaxed);
					if (g_logged.load() < 6) {
						g_logged.fetch_add(1);
						logger::info("hole: dropped a contact of the player's capsule with surface {:p} at local ({:.1f},{:.1f},{:.1f}) of portal {} (player centre local ({:.1f},{:.1f},{:.1f}))", other, x, y, z, k, px, py, pz);
					}
					return true;
				}
				if (floorPortal && pc && g_keptLogged.load() < 20) {
					g_keptLogged.fetch_add(1);
					logger::info("hole: kept a contact of the player's capsule over floor portal {} with surface {:p} at local ({:.1f},{:.1f},{:.1f}) (player centre local ({:.1f},{:.1f},{:.1f}))", k, other, x, y, z, px, py, pz);
				}
			}
			return false;
		}

		void HookAll(RE::hkpCdPointCollector* a_this, const RE::hkpCdPoint& a_pt)
		{
			if (ShouldDrop(a_pt)) {
				return;
			}
			g_origAll(a_this, a_pt);
		}

		void HookChar(RE::hkpCdPointCollector* a_this, const RE::hkpCdPoint& a_pt)
		{
			if (ShouldDrop(a_pt)) {
				return;
			}
			g_origChar(a_this, a_pt);
		}

		// ---- rigid bodies (props, ragdoll bones, grenades, the capsule body of an NPC or animal): a contact listener ----
		// The collector hooks above only see point queries (the player's proxy capsule and the like). A rigid body meets the wall through the solver's contact points, which Havok
		// reports to the listeners of the two bodies (hkpContactListener::contactPointCallback, slot 0) while the points are still being built. The game's own
		// bhkBackfaceContactListener does the same to drop a contact (read offline, id 79086: it sets bit 8, "disabled", in the contact point's properties flags). This listener is
		// added to every body that is near a portal and drops the contacts of that body with the surfaces the portal sits on, under the same conditions as for the player:
		// the contact lies inside the oval's volume and the body's own centre is inside the oval prism. The vtable is built by hand (7 slots as in the game: 0 contact point
		// callback, 3 destructor, the rest empty) because the listener is only ever handed to Havok as a pointer.
		struct Tracked
		{
			NiPoint3 centre{};
			float    radius{ 20 };
			int      seen{ 0 };
		};
		std::mutex                                g_tm;
		std::map<const void*, Tracked>            g_tracked;  // hkpRigidBody* -> its entity's centre and size, refreshed every frame by travel
		std::atomic<long>                         g_rbSeen{ 0 }, g_rbDropped{ 0 };
		std::atomic<int>                          g_rbLogged{ 0 };

		void ListenerContact(void*, const RE::hkpContactPointEvent* a_e)
		{
			if (!g_enabled.load(std::memory_order_relaxed) || !a_e || !a_e->contactPoint || !a_e->contactPointProperties) {
				return;
			}
			NiPoint3    centre{};
			float       radius = 20.0f;
			const void* other = nullptr;
			{
				std::lock_guard lk(g_tm);
				for (int k = 0; k < 2; ++k) {
					if (auto it = g_tracked.find(a_e->bodies[k]); it != g_tracked.end() && a_e->bodies[1 - k]) {
						centre = it->second.centre;
						radius = it->second.radius;
						other = a_e->bodies[1 - k]->GetCollidable();
						break;
					}
				}
			}
			if (!other) {
				return;
			}
			Slot s[2];
			{
				std::lock_guard lk(g_m);
				s[0] = g_slot[0];
				s[1] = g_slot[1];
			}
			g_rbSeen.fetch_add(1, std::memory_order_relaxed);
			alignas(16) float q[4];
			_mm_store_ps(q, a_e->contactPoint->position.quad);
			const float    inv = 1.0f / RE::bhkWorld::GetWorldScale();
			const NiPoint3 cp{ q[0] * inv, q[1] * inv, q[2] * inv };
			for (int k = 0; k < 2; ++k) {
				if (!s[k].valid || !InSet(s[k], other)) continue;
				const NiPoint3 dp = centre - s[k].c;
				const float    px = Dot(dp, s[k].r), py = Dot(dp, s[k].u), pz = Dot(dp, s[k].n);
				// the body's centre must be inside the oval (widened by the body's size) and near the plane
				const float ex = px / (s[k].hw + radius), ey = py / (s[k].hh + radius);
				if (std::fabs(pz) >= radius + 140.0f || ex * ex + ey * ey > 1.0f) continue;
				const NiPoint3 d = cp - s[k].c;
				const float    x = Dot(d, s[k].r), y = Dot(d, s[k].u), z = Dot(d, s[k].n);
				const float    cx = x / (s[k].hw + radius + 8.0f), cy = y / (s[k].hh + radius + 8.0f);
				if (cx * cx + cy * cy <= 1.0f && std::fabs(z) <= 80.0f) {
					a_e->contactPointProperties->flags.set(RE::hkContactPointMaterial::Flag::kIsDisabled);
					g_rbDropped.fetch_add(1, std::memory_order_relaxed);
					if (g_rbLogged.load() < 12) {
						g_rbLogged.fetch_add(1);
						logger::info("hole: dropped a contact of a rigid body with surface {:p} at local ({:.1f},{:.1f},{:.1f}) of portal {} (body centre local ({:.1f},{:.1f},{:.1f}), size {:.0f}, event type {})", other, x, y, z, k, px, py, pz, radius, static_cast<int>(a_e->type));
					}
					return;
				}
			}
		}

		void ListenerNoop(void*) {}

		void* g_listenerVt[7] = { reinterpret_cast<void*>(&ListenerContact), reinterpret_cast<void*>(&ListenerNoop), reinterpret_cast<void*>(&ListenerNoop), reinterpret_cast<void*>(&ListenerNoop),
			reinterpret_cast<void*>(&ListenerNoop), reinterpret_cast<void*>(&ListenerNoop), reinterpret_cast<void*>(&ListenerNoop) };
		struct ListenerObj
		{
			void** vt;
		} g_listener{ g_listenerVt };

		RE::hkpContactListener* Listener() { return reinterpret_cast<RE::hkpContactListener*>(&g_listener); }
		int                     g_frame{ 0 };

		bool Patch(void** a_vt, void* a_hook, AddCd_t* a_orig, const char* a_name)
		{
			DWORD old = 0;
			if (!VirtualProtect(&a_vt[1], sizeof(void*), PAGE_READWRITE, &old)) {
				logger::error("hole: VirtualProtect failed for {}", a_name);
				return false;
			}
			*a_orig = reinterpret_cast<AddCd_t>(a_vt[1]);
			a_vt[1] = a_hook;
			VirtualProtect(&a_vt[1], sizeof(void*), old, &old);
			logger::info("hole: {} vtable {:p} slot 1 AddCdPoint patched (orig rva {:#x})", a_name, static_cast<void*>(a_vt), reinterpret_cast<std::uintptr_t>(*a_orig) - REL::Module::get().base());
			return true;
		}
	}

	bool Install()
	{
		auto** vtAll = reinterpret_cast<void**>(RE::VTABLE_hkpAllCdPointCollector[0].address());
		auto** vtChar = reinterpret_cast<void**>(RE::VTABLE_bhkCharacterPointCollector[0].address());
		bool   ok = Patch(vtAll, reinterpret_cast<void*>(&HookAll), &g_origAll, "hkpAllCdPointCollector");
		ok &= Patch(vtChar, reinterpret_cast<void*>(&HookChar), &g_origChar, "bhkCharacterPointCollector");
		{
			auto** vtL = reinterpret_cast<void**>(RE::VTABLE_bhkCharProxyController[0].address());
			DWORD  old = 0;
			if (VirtualProtect(&vtL[1], sizeof(void*), PAGE_READWRITE, &old)) {
				g_origProc = reinterpret_cast<Proc_t>(vtL[1]);
				vtL[1] = reinterpret_cast<void*>(&ProcHook);
				VirtualProtect(&vtL[1], sizeof(void*), old, &old);
				logger::info("hole: bhkCharProxyController listener vtable {:p} slot 1 ProcessConstraintsCallback patched (orig rva {:#x})", static_cast<void*>(vtL), reinterpret_cast<std::uintptr_t>(g_origProc) - REL::Module::get().base());
			} else {
				ok = false;
			}
		}
		return ok;
	}

	void Tick()
	{
		auto*       p = RE::PlayerCharacter::GetSingleton();
		auto*       cc = p ? p->GetCharController() : nullptr;
		const void* coll = nullptr;
		if (cc) {
			std::uintptr_t vt = 0;
			ReadSafeT(cc, vt);
			if (vt == RE::VTABLE_bhkCharProxyController[1].address()) {
				auto* pc = reinterpret_cast<RE::bhkCharProxyController*>(reinterpret_cast<std::uint8_t*>(cc) - 0x10);
				auto* hp = pc->GetCharacterProxy();
				if (hp && hp->shapePhantom) coll = hp->shapePhantom->GetCollidable();
			}
		}
		g_playerColl.store(coll);
		if (p) {
			std::lock_guard lk(g_m);
			g_playerCentre = p->GetPosition() + NiPoint3{ 0, 0, 64.0f };
		}
	}

	void SetExtra(const std::vector<Body>& a_bodies)
	{
		std::lock_guard lk(g_m);
		g_extra = a_bodies;
		g_extraCount.store(static_cast<int>(g_extra.size()), std::memory_order_relaxed);
	}

	const void* CollidableOf(RE::Actor* a_actor)
	{
		auto* cc = a_actor ? a_actor->GetCharController() : nullptr;
		if (!cc) {
			return nullptr;
		}
		std::uintptr_t vt = 0;
		ReadSafeT(cc, vt);
		if (vt != RE::VTABLE_bhkCharProxyController[1].address()) {
			return nullptr;
		}
		auto* pc = reinterpret_cast<RE::bhkCharProxyController*>(reinterpret_cast<std::uint8_t*>(cc) - 0x10);
		auto* hp = pc->GetCharacterProxy();
		return hp && hp->shapePhantom ? hp->shapePhantom->GetCollidable() : nullptr;
	}

	void Drive(RE::Actor* a_actor, float a_vx, float a_vy)
	{
		g_driveVx = a_vx;
		g_driveVy = a_vy;
		g_driveCC = a_actor ? static_cast<void*>(a_actor->GetCharController()) : nullptr;
	}

	void SetFlight(bool a_on, float a_vx, float a_vy)
	{
		g_flightVx = a_vx;
		g_flightVy = a_vy;
		g_flightOn = a_on;
	}

	void SetEnabled(bool a_on)
	{
		g_enabled = a_on;
		g_logged = 0;
		logger::info("hole: {}", a_on ? "ON" : "off");
	}

	void Update(int a_index, const portals::Portal& a_p, const std::vector<const void*>& a_surfaces)
	{
		std::lock_guard lk(g_m);
		Slot&           s = g_slot[a_index & 1];
		s = Slot{};
		s.valid = a_p.valid;
		s.c = a_p.c; s.n = a_p.n; s.u = a_p.u; s.r = a_p.r; s.hw = a_p.hw; s.hh = a_p.hh;
		for (const void* c : a_surfaces) {
			if (s.nsurf < static_cast<int>(s.surf.size())) s.surf[s.nsurf++] = c;
		}
	}

	void SetBody(RE::hkpRigidBody* a_rb, const RE::NiPoint3& a_centre, float a_radius)
	{
		bool added = false;
		{
			std::lock_guard lk(g_tm);
			auto [it, inserted] = g_tracked.try_emplace(a_rb);
			it->second.centre = a_centre;
			it->second.radius = a_radius;
			added = inserted;
		}
		if (added) {
			a_rb->AddContactListener(Listener());
		}
	}

	void RemoveBody(RE::hkpRigidBody* a_rb, bool a_detach)
	{
		{
			std::lock_guard lk(g_tm);
			g_tracked.erase(a_rb);
		}
		if (a_detach) {
			a_rb->RemoveContactListener(Listener());
		}
	}

	int TrackedCount()
	{
		std::lock_guard lk(g_tm);
		return static_cast<int>(g_tracked.size());
	}

	// ---- projectiles ----
	// A projectile meets a wall through Havok callbacks (the callers of the vtable's AddImpact, id 44203 is the base implementation every projectile class ends in) that record an
	// impact; ProcessImpacts later explodes or sticks it. Dropping the record for a hit on the surface a portal sits on, inside the oval, while the projectile's own centre is inside the
	// oval's column, lets it fly on into the wall volume (its phantom is moved by the engine, it is not stopped by the contact); the travel sweep then carries it when its centre crosses
	// the plane. No margin guess: G97b carried a firebolt 57 units early with a 60-unit margin and G97c carried nothing with a few units.
	namespace
	{
		using AddImpact_t = void* (*)(RE::Projectile*, RE::TESObjectREFR*, const NiPoint3*, const NiPoint3*, const void*, std::int32_t, std::uint8_t);
		AddImpact_t       g_origAddImpact{ nullptr };
		std::atomic<bool> g_projHole{ false };
		std::atomic<long> g_projSeen{ 0 }, g_projDropped{ 0 };
		std::atomic<int>  g_impactLogged{ 0 };  // G111: every wall hit is logged for the first 150 (who hit what, where): does a twin stream at the exit hit anything?
		const hooks::Spec kAddImpact{ "Projectile::AddImpact (base)", 44203, { 0x4C, 0x8B, 0xDC, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83 } };

		bool DropProjectileHit(const void* a_coll, const NiPoint3& a_hit, const NiPoint3& a_centre)
		{
			Slot s[2];
			{
				std::lock_guard lk(g_m);
				s[0] = g_slot[0];
				s[1] = g_slot[1];
			}
			const void* root = Root(reinterpret_cast<const RE::hkpCdBody*>(a_coll));
			for (int k = 0; k < 2; ++k) {
				if (!s[k].valid || (!InSet(s[k], a_coll) && !InSet(s[k], root))) continue;
				const NiPoint3 dp = a_centre - s[k].c;
				const float    px = Dot(dp, s[k].r), py = Dot(dp, s[k].u), pz = Dot(dp, s[k].n);
				const float    ex = px / (s[k].hw + 8.0f), ey = py / (s[k].hh + 8.0f);
				if (std::fabs(pz) >= 140.0f || ex * ex + ey * ey > 1.0f) continue;  // the projectile's centre must be inside the oval's column, near the plane
				const NiPoint3 d = a_hit - s[k].c;
				const float    x = Dot(d, s[k].r), y = Dot(d, s[k].u), z = Dot(d, s[k].n);
				const float    cx = x / (s[k].hw + 8.0f), cy = y / (s[k].hh + 8.0f);
				if (cx * cx + cy * cy <= 1.0f && std::fabs(z) <= 80.0f) {
					return true;
				}
			}
			return false;
		}

		void* HookAddImpact(RE::Projectile* a_p, RE::TESObjectREFR* a_ref, const NiPoint3* a_loc, const NiPoint3* a_vel, const void* a_coll, std::int32_t a_a6, std::uint8_t a_a7)
		{
			if (g_projHole.load(std::memory_order_relaxed) && a_p && a_loc && a_coll) {
				g_projSeen.fetch_add(1, std::memory_order_relaxed);
				travel::NoteImpact(a_p, a_ref);
				if (g_impactLogged.load(std::memory_order_relaxed) < 150) {
					g_impactLogged.fetch_add(1, std::memory_order_relaxed);
					const auto* b = a_p->GetProjectileBase();
					logger::info("hole: impact of {} {:08X} at ({:.1f},{:.1f},{:.1f}) on ref {:08X} '{}' (projectile at ({:.1f},{:.1f},{:.1f}), living {:.3f})", a_p->IsFlameProjectile() ? "flame" : a_p->IsBeamProjectile() ? "beam" : (b && b->IsCone()) ? "cone" : a_p->IsMissileProjectile() ? "missile" : "projectile",
						a_p->GetFormID(), a_loc->x, a_loc->y, a_loc->z, a_ref ? a_ref->GetFormID() : 0u, a_ref && a_ref->GetName() ? a_ref->GetName() : "", a_p->GetPosition().x, a_p->GetPosition().y, a_p->GetPosition().z, a_p->GetProjectileRuntimeData().livingTime);
				}
				// a beam's own position stays at the caster's hand, so its hit on the entry wall is judged by the hit point itself (G340/G342: Chain Lightning chained from that hit at the
				// entry wall to a target near the exit, and Sparks' hits there flashed the picture dark)
				if (DropProjectileHit(a_coll, *a_loc, a_p->IsBeamProjectile() ? *a_loc : a_p->GetPosition())) {
					travel::NoteDroppedImpact(a_p, *a_loc);
					const long n = g_projDropped.fetch_add(1, std::memory_order_relaxed) + 1;
					if (n <= 12) {
						logger::info("hole: dropped the wall impact of projectile {:08X} at ({:.1f},{:.1f},{:.1f}), surface {:p} (projectile at ({:.1f},{:.1f},{:.1f}))", a_p->GetFormID(), a_loc->x, a_loc->y, a_loc->z, a_coll, a_p->GetPosition().x, a_p->GetPosition().y, a_p->GetPosition().z);
					}
					return nullptr;
				}
			}
			return g_origAddImpact(a_p, a_ref, a_loc, a_vel, a_coll, a_a6, a_a7);
		}
	}

	bool InstallProjectileHook()
	{
		static bool tried = false;
		if (!tried) {
			tried = true;
			if (hooks::Install(kAddImpact, reinterpret_cast<void*>(&HookAddImpact), reinterpret_cast<void**>(&g_origAddImpact))) {
				g_projHole = true;
			}
		}
		return g_origAddImpact != nullptr;
	}

	void SetProjectileHole(bool a_on)
	{
		g_projHole = a_on && g_origAddImpact != nullptr;
		logger::info("hole: the projectile wall-impact exception is {} (hook {})", g_projHole.load() ? "ON" : "off", g_origAddImpact ? "installed" : "not installed");
	}

	bool ProjectileHoleOn()
	{
		return g_projHole.load();
	}

	void LogStats()
	{
		logger::info("hole: contact points of the player's capsule seen {}, dropped {}; rigid bodies: {} tracked, contact points seen {}, dropped {}; projectile impacts seen {}, dropped {}", g_seen.load(), g_dropped.load(), TrackedCount(), g_rbSeen.load(), g_rbDropped.load(), g_projSeen.load(), g_projDropped.load());
		g_seen = 0;
		g_dropped = 0;
		g_rbSeen = 0;
		g_rbDropped = 0;
		g_projSeen = 0;
		g_projDropped = 0;
	}
}
