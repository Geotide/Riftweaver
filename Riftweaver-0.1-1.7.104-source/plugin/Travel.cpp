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

// Things that fly or fall through a portal (stage S7): projectiles first (arrows and the missile-like spells: firebolt, ice spike, ...).
//
// Once per frame (game thread, start of the frame function) every active missile projectile is swept against the two portals: when its next step would carry it through a
// portal's plane inside the oval, it is carried through the pair instead: position (just in front of the exit plane), velocity and orientation go through the same transform as the
// player's camera. The engine's own projectile update then moves it on from there. Nothing needs to be done about the wall behind the entry oval: the projectile is gone from it
// before its collision test could hit it, which is why the sweep looks one frame ahead.
namespace travel
{
	namespace
	{
		using RE::NiPoint3;

		std::atomic<bool> g_enabled{ true };
		std::atomic<float> g_lookFrames{ 1.25f };  // how many frames ahead the sweep looks
		std::atomic<bool> g_verbose{ false };
		std::atomic<bool> g_relaunch{ false };
		std::atomic<bool> g_selfHit{ true };  // a carried projectile can hit its own shooter (shoot yourself through a portal)
		double            g_castLeft{ 0 };  // test: a real cast by the player in progress (seconds left until it is finished)
		RE::MagicSystem::CastingSource g_castSource{ RE::MagicSystem::CastingSource::kRightHand };
		std::atomic<int>  g_projectiles{ 0 };
		int               g_logged{ 0 };
		double            g_dt{ 1.0 / 60.0 };

		// Test projectiles that get a given speed a few frames after the launch (the engine initialises them later and resets the velocity written at launch)
		std::map<std::uint32_t, float> g_boost;
		// the last position of each projectile and the tick it was seen at: the engine's real step between two of our ticks (see TickProjectiles)
		std::map<std::uint32_t, std::pair<NiPoint3, int>> g_projPrev;
		// G366: Unrelenting Force carried through a portal reached the bandit past the exit but did not move it (the same shout without a portal pushed it ~78 units):
		// the engine pushes from the caster, who stands on the other side of the wall. So a carried projectile remembers where its shooter is seen from the exit (the
		// shooter's position mapped through the portal), and when it hits an actor its stagger effects push from there (game thread, queued from the impact hook)
		struct CarriedFrom
		{
			NiPoint3                              src;
			const void*                           proj{ nullptr };  // the projectile object (G367: a later projectile got the same id and was pushed as if carried)
			std::chrono::steady_clock::time_point at;
		};
		std::mutex                              g_carriedMutex;
		std::map<std::uint32_t, CarriedFrom>    g_carriedFrom;  // projectile id -> mapped shooter position
		struct PendingPush
		{
			RE::ActorHandle actor;
			NiPoint3        src;
			float           magnitude{ 0 };
			std::uint32_t   proj{ 0 };
		};
		std::vector<PendingPush>                g_pushes;
		std::atomic<float>                      g_pushScale{ 2.0f };  // G367: the effect's magnitude as is moved the bandit ~40 units, the engine's own push ~78 (G366)
		std::atomic<int>                        g_pushLogged{ 0 };
		// wall impacts of projectiles that the oval exception dropped (hole, physics thread): formID -> hit point; a projectile whose impact was dropped and that is
		// behind that portal's plane at a later tick without having been carried is carried from the hit point (the safety net under the sweep)
		std::mutex                                g_dropMutex;
		std::map<std::uint32_t, std::pair<NiPoint3, int>> g_dropped;
		bool g_approach{ false };  // test (autoshot): a projectile is within 300 units of a portal's plane and inside its extended oval this tick, so the frames of the approach are taken too
		// Projectiles that went through a portal: followed in the log until they end (where did the arrow stick, where did the spell explode)
		std::map<std::uint32_t, RE::ObjectRefHandle> g_follow;
		int g_followTick{ 0 };
		std::map<std::uint32_t, int> g_axesLater;  // follow ticks after a carry whose node axes were logged (the engine's own frames may realign the node)

		float    Dot(const NiPoint3& a, const NiPoint3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
		float    Len(const NiPoint3& a) { return std::sqrt(Dot(a, a)); }
		NiPoint3 Norm(const NiPoint3& a)
		{
			const float l = Len(a);
			return l > 1e-6f ? a * (1.0f / l) : NiPoint3{ 1, 0, 0 };
		}
		NiPoint3 MapPoint(const portals::Portal& A, const portals::Portal& B, const NiPoint3& p)
		{
			const NiPoint3 d = p - A.c;
			return B.c + B.r * (-Dot(d, A.r)) + B.u * Dot(d, A.u) + B.n * (-Dot(d, A.n));
		}
		NiPoint3 MapDir(const portals::Portal& A, const portals::Portal& B, const NiPoint3& v)
		{
			return B.r * (-Dot(v, A.r)) + B.u * Dot(v, A.u) + B.n * (-Dot(v, A.n));
		}

		// Does the segment p0 -> p1 pass from the front of A's plane to the back inside its oval? On success a_t is the fraction along the segment and a_hit the crossing point.
		bool Crosses(const portals::Portal& A, const NiPoint3& p0, const NiPoint3& p1, float& a_t, NiPoint3& a_hit)
		{
			const float d0 = Dot(p0 - A.c, A.n), d1 = Dot(p1 - A.c, A.n);
			if (d0 <= 0.0f || d1 > 0.0f) {
				return false;
			}
			a_t = d0 / (d0 - d1);
			a_hit = p0 + (p1 - p0) * a_t;
			const NiPoint3 d = a_hit - A.c;
			// the same oval as the wall-impact exception (hole: half axes + 8): G356, an arrow 93.8 units below the violet centre (hh 96) had its wall impact dropped
			// but was refused here (0.97 * hh = 93.1), so it flew on behind the wall; whatever the wall lets through must be carried
			const float    lx = Dot(d, A.r) / (A.hw + 8.0f), ly = Dot(d, A.u) / (A.hh + 8.0f);
			return lx * lx + ly * ly <= 1.0f;
		}

		// A projectile cannot be moved: writing its position (data.location) is undone by the engine's own movement, which continues from the collision phantom's position (G58: the carried
		// arrow was moved one frame along the mapped velocity from the ENTRY and hit the wall there). So it is carried by launching a new projectile of the same kind at the exit with the
		// mapped direction and speed, and ending the old one.
		// ---- shoot yourself (09:30Z) ----
		// G403/G404: clearing the system group in the phantom's stored filter did not let a carried arrow or Firebolt hit the player (the Firebolt flew through his hips); the
		// projectile asks its virtual GetCollisionGroup (slot 0xBB) for the shooter's group every frame when it casts. For a projectile carried through a portal that call answers
		// "no group" from then on. The entry is keyed by the form id, checked against the pointer and the living time (a new projectile reusing the id starts at 0).
		using GetCollisionGroup_t = std::uint32_t (*)(RE::Projectile*);
		GetCollisionGroup_t g_origGroupMissile{ nullptr };
		GetCollisionGroup_t g_origGroupArrow{ nullptr };
		std::mutex          g_selfHitMutex;
		std::unordered_map<std::uint32_t, std::pair<RE::Projectile*, float>> g_selfHitSet;
		std::unordered_map<std::uint32_t, std::pair<RE::Projectile*, RE::ObjectRefHandle>> g_clearedShooter;  // projectiles whose shooter was cleared, and that shooter
		std::atomic<int>    g_groupLogged{ 0 };

		// G405: with the group gone the Firebolt still flew through the player's hips, so the engine also leaves out the shooter by reference. When the path out of the exit
		// passes through the body of whoever fired the projectile, its shooter handle is cleared (the hit then counts as an unowned projectile's; the actor cause is kept).
		void NoteSelfHit(RE::Projectile* a_p, const NiPoint3& a_pos, const NiPoint3& a_dir)
		{
			if (!g_selfHit.load()) {
				return;
			}
			auto& rd = a_p->GetProjectileRuntimeData();
			{
				std::lock_guard lk(g_selfHitMutex);
				if (g_selfHitSet.size() > 256) {
					g_selfHitSet.clear();
				}
				g_selfHitSet[a_p->GetFormID()] = { a_p, rd.livingTime };
			}
			auto sref = rd.shooter.get();
			auto* sh = sref ? sref->As<RE::Actor>() : nullptr;
			if (!sh) {
				return;
			}
			const NiPoint3 bmin = sh->GetBoundMin(), bmax = sh->GetBoundMax();
			const float    halfH = 0.5f * (bmax.z - bmin.z);
			const NiPoint3 c = sh->GetPosition() + NiPoint3{ 0, 0, halfH };
			const NiPoint3 d = Norm(a_dir);
			const NiPoint3 rel = c - a_pos;
			const float    t = rel.x * d.x + rel.y * d.y + rel.z * d.z;
			if (t <= 0.0f || t > 3000.0f) {
				return;
			}
			const NiPoint3 q = a_pos + d * t;
			const float    r = 0.5f * (std::max)(bmax.x - bmin.x, bmax.y - bmin.y) + 25.0f;
			const float    dx = q.x - c.x, dy = q.y - c.y, dz = q.z - c.z;
			const bool     onPath = dx * dx + dy * dy < r * r && std::abs(dz) < halfH + 25.0f;
			logger::info("travel: projectile {:08X}: its shooter {:08X} is {:.0f} units along the path out of the exit, {:.0f} off it sideways and {:.0f} off the body's middle in height{}", a_p->GetFormID(),
				sh->GetFormID(), t, std::sqrt(dx * dx + dy * dy), dz, onPath ? ": on the path, shooter cleared (it can hit them)" : "");
			if (onPath) {
				std::lock_guard lk(g_selfHitMutex);
				if (g_clearedShooter.size() > 64) {
					g_clearedShooter.clear();
				}
				g_clearedShooter[a_p->GetFormID()] = { a_p, rd.shooter };
				rd.shooter = RE::ObjectRefHandle{};
			}
		}

		// G407: with the shooter cleared an arrow hurt the player (8.5 a hit), but the Firebolt's 25 fire damage did nothing: a spell is applied through its shooter's caster. The
		// handle goes back when it hits an actor (the AddImpact hook), before the engine processes the hit.
		RE::ObjectRefHandle RestoreShooter(RE::Projectile* a_p, RE::TESObjectREFR* a_ref)
		{
			if (!a_p) {
				return {};
			}
			RE::ObjectRefHandle h;
			{
				std::lock_guard lk(g_selfHitMutex);
				const auto it = g_clearedShooter.find(a_p->GetFormID());
				if (it == g_clearedShooter.end()) {
					return {};
				}
				if (it->second.first == a_p) {
					h = it->second.second;
				}
				g_clearedShooter.erase(it);
			}
			if (h) {
				a_p->GetProjectileRuntimeData().shooter = h;
				logger::info("travel: projectile {:08X} hit ref {:08X}: its shooter handle is back for the hit", a_p->GetFormID(), a_ref ? a_ref->GetFormID() : 0u);
			}
			return h;
		}

		std::uint32_t GroupFor(RE::Projectile* a_p, GetCollisionGroup_t a_orig)
		{
			const std::uint32_t g = a_orig ? a_orig(a_p) : 0;
			if (!g_selfHit.load() || !a_p) {
				return g;
			}
			bool carried = false;
			{
				std::lock_guard lk(g_selfHitMutex);
				if (const auto it = g_selfHitSet.find(a_p->GetFormID()); it != g_selfHitSet.end()) {
					carried = it->second.first == a_p && a_p->GetProjectileRuntimeData().livingTime + 0.001f >= it->second.second;
					if (!carried) {
						g_selfHitSet.erase(it);
					}
				}
			}
			if (!carried) {
				return g;
			}
			// the value is either a bare group number or a whole filter word (layer in the low bits, group in the high 16): keep the layer, drop the group
			const std::uint32_t r = g > 0xFFFFu ? (g & 0x0000FFFFu) : 0u;
			if (g_groupLogged.fetch_add(1) < 8) {
				logger::info("travel: projectile {:08X} (carried): GetCollisionGroup {:#x} -> {:#x} (it can hit its shooter)", a_p->GetFormID(), g, r);
			}
			return r;
		}

		std::uint32_t Hook_GroupMissile(RE::Projectile* a_p) { return GroupFor(a_p, g_origGroupMissile); }
		std::uint32_t Hook_GroupArrow(RE::Projectile* a_p) { return GroupFor(a_p, g_origGroupArrow); }

		void PatchGroup(void** a_vt, void* a_hook, GetCollisionGroup_t* a_orig, const char* a_name)
		{
			constexpr std::size_t kSlot = 0xBB;
			DWORD                 old = 0;
			if (!VirtualProtect(&a_vt[kSlot], sizeof(void*), PAGE_READWRITE, &old)) {
				logger::error("travel: VirtualProtect failed for the {} vtable (GetCollisionGroup)", a_name);
				return;
			}
			*a_orig = reinterpret_cast<GetCollisionGroup_t>(a_vt[kSlot]);
			a_vt[kSlot] = a_hook;
			VirtualProtect(&a_vt[kSlot], sizeof(void*), old, &old);
			logger::info("travel: {} GetCollisionGroup (slot 0xBB, rva {:#x}) patched: a carried projectile can hit its shooter", a_name, reinterpret_cast<std::uintptr_t>(*a_orig) - REL::Module::get().base());
		}

		void InstallSelfHitHooks()
		{
			static bool done = false;
			if (done) {
				return;
			}
			done = true;
			PatchGroup(reinterpret_cast<void**>(RE::VTABLE_MissileProjectile[0].address()), reinterpret_cast<void*>(&Hook_GroupMissile), &g_origGroupMissile, "MissileProjectile");
			PatchGroup(reinterpret_cast<void**>(RE::VTABLE_ArrowProjectile[0].address()), reinterpret_cast<void*>(&Hook_GroupArrow), &g_origGroupArrow, "ArrowProjectile");
		}

		void CarryProjectile(RE::Projectile* a_p, const portals::Portal& A, const portals::Portal& B, const NiPoint3& a_hit, const NiPoint3& a_vel)
		{
			const NiPoint3 v2 = MapDir(A, B, a_vel);
			const NiPoint3 dir = Norm(v2);
			auto&          rd = a_p->GetProjectileRuntimeData();
			{
				NiPoint3 shooterPos = a_hit - Norm(a_vel) * 200.0f;
				if (auto sh = rd.shooter.get()) {
					shooterPos = sh->GetPosition();
				}
				std::lock_guard lk(g_carriedMutex);
				if (g_carriedFrom.size() > 256) {
					g_carriedFrom.clear();
				}
				g_carriedFrom[a_p->GetFormID()] = { MapPoint(A, B, shooterPos), a_p, std::chrono::steady_clock::now() };
			}
			if (!g_relaunch.load()) {
				// Move the original. Its movement continues from the collision phantom's transform, not from the reference's position (G58), so the phantom is moved too.
				const NiPoint3 pos2 = MapPoint(A, B, a_hit) + B.n * 6.0f + dir * 1.0f;
				rd.linearVelocity = v2;
				rd.velocity = v2;
				// the angle goes in BEFORE the position, and the 3D is refreshed after both: G97b showed a carried firebolt whose fire trail still pointed along the ENTRY direction (the
				// 43 degrees between the two portals' normals), because the angle was written after the position write and nothing pushed it into the node
				a_p->data.angle.x = -std::asin(std::clamp(dir.z, -1.0f, 1.0f));  // pitch, positive looking down
				a_p->data.angle.z = std::atan2(dir.x, dir.y);                    // heading
				a_p->SetPosition(pos2);
				// the step measured on the next tick must not include this jump (G403: measured from the entry, ~250 units, the look-ahead saw the next portal at once and
				// carried the arrow again before it flew past the player it was aimed at)
				g_projPrev.erase(a_p->GetFormID());
				killcam::OnCarried(a_p, pos2, dir);
				NoteSelfHit(a_p, pos2, dir);
				if (auto* b = a_p->GetProjectileBase(); b && b->IsCone()) {
					// a cone (Ice Storm) grows from its origin; the origin is carried through the map with it, so the cone keeps its width and points along the new direction
					auto&          cd = static_cast<RE::ConeProjectile*>(a_p)->GetConeRuntimeData();
					const NiPoint3 o2 = MapPoint(A, B, cd.origin);
					logger::info("travel: cone {:08X} carried: origin ({:.1f},{:.1f},{:.1f}) -> ({:.1f},{:.1f},{:.1f}), now at ({:.1f},{:.1f},{:.1f}) moving ({:.0f},{:.0f},{:.0f})", a_p->GetFormID(), cd.origin.x, cd.origin.y, cd.origin.z, o2.x, o2.y, o2.z, pos2.x, pos2.y, pos2.z, v2.x, v2.y, v2.z);
					cd.origin = o2;
				}
				if (auto* r3 = a_p->Get3D()) {
					{
						// which of the node's axes is the flight direction (before the carry the node still points along the entry direction): logged so the model's convention is on record
						const NiPoint3 od = Norm(a_vel);
						const auto&    R = r3->world.rotate.entry;
						logger::info("travel: projectile {:08X} node axes before the carry, dotted with the flight direction: column 0 {:.2f}, column 1 {:.2f}, column 2 {:.2f}", a_p->GetFormID(), R[0][0] * od.x + R[1][0] * od.y + R[2][0] * od.z, R[0][1] * od.x + R[1][1] * od.y + R[2][1] * od.z,
							R[0][2] * od.x + R[1][2] * od.y + R[2][2] * od.z);
					}
					// the model's axes: +Y is the flight direction, +Z up (roll kept level): the columns of the node's rotation are right, forward, up
					NiPoint3 right = std::fabs(dir.z) > 0.999f ? NiPoint3{ 1, 0, 0 } : Norm(NiPoint3{ dir.y, -dir.x, 0.0f });
					const NiPoint3 up{ right.y * dir.z - right.z * dir.y, right.z * dir.x - right.x * dir.z, right.x * dir.y - right.y * dir.x };
					RE::NiMatrix3  m;
					m.entry[0][0] = right.x; m.entry[0][1] = dir.x; m.entry[0][2] = up.x;
					m.entry[1][0] = right.y; m.entry[1][1] = dir.y; m.entry[1][2] = up.y;
					m.entry[2][0] = right.z; m.entry[2][1] = dir.z; m.entry[2][2] = up.z;
					r3->local.rotate = m;
					r3->world.rotate = m;
					r3->local.translate = pos2;
					r3->world.translate = pos2;
					auto logAxes = [&](const char* a_when) {
						const auto& R = r3->world.rotate.entry;
						logger::info("travel: projectile {:08X} node axes {}, dotted with the mapped direction ({:.3f},{:.3f},{:.3f}): column 0 {:.3f}, column 1 {:.3f}, column 2 {:.3f}; node at ({:.1f},{:.1f},{:.1f}), projectile angle (pitch {:.1f}, heading {:.1f}) deg", a_p->GetFormID(), a_when, dir.x, dir.y, dir.z,
							R[0][0] * dir.x + R[1][0] * dir.y + R[2][0] * dir.z, R[0][1] * dir.x + R[1][1] * dir.y + R[2][1] * dir.z, R[0][2] * dir.x + R[1][2] * dir.y + R[2][2] * dir.z, r3->world.translate.x, r3->world.translate.y, r3->world.translate.z,
							a_p->data.angle.x * 57.29578f, a_p->data.angle.z * 57.29578f);
					};
					a_p->Update3DPosition(true);
					// what the engine's refresh left in the node: G100 showed the third axis off by about twice the direction's pitch (-0.09 for an arrow flying 2.9 degrees down, -0.02 for a bolt at 0.5), the
					// refresh builds the rotation from the angle with the pitch the other way round
					logAxes("after the engine's refresh");
					// so the rotation is put back and pushed down the node tree (the engine realigns the node to the velocity in its own frames from here)
					r3->local.rotate = m;
					r3->world.rotate = m;
					RE::NiUpdateData ud{};
					ud.time = 0.0f;
					ud.flags.set(RE::NiUpdateData::Flag::kDirty);
					r3->Update(ud);
					logAxes("after the carry");
				}
				if (rd.unk0E0 && rd.unk0E0->phantom) {
					auto*               ph = rd.unk0E0->phantom.get();
					const float         sc = RE::bhkWorld::GetWorldScale();
					const RE::hkVector4 hp(pos2.x * sc, pos2.y * sc, pos2.z * sc, 0.0f);
					ph->motionState.transform.translation = hp;
					ph->motionState.sweptTransform.centerOfMass0 = hp;
					ph->motionState.sweptTransform.centerOfMass1 = hp;
					// shoot yourself through a portal (09:30Z): a projectile shares its shooter's collision system group (bits 16-31 of the filter) and so never hits them
					// (G402: an arrow out of the exit went through the player's legs); once carried, the group is cleared and it can hit whoever fired it
					auto& fi = ph->collidable.broadPhaseHandle.collisionFilterInfo.filter;
					if ((fi >> 16) != 0 && g_selfHit.load()) {
						static int selfLogged = 0;
						if (selfLogged++ < 6) {
							logger::info("travel: projectile {:08X}: collision system group {} cleared after the carry (it can now hit its shooter)", a_p->GetFormID(), fi >> 16);
						}
						fi &= 0x0000FFFFu;
					}
				} else if (g_logged < 12) {
					logger::warn("travel: projectile {:08X} has no collision phantom", a_p->GetFormID());
				}
				g_projectiles.fetch_add(1);
				g_follow[a_p->GetFormID()] = a_p->GetHandle();
				g_axesLater[a_p->GetFormID()] = 0;
				if (g_logged < 12) {
					++g_logged;
					logger::info("travel: projectile {:08X} moved in place through portal {} -> {}: hit ({:.1f},{:.1f},{:.1f}) -> ({:.1f},{:.1f},{:.1f}); velocity ({:.0f},{:.0f},{:.0f}) -> ({:.0f},{:.0f},{:.0f}); phantom {}", a_p->GetFormID(), A.colour, B.colour, a_hit.x,
						a_hit.y, a_hit.z, pos2.x, pos2.y, pos2.z, a_vel.x, a_vel.y, a_vel.z, v2.x, v2.y, v2.z, rd.unk0E0 && rd.unk0E0->phantom ? "moved" : "missing");
				}
				return;
			}
			const NiPoint3 newPos = MapPoint(A, B, a_hit) + B.n * 24.0f + dir * 1.0f;
			auto           shooterRef = rd.shooter.get();
			auto*          shooter = shooterRef ? shooterRef->As<RE::Actor>() : nullptr;
			const RE::Projectile::ProjectileRot rot{ -std::asin(std::clamp(dir.z, -1.0f, 1.0f)), std::atan2(dir.x, dir.y) };  // pitch (positive looking down), heading
			RE::Projectile::LaunchData data = rd.spell ? RE::Projectile::LaunchData(shooter, newPos, rot, rd.spell) :
			                                  (rd.ammoSource && rd.weaponSource) ? RE::Projectile::LaunchData(shooter, newPos, rot, rd.ammoSource, rd.weaponSource) :
			                                                                         RE::Projectile::LaunchData(a_p->GetProjectileBase(), shooter, newPos, rot);
			data.power = rd.power;
			data.scale = rd.scale;
			data.castingSource = rd.castingSource;
			data.useOrigin = true;
			RE::ProjectileHandle h;
			RE::Projectile::Launch(&h, data);
			auto np = h.get();
			if (!np) {
				logger::warn("travel: the projectile could not be relaunched at the exit");
				return;
			}
			g_boost[np->GetFormID()] = Len(v2);  // the engine starts it at its own launch speed: set the carried speed once it is initialised
			// end the old one without its impact effects
			rd.flags.set(RE::Projectile::Flags::kDestroyed);
			a_p->Disable();
			g_projectiles.fetch_add(1);
			g_follow[np->GetFormID()] = np->GetHandle();
			g_follow[a_p->GetFormID()] = a_p->GetHandle();
			if (g_logged < 12) {
				++g_logged;
				logger::info("travel: projectile {:08X} '{}' through portal {} -> {}: hit ({:.1f},{:.1f},{:.1f}) -> ({:.1f},{:.1f},{:.1f}); velocity ({:.0f},{:.0f},{:.0f}) -> ({:.0f},{:.0f},{:.0f}) speed {:.0f}", a_p->GetFormID(),
					a_p->GetBaseObject() && a_p->GetBaseObject()->GetName() ? a_p->GetBaseObject()->GetName() : "", A.colour, B.colour, a_hit.x, a_hit.y, a_hit.z, newPos.x, newPos.y, newPos.z, a_vel.x, a_vel.y, a_vel.z, v2.x, v2.y, v2.z, Len(v2));
			}
		}

		// ---- loose props (rigid bodies) ----
		// The rotation of the portal map: R = [B.r B.u B.n] * diag(-1, 1, -1) * [A.r A.u A.n]^T (the same map the see-through camera and the player's crossing use).
		void MapMatrix(const portals::Portal& A, const portals::Portal& B, float R[3][3])
		{
			const float a[3][3] = { { A.r.x, A.u.x, A.n.x }, { A.r.y, A.u.y, A.n.y }, { A.r.z, A.u.z, A.n.z } };
			const float b[3][3] = { { B.r.x, B.u.x, B.n.x }, { B.r.y, B.u.y, B.n.y }, { B.r.z, B.u.z, B.n.z } };
			const float d[3] = { -1.0f, 1.0f, -1.0f };
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					R[i][j] = 0.0f;
					for (int k = 0; k < 3; ++k) {
						R[i][j] += b[i][k] * d[k] * a[j][k];
					}
				}
			}
		}

		// x, y, z, w of the quaternion of a rotation matrix
		void MatToQuat(const float R[3][3], float q[4])
		{
			const float tr = R[0][0] + R[1][1] + R[2][2];
			if (tr > 0.0f) {
				const float s = std::sqrt(tr + 1.0f) * 2.0f;
				q[3] = 0.25f * s; q[0] = (R[2][1] - R[1][2]) / s; q[1] = (R[0][2] - R[2][0]) / s; q[2] = (R[1][0] - R[0][1]) / s;
			} else if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
				const float s = std::sqrt(1.0f + R[0][0] - R[1][1] - R[2][2]) * 2.0f;
				q[3] = (R[2][1] - R[1][2]) / s; q[0] = 0.25f * s; q[1] = (R[0][1] + R[1][0]) / s; q[2] = (R[0][2] + R[2][0]) / s;
			} else if (R[1][1] > R[2][2]) {
				const float s = std::sqrt(1.0f + R[1][1] - R[0][0] - R[2][2]) * 2.0f;
				q[3] = (R[0][2] - R[2][0]) / s; q[0] = (R[0][1] + R[1][0]) / s; q[1] = 0.25f * s; q[2] = (R[1][2] + R[2][1]) / s;
			} else {
				const float s = std::sqrt(1.0f + R[2][2] - R[0][0] - R[1][1]) * 2.0f;
				q[3] = (R[1][0] - R[0][1]) / s; q[0] = (R[0][2] + R[2][0]) / s; q[1] = (R[1][2] + R[2][1]) / s; q[2] = 0.25f * s;
			}
		}

		// a * b (Hamilton product, x y z w)
		void QuatMul(const float a[4], const float b[4], float out[4])
		{
			out[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
			out[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
			out[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
			out[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
		}

		struct Pending
		{
			RE::ObjectRefHandle h;
			NiPoint3            dir;
			float               speed;
			int                 frames{ 0 };
		};
		std::vector<Pending>                g_pending;   // props spawned by the test, waiting for their 3D and body to load
		std::vector<RE::ObjectRefHandle>    g_traced;    // test props whose path is logged
		int                                 g_traceTick{ 0 };
		std::map<std::uint32_t, int>        g_cooldown;  // form id -> frames before this prop may be carried again
		std::atomic<int>                    g_props{ 0 };

		RE::bhkRigidBody* BodyOf(RE::TESObjectREFR* a_ref)
		{
			auto* root = a_ref ? a_ref->Get3D() : nullptr;
			auto* co = root ? root->GetCollisionObject() : nullptr;
			return co ? co->GetRigidBody() : nullptr;
		}

		bool IsDynamic(RE::hkpRigidBody* a_rb)
		{
			if (!a_rb) {
				return false;
			}
			switch (a_rb->motion.type.get()) {
			case RE::hkpMotion::MotionType::kDynamic:
			case RE::hkpMotion::MotionType::kSphereInertia:
			case RE::hkpMotion::MotionType::kBoxInertia:
			case RE::hkpMotion::MotionType::kThinBoxInertia:
				return true;
			default:
				return false;
			}
		}

		// ---- the shared passage: bodies near a portal are watched (their contacts with the portal's surface are dropped by hole's listener) and the ones that straddle a plane are
		// handed to the clip pass; an entity is carried to the other portal when its CENTRE crosses the plane, so that it can pass through gradually ----
		std::atomic<bool> g_passage{ false };  // test switch: gradual passage with the collision exception and the clip pass (off = carried as soon as it would touch the plane)
		std::atomic<bool> g_clipDraw{ true };  // test switch: the straddling entities are handed to the clip pass (off: everything else of the passage still runs)
		double            g_tickSumMs{ 0 }, g_tickMaxMs{ 0 };
		int               g_tickN{ 0 };

		struct Watch
		{
			RE::ObjectRefHandle            h;
			std::vector<RE::hkpRigidBody*> bodies;
			int                            seen{ 0 };
		};
		std::map<std::uint32_t, Watch> g_watch;  // form id -> the bodies registered with hole's contact listener
		int                            g_watchTick{ 0 };

		struct Straddle
		{
			RE::ObjectRefHandle h;
			int                 entry;   // the portal whose plane the entity crosses
			float               radius;
		};
		std::vector<Straddle> g_straddle;  // refreshed every frame

		struct PrevC
		{
			float    c0[2]{};
			NiPoint3 pos[2]{};
			int      tick[2]{ -10, -10 };
		};
		std::map<std::uint32_t, PrevC> g_prevC;  // form id -> distance of the centre from each portal's plane (and where it was) at the last frame it was seen

		// test: a prop pushed at a constant slow velocity (its own physics would drop it long before it reached the portal), and screenshots while something straddles a plane
		struct PropDrive
		{
			RE::ObjectRefHandle h;
			std::uint32_t       id{ 0 };
			NiPoint3            dir{};
			float               speed{ 0 };
			int                 frames{ 0 };
			NiPoint3            start{};
			bool                place{ false };    // put the prop at 'start' once its body exists
			bool                aligned{ false };  // the prop's long axis (local Z for a sword) has been turned along the direction of travel
		} g_drive;
		RE::ObjectRefHandle g_lastThrown;
		std::string         g_autoTag;
		int                 g_autoLeft{ 0 }, g_autoCount{ 0 }, g_autoEvery{ 12 };

		void WatchBodies(RE::TESObjectREFR* a_ref, const std::vector<RE::hkpRigidBody*>& a_bodies, const NiPoint3& a_centre, float a_radius)
		{
			auto& w = g_watch[a_ref->GetFormID()];
			w.h = a_ref->CreateRefHandle();
			w.seen = g_watchTick;
			for (auto* old : w.bodies) {
				if (std::find(a_bodies.begin(), a_bodies.end(), old) == a_bodies.end()) {
					hole::RemoveBody(old, true);
				}
			}
			for (auto* rb : a_bodies) {
				hole::SetBody(rb, a_centre, a_radius);
			}
			w.bodies = a_bodies;
		}

		bool Straddles(const portals::Portal& P, const NiPoint3& a_centre, float a_radius)
		{
			const NiPoint3 d = a_centre - P.c;
			if (std::fabs(Dot(d, P.n)) >= a_radius) {
				return false;
			}
			const float ex = Dot(d, P.r) / (P.hw + a_radius), ey = Dot(d, P.u) / (P.hh + a_radius);
			return ex * ex + ey * ey <= 1.0f;
		}

		void NoteStraddler(RE::TESObjectREFR* a_ref, int a_entry, float a_radius)
		{
			g_straddle.push_back({ a_ref->CreateRefHandle(), a_entry, a_radius });
		}

		// Bodies of entities that were not seen this frame are dropped from the listener. The entity may be gone (cell change, deleted): its bodies are then not touched.
		void EndWatch(const std::function<bool(RE::TESObjectREFR*, RE::hkpRigidBody*)>& a_stillOwns)
		{
			for (auto it = g_watch.begin(); it != g_watch.end();) {
				if (it->second.seen == g_watchTick) {
					++it;
					continue;
				}
				auto ref = it->second.h.get();
				for (auto* rb : it->second.bodies) {
					hole::RemoveBody(rb, ref && ref->Get3D() && a_stillOwns(ref.get(), rb));
				}
				it = g_watch.erase(it);
			}
		}

		void CarryProp(RE::TESObjectREFR* a_ref, RE::bhkRigidBody* a_body, const portals::Portal& A, const portals::Portal& B, const NiPoint3& a_hit, const NiPoint3& a_vel, float a_radius, bool a_clip = false)
		{
			const float s = RE::bhkWorld::GetWorldScale();
			float       R[3][3];
			MapMatrix(A, B, R);
			// angular velocity (Havok units already are radians per second)
			alignas(16) float av[4];
			_mm_store_ps(av, a_body->GetRigidBody()->motion.angularVelocity.quad);
			const float aw[3] = { R[0][0] * av[0] + R[0][1] * av[1] + R[0][2] * av[2], R[1][0] * av[0] + R[1][1] * av[1] + R[1][2] * av[2], R[2][0] * av[0] + R[2][1] * av[1] + R[2][2] * av[2] };
			// orientation: the old rotation, then the map
			RE::hkQuaternion qOld;
			a_body->GetRotation(qOld);
			alignas(16) float qo[4];
			_mm_store_ps(qo, qOld.vec.quad);
			float qm[4], qn[4];
			MatToQuat(R, qm);
			QuatMul(qm, qo, qn);
			const float ql = std::sqrt(qn[0] * qn[0] + qn[1] * qn[1] + qn[2] * qn[2] + qn[3] * qn[3]);
			RE::hkQuaternion qNew;
			qNew.vec = RE::hkVector4(qn[0] / ql, qn[1] / ql, qn[2] / ql, qn[3] / ql);

			const NiPoint3 v2 = MapDir(A, B, a_vel);
			// carried before it touches the plane: put clear of the exit wall; carried with its centre on the plane (gradual passage): exactly where the map puts it, the exit wall is let through
			const NiPoint3 newPos = MapPoint(A, B, a_hit) + (a_clip ? NiPoint3{ 0, 0, 0 } : B.n * (a_radius + 3.0f));
			RE::hkVector4  hp(newPos.x * s, newPos.y * s, newPos.z * s, 0.0f);
			a_body->SetPositionAndRotation(hp, qNew);
			a_body->SetLinearVelocity(RE::hkVector4(v2.x * s, v2.y * s, v2.z * s, 0.0f));
			a_body->SetAngularVelocity(RE::hkVector4(aw[0], aw[1], aw[2], 0.0f));
			a_ref->SetPosition(newPos);
			g_cooldown[a_ref->GetFormID()] = a_clip ? 6 : 20;
			g_prevC.erase(a_ref->GetFormID());
			if (g_drive.frames > 0 && g_drive.id == a_ref->GetFormID()) {
				g_drive.dir = Norm(v2);
			}
			if (a_clip) {
				shot::TriggerRing();  // test: the frames around a gradual passage
			}
			g_props.fetch_add(1);
			logger::info("travel: prop {:08X} '{}' through portal {} -> {}: hit ({:.1f},{:.1f},{:.1f}) -> ({:.1f},{:.1f},{:.1f}); velocity ({:.0f},{:.0f},{:.0f}) -> ({:.0f},{:.0f},{:.0f}); radius {:.1f}", a_ref->GetFormID(),
				a_ref->GetBaseObject() && a_ref->GetBaseObject()->GetName() ? a_ref->GetBaseObject()->GetName() : "", A.colour, B.colour, a_hit.x, a_hit.y, a_hit.z, newPos.x, newPos.y, newPos.z, a_vel.x, a_vel.y, a_vel.z, v2.x, v2.y, v2.z, a_radius);
		}

		void TickProps(const portals::Portal (&a_p)[2])
		{
			auto* player = RE::PlayerCharacter::GetSingleton();
			auto* cell = player ? player->GetParentCell() : nullptr;
			if (!cell) {
				return;
			}
			const float s = RE::bhkWorld::GetWorldScale();
			const float inv = 1.0f / s;

			// test props waiting for their physics body
			for (auto it = g_pending.begin(); it != g_pending.end();) {
				auto  ref = it->h.get();
				auto* body = ref ? BodyOf(ref.get()) : nullptr;
				if (body && IsDynamic(body->GetRigidBody())) {
					body->SetLinearVelocity(RE::hkVector4(it->dir.x * it->speed * s, it->dir.y * it->speed * s, it->dir.z * it->speed * s, 0.0f));
					logger::info("travel: prop {:08X} thrown at {:.0f} units/s after {} frame(s)", ref->GetFormID(), it->speed, it->frames);
					g_traced.push_back(it->h);
					it = g_pending.erase(it);
				} else if (++it->frames > 180) {
					logger::warn("travel: test prop never got a physics body");
					it = g_pending.erase(it);
				} else {
					++it;
				}
			}
			for (auto it = g_cooldown.begin(); it != g_cooldown.end();) {
				it = (--it->second <= 0) ? g_cooldown.erase(it) : std::next(it);
			}

			if (g_drive.frames > 0) {
				auto  ref = g_drive.h.get();
				auto* body = ref ? BodyOf(ref.get()) : nullptr;
				if (body && IsDynamic(body->GetRigidBody())) {
					if (g_drive.place) {
						RE::hkVector4 hp(g_drive.start.x * s, g_drive.start.y * s, g_drive.start.z * s, 0.0f);
						body->SetPosition(hp);
						ref->SetPosition(g_drive.start);
						g_drive.place = false;
					}
					if (!g_drive.aligned) {
						// turn the long axis (the blade of a sword is its local Z) along the direction of travel: the quaternion of the rotation from Z to dir
						const NiPoint3 d = g_drive.dir;
						const float    ax = -d.y, ay = d.x, al = std::sqrt(ax * ax + ay * ay);
						const float    ang = std::acos(std::clamp(d.z, -1.0f, 1.0f));
						if (al > 1e-4f) {
							const float sn = std::sin(ang * 0.5f) / al;
							RE::hkQuaternion q;
							q.vec = RE::hkVector4(ax * sn, ay * sn, 0.0f, std::cos(ang * 0.5f));
							body->SetRotation(q);
						}
						g_drive.aligned = true;
					}
					NiPoint3 v = g_drive.dir * (g_drive.speed * s);
					v.z += 337.0f * static_cast<float>(g_dt) * s;  // gravity pulls the body down by about half a step's worth of speed between two resets of the velocity: cancel it, or the sword sinks as it goes
					body->SetLinearVelocity(RE::hkVector4(v.x, v.y, v.z, 0.0f));
					body->SetAngularVelocity(RE::hkVector4(0.0f, 0.0f, 0.0f, 0.0f));
					--g_drive.frames;
				} else {
					g_drive.frames = (ref ? g_drive.frames - 1 : 0);
				}
			}

			const float look = static_cast<float>(g_dt) * g_lookFrames.load();
			for (int i = 0; i < 2; ++i) {
				if (Len(a_p[i].c - player->GetPosition()) > 2200.0f) {
					continue;
				}
				cell->ForEachReferenceInRange(a_p[i].c, 420.0f, [&](RE::TESObjectREFR* a_ref) {
					if (!a_ref || a_ref == player || a_ref->IsDisabled() || a_ref->As<RE::Actor>() || a_ref->As<RE::Projectile>()) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
					auto* body = BodyOf(a_ref);
					auto* rb = body ? body->GetRigidBody() : nullptr;
					if (!IsDynamic(rb)) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
					alignas(16) float lv[4], pv[4];
					_mm_store_ps(lv, rb->motion.linearVelocity.quad);
					const NiPoint3 vel{ lv[0] * inv, lv[1] * inv, lv[2] * inv };
					RE::hkVector4 hp;
					body->GetCenterOfMassWorld(hp);
					_mm_store_ps(pv, hp.quad);
					const NiPoint3 pos{ pv[0] * inv, pv[1] * inv, pv[2] * inv };
					auto*          root = a_ref->Get3D();
					const float    radius = root ? (std::min)(root->worldBound.radius, 90.0f) : 10.0f;
					if (g_passage.load()) {
						// gradual passage: the body is let into the wall inside the oval (hole's contact listener), drawn cut by the clip pass while it straddles the plane, and carried
						// when its centre crosses the plane
						WatchBodies(a_ref, { rb }, pos, radius);
						if (Straddles(a_p[i], pos, radius)) {
							NoteStraddler(a_ref, i, radius);
						}
						const float vn = Dot(vel, a_p[i].n);
						const float c0 = Dot(pos - a_p[i].c, a_p[i].n);
						const float c1 = c0 + vn * look;
						// The centre's distance from the plane and position last frame: the crossing is also caught as a sign change, because the look-ahead window (a frame or so of
						// travel) is missed when a frame takes longer than the one before it (G80: the sword went on through the wall and out of the back of it, uncarried).
						auto&      pc = g_prevC[a_ref->GetFormID()];
						const bool havePrev = pc.tick[i] == g_watchTick - 1;
						const float    prevC0 = pc.c0[i];
						const NiPoint3 prevPos = pc.pos[i];
						pc.c0[i] = c0;
						pc.pos[i] = pos;
						pc.tick[i] = g_watchTick;
						const bool free = vn < -20.0f && !g_cooldown.contains(a_ref->GetFormID());
						auto       inOval = [&](const NiPoint3& a_at) {
                            const NiPoint3 dd = a_at - a_p[i].c;
                            const float    lx = Dot(dd, a_p[i].r) / (a_p[i].hw * 0.95f), ly = Dot(dd, a_p[i].u) / (a_p[i].hh * 0.95f);
                            return lx * lx + ly * ly <= 1.0f;
						};
						if (free && c0 > 0.0f && c1 <= 0.0f) {
							const float    t = c0 / (c0 - c1);
							const NiPoint3 hit = pos + vel * look * std::clamp(t, 0.0f, 1.0f);
							if (inOval(hit)) {
								CarryProp(a_ref, body, a_p[i], a_p[1 - i], hit, vel, radius, true);
							}
						} else if (free && havePrev && prevC0 > 0.0f && c0 <= 0.0f) {
							const NiPoint3 crossed = prevPos + (pos - prevPos) * (prevC0 / (prevC0 - c0));
							if (inOval(crossed)) {
								CarryProp(a_ref, body, a_p[i], a_p[1 - i], pos, vel, radius, true);  // the body is already past the plane: the map takes where it is now
							}
						}
						return RE::BSContainer::ForEachResult::kContinue;
					}
					if (Len(vel) < 60.0f || g_cooldown.contains(a_ref->GetFormID())) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
					// the sweep looks at the body's centre; the oval is shrunk by the body's radius so it fits through
					const float d0 = Dot(pos - a_p[i].c, a_p[i].n);
					const float d1 = Dot(pos + vel * look - a_p[i].c, a_p[i].n);
					if (d0 <= 0.0f || d1 > radius * 0.5f) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
					const float    t = (d0 - radius * 0.5f) / (d0 - d1 > 1e-3f ? d0 - d1 : 1.0f);
					const NiPoint3 hit = pos + vel * look * std::clamp(t, 0.0f, 1.0f);
					const NiPoint3 dd = hit - a_p[i].c;
					const float    lx = Dot(dd, a_p[i].r) / (std::max)(a_p[i].hw - radius, 8.0f), ly = Dot(dd, a_p[i].u) / (std::max)(a_p[i].hh - radius, 8.0f);
					if (lx * lx + ly * ly > 1.0f) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
					CarryProp(a_ref, body, a_p[i], a_p[1 - i], hit, vel, radius);
					return RE::BSContainer::ForEachResult::kContinue;
				});
			}

			// log the path of the test props
			if ((++g_traceTick % 6) == 0) {
				for (auto it = g_traced.begin(); it != g_traced.end();) {
					auto  ref = it->get();
					auto* body = ref ? BodyOf(ref.get()) : nullptr;
					if (!body || !body->GetRigidBody()) {
						it = g_traced.erase(it);
						continue;
					}
					alignas(16) float lv[4], pv[4];
					_mm_store_ps(lv, body->GetRigidBody()->motion.linearVelocity.quad);
					RE::hkVector4 hp;
					body->GetCenterOfMassWorld(hp);
					_mm_store_ps(pv, hp.quad);
					logger::info("travel: prop trace {:08X} at ({:.1f},{:.1f},{:.1f}) velocity ({:.0f},{:.0f},{:.0f})", ref->GetFormID(), pv[0] * inv, pv[1] * inv, pv[2] * inv, lv[0] * inv, lv[1] * inv, lv[2] * inv);
					++it;
				}
			}
		}

		// ---- other actors (NPCs, creatures): walking through as the player does ----
		// Every frame the actors within reach of a portal are handed to the collision exception (their contacts with the portal's surface are dropped while the actor's capsule centre is inside
		// the oval prism), and the actor's capsule centre is compared with the previous frame's: from the front of the plane to the back inside the oval carries the actor through the pair
		// (position, facing, controller velocity, fall bookkeeping), like the player's crossing.
		struct ActorTrack
		{
			NiPoint3 centre{};
			bool     seen{ false };
			float    sink{ 0.0f };  // trapdoor: the speed (units/s) at which an actor standing on a floor portal is being lowered into it
		};
		std::atomic<bool> g_trapdoor{ true };  // a living actor standing over a floor portal's oval drops into it (Riftweaver.ini Trapdoor, default on; G327-G331)
		std::map<std::uint32_t, ActorTrack> g_actorPrev;
		std::map<std::uint32_t, int>        g_actorCooldown;
		RE::ActorHandle                     g_testNpc;
		std::atomic<int>                    g_actorsCarried{ 0 };
		int                                 g_npcTraceFrames{ 0 };  // after the carry the test NPC's path is logged for a while
		std::atomic<int>                    g_driveColour{ -1 };  // test: the portal the test NPC is walked into (-1 = none)
		std::atomic<float>                  g_driveSpeed{ 0 };
		NiPoint3                            g_afterDir{};        // test: after the carry the test NPC goes on walking along the mapped direction ...
		int                                 g_afterFrames{ 0 };  // ... for this many frames

		// the capsule's rigid body of an NPC or animal (the rigid-body character controller); null for any other kind of controller
		RE::hkpRigidBody* CapsuleBodyOf(RE::Actor* a_a)
		{
			auto* cc = a_a ? a_a->GetCharController() : nullptr;
			if (!cc) {
				return nullptr;
			}
			std::uintptr_t vt = 0;
			ReadSafeT(cc, vt);
			if (vt != RE::VTABLE_bhkCharRigidBodyController[0].address()) {
				return nullptr;
			}
			auto* rc = reinterpret_cast<RE::bhkCharRigidBodyController*>(cc);
			return rc->charRigidBody.rigidBody ? rc->charRigidBody.rigidBody->GetRigidBody() : nullptr;
		}

		float ActorHalfHeight(RE::Actor* a_a)
		{
			return std::clamp(a_a->GetBoundMax().z * 0.5f, 10.0f, 90.0f);
		}

		void CarryActor(RE::Actor* a_a, const portals::Portal& A, const portals::Portal& B, const NiPoint3& a_centre, const NiPoint3& a_vel, float a_halfH)
		{
			const NiPoint3 nc = MapPoint(A, B, a_centre);
			NiPoint3 feet = nc - NiPoint3{ 0, 0, a_halfH };
			// two portals are rarely at the same height above their floors: the mapped feet can end up under the exit's floor, so they are raised onto it
			if (const auto ray = world::Raycast(nc + NiPoint3{ 0, 0, 10.0f }, NiPoint3{ 0, 0, -1 }, a_halfH * 2.0f + 200.0f, RE::COL_LAYER::kLOS); ray.hit) {
				const float floorZ = nc.z + 10.0f - ray.dist;
				if (floorZ > feet.z - 1.0f) {
					logger::info("travel: actor {:08X}: feet raised {:.1f} units onto the exit's floor", a_a->GetFormID(), floorZ + 1.0f - feet.z);
					feet.z = floorZ + 1.0f;
				}
			}
			const float    h = a_a->GetAngleZ();
			const NiPoint3 mf = Norm(MapDir(A, B, NiPoint3{ std::sin(h), std::cos(h), 0.0f }));
			const float    yaw = std::atan2(mf.x, mf.y);
			const NiPoint3 v2 = MapDir(A, B, a_vel);
			a_a->SetPosition(feet, true);
			a_a->SetHeading(yaw);
			if (auto* cc = a_a->GetCharController()) {
				const float         s = RE::bhkWorld::GetWorldScale();
				const RE::hkVector4 hv(v2.x * s, v2.y * s, v2.z * s, 0.0f);
				cc->SetLinearVelocityImpl(hv);
				cc->outVelocity = hv;
				cc->initialVelocity = hv;
				cc->fallStartHeight = feet.z;
				cc->fallTime = 0.0f;
			}
			g_actorCooldown[a_a->GetFormID()] = 12;
			g_actorsCarried.fetch_add(1);
			logger::info("travel: actor {:08X} '{}' through portal {} -> {}: centre ({:.1f},{:.1f},{:.1f}) -> ({:.1f},{:.1f},{:.1f}); facing {:.1f} -> {:.1f} deg; velocity ({:.0f},{:.0f},{:.0f}) -> ({:.0f},{:.0f},{:.0f})", a_a->GetFormID(),
				a_a->GetName() ? a_a->GetName() : "", A.colour, B.colour, a_centre.x, a_centre.y, a_centre.z, nc.x, nc.y, nc.z, h * 57.29578f, yaw * 57.29578f, a_vel.x, a_vel.y, a_vel.z, v2.x, v2.y, v2.z);
		}

		// NPCs and creatures do not use the player's proxy character controller but a rigid-body one (G64: "controller rigid body"), which the collision exception does not cover,
		// so an actor cannot be let into the wall. Instead it is carried when its capsule would touch the portal's plane: the sweep looks one step ahead along the actor's velocity
		// and carries it as soon as the capsule's edge (its radius) reaches the plane inside the oval, like a prop.
		void TickActors(const portals::Portal (&a_p)[2])
		{
			auto* player = RE::PlayerCharacter::GetSingleton();
			auto* cell = player ? player->GetParentCell() : nullptr;
			if (!cell) {
				return;
			}
			std::vector<RE::Actor*> nearby;
			for (int i = 0; i < 2; ++i) {
				if (Len(a_p[i].c - player->GetPosition()) > 2500.0f) {
					continue;
				}
				cell->ForEachReferenceInRange(a_p[i].c, 380.0f, [&](RE::TESObjectREFR* a_ref) {
					auto* a = a_ref ? a_ref->As<RE::Actor>() : nullptr;
					if (a && a != player && !a->IsDead() && !a->IsInRagdollState() && !a->IsDisabled() && a->Is3DLoaded() && std::find(nearby.begin(), nearby.end(), a) == nearby.end()) {
						nearby.push_back(a);
					}
					return RE::BSContainer::ForEachResult::kContinue;
				});
			}
			std::vector<hole::Body> bodies;
			std::set<std::uint32_t> present;
			for (auto it = g_actorCooldown.begin(); it != g_actorCooldown.end();) {
				it = (--it->second <= 0) ? g_actorCooldown.erase(it) : std::next(it);
			}
			const float dt = static_cast<float>(g_dt);
			const float look = dt * g_lookFrames.load();
			for (auto* a : nearby) {
				const std::uint32_t id = a->GetFormID();
				present.insert(id);
				const float    halfH = ActorHalfHeight(a);
				const NiPoint3 centre = a->GetPosition() + NiPoint3{ 0, 0, halfH };
				if (const void* coll = hole::CollidableOf(a)) {
					bodies.push_back({ coll, centre });  // a proxy-controller actor (the player's kind): the collision exception covers it
				}
				auto& tr = g_actorPrev[id];
				RE::hkpRigidBody* capBody = g_passage.load() ? CapsuleBodyOf(a) : nullptr;
				if (capBody) {
					// gradual passage for an NPC or animal: the capsule's body is let into the wall inside the oval (hole's contact listener), the actor is drawn cut while it straddles the plane, and it
					// is carried (position, facing, controller velocity, fall bookkeeping, feet onto the exit floor) when its centre crosses the plane
					const NiPoint3 bmin = a->GetBoundMin(), bmax = a->GetBoundMax();
					const float    radius = std::clamp(0.5f * (std::max)(bmax.x - bmin.x, bmax.y - bmin.y), 6.0f, 40.0f);
					WatchBodies(a, { capBody }, centre, radius);
					// trapdoor: an actor standing over a floor portal's oval is lowered into it like a fall (its controller would keep it on the ground, the oval is open only for its body);
					// the crossing below then carries it as its centre goes through the plane
					bool sinking = false;
					for (int i = 0; i < 2 && g_trapdoor.load() && !g_actorCooldown.contains(id); ++i) {
						if (a_p[i].n.z < 0.7f) {
							continue;
						}
						const float    c0 = Dot(centre - a_p[i].c, a_p[i].n);
						const NiPoint3 dd = centre - a_p[i].c;
						// starts where the whole body fits; once sinking it goes on while the centre is anywhere over the oval (G376: a fighting bandit walked out of the
						// narrow start zone half way down and the sink stopped)
						const bool     started = tr.sink > 0.0f;
						const float    ax = started ? a_p[i].hw : (std::max)(a_p[i].hw - radius, 8.0f), ay = started ? a_p[i].hh : (std::max)(a_p[i].hh - radius, 8.0f);
						const float    lx = Dot(dd, a_p[i].r) / ax, ly = Dot(dd, a_p[i].u) / ay;
						if (c0 < -halfH || c0 > halfH + 40.0f || lx * lx + ly * ly > 1.0f) {
							continue;
						}
						if (tr.sink == 0.0f) {
							logger::info("travel: trapdoor: actor {:08X} '{}' stands over the floor portal {} (centre {:.1f} above its plane): dropping it in", id, a->GetName() ? a->GetName() : "", a_p[i].colour, c0);
						}
						if (tr.sink >= 450.0f) {
							// a fighting actor's AI holds it up against the step; after about 0.45 s of sinking it is carried anyway, just past the plane, falling at the sink speed
							const NiPoint3 fallV = a_p[i].n * -(std::max)(tr.sink, 500.0f);
							logger::info("travel: trapdoor: actor {:08X} carried after sinking (centre {:.1f} above the plane)", id, c0);
							CarryActor(a, a_p[i], a_p[1 - i], centre - a_p[i].n * (c0 + 10.0f), fallV, halfH);
							g_prevC.erase(id);
							tr.sink = 0.0f;
							sinking = true;
							break;
						}
						tr.sink = (std::min)(tr.sink + 1000.0f * dt, 1400.0f);
						// pulled toward the oval's middle as it sinks (its AI would otherwise walk it off the edge before its centre is through)
						const NiPoint3 side = dd - a_p[i].n * Dot(dd, a_p[i].n);
						const float    pull = (std::min)(1.0f, 20.0f * dt);
						a->SetPosition(a->GetPosition() - a_p[i].n * (tr.sink * dt) - side * pull, false);
						sinking = true;
						break;
					}
					if (!sinking) {
						tr.sink = 0.0f;
					}
					const NiPoint3 vel = tr.seen ? (centre - tr.centre) * (1.0f / dt) : NiPoint3{ 0, 0, 0 };
					for (int i = 0; i < 2; ++i) {
						if (Straddles(a_p[i], centre, (std::max)(radius, halfH))) {
							NoteStraddler(a, i, (std::max)(radius, halfH));
						}
						const float vn = Dot(vel, a_p[i].n);
						const float c0 = Dot(centre - a_p[i].c, a_p[i].n);
						const float c1 = c0 + vn * look;
						auto&       pc = g_prevC[id];
						const bool  havePrev = pc.tick[i] == g_watchTick - 1;
						const float prevC0 = pc.c0[i];
						const NiPoint3 prevPos = pc.pos[i];
						pc.c0[i] = c0;
						pc.pos[i] = centre;
						pc.tick[i] = g_watchTick;
						if (!tr.seen || vn >= -10.0f || Len(vel) < 30.0f || g_actorCooldown.contains(id)) {
							continue;
						}
						auto fits = [&](const NiPoint3& a_at) {
							const NiPoint3 dd = a_at - a_p[i].c;
							// the centre must be where the whole body fits; through a floor or ceiling portal the body's footprint is its radius both ways (its height lies along the normal)
							const float    ext = std::abs(a_p[i].n.z) > 0.7f ? radius : halfH * 0.5f;
							const float    lx = Dot(dd, a_p[i].r) / (std::max)(a_p[i].hw - radius, 8.0f), ly = Dot(dd, a_p[i].u) / (std::max)(a_p[i].hh - ext, 8.0f);
							return lx * lx + ly * ly <= 1.0f;
						};
						bool carry = false;
						if (c0 > 0.0f && c1 <= 0.0f) {
							carry = fits(centre + vel * look * std::clamp(c0 / (c0 - c1), 0.0f, 1.0f));
						} else if (havePrev && prevC0 > 0.0f && c0 <= 0.0f) {
							carry = fits(prevPos + (centre - prevPos) * (prevC0 / (prevC0 - c0)));
						}
						if (carry) {
							CarryActor(a, a_p[i], a_p[1 - i], centre, vel, halfH);
							g_prevC.erase(id);
							if (g_driveColour.load() >= 0 && a == g_testNpc.get().get()) {
								g_driveColour = -1;
								g_npcTraceFrames = 240;
								const NiPoint3 d2 = MapDir(a_p[i], a_p[1 - i], vel);
								g_afterDir = Norm(NiPoint3{ d2.x, d2.y, 0.0f });
								g_afterFrames = 240;
							}
							shot::TriggerRing();
							break;
						}
					}
				}
				if (!capBody && tr.seen && !g_actorCooldown.contains(id)) {
					const NiPoint3 vel = (centre - tr.centre) * (1.0f / dt);  // from the position change: the controller's own velocity is zeroed by the wall it is pressed against
					const NiPoint3 bmin = a->GetBoundMin(), bmax = a->GetBoundMax();
					const float    radius = std::clamp(0.5f * (std::max)(bmax.x - bmin.x, bmax.y - bmin.y), 6.0f, 40.0f);  // the capsule's own radius: 22 for a person, about 10 for a chicken
					for (int i = 0; i < 2 && Len(vel) > 30.0f; ++i) {
						const float vn = Dot(vel, a_p[i].n);
						const float d0 = Dot(centre - a_p[i].c, a_p[i].n);
						if (vn >= -10.0f || d0 <= 0.0f) {
							continue;
						}
						const float    dNext = d0 + vn * look;
						if (dNext > radius) {
							continue;  // not about to touch the plane yet
						}
						const float    tHit = (std::max)(0.0f, (d0 - radius * 0.5f) / (-vn));
						const NiPoint3 hit = centre + vel * (std::min)(tHit, look);
						const NiPoint3 dd = hit - a_p[i].c;
						const float    lx = Dot(dd, a_p[i].r) / (std::max)(a_p[i].hw - radius, 8.0f), ly = Dot(dd, a_p[i].u) / (std::max)(a_p[i].hh - halfH * 0.5f, 8.0f);  // the centre must be where the whole body fits
						if (lx * lx + ly * ly > 1.0f) {
							continue;
						}
						// the centre goes to the exit plane's front, a radius and a little more away from the wall
						const NiPoint3 onPlane = hit - a_p[i].n * Dot(hit - a_p[i].c, a_p[i].n);
						CarryActor(a, a_p[i], a_p[1 - i], onPlane + a_p[i].n * -(radius + 6.0f), vel, halfH);
						if (g_driveColour.load() >= 0 && a == g_testNpc.get().get()) {
							g_driveColour = -1;
							g_npcTraceFrames = 240;
						}
						break;
					}
				}
				// while a trapdoor lowers the actor, keep the centre this frame started with, so the next frame's velocity includes the step (otherwise the position after a carry, as before)
				tr.centre = tr.sink > 0.0f && !g_actorCooldown.contains(id) ? centre : a->GetPosition() + NiPoint3{ 0, 0, halfH };
				tr.seen = true;
			}
			for (auto it = g_actorPrev.begin(); it != g_actorPrev.end();) {
				it = present.contains(it->first) ? std::next(it) : g_actorPrev.erase(it);
			}
			hole::SetExtra(bodies);

			if (g_npcTraceFrames > 0) {
				--g_npcTraceFrames;
				if (auto npc = g_testNpc.get(); npc && (g_npcTraceFrames % 12) == 0) {
					const NiPoint3 q = npc->GetPosition();
					auto* r3 = npc->Get3D();
						logger::info("travel: npc after the carry {:08X} feet ({:.1f},{:.1f},{:.1f}) heading {:.1f} deg; 3D loaded {} root {:p} at ({:.1f},{:.1f},{:.1f}) flags {:#x} bound radius {:.0f} hidden {} disabled {} dead {} same cell as the player {}", npc->GetFormID(), q.x, q.y, q.z,
							npc->GetAngleZ() * 57.29578f, npc->Is3DLoaded(), static_cast<void*>(r3), r3 ? r3->world.translate.x : 0.0f, r3 ? r3->world.translate.y : 0.0f, r3 ? r3->world.translate.z : 0.0f, r3 ? r3->flags.underlying() : 0u,
							r3 ? r3->worldBound.radius : 0.0f, r3 ? r3->GetFlags().any(RE::NiAVObject::Flag::kHidden) : false, npc->IsDisabled(), npc->IsDead(), npc->GetParentCell() == RE::PlayerCharacter::GetSingleton()->GetParentCell());
				}
			}
			if (g_afterFrames > 0) {
				--g_afterFrames;
				if (auto npc = g_testNpc.get(); npc && npc->Is3DLoaded()) {
					const NiPoint3 q = npc->GetPosition();
					npc->SetPosition(NiPoint3{ q.x + g_afterDir.x * g_driveSpeed.load() * dt, q.y + g_afterDir.y * g_driveSpeed.load() * dt, q.z }, true);
				}
			}
			// test: walk the spawned NPC at the centre of a portal by moving it a step per frame (its own AI stands still and the physics would zero a velocity set from here)
			if (const int col = g_driveColour.load(); col >= 0) {
				if (auto npc = g_testNpc.get(); npc && npc->Is3DLoaded()) {
					const NiPoint3 q = npc->GetPosition();
					static int     traceTick = 0;
					if ((++traceTick % 6) == 0) {
						logger::info("travel: npc trace {:08X} feet ({:.1f},{:.1f},{:.1f}) heading {:.1f} deg; distance of its centre to the plane of portal {}: {:.1f}", npc->GetFormID(), q.x, q.y, q.z, npc->GetAngleZ() * 57.29578f, col & 1,
							Dot(q + NiPoint3{ 0, 0, ActorHalfHeight(npc.get()) } - a_p[col & 1].c, a_p[col & 1].n));
					}
					const NiPoint3 d = a_p[col & 1].c - q;
					const float    l = std::sqrt(d.x * d.x + d.y * d.y);
					if (l > 1.0f) {
						const float step = (std::min)(g_driveSpeed.load() * dt, l);
						npc->SetPosition(NiPoint3{ q.x + d.x / l * step, q.y + d.y / l * step, q.z }, true);
					}
				}
			}
		}

		// ---- ragdolls: a dead or knocked-out actor is several rigid bodies (one per bone) held together by constraints ----
		void CollectBodies(RE::NiAVObject* a_o, std::vector<RE::bhkRigidBody*>& a_out, int a_depth)
		{
			if (!a_o || a_depth > 40) {
				return;
			}
			if (auto* co = a_o->GetCollisionObject()) {
				if (auto* b = co->GetRigidBody(); b && IsDynamic(b->GetRigidBody()) && std::find(a_out.begin(), a_out.end(), b) == a_out.end()) {
					a_out.push_back(b);
				}
			}
			if (auto* n = a_o->AsNode()) {
				for (auto& c : n->GetChildren()) {
					CollectBodies(c.get(), a_out, a_depth + 1);
				}
			}
		}

		NiPoint3 BodyPoint(RE::bhkRigidBody* a_b, bool a_comWorld)
		{
			const float   inv = 1.0f / RE::bhkWorld::GetWorldScale();
			RE::hkVector4 hp;
			if (a_comWorld) {
				a_b->GetCenterOfMassWorld(hp);
			} else {
				a_b->GetPosition(hp);
			}
			alignas(16) float v[4];
			_mm_store_ps(v, hp.quad);
			return { v[0] * inv, v[1] * inv, v[2] * inv };
		}

		// One body through the map: its own origin is mapped (the whole ragdoll moves rigidly, so the constraints between its bodies stay satisfied), then its orientation,
		// linear and angular velocity.
		void MapBody(RE::bhkRigidBody* a_b, const portals::Portal& A, const portals::Portal& B, const float R[3][3], const NiPoint3& a_delta, const NiPoint3& a_exitShift)
		{
			const float    s = RE::bhkWorld::GetWorldScale();
			const float    inv = 1.0f / s;
			auto*          rb = a_b->GetRigidBody();
			alignas(16) float lv[4], av[4], qo[4];
			_mm_store_ps(lv, rb->motion.linearVelocity.quad);
			_mm_store_ps(av, rb->motion.angularVelocity.quad);
			RE::hkQuaternion qOld;
			a_b->GetRotation(qOld);
			_mm_store_ps(qo, qOld.vec.quad);
			const NiPoint3 v2 = MapDir(A, B, NiPoint3{ lv[0] * inv, lv[1] * inv, lv[2] * inv });
			const float    aw[3] = { R[0][0] * av[0] + R[0][1] * av[1] + R[0][2] * av[2], R[1][0] * av[0] + R[1][1] * av[1] + R[1][2] * av[2], R[2][0] * av[0] + R[2][1] * av[1] + R[2][2] * av[2] };
			float          qm[4], qn[4];
			MatToQuat(R, qm);
			QuatMul(qm, qo, qn);
			const float ql = std::sqrt(qn[0] * qn[0] + qn[1] * qn[1] + qn[2] * qn[2] + qn[3] * qn[3]);
			RE::hkQuaternion qNew;
			qNew.vec = RE::hkVector4(qn[0] / ql, qn[1] / ql, qn[2] / ql, qn[3] / ql);
			const NiPoint3 np = MapPoint(A, B, BodyPoint(a_b, false) + a_delta) + a_exitShift;
			RE::hkVector4  hp(np.x * s, np.y * s, np.z * s, 0.0f);
			a_b->SetPositionAndRotation(hp, qNew);
			a_b->SetLinearVelocity(RE::hkVector4(v2.x * s, v2.y * s, v2.z * s, 0.0f));
			a_b->SetAngularVelocity(RE::hkVector4(aw[0], aw[1], aw[2], 0.0f));
		}

		std::atomic<int> g_ragdolls{ 0 };
		int              g_ragTrace{ 0 };

		// ---- summoned creatures (conjure spells, delivery "target location") ----
		// A conjure spell has no projectile: the creature is created where the cast's aim ray ends. Cast into a portal's oval that is the entry wall, so the creature appears in front of the wall
		// next to the oval. When the camera's view ray goes through the oval of the portal the creature stands at, it is moved to where that ray lands after the exit, in front of the surface it lands
		// on, onto the floor there, facing away from the surface; once (the first tick it is seen with its 3D loaded).
		std::set<std::uint32_t> g_summonSeen;
		std::atomic<int>        g_summonsMoved{ 0 };

		void TickSummons(const portals::Portal (&a_p)[2])
		{
			auto*          player = RE::PlayerCharacter::GetSingleton();
			auto*          cell = player ? player->GetParentCell() : nullptr;
			world::CamInfo cam;
			if (!cell || !world::CameraFull(cam)) {
				return;
			}
			std::set<std::uint32_t> now;
			for (int i = 0; i < 2; ++i) {
				const portals::Portal& A = a_p[i];
				const portals::Portal& B = a_p[1 - i];
				if (Len(A.c - player->GetPosition()) > 2500.0f) {
					continue;
				}
				cell->ForEachReferenceInRange(A.c, 380.0f, [&](RE::TESObjectREFR* a_ref) {
					auto* a = a_ref ? a_ref->As<RE::Actor>() : nullptr;
					if (!a || a == player || a->IsDead() || a->IsDisabled() || !a->IsSummonedByPlayer()) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
					const std::uint32_t id = a->GetFormID();
					now.insert(id);
					if (g_summonSeen.contains(id) || !a->Is3DLoaded()) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
					g_summonSeen.insert(id);
					const NiPoint3 P = a->GetPosition();
					// the camera's view ray through the oval of either portal (G342: a summon cast into violet was first found near teal, judged against teal only and left at the entry)
					int      e = i;
					bool     aimed = false;
					NiPoint3 H{};
					for (int k = 0; k < 2 && !aimed; ++k) {
						const portals::Portal& K = a_p[k];
						const float            dn = Dot(cam.fwd, K.n);
						if (dn >= -0.05f) {
							continue;
						}
						const float    t = Dot(K.c - cam.pos, K.n) / dn;
						const NiPoint3 Hk = cam.pos + cam.fwd * t;
						const NiPoint3 dd = Hk - K.c;
						const float    lx = Dot(dd, K.r) / (K.hw * 1.15f), ly = Dot(dd, K.u) / (K.hh * 1.15f);
						if (t > 0.0f && lx * lx + ly * ly <= 1.0f) {
							aimed = true;
							e = k;
							H = Hk;
						}
					}
					const portals::Portal& A = a_p[e];
					const portals::Portal& B = a_p[1 - e];
					logger::info("travel: summon {:08X} '{}' first seen at ({:.1f},{:.1f},{:.1f}), {:.0f} units from portal {}'s plane crossing of the view ray; the view ray {} the oval", id, a->GetName() ? a->GetName() : "", P.x, P.y, P.z, aimed ? Len(P - H) : -1.0f, A.colour,
						aimed ? "goes through" : "misses");
					if (!aimed || Len(P - H) > 260.0f) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
					const NiPoint3 H2 = MapPoint(A, B, H);
					const NiPoint3 D2 = Norm(MapDir(A, B, cam.fwd));
					const auto     hit = world::Raycast(H2 + B.n * 2.0f, D2, 3000.0f, RE::COL_LAYER::kLOS);
					NiPoint3       feet = hit.hit ? H2 + D2 * std::clamp(hit.dist - 45.0f, 30.0f, 400.0f) : H2 + B.n * 60.0f;  // on the ray, in the middle of the exit's window, not along the landing wall's normal (G107: it stood at the oval's edge)
					const NiPoint3 facing = hit.hit ? hit.normal : B.n;
					if (const auto ray = world::Raycast(feet + NiPoint3{ 0, 0, 40.0f }, NiPoint3{ 0, 0, -1 }, 400.0f, RE::COL_LAYER::kLOS); ray.hit) {
						feet.z = feet.z + 40.0f - ray.dist + 1.0f;
					}
					const float yaw = std::atan2(facing.x, facing.y);
					a->SetPosition(feet, true);
					a->SetHeading(yaw);
					// the conjure's vortex (a summon placement effect at the creature's first spot, on the entry wall) goes with it (PLAN: "the conjure's own cast art stays at the entry wall")
					if (auto* pl = RE::ProcessLists::GetSingleton()) {
						int moved = 0;
						pl->ForEachMagicTempEffect([&](RE::BSTempEffect* a_e) {
							if (auto* sp = a_e ? skyrim_cast<RE::SummonPlacementEffect*>(a_e) : nullptr; sp && Len(sp->location - P) < 250.0f) {
								const NiPoint3 delta = feet - sp->location;
								sp->location = feet;
								if (auto* o = sp->artObject3D.get()) {
									o->local.translate += delta;
									o->world.translate += delta;
									RE::NiUpdateData ud{};
									o->Update(ud);
								}
								++moved;
							}
							return RE::BSContainer::ForEachResult::kContinue;
						});
						logger::info("travel: summon {:08X}: {} summon placement effect(s) moved with it", id, moved);
					}
					if (auto* cc = a->GetCharController()) {
						const RE::hkVector4 zero(0.0f, 0.0f, 0.0f, 0.0f);
						cc->SetLinearVelocityImpl(zero);
						cc->outVelocity = zero;
						cc->initialVelocity = zero;
						cc->fallStartHeight = feet.z;
						cc->fallTime = 0.0f;
					}
					g_actorCooldown[id] = 12;
					g_summonsMoved.fetch_add(1);
					logger::info("travel: summon {:08X} moved through portal {} -> {}: view ray enters at ({:.1f},{:.1f},{:.1f}) -> from the exit ({:.1f},{:.1f},{:.1f}) direction ({:.3f},{:.3f},{:.3f}): {}; feet at ({:.1f},{:.1f},{:.1f}), facing {:.1f} deg", id, A.colour, B.colour, H.x, H.y, H.z, H2.x, H2.y, H2.z, D2.x, D2.y, D2.z,
						hit.hit ? std::format("lands at ({:.1f},{:.1f},{:.1f}) after {:.0f}, normal ({:.2f},{:.2f},{:.2f})", hit.pos.x, hit.pos.y, hit.pos.z, hit.dist, hit.normal.x, hit.normal.y, hit.normal.z) : "nothing within 3000", feet.x, feet.y, feet.z, yaw * 57.29578f);
					return RE::BSContainer::ForEachResult::kContinue;
				});
			}
			for (auto it = g_summonSeen.begin(); it != g_summonSeen.end();) {
				it = now.contains(*it) ? std::next(it) : g_summonSeen.erase(it);
			}
		}

		void TickRagdolls(const portals::Portal (&a_p)[2])
		{
			auto* player = RE::PlayerCharacter::GetSingleton();
			auto* cell = player ? player->GetParentCell() : nullptr;
			if (!cell) {
				return;
			}
			const float inv = 1.0f / RE::bhkWorld::GetWorldScale();
			const float look = static_cast<float>(g_dt) * g_lookFrames.load();
			for (int i = 0; i < 2; ++i) {
				if (Len(a_p[i].c - player->GetPosition()) > 2500.0f) {
					continue;
				}
				cell->ForEachReferenceInRange(a_p[i].c, 600.0f, [&](RE::TESObjectREFR* a_ref) {
					auto* a = a_ref ? a_ref->As<RE::Actor>() : nullptr;
					// dead bodies, and living actors knocked down into a ragdoll (G386: a bandit thrown by Fus Ro Dah flew as a ragdoll, which only the dead were handled as,
					// and his bodies hit the wall around the oval)
					if (!a || a == player || !(a->IsDead() || a->IsInRagdollState()) || !a->Is3DLoaded() || (!g_passage.load() && g_actorCooldown.contains(a->GetFormID()))) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
					auto* root = a->Get3D();
					std::vector<RE::bhkRigidBody*> bodies;
					CollectBodies(root, bodies, 0);
					if (bodies.size() < 2) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
					NiPoint3 centroid{ 0, 0, 0 }, vel{ 0, 0, 0 };
					for (auto* b : bodies) {
						centroid = centroid + BodyPoint(b, true);
						alignas(16) float lv[4];
						_mm_store_ps(lv, b->GetRigidBody()->motion.linearVelocity.quad);
						vel = vel + NiPoint3{ lv[0] * inv, lv[1] * inv, lv[2] * inv };
					}
					const float n = static_cast<float>(bodies.size());
					centroid = centroid * (1.0f / n);
					vel = vel * (1.0f / n);
					if (g_ragTrace > 0 && (g_ragTrace % 6) == 0) {
						logger::info("travel: ragdoll {:08X} '{}': {} bodies, centroid ({:.1f},{:.1f},{:.1f}) velocity ({:.0f},{:.0f},{:.0f}); distance to the plane of portal {}: {:.1f}", a->GetFormID(), a->GetName() ? a->GetName() : "", bodies.size(), centroid.x,
							centroid.y, centroid.z, vel.x, vel.y, vel.z, i, Dot(centroid - a_p[i].c, a_p[i].n));
					}
					if (g_ragTrace > 0) {
						--g_ragTrace;
					}
										if (g_passage.load()) {
						// gradual passage for the whole ragdoll: every body is watched (let through the wall inside the oval), the 3D is drawn cut while the set straddles the plane, and the set is
						// carried rigidly when its centroid crosses it
						float prad = 30.0f;
						for (auto* b : bodies) {
							prad = (std::max)(prad, Len(BodyPoint(b, true) - centroid) + 15.0f);
						}
						prad = (std::min)(prad, 110.0f);
						std::vector<RE::hkpRigidBody*> hk;
						for (auto* b : bodies) {
							hk.push_back(b->GetRigidBody());
						}
						WatchBodies(a, hk, centroid, prad);
						if (Straddles(a_p[i], centroid, prad)) {
							NoteStraddler(a, i, prad);
						}
						const float    vn = Dot(vel, a_p[i].n);
						const float    c0 = Dot(centroid - a_p[i].c, a_p[i].n);
						const float    c1 = c0 + vn * look;
						auto&          pc = g_prevC[a->GetFormID()];
						const bool     havePrev = pc.tick[i] == g_watchTick - 1;
						const float    prevC0 = pc.c0[i];
						const NiPoint3 prevPos = pc.pos[i];
						pc.c0[i] = c0;
						pc.pos[i] = centroid;
						pc.tick[i] = g_watchTick;
						const bool free = vn < -20.0f && !g_actorCooldown.contains(a->GetFormID());
						auto       inOval = [&](const NiPoint3& a_at) {
							const NiPoint3 dd = a_at - a_p[i].c;
							const float    lx = Dot(dd, a_p[i].r) / (a_p[i].hw * 0.95f), ly = Dot(dd, a_p[i].u) / (a_p[i].hh * 0.95f);
							return lx * lx + ly * ly <= 1.0f;
						};
						bool carry = false;
						if (free && c0 > 0.0f && c1 <= 0.0f) {
							carry = inOval(centroid + vel * look * std::clamp(c0 / (c0 - c1), 0.0f, 1.0f));
						} else if (free && havePrev && prevC0 > 0.0f && c0 <= 0.0f) {
							carry = inOval(prevPos + (centroid - prevPos) * (prevC0 / (prevC0 - c0)));
						}
						if (carry) {
							float R[3][3];
							MapMatrix(a_p[i], a_p[1 - i], R);
							// the bodies that had not crossed yet would land behind the exit surface (G384: a body dropped into a floor portal came out of the ceiling portal with
							// its limbs inside the ceiling and hung there); the whole set is moved out along the exit's normal until every body is in front of it
							const portals::Portal& Bx = a_p[1 - i];
							float                  minD = 1e9f;
							for (auto* b : bodies) {
								minD = (std::min)(minD, Dot(MapPoint(a_p[i], Bx, BodyPoint(b, false)) - Bx.c, Bx.n));
							}
							NiPoint3 shift = minD < 12.0f ? Bx.n * (12.0f - minD) : NiPoint3{ 0, 0, 0 };
							// endless tumble (a floor portal under a ceiling portal): falling straight through such a pair the set is eased toward the exit portal's centre, as the
							// player is (G386: the body looped four times, drifting a little each time, and landed beside the oval)
							if (std::abs(a_p[i].n.z) > 0.7f && std::abs(Bx.n.z) > 0.7f && Len(vel) > 100.0f && std::abs(Dot(Norm(vel), a_p[i].n)) > 0.8f) {
								NiPoint3    off = MapPoint(a_p[i], Bx, centroid) - Bx.c;
								off = off - Bx.n * Dot(off, Bx.n);
								const float len = Len(off);
								if (len > 1.0f) {
									shift = shift - off * ((std::min)(len * 0.5f, 30.0f) / len);
								}
							}
							for (auto* b : bodies) {
								MapBody(b, a_p[i], Bx, R, NiPoint3{ 0, 0, 0 }, shift);
							}
							a->SetPosition(MapPoint(a_p[i], Bx, a->GetPosition()) + shift, false);
							if (minD < 12.0f) {
								logger::info("travel: ragdoll {:08X}: moved {:.0f} units out of the exit surface (its deepest body would have been {:.0f} behind it)", a->GetFormID(), 12.0f - minD, -minD);
							}
							g_actorCooldown[a->GetFormID()] = 6;
							g_prevC.erase(a->GetFormID());
							g_ragdolls.fetch_add(1);
							shot::TriggerRing();
							const NiPoint3 v2 = MapDir(a_p[i], a_p[1 - i], vel);
							logger::info("travel: ragdoll {:08X} '{}' ({} bodies, radius {:.0f}) through portal {} -> {} (gradual): centroid ({:.1f},{:.1f},{:.1f}) velocity ({:.0f},{:.0f},{:.0f}) -> ({:.0f},{:.0f},{:.0f})", a->GetFormID(),
								a->GetName() ? a->GetName() : "", bodies.size(), prad, a_p[i].colour, a_p[1 - i].colour, centroid.x, centroid.y, centroid.z, vel.x, vel.y, vel.z, v2.x, v2.y, v2.z);
						}
						return RE::BSContainer::ForEachResult::kContinue;
					}
					if (g_actorCooldown.contains(a->GetFormID())) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
					if (Len(vel) < 60.0f) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
					float radius = 30.0f;
					for (auto* b : bodies) {
						radius = (std::max)(radius, Len(BodyPoint(b, true) - centroid) + 15.0f);
					}
					radius = (std::min)(radius, 110.0f);
					const float d0 = Dot(centroid - a_p[i].c, a_p[i].n);
					const float d1 = d0 + Dot(vel, a_p[i].n) * look;
					if (d0 <= 0.0f || d1 > radius * 0.5f || Dot(vel, a_p[i].n) >= -30.0f) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
					const float    t = (d0 - radius * 0.5f) / (d0 - d1 > 1e-3f ? d0 - d1 : 1.0f);
					const NiPoint3 hit = centroid + vel * look * std::clamp(t, 0.0f, 1.0f);
					const NiPoint3 dd = hit - a_p[i].c;
					const float    lx = Dot(dd, a_p[i].r) / a_p[i].hw, ly = Dot(dd, a_p[i].u) / a_p[i].hh;
					if (lx * lx + ly * ly > 1.0f) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
					float R[3][3];
					MapMatrix(a_p[i], a_p[1 - i], R);
					const NiPoint3 delta = hit - centroid;
					const NiPoint3 shift = a_p[1 - i].n * (radius + 3.0f);
					for (auto* b : bodies) {
						MapBody(b, a_p[i], a_p[1 - i], R, delta, shift);
					}
					a->SetPosition(MapPoint(a_p[i], a_p[1 - i], a->GetPosition() + delta) + shift, false);
					g_actorCooldown[a->GetFormID()] = 20;
					g_ragdolls.fetch_add(1);
					logger::info("travel: ragdoll {:08X} '{}' ({} bodies, radius {:.0f}) through portal {} -> {}: centroid ({:.1f},{:.1f},{:.1f}) velocity ({:.0f},{:.0f},{:.0f}) -> ({:.0f},{:.0f},{:.0f})", a->GetFormID(), a->GetName() ? a->GetName() : "", bodies.size(), radius,
						a_p[i].colour, a_p[1 - i].colour, centroid.x, centroid.y, centroid.z, vel.x, vel.y, vel.z, MapDir(a_p[i], a_p[1 - i], vel).x, MapDir(a_p[i], a_p[1 - i], vel).y, MapDir(a_p[i], a_p[1 - i], vel).z);
					return RE::BSContainer::ForEachResult::kContinue;
				});
			}
		}

		// ---- beams, flame streams and cones ----
		// Such a spell is ONE projectile that stays at the caster's hand and points along the aim (G74): a beam's range ends at the first thing in the way, so at a portal's wall it ends in the
		// wall; a stream's does not end at all. The part that comes out of the other portal is a second projectile of the same spell, with the same shooter and casting source, launched at
		// the exit with the mapped aim and range and kept there every frame while the source reaches the oval. It is the engine's own projectile, so it finds its hits and applies the
		// spell's effects like the original does.
		struct Twin
		{
			RE::ProjectileHandle h;
			int                  born{ 0 };
			NiPoint3             placed;       // where the last tick put it: the next tick compares (does the engine move a stream back to the caster's hand?)
			bool                 hasPlaced{ false };
			int                  driftLogged{ 0 };
		};
		std::map<std::uint32_t, Twin> g_twins;  // by the source projectile's id
		int                           g_tick{ 0 };
		int                           g_twinLogged{ 0 };
		std::atomic<int>              g_twinsMade{ 0 };
		std::atomic<int>              g_twinMode{ 0 };  // test: 0 = the other hand's casting source, 1 = the launch data's new-projectile flag, 2 = both
		std::map<std::uint32_t, int>  g_twinTry;        // source id -> tick of the last launch attempt

		// G359: a cone (Ice Storm) is not held at the hand like a flame or a beam, it drifts out at about 330 units/s; its twin pinned at the exit never moved toward
		// the target (no damage either way) and the source flew on through the wall (it hurt a townsman far behind the house). So a cone is carried like a
		// missile when it reaches the plane (`travel conecarry off`: the old twin)
		std::atomic<bool> g_coneCarry{ true };
		bool IsCone(RE::Projectile* a_p)
		{
			auto* b = a_p->GetProjectileBase();
			return b && b->IsCone();
		}
		bool IsStream(RE::Projectile* a_p)
		{
			auto* b = a_p->GetProjectileBase();
			return b && (b->IsBeam() || b->IsFlamethrower() || (b->IsCone() && !g_coneCarry.load()));
		}

		void KillProjectile(RE::Projectile* a_p)
		{
			a_p->GetProjectileRuntimeData().flags.set(RE::Projectile::Flags::kDestroyed);
			a_p->Disable();
		}

		void PlaceStream(RE::Projectile* a_p, const NiPoint3& a_pos, const NiPoint3& a_dir, float a_speed, float a_range)
		{
			auto& rd = a_p->GetProjectileRuntimeData();
			rd.linearVelocity = a_dir * a_speed;
			rd.range = a_range;
			a_p->SetPosition(a_pos);
			a_p->data.angle.x = -std::asin(std::clamp(a_dir.z, -1.0f, 1.0f));  // pitch, positive looking down
			a_p->data.angle.z = std::atan2(a_dir.x, a_dir.y);                    // heading
			if (rd.unk0E0 && rd.unk0E0->phantom) {
				auto*               ph = rd.unk0E0->phantom.get();
				const float         sc = RE::bhkWorld::GetWorldScale();
				const RE::hkVector4 hp(a_pos.x * sc, a_pos.y * sc, a_pos.z * sc, 0.0f);
				ph->motionState.transform.translation = hp;
				ph->motionState.sweptTransform.centerOfMass0 = hp;
				ph->motionState.sweptTransform.centerOfMass1 = hp;
			}
		}

		// G101: the engine's own update of a flame, beam or cone projectile (UpdateImpl, vtable slot 0xAB: ids 43894, 43749, 43789) puts it at the casting source's hand node with the player's
		// aim EVERY frame (SetPosition of the node's world position, the angles from the actor's aim), so a twin placed at the start of the frame was back at the left hand (32 units from the
		// source) after the engine's update: "was moved by the engine since the last tick" on every tick, its hits are found from there too. So the twin is put at the exit again right after
		// the engine's update, before anything that runs later in the projectile's frame (target pick, impacts, drawing).
		struct TwinPose
		{
			NiPoint3      pos, dir;
			float         speed{ 0 }, range{ 0 };
			std::uint32_t id{ 0 };
		};
		std::map<RE::Projectile*, TwinPose> g_twinPose;  // by the twin's address, filled by TickStreams, used by the engine's update hooks
		std::mutex                          g_poseMutex;  // the engine may update projectiles on another thread than the frame hook
		std::map<RE::Projectile*, float>    g_twinPickAt;  // the twin's living time at its last target pick (under g_poseMutex)
		std::atomic<int>                    g_picksRun{ 0 };
		std::atomic<bool>                   g_twinPickOn{ false };  // G113/G114: a flame or beam twin runs its own target pick at the exit: found only the wall behind the portal (off by default)
		std::atomic<float>                  g_twinStartOut{ 4.0f };  // how far in front of the exit plane a flame or beam twin starts (28 was tried for the pick)
		using UpdateImpl_t = void (*)(RE::Projectile*, float);
		UpdateImpl_t     g_origFlameUpdate{ nullptr }, g_origBeamUpdate{ nullptr }, g_origConeUpdate{ nullptr };
		std::atomic<int> g_poseApplied{ 0 };

		// the node's rotation columns right, forward (+Y), up for a direction (as in CarryProjectile) and the node's position
		void SetNodeAim(RE::Projectile* a_p, const NiPoint3& a_pos, const NiPoint3& a_dir)
		{
			auto* r3 = a_p->Get3D();
			if (!r3) {
				return;
			}
			const NiPoint3 right = std::fabs(a_dir.z) > 0.999f ? NiPoint3{ 1, 0, 0 } : Norm(NiPoint3{ a_dir.y, -a_dir.x, 0.0f });
			const NiPoint3 up{ right.y * a_dir.z - right.z * a_dir.y, right.z * a_dir.x - right.x * a_dir.z, right.x * a_dir.y - right.y * a_dir.x };
			RE::NiMatrix3  m;
			m.entry[0][0] = right.x; m.entry[0][1] = a_dir.x; m.entry[0][2] = up.x;
			m.entry[1][0] = right.y; m.entry[1][1] = a_dir.y; m.entry[1][2] = up.y;
			m.entry[2][0] = right.z; m.entry[2][1] = a_dir.z; m.entry[2][2] = up.z;
			r3->local.rotate = m;
			r3->world.rotate = m;
			r3->local.translate = a_pos;
			r3->world.translate = a_pos;
		}

		void PlaceTwinAfterUpdate(RE::Projectile* a_p)
		{
			TwinPose t;
			{
				std::lock_guard lk(g_poseMutex);
				const auto      it = g_twinPose.find(a_p);
				if (it == g_twinPose.end() || it->second.id != a_p->GetFormID()) {
					return;
				}
				t = it->second;
			}
			PlaceStream(a_p, t.pos, t.dir, t.speed, t.range);
			SetNodeAim(a_p, t.pos, t.dir);
			// G112: a flame or beam twin never hit anything at the exit (every wall hit of the run came from the source at the entry wall, and the twins' only hits were on their first frame, from the
			// caster's hand): the engine picks a flame's targets (a Havok query from the projectile's position along its angles, then AddImpact for what it finds) only when the projectile is created,
			// so the twin picks again from the exit here, about as often as the source's own hits come (every 0.22 s for a flame, every frame for a beam)
			if (g_twinPickOn.load() && (a_p->IsFlameProjectile() || a_p->IsBeamProjectile())) {
				const float lt = a_p->GetProjectileRuntimeData().livingTime;
				const float every = a_p->IsFlameProjectile() ? 0.2f : 0.0f;
				bool        run = false;
				{
					std::lock_guard lk(g_poseMutex);
					auto&           at = g_twinPickAt[a_p];
					if (lt - at >= every || lt < at) {
						at = lt;
						run = true;
					}
				}
				if (run) {
					a_p->RunTargetPick();
					if (g_picksRun.fetch_add(1) < 6) {
						logger::info("travel: twin {:08X} picked its targets from the exit ({:.1f},{:.1f},{:.1f}) direction ({:.3f},{:.3f},{:.3f}), living {:.3f}; its angles (pitch {:.1f}, heading {:.1f}) deg, position now ({:.1f},{:.1f},{:.1f}), velocity ({:.0f},{:.0f},{:.0f})", a_p->GetFormID(), t.pos.x, t.pos.y, t.pos.z, t.dir.x, t.dir.y, t.dir.z, lt,
						a_p->data.angle.x * 57.29578f, a_p->data.angle.z * 57.29578f, a_p->GetPosition().x, a_p->GetPosition().y, a_p->GetPosition().z, a_p->GetProjectileRuntimeData().linearVelocity.x, a_p->GetProjectileRuntimeData().linearVelocity.y, a_p->GetProjectileRuntimeData().linearVelocity.z);
					}
				}
			}
			if (g_poseApplied.fetch_add(1) < 3) {
				logger::info("travel: twin {:08X} put at the exit right after the engine's own update: ({:.1f},{:.1f},{:.1f}) direction ({:.3f},{:.3f},{:.3f}), now at ({:.1f},{:.1f},{:.1f})", a_p->GetFormID(), t.pos.x, t.pos.y, t.pos.z, t.dir.x, t.dir.y, t.dir.z,
					a_p->GetPosition().x, a_p->GetPosition().y, a_p->GetPosition().z);
			}
		}

		void Hook_FlameUpdate(RE::Projectile* a_p, float a_dt)
		{
			g_origFlameUpdate(a_p, a_dt);
			PlaceTwinAfterUpdate(a_p);
		}
		void Hook_BeamUpdate(RE::Projectile* a_p, float a_dt)
		{
			g_origBeamUpdate(a_p, a_dt);
			PlaceTwinAfterUpdate(a_p);
		}
		// a cone's hits (Ice Storm) are found by the engine around the cone that grows from its origin (ConeProjectile 0x1E8) to the projectile; the twin is put at the
		// exit after each update, its origin was left where the engine keeps it (G340-G347: Ice Storm into violet did no damage at the exit, into teal it hit). So
		// the twin's origin goes to the exit too (`travel coneorigin off` keeps the engine's)
		std::atomic<bool> g_coneOrigin{ true };
		std::atomic<int>  g_coneLogged{ 0 };
		void Hook_ConeUpdate(RE::Projectile* a_p, float a_dt)
		{
			g_origConeUpdate(a_p, a_dt);
			auto&          cd = static_cast<RE::ConeProjectile*>(a_p)->GetConeRuntimeData();
			const NiPoint3 before = cd.origin;
			const NiPoint3 posBefore = a_p->GetPosition();
			PlaceTwinAfterUpdate(a_p);
			bool     twin = false;
			NiPoint3 tp{};
			{
				std::lock_guard lk(g_poseMutex);
				if (const auto it = g_twinPose.find(a_p); it != g_twinPose.end() && it->second.id == a_p->GetFormID()) {
					twin = true;
					tp = it->second.pos;
				}
			}
			if (twin && g_coneOrigin.load()) {
				cd.origin = tp;
			}
			if (g_verbose.load() && g_coneLogged.fetch_add(1) < 80) {
				const auto& rd = a_p->GetProjectileRuntimeData();
				logger::info("travel: cone {:08X} ({}) after the engine's update at ({:.1f},{:.1f},{:.1f}) origin ({:.1f},{:.1f},{:.1f}); now at ({:.1f},{:.1f},{:.1f}) origin ({:.1f},{:.1f},{:.1f}); living {:.3f}, cone tangent {:.3f}, start radius {:.1f}, collisions {}", a_p->GetFormID(), twin ? "twin" : "source",
					posBefore.x, posBefore.y, posBefore.z, before.x, before.y, before.z, a_p->GetPosition().x, a_p->GetPosition().y, a_p->GetPosition().z, cd.origin.x, cd.origin.y, cd.origin.z, rd.livingTime, cd.coneAngleTangent, cd.initialCollisionSphereRadius, cd.collisions.size());
			}
		}

		// patch UpdateImpl (slot 0xAB) in the three stream classes' vtables, after checking that the slot holds the function the offline analysis found there
		bool PatchUpdate(void** a_vt, std::uint32_t a_expectId, void* a_hook, UpdateImpl_t* a_orig, const char* a_name)
		{
			constexpr std::size_t kSlot = 0xAB;
			if (a_vt[kSlot] != reinterpret_cast<void*>(REL::ID(a_expectId).address())) {
				logger::warn("travel: {} vtable slot 0xAB does not hold id {} (rva {:#x}): not patched (twin streams stay at the caster's hand)", a_name, a_expectId, reinterpret_cast<std::uintptr_t>(a_vt[kSlot]) - REL::Module::get().base());
				return false;
			}
			DWORD old = 0;
			if (!VirtualProtect(&a_vt[kSlot], sizeof(void*), PAGE_READWRITE, &old)) {
				logger::error("travel: VirtualProtect failed for the {} vtable", a_name);
				return false;
			}
			*a_orig = reinterpret_cast<UpdateImpl_t>(a_vt[kSlot]);
			a_vt[kSlot] = a_hook;
			VirtualProtect(&a_vt[kSlot], sizeof(void*), old, &old);
			logger::info("travel: {} UpdateImpl (id {}) patched: a twin stream is put at the exit again right after the engine's update", a_name, a_expectId);
			return true;
		}

		void InstallStreamHooks()
		{
			static bool done = false;
			if (done) {
				return;
			}
			done = true;
			PatchUpdate(reinterpret_cast<void**>(RE::VTABLE_FlameProjectile[0].address()), 43894, reinterpret_cast<void*>(&Hook_FlameUpdate), &g_origFlameUpdate, "FlameProjectile");
			PatchUpdate(reinterpret_cast<void**>(RE::VTABLE_BeamProjectile[0].address()), 43749, reinterpret_cast<void*>(&Hook_BeamUpdate), &g_origBeamUpdate, "BeamProjectile");
			PatchUpdate(reinterpret_cast<void**>(RE::VTABLE_ConeProjectile[0].address()), 43789, reinterpret_cast<void*>(&Hook_ConeUpdate), &g_origConeUpdate, "ConeProjectile");
		}

		// ---- the twin's own hit test (G336) ----
		// A flame or beam twin is drawn from the exit, but the engine applies none of its spell to what it touches there (G112-G114, G333: Flames, Frostbite, Sparks and Lightning Bolt do
		// 0 damage at the exit, 25 straight at the same target; G335: launching it from the same hand changes nothing). Chain Lightning and the cones (Ice Storm) do hurt at the exit, the first
		// through its explosion, the second through the cone's own query, so they are left to the engine. For the others the twin's line is tested here every tick: an actor whose centre lies
		// within the stream's width of the line, with no wall between the exit and it, gets the spell's value-modifier effects (health, and for dual effects the second value), scaled by its
		// elemental and magic resistance; per second for a concentration spell, once per target and twin for a fire-and-forget beam.
		std::atomic<bool>                                  g_twinHits{ true };
		// the engine's own impacts of a projectile on an actor (Hole.cpp's AddImpact hook): where the engine does hit with the twin (G339: Sparks and Lightning Bolt into violet), the own
		// hit test leaves that actor alone, so it is never hurt twice
		std::mutex                                                   g_impMutex;
		std::map<std::pair<std::uint32_t, std::uint32_t>, double>    g_engineImpact;  // (projectile, actor) -> seconds (steady clock)
		double NowSec()
		{
			return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
		}
		bool EngineHit(std::uint32_t a_proj, std::uint32_t a_actor, double a_within)
		{
			std::lock_guard lk(g_impMutex);
			const auto      it = g_engineImpact.find({ a_proj, a_actor });
			return it != g_engineImpact.end() && NowSec() - it->second <= a_within;
		}
		std::map<std::uint32_t, std::set<std::uint32_t>>   g_twinHitOnce;  // source id -> actors a fire-and-forget beam twin already hit
		std::atomic<int>                                   g_twinHitLogged{ 0 };

		float ResistFactor(RE::Actor* a_t, const RE::EffectSetting* a_base)
		{
			auto*       avo = a_t->AsActorValueOwner();
			const float cap = 85.0f;
			float       f = 1.0f;
			if (a_base->data.resistVariable != RE::ActorValue::kNone) {
				f *= 1.0f - std::clamp(avo->GetActorValue(a_base->data.resistVariable), -100.0f, cap) / 100.0f;
			}
			if (a_base->data.resistVariable != RE::ActorValue::kResistMagic) {
				f *= 1.0f - std::clamp(avo->GetActorValue(RE::ActorValue::kResistMagic), -100.0f, cap) / 100.0f;
			}
			return f;
		}

		// G408: with the shooter back for the hit the Firebolt still did nothing to the player who cast it (the engine never lets your own hostile spell hurt you). A spell of
		// yours that comes back through a portal and hits you gets its instant damage to health, magicka and stamina applied here, scaled by your resistances, on the main thread.
		void ApplySelfSpell(RE::Projectile* a_p, RE::Actor* a_self)
		{
			auto* spell = a_p->GetProjectileRuntimeData().spell;
			if (!spell || !a_self) {
				return;
			}
			std::vector<std::pair<RE::EffectSetting*, float>> hits;
			for (auto* e : spell->effects) {
				auto* mgef = e ? e->baseEffect : nullptr;
				if (!mgef || !mgef->IsDetrimental() || e->GetDuration() > 0) {
					continue;
				}
				const auto arch = mgef->GetArchetype();
				if (arch != RE::EffectSetting::Archetype::kValueModifier && arch != RE::EffectSetting::Archetype::kDualValueModifier && arch != RE::EffectSetting::Archetype::kPeakValueModifier) {
					continue;
				}
				const auto av = mgef->data.primaryAV;
				if (av == RE::ActorValue::kHealth || av == RE::ActorValue::kMagicka || av == RE::ActorValue::kStamina) {
					hits.emplace_back(mgef, e->GetMagnitude());
				}
			}
			if (hits.empty()) {
				return;
			}
			const auto id = a_p->GetFormID();
			SKSE::GetTaskInterface()->AddTask([h = a_self->GetHandle(), hits, id]() {
				auto a = h.get();
				if (!a || a->IsDead()) {
					return;
				}
				std::string done;
				for (const auto& [mgef, mag] : hits) {
					const float amount = mag * ResistFactor(a.get(), mgef);
					if (amount <= 0.0f) {
						continue;
					}
					a->AsActorValueOwner()->DamageActorValue(mgef->data.primaryAV, amount);
					if (mgef->GetArchetype() == RE::EffectSetting::Archetype::kDualValueModifier && mgef->data.secondaryAV != RE::ActorValue::kNone) {
						a->AsActorValueOwner()->DamageActorValue(mgef->data.secondaryAV, amount * mgef->data.secondAVWeight);
					}
					done += std::format("{}'{}' {:.2f}", done.empty() ? "" : ", ", mgef->GetName() ? mgef->GetName() : "?", amount);
				}
				logger::info("travel: projectile {:08X}: your own spell hit you through a portal: {}", id, done);
			});
		}

		void ApplyTwinHits(RE::Projectile* a_src, RE::Projectile* a_twin, const NiPoint3& a_from, const NiPoint3& a_dir, float a_len)
		{
			const auto& rd = a_src->GetProjectileRuntimeData();
			auto*       spell = rd.spell;
			auto*       base = a_src->GetProjectileBase();
			// cones (Ice Storm) are left to the engine: tested here (G345/G346) the cone's own hit came too late for EngineHit (80 = twice into teal) and still missed into violet
			const bool cone = false;
			if (!g_twinHits.load() || !spell || !base || base->data.explosionType || !(a_src->IsFlameProjectile() || a_src->IsBeamProjectile())) {
				return;
			}
			// G344: Chain Lightning's 40 into teal came from a chain off the entry wall, not from the twin, and the cone (Ice Storm) misses into violet; where the engine does hit with the
			// twin (into violet) EngineHit below keeps the target from being hurt twice, so these are tested here too
			auto* player = RE::PlayerCharacter::GetSingleton();
			auto* cell = player ? player->GetParentCell() : nullptr;
			if (!cell) {
				return;
			}
			const bool  once = spell->GetCastingType() != RE::MagicSystem::CastingType::kConcentration;
			if (a_twin && a_twin->GetProjectileRuntimeData().livingTime < 0.12f) {
				return;  // give the engine's own hit with the twin its first frames, so EngineHit knows about it (G340: Lightning Bolt into violet hurt twice, 50)
			}
			const float width = a_src->IsFlameProjectile() ? 55.0f : cone ? 70.0f : 40.0f;  // a cone widens further below
			const float len = (std::min)(a_len, 3000.0f);
			const float dt = static_cast<float>(g_dt);
			auto&       hitOnce = g_twinHitOnce[a_src->GetFormID()];
			cell->ForEachReferenceInRange(a_from + a_dir * (len * 0.5f), len * 0.5f + 120.0f, [&](RE::TESObjectREFR* a_ref) {
				auto* a = a_ref ? a_ref->As<RE::Actor>() : nullptr;
				if (!a || a == player || a->IsDead() || a->IsDisabled() || !a->Is3DLoaded() || (once && hitOnce.contains(a->GetFormID()))) {
					return RE::BSContainer::ForEachResult::kContinue;
				}
				if (a_twin && EngineHit(a_twin->GetFormID(), a->GetFormID(), once ? 1.0e9 : 0.5)) {
					return RE::BSContainer::ForEachResult::kContinue;  // the engine hit it with the twin itself
				}
				const NiPoint3 c = a->GetPosition() + NiPoint3{ 0, 0, ActorHalfHeight(a) };
				const float    s = Dot(c - a_from, a_dir);
				const float    off = Len(c - (a_from + a_dir * s));
				if (s < 0.0f || s > len || off > width + 25.0f + (cone ? s * 0.3f : 0.0f)) {
					return RE::BSContainer::ForEachResult::kContinue;
				}
				const auto wall = world::Raycast(a_from, a_dir, s, RE::COL_LAYER::kLOS);
				if (wall.hit && wall.dist < s - 45.0f && wall.ref != a) {
					return RE::BSContainer::ForEachResult::kContinue;  // something solid between the exit and the actor
				}
				std::string done;
				for (auto* e : spell->effects) {
					auto* mgef = e ? e->baseEffect : nullptr;
					if (!mgef || !mgef->IsDetrimental() || !e->conditions.IsTrue(player, a) || !mgef->conditions.IsTrue(player, a)) {
						continue;  // an effect only for some targets, by the spell's or the effect's own conditions (G336/G337: 'Disintegrate' 200 needs a nearly dead target and the perk)
					}
					const auto arch = mgef->GetArchetype();
					if (arch != RE::EffectSetting::Archetype::kValueModifier && arch != RE::EffectSetting::Archetype::kDualValueModifier && arch != RE::EffectSetting::Archetype::kPeakValueModifier) {
						continue;
					}
					const auto av = mgef->data.primaryAV;
					if (av != RE::ActorValue::kHealth && av != RE::ActorValue::kMagicka && av != RE::ActorValue::kStamina) {
						continue;  // only instant damage to the three pools (G345: Ice Storm's 'Slow' must not become a permanent loss of speed)
					}
					if (once && e->GetDuration() > 0) {
						continue;  // a timed effect of a fire-and-forget spell is not an instant hit
					}
					const float amount = e->GetMagnitude() * (once ? 1.0f : dt) * ResistFactor(a, mgef);
					if (amount <= 0.0f) {
						continue;
					}
					a->AsActorValueOwner()->DamageActorValue(mgef->data.primaryAV, amount);
					if (arch == RE::EffectSetting::Archetype::kDualValueModifier && mgef->data.secondaryAV != RE::ActorValue::kNone) {
						a->AsActorValueOwner()->DamageActorValue(mgef->data.secondaryAV, amount * mgef->data.secondAVWeight);
					}
					if (g_twinHitLogged.load() < 400) {
						done += std::format("{}'{}' {:.2f}", done.empty() ? "" : ", ", mgef->GetName() ? mgef->GetName() : "?", amount);
					}
				}
				if (once) {
					hitOnce.insert(a->GetFormID());
				}
				if (!done.empty() && (once || (g_tick % 20) == 0) && g_twinHitLogged.fetch_add(1) < 400) {
					logger::info("travel: twin hit: {:08X} '{}' {:.0f} units past the exit (off the line {:.0f}): {}", a->GetFormID(), a->GetName() ? a->GetName() : "", s, off, done);
				}
				return RE::BSContainer::ForEachResult::kContinue;
			});
		}

		void TickStreams(const portals::Portal (&a_p)[2], const std::vector<RE::ProjectileHandle>& a_handles)
		{
			++g_tick;
			auto* player = RE::PlayerCharacter::GetSingleton();
			if (!player) {
				return;
			}
			std::set<RE::Projectile*> twins;
			for (auto& [id, tw] : g_twins) {
				if (auto p = tw.h.get(); p) {
					twins.insert(p.get());
				}
			}
			{
				std::lock_guard lk(g_poseMutex);
				for (auto it = g_twinPose.begin(); it != g_twinPose.end();) {
					it = twins.contains(it->first) ? std::next(it) : g_twinPose.erase(it);
				}
				for (auto it = g_twinPickAt.begin(); it != g_twinPickAt.end();) {
					it = twins.contains(it->first) ? std::next(it) : g_twinPickAt.erase(it);
				}
			}
			std::set<std::uint32_t> hitting;
			for (auto& h : a_handles) {
				auto  ptr = h.get();
				auto* proj = ptr.get();
				if (!proj || twins.contains(proj) || !IsStream(proj)) {
					continue;
				}
				auto& rd = proj->GetProjectileRuntimeData();
				// a stream is over when the engine destroys it (the cast is released): it sets kProcessedImpacts as soon as the stream touches something (G76: the source was dropped at the wall
				// and no stream crossed), and a stream that is held against a wall keeps on living
				if (rd.flags.any(RE::Projectile::Flags::kDestroyed)) {
					continue;
				}
				auto shooter = rd.shooter.get();
				if (!shooter || shooter.get() != player) {
					continue;
				}
				const float speed = Len(rd.linearVelocity);
				if (speed < 100.0f) {
					continue;  // the first frame: the engine has not aimed it yet
				}
				const NiPoint3 O = proj->GetPosition();
				const NiPoint3 D = rd.linearVelocity * (1.0f / speed);
				const float    full = (std::max)(proj->GetProjectileBase()->data.range, rd.range);
				for (int i = 0; i < 2; ++i) {
					const portals::Portal& A = a_p[i];
					const portals::Portal& B = a_p[1 - i];
					const float            dn = Dot(D, A.n);
					if (dn > -0.05f) {
						continue;
					}
					const float t = Dot(A.c - O, A.n) / dn;
					if (t <= 0.0f || t > full) {
						continue;
					}
					const NiPoint3 H = O + D * t;
					const NiPoint3 dd = H - A.c;
					const float    lx = Dot(dd, A.r) / (A.hw * 0.97f), ly = Dot(dd, A.u) / (A.hh * 0.97f);
					if (lx * lx + ly * ly > 1.0f) {
						continue;
					}
					if (proj->IsBeamProjectile() && rd.range < t - 3.0f) {
						continue;  // the beam ends before the plane: something else is in the way
					}
					const std::uint32_t id = proj->GetFormID();
					hitting.insert(id);
					// G113: the twin's own target pick (a Havok cast from its position) returned only hits on the wall behind the exit portal, 12 to 22 units behind the twin (168 dropped by the oval exception), the
					// cast's start volume overlapping the wall slab the twin sat 4 units in front of, so a flame or beam twin starts 28 units out; the cone (Ice Storm) hit the guard from 4 units and keeps that
					const float    startOut = (proj->IsFlameProjectile() || proj->IsBeamProjectile()) ? g_twinStartOut.load() : 4.0f;
					const NiPoint3 H2 = MapPoint(A, B, H) + B.n * startOut;
					const NiPoint3 D2 = Norm(MapDir(A, B, D));
					const float    remaining = (std::max)(full - t, 16.0f);
					RE::NiPointer<RE::Projectile> twin;
					if (auto it = g_twins.find(id); it != g_twins.end()) {
						twin = it->second.h.get();
						if (!twin || twin->GetProjectileRuntimeData().flags.any(RE::Projectile::Flags::kDestroyed)) {
							if (g_twinLogged < 150) {
								++g_twinLogged;
								logger::info("travel: twin of {:08X} ended by itself (living {:.3f})", id, twin ? twin->GetProjectileRuntimeData().livingTime : -1.0f);
							}
							g_twins.erase(it);
							twin = nullptr;
						}
					}
					if (!twin) {
						if (auto f = g_twinTry.find(id); f != g_twinTry.end() && g_tick - f->second < 12) {
							break;  // tried a moment ago: the engine may have refused or ended it
						}
						g_twinTry[id] = g_tick;
						const RE::Projectile::ProjectileRot rot{ -std::asin(std::clamp(D2.z, -1.0f, 1.0f)), std::atan2(D2.x, D2.y) };
						RE::Projectile::LaunchData          data = rd.spell ? RE::Projectile::LaunchData(player, H2, rot, rd.spell) : RE::Projectile::LaunchData(proj->GetProjectileBase(), player, H2, rot);
						data.power = rd.power;
						data.scale = rd.scale;
						data.castingSource = rd.castingSource;
						data.useOrigin = true;
						// The engine's launch call hands back the caster's own live beam or flame projectile when it finds one for the same shooter and casting source (G75: the "twin" was the
						// source itself, and moving it ended it). Such a launch is made new by the other hand's casting source or by the launch data's flag at +0x9F (checked offline in
						// Projectile::Launch: the beam and flame cases only look the old one up when that byte is 0). Cones and barriers are always new.
						const bool reused = proj->GetProjectileBase()->IsBeam() || proj->GetProjectileBase()->IsFlamethrower();
						if (reused) {
							const int mode = g_twinMode.load();
							if (mode == 0 || mode == 2) {
								data.castingSource = rd.castingSource == RE::MagicSystem::CastingSource::kRightHand ? RE::MagicSystem::CastingSource::kLeftHand : RE::MagicSystem::CastingSource::kRightHand;
							}
							data.chainShatter = (mode == 1 || mode == 2);
						}
						RE::ProjectileHandle nh;
						RE::Projectile::Launch(&nh, data);
						twin = nh.get();
						if (!twin) {
							logger::warn("travel: the twin of {:08X} could not be launched at the exit", id);
							break;
						}
						if (twin.get() == proj) {
							if (g_twinLogged < 400) {
								++g_twinLogged;
								logger::warn("travel: the launch returned the source projectile {:08X} itself (mode {}); left alone", id, g_twinMode.load());
							}
							break;
						}
						g_twins[id] = { nh, g_tick };
						g_twinsMade.fetch_add(1);
						logger::info("travel: twin {:08X} of {} {:08X} launched through portal {} -> {}: source ({:.1f},{:.1f},{:.1f}) direction ({:.3f},{:.3f},{:.3f}) reaches the plane after {:.0f} of {:.0f}; exit ({:.1f},{:.1f},{:.1f}) direction ({:.3f},{:.3f},{:.3f})",
							twin->GetFormID(), proj->IsBeamProjectile() ? "beam" : proj->IsFlameProjectile() ? "flame" : "cone", id, A.colour, B.colour, O.x, O.y, O.z, D.x, D.y, D.z, t, full, H2.x, H2.y, H2.z, D2.x, D2.y, D2.z);
						logger::info("travel: the source's angles (pitch {:.1f}, heading {:.1f}) deg for the aim ({:.3f},{:.3f},{:.3f}); the twin's, as the launch left them (pitch {:.1f}, heading {:.1f}) deg", proj->data.angle.x * 57.29578f, proj->data.angle.z * 57.29578f, D.x, D.y, D.z, twin->data.angle.x * 57.29578f, twin->data.angle.z * 57.29578f);
					}
					if (auto tit = g_twins.find(id); tit != g_twins.end()) {
						// the engine updates the twin between our ticks: if it moves a stream back to the caster's hand (or re-aims it) the position here differs from where the last tick put it
						Twin&          tw = tit->second;
						const NiPoint3 now = twin->GetPosition();
						if (tw.hasPlaced && tw.driftLogged < 8 && Len(now - tw.placed) > 1.0f) {
							++tw.driftLogged;
							const auto& tr = twin->GetProjectileRuntimeData();
							logger::info("travel: twin {:08X} of {:08X} was moved by the engine since the last tick: put at ({:.1f},{:.1f},{:.1f}), now at ({:.1f},{:.1f},{:.1f}), linear ({:.0f},{:.0f},{:.0f}), living {:.3f}", twin->GetFormID(), id, tw.placed.x, tw.placed.y, tw.placed.z,
								now.x, now.y, now.z, tr.linearVelocity.x, tr.linearVelocity.y, tr.linearVelocity.z, tr.livingTime);
						}
						tw.placed = H2;
						tw.hasPlaced = true;
					}
					PlaceStream(twin.get(), H2, D2, speed, remaining);
					ApplyTwinHits(proj, twin.get(), H2, D2, remaining);
					{
						std::lock_guard lk(g_poseMutex);
						g_twinPose[twin.get()] = { H2, D2, speed, remaining, twin->GetFormID() };
					}
					if (g_twinLogged < 400 && (g_tick % 3) == 0) {
						++g_twinLogged;
						const auto& tr = twin->GetProjectileRuntimeData();
						logger::info("travel: twin {:08X} of {:08X}: at ({:.1f},{:.1f},{:.1f}) linear ({:.0f},{:.0f},{:.0f}) living {:.3f} range {:.0f} flags {:#x}; source living {:.3f} range {:.0f}", twin->GetFormID(), id, twin->GetPosition().x, twin->GetPosition().y,
							twin->GetPosition().z, tr.linearVelocity.x, tr.linearVelocity.y, tr.linearVelocity.z, tr.livingTime, tr.range, static_cast<std::uint32_t>(tr.flags.get()), rd.livingTime, rd.range);
					}
					break;
				}
			}
			for (auto it = g_twins.begin(); it != g_twins.end();) {
				if (hitting.contains(it->first)) {
					++it;
					continue;
				}
				g_twinHitOnce.erase(it->first);
				if (auto p = it->second.h.get(); p) {
					logger::info("travel: twin {:08X} of {:08X} ends (the source no longer reaches a portal), lived {:.3f}", p->GetFormID(), it->first, p->GetProjectileRuntimeData().livingTime);
					{
						std::lock_guard lk(g_poseMutex);
						g_twinPose.erase(p.get());
					}
					KillProjectile(p.get());
				}
				it = g_twins.erase(it);
			}
		}

		// ---- spells delivered at a target location (runes) ----
		// A rune does not fly: the cast puts it where the aim ray lands (G75: it stays at the point it was launched at, velocity zero). When that point is on the wall a portal sits on,
		// inside its oval, the aim ray went into the portal, so the ray goes on from the exit with the mapped direction and the rune is put where that ray lands.
		bool IsRune(RE::Projectile* a_p)
		{
			const auto& rd = a_p->GetProjectileRuntimeData();
			return a_p->IsGrenadeProjectile() && rd.spell && rd.spell->GetDelivery() == RE::MagicSystem::Delivery::kTargetLocation;
		}

		std::set<std::uint32_t> g_runesDone;

		// ---- decals stamped on a portal's wall (G104, G105) ----
		// The rune's glyph circle stayed on the entry wall around the oval: it is a decal the engine stamps onto the surface when the rune is created, and a moved node cannot take it along.
		// Every decal goes through BGSDecalManager::ApplyDecal (id 15203; G105: the rune's decal is three calls, size 169x169x64, from a worker thread, the Fireball's scorch likewise). A decal whose
		// origin lies in a portal's oval column (inside the oval x1.15, from 40 units behind the plane to 24 in front) is not stamped there. Its data is kept (the calls that carry a node), and when TickRunes
		// has found where the rune lands after the exit (game thread, where the ray test is safe) the decal is stamped there: origin, normal, referenced object and node of the landing surface.
		using ApplyDecal_t = std::uintptr_t (*)(RE::BGSDecalManager*, RE::DECAL_CREATION_DATA*, bool, RE::BGSDecalGroup*);
		ApplyDecal_t       g_origApplyDecal{ nullptr };
		std::atomic<bool>  g_decalGate{ false };
		std::atomic<int>   g_decalLogged{ 0 }, g_decalSkipped{ 0 }, g_decalMoved{ 0 };
		struct PendingDecal
		{
			RE::DECAL_CREATION_DATA                data;
			RE::BGSDecalManager*                   mgr{ nullptr };
			RE::BGSDecalGroup*                     group{ nullptr };
			bool                                   force{ false };
			NiPoint3                               origin;
			std::chrono::steady_clock::time_point  at;
		};
		std::vector<PendingDecal> g_pendingDecals;
		std::mutex                g_pendingMutex;

		std::uintptr_t Hook_ApplyDecal(RE::BGSDecalManager* a_self, RE::DECAL_CREATION_DATA* a_d, bool a_force, RE::BGSDecalGroup* a_group)
		{
			if (g_decalGate.load() && a_d) {
				const portals::Portal p[2] = { portals::Get(0), portals::Get(1) };
				for (int i = 0; i < 2; ++i) {
					const portals::Portal& A = p[i];
					if (!A.valid || !p[1 - i].valid) {
						continue;
					}
					const NiPoint3 P = { a_d->origin.x, a_d->origin.y, a_d->origin.z };
					const NiPoint3 dd = P - A.c;
					const float    dist = Dot(dd, A.n);
					const float    lx = Dot(dd, A.r) / (A.hw * 1.15f), ly = Dot(dd, A.u) / (A.hh * 1.15f);
					const bool     inside = dist > -40.0f && dist < 24.0f && lx * lx + ly * ly <= 1.0f;
					if (g_decalLogged.load() < 40 && (inside || g_decalLogged.load() < 12)) {
						g_decalLogged.fetch_add(1);
						logger::info("travel: decal at ({:.1f},{:.1f},{:.1f}) normal ({:.2f},{:.2f},{:.2f}) size {:.0f}x{:.0f}x{:.0f} texture set {:08X} node '{}' clone {} thread {}{}", P.x, P.y, P.z, a_d->direction.x, a_d->direction.y, a_d->direction.z, a_d->width, a_d->height, a_d->depth,
							a_d->texSet ? a_d->texSet->GetFormID() : 0u, a_d->avObj && a_d->avObj->name.c_str() ? a_d->avObj->name.c_str() : "", a_d->clone != nullptr, GetCurrentThreadId(), inside ? std::format(" - in portal {}'s oval column (distance {:.1f})", A.colour, dist) : "");
					}
					if (!inside) {
						continue;
					}
					g_decalSkipped.fetch_add(1);
					if (a_d->avObj && !a_d->clone) {  // the calls without a node do nothing in the engine (decal data note), so only the real one is kept for the exit
						std::lock_guard lk(g_pendingMutex);
						PendingDecal     pd;
						pd.data = *a_d;
						pd.mgr = a_self;
						pd.group = a_group;
						pd.force = a_force;
						pd.origin = P;
						pd.at = std::chrono::steady_clock::now();
						g_pendingDecals.push_back(std::move(pd));
						if (g_pendingDecals.size() > 16) {
							g_pendingDecals.erase(g_pendingDecals.begin());
						}
					}
					return 0;  // not stamped on the portal's wall
				}
			}
			return g_origApplyDecal(a_self, a_d, a_force, a_group);
		}

		// the rune at a_runePos landed on a_hit after the exit: the decal kept for it is stamped there (game thread). G108: the engine stamps a decal once per reference whose geometry the decal's box
		// touches (the control rune made calls for 'WRStableStr01End', 'WRStableWallBackStr01', 'WRStableStr01' and 'WRStableWallShortStr01', two each), not once on the one reference the ray hit, so
		// the references around the landing (static, movable static, furniture, door, activator, within the box's reach) get one call each, with `force` set and the LOD-gate flag raised (the engine's
		// gate compares with the player's position, which is at the entry portal).
		void StampPendingDecals(const NiPoint3& a_runePos, const world::Hit& a_hit)
		{
			std::vector<PendingDecal> mine;
			{
				std::lock_guard lk(g_pendingMutex);
				const auto      now = std::chrono::steady_clock::now();
				for (auto it = g_pendingDecals.begin(); it != g_pendingDecals.end();) {
					const bool old = std::chrono::duration<double>(now - it->at).count() > 3.0;
					if (!old && Len(it->origin - a_runePos) < 40.0f) {
						mine.push_back(std::move(*it));
						it = g_pendingDecals.erase(it);
					} else {
						it = old ? g_pendingDecals.erase(it) : std::next(it);
					}
				}
			}
			if (mine.empty()) {
				return;
			}
			auto* player = RE::PlayerCharacter::GetSingleton();
			auto* cell = player ? player->GetParentCell() : nullptr;
			if (!a_hit.hit || !cell || !g_origApplyDecal) {
				logger::info("travel: the decal for the rune at ({:.1f},{:.1f},{:.1f}) is dropped: no landing or no cell (hit {})", a_runePos.x, a_runePos.y, a_runePos.z, a_hit.hit);
				return;
			}
			const PendingDecal& tpl = mine.front();  // the three calls of one rune only differ in the node
			const float         reach = (std::max)(tpl.data.width, tpl.data.height) * 0.6f + 40.0f;
			std::vector<RE::TESObjectREFR*> targets;
			cell->ForEachReferenceInRange(a_hit.pos, reach, [&](RE::TESObjectREFR* a_ref) {
				if (!a_ref || a_ref->IsDisabled() || !a_ref->Get3D() || a_ref->As<RE::Actor>() || a_ref->As<RE::Projectile>()) {
					return RE::BSContainer::ForEachResult::kContinue;
				}
				auto* base = a_ref->GetBaseObject();
				if (base && base->Is(RE::FormType::Static, RE::FormType::MovableStatic, RE::FormType::Furniture, RE::FormType::Door, RE::FormType::Activator)) {
					targets.push_back(a_ref);
				}
				return RE::BSContainer::ForEachResult::kContinue;
			});
			if (a_hit.ref && a_hit.ref->Get3D() && std::find(targets.begin(), targets.end(), a_hit.ref) == targets.end()) {
				targets.push_back(a_hit.ref);  // the reference the ray hit, whatever its kind
			}
			if (targets.size() > 12) {
				targets.resize(12);
			}
			const std::uint32_t c0 = tpl.mgr->decalCount;
			int                 n = 0;
			for (auto* ref : targets) {
				RE::DECAL_CREATION_DATA d = tpl.data;
				d.origin = a_hit.pos;
				d.direction = a_hit.normal;
				d.surfaceNormal = a_hit.normal;
				d.objRef = ref->GetHandle();
				d.avObj = RE::NiPointer<RE::NiAVObject>(ref->Get3D());
				d.parentCell = cell;
				d.unkBB = 1;  // skips the engine's decal distance gate (800 units from the player)
				g_origApplyDecal(tpl.mgr, &d, true, tpl.group);
				++n;
				if (n <= 12) {
					logger::info("travel: decal (texture set {:08X}, size {:.0f}x{:.0f}x{:.0f}) stamped at the exit landing ({:.1f},{:.1f},{:.1f}) normal ({:.2f},{:.2f},{:.2f}) on ref {:08X} '{}' (form type {})", tpl.data.texSet ? tpl.data.texSet->GetFormID() : 0u, tpl.data.width, tpl.data.height,
						tpl.data.depth, a_hit.pos.x, a_hit.pos.y, a_hit.pos.z, a_hit.normal.x, a_hit.normal.y, a_hit.normal.z, ref->GetFormID(), d.avObj->name.c_str() ? d.avObj->name.c_str() : "",
						ref->GetBaseObject() ? static_cast<int>(ref->GetBaseObject()->GetFormType()) : -1);
				}
			}
			g_decalMoved.fetch_add(n);
			logger::info("travel: {} decal call(s) at the exit landing for the rune at ({:.1f},{:.1f},{:.1f}) (reach {:.0f}); decal manager decalCount {} -> {}", n, a_runePos.x, a_runePos.y, a_runePos.z, reach, c0, tpl.mgr->decalCount);
		}

		void InstallDecalHook()
		{
			static bool done = false;
			g_decalGate = true;
			if (done) {
				return;
			}
			done = true;
			static const hooks::Spec kApplyDecal{ "BGSDecalManager::ApplyDecal", 15203, { 0x4C, 0x8B, 0xDC, 0x56, 0x57, 0x41, 0x54, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83, 0xEC, 0x50, 0x49 } };
			if (hooks::Install(kApplyDecal, reinterpret_cast<void*>(&Hook_ApplyDecal), reinterpret_cast<void**>(&g_origApplyDecal))) {
				logger::info("travel: hooked BGSDecalManager::ApplyDecal (id 15203): a decal in a portal's oval is kept for the rune's exit landing, where it is stamped on every reference around it (as the engine's own cast does), otherwise dropped");
			}
		}

		// G103: the relocated rune's data.location and collision phantom moved, but its 3D (the rune circle, the glyph) stayed on the entry wall around the oval: nothing moves a rune's node but
		// the rune. So the node goes to the landing point too, turned so that what lay on the entry wall (facing a_fromN) lies on the landing surface (facing a_toN): the smallest rotation that
		// takes the one normal onto the other, applied to the node's own rotation, whichever of its axes is the facing one.
		void MoveRuneNode(RE::Projectile* a_p, const NiPoint3& a_pos, const NiPoint3& a_fromN, const NiPoint3& a_toN)
		{
			auto* r3 = a_p->Get3D();
			if (!r3) {
				logger::warn("travel: rune {:08X} has no 3D to move", a_p->GetFormID());
				return;
			}
			const NiPoint3 f = Norm(a_fromN), t = Norm(a_toN);
			float          R[3][3];
			const float    c = Dot(f, t);
			if (c < -0.9999f) {  // opposite: half a turn about any axis across the old normal
				NiPoint3 ax = std::fabs(f.x) < 0.9f ? NiPoint3{ 1, 0, 0 } : NiPoint3{ 0, 1, 0 };
				ax = Norm(NiPoint3{ f.y * ax.z - f.z * ax.y, f.z * ax.x - f.x * ax.z, f.x * ax.y - f.y * ax.x });
				const float a[3] = { ax.x, ax.y, ax.z };
				for (int i = 0; i < 3; ++i) {
					for (int j = 0; j < 3; ++j) R[i][j] = 2.0f * a[i] * a[j] - (i == j ? 1.0f : 0.0f);
				}
			} else {  // Rodrigues: R = I + [v]x + [v]x^2 / (1 + c), v = f x t
				const NiPoint3 v{ f.y * t.z - f.z * t.y, f.z * t.x - f.x * t.z, f.x * t.y - f.y * t.x };
				const float    K[3][3] = { { 0, -v.z, v.y }, { v.z, 0, -v.x }, { -v.y, v.x, 0 } };
				for (int i = 0; i < 3; ++i) {
					for (int j = 0; j < 3; ++j) {
						float k2 = 0.0f;
						for (int m = 0; m < 3; ++m) k2 += K[i][m] * K[m][j];
						R[i][j] = (i == j ? 1.0f : 0.0f) + K[i][j] + k2 / (1.0f + c);
					}
				}
			}
			const auto& O = r3->world.rotate.entry;
			logger::info("travel: rune {:08X} node axes before the move, dotted with the entry wall's normal ({:.2f},{:.2f},{:.2f}): column 0 {:.2f}, column 1 {:.2f}, column 2 {:.2f}; node at ({:.1f},{:.1f},{:.1f})", a_p->GetFormID(), f.x, f.y, f.z,
				O[0][0] * f.x + O[1][0] * f.y + O[2][0] * f.z, O[0][1] * f.x + O[1][1] * f.y + O[2][1] * f.z, O[0][2] * f.x + O[1][2] * f.y + O[2][2] * f.z, r3->world.translate.x, r3->world.translate.y, r3->world.translate.z);
			RE::NiMatrix3 m;
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					float sum = 0.0f;
					for (int k = 0; k < 3; ++k) sum += R[i][k] * O[k][j];
					m.entry[i][j] = sum;
				}
			}
			auto apply = [&]() {
				r3->local.rotate = m;
				r3->world.rotate = m;
				r3->local.translate = a_pos;
				r3->world.translate = a_pos;
			};
			apply();
			a_p->Update3DPosition(true);
			apply();  // the engine's refresh may have written the old rotation back (G100: it rebuilds a node's rotation from the angle)
			RE::NiUpdateData ud{};
			ud.time = 0.0f;
			ud.flags.set(RE::NiUpdateData::Flag::kDirty);
			r3->Update(ud);
			const auto& N = r3->world.rotate.entry;
			logger::info("travel: rune {:08X} node moved to ({:.1f},{:.1f},{:.1f}), axes against the landing surface's normal ({:.2f},{:.2f},{:.2f}): column 0 {:.2f}, column 1 {:.2f}, column 2 {:.2f}", a_p->GetFormID(), r3->world.translate.x, r3->world.translate.y, r3->world.translate.z, t.x, t.y,
				t.z, N[0][0] * t.x + N[1][0] * t.y + N[2][0] * t.z, N[0][1] * t.x + N[1][1] * t.y + N[2][1] * t.z, N[0][2] * t.x + N[1][2] * t.y + N[2][2] * t.z);
		}

		void TickRunes(const portals::Portal (&a_p)[2], const std::vector<RE::ProjectileHandle>& a_handles)
		{
			auto*          player = RE::PlayerCharacter::GetSingleton();
			world::CamInfo cam;
			if (!player || !world::CameraFull(cam)) {
				return;
			}
			std::set<std::uint32_t> alive;
			for (auto& h : a_handles) {
				auto  ptr = h.get();
				auto* proj = ptr.get();
				if (!proj || !IsRune(proj)) {
					continue;
				}
				const std::uint32_t id = proj->GetFormID();
				auto&               rd = proj->GetProjectileRuntimeData();
				alive.insert(id);
				if (g_runesDone.contains(id) || rd.flags.any(RE::Projectile::Flags::kDestroyed)) {
					continue;
				}
				auto shooter = rd.shooter.get();
				if (!shooter || shooter.get() != player) {
					continue;
				}
				const NiPoint3 P = proj->GetPosition();
				for (int i = 0; i < 2; ++i) {
					const portals::Portal& A = a_p[i];
					const portals::Portal& B = a_p[1 - i];
					const float            dist = Dot(P - A.c, A.n);
					const NiPoint3         dd = P - A.c;
					const float            lx = Dot(dd, A.r) / A.hw, ly = Dot(dd, A.u) / A.hh;
					if (dist < -6.0f || dist > 14.0f || lx * lx + ly * ly > 1.0f) {
						continue;  // not on this portal's wall inside its oval
					}
					const NiPoint3 D = Norm(P - cam.pos);
					if (Dot(D, A.n) > -0.05f) {
						continue;
					}
					g_runesDone.insert(id);
					const NiPoint3 H = MapPoint(A, B, P - A.n * dist);
					const NiPoint3 D2 = Norm(MapDir(A, B, D));
					// the ray starts clear of the exit wall's relief (G372: from 2 units out it hit the back of a beam on the exit wall, 3 units from the exit, facing into the
					// wall, and the rune and its ring sat on the wall), and a surface that faces away from the ray (a back face) is passed through
					const NiPoint3 H0 = H + B.n * 18.0f;
					auto           hit = world::Raycast(H0, D2, 3000.0f, RE::COL_LAYER::kLOS);
					for (int bf = 0; bf < 3 && hit.hit && Dot(hit.normal, D2) > 0.0f; ++bf) {
						const NiPoint3 from = hit.pos + D2 * 2.0f;
						hit = world::Raycast(from, D2, 3000.0f, RE::COL_LAYER::kLOS);
					}
					logger::info("travel: rune {:08X} '{}' at ({:.1f},{:.1f},{:.1f}) is on portal {}'s wall (distance {:.1f}); aim ({:.3f},{:.3f},{:.3f}) -> from the exit ({:.1f},{:.1f},{:.1f}) direction ({:.3f},{:.3f},{:.3f}): {}", id,
						rd.spell && rd.spell->GetName() ? rd.spell->GetName() : "", P.x, P.y, P.z, A.colour, dist, D.x, D.y, D.z, H.x, H.y, H.z, D2.x, D2.y, D2.z,
						hit.hit ? std::format("lands at ({:.1f},{:.1f},{:.1f}) after {:.0f}, surface normal ({:.2f},{:.2f},{:.2f})", hit.pos.x, hit.pos.y, hit.pos.z, hit.dist, hit.normal.x, hit.normal.y, hit.normal.z) : "nothing within 3000");
					if (hit.hit) {
						const NiPoint3 np = hit.pos + hit.normal * 1.5f;
						proj->SetPosition(np);
						if (rd.unk0E0 && rd.unk0E0->phantom) {
							auto*               ph = rd.unk0E0->phantom.get();
							const float         sc = RE::bhkWorld::GetWorldScale();
							const RE::hkVector4 hp(np.x * sc, np.y * sc, np.z * sc, 0.0f);
							ph->motionState.transform.translation = hp;
							ph->motionState.sweptTransform.centerOfMass0 = hp;
							ph->motionState.sweptTransform.centerOfMass1 = hp;
						}
						MoveRuneNode(proj, np, A.n, hit.normal);
						StampPendingDecals(P, hit);
						{
							int element = 0;  // the ring's tint from the rune's element (resist value of its damage effect)
							if (auto* sp = rd.spell) {
								for (auto* e : sp->effects) {
									if (e && e->baseEffect) {
										const auto rv = e->baseEffect->data.resistVariable;
										if (rv == RE::ActorValue::kResistFrost) {
											element = 1;
											break;
										}
										if (rv == RE::ActorValue::kResistShock) {
											element = 2;
											break;
										}
										if (rv == RE::ActorValue::kResistFire) {
											element = 0;
											break;
										}
									}
								}
							}
							overlay::AddRuneRing(id, hit.pos, hit.normal, element, 80.0f);
						}
						g_projectiles.fetch_add(1);
					}
					break;
				}
			}
			for (auto it = g_runesDone.begin(); it != g_runesDone.end();) {
				it = alive.contains(*it) ? std::next(it) : g_runesDone.erase(it);
			}
			{
				std::set<std::uint32_t> armed;  // runes still lying in wait (a triggered rune is destroyed or has processed its impact)
				for (auto& h : a_handles) {
					auto  ptr = h.get();
					auto* proj = ptr.get();
					if (proj && IsRune(proj) && !proj->GetProjectileRuntimeData().flags.any(RE::Projectile::Flags::kDestroyed, RE::Projectile::Flags::kProcessedImpacts)) {
						armed.insert(proj->GetFormID());
					}
				}
				overlay::KeepRuneRings(armed);
			}
		}

		void FollowProjectiles()
		{
			++g_followTick;
			for (auto it = g_follow.begin(); it != g_follow.end();) {
				auto  ptr = it->second.get();
				auto* proj = ptr ? ptr->As<RE::Projectile>() : nullptr;
				if (!proj) {
					logger::info("travel: projectile {:08X} is gone", it->first);
					it = g_follow.erase(it);
					continue;
				}
				const auto&    rd = proj->GetProjectileRuntimeData();
				const NiPoint3 pos = proj->GetPosition();
				const bool     ended = rd.flags.any(RE::Projectile::Flags::kDestroyed, RE::Projectile::Flags::kProcessedImpacts);
				logger::info("travel: follow {:08X} at ({:.1f},{:.1f},{:.1f}) linear ({:.0f},{:.0f},{:.0f}) distance {:.0f} living {:.3f}{}", it->first, pos.x, pos.y, pos.z, rd.linearVelocity.x, rd.linearVelocity.y, rd.linearVelocity.z, rd.distanceMoved,
					rd.livingTime, ended ? " ENDED" : "");
				if (auto* r3 = proj->Get3D(); r3 && g_axesLater[it->first] < 2) {
					++g_axesLater[it->first];
					const NiPoint3 fd = Norm(NiPoint3{ rd.linearVelocity.x, rd.linearVelocity.y, rd.linearVelocity.z });
					const auto&    R = r3->world.rotate.entry;
					logger::info("travel: follow {:08X} node axes {} tick(s) after the carry, dotted with the flight direction ({:.3f},{:.3f},{:.3f}): column 0 {:.3f}, column 1 {:.3f}, column 2 {:.3f}", it->first, g_axesLater[it->first], fd.x, fd.y, fd.z,
						R[0][0] * fd.x + R[1][0] * fd.y + R[2][0] * fd.z, R[0][1] * fd.x + R[1][1] * fd.y + R[2][1] * fd.z, R[0][2] * fd.x + R[1][2] * fd.y + R[2][2] * fd.z);
				}
				it = ended ? g_follow.erase(it) : std::next(it);
			}
		}

		void TickProjectiles(const portals::Portal (&a_p)[2])
		{
			FollowProjectiles();
			{
				std::vector<PendingPush> pushes;
				{
					std::lock_guard lk(g_carriedMutex);
					pushes.swap(g_pushes);
				}
				for (const auto& pp : pushes) {
					auto actor = pp.actor.get();
					auto* proc = actor ? actor->GetActorRuntimeData().currentProcess : nullptr;
					if (!proc || actor->IsDead()) {
						continue;
					}
					const float mag = pp.magnitude * g_pushScale.load();
					proc->KnockExplosion(actor.get(), pp.src, mag);
					logger::info("travel: {:08X} '{}' pushed by carried projectile {:08X} from ({:.1f},{:.1f},{:.1f}) (its shooter seen from the exit), magnitude {:.2f}", actor->GetFormID(), actor->GetName(), pp.proj, pp.src.x, pp.src.y, pp.src.z, mag);
				}
			}
			g_approach = false;
			auto* mgr = RE::Projectile::Manager::GetSingleton();
			if (!mgr) {
				return;
			}
			std::vector<RE::ProjectileHandle> handles;
			{
				RE::BSSpinLockGuard lock(mgr->projectileLock);
				for (auto& h : mgr->unlimited) {
					handles.push_back(h);
				}
				for (auto& h : mgr->limited) {
					handles.push_back(h);
				}
				for (auto& h : mgr->pending) {
					handles.push_back(h);
				}
			}
			const float look = static_cast<float>(g_dt) * g_lookFrames.load();
			for (auto& h : handles) {
				auto ptr = h.get();
				auto* proj = ptr.get();
				if (proj && g_verbose.load()) {
					static std::set<std::uint32_t> seen;
					if (seen.insert(proj->GetFormID()).second) {
						const auto& r0 = proj->GetProjectileRuntimeData();
						logger::info("travel: new projectile {:08X} type missile {} flame {} beam {} grenade {} barrier {} at ({:.1f},{:.1f},{:.1f}) linear ({:.0f},{:.0f},{:.0f}) base '{}'", proj->GetFormID(), proj->IsMissileProjectile(), proj->IsFlameProjectile(), proj->IsBeamProjectile(),
							proj->IsGrenadeProjectile(), proj->IsBarrierProjectile(), proj->GetPosition().x, proj->GetPosition().y, proj->GetPosition().z, r0.linearVelocity.x, r0.linearVelocity.y, r0.linearVelocity.z,
							proj->GetProjectileBase() && proj->GetProjectileBase()->GetName() ? proj->GetProjectileBase()->GetName() : "");
					}
				}
				if (proj && g_verbose.load() && !proj->IsMissileProjectile() && (g_followTick % 3) == 0 && g_logged < 120) {
					++g_logged;
					const auto& r1 = proj->GetProjectileRuntimeData();
					logger::info("travel: non-missile projectile {:08X} (flame {} beam {} grenade {} barrier {}) at ({:.1f},{:.1f},{:.1f}) linear ({:.0f},{:.0f},{:.0f}) living {:.3f} range {:.0f}", proj->GetFormID(), proj->IsFlameProjectile(), proj->IsBeamProjectile(), proj->IsGrenadeProjectile(),
						proj->IsBarrierProjectile(), proj->GetPosition().x, proj->GetPosition().y, proj->GetPosition().z, r1.linearVelocity.x, r1.linearVelocity.y, r1.linearVelocity.z, r1.livingTime, r1.range);
				}
				if (!proj || !(proj->IsMissileProjectile() || proj->IsFlameProjectile() || proj->IsGrenadeProjectile() || (IsCone(proj) && g_coneCarry.load())) || IsStream(proj) || IsRune(proj)) {
					continue;  // beams, flame streams and cones stay at the caster's hand (TickStreams); runes are put where the aim ray lands (TickRunes)
				}
				auto& rd = proj->GetProjectileRuntimeData();
				if (rd.flags.any(RE::Projectile::Flags::kDestroyed, RE::Projectile::Flags::kProcessedImpacts)) {
					continue;
				}
				if (auto it = g_boost.find(proj->GetFormID()); it != g_boost.end() && g_logged < 60) {
					++g_logged;
					logger::info("travel: boost candidate {:08X}: living {:.3f} flags {:#x} speed {:.0f}", proj->GetFormID(), rd.livingTime, static_cast<std::uint32_t>(rd.flags.get()), Len(rd.linearVelocity));
				}
				if (auto it = g_boost.find(proj->GetFormID()); it != g_boost.end() && rd.livingTime > 0.03f) {
					const NiPoint3 d = Norm(rd.linearVelocity);
					rd.linearVelocity = d * it->second;
					logger::info("travel: test boost: projectile {:08X} now {:.0f} units/s along ({:.3f},{:.3f},{:.3f})", proj->GetFormID(), it->second, d.x, d.y, d.z);
					g_boost.erase(it);
				}
				const NiPoint3 pos = proj->GetPosition();
				const NiPoint3 vel = rd.linearVelocity;
				if (g_passage.load()) {
					// a projectile that straddles a portal's plane (an arrow's shaft, a slow spell) is drawn cut like any other entity; it is still carried by the sweep below
					if (auto* r3 = proj->Get3D()) {
						// a spell projectile is an effect glow (additive sprites): a cut draw of it smeared across the oval in G97b, so it is not handed to the clip pass; it is carried whole
							const float rad = rd.spell ? 0.0f : std::clamp(r3->worldBound.radius, 4.0f, 60.0f);
						for (int i = 0; i < 2; ++i) {
							if (Straddles(a_p[i], pos, rad)) {
								NoteStraddler(proj, i, rad);
							}
						}
					}
				}
				const float speed = Len(vel);
					if (g_passage.load() && speed >= 8.0f && (Straddles(a_p[0], pos, 300.0f) || Straddles(a_p[1], pos, 300.0f))) {
						g_approach = true;
					}
					if (speed < 8.0f) {
					continue;
				}
				if (g_verbose.load() && g_logged < 40 && (g_watchTick % 6) == 0) {
					++g_logged;
					logger::info("travel: projectile {:08X} at ({:.1f},{:.1f},{:.1f}) linear ({:.0f},{:.0f},{:.0f}) velocity ({:.0f},{:.0f},{:.0f}) distance {:.0f} living {:.3f}", proj->GetFormID(), pos.x, pos.y, pos.z, vel.x, vel.y, vel.z,
						rd.velocity.x, rd.velocity.y, rd.velocity.z, rd.distanceMoved, rd.livingTime);
				}
				// the step the engine really takes between two of our ticks: G97 saw a firebolt advance 89 units per tick (the projectile's own time step was 0.036 s per rendered frame of
					// 0.0167 s), more than twice speed * frame time, so the one-frame look-ahead missed the plane and the bolt hit the wall first; the measured step is what must be covered
					float    measured = 0.0f;
					bool     havePrev = false;
					NiPoint3 prevPos{};
					{
						auto& pp = g_projPrev[proj->GetFormID()];
						if (pp.second == g_watchTick - 1) {
							measured = Len(pos - pp.first);
							havePrev = true;
							prevPos = pp.first;
						}
						pp = { pos, g_watchTick };
					}
					// a fast projectile can cross the plane between two ticks (the oval's wall-impact exception lets it through): G355, an arrow at about 1500 units/s
					// shot from 800 units was let through the violet wall, never carried, and flew on behind it. So the step it just took is tested too, and it is carried
					// from where that step crossed the plane inside the oval
					if (havePrev && speed >= 100.0f) {
						bool carriedNow = false;
						for (int i = 0; i < 2 && !carriedNow; ++i) {
							float    t;
							NiPoint3 hit;
							if (Crosses(a_p[i], prevPos, pos, t, hit)) {
								logger::info("travel: projectile {:08X} crossed portal {} between two ticks (step {:.0f} units): carried from the crossing point", proj->GetFormID(), a_p[i].colour, measured);
								CarryProjectile(proj, a_p[i], a_p[1 - i], hit, vel);
								carriedNow = true;
							}
						}
						if (carriedNow) {
							continue;
						}
					}
					{
						bool     have = false;
						NiPoint3 dh{};
						{
							std::lock_guard lk(g_dropMutex);
							if (auto it = g_dropped.find(proj->GetFormID()); it != g_dropped.end()) {
								if (it->second.second < 0) {
									it->second.second = g_watchTick;  // stamped with our tick when first seen here
								}
								if (it->second.second < g_watchTick) {
									have = true;
									dh = it->second.first;
								}
							}
						}
						if (have) {
							bool carriedNow = false;
							for (int i = 0; i < 2 && !carriedNow; ++i) {
								const NiPoint3 dd = dh - a_p[i].c;
								const float    lx = Dot(dd, a_p[i].r) / (a_p[i].hw + 8.0f), ly = Dot(dd, a_p[i].u) / (a_p[i].hh + 8.0f);
								if (std::fabs(Dot(dd, a_p[i].n)) <= 80.0f && lx * lx + ly * ly <= 1.0f && Dot(pos - a_p[i].c, a_p[i].n) <= 0.0f) {
									const NiPoint3 onPlane = dh - a_p[i].n * Dot(dd, a_p[i].n);
									logger::info("travel: projectile {:08X} is behind portal {} after its wall impact there was dropped: carried from the impact point ({:.1f},{:.1f},{:.1f})", proj->GetFormID(), a_p[i].colour, dh.x, dh.y, dh.z);
									CarryProjectile(proj, a_p[i], a_p[1 - i], onPlane, vel);
									carriedNow = true;
								}
							}
							std::lock_guard lk(g_dropMutex);
							g_dropped.erase(proj->GetFormID());
							if (carriedNow) {
								continue;
							}
						}
					}
					float reach = (std::max)(speed * look, measured * 1.2f);
					if (speed < 100.0f && !hole::ProjectileHoleOn()) {
						// (fallback: with the impact exception on, a hit inside the oval is not recorded and the bolt flies on until its centre crosses the plane, no margin is needed)
						// a slow projectile is stopped by the wall before its centre could cross the plane (the sweep's one frame of travel is a fraction of a unit): it is looked at
						// one body radius ahead, so it is carried when its leading edge reaches the plane inside the oval, like a character capsule
						// G97b: the 60-unit extent of the fire effect carried the bolt about 57 units before the plane (a visible skip); G97c: the bolt's own collision radius (a few units) was
						// too little, the engine's hit test fired first (the bolt was alive 11 units from the plane at its last tick and then exploded on the wall, nothing carried). The hit
						// distance lies between the two: carry when the bolt is within twice its collision radius, never less than 24 units
						float rad = 24.0f;
						if (auto* pb = proj->GetProjectileBase()) {
							rad = std::clamp(pb->data.collisionRadius * 2.0f, 24.0f, 40.0f);
						}
						if (auto* r3 = proj->Get3D()) {
							(void)r3;
						}
						reach = (std::max)(reach, rad);
					}
					const NiPoint3 next = pos + vel * (reach / speed);
				for (int i = 0; i < 2; ++i) {
					float    t;
					NiPoint3 hit;
					if (Crosses(a_p[i], pos, next, t, hit)) {
						CarryProjectile(proj, a_p[i], a_p[1 - i], hit, vel);
						break;
					}
				}
			}
			TickStreams(a_p, handles);
			TickRunes(a_p, handles);
		}
	}

	void SetEnabled(bool a_on)
	{
		g_enabled = a_on;
		logger::info("travel: projectiles through portals {}", a_on ? "ON" : "off");
	}

	void SetVerbose(bool a_on)
	{
		g_verbose = a_on;
		g_logged = 0;
		g_twinLogged = 0;
	}

	void SetPushScale(float a_scale)
	{
		g_pushScale = a_scale;
		logger::info("travel: pushes by carried projectiles scaled x{:.2f}", a_scale);
	}

	void SetConeCarry(bool a_on)
	{
		g_coneCarry = a_on;
		logger::info("travel: cones (Ice Storm) are {}", a_on ? "carried like a missile" : "copied at the exit (twin)");
	}

	void SetConeOrigin(bool a_on)
	{
		g_coneOrigin = a_on;
		g_coneLogged = 0;
		logger::info("travel: a cone twin's origin is {}", a_on ? "moved to the exit with it" : "left to the engine");
	}

	void SetRelaunch(bool a_on)
	{
		g_relaunch = a_on;
		logger::info("travel: projectiles are carried by {}", a_on ? "launching a new one at the exit" : "moving the original (and its collision phantom)");
	}

	void SetLook(float a_frames)
	{
		g_lookFrames = a_frames;
		logger::info("travel: the sweep looks {:.2f} frame(s) ahead", a_frames);
	}

	void LogStats()
	{
		logger::info("travel: {} projectile(s) carried through portals ({} stream twin(s) launched); {} decal(s) kept off a portal's wall, {} of them stamped at the exit; enabled {}", g_projectiles.load(), g_twinsMade.load(), g_decalSkipped.load(), g_decalMoved.load(), g_enabled.load());
	}

	// The kinds of spell in the game data: casting type, delivery and the projectile's type, with a few examples of each (what has to go through a portal)
	void ScanSpells()
	{
		struct Row
		{
			int                      n{ 0 };
			std::vector<std::string> ex;
		};
		std::map<std::string, Row> rows;
		for (auto* sp : RE::TESDataHandler::GetSingleton()->GetFormArray<RE::SpellItem>()) {
			if (!sp || !sp->GetName() || !*sp->GetName()) {
				continue;
			}
			const auto type = sp->GetSpellType();
			if (type != RE::MagicSystem::SpellType::kSpell && type != RE::MagicSystem::SpellType::kPower && type != RE::MagicSystem::SpellType::kLesserPower && type != RE::MagicSystem::SpellType::kVoicePower) {
				continue;
			}
			std::string proj = "-";
			for (auto* e : sp->effects) {
				if (e && e->baseEffect && e->baseEffect->data.projectileBase) {
					const auto* b = e->baseEffect->data.projectileBase;
					proj.clear();
					proj += b->IsMissile() ? "missile " : "";
					proj += b->IsGrenade() ? "grenade " : "";
					proj += b->IsBeam() ? "beam " : "";
					proj += b->IsFlamethrower() ? "flame " : "";
					proj += b->IsCone() ? "cone " : "";
					proj += b->IsBarrier() ? "barrier " : "";
					proj += b->IsArrow() ? "arrow " : "";
					break;
				}
			}
			const std::string key = std::format("type {} casting {} delivery {} projectile {}", static_cast<int>(type), static_cast<int>(sp->GetCastingType()), static_cast<int>(sp->GetDelivery()), proj);
			auto&             r = rows[key];
			++r.n;
			if (r.ex.size() < 8) {
				r.ex.push_back(sp->GetName());
			}
		}
		for (auto& [k, r] : rows) {
			std::string names;
			for (auto& n : r.ex) {
				names += (names.empty() ? "" : ", ") + n;
			}
			logger::info("travel: spells {} x {}: {}", k, r.n, names);
		}
	}

	namespace
	{
		void TickTestNpc(double a_dt);
	}

	void Tick(double a_dt)
	{
		struct Timer
		{
			std::chrono::steady_clock::time_point t0{ std::chrono::steady_clock::now() };
			~Timer()
			{
				if (!g_passage.load() && !g_verbose.load()) {
					return;
				}
				const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
				g_tickSumMs += ms;
				g_tickMaxMs = (std::max)(g_tickMaxMs, ms);
				if (++g_tickN >= 240) {
					logger::info("travel: tick cost avg {:.3f} ms max {:.3f} ms over {} frames ({} watched entit(ies), {} body(ies) tracked, {} straddler(s) now)", g_tickSumMs / g_tickN, g_tickMaxMs, g_tickN, g_watch.size(), hole::TrackedCount(), g_straddle.size());
					g_tickSumMs = g_tickMaxMs = 0;
					g_tickN = 0;
				}
			}
		} tickTimer;
		g_dt = std::clamp(a_dt, 1.0 / 240.0, 1.0 / 20.0);
		TickTestNpc(a_dt);
		if (g_castLeft > 0) {
			g_castLeft -= a_dt;
			if (g_castLeft <= 0) {
				auto* player = RE::PlayerCharacter::GetSingleton();
				if (auto* caster = player ? player->GetMagicCaster(g_castSource) : nullptr) {
					logger::info("travel: finishing the cast (caster state {})", static_cast<std::uint32_t>(caster->state.get()));
					caster->FinishCast();
				}
			}
		}
		++g_watchTick;
		g_straddle.clear();
		const auto owns = [](RE::TESObjectREFR* a_ref, RE::hkpRigidBody* a_rb) {
			if (auto* b = BodyOf(a_ref); b && b->GetRigidBody() == a_rb) {
				return true;
			}
			std::vector<RE::bhkRigidBody*> v;
			CollectBodies(a_ref->Get3D(), v, 0);
			for (auto* b : v) {
				if (b->GetRigidBody() == a_rb) {
					return true;
				}
			}
			return false;
		};
		if (!g_enabled.load()) {
			EndWatch(owns);
			return;
		}
		const portals::Portal p[2] = { portals::Get(0), portals::Get(1) };
		if (!p[0].valid || !p[1].valid) {
			EndWatch(owns);
			return;
		}
		TickProjectiles(p);
		TickProps(p);
		TickActors(p);
		TickSummons(p);
		TickRagdolls(p);
		EndWatch(owns);
		for (auto it = g_prevC.begin(); it != g_prevC.end();) {
			it = (((std::max)(it->second.tick[0], it->second.tick[1])) < g_watchTick - 120) ? g_prevC.erase(it) : std::next(it);
		}
		if (g_autoLeft > 0 && (!g_straddle.empty() || g_approach || !g_twins.empty()) && (g_watchTick % g_autoEvery) == 0 && !shot::Busy()) {
			shot::Request(std::format("{}_{:02d}", g_autoTag, g_autoCount++));
			--g_autoLeft;
		}
		if (g_verbose.load() && !g_straddle.empty() && (g_watchTick % 6) == 0 && g_logged < 400) {
			++g_logged;
			for (const auto& st : g_straddle) {
				auto ref = st.h.get();
				logger::info("travel: straddling portal {}: {:08X} radius {:.0f}; {} watched entit(ies), {} body(ies) tracked", st.entry, ref ? ref->GetFormID() : 0, st.radius, g_watch.size(), hole::TrackedCount());
			}
		}
	}

	namespace
	{
		void ResetSkinsUnder(RE::NiAVObject* a_o, int a_depth)
		{
			if (!a_o || a_depth > 48) {
				return;
			}
			if (auto* g = a_o->AsGeometry()) {
				if (auto* si = g->GetGeometryRuntimeData().skinInstance.get()) {
					si->frameID = 0;
				}
			}
			if (auto* n = a_o->AsNode()) {
				for (auto& c : n->GetChildren()) {
					ResetSkinsUnder(c.get(), a_depth + 1);
				}
			}
		}
	}

	namespace
	{
		int g_skinLogLeft{ 0 };

		RE::BSGeometry* FirstSkinned(RE::NiAVObject* a_o, int a_depth)
		{
			if (!a_o || a_depth > 48) {
				return nullptr;
			}
			if (auto* g = a_o->AsGeometry()) {
				if (g->GetGeometryRuntimeData().skinInstance) {
					return g;
				}
			}
			if (auto* n = a_o->AsNode()) {
				for (auto& c : n->GetChildren()) {
					if (auto* r = FirstSkinned(c.get(), a_depth + 1)) {
						return r;
					}
				}
			}
			return nullptr;
		}
	}

	void SetSkinLog(int a_calls)
	{
		g_skinLogLeft = a_calls;
	}

	void ActorRoots(std::vector<RE::NiAVObject*>& a_out)
	{
		a_out.clear();
		auto* player = RE::PlayerCharacter::GetSingleton();
		auto* cell = player ? player->GetParentCell() : nullptr;
		if (!cell) {
			return;
		}
		cell->ForEachReferenceInRange(player->GetPosition(), 3500.0f, [&](RE::TESObjectREFR* a_ref) {
			if (auto* a = a_ref ? a_ref->As<RE::Actor>() : nullptr; a && a != player && a->Is3DLoaded()) {
				if (auto* r = a->Get3D()) {
					a_out.push_back(r);
				}
			}
			return RE::BSContainer::ForEachResult::kContinue;
		});
	}

	// test: the state of the test NPC's first skinned geometry at a named moment of the frame (when are the cached bone matrices rebuilt, and relative to what?)
	void LogSkin(const char* a_tag)
	{
		if (g_skinLogLeft <= 0) {
			return;
		}
		auto  npc = g_testNpc.get();
		auto* g = npc ? FirstSkinned(npc->Get3D(), 0) : nullptr;
		if (!g) {
			return;
		}
		--g_skinLogLeft;
		auto*  si = g->GetGeometryRuntimeData().skinInstance.get();
		float  m[12]{};
		void*  bm = si->boneMatrices;
		if (bm) {
			for (int i = 0; i < 12; ++i) {
				float v = 0;
				ReadSafeT(static_cast<const std::uint8_t*>(bm) + i * 4, v);
				m[i] = v;
			}
		}
		std::uint32_t present = 0;  // the engine's frame counter (id 411489, +1 after every Present): the skin cache is valid when frameID equals it
		ReadSafeT(reinterpret_cast<const void*>(REL::ID(411489).address()), present);
		logger::info("skin: {}: '{}' skin instance {:p} frameID {} (engine frame counter {}) bones {} matrices {:p} first matrix [{:.1f} {:.1f} {:.1f} {:.1f} | {:.1f} {:.1f} {:.1f} {:.1f} | {:.1f} {:.1f} {:.1f} {:.1f}]", a_tag, g->name.c_str() ? g->name.c_str() : "", static_cast<void*>(si), si->frameID, present, si->numMatrices,
			bm, m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11]);
	}

	namespace
	{
		void CollectGeoms(RE::NiAVObject* a_o, int a_depth, std::vector<RE::BSGeometry*>& a_out);
	}

	// test: hide or show every geometry of the test NPC whose name contains a_sub (kHidden on the geometry only)
	void HideNpcGeom(const std::string& a_sub, bool a_hide)
	{
		auto  npc = g_testNpc.get();
		auto* root = npc ? npc->Get3D() : nullptr;
		if (!root) {
			logger::info("travel: geom hide/show: no test NPC 3D");
			return;
		}
		std::vector<RE::BSGeometry*> geoms;
		CollectGeoms(root, 0, geoms);
		int n = 0;
		for (auto* g : geoms) {
			const std::string nm = g->name.c_str() ? g->name.c_str() : "";
			if (nm.find(a_sub) == std::string::npos) {
				continue;
			}
			if (a_hide) {
				g->flags.set(RE::NiAVObject::Flag::kHidden);
			} else {
				g->flags.reset(RE::NiAVObject::Flag::kHidden);
			}
			++n;
		}
		logger::info("travel: {} {} geometr{} of the test NPC matching '{}'", a_hide ? "hid" : "showed", n, n == 1 ? "y" : "ies", a_sub);
	}

	// test: hide every geometry of the test NPC except those whose name contains a_sub
	void HideNpcExcept(const std::string& a_sub)
	{
		auto  npc = g_testNpc.get();
		auto* root = npc ? npc->Get3D() : nullptr;
		if (!root) {
			logger::info("travel: geom only: no test NPC 3D");
			return;
		}
		std::vector<RE::BSGeometry*> geoms;
		CollectGeoms(root, 0, geoms);
		int shown = 0, hidden = 0;
		for (auto* g : geoms) {
			const std::string nm = g->name.c_str() ? g->name.c_str() : "";
			if (nm.find(a_sub) != std::string::npos) {
				g->flags.reset(RE::NiAVObject::Flag::kHidden);
				++shown;
			} else {
				g->flags.set(RE::NiAVObject::Flag::kHidden);
				++hidden;
			}
		}
		logger::info("travel: only geometries matching '{}' of the test NPC are drawn now ({} shown, {} hidden)", a_sub, shown, hidden);
	}

	void HideNpc(bool a_hide)
	{
		auto  npc = g_testNpc.get();
		auto* root = npc ? npc->Get3D() : nullptr;
		if (!root) {
			logger::info("travel: hide/show: no test NPC 3D");
			return;
		}
		if (a_hide) {
			root->flags.set(RE::NiAVObject::Flag::kHidden);
		} else {
			root->flags.reset(RE::NiAVObject::Flag::kHidden);
		}
		logger::info("travel: the test NPC's 3D root is now {} (flags {:#x})", a_hide ? "hidden" : "shown", root->flags.underlying());
	}

	namespace
	{
		void CollectGeoms(RE::NiAVObject* a_o, int a_depth, std::vector<RE::BSGeometry*>& a_out)
		{
			if (!a_o || a_depth > 48 || a_out.size() >= 24) {
				return;
			}
			if (auto* g = a_o->AsGeometry()) {
				a_out.push_back(g);
			}
			if (auto* n = a_o->AsNode()) {
				for (auto& c : n->GetChildren()) {
					CollectGeoms(c.get(), a_depth + 1, a_out);
				}
			}
		}
	}

	void LogNpcBounds(const char* a_tag, const float* a_planes, const RE::NiPoint3& a_camPos)
	{
		auto  npc = g_testNpc.get();
		auto* root = npc ? npc->Get3D() : nullptr;
		if (!root) {
			logger::info("bounds: {}: no test NPC 3D", a_tag);
			return;
		}
		logger::info("bounds: {}: root '{}' flags {:#x} at ({:.0f},{:.0f},{:.0f}), bound centre ({:.0f},{:.0f},{:.0f}) radius {:.0f}, virtual camera ({:.0f},{:.0f},{:.0f}), planes {}", a_tag, root->name.c_str() ? root->name.c_str() : "", root->flags.underlying(),
			root->world.translate.x, root->world.translate.y, root->world.translate.z, root->worldBound.center.x, root->worldBound.center.y, root->worldBound.center.z, root->worldBound.radius, a_camPos.x, a_camPos.y, a_camPos.z, a_planes ? "custom" : "none");
		std::vector<RE::BSGeometry*> geoms;
		CollectGeoms(root, 0, geoms);
		for (auto* g : geoms) {
			std::uint32_t chain = 0;  // the flags of the geometry and every ancestor, OR-ed (a hidden ancestor hides it)
			for (RE::NiAVObject* o = g; o; o = o->parent) {
				chain |= o->flags.underlying();
			}
			auto*         sp = g->GetGeometryRuntimeData().shaderProperty.get();
			const auto&   b = g->worldBound;
			std::string   dist;
			if (a_planes) {
				for (int i = 0; i < 6; ++i) {
					const float* p = a_planes + i * 4;
					dist += std::format(" {:.0f}", p[0] * b.center.x + p[1] * b.center.y + p[2] * b.center.z - p[3]);
				}
			}
			std::string detail;
			if (auto* ts = g->AsTriShape()) {
				const auto& tr = ts->GetTrishapeRuntimeData();
				detail = std::format(" {} tris {} verts", tr.triangleCount, tr.vertexCount);
			}
			if (auto* ap = g->GetGeometryRuntimeData().alphaProperty.get()) {
				detail += std::format(" alpha {:#06x} threshold {}", static_cast<unsigned>(ap->alphaFlags), static_cast<unsigned>(ap->alphaThreshold));
			} else {
				detail += " no alpha property";
			}
			if (auto* lsp = netimmerse_cast<RE::BSLightingShaderProperty*>(sp)) {
				if (auto* mat = static_cast<RE::BSLightingShaderMaterialBase*>(lsp->material); mat && mat->textureSet) {
					const char* tex = mat->textureSet->GetTexturePath(RE::BSTextureSet::Texture::kDiffuse);
					detail += std::format(" diffuse '{}'", tex ? tex : "");
				}
			}
			logger::info("bounds:   '{}' flags {:#x} chain {:#x} skinned {} shaderProp {:p} flags {:#x} bound ({:.0f},{:.0f},{:.0f}) r {:.0f};{} N.P-c per plane [near far l r t b]:{}", g->name.c_str() ? g->name.c_str() : "", g->flags.underlying(), chain, g->GetGeometryRuntimeData().skinInstance != nullptr,
				static_cast<void*>(sp), sp ? static_cast<std::uint64_t>(sp->flags.underlying()) : 0ull, b.center.x, b.center.y, b.center.z, b.radius, detail, dist);
		}
	}

	// A skinned mesh's bone matrices are cached per frame by the first draw that builds them (NiSkinInstance::frameID, G39-G41); a draw from another camera then reuses the matrices built
	// for the first camera. Zeroing the frame id of every skin instance of the actors near the player makes the next draw rebuild them for its own camera.
	void ResetSkinCaches()
	{
		auto* player = RE::PlayerCharacter::GetSingleton();
		auto* cell = player ? player->GetParentCell() : nullptr;
		if (!cell) {
			return;
		}
		ResetSkinsUnder(player->Get3D(false), 0);
		cell->ForEachReferenceInRange(player->GetPosition(), 3000.0f, [&](RE::TESObjectREFR* a_ref) {
			if (auto* a = a_ref ? a_ref->As<RE::Actor>() : nullptr; a && a != player && a->Is3DLoaded()) {
				ResetSkinsUnder(a->Get3D(), 0);
			}
			return RE::BSContainer::ForEachResult::kContinue;
		});
	}

	namespace
	{
		void CountSkinsUnder(RE::NiAVObject* a_o, int a_depth, std::uint32_t a_frame, int& a_total, int& a_cur, std::vector<std::string>* a_names)
		{
			if (!a_o || a_depth > 48) {
				return;
			}
			if (auto* g = a_o->AsGeometry()) {
				if (auto* si = g->GetGeometryRuntimeData().skinInstance.get()) {
					++a_total;
					if (si->frameID == a_frame) {
						++a_cur;
						if (a_names && a_names->size() < 8) {
							a_names->push_back(g->name.c_str() ? g->name.c_str() : "?");
						}
					}
				}
			}
			if (auto* n = a_o->AsNode()) {
				for (auto& c : n->GetChildren()) {
					CountSkinsUnder(c.get(), a_depth + 1, a_frame, a_total, a_cur, a_names);
				}
			}
		}
	}

	// G284: the first-person arms (and the bow / spell hands skinned to them) vanish while a portal pass runs. The bone matrices of a skin instance are cached per frame by the first draw that
	// builds them; when a virtual pass draws the first-person skeleton (its culling reaches the player's first-person root), the engine's own first-person draw later in the frame reuses matrices
	// built for the virtual camera. Zero the frame id of every skin instance under the first-person root after the passes, so the real draw rebuilds them for the real camera.
	int ResetFirstPersonSkins(int a_log)
	{
		auto* player = RE::PlayerCharacter::GetSingleton();
		auto* root = player ? player->Get3D(true) : nullptr;
		if (!root) {
			return 0;
		}
		if (a_log) {
			std::uint32_t present = 0;
			ReadSafeT(reinterpret_cast<const void*>(REL::ID(411489).address()), present);
			int                      total = 0, cur = 0;
			std::vector<std::string> names;
			CountSkinsUnder(root, 0, present, total, cur, &names);
			std::string list;
			for (const auto& n : names) {
				list += " '" + n + "'";
			}
			logger::info("fpskin: first-person root {:p}: {} skin instance(s), {} with frameID == the engine's frame counter {} (built this frame, before the engine's own draw):{}", static_cast<void*>(root), total, cur, present, list);
		}
		ResetSkinsUnder(root, 0);
		return 1;
	}

	void SetClipDraw(bool a_on)
	{
		g_clipDraw = a_on;
		logger::info("travel: the clip pass for straddling entities {}", a_on ? "ON" : "off");
	}

	void SetPassage(bool a_on)
	{
		if (a_on) {
			hole::InstallProjectileHook();  // projectiles' wall impacts inside the oval are not recorded (hole.cpp)
			InstallStreamHooks();           // a twin stream is put at the exit again after the engine's own update of it
			InstallSelfHitHooks();          // a projectile carried through a portal can hit whoever fired it
			InstallDecalHook();             // a decal the engine stamps in a portal's oval goes to the exit landing instead
		}
		g_passage = a_on;
		logger::info("travel: gradual passage (collision exception for rigid bodies, clip pass, carried when the centre crosses) {}", a_on ? "ON" : "off");
	}

	void Straddlers(std::vector<Straddler>& a_out)
	{
		a_out.clear();
		if (!g_clipDraw.load()) {
			return;
		}
		for (const auto& s : g_straddle) {
			auto ref = s.h.get();
			if (auto* root = ref ? ref->Get3D() : nullptr) {
				a_out.push_back({ root, s.entry, s.radius });
			}
		}
	}

	void NoteDroppedImpact(RE::Projectile* a_p, const RE::NiPoint3& a_hit)
	{
		if (!a_p || a_p->IsBeamProjectile()) {
			return;
		}
		std::lock_guard lk(g_dropMutex);
		if (g_dropped.size() > 256) {
			g_dropped.clear();
		}
		g_dropped[a_p->GetFormID()] = { a_hit, -1 };
	}

	void NoteImpact(RE::Projectile* a_p, RE::TESObjectREFR* a_ref)
	{
		if (!a_p || !a_ref || !a_ref->As<RE::Actor>()) {
			return;
		}
		// only on an actor: a wall impact at the exit is dropped and the flight goes on
		if (const auto back = RestoreShooter(a_p, a_ref); back && back.get().get() == a_ref) {
			ApplySelfSpell(a_p, a_ref->As<RE::Actor>());
		}
		{
			std::lock_guard lk(g_carriedMutex);
			if (const auto it = g_carriedFrom.find(a_p->GetFormID()); it != g_carriedFrom.end() && it->second.proj == a_p && std::chrono::steady_clock::now() - it->second.at < std::chrono::seconds(4)) {
				const auto& rd = a_p->GetProjectileRuntimeData();
				float       mag = 0.0f;
				if (auto* mi = rd.spell) {
					for (auto* e : mi->effects) {
						if (!e || !e->baseEffect) {
							continue;
						}
						if (g_pushLogged.load() < 24) {
							g_pushLogged.fetch_add(1);
							logger::info("travel: carried projectile {:08X} hit actor {:08X}: effect '{}' archetype {} magnitude {:.2f}", a_p->GetFormID(), a_ref->GetFormID(), e->baseEffect->GetName(), static_cast<int>(e->baseEffect->GetArchetype()), e->GetMagnitude());
						}
						if (e->baseEffect->HasArchetype(RE::EffectSetting::Archetype::kStagger)) {
							mag = (std::max)(mag, e->GetMagnitude());
						}
					}
				}
				if (mag > 0.0f) {
					g_pushes.push_back({ a_ref->As<RE::Actor>()->GetHandle(), it->second.src, mag, a_p->GetFormID() });
				}
			}
		}
		std::lock_guard lk(g_impMutex);
		if (g_engineImpact.size() > 512) {
			g_engineImpact.clear();
		}
		g_engineImpact[{ a_p->GetFormID(), a_ref->GetFormID() }] = NowSec();
	}

	bool TestNpcCentre(RE::NiPoint3& a_out)
	{
		auto npc = g_testNpc.get();
		if (!npc || !npc->Is3DLoaded()) {
			return false;
		}
		if (npc->IsDead()) {
			std::vector<RE::bhkRigidBody*> bodies;
			CollectBodies(npc->Get3D(), bodies, 0);
			if (!bodies.empty()) {
				NiPoint3 c{ 0, 0, 0 };
				for (auto* b : bodies) {
					c = c + BodyPoint(b, true);
				}
				a_out = c * (1.0f / static_cast<float>(bodies.size()));
				return true;
			}
		}
		a_out = npc->GetPosition() + NiPoint3{ 0, 0, ActorHalfHeight(npc.get()) };
		return true;
	}

	void SetTrapdoor(bool a_on)
	{
		g_trapdoor = a_on;
		logger::info("travel: trapdoor (living actors drop into floor portals) {}", a_on ? "ON" : "off");
	}

	bool KillNpc()
	{
		auto npc = g_testNpc.get();
		if (!npc) {
			return false;
		}
		npc->KillImpl(nullptr, 100000.0f, true, true);  // no attacker, no bounty; the ragdoll starts at once
		g_ragTrace = 120;
		logger::info("travel: killed actor {:08X}; dead {}", npc->GetFormID(), npc->IsDead());
		return true;
	}

	bool FlingNpc(float a_speed, int a_colour)
	{
		auto npc = g_testNpc.get();
		const portals::Portal pt = portals::Get(a_colour & 1);
		if (!npc || !pt.valid) {
			return false;
		}
		std::vector<RE::bhkRigidBody*> bodies;
		CollectBodies(npc->Get3D(), bodies, 0);
		NiPoint3 centroid{ 0, 0, 0 };
		for (auto* b : bodies) {
			centroid = centroid + BodyPoint(b, true);
		}
		if (bodies.empty()) {
			logger::warn("travel: the actor has no ragdoll bodies yet");
			return false;
		}
		centroid = centroid * (1.0f / static_cast<float>(bodies.size()));
		const NiPoint3 d = Norm(pt.c - centroid);
		const float    s = RE::bhkWorld::GetWorldScale();
		for (auto* b : bodies) {
			b->SetLinearVelocity(RE::hkVector4(d.x * a_speed * s, d.y * a_speed * s, d.z * a_speed * s, 0.0f));
		}
		g_ragTrace = 240;
		logger::info("travel: flung {} ragdoll bodies at portal {} at {:.0f} units/s (centroid ({:.1f},{:.1f},{:.1f}))", bodies.size(), a_colour & 1, a_speed, centroid.x, centroid.y, centroid.z);
		return true;
	}

	bool SpawnNpc(const std::string& a_name)
	{
		auto*          player = RE::PlayerCharacter::GetSingleton();
		world::CamInfo cam;
		if (!player || !world::CameraFull(cam)) {
			return false;
		}
		// a_name may be a comma-separated list of candidates (the first one that exists is spawned)
		RE::TESNPC* npc = nullptr;
		std::istringstream names(a_name);
		for (std::string cand; !npc && std::getline(names, cand, ',');) {
			std::string want = cand;
			std::replace(want.begin(), want.end(), '_', ' ');
			std::transform(want.begin(), want.end(), want.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
			for (auto* n : RE::TESDataHandler::GetSingleton()->GetFormArray<RE::TESNPC>()) {
				std::string nm = n && n->GetName() ? n->GetName() : "";
				std::transform(nm.begin(), nm.end(), nm.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
				if (nm == want) {
					npc = n;
					break;
				}
			}
		}
		logger::info("travel: spawn actor '{}': {:p}", a_name, static_cast<void*>(npc));
		if (!npc) {
			return false;
		}
		auto ref = player->PlaceObjectAtMe(npc, false);
		auto* a = ref ? ref->As<RE::Actor>() : nullptr;
		if (!a) {
			return false;
		}
		g_driveColour = -1;
		if (a->IsDisabled()) {
			logger::info("travel: the spawned actor starts disabled: enabling it");
			a->Enable(false);
		}
		const NiPoint3 fwd = Norm(NiPoint3{ cam.fwd.x, cam.fwd.y, 0.0f });
		a->SetPosition(player->GetPosition() + fwd * 120.0f, true);
		g_testNpc = a->GetHandle();
		{
			auto*          cc = a->GetCharController();
			std::uintptr_t vt = 0;
			if (cc) {
				ReadSafeT(cc, vt);
			}
			const char* kind = !cc ? "none (yet)" : vt == RE::VTABLE_bhkCharProxyController[1].address() ? "proxy" : vt == RE::VTABLE_bhkCharRigidBodyController[0].address() ? "rigid body" : "other";
			const NiPoint3 bmin = a->GetBoundMin(), bmax = a->GetBoundMax();
			logger::info("travel: the spawned actor's controller: {}; name '{}', bound ({:.0f},{:.0f},{:.0f}) to ({:.0f},{:.0f},{:.0f})", kind, a->GetName() ? a->GetName() : "", bmin.x, bmin.y, bmin.z, bmax.x, bmax.y, bmax.z);
		}
		logger::info("travel: spawned actor {:08X} '{}' at ({:.1f},{:.1f},{:.1f}); persistent {} (form flags {:#x}: {} temporary), initially disabled {}", a->GetFormID(), a->GetName() ? a->GetName() : "", a->GetPosition().x, a->GetPosition().y, a->GetPosition().z, a->IsPersistent(), a->GetFormFlags(),
			(a->GetFormFlags() & (1u << 14)) ? "marked" : "not marked", a->IsInitiallyDisabled());
		return true;
	}

	void LogNpc()
	{
		auto npc = g_testNpc.get();
		if (!npc) {
			logger::info("travel: npc info: no test actor");
			return;
		}
		auto*          cc = npc->GetCharController();
		std::uintptr_t vt = 0;
		if (cc) {
			ReadSafeT(cc, vt);
		}
		const char* kind = !cc ? "none" : vt == RE::VTABLE_bhkCharProxyController[1].address() ? "proxy" : vt == RE::VTABLE_bhkCharRigidBodyController[0].address() ? "rigid body" : "other";
		const auto  p = npc->GetPosition();
		logger::info("travel: npc info {:08X} '{}': 3D loaded {} disabled {} dead {} deleted {} controller {} parent cell {:p} (player's {:p}) pos ({:.1f},{:.1f},{:.1f}) process {:p} ai driven {}", npc->GetFormID(), npc->GetName() ? npc->GetName() : "",
			npc->Is3DLoaded(), npc->IsDisabled(), npc->IsDead(), npc->IsDeleted(), kind, static_cast<void*>(npc->GetParentCell()), static_cast<void*>(RE::PlayerCharacter::GetSingleton()->GetParentCell()), p.x, p.y, p.z,
			static_cast<void*>(npc->GetActorRuntimeData().currentProcess), npc->IsAIEnabled());
	}

	void LogNpcBodies()
	{
		auto npc = g_testNpc.get();
		auto* root = npc ? npc->Get3D() : nullptr;
		if (!root) {
			logger::info("travel: npc bodies: no 3D");
			return;
		}
		int nodes = 0, withColl = 0, withBody = 0, dynamic = 0;
		std::function<void(RE::NiAVObject*, int)> walk = [&](RE::NiAVObject* o, int d) {
			if (!o || d > 40) {
				return;
			}
			++nodes;
			if (auto* co = o->GetCollisionObject()) {
				++withColl;
				if (auto* b = co->GetRigidBody()) {
					++withBody;
					if (auto* rb = b->GetRigidBody(); rb && IsDynamic(rb)) {
						++dynamic;
					} else if (rb && withBody <= 4) {
						logger::info("travel: npc bodies: '{}' has a body of motion type {}", o->name.c_str() ? o->name.c_str() : "", static_cast<int>(rb->motion.type.get()));
					}
				}
			}
			if (auto* n = o->AsNode()) {
				for (auto& c : n->GetChildren()) {
					walk(c.get(), d + 1);
				}
			}
		};
		walk(root, 0);
		logger::info("travel: npc bodies of {:08X}: {} nodes, {} with a collision object, {} with a rigid body, {} dynamic; dead {}", npc->GetFormID(), nodes, withColl, withBody, dynamic, npc->IsDead());
	}

	// ---- test (G98): a control for an animal that the game disables on its own? Independent of the portals: a position, a walk toward a point, and a watch that logs every change of
	// the test actor's state with the callers of Disable() ----
	namespace
	{
		struct WalkTo
		{
			bool   on{ false };
			float  x{ 0 }, y{ 0 }, speed{ 0 };
			double left{ 0 };
		} g_walkTo;
		int  g_npcWatchLeft{ 0 };
		int  g_npcWatchTick{ 0 };
		bool g_snapValid{ false };
		bool g_snapLoaded{ false }, g_snapDisabled{ false }, g_snapDeleted{ false }, g_snapDead{ false }, g_snapProcess{ false }, g_snapHandle{ false };

		// TESObjectREFR::Disable (Character vtable slot 0x89) only queues the work: it tail-calls this routine (id 36978: the task queue's QueueDisable, queue in rcx, the reference in rdx),
		// which is also the point every other path that disables a reference goes through
		using DisableFn = void (*)(void*, RE::TESObjectREFR*);
		DisableFn          g_origDisable{ nullptr };
		const hooks::Spec  kQueueDisable{ "TaskQueue::QueueDisable", 36978, { 0x48, 0x81, 0xEC, 0xA8, 0x00, 0x00, 0x00, 0xC7, 0x44, 0x24, 0x20, 0x50, 0x00, 0x00, 0x00, 0x48 } };

		void Hook_Disable(void* a_queue, RE::TESObjectREFR* a_this)
		{
			auto npc = g_testNpc.get();
			if (a_this && npc && static_cast<RE::TESObjectREFR*>(npc.get()) == a_this) {
				void*          frames[12]{};
				const USHORT   n = CaptureStackBackTrace(1, 12, frames, nullptr);
				const auto     base = REL::Module::get().base();
				std::string    s;
				for (USHORT i = 0; i < n; ++i) {
					const auto a = reinterpret_cast<std::uintptr_t>(frames[i]);
					s += std::format(" {:#x}", a >= base && a < base + 0x4000000 ? a - base : a);
				}
				logger::info("travel: Disable() called on the test actor {:08X} '{}'; callers (exe rva or raw address):{}", a_this->GetFormID(), a_this->GetName() ? a_this->GetName() : "", s);
			}
			g_origDisable(a_queue, a_this);
		}

		void InstallDisableTrace()
		{
			static bool done = false;
			if (done) {
				return;
			}
			done = true;
			if (hooks::Install(kQueueDisable, reinterpret_cast<void*>(&Hook_Disable), reinterpret_cast<void**>(&g_origDisable))) {
				logger::info("travel: the engine's queue-disable routine is traced: a disable of the test actor is logged with its callers");
			}
		}

		void TickTestNpc(double a_dt)
		{
			if (!g_walkTo.on && g_npcWatchLeft <= 0) {
				return;
			}
			auto  npc = g_testNpc.get();
			auto* player = RE::PlayerCharacter::GetSingleton();
			if (g_walkTo.on && npc && npc->Is3DLoaded()) {
				const NiPoint3 q = npc->GetPosition();
				const float    dx = g_walkTo.x - q.x, dy = g_walkTo.y - q.y;
				const float    l = std::sqrt(dx * dx + dy * dy);
				if (l > 1.0f) {
					const float step = (std::min)(g_walkTo.speed * static_cast<float>(a_dt), l);
					npc->SetPosition(NiPoint3{ q.x + dx / l * step, q.y + dy / l * step, q.z }, true);
				}
				g_walkTo.left -= a_dt;
				if (g_walkTo.left <= 0.0) {
					g_walkTo.on = false;
					logger::info("travel: the walk toward the point ended (actor at ({:.1f},{:.1f},{:.1f}))", npc->GetPosition().x, npc->GetPosition().y, npc->GetPosition().z);
				}
			}
			if (g_npcWatchLeft > 0) {
				--g_npcWatchLeft;
				++g_npcWatchTick;
				const bool handle = npc != nullptr;
				const bool loaded = handle && npc->Is3DLoaded();
				const bool disabled = handle && npc->IsDisabled();
				const bool deleted = handle && npc->IsDeleted();
				const bool dead = handle && npc->IsDead();
				const bool process = handle && npc->GetActorRuntimeData().currentProcess != nullptr;
				const bool change = !g_snapValid || handle != g_snapHandle || loaded != g_snapLoaded || disabled != g_snapDisabled || deleted != g_snapDeleted || dead != g_snapDead || process != g_snapProcess;
				if (change || (g_npcWatchTick % 60) == 0) {
					const NiPoint3 p = handle ? npc->GetPosition() : NiPoint3{ 0, 0, 0 };
					const NiPoint3 pp = player ? player->GetPosition() : NiPoint3{ 0, 0, 0 };
					logger::info("travel: watch{}: actor handle {} 3D loaded {} disabled {} deleted {} dead {} persistent {} formFlags {:#x} process {} same cell as the player {} at ({:.1f},{:.1f},{:.1f}) distance from the player {:.0f} 3D root {:p}", change ? " CHANGE" : "", handle, loaded, disabled,
						deleted, dead, handle && npc->IsPersistent(), handle ? npc->GetFormFlags() : 0u, process, handle && player && npc->GetParentCell() == player->GetParentCell(), p.x, p.y, p.z, std::sqrt((p.x - pp.x) * (p.x - pp.x) + (p.y - pp.y) * (p.y - pp.y) + (p.z - pp.z) * (p.z - pp.z)),
						handle ? static_cast<void*>(npc->Get3D()) : nullptr);
				}
				g_snapValid = true;
				g_snapHandle = handle;
				g_snapLoaded = loaded;
				g_snapDisabled = disabled;
				g_snapDeleted = deleted;
				g_snapDead = dead;
				g_snapProcess = process;
			}
		}
	}

	// a test teleport is not a movement: forget the actor's tracks (the sweep would read the jump as a velocity of tens of thousands of units a second toward the portal and carry the actor
	// through it, G109) and keep the sweep off it for a moment
	void ForgetActorTracks(RE::Actor* a_a)
	{
		const std::uint32_t id = a_a->GetFormID();
		g_actorPrev.erase(id);
		g_prevC.erase(id);
		g_actorCooldown[id] = 30;
	}

	bool SetNpcPos(float a_x, float a_y, float a_z, float a_yawDeg)
	{
		auto npc = g_testNpc.get();
		if (!npc) {
			return false;
		}
		npc->SetPosition(NiPoint3{ a_x, a_y, a_z }, true);
		npc->SetHeading(a_yawDeg * 0.0174532925f);
		ForgetActorTracks(npc.get());
		logger::info("travel: put actor {:08X} at ({:.1f},{:.1f},{:.1f}) facing {:.1f} degrees", npc->GetFormID(), a_x, a_y, a_z, a_yawDeg);
		return true;
	}

	void WalkNpcTo(float a_x, float a_y, float a_speed, float a_seconds)
	{
		g_walkTo = { true, a_x, a_y, a_speed, a_seconds };
		logger::info("travel: walking the test actor toward ({:.1f},{:.1f}) at {:.0f} units/s for {:.1f} s", a_x, a_y, a_speed, a_seconds);
	}

	void WatchNpc(float a_seconds)
	{
		InstallDisableTrace();
		g_npcWatchLeft = static_cast<int>(a_seconds * 60.0f);
		g_npcWatchTick = 0;
		g_snapValid = false;
		logger::info("travel: watching the test actor for {:.0f} s (every change of its state is logged, and every second)", a_seconds);
	}

	// test (G109): hold the spawned actor in place for spell tests: restrained (it cannot move or fight) with a large health pool so that a few casts do not kill it
	// test: a console command run on the spawned actor (setrestrained 0, kill, ...)
	bool NpcConsole(const std::string& a_cmd)
	{
		auto  npc = g_testNpc.get();
		auto* factory = RE::IFormFactory::GetConcreteFormFactoryByType<RE::Script>();
		if (!npc || !factory) {
			return false;
		}
		if (auto* script = factory->Create()) {
			script->SetCommand(a_cmd);
			script->CompileAndRun(npc.get());
			delete script;
		}
		logger::info("travel: actor {:08X}: '{}'", npc->GetFormID(), a_cmd);
		return true;
	}

	bool HoldNpc()
	{
		auto npc = g_testNpc.get();
		auto* factory = RE::IFormFactory::GetConcreteFormFactoryByType<RE::Script>();
		if (!npc || !factory) {
			return false;
		}
		for (const char* cmd : { "modav health 1000", "restoreav health 1000", "setrestrained 1" }) {
			if (auto* script = factory->Create()) {
				script->SetCommand(cmd);
				script->CompileAndRun(npc.get());
				delete script;
			}
		}
		logger::info("travel: actor {:08X} held: restrained, health {:.0f}", npc->GetFormID(), npc->AsActorValueOwner()->GetActorValue(RE::ActorValue::kHealth));
		return true;
	}

	// test (G109): the spawned actor's health and the magic effects active on it (a hit by a spell shows as health lost and effects with their magnitudes)
	void LogNpcHealth(const std::string& a_tag)
	{
		auto npc = g_testNpc.get();
		if (!npc) {
			logger::warn("travel: no test actor for the health log '{}'", a_tag);
			return;
		}
		auto*       avo = npc->AsActorValueOwner();
		std::string fx;
		int         count = 0;
		if (auto* mt = npc->GetMagicTarget()) {
			if (auto* list = mt->GetActiveEffectList()) {
				for (auto* e : *list) {
					if (!e) {
						continue;
					}
					const auto* base = e->GetBaseObject();
					fx += std::format("{}'{}' magnitude {:.1f} elapsed {:.1f}s", fx.empty() ? "" : "; ", base && base->GetName() ? base->GetName() : "?", e->magnitude, e->elapsedSeconds);
					++count;
				}
			}
		}
		const NiPoint3 p = npc->GetPosition();
		logger::info("travel: health [{}]: actor {:08X} '{}' health {:.1f} (permanent {:.1f}), dead {}, at ({:.1f},{:.1f},{:.1f}); {} active effect(s): {}", a_tag, npc->GetFormID(), npc->GetName() ? npc->GetName() : "", avo->GetActorValue(RE::ActorValue::kHealth),
			avo->GetPermanentActorValue(RE::ActorValue::kHealth), npc->IsDead(), p.x, p.y, p.z, count, fx);
	}

	// test (G110): the spawned actor on the cast's own ray after the exit: the camera's view ray through the oval it enters, continued from the other portal, a_dist units in front of that
	// exit's plane, facing the exit (the G110 guard stood 14 to 26 units beside the twin's ray and the Fireball and the Flames missed it)
	bool PutNpcOnRay(float a_dist)
	{
		auto           npc = g_testNpc.get();
		world::CamInfo cam;
		if (!npc || !world::CameraFull(cam)) {
			return false;
		}
		const portals::Portal p[2] = { portals::Get(0), portals::Get(1) };
		for (int i = 0; i < 2; ++i) {
			const portals::Portal& A = p[i];
			const portals::Portal& B = p[1 - i];
			if (!A.valid || !B.valid) {
				continue;
			}
			const float dn = Dot(cam.fwd, A.n);
			if (dn > -0.05f) {
				continue;
			}
			const float    t = Dot(A.c - cam.pos, A.n) / dn;
			const NiPoint3 H = cam.pos + cam.fwd * t;
			const NiPoint3 dd = H - A.c;
			if (t <= 0.0f || Dot(dd, A.r) * Dot(dd, A.r) / (A.hw * A.hw) + Dot(dd, A.u) * Dot(dd, A.u) / (A.hh * A.hh) > 1.0f) {
				continue;
			}
			const NiPoint3 H2 = MapPoint(A, B, H);
			const NiPoint3 D2 = Norm(MapDir(A, B, cam.fwd));
			const float    along = a_dist / (std::max)(Dot(D2, B.n), 0.2f);
			NiPoint3       at = H2 + D2 * along;
			float          z = at.z - 70.0f;
			if (const auto ray = world::Raycast(NiPoint3{ at.x, at.y, at.z + 40.0f }, NiPoint3{ 0, 0, -1 }, 600.0f, RE::COL_LAYER::kLOS); ray.hit) {
				z = at.z + 40.0f - ray.dist + 1.0f;
			}
			npc->SetPosition(NiPoint3{ at.x, at.y, z }, true);
			npc->SetHeading(std::atan2(-D2.x, -D2.y));
			ForgetActorTracks(npc.get());
			logger::info("travel: put actor {:08X} on the cast's ray after portal {}: ray through the oval at ({:.1f},{:.1f},{:.1f}) -> exit ({:.1f},{:.1f},{:.1f}) direction ({:.3f},{:.3f},{:.3f}), {:.0f} units in front of the exit plane: feet at ({:.1f},{:.1f},{:.1f}), facing the exit",
				npc->GetFormID(), B.colour, H.x, H.y, H.z, H2.x, H2.y, H2.z, D2.x, D2.y, D2.z, a_dist, at.x, at.y, z);
			return true;
		}
		logger::warn("travel: the view ray goes through no portal's oval: the actor was not put");
		return false;
	}

	bool PutNpc(int a_colour, float a_dist, float a_yawDeg)
	{
		auto                  npc = g_testNpc.get();
		const portals::Portal pt = portals::Get(a_colour & 1);
		if (!npc || !pt.valid) {
			return false;
		}
		const NiPoint3 at = pt.c + pt.n * a_dist;
		float          z = at.z - 70.0f;
		if (const auto ray = world::Raycast(NiPoint3{ at.x, at.y, at.z + 40.0f }, NiPoint3{ 0, 0, -1 }, 600.0f, RE::COL_LAYER::kLOS); ray.hit) {
			z = at.z + 40.0f - ray.dist + 1.0f;
		}
		npc->SetPosition(NiPoint3{ at.x, at.y, z }, true);
		ForgetActorTracks(npc.get());
		npc->SetHeading(std::atan2(-pt.n.x, -pt.n.y) + a_yawDeg * 0.0174532925f);
		logger::info("travel: put actor {:08X} {:.0f} units in front of portal {} at ({:.1f},{:.1f},{:.1f}), facing it (turned {:.0f} degrees)", npc->GetFormID(), a_dist, a_colour & 1, at.x, at.y, z, a_yawDeg);
		return true;
	}

	bool DriveNpc(float a_speed, int a_colour)
	{
		auto npc = g_testNpc.get();
		if (!npc) {
			logger::warn("travel: no test actor to drive");
			return false;
		}
		g_driveSpeed = a_speed;
		g_driveColour = a_colour & 1;
		logger::info("travel: driving actor {:08X} at {:.0f} units/s toward portal {}", npc->GetFormID(), a_speed, a_colour & 1);
		return true;
	}

	bool Throw(const std::string& a_what, float a_speed)
	{
		auto*          player = RE::PlayerCharacter::GetSingleton();
		world::CamInfo cam;
		if (!player || !world::CameraFull(cam)) {
			return false;
		}
		std::uint32_t id = 0x0005AD9E;  // iron ingot
		if (a_what == "apple") {
			id = 0x00064B3F;
		} else if (a_what != "ingot") {
			id = static_cast<std::uint32_t>(std::stoul(a_what, nullptr, 16));
		}
		auto* base = RE::TESForm::LookupByID<RE::TESBoundObject>(id);
		logger::info("travel: throw '{}' ({:08X}): base {:p} '{}'", a_what, id, static_cast<void*>(base), base && base->GetName() ? base->GetName() : "");
		if (!base) {
			return false;
		}
		auto ref = player->PlaceObjectAtMe(base, false);
		if (!ref) {
			return false;
		}
		const NiPoint3 at = cam.pos + cam.fwd * 70.0f;
		ref->SetPosition(at);
		g_pending.push_back({ ref->CreateRefHandle(), cam.fwd, a_speed, 0 });
		g_lastThrown = ref->CreateRefHandle();
		logger::info("travel: spawned {:08X} at ({:.1f},{:.1f},{:.1f}), will be thrown along ({:.3f},{:.3f},{:.3f})", ref->GetFormID(), at.x, at.y, at.z, cam.fwd.x, cam.fwd.y, cam.fwd.z);
		return true;
	}

	bool DriveProp(float a_speed, int a_colour, float a_seconds, float a_startDist)
	{
		auto ref = g_lastThrown.get();
		const portals::Portal pt = portals::Get(a_colour & 1);
		if (!ref || !pt.valid) {
			return false;
		}
		g_drive.h = g_lastThrown;
		g_drive.id = ref->GetFormID();
		if (a_startDist > 0.0f) {
			g_drive.start = pt.c + pt.n * a_startDist;
			g_drive.place = true;
			g_drive.dir = pt.n * -1.0f;
		} else {
			g_drive.place = false;
			g_drive.dir = Norm(pt.c - ref->GetPosition());
		}
		g_drive.speed = a_speed;
		g_drive.frames = static_cast<int>(a_seconds * 60.0f);
		g_drive.aligned = false;
		logger::info("travel: pushing prop {:08X} at {:.0f} units/s toward portal {} along ({:.3f},{:.3f},{:.3f}) for {:.1f} s", g_drive.id, a_speed, a_colour & 1, g_drive.dir.x, g_drive.dir.y, g_drive.dir.z, a_seconds);
		return true;
	}

	void AutoShot(const std::string& a_tag, int a_count, int a_every)
	{
		g_autoTag = a_tag;
		g_autoLeft = a_count;
		g_autoCount = 0;
		g_autoEvery = (std::max)(a_every, 1);
		logger::info("travel: up to {} screenshot(s) '{}' every {} frame(s) while something straddles a portal's plane", a_count, a_tag, g_autoEvery);
	}

	void SetTwinPick(bool a_on, float a_startOut)
	{
		g_twinPickOn = a_on;
		g_twinStartOut = a_startOut;
		logger::info("travel: a flame or beam twin runs its own target pick: {}; it starts {:.0f} units out of the exit plane", a_on ? "ON" : "off", a_startOut);
	}

	void SetTwinMode(const std::string& a_mode)
	{
		g_twinMode = a_mode == "shatter" ? 1 : a_mode == "both" ? 2 : 0;
		logger::info("travel: a beam or flame twin avoids the engine's reuse by: {}", g_twinMode.load() == 1 ? "the launch data's new-projectile flag (same casting source)" : g_twinMode.load() == 2 ? "the other hand's casting source and the flag" : "the other hand's casting source");
	}

	// test: put a named spell in the player's right hand and draw, so that a pressed attack button is the game's own cast (as in play: magicka, concentration while held)
	bool EquipSpell(const std::string& a_name)
	{
		auto* player = RE::PlayerCharacter::GetSingleton();
		if (!player) {
			return false;
		}
		std::string want = a_name;
		std::transform(want.begin(), want.end(), want.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		want.erase(std::remove(want.begin(), want.end(), ' '), want.end());
		// the data holds several spells of one name (the player's and the versions made for NPCs): the player's has a skill and comes from the base game
		RE::SpellItem* spell = nullptr;
		int            best = -1;
		for (auto* sp : RE::TESDataHandler::GetSingleton()->GetFormArray<RE::SpellItem>()) {
			std::string n = sp && sp->GetName() ? sp->GetName() : "";
			std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
			n.erase(std::remove(n.begin(), n.end(), ' '), n.end());
			if (n != want || sp->GetSpellType() != RE::MagicSystem::SpellType::kSpell) {
				continue;
			}
			const int score = (sp->GetAssociatedSkill() != RE::ActorValue::kNone ? 4 : 0) + (sp->GetFormID() < 0x01000000 ? 2 : 0) + (sp->data.castingPerk ? 1 : 0);
			if (score > best) {
				best = score;
				spell = sp;
			}
		}
		if (!spell) {
			logger::warn("travel: no spell named '{}'", a_name);
			return false;
		}
		logger::info("travel: spell '{}' -> {:08X} (score {}, casting type {}, delivery {})", a_name, spell->GetFormID(), best, static_cast<int>(spell->GetCastingType()), static_cast<int>(spell->GetDelivery()));
		if (!player->HasSpell(spell)) {
			player->AddSpell(spell);
		}
		auto* slot = RE::TESForm::LookupByID<RE::BGSEquipSlot>(0x00013F42);  // right hand
		if (auto* em = RE::ActorEquipManager::GetSingleton()) {
			em->EquipSpell(player, spell, slot);
		}
		if (!player->AsActorState()->IsWeaponDrawn()) {
			player->DrawWeaponMagicHands(true);
		}
		// test only: a cast that fails for want of magicka would read as "the game's cast produced nothing" (the spells of one run cost about 600 together)
		if (auto* avo = player->AsActorValueOwner()) {
			const float before = avo->GetActorValue(RE::ActorValue::kMagicka);
			avo->RestoreActorValue(RE::ActorValue::kMagicka, 1000.0f);
			logger::info("travel: magicka {:.0f} -> {:.0f} (test refill before the cast of '{}')", before, avo->GetActorValue(RE::ActorValue::kMagicka), a_name);
		}
		auto* eq = player->GetActorRuntimeData().selectedSpells[RE::Actor::SlotTypes::kRightHand];
		logger::info("travel: equipped spell '{}' ({:08X}) in the right hand; selected {:p}; weapon drawn {}", a_name, spell->GetFormID(), static_cast<void*>(eq), player->AsActorState()->IsWeaponDrawn());
		return true;
	}

	// test: a REAL cast by the player of a named spell (the caster's own state machine: concentration spells need the cast held), finished after a_seconds
	bool Cast(const std::string& a_name, float a_seconds)
	{
		auto* player = RE::PlayerCharacter::GetSingleton();
		if (!player) {
			return false;
		}
		std::string want = a_name;
		std::transform(want.begin(), want.end(), want.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
		RE::SpellItem* spell = nullptr;
		for (auto* sp : RE::TESDataHandler::GetSingleton()->GetFormArray<RE::SpellItem>()) {
			std::string n = sp && sp->GetName() ? sp->GetName() : "";
			std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
			n.erase(std::remove(n.begin(), n.end(), ' '), n.end());
			if (n == want) {
				spell = sp;
				break;
			}
		}
		auto* caster = player->GetMagicCaster(g_castSource);
		logger::info("travel: cast '{}': spell {:p} caster {:p}", a_name, static_cast<void*>(spell), static_cast<void*>(caster));
		if (!spell || !caster) {
			return false;
		}
		logger::info("travel: cast '{}': casting type {} delivery {}", a_name, static_cast<int>(spell->GetCastingType()), static_cast<int>(spell->GetDelivery()));
		caster->SetCurrentSpell(spell);
		// the caster's state machine does not start for a player who has no spell in hand (state stays 0, G73); the immediate cast is what a script's Spell.Cast uses
		caster->CastSpellImmediate(spell, false, nullptr, 1.0f, false, 0.0f, player);
		g_castLeft = a_seconds;
		logger::info("travel: cast requested, caster state {}", static_cast<std::uint32_t>(caster->state.get()));
		return true;
	}

	// test: shoot an arrow or a spell from the player along the view direction (a_what: "arrow", "firebolt", "icespike" ...)
	bool Shoot(const std::string& a_what, float a_speed)
	{
		auto* player = RE::PlayerCharacter::GetSingleton();
		world::CamInfo cam;
		if (!player || !world::CameraFull(cam)) {
			return false;
		}
		const NiPoint3 origin = cam.pos + cam.fwd * 40.0f;
		RE::Projectile::ProjectileRot rot{ -std::asin(std::clamp(cam.fwd.z, -1.0f, 1.0f)), std::atan2(cam.fwd.x, cam.fwd.y) };
		RE::ProjectileHandle          h;
		if (a_what == "arrow") {
			auto* ammo = RE::TESForm::LookupByID<RE::TESAmmo>(0x0001397D);
			auto* bow = RE::TESForm::LookupByID<RE::TESObjectWEAP>(0x00013985);
			logger::info("travel: shoot arrow: ammo {:p} '{}', bow {:p} '{}'", static_cast<void*>(ammo), ammo ? ammo->GetName() : "", static_cast<void*>(bow), bow ? bow->GetName() : "");
			if (!ammo || !bow) {
				return false;
			}
			// a fully drawn bow: power 1 (LaunchArrow's own default is a barely drawn one, 530 units/s)
			RE::Projectile::LaunchData data(player, origin, rot, ammo, bow);
			data.power = 1.0f;
			RE::Projectile::Launch(&h, data);
			// the engine's launch speed here is only 530 units/s whatever the power (the draw is not simulated): give the arrow a bow's speed
			if (auto p = h.get(); p) {
				g_boost[p->GetFormID()] = a_speed;
			}
		} else {
			RE::SpellItem* spell = nullptr;
			std::string    want = a_what;
			std::transform(want.begin(), want.end(), want.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
			for (auto* s : RE::TESDataHandler::GetSingleton()->GetFormArray<RE::SpellItem>()) {
				std::string n = s && s->GetName() ? s->GetName() : "";
				std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
				n.erase(std::remove(n.begin(), n.end(), ' '), n.end());
				if (want == "grenade") {
					// the first spell whose effect uses a grenade-type (lobbed) projectile
					for (auto* e : s ? s->effects : RE::BSTArray<RE::Effect*>{}) {
						if (e && e->baseEffect && e->baseEffect->data.projectileBase && e->baseEffect->data.projectileBase->IsGrenade()) {
							logger::info("travel: grenade-type spell candidate '{}' ({:08X})", s->GetName() ? s->GetName() : "", s->GetFormID());
							if (!spell) {
								spell = s;
							}
						}
					}
					continue;
				}
				if (n == want) {
					spell = s;
					break;
				}
			}
			logger::info("travel: shoot spell '{}': {:p}", a_what, static_cast<void*>(spell));
			if (!spell) {
				return false;
			}
			RE::Projectile::LaunchSpell(&h, player, spell, origin, rot);
			// a slow-projectile test: give the spell's projectile that speed once it is initialised (a spell flies at its own speed otherwise; a_speed >= 1000 means "leave it")
			if (a_speed < 1000.0f) {
				if (auto p = h.get(); p) {
					g_boost[p->GetFormID()] = a_speed;
				}
			}
		}
		logger::info("travel: shot '{}' from ({:.1f},{:.1f},{:.1f}) direction ({:.3f},{:.3f},{:.3f})", a_what, origin.x, origin.y, origin.z, cam.fwd.x, cam.fwd.y, cam.fwd.z);
		return true;
	}
}
