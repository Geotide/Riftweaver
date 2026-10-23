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

// Through-portal killcam (2026-10-22, backlog new feature): the game's own arrow killcam decides at release, from the projectile's straight path, whether the shot
// will kill (G394/G395: through a portal that path ends at the wall, so it never starts). When a projectile the player fired is carried through a portal and its path out of
// the exit meets a living actor the hit should kill, time slows and the camera rides behind the projectile out of the exit to the hit, holds on the target, then returns.
namespace killcam
{
	namespace
	{
		using NiPoint3 = RE::NiPoint3;

		std::atomic<bool> g_enabled{ true };
		std::mutex        g_mutex;
		bool              g_active{ false };
		bool              g_ownFree{ false };  // we switched the free camera on (and switch it off again)
		RE::ObjectRefHandle g_proj;
		RE::ActorHandle     g_target;
		NiPoint3            g_camPos{}, g_lastPos{}, g_lastDir{ 0, 1, 0 };
		float               g_yaw{ 0 }, g_pitch{ 0 };
		bool                g_holding{ false };
		std::chrono::steady_clock::time_point g_start, g_holdStart;

		constexpr float kSlow = 0.3f;     // global time multiplier while the killcam runs
		constexpr float kBack = 75.0f;    // camera this far behind the projectile
		constexpr float kUp = 12.0f;      // and this far above its line
		constexpr double kHold = 1.1;     // seconds (real) on the target after the hit
		constexpr double kMax = 4.0;      // seconds (real) at most

		float Len(const NiPoint3& a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
		NiPoint3 Norm(const NiPoint3& a)
		{
			const float l = Len(a);
			return l > 1e-6f ? a * (1.0f / l) : NiPoint3{ 0, 1, 0 };
		}

		RE::FreeCameraState* FreeState()
		{
			auto* pc = RE::PlayerCamera::GetSingleton();
			if (!pc || !pc->IsInFreeCameraMode()) {
				return nullptr;
			}
			return skyrim_cast<RE::FreeCameraState*>(pc->currentState.get());
		}

		void Aim(const NiPoint3& a_from, const NiPoint3& a_at)
		{
			const NiPoint3 d = a_at - a_from;
			g_yaw = std::atan2(d.x, d.y);
			g_pitch = -std::atan2(d.z, std::sqrt(d.x * d.x + d.y * d.y));
		}

		void Stop(const char* a_why)
		{
			if (auto* t = RE::BSTimer::GetSingleton()) {
				t->SetGlobalTimeMultiplier(1.0f, true);
			}
			if (g_ownFree) {
				if (auto* pc = RE::PlayerCamera::GetSingleton(); pc && pc->IsInFreeCameraMode()) {
					pc->ToggleFreeCameraMode(false);
				}
			}
			const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - g_start).count();
			logger::info("killcam: ended after {:.2f} s ({})", secs, a_why);
			g_active = false;
			g_ownFree = false;
			g_holding = false;
		}

		// what the projectile does to health on a hit: the bow's and the arrow's damage (scaled by the draw), or the spell's instant health damage
		float Damage(RE::Projectile* a_p)
		{
			const auto& rd = a_p->GetProjectileRuntimeData();
			float       dmg = 0.0f;
			if (rd.weaponSource || rd.ammoSource) {
				dmg = (rd.weaponSource ? static_cast<float>(rd.weaponSource->attackDamage) : 0.0f) + (rd.ammoSource ? rd.ammoSource->data.damage : 0.0f);
				dmg *= std::clamp(rd.power, 0.35f, 1.0f);
			}
			if (auto* sp = rd.spell) {
				for (auto* e : sp->effects) {
					if (e && e->baseEffect && e->baseEffect->data.primaryAV == RE::ActorValue::kHealth && e->baseEffect->IsDetrimental()) {
						dmg += e->GetMagnitude();
					}
				}
			}
			return dmg;
		}
	}

	void SetEnabled(bool a_on)
	{
		g_enabled = a_on;
		logger::info("killcam: through-portal killcam {}", a_on ? "ON" : "off");
	}

	bool Active()
	{
		std::lock_guard lk(g_mutex);
		return g_active;
	}

	void OnCarried(RE::Projectile* a_p, const NiPoint3& a_exitPos, const NiPoint3& a_dir)
	{
		if (!g_enabled.load() || !a_p) {
			return;
		}
		auto* player = RE::PlayerCharacter::GetSingleton();
		const auto& rd = a_p->GetProjectileRuntimeData();
		auto        shooter = rd.shooter.get();
		if (!player || !shooter || shooter.get() != player) {
			return;
		}
		std::lock_guard lk(g_mutex);
		if (g_active) {
			return;
		}
		// the nearest living actor whose body the path out of the exit passes through
		const NiPoint3 d = Norm(a_dir);
		RE::Actor*     hit = nullptr;
		float          bestT = 2500.0f;
		if (auto* cell = player->GetParentCell()) {
			cell->ForEachReferenceInRange(a_exitPos, 2500.0f, [&](RE::TESObjectREFR* a_ref) {
				auto* a = a_ref ? a_ref->As<RE::Actor>() : nullptr;
				if (!a || a == player || a->IsDead() || a->IsDisabled() || !a->Is3DLoaded()) {
					return RE::BSContainer::ForEachResult::kContinue;
				}
				const NiPoint3 bmin = a->GetBoundMin(), bmax = a->GetBoundMax();
				const NiPoint3 c = a->GetPosition() + NiPoint3{ 0, 0, 0.5f * (bmax.z - bmin.z) };
				const float    t = (c - a_exitPos).x * d.x + (c - a_exitPos).y * d.y + (c - a_exitPos).z * d.z;
				if (t <= 0.0f || t >= bestT) {
					return RE::BSContainer::ForEachResult::kContinue;
				}
				const NiPoint3 q = a_exitPos + d * t;
				const float    r = 0.5f * (std::max)(bmax.x - bmin.x, bmax.y - bmin.y) + 12.0f;
				const float    dx = q.x - c.x, dy = q.y - c.y, dz = q.z - c.z;
				if (dx * dx + dy * dy < r * r && std::abs(dz) < 0.5f * (bmax.z - bmin.z) + 15.0f) {
					bestT = t;
					hit = a;
				}
				return RE::BSContainer::ForEachResult::kContinue;
			});
		}
		if (!hit) {
			return;
		}
		const float health = hit->AsActorValueOwner()->GetActorValue(RE::ActorValue::kHealth);
		const float dmg = Damage(a_p);
		if (dmg <= 0.0f || health > dmg) {
			logger::info("killcam: {:08X} '{}' is on the path {:.0f} units past the exit, health {:.1f} against about {:.1f} damage: no killcam", hit->GetFormID(), hit->GetName() ? hit->GetName() : "", bestT, health, dmg);
			return;
		}
		auto* pc = RE::PlayerCamera::GetSingleton();
		if (!pc) {
			return;
		}
		g_ownFree = !pc->IsInFreeCameraMode();
		if (g_ownFree) {
			pc->ToggleFreeCameraMode(false);
		}
		if (auto* t = RE::BSTimer::GetSingleton()) {
			t->SetGlobalTimeMultiplier(kSlow, true);
		}
		g_proj = a_p->GetHandle();
		g_target = hit->GetHandle();
		g_lastPos = a_exitPos;
		g_lastDir = d;
		g_camPos = a_exitPos - d * kBack + NiPoint3{ 0, 0, kUp };
		Aim(g_camPos, a_exitPos + d * 200.0f);
		g_holding = false;
		g_active = true;
		g_start = std::chrono::steady_clock::now();
		logger::info("killcam: started: projectile {:08X} out of the exit at ({:.1f},{:.1f},{:.1f}) toward {:08X} '{}' {:.0f} units on (health {:.1f}, about {:.1f} damage)", a_p->GetFormID(), a_exitPos.x, a_exitPos.y,
			a_exitPos.z, hit->GetFormID(), hit->GetName() ? hit->GetName() : "", bestT, health, dmg);
	}

	void Tick()
	{
		std::lock_guard lk(g_mutex);
		if (!g_active) {
			return;
		}
		const auto   now = std::chrono::steady_clock::now();
		const double age = std::chrono::duration<double>(now - g_start).count();
		if (age > kMax) {
			Stop("time limit");
			return;
		}
		auto* fs = FreeState();
		if (!fs) {
			Stop("free camera gone");
			return;
		}
		if (!g_holding) {
			auto ref = g_proj.get();
			auto* p = ref ? ref->As<RE::Projectile>() : nullptr;
			const bool alive = p && !p->GetProjectileRuntimeData().flags.any(RE::Projectile::Flags::kDestroyed, RE::Projectile::Flags::kProcessedImpacts);
			if (alive) {
				const auto& rd = p->GetProjectileRuntimeData();
				const NiPoint3 pos = p->GetPosition();
				const NiPoint3 dir = Len(rd.linearVelocity) > 10.0f ? Norm(rd.linearVelocity) : g_lastDir;
				g_lastPos = pos;
				g_lastDir = dir;
				g_camPos = pos - dir * kBack + NiPoint3{ 0, 0, kUp };
				Aim(g_camPos, pos + dir * 200.0f);
			} else {
				g_holding = true;
				g_holdStart = now;
				logger::info("killcam: the projectile hit after {:.2f} s; holding on the target", age);
			}
		}
		if (g_holding) {
			if (auto t = g_target.get()) {
				const NiPoint3 bmin = t->GetBoundMin(), bmax = t->GetBoundMax();
				Aim(g_camPos, t->GetPosition() + NiPoint3{ 0, 0, 0.5f * (bmax.z - bmin.z) });
			}
			if (std::chrono::duration<double>(now - g_holdStart).count() > kHold) {
				auto t = g_target.get();
				Stop(t && t->IsDead() ? "target dead" : "target alive");
				return;
			}
		}
		fs->translation = g_camPos;
		fs->rotation.x = g_pitch;
		fs->rotation.y = g_yaw;
	}
}
