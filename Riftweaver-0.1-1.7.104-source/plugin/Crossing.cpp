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

#include <xmmintrin.h>

// Walking through a portal. Once per frame (game thread, start of the frame function) the player's capsule centre is compared with the previous frame's:
// if it went from the front of a portal's plane to the back inside the oval, the player is carried through the pair: position, facing and velocity go
// through the same transform as the virtual camera (so the picture the player saw through the oval is exactly what they now stand in), and the fall
// bookkeeping is reset so a fall through a portal does not hurt.
namespace crossing
{
	namespace
	{
		using RE::NiPoint3;

		std::atomic<bool> g_enabled{ true };
		bool              g_havePrev{ false };
		NiPoint3          g_prev{};
		int               g_cooldown{ 0 };
		int               g_lastExit{ -1 };  // the portal the last crossing came out of: only crossing back through it waits for the cooldown (G380: a floor/ceiling loop at terminal speed reached the floor portal within the 8-frame cooldown, the crossing was skipped and the player fell through the hole into the void)
		std::atomic<int>  g_count{ 0 };
		float             g_sinkSpeed{ 0.0f };  // downward speed while falling into a floor portal (units/s)
		int               g_sinkLog{ 0 }, g_sinkTick{ 0 };
		float             g_sinkTime{ 0.0f };  // how long the player has been falling over a floor portal's oval  // the first falls are logged (a creeping fall shows as a centre that barely drops while the speed grows)
		constexpr float   kEyeUp = 56.0f;       // the camera sits this far above the player's capsule centre (feet + 64, eye at feet + 120)
		std::atomic<bool> g_mapEye{ false };    // test switch: map the camera (eye) exactly and time the crossing by whichever of eye and centre leads; default maps the capsule centre

		// Momentum after the exit. The game's controller recomputes its air velocity every step and bleeds the horizontal part away within a tenth of a
		// second (measured: the carried-over velocity (-1380, 215, -95) was (-80, 12, -218) 0.2 s later), so for a while after a fast crossing the horizontal
		// velocity is simply set again every frame (the engine keeps doing gravity and collision) until the player lands or runs into something.
		struct Flight
		{
			bool     active{ false };
			NiPoint3 vh{};           // horizontal velocity to keep (units/s)
			double   t{ 0 };
			double   stuck{ 0 };
			NiPoint3 lastPos{};
			float    apex{ -1e9f };  // highest feet z reached
		};
		Flight g_flight;
		std::atomic<int> g_flightMode{ 1 };  // 0 off, 1 re-set the horizontal velocity every frame

		// Camera roll after a crossing: the mapped camera's up vector against the upright camera the engine builds from the player's heading and pitch.
		struct Roll
		{
			std::atomic<bool> enabled{ true };
			double            duration{ 0.35 };
			std::string       shotTag;
			bool              pending{ false };  // a crossing happened; waiting for the engine's camera to show the new heading and pitch
			NiPoint3          upMapped{}, fwdMapped{};
			int               wait{ 0 };
			bool              active{ false };
			double            t{ 0 };
			float             theta0{ 0 };
			int               shotIndex{ 0 };
			int               logged{ 0 };
			float             lastTheta{ 0 };   // the roll written this frame
			bool              writeNow{ false };  // a roll is in progress: write it again at the later points of the frame
			NiPoint3          wroteF{}, wroteU{}, wroteR{}, origF{}, origU{}, origR{};  // what we wrote last frame, and the engine's basis it came from
			bool              haveWrote{ false };
		};
		Roll g_roll;

		float    Dot(const NiPoint3& a, const NiPoint3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
		NiPoint3 Norm(const NiPoint3& a)
		{
			const float l = std::sqrt(Dot(a, a));
			return l > 1e-6f ? a * (1.0f / l) : NiPoint3{ 1, 0, 0 };
		}

		NiPoint3 MapPoint(const portals::Portal& A, const portals::Portal& B, const NiPoint3& p)
		{
			const NiPoint3 d = p - A.c;
			const float    x = Dot(d, A.r), y = Dot(d, A.u), z = Dot(d, A.n);
			return B.c + B.r * (-x) + B.u * y + B.n * (-z);
		}
		NiPoint3 MapDir(const portals::Portal& A, const portals::Portal& B, const NiPoint3& v)
		{
			const float x = Dot(v, A.r), y = Dot(v, A.u), z = Dot(v, A.n);
			return B.r * (-x) + B.u * y + B.n * (-z);
		}

		void Teleport(RE::PlayerCharacter* a_p, const portals::Portal& A, const portals::Portal& B, const NiPoint3& a_centre, const NiPoint3& a_velocity)
		{
			constexpr float kEye = 64.0f;
			// The capsule CENTRE is mapped. (G20 tried mapping the eye exactly so the camera continues without a jump: for a floor exit the upright capsule's centre then
			// lands about 45 units under the plane, its feet beyond the 80 units in which the exception drops the surface's contacts, and the player fell through the
			// hole into the void (z -8877). Capsule safety wins; for floor and ceiling portals the camera jumps by up to 56 units at the teleport, as in Portal.)
			const bool      eyeMode = g_mapEye.load();
			NiPoint3        newCentre = eyeMode ? MapPoint(A, B, a_centre + NiPoint3{ 0, 0, kEyeUp }) - NiPoint3{ 0, 0, kEyeUp } : MapPoint(A, B, a_centre);
			// endless fall loop (a floor portal under a ceiling portal): falling straight through such a pair, the exit is eased toward the exit portal's centre (35 % of
			// the offset, at most 25 units a crossing). G379: two hand-placed portals were 8 units out of line, every loop moved the player 8 units toward the edge until
			// he dropped out; with the ease the loop settles in the middle and goes on
			if (std::abs(A.n.z) > 0.7f && std::abs(B.n.z) > 0.7f && std::sqrt(Dot(a_velocity, a_velocity)) > 100.0f && std::abs(Dot(Norm(a_velocity), A.n)) > 0.9f) {
				NiPoint3    off = newCentre - B.c;
				off = off - B.n * Dot(off, B.n);
				const float len = std::sqrt(Dot(off, off));
				if (len > 1.0f) {
					const float step = (std::min)(len * 0.35f, 25.0f);
					newCentre = newCentre - off * (step / len);
					logger::info("crossing: fall loop: exit eased {:.1f} units toward the exit portal's centre (offset was {:.1f})", step, len);
				}
			}
			const NiPoint3  newEye = newCentre + NiPoint3{ 0, 0, kEyeUp };
			const NiPoint3  newFeet = newCentre - NiPoint3{ 0, 0, kEye };
			world::CamInfo  cam;
			float           yawDeg = 0, pitchDeg = 0;
			if (world::CameraFull(cam)) {
				const NiPoint3 f = Norm(MapDir(A, B, cam.fwd));
				g_roll.upMapped = Norm(MapDir(A, B, cam.up));
				g_roll.fwdMapped = f;
				g_roll.pending = true;
				g_roll.wait = 0;
				g_roll.active = false;
				yawDeg = std::atan2(f.x, f.y) * 57.29578f;
				pitchDeg = -std::asin(std::clamp(f.z, -1.0f, 1.0f)) * 57.29578f;
			}
			auto*          cc = a_p->GetCharController();
			RE::hkVector4  oldV;
			if (cc) {
				cc->GetLinearVelocityImpl(oldV);
			}
			alignas(16) float ov[4];
			_mm_store_ps(ov, oldV.quad);
			const float s = RE::bhkWorld::GetWorldScale();
			// the controller's own velocity is the truth (m/s in Havok units -> game units/s); the position difference over a frame is only a fallback
			NiPoint3 vUse = a_velocity;
			if (cc && (ov[0] * ov[0] + ov[1] * ov[1] + ov[2] * ov[2]) > 1e-8f) {
				vUse = NiPoint3{ ov[0] / s, ov[1] / s, ov[2] / s };
			}
			const NiPoint3 v2 = MapDir(A, B, vUse);
			logger::info("crossing: portal {} -> {}: centre ({:.1f},{:.1f},{:.1f}) -> ({:.1f},{:.1f},{:.1f}); facing yaw {:.1f} pitch {:.1f}; velocity measured ({:.0f},{:.0f},{:.0f}) units/s -> ({:.0f},{:.0f},{:.0f}); controller velocity before ({:.0f},{:.0f},{:.0f}) units/s",
				A.colour, B.colour, a_centre.x, a_centre.y, a_centre.z, newCentre.x, newCentre.y, newCentre.z, yawDeg, pitchDeg, a_velocity.x, a_velocity.y, a_velocity.z, v2.x, v2.y, v2.z, ov[0] / s, ov[1] / s, ov[2] / s);
			shot::TriggerRing();
			portals::NoteCross(B.colour);
			a_p->SetPosition(newFeet, true);
			a_p->SetHeading(yawDeg * 0.017453292f);
			a_p->SetLooking(pitchDeg * 0.017453292f);
			if (cc) {
				const RE::hkVector4 hv(v2.x * s, v2.y * s, v2.z * s, 0.0f);
				cc->SetLinearVelocityImpl(hv);
				cc->outVelocity = hv;      // what the controller integrates this step
				cc->initialVelocity = hv;  // what an airborne state starts from
				cc->fallStartHeight = newFeet.z;  // a fall through a portal restarts here: no fall damage from before
				cc->fallTime = 0.0f;
			}
			logger::info("crossing: the eye now sits {:.1f} units in front of the exit surface (negative = behind it)", Dot(newEye - B.c, B.n));
			g_prev = newCentre;
			g_cooldown = 8;
			g_lastExit = B.colour;
			g_count.fetch_add(1);
			// a fast or airborne exit starts a flight (a walk does not: the player's own input carries it)
			const bool fast = std::sqrt(v2.x * v2.x + v2.y * v2.y) > 450.0f || std::fabs(v2.z) > 200.0f;
			if (g_flightMode.load() == 1 && fast) {
				g_flight = Flight{};
				g_flight.active = true;
				g_flight.vh = NiPoint3{ v2.x, v2.y, 0.0f };
				g_flight.lastPos = newFeet;
				g_flight.apex = newFeet.z;
				hole::SetFlight(true, v2.x, v2.y);
				logger::info("crossing: flight started, horizontal velocity ({:.0f},{:.0f}) units/s", v2.x, v2.y);
			}
		}
	}

	void SetEnabled(bool a_on)
	{
		g_enabled = a_on;
		logger::info("crossing: {}", a_on ? "ON" : "off");
	}

	int Count()
	{
		return g_count.load();
	}

	void SetMapEye(bool a_on)
	{
		g_mapEye = a_on;
		logger::info("crossing: mapping {}", a_on ? "the EYE exactly (trigger: whichever of eye and centre leads)" : "the capsule CENTRE (default)");
	}

	void SetRoll(bool a_on, double a_seconds, const std::string& a_shotTag)
	{
		g_roll.enabled = a_on;
		if (a_seconds > 0) {
			g_roll.duration = a_seconds;
		}
		g_roll.shotTag = a_shotTag;
		logger::info("roll: {} duration {:.2f} s shots '{}'", a_on ? "ON" : "off", g_roll.duration, a_shotTag);
	}

	namespace
	{
		using CamUpdate_t = void (*)(void*);
		CamUpdate_t       g_origCamUpdate{ nullptr };
		const hooks::Spec kCamUpdate{ "NiCamera update (id 70641)", 70641, { 0x48, 0x8B, 0xC4, 0x48, 0x81, 0xEC, 0xC8, 0x00, 0x00, 0x00, 0xF3, 0x0F, 0x10, 0x99, 0xA0, 0x00 } };

		// Rolls the camera's current (engine-built, upright) basis by the roll of this frame, unless it already holds it. Returns true when it wrote.
		bool RollInPlace(RE::NiCamera* a_cam, const char* a_where)
		{
			auto&    m = a_cam->world.rotate.entry;
			NiPoint3 f{ m[0][0], m[1][0], m[2][0] }, u{ m[0][1], m[1][1], m[2][1] }, r{ m[0][2], m[1][2], m[2][2] };
			auto     same = [](const NiPoint3& a, const NiPoint3& b) { return std::fabs(a.x - b.x) + std::fabs(a.y - b.y) + std::fabs(a.z - b.z) < 1e-6f; };
			if (same(f, g_roll.wroteF) && same(u, g_roll.wroteU) && same(r, g_roll.wroteR)) {
				return false;  // still holding our roll
			}
			f = Norm(f);
			u = Norm(u);
			r = Norm(r);
			const float    c = std::cos(g_roll.lastTheta), sn = std::sin(g_roll.lastTheta);
			const NiPoint3 un = u * c + r * sn, rn = r * c - u * sn;
			m[0][1] = un.x; m[1][1] = un.y; m[2][1] = un.z;
			m[0][2] = rn.x; m[1][2] = rn.y; m[2][2] = rn.z;
			g_roll.origF = f; g_roll.origU = u; g_roll.origR = r;
			g_roll.wroteF = f; g_roll.wroteU = un; g_roll.wroteR = rn;
			if (g_roll.logged <= 14) {
				logger::info("roll: the camera was rebuilt ({}): rolled again by {:.1f} deg", a_where, g_roll.lastTheta * 57.29578f);
			}
			return true;
		}

		void Hook_CamUpdate(void* a_cam)
		{
			if (g_roll.writeNow && g_roll.haveWrote && a_cam == static_cast<void*>(RE::Main::WorldRootCamera())) {
				RollInPlace(static_cast<RE::NiCamera*>(a_cam), "camera update");
			}
			g_origCamUpdate(a_cam);
		}
	}

	bool Install()
	{
		return hooks::Install(kCamUpdate, reinterpret_cast<void*>(&Hook_CamUpdate), reinterpret_cast<void**>(&g_origCamUpdate));
	}

	void* CamUpdateOriginal()
	{
		return reinterpret_cast<void*>(g_origCamUpdate);
	}

	void RollBuildCheck(const float* a_viewUp)
	{
		if (!g_roll.active || !g_roll.haveWrote || g_roll.logged > 14) {
			return;
		}
		const NiPoint3 up{ a_viewUp[0], a_viewUp[1], a_viewUp[2] };
		logger::info("roll: the view matrices just built have up ({:.3f},{:.3f},{:.3f}); the roll written this frame has up ({:.3f},{:.3f},{:.3f}): {:.1f} deg apart",
			up.x, up.y, up.z, g_roll.wroteU.x, g_roll.wroteU.y, g_roll.wroteU.z, std::acos(std::clamp(Dot(Norm(up), Norm(g_roll.wroteU)), -1.0f, 1.0f)) * 57.29578f);
	}

	void ReapplyRoll(const char* a_where)
	{
		if (!g_roll.writeNow || !g_roll.haveWrote) {
			return;
		}
		auto* cam = RE::Main::WorldRootCamera();
		if (!cam) {
			return;
		}
		auto&    m = cam->world.rotate.entry;
		NiPoint3 f{ m[0][0], m[1][0], m[2][0] }, u{ m[0][1], m[1][1], m[2][1] }, r{ m[0][2], m[1][2], m[2][2] };
		auto     same = [](const NiPoint3& a, const NiPoint3& b) { return std::fabs(a.x - b.x) + std::fabs(a.y - b.y) + std::fabs(a.z - b.z) < 1e-6f; };
		if (same(f, g_roll.wroteF) && same(u, g_roll.wroteU) && same(r, g_roll.wroteR)) {
			return;  // still holding our roll
		}
		// the engine rebuilt the camera: roll its new (upright) basis by the same amount
		f = Norm(f);
		u = Norm(u);
		r = Norm(r);
		const float    c = std::cos(g_roll.lastTheta), sn = std::sin(g_roll.lastTheta);
		const NiPoint3 un = u * c + r * sn, rn = r * c - u * sn;
		m[0][1] = un.x; m[1][1] = un.y; m[2][1] = un.z;
		m[0][2] = rn.x; m[1][2] = rn.y; m[2][2] = rn.z;
		g_roll.origF = f; g_roll.origU = u; g_roll.origR = r;
		g_roll.wroteF = f; g_roll.wroteU = un; g_roll.wroteR = rn;
		view::RefreshCamera(cam);
		if (g_roll.logged <= 14) {
			logger::info("roll: the camera was rebuilt before {}: rolled again by {:.1f} deg", a_where, g_roll.lastTheta * 57.29578f);
		}
	}

	void RollCheck(const char* a_where)
	{
		if (!g_roll.active || !g_roll.haveWrote || g_roll.logged > 14) {
			return;
		}
		auto* cam = RE::Main::WorldRootCamera();
		if (!cam) {
			return;
		}
		const auto& m = cam->world.rotate.entry;
		const NiPoint3 u{ m[0][1], m[1][1], m[2][1] };
		logger::info("roll: check {}: camera up ({:.3f},{:.3f},{:.3f}) differs from the written up by {:.1f} deg", a_where, u.x, u.y, u.z,
			std::acos(std::clamp(Dot(Norm(u), Norm(g_roll.wroteU)), -1.0f, 1.0f)) * 57.29578f);
	}

	void ApplyRoll(double a_dt)
	{
		auto* cam = RE::Main::WorldRootCamera();
		if (!cam) {
			return;
		}
		if (!g_roll.enabled.load()) {
			g_roll.pending = false;
			g_roll.active = false;
			g_roll.writeNow = false;
			return;
		}
		if (!g_roll.pending && !g_roll.active) {
			g_roll.haveWrote = false;
			return;
		}
		auto&    m = cam->world.rotate.entry;
		NiPoint3 f{ m[0][0], m[1][0], m[2][0] }, u{ m[0][1], m[1][1], m[2][1] }, r{ m[0][2], m[1][2], m[2][2] };
		auto     same = [](const NiPoint3& a, const NiPoint3& b) { return std::fabs(a.x - b.x) + std::fabs(a.y - b.y) + std::fabs(a.z - b.z) < 1e-6f; };
		// if the engine did not rebuild the camera since we rolled it, start from the basis we rolled
		if (g_roll.haveWrote && same(f, g_roll.wroteF) && same(u, g_roll.wroteU) && same(r, g_roll.wroteR)) {
			f = g_roll.origF;
			u = g_roll.origU;
			r = g_roll.origR;
		}
		f = Norm(f);
		u = Norm(u);
		r = Norm(r);
		if (g_roll.pending) {
			++g_roll.wait;
			// the engine's camera shows the mapped heading and pitch (within about 10 degrees) one or two frames after SetHeading / SetLooking
			if (Dot(f, g_roll.fwdMapped) > 0.985f || g_roll.wait > 6) {
				g_roll.pending = false;
				const NiPoint3 up = g_roll.upMapped;
				const NiPoint3 upP = up - f * Dot(f, up);
				const float    len = std::sqrt(Dot(upP, upP));
				float          theta = 0.0f;
				if (len > 0.25f) {
					theta = std::atan2(Dot(upP, r), Dot(upP, u));
				}
				logger::info("roll: after the crossing the camera forward is within {:.1f} deg of the mapped direction (waited {} frame(s)); mapped up ({:.3f},{:.3f},{:.3f}) against the engine's upright camera: roll {:.1f} deg",
					std::acos(std::clamp(Dot(f, g_roll.fwdMapped), -1.0f, 1.0f)) * 57.29578f, g_roll.wait, up.x, up.y, up.z, theta * 57.29578f);
				if (std::fabs(theta) > 0.035f) {  // 2 degrees
					g_roll.active = true;
					g_roll.t = 0;
					g_roll.theta0 = theta;
					g_roll.shotIndex = 0;
					g_roll.logged = 0;
				} else {
					return;
				}
			} else {
				return;
			}
		}
		if (!g_roll.active) {
			return;
		}
		const double s = std::clamp(g_roll.t / g_roll.duration, 0.0, 1.0);
		const float  ease = static_cast<float>(s * s * (3.0 - 2.0 * s));
		const float  theta = g_roll.theta0 * (1.0f - ease);
		const float  c = std::cos(theta), sn = std::sin(theta);
		const NiPoint3 un = u * c + r * sn, rn = r * c - u * sn;
		g_roll.origF = f; g_roll.origU = u; g_roll.origR = r;
		m[0][1] = un.x; m[1][1] = un.y; m[2][1] = un.z;
		m[0][2] = rn.x; m[1][2] = rn.y; m[2][2] = rn.z;
		g_roll.wroteF = f; g_roll.wroteU = un; g_roll.wroteR = rn;
		g_roll.haveWrote = true;
		g_roll.lastTheta = theta;
		g_roll.writeNow = true;
		view::RefreshCamera(cam);
		if (g_roll.logged < 3 || (g_roll.logged % 6) == 0 || s >= 1.0) {
			logger::info("roll: t {:.3f} s theta {:.1f} deg camera up ({:.3f},{:.3f},{:.3f}) (dot with mapped up {:.3f}, with world up {:.3f})", g_roll.t, theta * 57.29578f, un.x, un.y, un.z, Dot(un, g_roll.upMapped), un.z);
		}
		++g_roll.logged;
		if (!g_roll.shotTag.empty() && !shot::Busy()) {
			static const double at[4] = { 0.0, 0.10, 0.22, 0.45 };
			if (g_roll.shotIndex < 4 && (g_roll.t >= at[g_roll.shotIndex] || s >= 1.0)) {
				shot::Request(std::format("{}_{}", g_roll.shotTag, g_roll.shotIndex));
				++g_roll.shotIndex;
			}
		}
		g_roll.t += (std::min)(a_dt, 0.05);
		if (s >= 1.0 && (g_roll.shotTag.empty() || g_roll.shotIndex >= 4)) {
			g_roll.active = false;
			g_roll.haveWrote = false;
			g_roll.writeNow = false;
		}
	}

	void SetFlightMode(int a_mode)
	{
		g_flightMode = a_mode;
		g_flight.active = false;
		logger::info("crossing: flight mode {}", a_mode);
	}

	void Tick(double a_dt)
	{
		auto* p = RE::PlayerCharacter::GetSingleton();
		if (!p || !p->Is3DLoaded()) {
			g_havePrev = false;
			return;
		}
		// Portals belong to the area they were placed in (cross-loading-door portals are out of scope for now): leaving it through a loading door resets them.
		{
			const std::uint32_t area = world::CurrentArea();
			if (area) {
				const auto a0 = portals::Get(0), a1 = portals::Get(1);
				if ((a0.valid && a0.area && a0.area != area) || (a1.valid && a1.area && a1.area != area)) {
					logger::info("crossing: the player left the area the portals were placed in (now {:#x}); portals reset", area);
					portals::Clear();
					hole::SetFlight(false, 0, 0);
					g_flight.active = false;
				}
			}
		}
		const NiPoint3 centre = p->GetPosition() + NiPoint3{ 0, 0, 64.0f };
		if (!g_enabled || !g_havePrev || (centre - g_prev).x * (centre - g_prev).x + (centre - g_prev).y * (centre - g_prev).y + (centre - g_prev).z * (centre - g_prev).z > 250.0f * 250.0f) {
			g_prev = centre;
			g_havePrev = true;
			return;
		}
		// ---- flight after a fast exit ----
		if (g_flight.active) {
			auto* cc = p->GetCharController();
			if (!cc) {
				g_flight.active = false;
				hole::SetFlight(false, 0, 0);
			} else {
				const double dt = (std::min)((std::max)(a_dt, 1.0 / 240.0), 0.1);
				g_flight.t += dt;
				const NiPoint3 feet = p->GetPosition();
				g_flight.apex = (std::max)(g_flight.apex, feet.z);
				const float s = RE::bhkWorld::GetWorldScale();
				alignas(16) float ov[4]{};
				RE::hkVector4     v;
				cc->GetLinearVelocityImpl(v);
				_mm_store_ps(ov, v.quad);
				const float vz = ov[2];  // vertical stays the engine's (gravity, landing)
				(void)vz;
				(void)s;
				logger::info("crossing: flight t {:.3f} feet ({:.1f},{:.1f},{:.1f}) vz {:.0f}", g_flight.t, feet.x, feet.y, feet.z, vz / s);
				const double moved = std::sqrt(static_cast<double>((feet.x - g_flight.lastPos.x) * (feet.x - g_flight.lastPos.x) + (feet.y - g_flight.lastPos.y) * (feet.y - g_flight.lastPos.y)));
				const double expect = std::sqrt(static_cast<double>(g_flight.vh.x * g_flight.vh.x + g_flight.vh.y * g_flight.vh.y)) * dt;
				g_flight.stuck = (moved < 0.25 * expect) ? g_flight.stuck + dt : 0.0;
				g_flight.lastPos = feet;
				const bool landed = g_flight.t > 0.25 && cc->fallTime == 0.0f;
				if (landed || g_flight.stuck > 0.25 || g_flight.t > 3.0) {
					logger::info("crossing: flight ended after {:.2f} s ({}) at ({:.0f},{:.0f},{:.0f})", g_flight.t, landed ? "landed" : (g_flight.stuck > 0.25 ? "blocked" : "timeout"), feet.x, feet.y, feet.z);
					g_flight.active = false;
					hole::SetFlight(false, 0, 0);
				}
			}
		}
		const auto pb = portals::Get(0), po = portals::Get(1);
		// Falling into a floor portal. The collision exception removes the ground's support inside the oval, but the game's controller then just creeps down
		// (it still believes it is standing); the player has to sink until the centre passes the plane. So while the centre is above a floor portal's plane,
		// inside its oval, the player falls under gravity: the speed they already have, growing at 1372 units/s^2 (9.8 m/s^2 in game units).
		NiPoint3 centreNow = centre;
		if (pb.valid && po.valid) {
			bool  sinking = false;
			int   ks = 0;
			float kpz = 0.0f;
			for (int k = 0; k < 2 && !sinking; ++k) {
				const portals::Portal& A = k == 0 ? pb : po;
				if (A.n.z < 0.7f) continue;  // not a floor portal
				if (g_cooldown > 0 && A.colour == g_lastExit) continue;  // just came out of this one
				const NiPoint3 d = centre - A.c;
				const float    x = Dot(d, A.r) / A.hw, y = Dot(d, A.u) / A.hh, pz = Dot(d, A.n);
				if (x * x + y * y <= 1.0f && pz > 0.0f && pz < 90.0f) {
					sinking = true;
					ks = k;
					kpz = pz;
				}
			}
			if (sinking) {
				auto*             cc = p->GetCharController();
				const float       s = RE::bhkWorld::GetWorldScale();
				alignas(16) float ov[4]{};
				if (cc) {
					RE::hkVector4 v;
					cc->GetLinearVelocityImpl(v);
					_mm_store_ps(ov, v.quad);
				}
				const float fallSpeed = -ov[2] / s;  // positive = downward
				// The player falls in under normal gravity, keeping the speed they already have (2026-10-21: no instant pull, no slow sink, just easier to fall through).
				// The floor's contacts under the capsule are already dropped (Hole.cpp, the oval + the capsule radius); what kept the player creeping down was the controller
				// still in its on-ground state (no gravity on the ground, every step pushed back up). So it is told it is in the air, and the fall is integrated here as well.
				(void)kpz;
				g_sinkSpeed = (std::max)(g_sinkSpeed, fallSpeed);
				// a long frame (a screenshot being written, a load hitch) must not throw the player thousands of units down: G20/G21 fell to z -8900 that way
				const float dtc = static_cast<float>((std::min)(a_dt, 0.05));
				g_sinkTime += dtc;
				const bool  finishNow = g_sinkTime > 0.3f;  // still held up after 0.3 s over the oval (the controller's ground snap wins now and then, G352: up to 0.6 s)
				g_sinkSpeed = (std::min)(g_sinkSpeed + 1372.0f * dtc, 1400.0f);
				// pulled toward the middle of the oval while it falls: the oval is long and narrow, and a player over its tip still rests on the ground outside it with part
				// of the capsule (the author's 21f log: centre 93-99 units along a 96-unit half length, held 40-62 units above the plane while the fall speed grew); the target
				// is the nearest point of the oval shrunk by the capsule radius, reached at up to 500 units/s, so nothing outside the oval holds the player up
				const portals::Portal& A = ks == 0 ? pb : po;
				const NiPoint3 dd = centre - A.c;
				const float    lx = Dot(dd, A.r), ly = Dot(dd, A.u);
				const float    ax = (std::max)(A.hw - 22.0f, 8.0f), ay = (std::max)(A.hh - 22.0f, 8.0f);
				const float    q = std::sqrt((lx * lx) / (ax * ax) + (ly * ly) / (ay * ay));
				NiPoint3       side{ 0, 0, 0 };
				if (q > 1.0f) {
					const NiPoint3 want = A.r * (lx / q - lx) + A.u * (ly / q - ly);  // from where the centre is to the shrunk oval's edge
					const float    wl = std::sqrt(Dot(want, want));
					const float    step = (std::min)(wl, 500.0f * dtc);
					side = want * (step / (std::max)(wl, 0.001f));
				}
				// the step down: this frame's fall, or, once 0.3 s have passed, all the way to just past the plane (the crossing below then carries the player in this frame)
				const float    down = finishNow ? (kpz + 2.0f) : g_sinkSpeed * dtc;
				if (finishNow) {
					logger::info("crossing: the fall into floor portal {} completes after {:.2f} s (centre was {:.1f} above its plane)", ks, g_sinkTime, kpz);
					g_sinkTime = 0.0f;
				}
				const NiPoint3 feet = p->GetPosition();
				p->SetPosition(NiPoint3{ feet.x + side.x, feet.y + side.y, feet.z - down + side.z }, true);
				if (cc) {
					// the controller keeps its own position and put the player back on the ground every frame (G350/G351: the centre snapped back to the same standing
					// height while the fall speed grew); so its own position is moved as well
					RE::hkVector4 hp;
					cc->GetPositionImpl(hp, false);
					alignas(16) float pp[4];
					_mm_store_ps(pp, hp.quad);
					cc->SetPositionImpl(RE::hkVector4(pp[0] + side.x * s, pp[1] + side.y * s, pp[2] - down * s, 0.0f), false, false);
					cc->flags.reset(RE::CHARACTER_FLAGS::kSupport);
					cc->context.currentState = RE::hkpCharacterStateType::kInAir;
					cc->wantState = RE::hkpCharacterStateType::kInAir;
					cc->SetLinearVelocityImpl(RE::hkVector4(ov[0], ov[1], -g_sinkSpeed * s, 0.0f));
				}
				if (g_sinkLog < 40 && (++g_sinkTick % 4) == 0) {
					++g_sinkLog;
					logger::info("crossing: falling into floor portal {}: centre {:.1f} above its plane, fall speed {:.0f} (controller had {:.0f})", ks, kpz, g_sinkSpeed, fallSpeed);
				}
				centreNow = p->GetPosition() + NiPoint3{ 0, 0, 64.0f };
			} else {
				g_sinkSpeed = 0.0f;
				g_sinkTime = 0.0f;
			}
		}
		if (pb.valid && po.valid) {
			for (int k = 0; k < 2; ++k) {
				const portals::Portal& A = k == 0 ? pb : po;
				const portals::Portal& B = k == 0 ? po : pb;
				if (g_cooldown > 0 && A.colour == g_lastExit) {
					continue;  // just came out of this one
				}
				// Timed by the capsule centre passing the plane (timing it by the eye was tried in G16 and G20: a floor fall then needs the feet about 120 units under
				// the plane, where they hit collision the exception does not open).
				const NiPoint3         eyeOff{ 0, 0, g_mapEye.load() ? kEyeUp : 0.0f };
				const float            dPrev = (std::min)(Dot(g_prev - A.c, A.n), Dot(g_prev + eyeOff - A.c, A.n)), dNow = (std::min)(Dot(centreNow - A.c, A.n), Dot(centreNow + eyeOff - A.c, A.n));
				if (dPrev > 0.0f && dNow <= 0.0f) {
					const float    t = dPrev / (dPrev - dNow);
					const NiPoint3 hit = g_prev + (centreNow - g_prev) * t;
					const NiPoint3 d = hit - A.c;
					const float    x = Dot(d, A.r) / (A.hw + 8.0f), y = Dot(d, A.u) / (A.hh + 8.0f);
					if (x * x + y * y <= 1.0f) {
						const double   dt = (std::max)(a_dt, 1.0 / 240.0);
						const NiPoint3 v = (centreNow - g_prev) * static_cast<float>(1.0 / dt);
						Teleport(p, A, B, centreNow, v);
						g_sinkSpeed = 0.0f;
						return;
					}
				}
			}
		}
		if (g_cooldown > 0) {
			--g_cooldown;
		}
		g_prev = centreNow;
	}
}
