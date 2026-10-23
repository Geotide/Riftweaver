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

// Portal state and placement.
// Placement = one ray from the camera to find the surface and its normal, then a ray grid over the portal's footprint fitted
// with a plane (least squares over the height field along the first normal), then a slide to the nearest position where the
// surface is flat enough. On terrain the plane is pushed forward to the front-most sample, so nothing pokes through the portal
// and a gap behind it on uneven ground is accepted (as requested).
namespace portals
{
	namespace
	{
		using RE::NiPoint3;

		std::mutex g_mutex;
		Portal     g_portal[2];
		Ghost      g_ghost[2];      // G310: the portal a recast replaced, collapsing for kCloseSec

		// G310: the clock of an animation. It starts at the first rendered frame that asks (the placement happens on the game thread a frame or more before the first frame that can show it) and advances once per
		// rendered frame by that frame's time, at most 40 ms, so a hitch at the placement cannot swallow the animation: it always spans several frames.
		struct AnimClock
		{
			double        key{ 0 };
			double        lastNow{ 0 };
			float         t{ 0 };
			std::uint64_t frame{ 0 };
			bool          have{ false };
			float         Step(double a_key, double a_now, double a_dur)
			{
				const std::uint64_t f = frame::Count();
				if (!have || a_key != key) {
					have = true;
					key = a_key;
					lastNow = a_now;
					frame = f;
					t = (a_now - a_key > 1.5) ? static_cast<float>(a_dur) : 0.0f;  // not new (the overlay was not drawing when it was placed): already done
				} else if (f != frame) {
					t += static_cast<float>((std::min)(a_now - lastNow, 0.04));
					lastNow = a_now;
					frame = f;
				}
				return std::clamp(t / static_cast<float>(a_dur), 0.0f, 1.0f);
			}
		};
		AnimClock g_openClock[2], g_closeClock[2];
		CrossInfo  g_cross;         // G310: the last walk through a portal
		// Size: derived from the player's bound box (measured: 44 wide, 128 tall): 2.0 x the width and 2.5 x the height (17:59Z) = 88 x 320, as half sizes.
		// SetSize() overrides it (tests), SetFactors() changes the multiples.
		float      g_hw = 44.0f;
		float      g_hh = 160.0f;
		bool       g_floorSnap = false;  // off by default: portals go exactly where the player aims. When on, a wall portal whose lower edge is within 110 units of the floor is moved down to sit on it.
		float      g_widthFactor = 2.0f, g_heightFactor = 1.5f;  // the author, 18:11Z: back to 2.0 x the player width and 1.5 x its height (88 x 192 with the measured 44 x 128 bound box)
		float      g_widthUnits = 0.0f, g_heightUnits = 0.0f;      // absolute sizes in units (0 = use the factor)
		bool       g_sizeOverride = false;
		float      g_margin = 24.0f;
		bool       g_verbose = false;

		double NowSec()
		{
			using clock = std::chrono::steady_clock;
			return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
		}

		float   Dot(const NiPoint3& a, const NiPoint3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
		NiPoint3 Cross(const NiPoint3& a, const NiPoint3& b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
		float   Len(const NiPoint3& a) { return std::sqrt(Dot(a, a)); }
		NiPoint3 Norm(const NiPoint3& a)
		{
			const float l = Len(a);
			return l > 1e-6f ? a * (1.0f / l) : NiPoint3{ 0, 0, 1 };
		}

		// What a portal may be placed on: static level geometry and terrain. Doors, gates, mills (AnimStatic), props, clutter,
		// actors, trees, water and anything that moves are refused.
		bool Placeable(std::uint32_t a_layer)
		{
			// anything that stays put: statics, terrain and ground, trees, animated statics (doors, gates; 2026-10-21: "place portals wherever"); actors, loose physics objects
			// (clutter, props, debris) and water stay refused, a portal on something that moves would hang in the air
			return a_layer == static_cast<std::uint32_t>(RE::COL_LAYER::kStatic) || a_layer == static_cast<std::uint32_t>(RE::COL_LAYER::kTerrain) ||
			       a_layer == static_cast<std::uint32_t>(RE::COL_LAYER::kGround) || a_layer == static_cast<std::uint32_t>(RE::COL_LAYER::kTrees) ||
			       a_layer == static_cast<std::uint32_t>(RE::COL_LAYER::kAnimStatic);
		}
		// free placement (default on): the portal opens exactly where the aim ray hits, at its full size, even when the surface there is smaller than the oval, uneven or crowded; the fit of
		// the surface is still used for the plane when it succeeds at the aimed point, and nothing is searched around it any more (the old search moved the oval up to 70 units to find room)
		bool g_freePlace = true;
		bool IsTerrain(std::uint32_t a_layer)
		{
			return a_layer == static_cast<std::uint32_t>(RE::COL_LAYER::kTerrain) || a_layer == static_cast<std::uint32_t>(RE::COL_LAYER::kGround);
		}

		struct Fit
		{
			bool     ok{ false };
			NiPoint3 c{}, n{ 0, 0, 1 };
			NiPoint3 u{}, r{};
			float    frac{ 0 };
			float    maxResid{ 1e9f };
			float    front{ 0 };  // the front-most sample's distance in front of the fitted plane (>= 0)
			int      hits{ 0 }, total{ 0 };
			int      terrainHits{ 0 };
			std::string map;  // one row per ray: x y result (for the verbose log)
			std::vector<const void*> surfaces;  // the distinct collision objects the rays hit: "the surface the portal sits on"
		};

		// Ray grid over the footprint around a_centre, in the frame (a_u, a_r, a_n); fits h = h0 + a*x + b*y.
		Fit FitAt(const NiPoint3& a_centre, const NiPoint3& a_n, const NiPoint3& a_u, const NiPoint3& a_r)
		{
			Fit f;
			struct S
			{
				float x, y, h;
			};
			std::vector<S> pts;
			constexpr int kNx = 5, kNy = 13;
			for (int iy = 0; iy < kNy; ++iy) {
				for (int ix = 0; ix < kNx; ++ix) {
					const float x = (ix - (kNx - 1) * 0.5f) / ((kNx - 1) * 0.5f) * g_hw * 0.92f;
					const float y = (iy - (kNy - 1) * 0.5f) / ((kNy - 1) * 0.5f) * g_hh * 0.92f;
					const float ex = x / g_hw, ey = y / g_hh;
					if (ex * ex + ey * ey > 1.0f) {
						continue;  // outside the oval
					}
					++f.total;
					const NiPoint3 origin = a_centre + a_r * x + a_u * y + a_n * 70.0f;
					const auto     h = world::Raycast(origin, a_n * -1.0f, 160.0f, RE::COL_LAYER::kLOS);
					if (!h.hit || !Placeable(h.layer)) {
						f.map += std::format("   ray ({:+.0f},{:+.0f}) {}\n", x, y, h.hit ? std::format("hit {} at height {:+.1f} (not placeable)", world::LayerNameOf(h.layer), 70.0f - h.dist) : std::string{ "no hit" });
						continue;
					}
					if (IsTerrain(h.layer)) {
						++f.terrainHits;
					}
					if (h.collidable && std::find(f.surfaces.begin(), f.surfaces.end(), h.collidable) == f.surfaces.end()) {
						f.surfaces.push_back(h.collidable);
					}
					pts.push_back({ x, y, 70.0f - h.dist });  // height of the surface above the centre plane, along a_n
				}
			}
			f.hits = static_cast<int>(pts.size());
			f.frac = f.total ? static_cast<float>(f.hits) / f.total : 0.0f;
			if (pts.size() < 6) {
				return f;
			}
			// least squares for h = h0 + a x + b y
			double sxx = 0, sxy = 0, sx = 0, syy = 0, sy = 0, sn = 0, shx = 0, shy = 0, sh = 0;
			for (const auto& p : pts) {
				sxx += p.x * p.x; sxy += p.x * p.y; sx += p.x; syy += p.y * p.y; sy += p.y; sn += 1;
				shx += p.h * p.x; shy += p.h * p.y; sh += p.h;
			}
			// solve [[sxx sxy sx][sxy syy sy][sx sy sn]] [a b h0] = [shx shy sh]
			double m[3][4] = { { sxx, sxy, sx, shx }, { sxy, syy, sy, shy }, { sx, sy, sn, sh } };
			for (int i = 0; i < 3; ++i) {
				int piv = i;
				for (int k = i + 1; k < 3; ++k) {
					if (std::fabs(m[k][i]) > std::fabs(m[piv][i])) piv = k;
				}
				if (std::fabs(m[piv][i]) < 1e-9) {
					return f;
				}
				for (int c = 0; c < 4; ++c) std::swap(m[i][c], m[piv][c]);
				for (int k = 0; k < 3; ++k) {
					if (k == i) continue;
					const double fac = m[k][i] / m[i][i];
					for (int c = i; c < 4; ++c) m[k][c] -= fac * m[i][c];
				}
			}
			const float a = static_cast<float>(m[0][3] / m[0][0]);
			const float b = static_cast<float>(m[1][3] / m[1][1]);
			const float h0 = static_cast<float>(m[2][3] / m[2][2]);
			// plane normal: (-a, -b, 1) in the (r, u, n) basis
			f.n = Norm(a_n - a_r * a - a_u * b);
			float maxResid = 0, front = -1e9f;
			for (const auto& p : pts) {
				const float resid = p.h - (h0 + a * p.x + b * p.y);
				maxResid = (std::max)(maxResid, std::fabs(resid));
				front = (std::max)(front, resid);  // residuals are along a_n, close enough to along the fitted normal
			}
			f.maxResid = maxResid;
			f.front = (std::max)(0.0f, front);
			f.c = a_centre + a_n * h0;
			// re-orthonormalise the frame on the fitted plane
			f.u = Norm(a_u - f.n * Dot(a_u, f.n));
			f.r = Cross(f.u, f.n);
			f.ok = true;
			return f;
		}

		// Free space around the oval, in the plane of the surface: rays from just in front of the surface, from the centre outward in 16 directions, must
		// not hit anything within the oval's radius in that direction plus the margin. A perpendicular wall, a post or a prop that close would clip the
		// picture and narrow the opening the player has to fit through. Directions that point at the ground are skipped (floor contact is wanted).
		bool ClearAround(const NiPoint3& a_c, const NiPoint3& a_n, const NiPoint3& a_u, const NiPoint3& a_r, std::string* a_what)
		{
			const NiPoint3 o = a_c + a_n * 12.0f;
			for (int k = 0; k < 16; ++k) {
				const float th = k * 0.39269908f;
				const float ct = std::cos(th), st = std::sin(th);
				const NiPoint3 d = a_r * ct + a_u * st;
				if (d.z < -0.25f || d.z > 0.5f) {
					continue;  // down: floor contact is wanted; steeply up: only the player's head height matters and that is inside the oval anyway
				}
				const float rho = 1.0f / std::sqrt((ct / g_hw) * (ct / g_hw) + (st / g_hh) * (st / g_hh));
				const auto  h = world::Raycast(o, d, rho + g_margin, RE::COL_LAYER::kLOS);
				if (h.hit) {
					if (a_what) {
						*a_what = std::format("direction {:.0f} deg in the plane: {} {:.0f} units from the centre (oval edge at {:.0f}, margin {:.0f})", th * 57.29578f, world::LayerNameOf(h.layer), h.dist, rho, g_margin);
					}
					return false;
				}
			}
			return true;
		}
	}

	Portal Get(int a_colour)
	{
		std::lock_guard lk(g_mutex);
		return g_portal[a_colour & 1];
	}

	Ghost GetGhost(int a_colour)
	{
		std::lock_guard lk(g_mutex);
		Ghost           g = g_ghost[a_colour & 1];
		if (!g.p.valid) {
			return Ghost{};
		}
		g.progress = g_closeClock[a_colour & 1].Step(g.closedAtSec, NowSec(), kCloseSec);
		if (g.progress >= 1.0f) {  // finished: gone
			g_ghost[a_colour & 1] = Ghost{};
			return Ghost{};
		}
		return g;
	}

	float OpenFraction(int a_colour, double a_now)
	{
		std::lock_guard lk(g_mutex);
		const Portal&   p = g_portal[a_colour & 1];
		if (!p.valid || spells::GetStyle().openAnim <= 0.001f) {
			return 1.0f;
		}
		const float f = g_openClock[a_colour & 1].Step(p.placedAtSec, a_now, kOpenSec);
		return f < 1.0f ? (std::max)(f, 0.065f) : f;  // the first drawn frame already shows a slit (at exactly 0 the oval is a degenerate dot and the shader draws a stray horizontal flare)
	}

	void NoteCross(int a_exitColour)
	{
		std::lock_guard lk(g_mutex);
		g_cross.at = NowSec();
		g_cross.exitColour = a_exitColour & 1;
	}

	CrossInfo LastCross()
	{
		std::lock_guard lk(g_mutex);
		return g_cross;
	}

	void Clear()
	{
		std::lock_guard lk(g_mutex);
		g_portal[0] = Portal{};
		g_portal[1] = Portal{};
		g_ghost[0] = Ghost{};  // a cleared portal (a loaded save, a new game) is gone at once
		g_ghost[1] = Ghost{};
		g_cross = CrossInfo{};
		hole::Update(0, g_portal[0], {});
		hole::Update(1, g_portal[1], {});
		logger::info("portals: cleared");
	}

	void SetVerbose(bool a_on)
	{
		g_verbose = a_on;
	}

	void SetMargin(float a_units)
	{
		g_margin = a_units;
		logger::info("portals: clearance margin {:.0f} units", a_units);
	}

	void SetFloorSnap(bool a_on)
	{
		g_floorSnap = a_on;
		logger::info("portals: floor snap {}", a_on ? "ON" : "off");
	}

	void SetFactors(float a_widthFactor, float a_heightFactor)
	{
		g_widthFactor = a_widthFactor;
		g_heightFactor = a_heightFactor;
		g_widthUnits = 0.0f;
		g_heightUnits = 0.0f;
		g_sizeOverride = false;
		logger::info("portals: size = {:.2f} x the player's width, {:.2f} x the player's height", a_widthFactor, a_heightFactor);
	}

	void SetUnits(float a_width, float a_height)
	{
		g_widthUnits = a_width;
		g_heightUnits = a_height;
		g_sizeOverride = false;
		logger::info("portals: size {:.0f} wide ({}) x {:.0f} tall ({})", a_width, a_width > 0 ? "units" : "player width x factor", a_height, a_height > 0 ? "units" : "player height x factor");
	}

	void SetSize(float a_halfW, float a_halfH)
	{
		g_sizeOverride = true;
		g_hw = a_halfW;
		g_hh = a_halfH;
		logger::info("portals: size set to {:.0f} x {:.0f} (half {:.1f} x {:.1f})", a_halfW * 2, a_halfH * 2, a_halfW, a_halfH);
	}

	bool PlaceFromCamera(int a_colour, std::string& a_why)
	{
		world::CamInfo cam;
		if (!world::CameraFull(cam)) {
			a_why = "no camera";
			return false;
		}
		return PlaceFrom(cam, a_colour, a_why, true);
	}

	void Scan()
	{
		world::CamInfo base;
		if (!world::CameraFull(base)) {
			return;
		}
		int ok = 0, tried = 0;
		for (const float pitch : { -12.0f, 0.0f, 12.0f }) {
			for (int yaw = 0; yaw < 360; yaw += 10) {
				const float yr = yaw * 0.017453292f, pr = pitch * 0.017453292f;
				world::CamInfo c = base;
				c.fwd = Norm(NiPoint3{ std::sin(yr) * std::cos(pr), std::cos(yr) * std::cos(pr), -std::sin(pr) });
				c.right = Norm(Cross(c.fwd, NiPoint3{ 0, 0, 1 }) * -1.0f);
				c.up = Cross(c.right, c.fwd);
				std::string why;
				++tried;
				if (PlaceFrom(c, 0, why, false)) ++ok;
			}
		}
		logger::info("scan: {} of {} directions would take a portal of the current size", ok, tried);
	}

	bool PlaceFrom(const world::CamInfo& cam, int a_colour, std::string& a_why, bool a_commit)
	{
		const char*       name = a_colour == 0 ? "teal" : "violet";
		if (!g_sizeOverride) {
			if (auto* pl = RE::PlayerCharacter::GetSingleton()) {
				const auto bmin = pl->GetBoundMin(), bmax = pl->GetBoundMax();
				const float width = bmax.x - bmin.x, height = bmax.z - bmin.z;
				if (width > 10.0f && height > 10.0f) {
					g_hw = (g_widthUnits > 0.0f ? g_widthUnits : width * g_widthFactor) * 0.5f;
					g_hh = (g_heightUnits > 0.0f ? g_heightUnits : height * g_heightFactor) * 0.5f;
				}
			}
		}
		// as far as anything with collision is loaded (2026-10-21: casts at a far hillside opened nothing; 6000 was the old reach); farther than the loaded cells nothing can be hit
		auto first = world::Raycast(cam.pos, cam.fwd, 50000.0f, RE::COL_LAYER::kLOS);
		if (!first.hit) {
			a_why = "the ray hit nothing";
			return false;
		}
		// trapdoor aim (G375): aimed at an actor's feet the ray stops on its character collision; when that hit is at the feet (within 35 units of the actor's base) the ray
		// goes on to the floor right under it, and the portal opens there. Aimed higher (the body, a horse's saddle) it stays refused.
		if (first.layer == static_cast<std::uint32_t>(RE::COL_LAYER::kCharController) || first.layer == static_cast<std::uint32_t>(RE::COL_LAYER::kActorZone) ||
			first.layer == static_cast<std::uint32_t>(RE::COL_LAYER::kBiped) || first.layer == static_cast<std::uint32_t>(RE::COL_LAYER::kBipedNoCC)) {
			RE::TESObjectREFR* actor = nullptr;
			float              bestD = 90.0f * 90.0f;
			if (auto* pl = RE::PlayerCharacter::GetSingleton(); pl && pl->GetParentCell()) {
				pl->GetParentCell()->ForEachReferenceInRange(first.pos, 200.0f, [&](RE::TESObjectREFR* a_ref) {
					if (a_ref && a_ref != pl && a_ref->GetFormType() == RE::FormType::ActorCharacter && !a_ref->IsDisabled()) {
						const NiPoint3 c = a_ref->GetPosition();
						const float    d2 = (c.x - first.pos.x) * (c.x - first.pos.x) + (c.y - first.pos.y) * (c.y - first.pos.y);
						if (d2 < bestD) {
							bestD = d2;
							actor = a_ref;
						}
					}
					return RE::BSContainer::ForEachResult::kContinue;
				});
			}
			if (!actor && first.dist < 250.0f) {
				// an actor zone with no other actor near the hit (G383: the player's own, looking down at the floor in the inn): the ray goes on past it
				auto past = first;
				for (int k = 0; k < 4 && past.hit && !Placeable(past.layer); ++k) {
					past = world::Raycast(past.pos + cam.fwd * 4.0f, cam.fwd, 50000.0f, RE::COL_LAYER::kLOS);
				}
				if (past.hit && Placeable(past.layer)) {
					logger::info("portals: the ray passed an actor zone with no actor near it (layer {}, {:.0f} units out)", first.layer, first.dist);
					first = past;
				}
			}
			if (actor && first.pos.z - actor->GetPosition().z < 35.0f) {
				auto below = first;
				for (int k = 0; k < 4 && below.hit && !Placeable(below.layer); ++k) {
					below = world::Raycast(below.pos + cam.fwd * 4.0f, cam.fwd, 400.0f, RE::COL_LAYER::kLOS);
				}
				const NiPoint3 c = actor->GetPosition();
				if (below.hit && Placeable(below.layer) && below.normal.z > 0.7f && std::abs(below.pos.z - c.z) < 40.0f &&
					(below.pos.x - c.x) * (below.pos.x - c.x) + (below.pos.y - c.y) * (below.pos.y - c.y) < 110.0f * 110.0f) {
					logger::info("portals: trapdoor aim: the ray met {:08X} '{}' at its feet ({:.0f} above its base); the floor under it at ({:.1f},{:.1f},{:.1f}) is used", actor->GetFormID(), actor->GetName() ? actor->GetName() : "", first.pos.z - c.z, below.pos.x, below.pos.y, below.pos.z);
					first = below;
				}
			}
		}
		if (!Placeable(first.layer)) {
			a_why = std::format("surface not allowed (collision layer {} {})", first.layer, world::LayerNameOf(first.layer));
			return false;
		}
		// Havok rays pass through live actors (their collision is a phantom), so actors are refused by a proximity test: the view ray against
		// each loaded actor's upright cylinder (radius from its bound box) in front of the hit point.
		if (auto* pl = RE::PlayerCharacter::GetSingleton()) {
			std::string blocker;
			RE::TES::GetSingleton()->ForEachReferenceInRange(pl, first.dist + 300.0f, [&](RE::TESObjectREFR* a_ref) {
				if (!a_ref || a_ref == pl || a_ref->GetFormType() != RE::FormType::ActorCharacter || a_ref->IsDisabled() || !a_ref->Is3DLoaded()) {
					return RE::BSContainer::ForEachResult::kContinue;
				}
				const NiPoint3 bmin = a_ref->GetBoundMin(), bmax = a_ref->GetBoundMax();
				const float    radius = 0.45f * (std::max)(bmax.x - bmin.x, bmax.y - bmin.y);
				const NiPoint3 c = a_ref->GetPosition();
				const NiPoint3 d = c - cam.pos;
				const float    t = Dot(d, cam.fwd);  // closest approach along the ray
				if (t <= 0 || t > first.dist + 40.0f) {
					return RE::BSContainer::ForEachResult::kContinue;
				}
				// a trapdoor: on a floor, an actor standing on that floor inside the oval does not block (the ray passes its phantom to the ground under it; the portal
				// opens under its feet and it drops through); refused only for walls and ceilings, or an actor standing elsewhere in the ray's path
				if (first.normal.z > 0.7f || first.normal.z < -0.7f) {
					const float fx = c.x - first.pos.x, fy = c.y - first.pos.y;
					if (std::abs(c.z - first.pos.z) < 60.0f && fx * fx + fy * fy < 110.0f * 110.0f) {
						return RE::BSContainer::ForEachResult::kContinue;
					}
				}
				const NiPoint3 q = cam.pos + cam.fwd * t;
				const float    dx = q.x - c.x, dy = q.y - c.y;
				// on a floor, a ray that only grazes an actor at foot height does not block either (G376: aimed at a moving enemy's feet, the ray passed his legs and met
				// the floor just behind him)
				if (first.normal.z > 0.7f && q.z < c.z + 45.0f) {
					return RE::BSContainer::ForEachResult::kContinue;
				}
				if (dx * dx + dy * dy < radius * radius && q.z > c.z + bmin.z - 10.0f && q.z < c.z + bmax.z + 10.0f) {
					const char* nm = a_ref->GetName();
					blocker = std::format("an actor is in the way ({:08X} '{}', {:.0f} units ahead)", a_ref->GetFormID(), nm ? nm : "", t);
					return RE::BSContainer::ForEachResult::kStop;
				}
				return RE::BSContainer::ForEachResult::kContinue;
			});
			if (!blocker.empty()) {
				a_why = blocker;
				return false;
			}
		}
		NiPoint3 n0 = first.normal;
		if (Dot(n0, cam.fwd) > 0) {
			n0 = n0 * -1.0f;  // face the viewer
		}
		const bool terrain = IsTerrain(first.layer);
		// frame: walls and steep slopes keep world-up; floors, ceilings and gentle slopes take the player's facing as "up"
		NiPoint3 u0;
		if (std::fabs(n0.z) < 0.75f) {
			u0 = Norm(NiPoint3{ 0, 0, 1 } - n0 * n0.z);
		} else {
			NiPoint3 fh = cam.fwd;
			fh.z = 0;
			if (Len(fh) < 0.05f) {
				fh = cam.up;
				fh.z = 0;
			}
			fh = Norm(fh);
			u0 = Norm(fh - n0 * Dot(fh, n0));
		}
		const NiPoint3 r0 = Cross(u0, n0);

		struct Cand
		{
			float du, dv;
		};
		std::vector<Cand> cands{ { 0, 0 } };
		for (const float step : { 14.0f, 28.0f, 42.0f, 56.0f, 70.0f }) {
			if (g_freePlace) {
				break;  // exactly at the aimed point
			}
			for (int iy = -1; iy <= 1; ++iy) {
				for (int ix = -1; ix <= 1; ++ix) {
					if (ix || iy) cands.push_back({ ix * step, iy * step });
				}
			}
		}
		// A wall portal sits on the floor when it can: the original Portal puts a wall portal flush with the floor if its lower edge is close to it, and everything that walks (the player, an NPC,
		// a chicken) can only enter an oval whose lower edge is at floor level. The aim point usually sits at eye height, which puts the lower edge of a 192-tall oval 24 units above the floor
		// (G49: the player got in only through the 32-unit band of the collision exception; a chicken walked under it, G70). So the candidates around the position that puts the lower edge on the
		// floor are tried first; if none of them fits, the placement falls back to the aimed position as before.
		if (g_floorSnap && std::fabs(n0.z) < 0.5f && std::fabs(u0.z) > 0.7f) {
			const NiPoint3 probe = first.pos + n0 * 24.0f;
			const auto     fl = world::Raycast(probe, NiPoint3{ 0, 0, -1 }, 500.0f, RE::COL_LAYER::kLOS);
			if (fl.hit) {
				const float floorZ = probe.z - fl.dist;
				const float gap = (first.pos.z - g_hh * u0.z) - floorZ;  // lower edge of the oval at the aimed position above the floor
				if (gap > 2.0f && gap <= 110.0f) {
					const float dvSnap = -gap / u0.z;
					std::vector<Cand> snap;
					for (const float du : { 0.0f, -14.0f, 14.0f, -28.0f, 28.0f, -42.0f, 42.0f, -56.0f, 56.0f, -70.0f, 70.0f }) {
						snap.push_back({ du, dvSnap });
					}
					cands.insert(cands.begin(), snap.begin(), snap.end());
					if (g_verbose) {
						logger::info("portals: floor at z {:.1f}, the oval's lower edge would be {:.1f} above it: trying the floor-level positions first", floorZ, gap);
					}
				}
			}
		}
		struct Level
		{
			const char* name;
			float       minFrac;
			float       maxResid;
		};
		// strict: a flat wall or floor. lenient: rocks and terrain, where the plane is the best fit and any gap sits behind the portal.
		const Level levels[] = { { "flat", 0.92f, 5.0f }, { "bumpy", 0.90f, 26.0f } };
		// terrain may be any slope and as bumpy as the "bumpy" level allows; a static surface (a wall, a rock) that is not flat must still be one surface:
		// within 12 units of the plane and within 8 degrees of the surface that was aimed at (otherwise the oval straddles two surfaces, such as a wall and the roof above it)
		const float staticBumpyResid = 12.0f, staticBumpyCos = 0.990f;
		Fit         best;
		const char* bestMode = nullptr;
		float       bestDu = 0, bestDv = 0;
		int         rejectedClear = 0;
		std::string lastClearWhy;
		for (const auto& lv : levels) {
			if (!terrain || std::string_view(lv.name) == "bumpy") {
				for (const auto& c : cands) {
					const NiPoint3 centre = first.pos + u0 * c.dv + r0 * c.du;
					const Fit      f = FitAt(centre, n0, u0, r0);
					const bool isFlat = std::string_view(lv.name) == "flat";
					const bool staticOk = terrain || isFlat || (f.maxResid <= staticBumpyResid && Dot(f.n, n0) > staticBumpyCos);
					if (f.ok && f.frac >= lv.minFrac && f.maxResid <= lv.maxResid && (!isFlat || Dot(f.n, n0) > 0.9976f) && staticOk) {  // flat: within 4 degrees of the aimed surface
						std::string why;
						// Terrain is exempt from the clearance test (its surface is allowed to be uneven); every static surface, flat or bumpy, needs the free margin.
						if (!terrain && !ClearAround(f.c, f.n, f.u, f.r, &why)) {
							++rejectedClear;
							lastClearWhy = why;
							if (g_verbose) {
								logger::info("portals: candidate ({:.0f},{:.0f}) [{}] fits but is too close to something: {}", c.du, c.dv, lv.name, why);
							}
							continue;
						}
						best = f;
						bestMode = lv.name;
						bestDu = c.du;
						bestDv = c.dv;
						break;
					}
				}
			} else {
				// terrain: skip the strict level, there is no flat terrain to demand (flat ground still reports small residuals in 'bumpy')
				continue;
			}
			if (bestMode) break;
		}
		if (!bestMode && g_freePlace) {
			// the aimed surface is smaller than the oval, uneven or crowded: the portal opens there anyway, flat on the aimed surface, overhanging where it must; the passage exception
			// covers every surface the footprint's rays touch
			best = FitAt(first.pos, n0, u0, r0);
			best.n = n0;
			best.u = u0;
			best.r = r0;
			best.c = first.pos;
			best.front = 0.0f;
			bestMode = rejectedClear ? "free (crowded)" : "free";
			logger::info("portals: free placement at the aim point ({:.1f},{:.1f},{:.1f}): {} of {} footprint rays hit a placeable surface{}", first.pos.x, first.pos.y, first.pos.z, best.hits, best.total,
				rejectedClear ? std::format(", close to something ({})", lastClearWhy) : std::string{});
		}
		if (!bestMode) {
			a_why = rejectedClear ? std::format("no clear spot near the aim point: {} candidate(s) were too close to a corner or obstacle (last: {})", rejectedClear, lastClearWhy)
			                      : std::string{ "no flat-enough, wide-enough surface near the aim point" };
			return false;
		}
		Portal p;
		p.valid = true;
		p.colour = a_colour;
		p.n = best.n;
		p.u = best.u;
		p.r = best.r;
		p.hw = g_hw;
		p.hh = g_hh;
		p.c = best.c + best.n * (best.front + 0.5f);  // front-most sample + a hair, so the oval never sinks into the surface
		p.placedAtSec = NowSec();
		if (auto* pl = RE::PlayerCharacter::GetSingleton()) {
			if (auto* cell = pl->GetParentCell()) p.cell = cell->GetFormID();
			if (auto* ws = pl->GetWorldspace()) p.worldspace = ws->GetFormID();
		}
		p.area = world::CurrentArea();
		p.fitMode = std::format("{}{}", terrain ? "terrain/" : "static/", bestMode);
		if (!a_commit) {
			const float tiltD = std::acos(std::clamp(p.n.z, -1.0f, 1.0f)) * 57.29578f;
			logger::info("scan: would place at ({:.0f}, {:.0f}, {:.0f}) normal ({:.2f}, {:.2f}, {:.2f}) tilt {:.0f} deg [{}] aim distance {:.0f} offset ({:.0f}, {:.0f})", p.c.x, p.c.y, p.c.z, p.n.x, p.n.y, p.n.z, tiltD, p.fitMode, first.dist, bestDu, bestDv);
			return true;
		}
		{
			std::lock_guard lk(g_mutex);
			const Portal& other = g_portal[1 - (a_colour & 1)];
			if (other.valid && Len(other.c - p.c) < (std::max)(p.hh, other.hh) * 1.1f && Dot(other.n, p.n) > 0.8f) {
				a_why = "overlaps the other portal";
				return false;
			}
			if (g_portal[a_colour & 1].valid && spells::GetStyle().closeAnim > 0.001f) {
				g_ghost[a_colour & 1] = Ghost{ g_portal[a_colour & 1], p.placedAtSec };  // the old one collapses while the new one opens
			}
			g_portal[a_colour & 1] = p;
		}
		shot::TriggerRing();  // G310: a ring armed by a test (the opening and closing strips) starts counting at the placement
		{
			std::vector<const void*> surf = best.surfaces;
			if (first.collidable && std::find(surf.begin(), surf.end(), first.collidable) == surf.end()) surf.push_back(first.collidable);
			hole::Update(a_colour, p, surf);
		}
		const float tilt = std::acos(std::clamp(p.n.z, -1.0f, 1.0f)) * 57.29578f;
		if (g_verbose) {
			// free space along the surface from the centre (to the nearest solid thing), to see how much room there is
			const NiPoint3 o = best.c + best.n * 12.0f;
			std::string    room;
			for (const auto& dd : { std::pair<const char*, NiPoint3>{ "left", best.r * -1.0f }, { "right", best.r }, { "up", best.u }, { "down", best.u * -1.0f } }) {
				const auto h = world::Raycast(o, dd.second, 400.0f, RE::COL_LAYER::kLOS);
				room += std::format(" {} {}", dd.first, h.hit ? std::format("{:.0f}", h.dist) : std::string{ ">400" });
			}
			logger::info("portals: free space from the centre along the surface (units; the oval needs {:.0f} sideways, {:.0f} vertically, plus the {:.0f} margin):{}", g_hw, g_hh, g_margin, room);
			logger::info("portals: ray map of the accepted fit ({} of {} rays hit a placeable surface; the rest are listed):\n{}", best.hits, best.total, best.map.empty() ? std::string{ "   (all rays hit)\n" } : best.map);
		}
		logger::info("portals: size {:.0f} x {:.0f} (player bound box); placed {} at ({:.1f}, {:.1f}, {:.1f}) normal ({:.3f}, {:.3f}, {:.3f}) tilt {:.1f} deg up ({:.2f}, {:.2f}, {:.2f}) [{}] aimed layer {} offset ({:.0f}, {:.0f}) rays {}/{} maxResid {:.1f} front {:.1f}",
			g_hw * 2.0f, g_hh * 2.0f, name, p.c.x, p.c.y, p.c.z, p.n.x, p.n.y, p.n.z, tilt, p.u.x, p.u.y, p.u.z, p.fitMode, first.layer, bestDu, bestDv, best.hits, best.total, best.maxResid, best.front);
		return true;
	}
}
