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

#include <xmmintrin.h>

namespace world
{
	namespace
	{
		constexpr float kPi = 3.14159265358979f;
		constexpr float kRad2Deg = 180.0f / kPi;
		constexpr float kDeg2Rad = kPi / 180.0f;

		const char* LayerName(std::uint32_t a_layer)
		{
			static const char* const kNames[] = {
				"Unidentified", "Static", "AnimStatic", "Transparent", "Clutter", "Weapon", "Projectile", "Spell", "Biped", "Trees",
				"Props", "Water", "Trigger", "Terrain", "Trap", "NonCollidable", "CloudTrap", "Ground", "Portal", "DebrisSmall",
				"DebrisLarge", "AcousticSpace", "ActorZone", "ProjectileZone", "GasTrap", "ShellCasting", "TransparentSmall", "InvisibleWall", "TransparentSmallAnim", "Ward",
				"CharController", "StairHelper", "DeadBip", "BipedNoCC", "AvoidBox", "CollisionBox", "CameraSphere", "DoorDetection", "ConeProjectile", "CameraPick",
				"ItemPick", "LineOfSight", "PathPick", "CustomPick1", "CustomPick2", "SpellExplosion", "DroppingPick"
			};
			return a_layer < std::size(kNames) ? kNames[a_layer] : "?";
		}

		std::string Edid(const RE::TESForm* a_form)
		{
			if (!a_form) {
				return "-";
			}
			const char* e = a_form->GetFormEditorID();
			return (e && *e) ? e : "(no editor id)";
		}

		std::string Describe(RE::TESObjectREFR* a_ref)
		{
			if (!a_ref) {
				return "(none)";
			}
			const auto* base = a_ref->GetBaseObject();
			const char* nm = a_ref->GetName();
			return std::format("ref {:08X} base {:08X} type {} '{}' {}", a_ref->GetFormID(), base ? base->GetFormID() : 0u,
				base ? static_cast<int>(base->GetFormType()) : -1, nm ? nm : "", base ? Edid(base) : std::string{ "-" });
		}

		std::string DescribeCell(const RE::TESObjectCELL* a_cell)
		{
			if (!a_cell) {
				return "(null)";
			}
			const char* nm = a_cell->GetName();
			return std::format("{:08X} '{}' edid {}{}", a_cell->GetFormID(), nm ? nm : "", Edid(a_cell), a_cell->IsInteriorCell() ? " [interior]" : " [exterior]");
		}

		float Dot(const RE::NiPoint3& a, const RE::NiPoint3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
		float Len(const RE::NiPoint3& a) { return std::sqrt(Dot(a, a)); }
		RE::NiPoint3 Norm(const RE::NiPoint3& a)
		{
			const float l = Len(a);
			return l > 1e-6f ? a * (1.0f / l) : RE::NiPoint3{ 1, 0, 0 };
		}

		std::filesystem::path SpotsFile()
		{
			auto d = logger::log_directory();
			return d ? (*d / "Riftweaver_spots.txt") : std::filesystem::path{ "Riftweaver_spots.txt" };
		}

		struct Spot
		{
			std::string  name;
			std::uint32_t anchor{ 0 };
			RE::NiPoint3 pos{};
			float        yawDeg{ 0 }, pitchDeg{ 0 };
			std::string  note;
		};

		std::vector<Spot> LoadSpots()
		{
			std::vector<Spot> v;
			std::ifstream     in(SpotsFile());
			for (std::string l; std::getline(in, l);) {
				if (l.empty() || l[0] == '#') {
					continue;
				}
				std::vector<std::string> f;
				std::stringstream        ss(l);
				for (std::string t; std::getline(ss, t, '|');) {
					f.push_back(t);
				}
				if (f.size() < 7) {
					continue;
				}
				Spot s;
				s.name = f[0];
				s.anchor = static_cast<std::uint32_t>(std::stoul(f[1], nullptr, 16));
				s.pos = { std::stof(f[2]), std::stof(f[3]), std::stof(f[4]) };
				s.yawDeg = std::stof(f[5]);
				s.pitchDeg = std::stof(f[6]);
				s.note = f.size() > 7 ? f[7] : std::string{};
				v.push_back(std::move(s));
			}
			return v;
		}

		void StoreSpots(const std::vector<Spot>& a_spots)
		{
			std::ofstream out(SpotsFile(), std::ios::trunc);
			out << "# name|anchorRefFormID(hex)|x|y|z|yawDeg|pitchDeg|note\n";
			for (const auto& s : a_spots) {
				out << std::format("{}|{:08X}|{:.2f}|{:.2f}|{:.2f}|{:.2f}|{:.2f}|{}\n", s.name, s.anchor, s.pos.x, s.pos.y, s.pos.z, s.yawDeg, s.pitchDeg, s.note);
			}
		}
	}

	const char* LayerNameOf(std::uint32_t a_layer)
	{
		return LayerName(a_layer);
	}

	bool Camera(RE::NiPoint3& a_pos, RE::NiPoint3& a_forward)
	{
		auto* cam = RE::Main::WorldRootCamera();
		if (!cam) {
			return false;
		}
		a_pos = cam->world.translate;
		// Skyrim camera: local X = forward, Y = up, Z = right; the world forward is the first column of the rotation.
		a_forward = Norm({ cam->world.rotate.entry[0][0], cam->world.rotate.entry[1][0], cam->world.rotate.entry[2][0] });
		return true;
	}

	bool CameraFull(CamInfo& a_out)
	{
		auto* cam = RE::Main::WorldRootCamera();
		if (!cam) {
			return false;
		}
		const auto& m = cam->world.rotate.entry;
		a_out.pos = cam->world.translate;
		a_out.fwd = Norm({ m[0][0], m[1][0], m[2][0] });
		a_out.up = Norm({ m[0][1], m[1][1], m[2][1] });
		a_out.right = Norm({ m[0][2], m[1][2], m[2][2] });
		const auto& fr = cam->GetRuntimeData2().viewFrustum;
		a_out.tanLeft = fr.fLeft;
		a_out.tanRight = fr.fRight;
		a_out.tanBottom = fr.fBottom;
		a_out.tanTop = fr.fTop;
		a_out.nearDist = fr.fNear;
		a_out.farDist = fr.fFar;
		return true;
	}

	Hit Raycast(const RE::NiPoint3& a_from, const RE::NiPoint3& a_dir, float a_maxDist, RE::COL_LAYER a_layer)
	{
		Hit  h;
		auto* player = RE::PlayerCharacter::GetSingleton();
		auto* cell = player ? player->GetParentCell() : nullptr;
		auto* bhk = cell ? cell->GetbhkWorld() : nullptr;
		if (!bhk) {
			return h;
		}
		const float     s = RE::bhkWorld::GetWorldScale();
		RE::bhkPickData pick;
		pick.rayInput.from = RE::hkVector4(a_from * s);
		pick.rayInput.to = RE::hkVector4((a_from + a_dir * a_maxDist) * s);
		pick.rayInput.filterInfo.filter = static_cast<std::uint32_t>(a_layer);
		pick.rayOutput.Reset();
		{
			RE::BSReadLockGuard lock(bhk->worldLock);
			bhk->PickObject(pick);
		}
		if (!pick.rayOutput.HasHit()) {
			return h;
		}
		alignas(16) float n[4];
		_mm_store_ps(n, pick.rayOutput.normal.quad);
		h.hit = true;
		h.dist = pick.rayOutput.hitFraction * a_maxDist;
		h.pos = a_from + a_dir * h.dist;
		h.normal = Norm({ n[0], n[1], n[2] });
		h.layer = static_cast<std::uint32_t>(pick.rayOutput.rootCollidable->GetCollisionLayer());
		h.ref = RE::TESHavokUtilities::FindCollidableRef(*pick.rayOutput.rootCollidable);
		h.collidable = pick.rayOutput.rootCollidable;
		return h;
	}

	std::uint32_t CurrentArea()
	{
		auto* p = RE::PlayerCharacter::GetSingleton();
		auto* cell = p ? p->GetParentCell() : nullptr;
		if (!cell) {
			return 0;
		}
		if (cell->IsInteriorCell()) {
			return cell->GetFormID();
		}
		auto* ws = p->GetWorldspace();
		return ws ? (ws->GetFormID() | 0x40000000u) : 0;
	}

	bool Ready()
	{
		auto* p = RE::PlayerCharacter::GetSingleton();
		auto* ui = RE::UI::GetSingleton();
		if (!p || !ui || !p->GetParentCell() || !p->Is3DLoaded()) {
			return false;
		}
		return !ui->IsMenuOpen(RE::LoadingMenu::MENU_NAME) && !ui->IsMenuOpen(RE::MainMenu::MENU_NAME);
	}

	void LogCenterRay()
	{
		CamInfo cam;
		if (!CameraFull(cam)) {
			return;
		}
		for (const auto layer : { RE::COL_LAYER::kLOS, RE::COL_LAYER::kProjectile, RE::COL_LAYER::kCharController, RE::COL_LAYER::kBiped }) {
			const Hit h = Raycast(cam.pos, cam.fwd, 6000.0f, layer);
			logger::info("ray: layer {} ({}) -> {} dist {:.1f} hit layer {} {}", static_cast<int>(layer), LayerName(static_cast<std::uint32_t>(layer)), h.hit ? "HIT" : "miss", h.dist, h.hit ? LayerName(h.layer) : "-", h.hit ? Describe(h.ref) : std::string{});
		}
	}

	void LogStatus()
	{
		auto* p = RE::PlayerCharacter::GetSingleton();
		if (!p) {
			logger::info("status: no player");
			return;
		}
		auto* cell = p->GetParentCell();
		auto* ws = p->GetWorldspace();
		const auto pos = p->GetPosition();
		logger::info("status: worldspace {} cell {}", ws ? std::format("{:08X} edid {}", ws->GetFormID(), Edid(ws)) : std::string{ "(none)" }, DescribeCell(cell));
		logger::info("status: player pos ({:.1f}, {:.1f}, {:.1f}) yaw {:.1f} deg pitch {:.1f} deg", pos.x, pos.y, pos.z, p->GetAngleZ() * kRad2Deg, p->GetAngleX() * kRad2Deg);
		RE::NiPoint3 cp, cf;
		if (Camera(cp, cf)) {
			logger::info("status: camera pos ({:.1f}, {:.1f}, {:.1f}) forward ({:.3f}, {:.3f}, {:.3f})", cp.x, cp.y, cp.z, cf.x, cf.y, cf.z);
		}
		if (auto* pc = RE::PlayerCamera::GetSingleton()) {
			logger::info("status: first person {} third person {} free camera {}", pc->IsInFirstPerson(), pc->IsInThirdPerson(), pc->IsInFreeCameraMode());
		}
		logger::info("status: ready {}", Ready());
		logger::info("status: health {:.1f}", p->AsActorValueOwner()->GetActorValue(RE::ActorValue::kHealth));
	}

	void LogMeasure()
	{
		auto* p = RE::PlayerCharacter::GetSingleton();
		if (!p) {
			return;
		}
		const auto bmin = p->GetBoundMin();
		const auto bmax = p->GetBoundMax();
		logger::info("measure: scale {:.3f} GetHeight {:.2f} bound radius {:.2f}", p->GetScale(), p->GetHeight(), p->GetBoundRadius());
		logger::info("measure: bound min ({:.1f}, {:.1f}, {:.1f}) max ({:.1f}, {:.1f}, {:.1f}) -> size ({:.1f}, {:.1f}, {:.1f})", bmin.x, bmin.y, bmin.z, bmax.x, bmax.y, bmax.z, bmax.x - bmin.x, bmax.y - bmin.y, bmax.z - bmin.z);
		if (auto* cc = p->GetCharController()) {
			logger::info("measure: char controller radius {:.2f} height {:.2f} actorHeight {:.2f} scale {:.3f} gravity {:.3f} fallStartHeight {:.2f} fallTime {:.3f}", cc->radius, cc->height, cc->actorHeight, cc->scale, cc->gravity, cc->fallStartHeight, cc->fallTime);
			logger::info("measure: (havok units; world scale {:.6f}) radius {:.2f} units, height {:.2f} units", RE::bhkWorld::GetWorldScale(), cc->radius / RE::bhkWorld::GetWorldScale(), cc->height / RE::bhkWorld::GetWorldScale());
		} else {
			logger::info("measure: no char controller");
		}
	}

	void LogRayFan(const std::string& a_arg)
	{
		int            n = 16;
		float          dist = 3000.0f;
		RE::COL_LAYER  layer = RE::COL_LAYER::kLOS;
		bool           quiet = false;
		std::istringstream in(a_arg);
		for (std::string tok; in >> tok;) {
			const auto eq = tok.find('=');
			if (eq == std::string::npos) {
				continue;
			}
			const auto k = tok.substr(0, eq);
			const auto v = tok.substr(eq + 1);
			if (k == "n") n = std::clamp(std::atoi(v.c_str()), 4, 128);
			else if (k == "dist") dist = std::stof(v);
			else if (k == "layer") layer = static_cast<RE::COL_LAYER>(std::atoi(v.c_str()));
			else if (k == "quiet") quiet = std::atoi(v.c_str()) != 0;
		}
		RE::NiPoint3 eye, fwd;
		if (!Camera(eye, fwd)) {
			logger::warn("rayfan: no camera");
			return;
		}
		logger::info("rayfan: eye ({:.1f}, {:.1f}, {:.1f}) n={} dist={} layer={} ({})", eye.x, eye.y, eye.z, n, dist, static_cast<int>(layer), LayerName(static_cast<std::uint32_t>(layer)));
		const float yaw0 = std::atan2(fwd.y, fwd.x);
		int         hits = 0;
		std::map<std::string, int> byLayer;
		int         flat = 0, slope = 0, steep = 0, wall = 0, ceil = 0;
		for (const float pitchDeg : { 0.0f, 25.0f, -25.0f, 55.0f, -55.0f, 85.0f, -85.0f }) {
			const float pitch = pitchDeg * kDeg2Rad;
			const int   count = (std::fabs(pitchDeg) > 80.0f) ? 1 : n;
			for (int i = 0; i < count; ++i) {
				const float yaw = yaw0 + (2.0f * kPi * i) / count;
				const RE::NiPoint3 d{ std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch) };
				const Hit h = Raycast(eye, d, dist, layer);
				if (!h.hit) {
					continue;
				}
				++hits;
				const float tiltDeg = std::acos(std::clamp(h.normal.z, -1.0f, 1.0f)) * kRad2Deg;
				++byLayer[LayerName(h.layer)];
				if (tiltDeg < 15) ++flat; else if (tiltDeg < 40) ++slope; else if (tiltDeg < 80) ++steep; else if (tiltDeg < 100) ++wall; else ++ceil;
				if (quiet) {
					continue;
				}
				logger::info("rayfan: yaw {:6.1f} pitch {:5.1f} -> dist {:7.1f} pos ({:.1f}, {:.1f}, {:.1f}) normal ({:.2f}, {:.2f}, {:.2f}) tilt {:.0f} deg layer {} {}",
					std::fmod(yaw * kRad2Deg + 360.0f, 360.0f), pitchDeg, h.dist, h.pos.x, h.pos.y, h.pos.z, h.normal.x, h.normal.y, h.normal.z, tiltDeg, LayerName(h.layer), Describe(h.ref));
			}
		}
		std::string layers;
		for (const auto& [k, v] : byLayer) {
			layers += std::format(" {}={}", k, v);
		}
		logger::info("rayfan: {} hits; tilt classes: flat(<15) {} slope(15-40) {} steep(40-80) {} wall(80-100) {} overhang/ceiling(>100) {}; layers:{}", hits, flat, slope, steep, wall, ceil, layers);
	}

	namespace
	{
		std::atomic<std::uint32_t> g_probeId{ 0 };
	}

	void ProbeFlags(const char* a_label)
	{
		const auto id = g_probeId.load();
		if (!id) {
			return;
		}
		auto* form = RE::TESForm::LookupByID<RE::Actor>(id);
		auto* r = form ? form->Get3D() : nullptr;
		if (r) {
			logger::info("probe: {:<34} root flags {:#x} ({}{})", a_label, static_cast<unsigned>(r->GetFlags().underlying()), r->GetFlags().any(RE::NiAVObject::Flag::kNotVisible) ? "NOT visible" : "visible", r->GetFlags().any(RE::NiAVObject::Flag::kAccumulated) ? ", accumulated" : "");
		}
	}

	void KeepActorsAnimated(float a_radius)
	{
		auto* p = RE::PlayerCharacter::GetSingleton();
		auto* tes = RE::TES::GetSingleton();
		if (!p || !tes || !p->GetParentCell()) {
			return;
		}
		RE::NiPoint3 centres[2]{};
		int          n = 0;
		for (int i = 0; i < 2; ++i) {
			const auto pt = portals::Get(i);
			if (pt.valid) {
				centres[n++] = pt.c;
			}
		}
		if (n == 0) {
			return;
		}
		int fixed = 0;
		tes->ForEachReferenceInRange(p, a_radius + 4000.0f, [&](RE::TESObjectREFR* a_ref) {
			if (!a_ref || a_ref == p || a_ref->GetFormType() != RE::FormType::ActorCharacter || a_ref->IsDisabled()) {
				return RE::BSContainer::ForEachResult::kContinue;
			}
			auto* root = a_ref->Get3D();
			if (!root) {
				return RE::BSContainer::ForEachResult::kContinue;
			}
			const auto q = a_ref->GetPosition();
			bool       isNear = false;
			for (int i = 0; i < n; ++i) {
				const float dx = q.x - centres[i].x, dy = q.y - centres[i].y, dz = q.z - centres[i].z;
				if (dx * dx + dy * dy + dz * dz < a_radius * a_radius) isNear = true;
			}
			if (isNear && root->GetFlags().any(RE::NiAVObject::Flag::kNotVisible)) {
				root->GetFlags().reset(RE::NiAVObject::Flag::kNotVisible);
				root->GetFlags().set(RE::NiAVObject::Flag::kAccumulated);
				++fixed;
			}
			return RE::BSContainer::ForEachResult::kContinue;
		});
		static int s_logged = 0;
		if (fixed && s_logged++ < 3) {
			logger::info("animfix: {} actor(s) near the portals had their root marked visible", fixed);
		}
	}

	namespace
	{
		std::string Lower(std::string s)
		{
			for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
			return s;
		}
		std::string NodeDesc(RE::NiAVObject* a_n, const RE::NiPoint3& a_me)
		{
			std::string chain;
			int         up = 0;
			for (auto* q = a_n->parent; q && up < 3; q = q->parent, ++up) {
				chain += std::format("/{}", q->name.c_str() ? q->name.c_str() : "?");
			}
			const auto  f = static_cast<unsigned>(a_n->GetFlags().underlying());
			const auto& w = a_n->world.translate;
			const float d = std::sqrt((w.x - a_me.x) * (w.x - a_me.x) + (w.y - a_me.y) * (w.y - a_me.y) + (w.z - a_me.z) * (w.z - a_me.z));
			std::string ctl;
			for (auto* c = a_n->GetControllers(); c && ctl.size() < 400; c = c->GetNext()) {
				const auto* rt = c->GetRTTI();
				ctl += std::format(" [{} flags {:#x} last {:.3f} start {:.2f} freq {:.3f}{}]", rt && rt->GetName() ? rt->GetName() : "?", static_cast<unsigned>(c->flags.underlying()), c->lastTime, c->startTime, c->frequency, (c->flags.underlying() & 8) ? "" : " INACTIVE");
			}
			std::string extra;
			if (auto* g = a_n->AsGeometry()) {
				if (auto* sp = g->GetGeometryRuntimeData().shaderProperty.get()) {
					const auto* rt = sp->GetRTTI();
					extra += std::format(" prop {} alpha {:.3f}", rt && rt->GetName() ? rt->GetName() : "?", sp->alpha);
					for (RE::NiTimeController* c = static_cast<RE::NiObjectNET*>(sp)->GetControllers(); c; c = c->GetNext()) {
						const auto* crt = c->GetRTTI();
						extra += std::format(" [propctl {} flags {:#x} last {:.3f} freq {:.3f}]", crt && crt->GetName() ? crt->GetName() : "?", static_cast<unsigned>(c->flags.underlying()), c->lastTime, c->frequency);
					}
				}
			}
			return std::format("'{}' flags {:#x} ({}{}) pos ({:.0f}, {:.0f}, {:.0f}) dist {:.0f} parents {} controllers{}{}", a_n->name.c_str() ? a_n->name.c_str() : "?", f, (f & 0x100000) ? "NOT visible" : "visible", (f & 0x4000000) ? ", accumulated" : "", w.x, w.y, w.z, d, chain, ctl.empty() ? " none" : ctl, extra);
		}
		RE::NiAVObject* WorldTop()
		{
			auto* p = RE::PlayerCharacter::GetSingleton();
			RE::NiAVObject* r = p ? p->Get3D() : nullptr;
			while (r && r->parent) r = r->parent;
			return r;
		}
		std::vector<RE::NiAVObject*> Matching(const std::string& a_substr, std::size_t& a_visited)
		{
			std::vector<RE::NiAVObject*> out;
			auto* root = WorldTop();
			if (!root) return out;
			const std::string want = Lower(a_substr);
			std::vector<RE::NiAVObject*> st{ root };
			while (!st.empty()) {
				auto* n = st.back();
				st.pop_back();
				++a_visited;
				if (n->name.c_str() && Lower(n->name.c_str()).find(want) != std::string::npos) out.push_back(n);
				if (auto* nn = n->AsNode()) {
					for (auto& c : nn->GetChildren()) {
						if (c) st.push_back(c.get());
					}
				}
			}
			return out;
		}
	}

	void FindNodes(const std::string& a_substr, int a_max)
	{
		auto*                        p = RE::PlayerCharacter::GetSingleton();
		std::size_t                  visited = 0;
		auto                         v = Matching(a_substr, visited);
		const auto                   me = p ? p->GetPosition() : RE::NiPoint3{};
		std::sort(v.begin(), v.end(), [&](RE::NiAVObject* a, RE::NiAVObject* b) {
			auto d = [&](RE::NiAVObject* n) { const auto& w = n->world.translate; return (w.x - me.x) * (w.x - me.x) + (w.y - me.y) * (w.y - me.y) + (w.z - me.z) * (w.z - me.z); };
			return d(a) < d(b);
		});
		int vis = 0, notvis = 0;
		for (auto* n : v) ((n->GetFlags().underlying() & 0x100000) ? notvis : vis)++;
		logger::info("findnode '{}': {} node(s) of {} visited; {} visible, {} NOT visible; nearest {}:", a_substr, v.size(), visited, vis, notvis, std::min<int>(a_max, static_cast<int>(v.size())));
		for (int i = 0; i < a_max && i < static_cast<int>(v.size()); ++i) {
			logger::info("findnode:   {}", NodeDesc(v[i], me));
		}
	}

	void NodeTrack(const std::string& a_substr, int a_frames)
	{
		auto*       p = RE::PlayerCharacter::GetSingleton();
		std::size_t visited = 0;
		auto        v = Matching(a_substr, visited);
		if (v.empty() || !p) {
			logger::info("nodetrack: no node matching '{}'", a_substr);
			return;
		}
		const auto me = p->GetPosition();
		auto       d = [&](RE::NiAVObject* n) { const auto& w = n->world.translate; return (w.x - me.x) * (w.x - me.x) + (w.y - me.y) * (w.y - me.y) + (w.z - me.z) * (w.z - me.z); };
		auto*      best = *std::min_element(v.begin(), v.end(), [&](RE::NiAVObject* a, RE::NiAVObject* b) { return d(a) < d(b); });
		auto       keep = std::make_shared<RE::NiPointer<RE::NiAVObject>>(best);
		auto       left = std::make_shared<int>(a_frames);
		logger::info("nodetrack: tracking {}", NodeDesc(best, me));
		frame::SetPerFrame([keep, left](double) {
			auto* pl = RE::PlayerCharacter::GetSingleton();
			logger::info("nodetrack: {}", NodeDesc(keep->get(), pl ? pl->GetPosition() : RE::NiPoint3{}));
			return --(*left) > 0;
		});
	}

	void AnimTrack(float a_radius, int a_frames, const std::string& a_name)
	{
		auto* p = RE::PlayerCharacter::GetSingleton();
		auto* tes = RE::TES::GetSingleton();
		if (!p || !tes) {
			return;
		}
		RE::Actor* best = nullptr;
		float      bestD = 1e30f;
		auto       lower = [](std::string s) { for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c))); return s; };
		const std::string want = lower(a_name);
		tes->ForEachReferenceInRange(p, a_radius, [&](RE::TESObjectREFR* a_ref) {
			if (!a_ref || a_ref == p || a_ref->GetFormType() != RE::FormType::ActorCharacter || a_ref->IsDisabled() || !a_ref->Get3D()) {
				return RE::BSContainer::ForEachResult::kContinue;
			}
			const char* nm = a_ref->GetName();
			if (!want.empty() && lower(nm ? nm : "").find(want) == std::string::npos) {
				return RE::BSContainer::ForEachResult::kContinue;
			}
			const auto  q = a_ref->GetPosition(), me = p->GetPosition();
			const float d = (q.x - me.x) * (q.x - me.x) + (q.y - me.y) * (q.y - me.y) + (q.z - me.z) * (q.z - me.z);
			if (d < bestD) {
				bestD = d;
				best = a_ref->As<RE::Actor>();
			}
			return RE::BSContainer::ForEachResult::kContinue;
		});
		if (!best) {
			logger::info("animtrack: no actor matching '{}' with loaded 3D within {:.0f} units", a_name, a_radius);
			return;
		}
		const auto  id = best->GetFormID();
		const char* nm = best->GetName();
		auto*       root = best->Get3D();
		const auto  proc = best->GetActorRuntimeData().currentProcess;
		logger::info("animtrack: tracking {:08X} '{}' at {:.0f} units, 3D root '{}' flags {:#x}, process level {}", id, nm ? nm : "", std::sqrt(bestD), root && root->name.c_str() ? root->name.c_str() : "?", root ? static_cast<unsigned>(root->GetFlags().underlying()) : 0u,
			proc ? static_cast<int>(proc->processLevel.get()) : -1);
		g_probeId = id;
		auto prev = std::make_shared<std::vector<RE::NiPoint3>>();
		auto left = std::make_shared<int>(a_frames);
		auto idx = std::make_shared<int>(0);
		frame::SetPerFrame([id, prev, left, idx](double) {
			auto* form = RE::TESForm::LookupByID<RE::Actor>(id);
			auto* r = form ? form->Get3D() : nullptr;
			if (!r) {
				logger::info("animtrack: the actor's 3D is gone");
				g_probeId = 0;
				return false;
			}
			std::vector<RE::NiAVObject*> q{ r };
			std::vector<RE::NiPoint3>    cur;
			for (std::size_t i = 0; i < q.size() && q.size() < 300; ++i) {
				cur.push_back(q[i]->world.translate);
				if (auto* n = q[i]->AsNode()) {
					for (auto& c : n->GetChildren()) {
						if (c && q.size() < 300) q.push_back(c.get());
					}
				}
			}
			double motion = 0.0;
			if (prev->size() == cur.size()) {
				for (std::size_t i = 0; i < cur.size(); ++i) {
					motion += std::fabs(cur[i].x - (*prev)[i].x) + std::fabs(cur[i].y - (*prev)[i].y) + std::fabs(cur[i].z - (*prev)[i].z);
				}
			}
			*prev = cur;
			const auto proc = form->GetActorRuntimeData().currentProcess;
			logger::info("animtrack: frame {} nodes {} motion {:.4f} root flags {:#x} process level {}", (*idx)++, cur.size(), motion, static_cast<unsigned>(r->GetFlags().underlying()), proc ? static_cast<int>(proc->processLevel.get()) : -1);
			if (--(*left) > 0) {
				return true;
			}
			g_probeId = 0;
			return false;
		});
	}

	float LogActors(float a_radius)
	{
		auto* p = RE::PlayerCharacter::GetSingleton();
		auto* tes = RE::TES::GetSingleton();
		if (!p || !tes) {
			return -1.0f;
		}
		struct A
		{
			float       d;
			std::string text;
		};
		std::vector<A> v;
		tes->ForEachReferenceInRange(p, a_radius, [&](RE::TESObjectREFR* a_ref) {
			if (!a_ref || a_ref == p || a_ref->GetFormType() != RE::FormType::ActorCharacter || a_ref->IsDisabled()) {
				return RE::BSContainer::ForEachResult::kContinue;
			}
			const auto  q = a_ref->GetPosition(), me = p->GetPosition();
			const float d = std::sqrt((q.x - me.x) * (q.x - me.x) + (q.y - me.y) * (q.y - me.y) + (q.z - me.z) * (q.z - me.z));
			const auto* act = a_ref->As<RE::Actor>();
			const char* nm = a_ref->GetName();
			v.push_back({ d, std::format("{:08X} '{}'{}{}", a_ref->GetFormID(), nm ? nm : "", (act && act->IsGuard()) ? " [GUARD]" : "", (act && act->IsDead()) ? " [dead]" : "") });
			return RE::BSContainer::ForEachResult::kContinue;
		});
		std::sort(v.begin(), v.end(), [](const A& a, const A& b) { return a.d < b.d; });
		logger::info("actors: {} actor(s) within {:.0f} units; nearest {}", v.size(), a_radius, v.empty() ? std::string{ "none" } : std::format("{:.0f}", v.front().d));
		for (std::size_t i = 0; i < v.size() && i < 8; ++i) {
			logger::info("actors:   {:6.0f} units  {}", v[i].d, v[i].text);
		}
		return v.empty() ? -1.0f : v.front().d;
	}

	void LogDoors(float a_radius)
	{
		auto* p = RE::PlayerCharacter::GetSingleton();
		auto* tes = RE::TES::GetSingleton();
		if (!p || !tes) {
			return;
		}
		int found = 0;
		tes->ForEachReferenceInRange(p, a_radius, [&](RE::TESObjectREFR* a_ref) {
			if (!a_ref) {
				return RE::BSContainer::ForEachResult::kContinue;
			}
			auto* ex = a_ref->extraList.GetByType<RE::ExtraTeleport>();
			if (!ex || !ex->teleportData) {
				return RE::BSContainer::ForEachResult::kContinue;
			}
			++found;
			const auto pos = a_ref->GetPosition();
			const auto me = p->GetPosition();
			const float d = std::sqrt((pos.x - me.x) * (pos.x - me.x) + (pos.y - me.y) * (pos.y - me.y) + (pos.z - me.z) * (pos.z - me.z));
			auto linked = ex->teleportData->linkedDoor.get();
			logger::info("doors: {} at ({:.1f}, {:.1f}, {:.1f}) dist {:.0f} yaw {:.1f}", Describe(a_ref), pos.x, pos.y, pos.z, d, a_ref->GetAngleZ() * kRad2Deg);
			if (linked) {
				const auto lp = linked->GetPosition();
				logger::info("doors:    -> {} at ({:.1f}, {:.1f}, {:.1f}) cell {} worldspace {}", Describe(linked.get()), lp.x, lp.y, lp.z, DescribeCell(linked->GetParentCell()),
					linked->GetWorldspace() ? Edid(linked->GetWorldspace()) : std::string{ "(none)" });
			} else {
				logger::info("doors:    -> linked door not resolved (destination cell not loaded)");
			}
			return RE::BSContainer::ForEachResult::kContinue;
		});
		logger::info("doors: {} loading doors within {:.0f} units", found, a_radius);
	}

	bool PoseTestLines(int a_colour, std::vector<std::string>& a_out)
	{
		const auto A = portals::Get(a_colour);
		const auto B = portals::Get(1 - a_colour);
		auto*      p = RE::PlayerCharacter::GetSingleton();
		CamInfo    cam;
		if (!A.valid || !B.valid || !p || !CameraFull(cam)) {
			logger::warn("posetest: needs both portals placed and a camera");
			return false;
		}
		auto mapPoint = [&](const RE::NiPoint3& q) {
			const RE::NiPoint3 d = q - A.c;
			const float        x = Dot(d, A.r), y = Dot(d, A.u), z = Dot(d, A.n);
			return B.c + B.r * (-x) + B.u * y + B.n * (-z);
		};
		auto mapDir = [&](const RE::NiPoint3& v) {
			const float x = Dot(v, A.r), y = Dot(v, A.u), z = Dot(v, A.n);
			return B.r * (-x) + B.u * y + B.n * (-z);
		};
		const RE::NiPoint3 pos = mapPoint(cam.pos);
		const RE::NiPoint3 f = Norm(mapDir(cam.fwd));
		const float        behind = Dot(B.c - pos, B.n);  // how far the virtual camera sits behind the exit plane
		const RE::NiPoint3 pos2 = pos + B.n * (2.0f * behind);  // reflected through the exit plane: in front of it, looking the same way
		const float        eye = cam.pos.z - p->GetPosition().z;
		const float        yaw = std::fmod(std::atan2(f.x, f.y) * kRad2Deg + 360.0f, 360.0f);
		const float        pitch = -std::asin(std::clamp(f.z, -1.0f, 1.0f)) * kRad2Deg;
		logger::info("posetest: portal {} main cam ({:.1f},{:.1f},{:.1f}); virtual cam ({:.1f},{:.1f},{:.1f}) is {:.1f} behind the exit plane; reference pose ({:.1f},{:.1f},{:.1f}) yaw {:.1f} pitch {:.1f} (eye {:.1f})",
			a_colour, cam.pos.x, cam.pos.y, cam.pos.z, pos.x, pos.y, pos.z, behind, pos2.x, pos2.y, pos2.z, yaw, pitch, eye);
		a_out.push_back(std::format("console player.setpos x {:.2f}", pos2.x));
		a_out.push_back(std::format("console player.setpos y {:.2f}", pos2.y));
		a_out.push_back(std::format("console player.setpos z {:.2f}", pos2.z - eye));
		a_out.push_back(std::format("console player.setangle z {:.2f}", yaw));
		a_out.push_back(std::format("console player.setangle x {:.2f}", pitch));
		return true;
	}

	bool AimGroundLines(float a_x, float a_y, float a_z, std::vector<std::string>& a_out)
	{
		// stand 200 units south of the target (feet 6 above the ground), looking at it from eye height: pitch down atan(120/200)
		const float px = a_x, py = a_y - 200.0f;
		a_out.push_back(std::format("console player.setpos x {:.2f}", px));
		a_out.push_back(std::format("console player.setpos y {:.2f}", py));
		a_out.push_back(std::format("console player.setpos z {:.2f}", a_z + 6.0f));
		a_out.push_back("console player.setangle z 0");   // facing +Y (north)
		a_out.push_back(std::format("console player.setangle x {:.2f}", std::atan(120.5f / 200.0f) * kRad2Deg));
		a_out.push_back("waitframes 90");
		return true;
	}

	bool DropLines(int a_colour, float a_height, std::vector<std::string>& a_out)
	{
		const auto A = portals::Get(a_colour);
		if (!A.valid) {
			logger::warn("dropabove: that portal is not placed");
			return false;
		}
		const RE::NiPoint3 pos = A.c + RE::NiPoint3{ 0, 0, a_height };  // straight above the portal's centre (a floor portal's normal can be tilted)
		a_out.push_back(std::format("console player.setpos x {:.2f}", pos.x));
		a_out.push_back(std::format("console player.setpos y {:.2f}", pos.y));
		a_out.push_back(std::format("console player.setpos z {:.2f}", pos.z));  // setpos sets the feet
		a_out.push_back("console player.setangle x 0");
		logger::info("dropabove: portal {} height {:.0f} -> feet at ({:.1f},{:.1f},{:.1f})", a_colour, a_height, pos.x, pos.y, pos.z);
		return true;
	}

	bool FaceLines(int a_colour, float a_dist, float a_lateral, std::vector<std::string>& a_out)
	{
		const auto A = portals::Get(a_colour);
		if (!A.valid) {
			logger::warn("faceportal: that portal is not placed");
			return false;
		}
		const RE::NiPoint3 pos = A.c + A.n * a_dist + A.r * a_lateral;
		const float        z = A.c.z - A.hh + 6.0f;  // feet at the oval's bottom (+ a little): standing on the floor in front of a wall portal
		RE::NiPoint3       f = A.n * -1.0f;
		const float        yaw = std::fmod(std::atan2(f.x, f.y) * kRad2Deg + 360.0f, 360.0f);
		a_out.push_back(std::format("console player.setpos x {:.2f}", pos.x));
		a_out.push_back(std::format("console player.setpos y {:.2f}", pos.y));
		a_out.push_back(std::format("console player.setpos z {:.2f}", z));
		a_out.push_back(std::format("console player.setangle z {:.2f}", yaw));
		a_out.push_back("console player.setangle x 0");
		a_out.push_back("waitframes 90");
		logger::info("faceportal: portal {} dist {:.0f} lateral {:.0f} -> pos ({:.1f},{:.1f},{:.1f}) yaw {:.1f}", a_colour, a_dist, a_lateral, pos.x, pos.y, z, yaw);
		return true;
	}

	bool FaceBackLines(int a_colour, float a_dist, float a_lateral, std::vector<std::string>& a_out)
	{
		const auto A = portals::Get(a_colour);
		if (!A.valid) {
			logger::warn("faceback: that portal is not placed");
			return false;
		}
		const auto         O = portals::Get(1 - a_colour);  // the portal the player stood in front of: the map keeps the height above the portal's CENTRE, so the feet are (O's half height - 6) below the exit's centre
		const RE::NiPoint3 pos = A.c - A.n * a_dist + A.r * a_lateral;
		const float        z = A.c.z - (O.valid ? O.hh : A.hh) + 6.0f;
		const RE::NiPoint3 f = A.n;
		const float        yaw = std::fmod(std::atan2(f.x, f.y) * kRad2Deg + 360.0f, 360.0f);
		a_out.push_back(std::format("console player.setpos x {:.2f}", pos.x));
		a_out.push_back(std::format("console player.setpos y {:.2f}", pos.y));
		a_out.push_back(std::format("console player.setpos z {:.2f}", z));
		a_out.push_back(std::format("console player.setangle z {:.2f}", yaw));
		a_out.push_back("console player.setangle x 0");
		a_out.push_back("waitframes 90");
		logger::info("faceback: portal {} dist {:.0f} lateral {:.0f} -> pos ({:.1f},{:.1f},{:.1f}) yaw {:.1f}", a_colour, a_dist, a_lateral, pos.x, pos.y, z, yaw);
		return true;
	}

	bool SaveSpot(const std::string& a_name)
	{
		auto* p = RE::PlayerCharacter::GetSingleton();
		auto* tes = RE::TES::GetSingleton();
		if (!p || !tes || !p->GetParentCell()) {
			return false;
		}
		// anchor: the nearest placed (non-dynamic) reference with loaded 3D, so a later `player.moveto` loads the right cell
		RE::TESObjectREFR* best = nullptr;
		float              bestD = 1e30f;
		const auto         me = p->GetPosition();
		auto*              myCell = p->GetParentCell();
		tes->ForEachReferenceInRange(p, 3000.0f, [&](RE::TESObjectREFR* a_ref) {
			if (!a_ref || a_ref == p || (a_ref->GetFormID() >> 24) >= 0xFF || !a_ref->Is3DLoaded() || a_ref->IsDisabled() || a_ref->GetParentCell() != myCell) {
				return RE::BSContainer::ForEachResult::kContinue;
			}
			const auto* base = a_ref->GetBaseObject();
			if (!base || (base->GetFormType() != RE::FormType::Static && base->GetFormType() != RE::FormType::Door && base->GetFormType() != RE::FormType::Activator)) {
				return RE::BSContainer::ForEachResult::kContinue;
			}
			const auto  q = a_ref->GetPosition();
			const float d = (q.x - me.x) * (q.x - me.x) + (q.y - me.y) * (q.y - me.y) + (q.z - me.z) * (q.z - me.z);
			if (d < bestD) {
				bestD = d;
				best = a_ref;
			}
			return RE::BSContainer::ForEachResult::kContinue;
		});
		if (!best) {
			logger::warn("savespot: no anchor reference near the player");
			return false;
		}
		Spot s;
		s.name = a_name;
		s.anchor = best->GetFormID();
		s.pos = me;
		s.yawDeg = p->GetAngleZ() * kRad2Deg;
		s.pitchDeg = p->GetAngleX() * kRad2Deg;
		{
			auto* cell = p->GetParentCell();
			const char* e = cell->GetFormEditorID();
			s.note = std::format("cell {:08X} {}", cell->GetFormID(), (e && *e) ? e : "-");
		}
		auto spots = LoadSpots();
		std::erase_if(spots, [&](const Spot& o) { return o.name == s.name; });
		spots.push_back(s);
		StoreSpots(spots);
		logger::info("savespot: '{}' anchor {:08X} pos ({:.1f}, {:.1f}, {:.1f}) yaw {:.1f} pitch {:.1f} [{}] -> {}", s.name, s.anchor, s.pos.x, s.pos.y, s.pos.z, s.yawDeg, s.pitchDeg, s.note, SpotsFile().string());
		return true;
	}

	bool SpotCommands(const std::string& a_name, std::vector<std::string>& a_out)
	{
		for (const auto& s : LoadSpots()) {
			if (s.name != a_name) {
				continue;
			}
			a_out.push_back(std::format("console player.moveto {:X}", s.anchor));
			a_out.push_back("waitready 90000");
			if (s.note.rfind("cell ", 0) == 0) {
				a_out.push_back("assertcell " + s.note.substr(5, 8));  // fail fast: setpos in the wrong cell drops the player into the void
			}
			a_out.push_back(std::format("console player.setpos x {:.2f}", s.pos.x));
			a_out.push_back(std::format("console player.setpos y {:.2f}", s.pos.y));
			a_out.push_back(std::format("console player.setpos z {:.2f}", s.pos.z));
			a_out.push_back(std::format("console player.setangle z {:.2f}", s.yawDeg));
			a_out.push_back(std::format("console player.setangle x {:.2f}", s.pitchDeg));
			a_out.push_back("waitframes 240");  // the screen fades in after a cell change
			a_out.push_back("npcs 1500");       // who is nearby (a guard's greeting opens a dialogue that blocks the controls)
			a_out.push_back("menuguard");       // close a dialogue if one opened, abort the script if it will not close
			return true;
		}
		logger::warn("spot: '{}' not found in {}", a_name, SpotsFile().string());
		return false;
	}
}
