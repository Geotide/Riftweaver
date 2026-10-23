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

// Stage S3 spike tools: what kind of character controller the player has, what the portal surface's collision object looks like, and
// a scripted walk (a real W key press) to measure how close the player's capsule gets to a wall.
namespace spike
{
	namespace
	{
		void Key(WORD a_scan, bool a_down)
		{
			INPUT in{};
			in.type = INPUT_KEYBOARD;
			in.ki.wScan = a_scan;
			in.ki.dwFlags = KEYEVENTF_SCANCODE | (a_down ? 0 : KEYEVENTF_KEYUP);
			SendInput(1, &in, sizeof(in));
		}

		std::string Hex(std::uint32_t v) { return std::format("{:#010x}", v); }

		void DescribeCollidable(const char* a_what, const RE::hkpCollidable* a_c)
		{
			if (!a_c) {
				logger::info("spike: {}: (null collidable)", a_what);
				return;
			}
			void*         shape = nullptr;
			std::uint32_t shapeType = 0xFFFFFFFF;
			ReadSafeT(a_c, shape);
			if (shape) {
				ReadSafeT(static_cast<const std::uint8_t*>(shape) + 0x18, shapeType);
			}
			const auto fi = a_c->broadPhaseHandle.collisionFilterInfo.filter;
			logger::info("spike: {}: collidable {:p} owner {:p} filterInfo {} (layer {} {}, system group {}, flags {:#x}) shape {:p} type {}", a_what, static_cast<const void*>(a_c), a_c->GetOwner(), Hex(fi),
				static_cast<int>(fi & 0x7F), world::LayerNameOf(fi & 0x7F), fi >> 16, fi & 0xFF80, shape, shapeType);
		}
	}

	void FocusGame()
	{
		// The window handle comes from the engine. (A first version enumerated windows and read their titles: GetWindowText sends WM_GETTEXT to
		// windows owned by other threads of the process, and that froze the game.)
		auto* rw = RE::BSGraphics::Renderer::GetCurrentRenderWindow();
		HWND  hwnd = rw ? reinterpret_cast<HWND>(rw->hWnd) : nullptr;
		if (!hwnd) {
			logger::warn("spike: no game window");
			return;
		}
		keybd_event(VK_MENU, 0, 0, 0);  // the same trick the prototype used to be allowed to take the foreground
		keybd_event(VK_MENU, 0, KEYEVENTF_KEYUP, 0);
		SetForegroundWindow(hwnd);
		logger::info("spike: game window {:p} foreground={}", static_cast<void*>(hwnd), GetForegroundWindow() == hwnd);
	}

	bool BlockingMenuOpen(std::string* a_what)
	{
		auto* ui = RE::UI::GetSingleton();
		if (!ui) {
			return false;
		}
		static const char* const kMenus[] = { "Dialogue Menu", "ContainerMenu", "BarterMenu", "Journal Menu", "TweenMenu", "InventoryMenu", "MagicMenu", "MapMenu", "MessageBoxMenu",
			"Sleep/Wait Menu", "StatsMenu", "LevelUp Menu", "Book Menu", "Lockpicking Menu", "TrainingMenu", "GiftMenu", "Crafting Menu", "Console", "Cursor Menu" };
		for (const char* m : kMenus) {
			if (std::string_view(m) == "Cursor Menu") {
				continue;
			}
			if (ui->IsMenuOpen(m)) {
				if (a_what) *a_what = m;
				return true;
			}
		}
		if (auto* cm = RE::ControlMap::GetSingleton(); cm && !cm->IsMovementControlsEnabled()) {
			if (a_what) *a_what = "movement controls disabled";
			return true;
		}
		return false;
	}

	void PressMouse(const std::string& a_button)
	{
		const bool left = a_button == "left";
		if (!left && a_button != "right") {
			logger::warn("spike: unknown mouse button '{}'", a_button);
			return;
		}
		FocusGame();
		INPUT in[2]{};
		in[0].type = INPUT_MOUSE;
		in[0].mi.dwFlags = left ? MOUSEEVENTF_LEFTDOWN : MOUSEEVENTF_RIGHTDOWN;
		in[1].type = INPUT_MOUSE;
		in[1].mi.dwFlags = left ? MOUSEEVENTF_LEFTUP : MOUSEEVENTF_RIGHTUP;
		SendInput(1, &in[0], sizeof(INPUT));
		Sleep(120);
		SendInput(1, &in[1], sizeof(INPUT));
		logger::info("spike: clicked the {} mouse button", a_button);
	}

	void PressKey(const std::string& a_name)
	{
		WORD scan = 0;
		if (a_name == "esc") scan = 0x01;
		else if (a_name == "tab") scan = 0x0F;
		else if (a_name == "enter") scan = 0x1C;
		else if (a_name == "space") scan = 0x39;
		else if (a_name == "w") scan = 0x11;
		else if (a_name == "r") scan = 0x13;  // ready weapon
		else if (a_name == "p") scan = 0x19;  // the Magic menu
		else {
			logger::warn("spike: unknown key '{}'", a_name);
			return;
		}
		FocusGame();
		Key(scan, true);
		Key(scan, false);
		logger::info("spike: pressed {}", a_name);
	}

	void LogController()
	{
		auto* p = RE::PlayerCharacter::GetSingleton();
		auto* cc = p ? p->GetCharController() : nullptr;
		if (!cc) {
			logger::warn("spike: no char controller");
			return;
		}
		std::uintptr_t vt = 0;
		ReadSafeT(cc, vt);
		const auto vProxy1 = RE::VTABLE_bhkCharProxyController[1].address();
		const auto vRb0 = RE::VTABLE_bhkCharRigidBodyController[0].address();
		const char* kind = vt == vProxy1 ? "bhkCharProxyController" : vt == vRb0 ? "bhkCharRigidBodyController" : "UNKNOWN";
		logger::info("spike: player controller {:p} vtable rva {:#x}: {} (proxy ctl vtable rva {:#x}, rigid body ctl vtable rva {:#x})", static_cast<void*>(cc), vt - REL::Module::get().base(), kind,
			vProxy1 - REL::Module::get().base(), vRb0 - REL::Module::get().base());
		logger::info("spike: radius {:.2f} height {:.2f} (havok units)", cc->radius, cc->height);
		if (vt == vProxy1) {
			auto* pc = reinterpret_cast<RE::bhkCharProxyController*>(reinterpret_cast<std::uint8_t*>(cc) - 0x10);
			auto* hp = pc->GetCharacterProxy();
			logger::info("spike: hkpCharacterProxy {:p}, listeners {}, shapePhantom {:p}", static_cast<void*>(hp), hp ? hp->listeners.size() : -1, hp ? static_cast<void*>(hp->shapePhantom) : nullptr);
			if (hp && hp->shapePhantom) {
				DescribeCollidable("player proxy phantom", hp->shapePhantom->GetCollidable());
			}
		} else if (vt == vRb0) {
			auto* rbc = reinterpret_cast<RE::bhkCharRigidBodyController*>(cc);
			auto* rb = rbc->charRigidBody.rigidBody ? rbc->charRigidBody.rigidBody->GetRigidBody() : nullptr;
			logger::info("spike: character rigid body {:p}", static_cast<void*>(rb));
			if (rb) {
				DescribeCollidable("player rigid body", rb->GetCollidable());
			}
		}
	}

	void LogWall()
	{
		world::CamInfo cam;
		if (!world::CameraFull(cam)) {
			return;
		}
		const auto h = world::Raycast(cam.pos, cam.fwd, 6000.0f, RE::COL_LAYER::kLOS);
		if (!h.hit) {
			logger::info("spike: wall ray hit nothing");
			return;
		}
		logger::info("spike: wall ray hit at ({:.1f},{:.1f},{:.1f}) normal ({:.3f},{:.3f},{:.3f}) dist {:.1f} layer {}", h.pos.x, h.pos.y, h.pos.z, h.normal.x, h.normal.y, h.normal.z, h.dist, world::LayerNameOf(h.layer));
		DescribeCollidable("surface hit", static_cast<const RE::hkpCollidable*>(h.collidable));
		if (h.collidable) {
			std::uintptr_t vt = 0;
			ReadSafeT(static_cast<const RE::hkpCollidable*>(h.collidable)->GetOwner(), vt);
			logger::info("spike: owner object vtable rva {:#x}", vt ? vt - REL::Module::get().base() : 0);
		}
	}

	void Walk(double a_seconds, unsigned short a_scan)
	{
		struct Sample
		{
			double       t;
			RE::NiPoint3 pos;
		};
		auto samples = std::make_shared<std::vector<Sample>>();
		auto t = std::make_shared<double>(0.0);
		FocusGame();
		Key(a_scan, true);  // W (walk), A / D (strafe)
		const double secs = a_seconds;
		frame::SetPerFrame([samples, t, secs, a_scan](double a_dt) {
			auto* p = RE::PlayerCharacter::GetSingleton();
			*t += a_dt;
			if (p) {
				samples->push_back({ *t, p->GetPosition() });
			}
			if (*t < secs) {
				return true;
			}
			Key(a_scan, false);
			if (samples->size() > 2) {
				const auto& a = samples->front();
				const auto& b = samples->back();
				double      vmax = 0;
				for (std::size_t i = 1; i < samples->size(); ++i) {
					const double dt = (*samples)[i].t - (*samples)[i - 1].t;
					const auto   d = (*samples)[i].pos - (*samples)[i - 1].pos;
					if (dt > 1e-4) vmax = (std::max)(vmax, std::sqrt(static_cast<double>(d.x * d.x + d.y * d.y)) / dt);
				}
				logger::info("spike: walk {:.1f} s ({} frames): start ({:.1f},{:.1f},{:.1f}) end ({:.1f},{:.1f},{:.1f}) travelled {:.1f} units, max horizontal speed {:.1f} units/s",
					*t, samples->size(), a.pos.x, a.pos.y, a.pos.z, b.pos.x, b.pos.y, b.pos.z, std::sqrt(static_cast<double>((b.pos.x - a.pos.x) * (b.pos.x - a.pos.x) + (b.pos.y - a.pos.y) * (b.pos.y - a.pos.y))), vmax);
				const auto bl = portals::Get(0);
				if (bl.valid) {
					const float dist = (b.pos.x - bl.c.x) * bl.n.x + (b.pos.y - bl.c.y) * bl.n.y + (b.pos.z + 64.0f - bl.c.z) * bl.n.z;
					logger::info("spike: walk end: capsule centre is {:.1f} units in front of the blue portal's plane (negative = behind it)", dist);
				}
				std::string tl;
				for (std::size_t i = 0; i < samples->size(); i += (std::max<std::size_t>)(1, samples->size() / 16)) {
					const auto& s = (*samples)[i];
					tl += std::format(" [{:.2f}s ({:.1f},{:.1f})]", s.t, s.pos.x, s.pos.y);
				}
				logger::info("spike: walk track:{}", tl);
			}
			return false;
		});
	}
}

namespace spike
{
	// Logs the player's position, the controller's velocity and the health a few times a second for a_seconds (to follow a fall or a flight).
	void Track(double a_seconds)
	{
		auto t = std::make_shared<double>(0.0);
		auto lastLog = std::make_shared<double>(-1.0);
		auto* p0 = RE::PlayerCharacter::GetSingleton();
		const double hp0 = p0 ? p0->AsActorValueOwner()->GetActorValue(RE::ActorValue::kHealth) : 0.0;
		logger::info("spike: track {:.1f} s, health at start {:.1f}", a_seconds, hp0);
		frame::SetPerFrame([t, lastLog, a_seconds](double a_dt) {
			auto* p = RE::PlayerCharacter::GetSingleton();
			*t += a_dt;
			if (p && *t - *lastLog >= 0.2) {
				*lastLog = *t;
				auto*              cc = p->GetCharController();
				RE::hkVector4      v;
				alignas(16) float  q[4]{};
				if (cc) {
					cc->GetLinearVelocityImpl(v);
					_mm_store_ps(q, v.quad);
				}
				const float s = RE::bhkWorld::GetWorldScale();
				const auto  pos = p->GetPosition();
				alignas(16) float o[4]{}, ini[4]{}, mod[4]{};
				if (cc) {
					_mm_store_ps(o, cc->outVelocity.quad);
					_mm_store_ps(ini, cc->initialVelocity.quad);
					_mm_store_ps(mod, cc->velocityMod.quad);
				}
				logger::info("spike: track t {:.2f} pos ({:.0f},{:.0f},{:.0f}) velocity ({:.0f},{:.0f},{:.0f}) units/s health {:.1f} fallStart {:.1f} fallTime {:.2f} | out ({:.0f},{:.0f},{:.0f}) initial ({:.0f},{:.0f},{:.0f}) mod ({:.0f},{:.0f},{:.0f})", *t, pos.x, pos.y, pos.z, q[0] / s, q[1] / s, q[2] / s,
					p->AsActorValueOwner()->GetActorValue(RE::ActorValue::kHealth), cc ? cc->fallStartHeight : 0.0f, cc ? cc->fallTime : 0.0f, o[0] / s, o[1] / s, o[2] / s, ini[0] / s, ini[1] / s, ini[2] / s, mod[0] / s, mod[1] / s, mod[2] / s);
			}
			return *t < a_seconds;
		});
	}
}

namespace camera
{
	void SetThirdPerson(bool a_on)
	{
		auto* pc = RE::PlayerCamera::GetSingleton();
		if (!pc) {
			return;
		}
		if (a_on) {
			pc->ForceThirdPerson();
		} else {
			pc->ForceFirstPerson();
		}
		logger::info("camera: {} person requested (in first person now: {})", a_on ? "third" : "first", pc->IsInFirstPerson());
	}
}
