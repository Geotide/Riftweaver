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

// Test input: clicks and keys put through the game's own input queue (the one the real mouse and keyboard feed), on the game thread at the start of a frame. SendInput does not reach the game's
// DirectInput devices. Used by the harness to cast the portal spells like a player: the left mouse button is the right hand ("Right Attack/Block"), the right one the left hand.
namespace input
{
	// a mouse button held for a number of frames, pressed and released through the input queue; run from Tick (not the shared per-frame job), so a walk or a spin can
	// run while a cast is held
	struct Hold
	{
		bool left{ false };
		int  frames{ 0 }, hold{ 0 };
		bool active{ false };
	};
	Hold       g_hold[2];
	std::mutex g_holdMutex;

	void InjectMouse(const std::string& a_button, int a_hold)
	{
		const bool left = a_button == "left";
		if (!left && a_button != "right") {
			logger::warn("input: unknown mouse button '{}'", a_button);
			return;
		}
		std::lock_guard lk(g_holdMutex);
		g_hold[left ? 0 : 1] = Hold{ left, 0, a_hold, true };
	}

	void TickShout();

	void TickHolds()
	{
		auto* ue = RE::UserEvents::GetSingleton();
		auto* q = RE::BSInputEventQueue::GetSingleton();
		if (!ue || !q) {
			return;
		}
		std::lock_guard lk(g_holdMutex);
		for (auto& h : g_hold) {
			if (!h.active) {
				continue;
			}
			const auto& name = h.left ? ue->rightAttack : ue->leftAttack;  // the left mouse button is "Right Attack/Block", the right one "Left Attack/Block"
			// pressed, then held (one event per frame with the time held so far, as a real button sends), then released with the whole time held: a bow's draw is read
			// from it (G354: a release that said 0.13 s fired every arrow at about 600 units/s, a barely drawn bow)
			const float held = static_cast<float>(h.frames) / 60.0f;
			if (h.frames == 0) {
				q->PushOntoInputQueue(RE::ButtonEvent::Create(RE::INPUT_DEVICE::kMouse, name, h.left ? 0 : 1, 1.0f, 0.0f));
			} else if (h.frames < h.hold) {
				q->PushOntoInputQueue(RE::ButtonEvent::Create(RE::INPUT_DEVICE::kMouse, name, h.left ? 0 : 1, 1.0f, held));
			} else if (h.frames == h.hold) {
				q->PushOntoInputQueue(RE::ButtonEvent::Create(RE::INPUT_DEVICE::kMouse, name, h.left ? 0 : 1, 0.0f, held));
				logger::info("input: injected a {} mouse click (held {} frames) into the input queue", h.left ? "left" : "right", h.hold);
				h.active = false;
				continue;
			}
			++h.frames;
		}
	}

	// mouse look through the game's input queue (tests: pan the view with a cast held, the way a player turns with the mouse): dx, dy per frame for a number of frames;
	// pushed from the frame hook (Tick), separate from the per-frame job the mouse buttons use, so a held cast and a pan run together
	std::atomic<int> g_moveFrames{ 0 }, g_moveDx{ 0 }, g_moveDy{ 0 };
	void InjectMouseMove(int a_dx, int a_dy, int a_frames)
	{
		g_moveDx = a_dx;
		g_moveDy = a_dy;
		g_moveFrames = a_frames;
		logger::info("input: mouse look {} {} per frame for {} frames", a_dx, a_dy, a_frames);
	}
	void Tick()
	{
		TickHolds();
		TickShout();
		if (g_moveFrames.load() <= 0) {
			return;
		}
		if (auto* q = RE::BSInputEventQueue::GetSingleton()) {
			q->AddMouseMoveEvent(g_moveDx.load(), g_moveDy.load());
		}
		g_moveFrames.fetch_sub(1);
	}

	// the game's own Jump control, pressed and released through the input queue (tests of jumping onto a floor portal)
	void InjectJump()
	{
		auto* ue = RE::UserEvents::GetSingleton();
		auto* q = RE::BSInputEventQueue::GetSingleton();
		if (!ue || !q) {
			return;
		}
		q->PushOntoInputQueue(RE::ButtonEvent::Create(RE::INPUT_DEVICE::kKeyboard, ue->jump, 0x39, 1.0f, 0.0f));
		q->PushOntoInputQueue(RE::ButtonEvent::Create(RE::INPUT_DEVICE::kKeyboard, ue->jump, 0x39, 0.0f, 0.1f));
		logger::info("input: injected the jump key into the input queue");
	}

	// the game's own Shout control (a tap = the first word), pressed now and released a few frames later through the input queue (tests of shouts through a portal)
	std::atomic<int> g_shoutFrames{ -1 }, g_shoutHold{ 6 };
	void InjectShout(int a_holdFrames)
	{
		g_shoutHold = (std::max)(a_holdFrames, 2);
		g_shoutFrames = 0;
		logger::info("input: shout key pressed through the input queue");
	}
	void TickShout()
	{
		const int f = g_shoutFrames.load();
		if (f < 0) {
			return;
		}
		auto* ue = RE::UserEvents::GetSingleton();
		auto* q = RE::BSInputEventQueue::GetSingleton();
		if (!ue || !q) {
			return;
		}
		if (f == 0) {
			q->PushOntoInputQueue(RE::ButtonEvent::Create(RE::INPUT_DEVICE::kKeyboard, ue->shout, 0x2C, 1.0f, 0.0f));
		} else if (f < g_shoutHold.load()) {
			q->PushOntoInputQueue(RE::ButtonEvent::Create(RE::INPUT_DEVICE::kKeyboard, ue->shout, 0x2C, 1.0f, f / 60.0f));
		} else {
			q->PushOntoInputQueue(RE::ButtonEvent::Create(RE::INPUT_DEVICE::kKeyboard, ue->shout, 0x2C, 0.0f, f / 60.0f));
			g_shoutFrames = -1;
			return;
		}
		g_shoutFrames = f + 1;
	}

	void InjectQuicksave()
	{
		// press now, release 8 frames later, on the game thread at the start of a frame, through the queue the real keyboard feeds (F5 = DirectInput scan code 0x3F)
		auto frames = std::make_shared<int>(0);
		frame::SetPerFrame([frames](double) {
			auto* ue = RE::UserEvents::GetSingleton();
			auto* q = RE::BSInputEventQueue::GetSingleton();
			if (!ue || !q) {
				return false;
			}
			if (*frames == 0) {
				q->PushOntoInputQueue(RE::ButtonEvent::Create(RE::INPUT_DEVICE::kKeyboard, ue->quicksave, 0x3F, 1.0f, 0.0f));
			} else if (*frames == 8) {
				q->PushOntoInputQueue(RE::ButtonEvent::Create(RE::INPUT_DEVICE::kKeyboard, ue->quicksave, 0x3F, 0.0f, 0.13f));
				logger::info("input: injected the quicksave key (F5) into the input queue");
				return false;
			}
			++*frames;
			return true;
		});
	}

}
