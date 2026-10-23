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

// File-based remote control so tests need no keyboard input: write command lines to
// <Documents>\My Games\Skyrim Special Edition\SKSE\Riftweaver.cmd. The plugin polls, consumes and logs them and runs them
// strictly in order on the game's main thread. A command may start a wait (sleep, frames, ready, shot); the queue
// resumes when it is over. Every wait has a timeout so a stuck state ends in the log instead of hanging the test.
namespace harness
{
	// mute on|off: test runs make no sound (2026-10-22: away from the PC, no noise). In memory only: every top-level sound category's volume is set to 0 (put back
	// on `mute off`) and our own portal sounds are turned off; his settings files are not touched (the game writes them from its settings menu, and test runs are closed
	// by PID). The runner's -Mute switch leaves Riftweaver_mute.flag in the log folder, and the plugin then mutes at data load, before the menu music starts.
	void Mute(bool a_on)
	{
		static std::vector<std::pair<RE::BGSSoundCategory*, float>> s_saved;
		static float                                                s_sounds = 1.0f, s_hum = 1.0f;
		int                                                         n0 = 0;
		auto                                                        st = spells::GetStyle();
		if (a_on) {
			if (s_saved.empty()) {
				s_sounds = st.sounds;
				s_hum = st.humVolume;
			}
			for (auto* c : RE::TESDataHandler::GetSingleton()->GetFormArray<RE::BGSSoundCategory>()) {
				if (c && !c->parentCategory) {
					if (std::none_of(s_saved.begin(), s_saved.end(), [c](const auto& e) { return e.first == c; })) {
						s_saved.emplace_back(c, c->GetCategoryVolume());
					}
					c->SetCategoryVolume(0.0f);
					++n0;
				}
			}
			st.sounds = 0.0f;
			st.humVolume = 0.0f;
		} else {
			for (auto& [c, v] : s_saved) {
				c->SetCategoryVolume(v);
				++n0;
			}
			s_saved.clear();
			st.sounds = s_sounds;
			st.humVolume = s_hum;
		}
		spells::SetStyle(st);
		logger::info("harness: mute {}: {} top-level sound categor(ies) {}, portal sounds {}", a_on ? "on" : "off", n0, a_on ? "set to volume 0 (in memory)" : "put back", a_on ? "off" : "back");
	}

	namespace
	{
		using Clock = std::chrono::steady_clock;

		struct Cmd
		{
			std::string name;
			std::string arg;
		};

		std::mutex            g_mutex;
		std::deque<Cmd>       g_queue;
		std::filesystem::path g_cmdFile;
		std::atomic<bool>     g_run{ false };
		std::atomic<bool>     g_tickQueued{ false };

		enum class Wait
		{
			None,
			Time,
			Frames,
			Ready,
			Shot
		};
		Wait               g_wait{ Wait::None };
		Clock::time_point  g_until{};          // end of a sleep, or the timeout of any other wait
		std::uint64_t      g_untilFrame{ 0 };
		bool               g_readySeen{ false };
		std::uint64_t      g_readyFrame{ 0 };

		// portal colour words: "orange" or "violet" (or "1") = portal index 1, everything else ("blue", "teal", "0") = index 0
		bool IsViolet(const std::string& a_s)
		{
			return a_s.rfind("orange", 0) == 0 || a_s.rfind("violet", 0) == 0 || a_s == "1" || a_s.rfind("1 ", 0) == 0;
		}

		void RunConsole(const std::string& a_text)
		{
			auto* player = RE::PlayerCharacter::GetSingleton();
			auto* factory = RE::IFormFactory::GetConcreteFormFactoryByType<RE::Script>();
			if (!factory) {
				logger::error("harness: no Script factory");
				return;
			}
			auto* script = factory->Create();
			if (!script) {
				logger::error("harness: script create failed");
				return;
			}
			script->SetCommand(a_text);
			script->CompileAndRun(player);
			delete script;
			logger::info("harness: ran console '{}'", a_text);
		}

		// newest *.ess in the Saves folder; a_prefix filters by file name prefix ("Quicksave" or "")
		std::string FindNewestSave(const std::string& a_prefix)
		{
			auto dir = logger::log_directory();
			if (!dir) {
				return {};
			}
			const auto saves = dir->parent_path() / "Saves";
			std::error_code                 ec;
			std::filesystem::file_time_type best{};
			std::string                     bestName;
			for (const auto& e : std::filesystem::directory_iterator(saves, ec)) {
				if (!e.is_regular_file() || e.path().extension() != ".ess") {
					continue;
				}
				const auto stem = e.path().stem().string();
				if (!a_prefix.empty() && stem.rfind(a_prefix, 0) != 0) {
					continue;
				}
				const auto t = e.last_write_time();
				if (bestName.empty() || t > best) {
					best = t;
					bestName = stem;
				}
			}
			return bestName;
		}

		void LoadSave(std::string a_name)
		{
			if (a_name == "latestquick") {
				a_name = FindNewestSave("Quicksave");
			} else if (a_name == "latest") {
				a_name = FindNewestSave("");
			}
			if (a_name.empty()) {
				logger::error("harness: no save found");
				return;
			}
			logger::info("harness: loading save '{}'", a_name);
			if (auto* mgr = RE::BGSSaveLoadManager::GetSingleton()) {
				mgr->Load(a_name.c_str());
				logger::info("harness: BGSSaveLoadManager::Load('{}') returned", a_name);
			} else {
				logger::error("harness: no BGSSaveLoadManager");
			}
		}

		void StartWait(Wait a_kind, std::chrono::milliseconds a_timeout)
		{
			g_wait = a_kind;
			g_until = Clock::now() + a_timeout;
			g_readySeen = false;
		}

		bool WaitDone()
		{
			const auto now = Clock::now();
			switch (g_wait) {
			case Wait::None:
				return true;
			case Wait::Time:
				if (now >= g_until) {
					g_wait = Wait::None;
					return true;
				}
				return false;
			case Wait::Frames:
				if (frame::Count() >= g_untilFrame) {
					g_wait = Wait::None;
					return true;
				}
				if (now >= g_until) {
					logger::warn("harness: waitframes timed out (frames advance only while in game)");
					g_wait = Wait::None;
					return true;
				}
				return false;
			case Wait::Ready:
				if (world::Ready()) {
					if (!g_readySeen) {
						g_readySeen = true;
						g_readyFrame = frame::Count();
					}
					if (frame::Count() >= g_readyFrame + 60) {
						logger::info("harness: ready");
						g_wait = Wait::None;
						return true;
					}
				} else {
					g_readySeen = false;
				}
				if (now >= g_until) {
					logger::warn("harness: waitready timed out");
					g_wait = Wait::None;
					return true;
				}
				return false;
			case Wait::Shot:
				if (!shot::Busy()) {
					g_wait = Wait::None;
					return true;
				}
				if (now >= g_until) {
					logger::warn("harness: shot timed out");
					g_wait = Wait::None;
					return true;
				}
				return false;
			}
			return true;
		}

		void PushFront(const std::vector<std::string>& a_lines)
		{
			std::lock_guard lk(g_mutex);
			for (auto it = a_lines.rbegin(); it != a_lines.rend(); ++it) {
				const auto sp = it->find(' ');
				g_queue.push_front({ it->substr(0, sp), sp == std::string::npos ? std::string{} : it->substr(sp + 1) });
			}
		}

		void Run(const Cmd& c)
		{
			const auto& n = c.name;
			const auto& a = c.arg;
			logger::info("harness: > {} {}", n, a);
			if (n == "log") {
				logger::info("harness: MARK {}", a);
			} else if (n == "sleep") {
				StartWait(Wait::Time, std::chrono::milliseconds((std::max)(0, std::atoi(a.c_str()))));
			} else if (n == "waitframes") {
				g_untilFrame = frame::Count() + static_cast<std::uint64_t>((std::max)(0, std::atoi(a.c_str())));
				StartWait(Wait::Frames, std::chrono::seconds(30));
			} else if (n == "waitready") {
				const int ms = a.empty() ? 90000 : std::atoi(a.c_str());
				StartWait(Wait::Ready, std::chrono::milliseconds(ms));
			} else if (n == "loadsave") {
				LoadSave(a);
			} else if (n == "console") {
				RunConsole(a);
			} else if (n == "closemsg") {
				if (auto* q = RE::UIMessageQueue::GetSingleton()) {
					q->AddMessage(RE::MessageBoxMenu::MENU_NAME, RE::UI_MESSAGE_TYPE::kHide, nullptr);
				}
			} else if (n == "shot") {
				shot::Request(a.empty() ? "shot" : a);
				StartWait(Wait::Shot, std::chrono::seconds(5));
			} else if (n == "spin") {
				std::istringstream in(a);
				double             rate = 90, secs = 3;
				in >> rate >> secs;
				auto left = std::make_shared<double>(secs);
				frame::SetPerFrame([rate, left](double a_dt) {
					auto* p = RE::PlayerCharacter::GetSingleton();
					if (!p || *left <= 0) {
						return false;
					}
					const double step = (std::min)(a_dt, 0.1) * rate;
					p->SetHeading(p->GetAngleZ() + static_cast<float>(step * 0.017453292519943295));
					*left -= a_dt;
					return *left > 0;
				});
				StartWait(Wait::Time, std::chrono::milliseconds(static_cast<int>(secs * 1000) + 300));
			} else if (n == "ftime") {
				// ftime <seconds> <label> [async]: the frame times of the next seconds (async: the script goes on at once, the numbers are logged when the time is over) (the engine's per-frame hook's delta): average, median, 95th and 99th percentile, worst, frames a second
				std::istringstream in(a);
				double             secs = 6;
				std::string        label = "ftime", mode;
				in >> secs >> label >> mode;
				auto samples = std::make_shared<std::vector<double>>();
				auto left = std::make_shared<double>(secs);
				frame::SetPerFrame([samples, left, label](double a_dt) {
					samples->push_back(a_dt);
					*left -= a_dt;
					if (*left > 0) {
						return true;
					}
					std::vector<double> v = *samples;
					std::sort(v.begin(), v.end());
					double sum = 0;
					for (double x : v) sum += x;
					const auto pct = [&](double p) { return v.empty() ? 0.0 : v[(std::min)(v.size() - 1, static_cast<std::size_t>(p * static_cast<double>(v.size())))] * 1000.0; };
					logger::info("harness: frame times [{}]: {} frames in {:.1f} s, average {:.2f} ms ({:.1f} fps), median {:.2f}, 95th {:.2f}, 99th {:.2f}, worst {:.2f} ms", label, v.size(), sum, v.empty() ? 0.0 : sum / static_cast<double>(v.size()) * 1000.0, sum > 0 ? static_cast<double>(v.size()) / sum : 0.0, pct(0.5), pct(0.95), pct(0.99), v.empty() ? 0.0 : v.back() * 1000.0);
					return false;
				});
				if (mode != "async") {
					StartWait(Wait::Time, std::chrono::milliseconds(static_cast<int>(secs * 1000) + 500));
				}
			} else if (n == "viewstats") {
				view::LogStats();
				diag::LogChain();
			} else if (n == "depthshot") {
				view::RequestDepthDump(a.empty() ? "depth" : a);
				g_untilFrame = frame::Count() + 4;
				StartWait(Wait::Frames, std::chrono::seconds(10));
			} else if (n == "quicksave") {
				input::InjectQuicksave();
			} else if (n == "aimthrough") {
				// aimthrough <teal|violet>: turn the player so the view ray goes into that portal and comes out at the test actor (its body's centre once dead)
				const int  ai = IsViolet(a) ? 1 : 0;
				const auto A = portals::Get(ai), B = portals::Get(1 - ai);
				RE::NiPoint3   t;
				world::CamInfo ci;
				auto*          p = RE::PlayerCharacter::GetSingleton();
				if (p && A.valid && B.valid && travel::TestNpcCentre(t) && world::CameraFull(ci)) {
					const RE::NiPoint3 d = t - B.c;
					const RE::NiPoint3 tv = A.c + A.r * (-(d.x * B.r.x + d.y * B.r.y + d.z * B.r.z)) + A.u * (d.x * B.u.x + d.y * B.u.y + d.z * B.u.z) + A.n * (-(d.x * B.n.x + d.y * B.n.y + d.z * B.n.z));
					const RE::NiPoint3 dir = tv - ci.pos;
					const float        heading = std::atan2(dir.x, dir.y);
					const float        pitchDeg = std::atan2(-dir.z, std::sqrt(dir.x * dir.x + dir.y * dir.y)) * 57.29578f;
					p->SetHeading(heading);
					logger::info("harness: aimthrough {}: target ({:.1f},{:.1f},{:.1f}) seen behind the entry at ({:.1f},{:.1f},{:.1f}): heading {:.1f}, pitch {:.1f} down", ai ? "violet" : "teal", t.x, t.y, t.z, tv.x, tv.y, tv.z, heading * 57.29578f, pitchDeg);
					PushFront({ std::format("look {:.2f}", pitchDeg) });
				} else {
					logger::warn("harness: aimthrough: needs both portals, a test actor and the camera");
				}
			} else if (n == "aimarc" || n == "placethrough") {
				// aimarc <teal|violet> [height]: aim a full-draw arrow at that portal's centre (plus height along its up axis), raised for the drop over the distance
				// (G356: an iron arrow from a hunting bow flies about 1530 units/s and falls about 534 units/s^2, 113 units over 1000 units)
				// placethrough <teal|violet> <dist>: put the held test actor on the line from the camera through that portal's centre, dist units past the other portal
				std::istringstream in(a);
				std::string        col;
				float              v = 0.0f, feet = 0.0f;
				in >> col >> v >> feet;  // placethrough: an optional third value puts the feet at that height (on the ground: a held actor in the air took no damage, G360)
				const int      ai = IsViolet(col) ? 1 : 0;
				const auto     A = portals::Get(ai), B = portals::Get(1 - ai);
				world::CamInfo ci;
				auto*          p = RE::PlayerCharacter::GetSingleton();
				if (p && A.valid && B.valid && world::CameraFull(ci)) {
					const auto     dot = [](const RE::NiPoint3& x, const RE::NiPoint3& y) { return x.x * y.x + x.y * y.y + x.z * y.z; };
					if (n == "aimarc") {
						const RE::NiPoint3 aim = A.c + A.u * v;
						RE::NiPoint3       dir = aim - ci.pos;
						const float        dist = std::sqrt(dot(dir, dir)), tt = dist / 1530.0f;
						dir.z += 0.5f * 534.0f * tt * tt;
						const float heading = std::atan2(dir.x, dir.y);
						const float pitchDeg = std::atan2(-dir.z, std::sqrt(dir.x * dir.x + dir.y * dir.y)) * 57.29578f;
						p->SetHeading(heading);
						logger::info("harness: aimarc {}: centre {:.0f} units away, raised {:.1f} for the drop: heading {:.1f}, pitch {:.1f} down", ai ? "violet" : "teal", dist, 0.5f * 534.0f * tt * tt, heading * 57.29578f, pitchDeg);
						PushFront({ std::format("look {:.2f}", pitchDeg) });
					} else {
						RE::NiPoint3 d = A.c - ci.pos;
						const float  len = std::sqrt(dot(d, d));
						d = d * (1.0f / len);
						const RE::NiPoint3 m = B.r * (-dot(d, A.r)) + B.u * dot(d, A.u) + B.n * (-dot(d, A.n));
						const RE::NiPoint3 at = B.c + m * v;
						const float        ang = std::atan2(-m.x, -m.y) * 57.29578f;
						logger::info("harness: placethrough {}: the line through the centre leaves {} along ({:.3f},{:.3f},{:.3f}); actor centre at ({:.1f},{:.1f},{:.1f})", ai ? "violet" : "teal", ai ? "teal" : "violet", m.x, m.y, m.z, at.x, at.y, at.z);
						PushFront({ std::format("npc setpos {:.1f} {:.1f} {:.1f} {:.0f}", at.x, at.y, feet != 0.0f ? feet : at.z - 70.0f, ang < 0.0f ? ang + 360.0f : ang) });
					}
				} else {
					logger::warn("harness: {}: needs both portals and the camera", n);
				}
			} else if (n == "dropinto") {
				// dropinto <teal|violet> <height>: put the player <height> units straight above that portal's centre (momentum and loop tests with floor portals)
				std::istringstream in(a);
				std::string        col, who;
				float              h = 300.0f;
				in >> col >> h >> who;  // dropinto <colour> <height> npc: the test actor (or its body) instead of the player
				const auto P = portals::Get(IsViolet(col) ? 1 : 0);
				if (P.valid && who == "npc") {
					logger::info("harness: dropinto {}: test actor put {:.0f} units above ({:.1f},{:.1f},{:.1f})", col, h, P.c.x, P.c.y, P.c.z);
					PushFront({ std::format("npc setpos {:.1f} {:.1f} {:.1f} 0", P.c.x, P.c.y, P.c.z + h) });
				} else if (P.valid) {
					logger::info("harness: dropinto {}: player put {:.0f} units above ({:.1f},{:.1f},{:.1f})", col, h, P.c.x, P.c.y, P.c.z);
					PushFront({ std::format("console player.setpos x {:.1f}", P.c.x), std::format("console player.setpos y {:.1f}", P.c.y), std::format("console player.setpos z {:.1f}", P.c.z + h) });
				} else {
					logger::warn("harness: dropinto {}: portal not placed", col);
				}
			} else if (n == "rune") {
				// rune look <dist>: stand <dist> units from the newest rune ring, on the side of the nearest portal, and look at it; rune npc: put the test actor on it (triggers the rune)
				std::istringstream in(a);
				std::string        what;
				float              dist = 220.0f;
				in >> what >> dist;
				RE::NiPoint3 rp, rn;
				if (!overlay::LastRuneRing(rp, rn)) {
					logger::warn("harness: rune {}: no rune ring", a);
				} else if (what == "npc") {
					PushFront({ std::format("npc setpos {:.1f} {:.1f} {:.1f} 0", rp.x, rp.y, rp.z + 5.0f) });
				} else {
					const portals::Portal pp[2] = { portals::Get(0), portals::Get(1) };
					RE::NiPoint3          e = rp + RE::NiPoint3{ 0.0f, 100.0f, 0.0f };
					float                 bestD = 1e30f;
					for (const auto& q : pp) {
						if (q.valid && q.c.GetDistance(rp) < bestD) {
							bestD = q.c.GetDistance(rp);
							e = q.c + q.n * 60.0f;
						}
					}
					RE::NiPoint3 dir{ e.x - rp.x, e.y - rp.y, 0.0f };
					const float  l = std::sqrt(dir.x * dir.x + dir.y * dir.y);
					dir = l > 1.0f ? dir * (1.0f / l) : RE::NiPoint3{ 0.0f, 1.0f, 0.0f };
					const RE::NiPoint3 pos = rp + dir * dist;
					const float        heading = std::atan2(-dir.x, -dir.y);
					const float        pitch = std::atan2(120.0f, dist) * 57.29578f;
					logger::info("harness: rune look: ring at ({:.1f},{:.1f},{:.1f}), player put at ({:.1f},{:.1f},{:.1f}) looking at it, pitch {:.1f} down", rp.x, rp.y, rp.z, pos.x, pos.y, pos.z, pitch);
					PushFront({ std::format("console player.setpos x {:.1f}", pos.x), std::format("console player.setpos y {:.1f}", pos.y), std::format("console player.setpos z {:.1f}", pos.z + 5.0f),
						std::format("console player.setangle z {:.1f}", heading * 57.29578f), "waitframes 10", std::format("look {:.2f}", pitch) });
				}
			} else if (n == "standclear") {
				// standclear <name> <dist>: put the player <dist> units from the nearest reference of that name, at the first of 16 directions where the ground is found and nothing
				// blocks the eye's line to the reference (placement refusal tests: G371's spot had a stall wall between the player and the horse)
				std::istringstream in(a);
				std::string        want;
				float              dist = 250.0f;
				in >> want >> dist;
				auto*              p = RE::PlayerCharacter::GetSingleton();
				RE::TESObjectREFR* ref = nullptr;
				float              best = 1e30f;
				if (auto* cell = p ? p->GetParentCell() : nullptr) {
					cell->ForEachReferenceInRange(p->GetPosition(), 6000.0f, [&](RE::TESObjectREFR* r) {
						if (r && r->GetName() && want == r->GetName() && !r->IsDisabled()) {
							const float d = r->GetPosition().GetDistance(p->GetPosition());
							if (d < best) {
								best = d;
								ref = r;
							}
						}
						return RE::BSContainer::ForEachResult::kContinue;
					});
				}
				bool done = false;
				if (p && ref) {
					const RE::NiPoint3 c = ref->GetPosition() + RE::NiPoint3{ 0.0f, 0.0f, 90.0f };
					for (int k = 0; k < 16 && !done; ++k) {
						const float        th = k * 0.3926991f;
						const RE::NiPoint3 dir{ std::sin(th), std::cos(th), 0.0f };
						RE::NiPoint3       pos = c + dir * dist;
						const auto         down = world::Raycast(pos + RE::NiPoint3{ 0.0f, 0.0f, 200.0f }, RE::NiPoint3{ 0.0f, 0.0f, -1.0f }, 600.0f, RE::COL_LAYER::kLOS);
						if (!down.hit) {
							continue;
						}
						pos.z = down.pos.z;
						const RE::NiPoint3 eye = pos + RE::NiPoint3{ 0.0f, 0.0f, 120.0f };
						RE::NiPoint3       to = c - eye;
						const float        len = std::sqrt(to.x * to.x + to.y * to.y + to.z * to.z);
						to = to * (1.0f / len);
						const auto los = world::Raycast(eye, to, len, RE::COL_LAYER::kLOS);
						if (los.hit && los.dist < len - 60.0f) {
							continue;
						}
						PushFront({ std::format("console player.setpos x {:.1f}", pos.x), std::format("console player.setpos y {:.1f}", pos.y), std::format("console player.setpos z {:.1f}", pos.z + 5.0f) });
						logger::info("harness: standclear '{}' {:08X} at ({:.1f},{:.1f},{:.1f}): player put {:.0f} units away at ({:.1f},{:.1f},{:.1f}), direction {} of 16, line of sight clear", want, ref->GetFormID(), ref->GetPosition().x, ref->GetPosition().y, ref->GetPosition().z, dist, pos.x, pos.y, pos.z, k);
						done = true;
					}
				}
				if (!done) {
					logger::warn("harness: standclear {}: no reference or no clear spot", a);
				}
			} else if (n == "aimref") {
				// aimref <hex ref id | nearest:<name>> [height]: turn the camera to a reference's position plus height (default: the middle of its bound box); placement refusal tests (a horse, its saddle)
				std::istringstream in(a);
				std::string        what;
				float              hgt = -1.0f;
				in >> what >> hgt;
				RE::TESObjectREFR* ref = nullptr;
				auto*              p = RE::PlayerCharacter::GetSingleton();
				if (what.rfind("nearest:", 0) == 0 && p) {
					const std::string want = what.substr(8);
					float             best = 1e30f;
					if (auto* cell = p->GetParentCell()) {
						cell->ForEachReferenceInRange(p->GetPosition(), 4000.0f, [&](RE::TESObjectREFR* r) {
							if (r && r->GetName() && want == r->GetName() && !r->IsDisabled()) {
								const float d = r->GetPosition().GetDistance(p->GetPosition());
								if (d < best) {
									best = d;
									ref = r;
								}
							}
							return RE::BSContainer::ForEachResult::kContinue;
						});
					}
				} else {
					ref = RE::TESForm::LookupByID<RE::TESObjectREFR>(static_cast<RE::FormID>(std::stoul(what, nullptr, 16)));
				}
				world::CamInfo ci;
				if (p && ref && world::CameraFull(ci)) {
					RE::NiPoint3 t = ref->GetPosition();
					t.z += hgt >= 0.0f ? hgt : 0.5f * (ref->GetBoundMax().z - ref->GetBoundMin().z);
					const RE::NiPoint3 d = t - ci.pos;
					const float        heading = std::atan2(d.x, d.y);
					const float        pitchDeg = std::atan2(-d.z, std::sqrt(d.x * d.x + d.y * d.y)) * 57.29578f;
					p->SetHeading(heading);
					logger::info("harness: aimref {:08X} '{}' at ({:.1f},{:.1f},{:.1f}), aim point ({:.1f},{:.1f},{:.1f}) {:.0f} units away: heading {:.1f}, pitch {:.1f} down", ref->GetFormID(), ref->GetName() ? ref->GetName() : "",
						ref->GetPosition().x, ref->GetPosition().y, ref->GetPosition().z, t.x, t.y, t.z, std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z), heading * 57.29578f, pitchDeg);
					PushFront({ std::format("look {:.2f}", pitchDeg) });
				} else {
					logger::warn("harness: aimref {}: reference or camera not found", what);
				}
			} else if (n == "shout") {
				// shout <hex shout id>: add the shout, equip it as the power and tap the Shout key (Unrelenting Force is 13e07; teach and unlock its words with the console first)
				auto* p = RE::PlayerCharacter::GetSingleton();
				std::istringstream shin(a);
				std::string        sid;
				int                hold = 6;
				shin >> sid >> hold;  // shout <id> [hold frames]: a tap is the first word, about 90 frames all three
				auto* sh = RE::TESForm::LookupByID<RE::TESShout>(static_cast<RE::FormID>(std::stoul(sid, nullptr, 16)));
				auto* em = RE::ActorEquipManager::GetSingleton();
				if (p && sh && em) {
					p->AddShout(sh);
					em->EquipShout(p, sh);
					logger::info("harness: shout {:#x} ({}) equipped; tapping the Shout key", sh->GetFormID(), sh->GetName());
					input::InjectShout(hold);
				} else {
					logger::warn("harness: shout {}: not found", a);
				}
			} else if (n == "jump") {
				input::InjectJump();
			} else if (n == "mousemove") {
				// mousemove <dx> <dy> <frames>: mouse look through the input queue (pan the view with a cast held)
				std::istringstream in(a);
				int dx = 0, dy = 0, frames = 30;
				in >> dx >> dy >> frames;
				input::InjectMouseMove(dx, dy, frames);
			} else if (n == "mute") {
				Mute(a != "off");
			} else if (n == "aimat") {
				// aimat blue|orange: turn the player to face the centre of that portal (angled approaches)
				const auto pt = portals::Get(IsViolet(a) ? 1 : 0);
				if (auto* p = RE::PlayerCharacter::GetSingleton(); p && pt.valid) {
					const auto d = pt.c - p->GetPosition();
					p->SetHeading(std::atan2(d.x, d.y));
					logger::info("harness: facing the portal's centre, heading {:.1f} degrees", std::atan2(d.x, d.y) * 57.29578f);
				}
			} else if (n == "turn") {
				// turn <yaw degrees>: face a compass heading (0 = north, 90 = east) with the pitch level; not a console command
				if (auto* p = RE::PlayerCharacter::GetSingleton()) {
					p->SetHeading(std::stof(a) * 0.017453292f);
					p->SetLooking(0.0f);
					logger::info("harness: turned to heading {} degrees", a);
				}
			} else if (n == "look") {
				// look <pitch degrees>: positive = down, negative = up (tests that place a portal above or below eye level)
				if (auto* p = RE::PlayerCharacter::GetSingleton()) {
					p->SetLooking(std::stof(a) * 0.017453292f);
					logger::info("harness: looking pitch {} degrees", a);
				}
			} else if (n == "travel") {
				if (a == "stats") travel::LogStats();
				else if (a == "verbose on" || a == "verbose off") travel::SetVerbose(a == "verbose on");
				else if (a.rfind("look ", 0) == 0) travel::SetLook(std::stof(a.substr(5)));
				else if (a == "relaunch on" || a == "relaunch off") travel::SetRelaunch(a == "relaunch on");
				else if (a == "coneorigin on" || a == "coneorigin off") travel::SetConeOrigin(a == "coneorigin on");
				else if (a == "conecarry on" || a == "conecarry off") travel::SetConeCarry(a == "conecarry on");
				else if (a.rfind("pushscale ", 0) == 0) travel::SetPushScale(std::stof(a.substr(10)));
				else if (a.rfind("twin ", 0) == 0) travel::SetTwinMode(a.substr(5));
				else if (a.rfind("twinpick ", 0) == 0) {
					std::istringstream in(a.substr(9));
					std::string        on;
					float              out = 4.0f;
					in >> on >> out;
					travel::SetTwinPick(on == "on", out);
				}
				else travel::SetEnabled(a != "off");
			} else if (n == "npc") {
				// npc spawn <name> | npc drive <blue|orange> [speed]
				std::istringstream in(a);
				std::string        what, who;
				in >> what >> who;
				if (what == "kill") {
					travel::KillNpc();
				} else if (what == "fling") {
					float speed = 600.0f;
					in >> speed;
					travel::FlingNpc(speed, IsViolet(who) ? 1 : 0);
				} else if (what == "setpos") {
					float y = 0, z = 0, yaw = 0;
					in >> y >> z >> yaw;
					travel::SetNpcPos(std::stof(who), y, z, yaw);
				} else if (what == "walkto") {
					float y = 0, speed = 45.0f, secs = 5.0f;
					in >> y >> speed >> secs;
					travel::WalkNpcTo(std::stof(who), y, speed, secs);
				} else if (what == "watch") {
					travel::WatchNpc(std::stof(who));
				} else if (what == "only") {
					travel::HideNpcExcept(who);
				} else if (what == "geom") {
					std::string mode;
					in >> mode;
					travel::HideNpcGeom(who, mode == "hide");
				} else if (what == "hide" || what == "show") {
					travel::HideNpc(what == "hide");
				} else if (what == "info") {
					travel::LogNpc();
				} else if (what == "put") {
					float dist = 200.0f;
					in >> dist;
					float yaw = 0.0f;
					in >> yaw;
					travel::PutNpc(IsViolet(who) ? 1 : 0, dist, yaw);
				} else if (what == "putray") {
					travel::PutNpcOnRay(std::stof(who));
				} else if (what == "run") {
					// npc run <console command>: on the spawned actor (setrestrained 0, ...)
					std::string rest;
					std::getline(in, rest);
					travel::NpcConsole(std::string(who) + rest);
				} else if (what == "hold") {
					travel::HoldNpc();
				} else if (what == "health") {
					travel::LogNpcHealth(who);
				} else if (what == "spawn") {
					travel::SpawnNpc(who);
				} else if (what == "drive") {
					float speed = 150.0f;
					in >> speed;
					travel::DriveNpc(speed, IsViolet(who) ? 1 : 0);
				}
			} else if (n == "skinlog") {
				travel::SetSkinLog(std::atoi(a.c_str()));
			} else if (n == "propdrive") {
				std::istringstream in(a);
				std::string        who;
				float              speed = 40.0f, secs = 12.0f, start = -1.0f;
				in >> who >> speed >> secs >> start;
				travel::DriveProp(speed, IsViolet(who) ? 1 : 0, secs, start);
			} else if (n == "autoshot") {
				std::istringstream in(a);
				std::string        tag = "auto";
				int                count = 12, every = 12;
				in >> tag >> count >> every;
				travel::AutoShot(tag, count, every);
			} else if (n == "passage") {
				if (a == "noclip") {
					travel::SetPassage(true);
					travel::SetClipDraw(false);
				} else {
					travel::SetClipDraw(true);
					travel::SetPassage(a != "off");
				}
			} else if (n == "spells") {
				travel::ScanSpells();
			} else if (n == "equipspell") {
				travel::EquipSpell(a);
			} else if (n == "cast") {
				std::istringstream in(a);
				std::string        what = "flames";
				float              secs = 1.5f;
				in >> what >> secs;
				travel::Cast(what, secs);
			} else if (n == "throw") {
				std::istringstream in(a);
				std::string        what = "ingot";
				float              speed = 1500.0f;
				in >> what >> speed;
				travel::Throw(what, speed);
			} else if (n == "shoot") {
				std::istringstream in(a);
				std::string        what = "arrow";
				float              speed = 4000.0f;
				in >> what >> speed;
				travel::Shoot(what, speed);
			} else if (n == "ghost") {
				if (a == "stats") ghost::LogStats();
				else if (a == "front on" || a == "front off") ghost::SetDebugFront(a == "front on");
				else ghost::SetEnabled(a != "off");
			} else if (n == "label") {
				label::Set(a == "off" ? std::string() : a);
			} else if (n == "drawhands") {
				// drawhands on|off: draw or sheathe the weapon / magic hands of the player (tests of the first-person model)
				if (auto* p = RE::PlayerCharacter::GetSingleton()) {
					p->DrawWeaponMagicHands(a != "off");
					logger::info("harness: hands {}", a != "off" ? "drawn" : "sheathed");
				}
			} else if (n == "equipspell") {
				// equipspell <hex form id> [left|right]: add the spell and put it in a hand (Flames is 12fcd)
				std::istringstream in(a);
				std::string        id, hand = "left";
				in >> id >> hand;
				auto* p = RE::PlayerCharacter::GetSingleton();
				auto* em = RE::ActorEquipManager::GetSingleton();
				auto* spell = p ? RE::TESForm::LookupByID<RE::SpellItem>(static_cast<RE::FormID>(std::stoul(id, nullptr, 16))) : nullptr;
				auto* slot = RE::TESForm::LookupByID<RE::BGSEquipSlot>(hand == "right" ? 0x00013F42u : 0x00013F43u);
				if (p && em && spell) {
					p->AddSpell(spell);
					em->EquipSpell(p, spell, slot);
					logger::info("harness: spell {:#x} ({}) equipped in the {} hand", spell->GetFormID(), spell->GetName(), hand);
				} else {
					logger::warn("harness: equipspell {}: form not found", id);
				}
			} else if (n == "givespells") {
				// givespells: add both portal spells to the player (the gift on load does the same when Riftweaver.ini GiveSpells=1)
				spells::GiveAll();
			} else if (n == "spellequip") {
				// spellequip violet|teal [left|right]: put a portal spell in a hand (default left) and draw the hands; the test then casts it with `mouse right` (left hand) or `mouse left` (right hand), held ~45 frames
				std::istringstream in(a);
				std::string        col, hand = "left";
				in >> col >> hand;
				spells::Equip(IsViolet(col) ? 1 : 0, hand == "right");
			} else if (n == "style") {
				// style particles=0|1 density=<x> alpha=<x> halo=<x> rim=<x>: the look of the portals (same keys as Riftweaver.ini) until the ini changes again; no argument = log the current values
				auto               st = spells::GetStyle();
				std::istringstream in(a);
				std::string        tok;
				while (in >> tok) {
					const auto eq = tok.find('=');
					if (eq == std::string::npos) continue;
					const auto k = tok.substr(0, eq);
					const auto v = tok.substr(eq + 1);
					try {
						if (k == "particles") st.particles = std::stoi(v) != 0;
						else if (k == "density") st.density = std::stof(v);
						else if (k == "alpha") st.alpha = std::stof(v);
						else if (k == "halo") st.halo = std::stof(v);
						else if (k == "rim") st.rim = std::stof(v);
						else if (k == "rimflow") st.rimFlow = std::stof(v);
						else if (k == "glints") st.rimGlints = std::stof(v);
						else if (k == "ripple") st.ripple = std::stof(v);
						else if (k == "sheen") st.sheen = std::stof(v);
						else if (k == "swirl") st.mistSwirl = std::stof(v);
						else if (k == "embers") st.embers = std::stof(v);
						else if (k == "light") st.light = std::stoi(v) != 0;
						else if (k == "lightradius") st.lightRadius = std::stof(v);
						else if (k == "lightintensity") st.lightIntensity = std::stof(v);
						else if (k == "openanim") st.openAnim = std::stof(v);
						else if (k == "closeanim") st.closeAnim = std::stof(v);
						else if (k == "handrunes") st.handRunes = std::stof(v);
						else if (k == "releaseflash") st.releaseFlash = std::stof(v);
						else if (k == "walkripple") st.walkRipple = std::stof(v);
						else if (k == "exitpuff") st.exitPuff = std::stof(v);
						else if (k == "sounds") st.sounds = std::stof(v);
						else if (k == "hum") st.humVolume = std::stof(v);
					} catch (...) {
						logger::warn("harness: style: bad value in '{}'", tok);
					}
				}
				spells::SetStyle(st);
			} else if (n == "spellgift") {
				if (a == "fresh") spells::RemoveAllForTest();
				spells::OnLoad(false);  // test: the same call the post-load event makes (the gift runs ~2 s later)
			} else if (n == "camstate") {
				view::DumpCamState();
			} else if (n == "spellstatus") {
				spells::LogStatus();
				fx::LogStatus();
			} else if (n == "fx") {
				// fx on|off, or fx reach <units> (the particles' limit from the rim; 30 by default, 100000 = unlimited)
				if (a.rfind("reach", 0) == 0) fx::SetReach(std::stof(a.substr(5)));
				else if (a.rfind("handcarry", 0) == 0) fx::SetHandCarry(a.find("off") == std::string::npos);
				else fx::SetEnabled(a != "off");
			} else if (n == "mouse") {
				if (a.rfind("real", 0) == 0) {
					spike::PressMouse(a.substr(5));  // SendInput (does not reach the game's mouse)
				} else {
					std::istringstream in(a);
					std::string        button;
					int                hold = 8;
					in >> button >> hold;  // "left" or "right", optionally held for that many frames
					input::InjectMouse(button, hold);
					if (a.find("nowait") == std::string::npos) {  // `mouse right 300 nowait`: the next commands (a pan) run while the button is held
						StartWait(Wait::Time, std::chrono::milliseconds(400 + 17 * (std::max)(hold - 8, 0)));
					}
				}
			} else if (n == "thirdperson") {
				camera::SetThirdPerson(a != "off");
			} else if (n == "mapmode") {
				crossing::SetMapEye(a == "eye");
			} else if (n == "vmean") {
				diag::LogViewMeans(a.empty() ? "vmean" : a);
				g_untilFrame = frame::Count() + 4;
				StartWait(Wait::Frames, std::chrono::seconds(10));
			} else if (n == "hdrdump") {
				{
					std::istringstream in(a);
					std::string        label = "hdr";
					int                stage = 0;
					in >> label >> stage;
					if (stage > 0) {
						view::RequestStageDump(label, stage);
					} else {
						diag::RequestHdrDump(label);
					}
				}
				g_untilFrame = frame::Count() + 4;
				StartWait(Wait::Frames, std::chrono::seconds(10));
			} else if (n == "parity") {
				std::istringstream in(a);
				std::string        label = "parity";
				int                count = 3;
				in >> label >> count;
				diag::RequestParity(label, count);
				g_untilFrame = frame::Count() + static_cast<std::uint64_t>(count) + 3;
				StartWait(Wait::Frames, std::chrono::seconds(10));
			} else if (n == "chain") {
				diag::SetChain(a != "off");
			} else if (n == "mvstats") {
				std::istringstream in(a);
				std::string        label = "mv", mode;
				int                count = 4;
				in >> label >> count >> mode;
				diag::RequestMv(label, count);
				if (mode != "async") {  // async: the samples are taken while the next command runs (for example a spin)
					g_untilFrame = frame::Count() + static_cast<std::uint64_t>(count) + 3;
					StartWait(Wait::Frames, std::chrono::seconds(10));
				}
			} else if (n == "cross") {
				if (a.rfind("flight", 0) == 0) crossing::SetFlightMode(std::atoi(a.c_str() + 6));
				else if (a == "stats") logger::info("crossing: {} crossing(s) so far", crossing::Count());
				else crossing::SetEnabled(a != "off");
			} else if (n == "ring") {
				std::istringstream in(a);
				std::string        tag = "ring";
				int                pre = 8, post = 16;
				in >> tag >> pre >> post;
				if (tag == "off") shot::DisarmRing();
				else shot::ArmRing(tag, pre, post);
			} else if (n == "ringtrigger") {
				shot::TriggerRing();
			} else if (n == "doorway") {
				doorway::g_enabled = (a != "off");
				logger::info("doorway: {}", a != "off" ? "ON" : "off");
			} else if (n == "roll") {
				std::istringstream in(a);
				std::string        mode, tag;
				double             secs = 0;
				in >> mode >> secs >> tag;
				crossing::SetRoll(mode != "off", secs, tag);
			} else if (n == "hole") {
				if (a == "stats") hole::LogStats();
				else if (a == "proj on" || a == "proj off") hole::SetProjectileHole(a == "proj on");
				else hole::SetEnabled(a != "off");
			} else if (n == "aimground") {
				std::istringstream in(a);
				float              x = 0, y = 0, z = 0;
				in >> x >> y >> z;
				std::vector<std::string> lines;
				if (world::AimGroundLines(x, y, z, lines)) PushFront(lines);
			} else if (n == "dropabove") {
				std::istringstream in(a);
				std::string        which;
				float              height = 600;
				in >> which >> height;
				std::vector<std::string> lines;
				if (world::DropLines(IsViolet(which) ? 1 : 0, height, lines)) {
					PushFront(lines);
				}
			} else if (n == "faceportal") {
				std::istringstream in(a);
				std::string        which;
				float              dist = 300, lateral = 0;
				in >> which >> dist >> lateral;
				std::vector<std::string> lines;
				if (world::FaceLines(IsViolet(which) ? 1 : 0, dist, lateral, lines)) {
					PushFront(lines);
				}
			} else if (n == "faceback") {
				std::istringstream in(a);
				std::string        which;
				float              dist = 300, lateral = 0;
				in >> which >> dist >> lateral;
				std::vector<std::string> lines;
				if (world::FaceBackLines(IsViolet(which) ? 1 : 0, dist, lateral, lines)) {
					PushFront(lines);
				}
			} else if (n == "notify") {
				RE::SendHUDMessage::ShowHUDMessage(a.c_str(), nullptr, true);  // on-screen text (top left), for tests that must say which build is on screen
			} else if (n == "ctrl") {
				spike::LogController();
			} else if (n == "wallinfo") {
				spike::LogWall();
			} else if (n == "focus") {
				spike::FocusGame();
			} else if (n == "track") {
				const double secs = a.empty() ? 4.0 : std::stod(a);
				spike::Track(secs);
				StartWait(Wait::Time, std::chrono::milliseconds(static_cast<int>(secs * 1000) + 400));
			} else if (n == "animtrack") {
				std::istringstream in(a);
				float              radius = 4000.0f;
				int                frames = 90;
				std::string       name;
				in >> radius >> frames >> name;
				world::AnimTrack(radius, frames, name);
			} else if (n == "findnode") {
				std::istringstream in(a);
				std::string        name;
				int                maxn = 8;
				in >> name >> maxn;
				world::FindNodes(name, maxn);
			} else if (n == "nodetrack") {
				std::istringstream in(a);
				std::string        name;
				int                frames = 60;
				in >> name >> frames;
				world::NodeTrack(name, frames);
			} else if (n == "npcs") {
				world::LogActors(a.empty() ? 1500.0f : std::stof(a));
			} else if (n == "key") {
				spike::PressKey(a);
			} else if (n == "menucheck" || n == "menuguard") {
				std::string what;
				const bool  open = spike::BlockingMenuOpen(&what);
				if (open) {
					logger::warn("harness: a blocking menu/state is open: {}", what);
				}
				if (n == "menuguard" && open) {
					spike::PressKey("esc");
					PushFront({ "sleep 1500", "menucheck strict" });
				} else if (a == "strict" && open) {
					logger::error("harness: ABORT: '{}' is still open after the exit key; clearing the queue (no test result is valid after this point)", what);
					std::lock_guard lk(g_mutex);
					g_queue.clear();
				} else if (!open) {
					logger::info("harness: no blocking menu open");
				}
			} else if (n == "strafe") {
				// strafe <left|right> <seconds>: holds A or D like walk holds W
				const bool   left = a.rfind("left", 0) == 0;
				const auto   sp = a.find(' ');
				const double secs = sp == std::string::npos ? 1.5 : std::stod(a.substr(sp + 1));
				spike::Walk(secs, left ? 0x1E : 0x20);
				StartWait(Wait::Time, std::chrono::milliseconds(static_cast<int>(secs * 1000) + 300));
			} else if (n == "walk") {
				{
					std::string what;
					if (spike::BlockingMenuOpen(&what)) {
						logger::warn("harness: walk postponed: '{}' is open; trying to close it first", what);
						PushFront({ "menuguard", "menucheck strict", "walk " + a });
						return;
					}
				}
				const double secs = a.empty() ? 4.0 : std::stod(a);
				spike::Walk(secs);
				StartWait(Wait::Time, std::chrono::milliseconds(static_cast<int>(secs * 1000) + 600));
			} else if (n == "ray") {
				world::LogCenterRay();
			} else if (n == "status") {
				world::LogStatus();
			} else if (n == "measure") {
				world::LogMeasure();
			} else if (n == "rayfan") {
				world::LogRayFan(a);
			} else if (n == "doors") {
				world::LogDoors(a.empty() ? 4000.0f : std::stof(a));
			} else if (n == "savespot") {
				world::SaveSpot(a);
			} else if (n == "spot") {
				std::vector<std::string> lines;
				if (world::SpotCommands(a, lines)) {
					PushFront(lines);
				}
			} else if (n == "place") {
				std::string why;
				const int   colour = IsViolet(a) ? 1 : 0;
				if (!portals::PlaceFromCamera(colour, why)) {
					logger::warn("harness: place {} refused: {}", colour == 0 ? "teal" : "violet", why);
				}
			} else if (n == "placelog") {
				portals::SetVerbose(a != "off");
			} else if (n == "margin") {
				portals::SetMargin(std::stof(a));
			} else if (n == "record") {
				// record start <name> [fps] [ffmpeg.exe path] | record stop (the game's own picture and sound only; files in the SKSE log folder)
				std::istringstream in(a);
				std::string        sub, name, ff = "ffmpeg";
				int                fps = 30;
				in >> sub >> name >> fps;
				std::getline(in >> std::ws, ff);
				if (ff.empty()) ff = "ffmpeg";
				if (sub == "start") record::Start(name, ff, fps);
				else record::Stop();
			} else if (n == "momentcross") {
				portals::NoteCross(IsViolet(a) ? 1 : 0);  // G310: the walk-through moment (ripple, tint, mist puff at that portal) without walking
			} else if (n == "clearportals") {
				portals::Clear();
			} else if (n == "scan") {
				portals::Scan();
			} else if (n == "portalunits") {
				std::istringstream in(a);
				float              w = 0, h = 0;
				in >> w >> h;
				portals::SetUnits(w, h);
			} else if (n == "portalfactors") {
				std::istringstream in(a);
				float              w = 2.0f, h = 2.5f;
				in >> w >> h;
				portals::SetFactors(w, h);
			} else if (n == "portalsize") {
				std::istringstream in(a);
				float              w = 88, h = 192;
				in >> w >> h;
				portals::SetSize(w * 0.5f, h * 0.5f);
			} else if (n == "overlay") {
				if (a == "on") overlay::Enable(true);
				else if (a == "off") overlay::Enable(false);
				else if (a == "depthfar on") overlay::SetDepthFar(true);
				else if (a == "depthfar off") overlay::SetDepthFar(false);
				else if (a.rfind("bias", 0) == 0) overlay::SetDepthBias(std::stof(a.substr(4)));
				else if (a.rfind("depth", 0) == 0) overlay::SetDepthMode(std::atoi(a.c_str() + 5));
			} else if (n == "depthprobe") {
				overlay::RequestProbe();
				StartWait(Wait::Frames, std::chrono::seconds(10));
				g_untilFrame = frame::Count() + 5;
			} else if (n == "view") {
				view::Configure(a);
			} else if (n == "viewshot") {
				for (int i = 0; i < 2; ++i) {
					shot::SaveResource(view::ViewTexture(i), std::format("{}_v{}", a.empty() ? "view" : a, i));
				}
			} else if (n == "posetest") {
				std::vector<std::string> lines;
				const int                colour = IsViolet(a) ? 1 : 0;
				if (world::PoseTestLines(colour, lines)) {
					PushFront(lines);
				}
			} else if (n == "assertcell") {
				auto* pl = RE::PlayerCharacter::GetSingleton();
				auto* cell = pl ? pl->GetParentCell() : nullptr;
				const auto want = static_cast<std::uint32_t>(std::stoul(a, nullptr, 16));
				if (!cell || cell->GetFormID() != want) {
					logger::error("harness: ASSERT FAILED: player cell is {:08X}, expected {:08X}; clearing the queue", cell ? cell->GetFormID() : 0u, want);
					std::lock_guard lk(g_mutex);
					g_queue.clear();
				} else {
					logger::info("harness: cell {:08X} ok", want);
				}
			} else if (n == "quit") {
				RunConsole("qqq");
			} else {
				logger::warn("harness: unknown command '{}'", n);
			}
		}

		// main thread, a few times per second
		void Tick()
		{
			g_tickQueued = false;
			for (;;) {
				if (!WaitDone()) {
					return;
				}
				Cmd c;
				{
					std::lock_guard lk(g_mutex);
					if (g_queue.empty()) {
						return;
					}
					c = std::move(g_queue.front());
					g_queue.pop_front();
				}
				Run(c);
			}
		}

		void Poll()
		{
			int tick = 0;
			while (g_run) {
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
				if (!g_tickQueued.exchange(true)) {
					SKSE::GetTaskInterface()->AddTask([] { Tick(); });
				}
				if (++tick % 4 != 0) {
					continue;
				}
				std::error_code ec;
				if (!std::filesystem::exists(g_cmdFile, ec)) {
					continue;
				}
				std::ifstream            in(g_cmdFile);
				std::vector<std::string> lines;
				for (std::string l; std::getline(in, l);) {
					while (!l.empty() && (l.back() == '\r' || l.back() == ' ')) {
						l.pop_back();
					}
					if (!l.empty() && l[0] != '#') {
						lines.push_back(l);
					}
				}
				in.close();
				std::filesystem::remove(g_cmdFile, ec);
				std::lock_guard lk(g_mutex);
				for (const auto& l : lines) {
					const auto sp = l.find(' ');
					g_queue.push_back({ l.substr(0, sp), sp == std::string::npos ? std::string{} : l.substr(sp + 1) });
				}
			}
		}
	}

	void Start()
	{
		auto dir = logger::log_directory();
		if (!dir) {
			return;
		}
		g_cmdFile = *dir / "Riftweaver.cmd";
		g_run = true;
		std::thread(Poll).detach();
		logger::info("harness: polling {}", g_cmdFile.string());
	}
}
