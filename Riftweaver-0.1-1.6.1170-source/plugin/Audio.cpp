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

#include <xaudio2.h>

// G310 phase 3: the portal sounds. Six original, procedurally rendered sounds in a teal (brighter) and a violet (darker) version (scripts/make_sounds.py), WAV files in
// Data\SKSE\Plugins\RiftweaverSounds\ next to the plugin, mixed by XAudio2 (the engine's own audio is not touched):
//   castloop  while a hand holds / charges one of our spells (a loop that fades in and out)
//   release   when the spell lands (a portal is placed), at the listener
//   open      the opening of a portal (positional: panned and attenuated by where the portal is)
//   hum       a quiet loop at each open portal, audible within about 600 units
//   walk      when the player walks through a portal
//   close     when a recast replaces a portal (positional)
// Riftweaver.ini: Sounds (0 = off, 1 = normal, up to 2 = louder), HumVolume (0 = no ambient hum).
namespace audio
{
	namespace
	{
		enum Snd
		{
			kCast,
			kRelease,
			kOpen,
			kHum,
			kWalk,
			kClose,
			kCount
		};
		const char* const kNames[kCount] = { "castloop", "release", "open", "hum", "walk", "close" };

		struct Clip
		{
			std::vector<std::int16_t> pcm;  // 44.1 kHz 16-bit mono
			bool                      ok{ false };
		};
		Clip g_clip[kCount][2];

		IXAudio2*               g_xa{ nullptr };
		IXAudio2MasteringVoice* g_master{ nullptr };
		UINT32                  g_outCh{ 2 };
		bool                    g_inited{ false };
		bool                    g_failed{ false };

		struct Loop
		{
			IXAudio2SourceVoice* v{ nullptr };
			bool                 running{ false };
			float                vol{ 0.0f };
		};
		Loop                              g_cast[2], g_hum[2];  // per colour
		std::vector<IXAudio2SourceVoice*> g_shots;
		double                            g_seenPlaced[2]{ 0, 0 };
		double                            g_seenGhost[2]{ 0, 0 };
		double                            g_seenCross{ 0 };
		double                            g_lastTick{ 0 };

		bool ReadWav(const std::filesystem::path& a_path, Clip& a_out)
		{
			std::ifstream f(a_path, std::ios::binary);
			if (!f) {
				return false;
			}
			std::vector<char> b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
			if (b.size() < 44 || std::memcmp(b.data(), "RIFF", 4) != 0 || std::memcmp(b.data() + 8, "WAVE", 4) != 0) {
				return false;
			}
			std::size_t pos = 12;
			bool        fmtOk = false;
			while (pos + 8 <= b.size()) {
				std::uint32_t len = 0;
				std::memcpy(&len, b.data() + pos + 4, 4);
				const char* id = b.data() + pos;
				const char* body = b.data() + pos + 8;
				if (std::memcmp(id, "fmt ", 4) == 0 && pos + 8 + 16 <= b.size()) {
					std::uint16_t tag, ch, bits;
					std::uint32_t rate;
					std::memcpy(&tag, body, 2);
					std::memcpy(&ch, body + 2, 2);
					std::memcpy(&rate, body + 4, 4);
					std::memcpy(&bits, body + 14, 2);
					fmtOk = tag == 1 && ch == 1 && rate == 44100 && bits == 16;
				} else if (std::memcmp(id, "data", 4) == 0 && fmtOk && pos + 8 + len <= b.size()) {
					a_out.pcm.resize(len / 2);
					std::memcpy(a_out.pcm.data(), body, len & ~1u);
					return !a_out.pcm.empty();
				}
				pos += 8 + len + (len & 1);
			}
			return false;
		}

		bool Init()
		{
			if (g_inited || g_failed) {
				return g_inited;
			}
			HMODULE self = nullptr;
			if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCWSTR>(&Init), &self)) {
				g_failed = true;
				return false;
			}
			wchar_t       buf[MAX_PATH]{};
			const DWORD   n = GetModuleFileNameW(self, buf, MAX_PATH);
			const auto    dir = std::filesystem::path(std::wstring(buf, n)).parent_path() / L"RiftweaverSounds";
			int           loaded = 0;
			for (int s = 0; s < kCount; ++s) {
				for (int c = 0; c < 2; ++c) {
					const auto file = dir / std::format(L"{}_{}.wav", std::wstring(kNames[s], kNames[s] + std::strlen(kNames[s])), c == 0 ? L"teal" : L"violet");
					g_clip[s][c].ok = ReadWav(file, g_clip[s][c]);
					loaded += g_clip[s][c].ok ? 1 : 0;
				}
			}
			if (loaded == 0) {
				logger::info("audio: no sounds in {}: no portal sounds", dir.string());
				g_failed = true;
				return false;
			}
			CoInitializeEx(nullptr, COINIT_MULTITHREADED);  // fine when the thread already has a mode
			if (FAILED(XAudio2Create(&g_xa, 0, XAUDIO2_DEFAULT_PROCESSOR)) || !g_xa || FAILED(g_xa->CreateMasteringVoice(&g_master)) || !g_master) {
				logger::warn("audio: XAudio2 could not be started: no portal sounds");
				g_failed = true;
				return false;
			}
			XAUDIO2_VOICE_DETAILS d{};
			g_master->GetVoiceDetails(&d);
			g_outCh = (std::max)(d.InputChannels, 2u);
			g_inited = true;
			logger::info("audio: portal sounds ready ({} of {} clips loaded from {}, {} output channels)", loaded, kCount * 2, dir.string(), g_outCh);
			return true;
		}

		IXAudio2SourceVoice* MakeVoice()
		{
			WAVEFORMATEX fmt{};
			fmt.wFormatTag = WAVE_FORMAT_PCM;
			fmt.nChannels = 1;
			fmt.nSamplesPerSec = 44100;
			fmt.wBitsPerSample = 16;
			fmt.nBlockAlign = 2;
			fmt.nAvgBytesPerSec = 88200;
			IXAudio2SourceVoice* v = nullptr;
			if (FAILED(g_xa->CreateSourceVoice(&v, &fmt)) || !v) {
				return nullptr;
			}
			return v;
		}

		void SetGain(IXAudio2SourceVoice* a_v, float a_gain, float a_pan)
		{
			const float pan = std::clamp(a_pan, -1.0f, 1.0f);
			float       m[8] = {};
			m[0] = std::sqrt(0.5f * (1.0f - pan));
			m[1] = std::sqrt(0.5f * (1.0f + pan));
			a_v->SetVolume(a_gain);
			a_v->SetOutputMatrix(g_master, 1, g_outCh, m);
		}

		bool Submit(IXAudio2SourceVoice* a_v, const Clip& a_clip, bool a_loop)
		{
			XAUDIO2_BUFFER b{};
			b.AudioBytes = static_cast<UINT32>(a_clip.pcm.size() * 2);
			b.pAudioData = reinterpret_cast<const BYTE*>(a_clip.pcm.data());
			b.Flags = a_loop ? 0 : XAUDIO2_END_OF_STREAM;
			b.LoopCount = a_loop ? XAUDIO2_LOOP_INFINITE : 0;
			return SUCCEEDED(a_v->SubmitSourceBuffer(&b)) && SUCCEEDED(a_v->Start());
		}

		// listener-relative pan (-1 left .. 1 right) and distance of a world point
		void Where(const world::CamInfo& a_cam, const RE::NiPoint3& a_p, float& a_pan, float& a_dist, bool& a_behind)
		{
			const RE::NiPoint3 d = a_p - a_cam.pos;
			const float        x = d.x * a_cam.right.x + d.y * a_cam.right.y + d.z * a_cam.right.z;
			const float        z = d.x * a_cam.fwd.x + d.y * a_cam.fwd.y + d.z * a_cam.fwd.z;
			a_dist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
			a_pan = std::sin(std::atan2(x, z));
			a_behind = z < 0.0f;
		}

		void PlayShot(int a_snd, int a_colour, float a_gain, float a_pan)
		{
			const Clip& c = g_clip[a_snd][a_colour & 1];
			if (!c.ok || g_shots.size() > 24) {
				return;
			}
			if (auto* v = MakeVoice()) {
				SetGain(v, a_gain, a_pan);
				if (Submit(v, c, false)) {
					static int s_logged = 0;
					if (s_logged++ < 60) {
						logger::info("audio: {} {} (gain {:.2f}, pan {:+.2f}, {} voice(s) playing)", kNames[a_snd], a_colour == 0 ? "teal" : "violet", a_gain, a_pan, g_shots.size() + 1);
					}
					g_shots.push_back(v);
				} else {
					v->DestroyVoice();
				}
			}
		}

		void ReapShots()
		{
			for (std::size_t i = 0; i < g_shots.size();) {
				XAUDIO2_VOICE_STATE s{};
				g_shots[i]->GetState(&s, XAUDIO2_VOICE_NOSAMPLESPLAYED);
				if (s.BuffersQueued == 0) {
					g_shots[i]->DestroyVoice();
					g_shots.erase(g_shots.begin() + static_cast<std::ptrdiff_t>(i));
				} else {
					++i;
				}
			}
		}

		// a loop voice that follows a target gain: starts when the gain rises above zero, stops once it has faded out
		void Drive(Loop& a_l, int a_snd, int a_colour, float a_target, float a_pan, float a_dt, float a_rate)
		{
			const Clip& c = g_clip[a_snd][a_colour & 1];
			if (!c.ok) {
				return;
			}
			if (a_target > a_l.vol) {
				a_l.vol = (std::min)(a_target, a_l.vol + a_rate * a_dt);
			} else {
				a_l.vol = (std::max)(a_target, a_l.vol - a_rate * a_dt);
			}
			if (a_l.vol > 0.002f) {
				if (!a_l.v) {
					a_l.v = MakeVoice();
				}
				if (a_l.v && !a_l.running) {
					a_l.v->FlushSourceBuffers();
					a_l.running = Submit(a_l.v, c, true);
					static int s_logged = 0;
					if (s_logged++ < 40) {
						logger::info("audio: loop {} {} started", kNames[a_snd], a_colour == 0 ? "teal" : "violet");
					}
				}
				if (a_l.v && a_l.running) {
					SetGain(a_l.v, a_l.vol, a_pan);
				}
			} else if (a_l.v && a_l.running) {
				a_l.v->Stop();
				a_l.v->FlushSourceBuffers();
				a_l.running = false;
			}
		}

		void StopAll()
		{
			for (auto* l : { &g_cast[0], &g_cast[1], &g_hum[0], &g_hum[1] }) {
				if (l->v && l->running) {
					l->v->Stop();
					l->v->FlushSourceBuffers();
					l->running = false;
				}
				l->vol = 0.0f;
			}
			for (auto* v : g_shots) {
				v->DestroyVoice();
			}
			g_shots.clear();
		}
	}

	void Tick()
	{
		if (g_failed) {
			return;
		}
		const auto st = spells::GetStyle();
		if (st.sounds <= 0.001f) {
			if (g_inited) {
				StopAll();
			}
			return;
		}
		if (!g_inited && !Init()) {
			return;
		}
		const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
		const float  dt = g_lastTick > 0.0 ? static_cast<float>(std::clamp(now - g_lastTick, 0.0, 0.1)) : 0.0f;
		g_lastTick = now;
		auto*      ui = RE::UI::GetSingleton();
		const bool silent = !world::Ready() || (ui && ui->GameIsPaused());
		g_master->SetVolume(silent ? 0.0f : std::clamp(st.sounds, 0.0f, 2.0f));
		world::CamInfo cam;
		if (!world::CameraFull(cam)) {
			return;
		}
		ReapShots();
		spells::HandFx hands[2];
		spells::GetHandFx(hands);
		const portals::Portal ps[2] = { portals::Get(0), portals::Get(1) };
		// the cast hold loop: one per colour while a hand holds that spell
		for (int c = 0; c < 2; ++c) {
			bool up = false;
			for (int h = 0; h < 2; ++h) {
				up = up || (hands[h].active && hands[h].colour == c && now - hands[h].at < 0.25);
			}
			Drive(g_cast[c], kCast, c, up ? 0.5f : 0.0f, 0.0f, dt, up ? 6.0f : 7.0f);
		}
		// a new portal: the release at the hand and the opening at the portal
		for (int c = 0; c < 2; ++c) {
			if (!ps[c].valid) {
				g_seenPlaced[c] = 0.0;
				continue;
			}
			if (ps[c].placedAtSec != g_seenPlaced[c]) {
				const bool fresh = now - ps[c].placedAtSec < 1.0 && g_seenPlaced[c] != 0.0 ? true : (now - ps[c].placedAtSec < 1.0);
				g_seenPlaced[c] = ps[c].placedAtSec;
				if (fresh) {
					float pan, dist;
					bool  behind;
					Where(cam, ps[c].c, pan, dist, behind);
					const float att = std::pow(std::clamp(1.0f - dist / 2000.0f, 0.12f, 1.0f), 1.5f) * (behind ? 0.8f : 1.0f);
					PlayShot(kRelease, c, 0.75f, 0.0f);
					PlayShot(kOpen, c, 0.95f * att, pan);
				}
			}
		}
		// a replaced portal closing
		for (int c = 0; c < 2; ++c) {
			const auto g = portals::GetGhost(c);
			if (g.p.valid && g.closedAtSec != g_seenGhost[c]) {
				g_seenGhost[c] = g.closedAtSec;
				float pan, dist;
				bool  behind;
				Where(cam, g.p.c, pan, dist, behind);
				const float att = std::pow(std::clamp(1.0f - dist / 2000.0f, 0.12f, 1.0f), 1.5f) * (behind ? 0.8f : 1.0f);
				PlayShot(kClose, c, 0.95f * att, pan);
			}
		}
		// the walk through
		{
			const auto cross = portals::LastCross();
			if (cross.at != g_seenCross) {
				g_seenCross = cross.at;
				if (cross.at > 0.0 && now - cross.at < 1.0 && cross.exitColour >= 0) {
					PlayShot(kWalk, cross.exitColour, 0.9f, 0.0f);
				}
			}
		}
		// the ambient hum of each open portal, audible within about 600 units
		for (int c = 0; c < 2; ++c) {
			float target = 0.0f, pan = 0.0f;
			if (ps[c].valid && st.humVolume > 0.001f && now - ps[c].placedAtSec > 0.3) {
				float dist;
				bool  behind;
				Where(cam, ps[c].c, pan, dist, behind);
				const float a = std::clamp(1.0f - dist / 600.0f, 0.0f, 1.0f);
				target = 0.32f * st.humVolume * a * a * (behind ? 0.85f : 1.0f);
			}
			Drive(g_hum[c], kHum, c, target, pan, dt, 1.5f);
		}
	}
}
