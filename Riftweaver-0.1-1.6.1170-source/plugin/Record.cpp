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

#include <audioclient.h>
#include <audioclientactivationparams.h>
#include <d3d11.h>
#include <dxgi.h>
#include <mmdeviceapi.h>
#include <wrl/client.h>
#include <wrl/implements.h>

// G310: records the game's own output for the final video, and nothing else on the PC:
//   video  the swap chain's back buffer, copied at Present (GPU copy into 3 staging textures, read two frames later), written as raw RGBA to an ffmpeg child process (stdin) at a constant frame rate
//          (frames the game was late for repeat the previous picture), encoded to <name>_video.mkv
//   audio  WASAPI PROCESS LOOPBACK of this process (Windows 10 2004+): only what the game itself plays, the engine's sounds and the portal sounds, never another program; float32 stereo 48 kHz,
//          aligned to the first video frame by the packets' QPC time, written to <name>_audio.wav
// The two are muxed afterwards by scripts/mux_video.sh. Started and stopped by the harness (record start <name> [fps] [ffmpeg.exe] / record stop); not part of normal play.
namespace record
{
	using Microsoft::WRL::ComPtr;
	namespace
	{
		struct Frame
		{
			std::vector<std::uint8_t> rgba;
			std::int64_t              slot{ 0 };
		};

		std::atomic<bool>       g_wanted{ false };   // record start was asked, the pipeline starts at the next Present
		std::atomic<bool>       g_active{ false };
		std::string             g_name;
		std::string             g_ffmpeg;
		int                     g_fps{ 30 };
		std::filesystem::path   g_dir;

		// video
		ComPtr<ID3D11Texture2D> g_stage[3];
		std::int64_t            g_slotOf[3]{ -1, -1, -1 };
		int                     g_cur{ 0 };
		UINT                    g_w{ 0 }, g_h{ 0 };
		std::int64_t            g_nextSlot{ 0 };
		std::int64_t            g_t0Qpc{ 0 };       // QueryPerformanceCounter at the first captured frame
		std::int64_t            g_freq{ 1 };
		HANDLE                  g_proc{ nullptr }, g_stdin{ nullptr };
		std::mutex              g_qMutex;
		std::condition_variable g_qCv;
		std::deque<Frame>       g_queue;
		std::vector<std::vector<std::uint8_t>> g_pool;
		std::thread             g_writer;
		std::atomic<bool>       g_writerStop{ false };
		std::atomic<int>        g_dropped{ 0 };
		std::int64_t            g_framesWritten{ 0 };

		// audio
		std::thread             g_audioThread;
		std::atomic<bool>       g_audioStop{ false };
		std::atomic<std::int64_t> g_t0Units{ 0 };    // the first video frame in 100 ns units of the QPC clock (what the audio packets are stamped with)
		std::atomic<bool>       g_audioOk{ false };
		std::int64_t            g_audioSamples{ 0 };

		void WriterImpl();
		void WriterMain()
		{
			try {
				WriterImpl();
			} catch (const std::exception& e) {
				logger::error("record: the video writer stopped on an exception: {}", e.what());
			}
		}
		void WriterImpl()
		{
			std::vector<std::uint8_t> last;
			std::int64_t              lastSlot = -1;
			auto write = [&](const std::vector<std::uint8_t>& a_buf) {
				DWORD done = 0;
				const BYTE* p = a_buf.data();
				std::size_t left = a_buf.size();
				while (left > 0) {
					if (!WriteFile(g_stdin, p, static_cast<DWORD>((std::min<std::size_t>)(left, 1u << 20)), &done, nullptr) || done == 0) {
						return false;
					}
					p += done;
					left -= done;
				}
				++g_framesWritten;
				return true;
			};
			for (;;) {
				Frame f;
				{
					std::unique_lock lk(g_qMutex);
					g_qCv.wait(lk, [] { return !g_queue.empty() || g_writerStop.load(); });
					if (g_queue.empty()) {
						return;
					}
					f = std::move(g_queue.front());
					g_queue.pop_front();
				}
				bool ok = true;
				while (ok && !last.empty() && lastSlot + 1 < f.slot) {  // frames the game was late for: the previous picture again
					ok = write(last);
					++lastSlot;
				}
				ok = ok && write(f.rgba);
				lastSlot = f.slot;
				{
					std::lock_guard lk(g_qMutex);
					g_pool.push_back(std::move(last));
				}
				last = std::move(f.rgba);
				if (!ok) {
					logger::warn("record: writing to ffmpeg failed; stopping the video");
					return;
				}
			}
		}

		bool SpawnFfmpeg(UINT a_w, UINT a_h)
		{
			SECURITY_ATTRIBUTES sa{ sizeof(sa), nullptr, TRUE };
			HANDLE              rd = nullptr, wr = nullptr;
			if (!CreatePipe(&rd, &wr, &sa, 64u << 20)) {
				return false;
			}
			SetHandleInformation(wr, HANDLE_FLAG_INHERIT, 0);
			const auto video = (g_dir / (g_name + "_video.mkv")).string();
			const auto cmd = std::format("\"{}\" -y -loglevel error -f rawvideo -pix_fmt rgba -video_size {}x{} -framerate {} -i - -vf scale=1280:720:flags=lanczos -c:v libx264 -preset veryfast -crf 14 -pix_fmt yuv420p \"{}\"",
				g_ffmpeg, a_w, a_h, g_fps, video);
			STARTUPINFOA si{};
			si.cb = sizeof(si);
			si.dwFlags = STARTF_USESTDHANDLES;
			si.hStdInput = rd;
			si.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
			si.hStdError = GetStdHandle(STD_ERROR_HANDLE);
			PROCESS_INFORMATION pi{};
			std::string         mut = cmd;
			const BOOL          ok = CreateProcessA(nullptr, mut.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi);
			CloseHandle(rd);
			if (!ok) {
				CloseHandle(wr);
				logger::error("record: cannot start ffmpeg ({}): error {}", g_ffmpeg, GetLastError());
				return false;
			}
			CloseHandle(pi.hThread);
			g_proc = pi.hProcess;
			g_stdin = wr;
			return true;
		}

		// ---- WASAPI process loopback ----
		class ActivateHandler : public Microsoft::WRL::RuntimeClass<Microsoft::WRL::RuntimeClassFlags<Microsoft::WRL::ClassicCom>, Microsoft::WRL::FtmBase, IActivateAudioInterfaceCompletionHandler>
		{
		public:
			HANDLE               done{ CreateEventW(nullptr, TRUE, FALSE, nullptr) };
			HRESULT              result{ E_FAIL };
			ComPtr<IAudioClient> client;
			~ActivateHandler() { CloseHandle(done); }
			STDMETHOD(ActivateCompleted)(IActivateAudioInterfaceAsyncOperation* a_op) override
			{
				HRESULT         hr = E_FAIL;
				ComPtr<IUnknown> unk;
				if (SUCCEEDED(a_op->GetActivateResult(&hr, &unk)) && SUCCEEDED(hr) && unk) {
					unk.As(&client);
					result = client ? S_OK : E_NOINTERFACE;
				} else {
					result = FAILED(hr) ? hr : E_FAIL;
				}
				SetEvent(done);
				return S_OK;
			}
		};

		void AudioImpl(const std::string& a_wavPath);
		void AudioMain(std::string a_wavPath)
		{
			try {
				AudioImpl(a_wavPath);
			} catch (const std::exception& e) {
				logger::error("record: the audio thread stopped on an exception: {}", e.what());
			}
		}
		void AudioImpl(const std::string& a_wavPath)
		{
			CoInitializeEx(nullptr, COINIT_MULTITHREADED);
			AUDIOCLIENT_ACTIVATION_PARAMS ac{};
			ac.ActivationType = AUDIOCLIENT_ACTIVATION_TYPE_PROCESS_LOOPBACK;
			ac.ProcessLoopbackParams.TargetProcessId = GetCurrentProcessId();
			ac.ProcessLoopbackParams.ProcessLoopbackMode = PROCESS_LOOPBACK_MODE_INCLUDE_TARGET_PROCESS_TREE;
			PROPVARIANT pv{};
			pv.vt = VT_BLOB;
			pv.blob.cbSize = sizeof(ac);
			pv.blob.pBlobData = reinterpret_cast<BYTE*>(&ac);
			auto                                        handler = Microsoft::WRL::Make<ActivateHandler>();
			ComPtr<IActivateAudioInterfaceAsyncOperation> op;
			HRESULT hr = ActivateAudioInterfaceAsync(VIRTUAL_AUDIO_DEVICE_PROCESS_LOOPBACK, __uuidof(IAudioClient), &pv, handler.Get(), &op);
			if (FAILED(hr) || WaitForSingleObject(handler->done, 5000) != WAIT_OBJECT_0 || FAILED(handler->result)) {
				logger::error("record: process loopback audio could not be activated (hr {:#x}, result {:#x}): no audio in the recording", static_cast<unsigned>(hr), static_cast<unsigned>(handler->result));
				return;
			}
			ComPtr<IAudioClient> client = handler->client;
			WAVEFORMATEX         fmt{};
			fmt.wFormatTag = WAVE_FORMAT_IEEE_FLOAT;
			fmt.nChannels = 2;
			fmt.nSamplesPerSec = 48000;
			fmt.wBitsPerSample = 32;
			fmt.nBlockAlign = 8;
			fmt.nAvgBytesPerSec = 48000 * 8;
			HANDLE ev = CreateEventW(nullptr, FALSE, FALSE, nullptr);
			hr = client->Initialize(AUDCLNT_SHAREMODE_SHARED, AUDCLNT_STREAMFLAGS_LOOPBACK | AUDCLNT_STREAMFLAGS_EVENTCALLBACK | AUDCLNT_STREAMFLAGS_AUTOCONVERTPCM | AUDCLNT_STREAMFLAGS_SRC_DEFAULT_QUALITY, 2000000, 0, &fmt, nullptr);
			ComPtr<IAudioCaptureClient> cap;
			if (FAILED(hr) || FAILED(client->SetEventHandle(ev)) || FAILED(client->GetService(IID_PPV_ARGS(&cap))) || FAILED(client->Start())) {
				logger::error("record: process loopback audio could not be started (hr {:#x}): no audio in the recording", static_cast<unsigned>(hr));
				CloseHandle(ev);
				return;
			}
			g_audioOk = true;
			std::vector<float> pcm;  // interleaved stereo from the first video frame on
			std::int64_t       written = 0;  // frames
			bool               first = true;
			std::int64_t       prevQpc = 0, prevFrames = 0;
			while (!g_audioStop.load()) {
				WaitForSingleObject(ev, 100);
				for (;;) {
					UINT32 n = 0;
					if (FAILED(cap->GetNextPacketSize(&n)) || n == 0) {
						break;
					}
					BYTE*   data = nullptr;
					UINT32  frames = 0;
					DWORD   flags = 0;
					UINT64  devPos = 0, qpc = 0;
					if (FAILED(cap->GetBuffer(&data, &frames, &flags, &devPos, &qpc))) {
						break;
					}
					const std::int64_t t0 = g_t0Units.load();
					if (t0 > 0 && frames > 0) {
						// where this packet starts, in frames since the first video frame: taken from the packet's QPC stamp for the first packet (a lead-in of silence, or the part before the first video frame is dropped),
						// then the stream is contiguous except for gaps the stamps show (nothing was playing); anything beyond a few seconds is a stamp error and is ignored
						std::int64_t skip = 0, pad = 0;
						if (first) {
							first = false;
							if (qpc && !(flags & AUDCLNT_BUFFERFLAGS_TIMESTAMP_ERROR)) {
								const std::int64_t off = (static_cast<std::int64_t>(qpc) - t0) * 48000 / 10000000;
								if (off < 0 && off > -96000) skip = -off;
								else if (off > 0 && off < 240000) pad = off;
							}
						} else if (qpc && prevQpc) {
							const std::int64_t expected = prevQpc + prevFrames * 10000000 / 48000;
							const std::int64_t gap = (static_cast<std::int64_t>(qpc) - expected) * 48000 / 10000000;
							if (gap > 480 && gap < 240000) pad = gap;
						}
						prevQpc = static_cast<std::int64_t>(qpc);
						prevFrames = frames;
						if (pad > 0) {
							pcm.insert(pcm.end(), static_cast<std::size_t>(pad * 2), 0.0f);
							written += pad;
						}
						if (skip < static_cast<std::int64_t>(frames)) {
							const std::int64_t take = static_cast<std::int64_t>(frames) - skip;
							if (flags & AUDCLNT_BUFFERFLAGS_SILENT) {
								pcm.insert(pcm.end(), static_cast<std::size_t>(take * 2), 0.0f);
							} else {
								const float* src = reinterpret_cast<const float*>(data) + skip * 2;
								pcm.insert(pcm.end(), src, src + take * 2);
							}
							written += take;
						}
					}
					cap->ReleaseBuffer(frames);
				}
			}
			client->Stop();
			CloseHandle(ev);
			// the WAV: 32-bit float stereo 48 kHz
			if (std::ofstream f(a_wavPath, std::ios::binary); f) {
				const std::uint32_t bytes = static_cast<std::uint32_t>(pcm.size() * 4);
				auto                w32 = [&](std::uint32_t v) { f.write(reinterpret_cast<const char*>(&v), 4); };
				auto                w16 = [&](std::uint16_t v) { f.write(reinterpret_cast<const char*>(&v), 2); };
				f.write("RIFF", 4);
				w32(36 + bytes);
				f.write("WAVEfmt ", 8);
				w32(16);
				w16(3);
				w16(2);
				w32(48000);
				w32(48000 * 8);
				w16(8);
				w16(32);
				f.write("data", 4);
				w32(bytes);
				f.write(reinterpret_cast<const char*>(pcm.data()), bytes);
			}
			g_audioSamples = written;
		}

		ID3D11Device* Device() { return reinterpret_cast<ID3D11Device*>(RE::BSGraphics::Renderer::GetDevice()); }

		std::int64_t Qpc()
		{
			LARGE_INTEGER v;
			QueryPerformanceCounter(&v);
			return v.QuadPart;
		}

		// reads a staging texture into a pooled buffer (rows without the pitch padding)
		bool Readback(ID3D11DeviceContext* a_ctx, int a_idx, Frame& a_out)
		{
			D3D11_MAPPED_SUBRESOURCE m{};
			if (FAILED(a_ctx->Map(g_stage[a_idx].Get(), 0, D3D11_MAP_READ, 0, &m))) {
				return false;
			}
			{
				std::lock_guard lk(g_qMutex);
				if (!g_pool.empty()) {
					a_out.rgba = std::move(g_pool.back());
					g_pool.pop_back();
				}
			}
			a_out.rgba.resize(static_cast<std::size_t>(g_w) * g_h * 4);
			for (UINT y = 0; y < g_h; ++y) {
				std::memcpy(a_out.rgba.data() + static_cast<std::size_t>(y) * g_w * 4, static_cast<const std::uint8_t*>(m.pData) + static_cast<std::size_t>(y) * m.RowPitch, static_cast<std::size_t>(g_w) * 4);
			}
			a_ctx->Unmap(g_stage[a_idx].Get(), 0);
			a_out.slot = g_slotOf[a_idx];
			return true;
		}

		void Enqueue(Frame&& a_f)
		{
			std::lock_guard lk(g_qMutex);
			if (g_queue.size() >= 12) {
				++g_dropped;
				return;
			}
			g_queue.push_back(std::move(a_f));
			g_qCv.notify_one();
		}

		void Finish(ID3D11DeviceContext* a_ctx)
		{
			// the frames still in the staging ring, oldest first
			for (int k = 1; k <= 3; ++k) {
				const int idx = (g_cur + k - 1) % 3;
				if (g_slotOf[idx] >= 0 && g_stage[idx]) {
					Frame f;
					if (Readback(a_ctx, idx, f)) {
						Enqueue(std::move(f));
					}
					g_slotOf[idx] = -1;
				}
			}
		}
	}

	bool Start(const std::string& a_name, const std::string& a_ffmpeg, int a_fps)
	{
		if (g_active.load() || g_wanted.load()) {
			return false;
		}
		if (auto d = logger::log_directory()) {
			g_dir = *d;
		}
		g_name = a_name;
		g_ffmpeg = a_ffmpeg;
		g_fps = (std::clamp)(a_fps, 10, 60);
		g_wanted = true;
		logger::info("record: start requested ('{}', {} fps, ffmpeg {}); the pipeline starts at the next presented frame", a_name, g_fps, a_ffmpeg);
		return true;
	}

	bool Active()
	{
		return g_active.load() || g_wanted.load();
	}

	void OnPresentImpl(void* a_swapChain);
	void OnPresent(void* a_swapChain)
	{
		if (!g_active.load() && !g_wanted.load()) {
			return;
		}
		try {
			OnPresentImpl(a_swapChain);
		} catch (const std::exception& e) {
			logger::error("record: exception while capturing a frame ({}): recording stopped", e.what());
			g_wanted = false;
			g_active = false;
		}
	}

	void OnPresentImpl(void* a_swapChain)
	{
		auto*                   sc = static_cast<IDXGISwapChain*>(a_swapChain);
		ComPtr<ID3D11Texture2D> bb;
		if (FAILED(sc->GetBuffer(0, IID_PPV_ARGS(&bb)))) {
			return;
		}
		D3D11_TEXTURE2D_DESC d{};
		bb->GetDesc(&d);
		ComPtr<ID3D11DeviceContext> ctx;
		Device()->GetImmediateContext(&ctx);
		LARGE_INTEGER fq;
		QueryPerformanceFrequency(&fq);
		g_freq = fq.QuadPart;
		if (g_wanted.exchange(false)) {
			if (d.Format != DXGI_FORMAT_R8G8B8A8_UNORM && d.Format != DXGI_FORMAT_R8G8B8A8_UNORM_SRGB) {
				logger::error("record: the back buffer format {} is not RGBA8: not recording", static_cast<int>(d.Format));
				return;
			}
			g_w = d.Width;
			g_h = d.Height;
			D3D11_TEXTURE2D_DESC sd = d;
			sd.Usage = D3D11_USAGE_STAGING;
			sd.BindFlags = 0;
			sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
			sd.MiscFlags = 0;
			sd.MipLevels = 1;
			sd.ArraySize = 1;
			for (int i = 0; i < 3; ++i) {
				g_stage[i].Reset();
				g_slotOf[i] = -1;
				if (FAILED(Device()->CreateTexture2D(&sd, nullptr, &g_stage[i]))) {
					logger::error("record: cannot create the staging textures");
					return;
				}
			}
			if (!SpawnFfmpeg(g_w, g_h)) {
				return;
			}
			g_queue.clear();
			g_dropped = 0;
			g_framesWritten = 0;
			g_nextSlot = 0;
			g_cur = 0;
			g_writerStop = false;
			g_writer = std::thread(WriterMain);
			g_t0Qpc = Qpc();
			g_t0Units = g_t0Qpc * 10000000 / g_freq;
			g_audioStop = false;
			g_audioOk = false;
			g_audioSamples = 0;
			g_audioThread = std::thread(AudioMain, (g_dir / (g_name + "_audio.wav")).string());
			g_active = true;
			logger::info("record: recording {}x{} at {} fps to {}_video.mkv (+ {}_audio.wav from this process only)", g_w, g_h, g_fps, g_name, g_name);
		}
		if (!g_active.load() || d.Width != g_w || d.Height != g_h) {
			return;
		}
		const std::int64_t slot = (Qpc() - g_t0Qpc) * g_fps / g_freq;
		if (slot < g_nextSlot) {
			return;  // not due yet
		}
		g_nextSlot = slot + 1;
		// read the oldest frame in the ring (copied two presents ago, so the GPU has finished it), then replace it by this one
		if (g_slotOf[g_cur] >= 0) {
			Frame f;
			if (Readback(ctx.Get(), g_cur, f)) {
				Enqueue(std::move(f));
			}
		}
		ctx->CopyResource(g_stage[g_cur].Get(), bb.Get());
		g_slotOf[g_cur] = slot;
		g_cur = (g_cur + 1) % 3;
	}

	void Stop()
	{
		g_wanted = false;
		if (!g_active.exchange(false)) {
			return;
		}
		ComPtr<ID3D11DeviceContext> ctx;
		Device()->GetImmediateContext(&ctx);
		Finish(ctx.Get());
		g_writerStop = true;
		g_qCv.notify_all();
		if (g_writer.joinable()) {
			g_writer.join();
		}
		if (g_stdin) {
			CloseHandle(g_stdin);  // end of input: ffmpeg finishes the file
			g_stdin = nullptr;
		}
		if (g_proc) {
			WaitForSingleObject(g_proc, 30000);
			CloseHandle(g_proc);
			g_proc = nullptr;
		}
		g_audioStop = true;
		if (g_audioThread.joinable()) {
			g_audioThread.join();
		}
		const std::int64_t secs = g_t0Qpc ? (Qpc() - g_t0Qpc) / g_freq : 0;
		logger::info("record: stopped: {} video frames written ({} dropped by a full queue), about {} s; audio {} ({:.2f} s)", g_framesWritten, g_dropped.load(), secs, g_audioOk.load() ? "recorded" : "NOT recorded", g_audioSamples / 48000.0);
		for (auto& t : g_stage) {
			t.Reset();
		}
	}
}
