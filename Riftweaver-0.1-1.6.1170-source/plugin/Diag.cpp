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

#include <d3d11_4.h>
#include <wrl/client.h>
#include <DirectXPackedVector.h>

// Engine-state diagnostics for the ghosting investigation (16:56Z: "when the second portal pops, objects around start ghosting").
// 1. chain: the engine's temporal anti-aliasing and motion vectors use the previous frame's camera matrices. Does the main camera's entry still hold
//    "last frame's view-projection" as its previous matrix once the virtual camera is built in the same frame?
// 2. mv: reads the motion-vector target back at the end of the world pass (before the overlay) and logs its statistics.
namespace diag
{
	using Microsoft::WRL::ComPtr;

	namespace
	{
		std::atomic<bool> g_chainOn{ false };

		struct Slot
		{
			float         lastVpu[16]{};
			bool          have{ false };
			void*         entry{ nullptr };
			int           n{ 0 }, broken{ 0 }, logged{ 0 }, moved{ 0 };
			double        maxRelPrev{ 0 }, sumRelCur{ 0 };
		};
		Slot g_slot[4];  // main (no jitter), main (jitter), virtual (no jitter), virtual (jitter)

		float MaxAbs(const float* a)
		{
			float m = 0;
			for (int i = 0; i < 16; ++i) m = (std::max)(m, std::fabs(a[i]));
			return m;
		}
		float MaxDiff(const float* a, const float* b)
		{
			float m = 0;
			for (int i = 0; i < 16; ++i) m = (std::max)(m, std::fabs(a[i] - b[i]));
			return m;
		}

		bool CacheInfo(const void* a_cam, bool a_jitter, std::uint32_t& a_size, bool& a_found, void*& a_foundPtr)
		{
			__try {
				auto* st = RE::BSGraphics::State::GetSingleton();
				a_size = st->GetRuntimeData().cameraDataCacheA.size();
				auto* f = st->FindCameraDataCache(static_cast<const RE::NiCamera*>(a_cam), a_jitter);
				a_found = f != nullptr;
				a_foundPtr = f;
				return true;
			} __except (EXCEPTION_EXECUTE_HANDLER) {
				return false;
			}
		}

		std::atomic<int> g_parityLeft{ 0 };
		std::string      g_parityLabel;
		std::atomic<int> g_mvLeft{ 0 };
		std::string      g_mvLabel;
		int              g_mvTaken{ 0 };
	}

	void SetChain(bool a_on)
	{
		g_chainOn = a_on;
		if (a_on) {
			for (auto& s : g_slot) s = Slot{};
		}
		logger::info("diag: chain logging {}", a_on ? "on" : "off");
	}

	void OnBuild(void* a_entry, const void* a_cam, bool a_jitter, bool a_main, bool a_virtual)
	{
		if (!g_chainOn.load(std::memory_order_relaxed) || (!a_main && !a_virtual)) {
			return;
		}
		auto*       e = static_cast<std::uint8_t*>(a_entry);
		const float* vpu = reinterpret_cast<const float*>(e + 0x140);   // view-projection, unjittered
		const float* prev = reinterpret_cast<const float*>(e + 0x180);  // the same, previous frame (ViewData +0x170 inside the entry's +0x10)
		Slot&       s = g_slot[(a_virtual ? 2 : 0) + (a_jitter ? 1 : 0)];
		const float scale = (std::max)(MaxAbs(vpu), 1e-6f);
		double      dPrev = -1, dCur = -1;
		if (s.have) {
			dPrev = MaxDiff(prev, s.lastVpu) / scale;
			dCur = MaxDiff(vpu, s.lastVpu) / scale;
			++s.n;
			if (dPrev > 1e-4) ++s.broken;
			s.maxRelPrev = (std::max)(s.maxRelPrev, dPrev);
			s.sumRelCur += dCur;
			if (s.entry != e) ++s.moved;
		}
		if (s.logged < 8) {
			++s.logged;
			std::uint32_t size = 0;
			bool          found = false;
			void*         fp = nullptr;
			const bool    ok = CacheInfo(a_cam, a_jitter, size, found, fp);
			logger::info("diag: build {} jitter={} entry {:p} (cache size {}{} found-in-cache {} -> {:p}) prev-vs-last-frame {:.2e} current-vs-last-frame {:.2e}",
				a_virtual ? "virtual" : "main", a_jitter, a_entry, size, ok ? "" : "?", found, fp, dPrev, dCur);
		}
		std::memcpy(s.lastVpu, vpu, sizeof(s.lastVpu));
		s.have = true;
		s.entry = e;
	}

	void LogChain()
	{
		static const char* names[4] = { "main", "main+jitter", "virtual", "virtual+jitter" };
		for (int i = 0; i < 4; ++i) {
			const Slot& s = g_slot[i];
			if (s.n > 0) {
				logger::info("diag: chain {}: {} builds, previous matrix differs from last frame's current in {} (max relative {:.2e}); mean camera change per build {:.2e}; entry moved {} time(s)",
					names[i], s.n, s.broken, s.maxRelPrev, s.sumRelCur / s.n, s.moved);
			}
		}
	}

	void RequestMv(const std::string& a_label, int a_frames)
	{
		g_mvLabel = a_label;
		g_mvTaken = 0;
		g_mvLeft = (std::max)(1, a_frames);
	}

	void RequestParity(const std::string& a_label, int a_frames)
	{
		g_parityLabel = a_label;
		g_parityLeft = (std::max)(1, a_frames);
	}

	namespace
	{
		std::atomic<int> g_dumpLeft{ 0 };
		std::string      g_dumpLabel;
		std::atomic<int> g_postLeft{ 0 };
		std::string      g_postLabel;
		std::atomic<int> g_postVlLeft{ 0 };
		std::atomic<int> g_preVlLeft{ 0 };
		std::string      g_postVlLabel;

		// the whole texture, tightly packed, behind a 16-byte header
		bool DumpRaw(ID3D11Device* a_dev, ID3D11DeviceContext* a_ctx, ID3D11Texture2D* a_tex, const std::filesystem::path& a_path)
		{
			D3D11_TEXTURE2D_DESC d{};
			a_tex->GetDesc(&d);
			if (d.Format != DXGI_FORMAT_R16G16B16A16_FLOAT || d.SampleDesc.Count != 1) {
				return false;
			}
			D3D11_TEXTURE2D_DESC sd = d;
			sd.Usage = D3D11_USAGE_STAGING;
			sd.BindFlags = 0;
			sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
			sd.MiscFlags = 0;
			sd.MipLevels = 1;
			sd.ArraySize = 1;
			ComPtr<ID3D11Texture2D> st;
			if (FAILED(a_dev->CreateTexture2D(&sd, nullptr, &st))) {
				return false;
			}
			a_ctx->CopySubresourceRegion(st.Get(), 0, 0, 0, 0, a_tex, 0, nullptr);
			D3D11_MAPPED_SUBRESOURCE m{};
			if (FAILED(a_ctx->Map(st.Get(), 0, D3D11_MAP_READ, 0, &m))) {
				return false;
			}
			std::vector<std::uint8_t> buf(16 + static_cast<std::size_t>(d.Width) * d.Height * 8);
			const std::uint32_t       hdr[4] = { 0x48363146u, d.Width, d.Height, static_cast<std::uint32_t>(d.Format) };
			std::memcpy(buf.data(), hdr, 16);
			for (std::uint32_t y = 0; y < d.Height; ++y) {
				std::memcpy(buf.data() + 16 + static_cast<std::size_t>(y) * d.Width * 8, static_cast<const std::uint8_t*>(m.pData) + static_cast<std::size_t>(y) * m.RowPitch, static_cast<std::size_t>(d.Width) * 8);
			}
			a_ctx->Unmap(st.Get(), 0);
			std::ofstream f(a_path, std::ios::binary);
			f.write(reinterpret_cast<const char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
			return f.good();
		}
	}

	bool DumpMainNow(const std::string& a_label)
	{
		auto* rdd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
		auto* mainT = rdd ? reinterpret_cast<ID3D11Texture2D*>(rdd->renderTargets[RE::RENDER_TARGETS::kMAIN].texture) : nullptr;
		auto* devd = reinterpret_cast<ID3D11Device*>(RE::BSGraphics::Renderer::GetDevice());
		std::filesystem::path dir;
		if (auto ld = logger::log_directory()) {
			dir = *ld;
		}
		if (!mainT || !devd || dir.empty()) {
			return false;
		}
		ComPtr<ID3D11DeviceContext> cx;
		devd->GetImmediateContext(&cx);
		const bool ok = DumpRaw(devd, cx.Get(), mainT, dir / ("Riftweaver_" + a_label + "_main.f16"));
		logger::info("diag: main target dump '{}': {}", a_label, ok ? "written" : "FAILED");
		return ok;
	}

	void RequestHdrDump(const std::string& a_label)
	{
		g_dumpLabel = a_label;
		g_dumpLeft = 1;
	}

	namespace
	{
		// mean R, G, B of the HDR target on every 4th pixel (each value clamped to 16 so a few sun-bright pixels do not dominate)
		bool MeanRgb(ID3D11Device* a_dev, ID3D11DeviceContext* a_ctx, ID3D11Texture2D* a_tex, double a_out[3], std::uint32_t a_row0, std::uint32_t a_row1)
		{
			D3D11_TEXTURE2D_DESC d{};
			a_tex->GetDesc(&d);
			if (d.Format != DXGI_FORMAT_R16G16B16A16_FLOAT || d.SampleDesc.Count != 1) {
				return false;
			}
			D3D11_TEXTURE2D_DESC sd = d;
			sd.Usage = D3D11_USAGE_STAGING;
			sd.BindFlags = 0;
			sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
			sd.MiscFlags = 0;
			sd.MipLevels = 1;
			sd.ArraySize = 1;
			ComPtr<ID3D11Texture2D> st;
			if (FAILED(a_dev->CreateTexture2D(&sd, nullptr, &st))) {
				return false;
			}
			a_ctx->CopySubresourceRegion(st.Get(), 0, 0, 0, 0, a_tex, 0, nullptr);
			D3D11_MAPPED_SUBRESOURCE m{};
			if (FAILED(a_ctx->Map(st.Get(), 0, D3D11_MAP_READ, 0, &m))) {
				return false;
			}
			double sum[3] = { 0, 0, 0 };
			std::size_t n = 0;
			const std::uint32_t r1 = (std::min)(a_row1, d.Height);
			for (std::uint32_t y = a_row0; y < r1; y += 4) {
				const auto* row = reinterpret_cast<const std::uint16_t*>(static_cast<const std::uint8_t*>(m.pData) + static_cast<std::size_t>(y) * m.RowPitch);
				for (std::uint32_t x = 0; x < d.Width; x += 4) {
					for (int c = 0; c < 3; ++c) {
						sum[c] += (std::min)(16.0f, (std::max)(0.0f, DirectX::PackedVector::XMConvertHalfToFloat(row[x * 4 + c])));
					}
					++n;
				}
			}
			a_ctx->Unmap(st.Get(), 0);
			for (int c = 0; c < 3; ++c) {
				a_out[c] = n ? sum[c] / n : 0;
			}
			return n > 0;
		}
	}

	namespace
	{
		std::atomic<int> g_vmeanLeft{ 0 };
		std::string      g_vmeanLabel;
	}

	void LogViewMeans(const std::string& a_label)
	{
		g_vmeanLabel = a_label;
		g_vmeanLeft = 2;
	}

	void SamplePost()
	{
		if (g_postLeft.exchange(0) <= 0) {
			return;
		}
		auto* rdd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
		auto* mainT = rdd ? reinterpret_cast<ID3D11Texture2D*>(rdd->renderTargets[RE::RENDER_TARGETS::kMAIN].texture) : nullptr;
		auto* devd = reinterpret_cast<ID3D11Device*>(RE::BSGraphics::Renderer::GetDevice());
		std::filesystem::path dir;
		if (auto ld = logger::log_directory()) {
			dir = *ld;
		}
		if (!mainT || !devd || dir.empty()) {
			return;
		}
		ComPtr<ID3D11DeviceContext> cx;
		devd->GetImmediateContext(&cx);
		const bool ok = DumpRaw(devd, cx.Get(), mainT, dir / ("Riftweaver_" + g_postLabel + "_post.f16"));
		logger::info("diag: hdr dump '{}': composited frame (after the overlay) {}", g_postLabel, ok ? "written" : "FAILED");
	}

	// G305: called right after the engine's own (real frame) VL composite returns: the HDR frame with the real VL added, as <label>_postvl.f16 (only after an hdrdump request of the same frame)
	void SamplePreVl()
	{
		if (g_preVlLeft.exchange(0) <= 0) {
			return;
		}
		auto* rdd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
		auto* mainT = rdd ? reinterpret_cast<ID3D11Texture2D*>(rdd->renderTargets[RE::RENDER_TARGETS::kMAIN].texture) : nullptr;
		auto* devd = reinterpret_cast<ID3D11Device*>(RE::BSGraphics::Renderer::GetDevice());
		std::filesystem::path dir;
		if (auto ld = logger::log_directory()) {
			dir = *ld;
		}
		if (!mainT || !devd || dir.empty()) {
			return;
		}
		ComPtr<ID3D11DeviceContext> cx;
		devd->GetImmediateContext(&cx);
		const bool ok = DumpRaw(devd, cx.Get(), mainT, dir / ("Riftweaver_" + g_postVlLabel + "_prevl.f16"));
		logger::info("diag: hdr dump '{}': frame just before the engine's real VL composite {}", g_postVlLabel, ok ? "written" : "FAILED");
	}

	void SamplePostVl()
	{
		if (g_postVlLeft.exchange(0) <= 0) {
			return;
		}
		auto* rdd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
		auto* mainT = rdd ? reinterpret_cast<ID3D11Texture2D*>(rdd->renderTargets[RE::RENDER_TARGETS::kMAIN].texture) : nullptr;
		auto* devd = reinterpret_cast<ID3D11Device*>(RE::BSGraphics::Renderer::GetDevice());
		std::filesystem::path dir;
		if (auto ld = logger::log_directory()) {
			dir = *ld;
		}
		if (!mainT || !devd || dir.empty()) {
			return;
		}
		ComPtr<ID3D11DeviceContext> cx;
		devd->GetImmediateContext(&cx);
		const bool ok = DumpRaw(devd, cx.Get(), mainT, dir / ("Riftweaver_" + g_postVlLabel + "_postvl.f16"));
		logger::info("diag: hdr dump '{}': frame after the engine's real VL composite {}", g_postVlLabel, ok ? "written" : "FAILED");
	}

	void SampleParity()
	{
		if (g_dumpLeft.load() > 0) {
			auto* rdd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			auto* mainT = rdd ? reinterpret_cast<ID3D11Texture2D*>(rdd->renderTargets[RE::RENDER_TARGETS::kMAIN].texture) : nullptr;
			auto* devd = reinterpret_cast<ID3D11Device*>(RE::BSGraphics::Renderer::GetDevice());
			std::filesystem::path dir;
			if (auto ld = logger::log_directory()) {
				dir = *ld;
			}
			if (mainT && devd && !dir.empty()) {
				g_dumpLeft = 0;
				g_postLabel = g_dumpLabel;
				g_postLeft = 1;
				g_postVlLabel = g_dumpLabel;
				g_postVlLeft = 1;
				g_preVlLeft = 1;
				ComPtr<ID3D11DeviceContext> cx;
				devd->GetImmediateContext(&cx);
				const bool okMain = DumpRaw(devd, cx.Get(), mainT, dir / ("Riftweaver_" + g_dumpLabel + "_main.f16"));
				std::string views;
				for (int i = 0; i < 1; ++i) {  // portal 0 only: the dumps are large
					if (view::ViewSRV(i) && DumpRaw(devd, cx.Get(), static_cast<ID3D11Texture2D*>(view::ViewTexture(i)), dir / ("Riftweaver_" + g_dumpLabel + "_v" + std::to_string(i) + ".f16"))) {
						views += " v" + std::to_string(i);
					}
				}
				logger::info("diag: hdr dump '{}': main frame {}, virtual views:{}", g_dumpLabel, okMain ? "written" : "FAILED", views.empty() ? " none this frame" : views);
				if (auto* mc = RE::Main::WorldRootCamera()) {
					const auto& t = mc->world.translate;
					const auto& r = mc->world.rotate;
					logger::info("diag: hdr dump '{}': real camera at ({:.1f}, {:.1f}, {:.1f}) forward ({:.3f}, {:.3f}, {:.3f})", g_dumpLabel, t.x, t.y, t.z, r.entry[0][0], r.entry[1][0], r.entry[2][0]);
				}
			}
		}
		if (g_vmeanLeft.load() > 0) {
			--g_vmeanLeft;
			auto* dev = reinterpret_cast<ID3D11Device*>(RE::BSGraphics::Renderer::GetDevice());
			if (dev) {
				ComPtr<ID3D11DeviceContext> ctx;
				dev->GetImmediateContext(&ctx);
				for (int i = 0; i < 2; ++i) {
					double rgb[3]{};
					if (view::ViewSRV(i) && MeanRgb(dev, ctx.Get(), static_cast<ID3D11Texture2D*>(view::ViewTexture(i)), rgb, 0, 100000)) {
						logger::info("diag: view mean '{}' portal {}: rgb ({:.3f}, {:.3f}, {:.3f})", g_vmeanLabel, i, rgb[0], rgb[1], rgb[2]);
					}
				}
			}
		}
		if (g_parityLeft.load() <= 0) {
			return;
		}
		--g_parityLeft;
		auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
		auto* mainTex = rd ? reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[RE::RENDER_TARGETS::kMAIN].texture) : nullptr;
		auto* dev = reinterpret_cast<ID3D11Device*>(RE::BSGraphics::Renderer::GetDevice());
		if (!mainTex || !dev) {
			return;
		}
		ComPtr<ID3D11DeviceContext> ctx;
		dev->GetImmediateContext(&ctx);
		double mainRgb[3]{};
		if (!MeanRgb(dev, ctx.Get(), mainTex, mainRgb, 0, 100000)) {
			logger::warn("diag: parity: main target not readable");
			return;
		}
		for (int i = 0; i < 2; ++i) {
			if (!view::ViewSRV(i)) {
				continue;  // no virtual view this frame
			}
			double vRgb[3]{};
			if (MeanRgb(dev, ctx.Get(), static_cast<ID3D11Texture2D*>(view::ViewTexture(i)), vRgb, 0, 100000)) {
				logger::info("diag: parity '{}' portal {}: main frame mean rgb ({:.3f}, {:.3f}, {:.3f}); virtual view ({:.3f}, {:.3f}, {:.3f}); ratio virtual/main ({:.2f}, {:.2f}, {:.2f})", g_parityLabel, i,
					mainRgb[0], mainRgb[1], mainRgb[2], vRgb[0], vRgb[1], vRgb[2], vRgb[0] / (std::max)(mainRgb[0], 1e-6), vRgb[1] / (std::max)(mainRgb[1], 1e-6), vRgb[2] / (std::max)(mainRgb[2], 1e-6));
			}
		}
	}

	void SampleMv()
	{
		if (g_mvLeft.load() <= 0) {
			return;
		}
		--g_mvLeft;
		auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
		auto* tex = rd ? reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[RE::RENDER_TARGETS::kMOTION_VECTOR].texture) : nullptr;
		if (!tex) {
			logger::warn("diag: no motion-vector target");
			return;
		}
		auto* dev = reinterpret_cast<ID3D11Device*>(RE::BSGraphics::Renderer::GetDevice());
		ComPtr<ID3D11DeviceContext> ctx;
		dev->GetImmediateContext(&ctx);
		D3D11_TEXTURE2D_DESC d{};
		tex->GetDesc(&d);
		D3D11_TEXTURE2D_DESC sd = d;
		sd.Usage = D3D11_USAGE_STAGING;
		sd.BindFlags = 0;
		sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
		sd.MiscFlags = 0;
		sd.MipLevels = 1;
		sd.ArraySize = 1;
		if (d.SampleDesc.Count != 1) {
			logger::warn("diag: motion-vector target is multisampled");
			return;
		}
		ComPtr<ID3D11Texture2D> st;
		if (FAILED(dev->CreateTexture2D(&sd, nullptr, &st))) {
			return;
		}
		ctx->CopySubresourceRegion(st.Get(), 0, 0, 0, 0, tex, 0, nullptr);
		D3D11_MAPPED_SUBRESOURCE m{};
		if (FAILED(ctx->Map(st.Get(), 0, D3D11_MAP_READ, 0, &m))) {
			return;
		}
		const UINT W = d.Width, H = d.Height;
		std::vector<float> vx(static_cast<std::size_t>(W) * H), vy(vx.size());
		bool ok = true;
		for (UINT y = 0; y < H && ok; ++y) {
			const auto* row = static_cast<const std::uint8_t*>(m.pData) + static_cast<std::size_t>(y) * m.RowPitch;
			for (UINT x = 0; x < W; ++x) {
				float a = 0, b = 0;
				switch (d.Format) {
				case DXGI_FORMAT_R16G16_FLOAT: {
					const auto* h = reinterpret_cast<const std::uint16_t*>(row) + x * 2;
					a = DirectX::PackedVector::XMConvertHalfToFloat(h[0]);
					b = DirectX::PackedVector::XMConvertHalfToFloat(h[1]);
					break;
				}
				case DXGI_FORMAT_R16G16B16A16_FLOAT: {
					const auto* h = reinterpret_cast<const std::uint16_t*>(row) + x * 4;
					a = DirectX::PackedVector::XMConvertHalfToFloat(h[0]);
					b = DirectX::PackedVector::XMConvertHalfToFloat(h[1]);
					break;
				}
				case DXGI_FORMAT_R32G32_FLOAT: {
					const auto* f = reinterpret_cast<const float*>(row) + x * 2;
					a = f[0];
					b = f[1];
					break;
				}
				case DXGI_FORMAT_R16G16_SNORM: {
					const auto* s16 = reinterpret_cast<const std::int16_t*>(row) + x * 2;
					a = (std::max)(-1.0f, s16[0] / 32767.0f);
					b = (std::max)(-1.0f, s16[1] / 32767.0f);
					break;
				}
				default:
					ok = false;
					break;
				}
				if (!ok) break;
				vx[static_cast<std::size_t>(y) * W + x] = a;
				vy[static_cast<std::size_t>(y) * W + x] = b;
			}
		}
		ctx->Unmap(st.Get(), 0);
		if (!ok) {
			logger::warn("diag: motion-vector format {} not supported by the readback", static_cast<int>(d.Format));
			g_mvLeft = 0;
			return;
		}
		// statistics on every 3rd pixel in both directions
		std::vector<float> mag;
		mag.reserve(static_cast<std::size_t>(W / 3 + 1) * (H / 3 + 1));
		double sum = 0, sumX = 0, sumY = 0;
		std::size_t nonzero = 0;
		for (UINT y = 0; y < H; y += 3) {
			for (UINT x = 0; x < W; x += 3) {
				const auto i = static_cast<std::size_t>(y) * W + x;
				const float mm = std::sqrt(vx[i] * vx[i] + vy[i] * vy[i]);
				mag.push_back(mm);
				sum += mm;
				sumX += vx[i];
				sumY += vy[i];
				if (mm > 1e-6f) ++nonzero;
			}
		}
		std::vector<float> sorted = mag;
		std::sort(sorted.begin(), sorted.end());
		auto pct = [&](double p) { return sorted.empty() ? 0.0f : sorted[(std::min)(sorted.size() - 1, static_cast<std::size_t>(p * sorted.size()))]; };
		logger::info("diag: mv '{}' #{} ({}x{} format {}): mean |v| {:.3e} mean v ({:.3e}, {:.3e}) p50 {:.3e} p90 {:.3e} p99 {:.3e} max {:.3e}; nonzero {:.1f}%",
			g_mvLabel, g_mvTaken, W, H, static_cast<int>(d.Format), sum / (std::max<std::size_t>)(mag.size(), 1), sumX / (std::max<std::size_t>)(mag.size(), 1), sumY / (std::max<std::size_t>)(mag.size(), 1),
			pct(0.5), pct(0.9), pct(0.99), sorted.empty() ? 0.0f : sorted.back(), 100.0 * nonzero / (std::max<std::size_t>)(mag.size(), 1));
		if (g_mvTaken == 0) {
			// picture: red = |x|, green = |y|, scaled by the 99.5th percentile
			const float scale = (std::max)(pct(0.995), 1e-7f) * 1.2f;
			std::vector<std::uint8_t> bgra(static_cast<std::size_t>(W) * H * 4, 255);
			for (std::size_t i = 0; i < static_cast<std::size_t>(W) * H; ++i) {
				auto to8 = [&](float v) { return static_cast<std::uint8_t>(std::clamp(std::fabs(v) / scale, 0.0f, 1.0f) * 255.0f + 0.5f); };
				bgra[i * 4 + 0] = 0;
				bgra[i * 4 + 1] = to8(vy[i]);
				bgra[i * 4 + 2] = to8(vx[i]);
			}
			shot::SaveBgra("mv_" + g_mvLabel, W, H, bgra);
		}
		++g_mvTaken;
	}
}
