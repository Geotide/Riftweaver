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

#include <d3d11_4.h>
#include <dxgi.h>
#include <wincodec.h>
#include <wrl/client.h>
#include <DirectXPackedVector.h>

namespace shot
{
	using Microsoft::WRL::ComPtr;

	namespace
	{
		std::filesystem::path   g_outDir;
		std::mutex              g_mutex;
		std::string             g_name;
		std::atomic<bool>       g_pending{ false };

		bool WritePng(const std::filesystem::path& a_path, UINT a_w, UINT a_h, const std::vector<std::uint8_t>& a_bgra)
		{
			CoInitializeEx(nullptr, COINIT_MULTITHREADED);  // RPC_E_CHANGED_MODE is fine: we only need COM to be usable on this thread
			ComPtr<IWICImagingFactory> factory;
			if (FAILED(CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&factory)))) {
				return false;
			}
			ComPtr<IWICStream> stream;
			if (FAILED(factory->CreateStream(&stream)) || FAILED(stream->InitializeFromFilename(a_path.wstring().c_str(), GENERIC_WRITE))) {
				return false;
			}
			ComPtr<IWICBitmapEncoder> enc;
			if (FAILED(factory->CreateEncoder(GUID_ContainerFormatPng, nullptr, &enc)) || FAILED(enc->Initialize(stream.Get(), WICBitmapEncoderNoCache))) {
				return false;
			}
			ComPtr<IWICBitmapFrameEncode> frame;
			ComPtr<IPropertyBag2>         props;
			if (FAILED(enc->CreateNewFrame(&frame, &props)) || FAILED(frame->Initialize(props.Get())) || FAILED(frame->SetSize(a_w, a_h))) {
				return false;
			}
			WICPixelFormatGUID fmt = GUID_WICPixelFormat32bppBGRA;
			frame->SetPixelFormat(&fmt);
			if (FAILED(frame->WritePixels(a_h, a_w * 4, static_cast<UINT>(a_bgra.size()), const_cast<BYTE*>(a_bgra.data())))) {
				return false;
			}
			return SUCCEEDED(frame->Commit()) && SUCCEEDED(enc->Commit());
		}

		float Tonemap(float v)
		{
			v = (std::max)(0.0f, v);
			v = v / (1.0f + v);
			return std::pow(v, 1.0f / 2.2f);
		}

		std::uint8_t To8(float v) { return static_cast<std::uint8_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f); }

		// Backbuffer formats seen in practice are 8-bit; HDR formats are tonemapped crudely so a screenshot is always possible.
		bool ConvertToBgra(DXGI_FORMAT a_fmt, const std::uint8_t* a_src, UINT a_pitch, UINT a_w, UINT a_h, std::vector<std::uint8_t>& a_out, double* a_meanLum)
		{
			a_out.assign(static_cast<std::size_t>(a_w) * a_h * 4, 255);
			double sum = 0;
			for (UINT y = 0; y < a_h; ++y) {
				const std::uint8_t* row = a_src + static_cast<std::size_t>(y) * a_pitch;
				std::uint8_t*       dst = a_out.data() + static_cast<std::size_t>(y) * a_w * 4;
				for (UINT x = 0; x < a_w; ++x) {
					float r = 0, g = 0, b = 0;
					switch (a_fmt) {
					case DXGI_FORMAT_R8G8B8A8_UNORM:
					case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
					case DXGI_FORMAT_R8G8B8A8_TYPELESS:
						r = row[x * 4 + 0] / 255.0f; g = row[x * 4 + 1] / 255.0f; b = row[x * 4 + 2] / 255.0f;
						dst[x * 4 + 0] = row[x * 4 + 2]; dst[x * 4 + 1] = row[x * 4 + 1]; dst[x * 4 + 2] = row[x * 4 + 0];
						break;
					case DXGI_FORMAT_B8G8R8A8_UNORM:
					case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
					case DXGI_FORMAT_B8G8R8A8_TYPELESS:
					case DXGI_FORMAT_B8G8R8X8_UNORM:
						b = row[x * 4 + 0] / 255.0f; g = row[x * 4 + 1] / 255.0f; r = row[x * 4 + 2] / 255.0f;
						dst[x * 4 + 0] = row[x * 4 + 0]; dst[x * 4 + 1] = row[x * 4 + 1]; dst[x * 4 + 2] = row[x * 4 + 2];
						break;
					case DXGI_FORMAT_R16G16B16A16_FLOAT: {
						const auto* h16 = reinterpret_cast<const std::uint16_t*>(row) + x * 4;
						r = DirectX::PackedVector::XMConvertHalfToFloat(h16[0]);
						g = DirectX::PackedVector::XMConvertHalfToFloat(h16[1]);
						b = DirectX::PackedVector::XMConvertHalfToFloat(h16[2]);
						dst[x * 4 + 0] = To8(Tonemap(b)); dst[x * 4 + 1] = To8(Tonemap(g)); dst[x * 4 + 2] = To8(Tonemap(r));
						break;
					}
					case DXGI_FORMAT_R10G10B10A2_UNORM: {
						std::uint32_t v;
						std::memcpy(&v, row + x * 4, 4);
						r = (v & 0x3FF) / 1023.0f; g = ((v >> 10) & 0x3FF) / 1023.0f; b = ((v >> 20) & 0x3FF) / 1023.0f;
						dst[x * 4 + 0] = To8(b); dst[x * 4 + 1] = To8(g); dst[x * 4 + 2] = To8(r);
						break;
					}
					default:
						return false;
					}
					sum += 0.2126 * r + 0.7152 * g + 0.0722 * b;
				}
			}
			if (a_meanLum) {
				*a_meanLum = sum / (static_cast<double>(a_w) * a_h);
			}
			return true;
		}

		bool SaveTexture(ID3D11Device* a_dev, ID3D11DeviceContext* a_ctx, ID3D11Resource* a_res, const std::filesystem::path& a_path, double* a_meanLum, std::string* a_info)
		{
			ComPtr<ID3D11Texture2D> src;
			if (FAILED(a_res->QueryInterface(IID_PPV_ARGS(&src)))) {
				return false;
			}
			D3D11_TEXTURE2D_DESC d{};
			src->GetDesc(&d);
			if (a_info) {
				*a_info = std::format("{}x{} format {} samples {}", d.Width, d.Height, static_cast<int>(d.Format), d.SampleDesc.Count);
			}
			if (d.SampleDesc.Count != 1) {
				return false;
			}
			D3D11_TEXTURE2D_DESC sd = d;
			sd.Usage = D3D11_USAGE_STAGING;
			sd.BindFlags = 0;
			sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
			sd.MiscFlags = 0;
			sd.MipLevels = 1;
			sd.ArraySize = 1;
			ComPtr<ID3D11Texture2D> staging;
			if (FAILED(a_dev->CreateTexture2D(&sd, nullptr, &staging))) {
				return false;
			}
			a_ctx->CopySubresourceRegion(staging.Get(), 0, 0, 0, 0, src.Get(), 0, nullptr);
			D3D11_MAPPED_SUBRESOURCE m{};
			if (FAILED(a_ctx->Map(staging.Get(), 0, D3D11_MAP_READ, 0, &m))) {
				return false;
			}
			std::vector<std::uint8_t> bgra;
			const bool                ok = ConvertToBgra(d.Format, static_cast<const std::uint8_t*>(m.pData), m.RowPitch, d.Width, d.Height, bgra, a_meanLum);
			a_ctx->Unmap(staging.Get(), 0);
			if (!ok) {
				logger::warn("shot: unsupported backbuffer format {}", static_cast<int>(d.Format));
				return false;
			}
			return WritePng(a_path, d.Width, d.Height, bgra);
		}

		ID3D11Device* Device() { return reinterpret_cast<ID3D11Device*>(RE::BSGraphics::Renderer::GetDevice()); }

		// ---- ring of recent frames ----
		struct Ring
		{
			std::atomic<bool>                    armed{ false };
			std::atomic<bool>                    trigger{ false };
			std::string                          tag;
			int                                  pre{ 8 }, post{ 16 }, size{ 24 };
			int                                  next{ 0 }, filled{ 0 }, countdown{ -1 };
			std::vector<ComPtr<ID3D11Texture2D>> tex;
			D3D11_TEXTURE2D_DESC                 desc{};
		};
		Ring g_ring;

		void RingDump(ID3D11DeviceContext* a_ctx)
		{
			const int n = g_ring.filled;
			struct Job
			{
				std::filesystem::path     path;
				UINT                      w, h;
				std::vector<std::uint8_t> bgra;
			};
			auto jobs = std::make_shared<std::vector<Job>>();
			for (int k = 0; k < n; ++k) {
				const int                idx = ((g_ring.next - n + k) % g_ring.size + g_ring.size) % g_ring.size;
				D3D11_MAPPED_SUBRESOURCE m{};
				if (FAILED(a_ctx->Map(g_ring.tex[idx].Get(), 0, D3D11_MAP_READ, 0, &m))) {
					continue;
				}
				std::vector<std::uint8_t> bgra;
				double                    lum = 0;
				const bool                ok = ConvertToBgra(g_ring.desc.Format, static_cast<const std::uint8_t*>(m.pData), m.RowPitch, g_ring.desc.Width, g_ring.desc.Height, bgra, &lum);
				a_ctx->Unmap(g_ring.tex[idx].Get(), 0);
				if (ok) {
					jobs->push_back({ g_outDir / std::format("Riftweaver_{}_{:02d}.png", g_ring.tag, k), g_ring.desc.Width, g_ring.desc.Height, std::move(bgra) });
				}
			}
			logger::info("shot: ring '{}' captured {} frame(s); encoding them on a worker thread", g_ring.tag, n);
			std::thread([jobs]() {
				for (auto& j : *jobs) {
					WritePng(j.path, j.w, j.h, j.bgra);
				}
			}).detach();
		}

		void RingCapture(IDXGISwapChain* a_sc)
		{
			if (!g_ring.armed.load()) {
				return;
			}
			ComPtr<ID3D11Texture2D> bb;
			if (FAILED(a_sc->GetBuffer(0, IID_PPV_ARGS(&bb)))) {
				return;
			}
			D3D11_TEXTURE2D_DESC d{};
			bb->GetDesc(&d);
			ID3D11Device*               dev = Device();
			ComPtr<ID3D11DeviceContext> ctx;
			dev->GetImmediateContext(&ctx);
			g_ring.size = (std::max)(2, g_ring.pre + g_ring.post);
			if (static_cast<int>(g_ring.tex.size()) != g_ring.size || g_ring.desc.Width != d.Width || g_ring.desc.Height != d.Height || g_ring.desc.Format != d.Format) {
				g_ring.tex.clear();
				g_ring.desc = d;
				g_ring.desc.Usage = D3D11_USAGE_STAGING;
				g_ring.desc.BindFlags = 0;
				g_ring.desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
				g_ring.desc.MiscFlags = 0;
				g_ring.desc.MipLevels = 1;
				g_ring.desc.ArraySize = 1;
				for (int i = 0; i < g_ring.size; ++i) {
					ComPtr<ID3D11Texture2D> t;
					if (FAILED(dev->CreateTexture2D(&g_ring.desc, nullptr, &t))) {
						logger::warn("shot: ring: cannot create staging texture {}", i);
						g_ring.armed = false;
						return;
					}
					g_ring.tex.push_back(t);
				}
				g_ring.next = 0;
				g_ring.filled = 0;
				g_ring.countdown = -1;
			}
			ctx->CopyResource(g_ring.tex[g_ring.next].Get(), bb.Get());
			g_ring.next = (g_ring.next + 1) % g_ring.size;
			g_ring.filled = (std::min)(g_ring.filled + 1, g_ring.size);
			if (g_ring.countdown < 0) {
				if (g_ring.trigger.exchange(false)) {
					g_ring.countdown = g_ring.post;
				}
			} else if (g_ring.countdown == 0) {
				RingDump(ctx.Get());
				g_ring.armed = false;
				g_ring.countdown = -1;
			} else {
				--g_ring.countdown;
			}
		}

		using Present_t = HRESULT(STDMETHODCALLTYPE*)(IDXGISwapChain*, UINT, UINT);
		Present_t g_origPresent{ nullptr };

		HRESULT STDMETHODCALLTYPE H_Present(IDXGISwapChain* a_sc, UINT a_si, UINT a_fl)
		{
			view::PostLogEnd();  // G271: the post chain runs after the frame function, up to Present
			label::Draw(a_sc);
			if (g_pending.load()) {
				std::string name;
				{
					std::lock_guard lk(g_mutex);
					name = g_name;
				}
				ComPtr<ID3D11Texture2D> bb;
				if (SUCCEEDED(a_sc->GetBuffer(0, IID_PPV_ARGS(&bb)))) {
					ID3D11Device*               dev = Device();
					ComPtr<ID3D11DeviceContext> ctx;
					dev->GetImmediateContext(&ctx);
					double      lum = 0;
					std::string info;
					const auto  path = g_outDir / ("Riftweaver_" + name + ".png");
					const bool  ok = SaveTexture(dev, ctx.Get(), bb.Get(), path, &lum, &info);
					logger::info("shot: '{}' (backbuffer {}) saved={} meanLuma={:.4f} -> {}", name, info, ok, lum, path.string());
				}
				g_pending = false;
			}
			RingCapture(a_sc);
			record::OnPresent(a_sc);
			return g_origPresent(a_sc, a_si, a_fl);
		}
	}

	bool Init()
	{
		if (auto d = logger::log_directory()) {
			g_outDir = *d;
		}
		auto* window = RE::BSGraphics::Renderer::GetCurrentRenderWindow();
		auto* sc = window ? reinterpret_cast<IDXGISwapChain*>(window->swapChain) : nullptr;
		if (!sc) {
			logger::error("shot: no swap chain");
			return false;
		}
		void** vt = *reinterpret_cast<void***>(sc);
		DWORD  old = 0;
		if (!VirtualProtect(&vt[8], sizeof(void*), PAGE_READWRITE, &old)) {
			return false;
		}
		g_origPresent = reinterpret_cast<Present_t>(vt[8]);
		vt[8] = reinterpret_cast<void*>(&H_Present);
		VirtualProtect(&vt[8], sizeof(void*), old, &old);
		logger::info("shot: swap chain {:p} vtable {:p}: Present hooked (orig {:p})", static_cast<void*>(sc), static_cast<void*>(vt), reinterpret_cast<void*>(g_origPresent));
		return true;
	}

	bool SaveResource(void* a_d3dTexture, const std::string& a_name)
	{
		if (!a_d3dTexture) {
			return false;
		}
		ID3D11Device*               dev = Device();
		ComPtr<ID3D11DeviceContext> ctx;
		dev->GetImmediateContext(&ctx);
		double      lum = 0;
		std::string info;
		const auto  path = g_outDir / ("Riftweaver_" + a_name + ".png");
		const bool  ok = SaveTexture(dev, ctx.Get(), static_cast<ID3D11Texture2D*>(a_d3dTexture), path, &lum, &info);
		logger::info("shot: resource '{}' ({}) saved={} meanLuma={:.4f} -> {}", a_name, info, ok, lum, path.string());
		return ok;
	}

	bool SaveBgra(const std::string& a_name, unsigned a_w, unsigned a_h, const std::vector<std::uint8_t>& a_bgra)
	{
		const auto path = g_outDir / ("Riftweaver_" + a_name + ".png");
		const bool ok = WritePng(path, a_w, a_h, a_bgra);
		logger::info("shot: image '{}' ({}x{}) saved={} -> {}", a_name, a_w, a_h, ok, path.string());
		return ok;
	}

	void Request(const std::string& a_name)
	{
		{
			std::lock_guard lk(g_mutex);
			g_name = a_name;
		}
		g_pending = true;
	}

	void ArmRing(const std::string& a_tag, int a_pre, int a_post)
	{
		g_ring.tag = a_tag;
		g_ring.pre = (std::max)(0, a_pre);
		g_ring.post = (std::max)(0, a_post);
		g_ring.tex.clear();
		g_ring.countdown = -1;
		g_ring.trigger = false;
		g_ring.armed = true;
		logger::info("shot: ring '{}' armed: {} frame(s) before and {} after the trigger", a_tag, g_ring.pre, g_ring.post);
	}

	void TriggerRing()
	{
		if (g_ring.armed.load()) {
			g_ring.trigger = true;
		}
	}

	void DisarmRing()
	{
		g_ring.armed = false;
	}

	bool Busy()
	{
		return g_pending.load();
	}
}
