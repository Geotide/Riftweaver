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

#include <d3d11.h>
#include <d3dcompiler.h>
#include <dxgi.h>
#include <wrl/client.h>

// A big on-screen label for interactive tests (G283: the person at the keyboard has to see which switch state is running). `label <text>` in the harness sets it, `label off` clears it.
// The text is rendered with GDI into a bitmap, uploaded as a texture and drawn into the swap chain's back buffer right before Present, with the device state saved and put back.
namespace label
{
	namespace
	{
		using Microsoft::WRL::ComPtr;

		std::mutex        g_mutex;
		std::string       g_text;
		std::atomic<bool> g_dirty{ false };

		ComPtr<ID3D11VertexShader>        g_vs;
		ComPtr<ID3D11PixelShader>         g_ps;
		ComPtr<ID3D11SamplerState>        g_smp;
		ComPtr<ID3D11BlendState>          g_bs;
		ComPtr<ID3D11RasterizerState>     g_rs;
		ComPtr<ID3D11DepthStencilState>   g_ds;
		ComPtr<ID3D11ShaderResourceView>  g_srv;
		UINT                              g_texW = 0, g_texH = 0, g_vpW = 0, g_vpH = 0;
		bool                              g_failed = false;

		bool CompileShaders(ID3D11Device* a_dev, UINT a_vpW, UINT a_vpH)
		{
			const std::string vsSrc = std::format(
				"struct O {{ float4 p : SV_Position; float2 uv : TEXCOORD0; }};\n"
				"O main(uint id : SV_VertexID) {{ float2 c = float2(id & 1, id >> 1); float2 px = float2(24.0, 24.0) + c * float2({}.0, {}.0); O o; o.p = float4(px.x / {}.0 * 2.0 - 1.0, 1.0 - px.y / {}.0 * 2.0, 0, 1); o.uv = c; return o; }}\n",
				g_texW, g_texH, a_vpW, a_vpH);
			const char* psSrc =
				"Texture2D t0 : register(t0); SamplerState s0 : register(s0);\n"
				"float4 main(float4 p : SV_Position, float2 uv : TEXCOORD0) : SV_Target { return t0.Sample(s0, uv); }\n";
			ComPtr<ID3DBlob> vb, pb, err;
			if (FAILED(D3DCompile(vsSrc.data(), vsSrc.size(), "label_vs", nullptr, nullptr, "main", "vs_5_0", 0, 0, &vb, &err)) || FAILED(a_dev->CreateVertexShader(vb->GetBufferPointer(), vb->GetBufferSize(), nullptr, &g_vs))) {
				logger::error("label: vertex shader failed: {}", err ? static_cast<const char*>(err->GetBufferPointer()) : "?");
				return false;
			}
			if (!g_ps) {
				if (FAILED(D3DCompile(psSrc, std::strlen(psSrc), "label_ps", nullptr, nullptr, "main", "ps_5_0", 0, 0, &pb, &err)) || FAILED(a_dev->CreatePixelShader(pb->GetBufferPointer(), pb->GetBufferSize(), nullptr, &g_ps))) {
					logger::error("label: pixel shader failed");
					return false;
				}
			}
			return true;
		}

		bool MakeStates(ID3D11Device* a_dev)
		{
			if (g_smp) {
				return true;
			}
			D3D11_SAMPLER_DESC sd{};
			sd.Filter = D3D11_FILTER_MIN_MAG_MIP_POINT;
			sd.AddressU = sd.AddressV = sd.AddressW = D3D11_TEXTURE_ADDRESS_CLAMP;
			sd.MaxLOD = D3D11_FLOAT32_MAX;
			D3D11_BLEND_DESC bd{};
			bd.RenderTarget[0].BlendEnable = TRUE;
			bd.RenderTarget[0].SrcBlend = D3D11_BLEND_ONE;  // premultiplied
			bd.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
			bd.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
			bd.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ONE;
			bd.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_INV_SRC_ALPHA;
			bd.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
			bd.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
			D3D11_RASTERIZER_DESC rd{};
			rd.FillMode = D3D11_FILL_SOLID;
			rd.CullMode = D3D11_CULL_NONE;
			rd.DepthClipEnable = TRUE;
			D3D11_DEPTH_STENCIL_DESC dd{};
			dd.DepthEnable = FALSE;
			return SUCCEEDED(a_dev->CreateSamplerState(&sd, &g_smp)) && SUCCEEDED(a_dev->CreateBlendState(&bd, &g_bs)) && SUCCEEDED(a_dev->CreateRasterizerState(&rd, &g_rs)) && SUCCEEDED(a_dev->CreateDepthStencilState(&dd, &g_ds));
		}

		// the text into a premultiplied RGBA texture: black translucent plate, bright yellow letters
		bool MakeTexture(ID3D11Device* a_dev, const std::string& a_text, UINT a_vpW, UINT a_vpH)
		{
			std::wstring wt(a_text.begin(), a_text.end());
			HDC          dc = CreateCompatibleDC(nullptr);
			const int    fontH = (std::max)(48, static_cast<int>(a_vpH / 12));
			HFONT        font = CreateFontW(-fontH, 0, 0, 0, FW_BOLD, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH | FF_SWISS, L"Arial");
			HGDIOBJ      old = SelectObject(dc, font);
			SIZE         ext{};
			GetTextExtentPoint32W(dc, wt.c_str(), static_cast<int>(wt.size()), &ext);
			const int w = (std::min)(static_cast<int>(a_vpW) - 48, static_cast<int>(ext.cx) + 40);
			const int h = static_cast<int>(ext.cy) + 24;
			BITMAPINFO bi{};
			bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
			bi.bmiHeader.biWidth = w;
			bi.bmiHeader.biHeight = -h;
			bi.bmiHeader.biPlanes = 1;
			bi.bmiHeader.biBitCount = 32;
			bi.bmiHeader.biCompression = BI_RGB;
			void*   bits = nullptr;
			HBITMAP bmp = CreateDIBSection(dc, &bi, DIB_RGB_COLORS, &bits, nullptr, 0);
			bool    ok = false;
			if (bmp && bits) {
				HGDIOBJ oldBmp = SelectObject(dc, bmp);
				std::memset(bits, 0, static_cast<std::size_t>(w) * h * 4);
				SetBkMode(dc, TRANSPARENT);
				SetTextColor(dc, RGB(255, 255, 255));
				TextOutW(dc, 20, 12, wt.c_str(), static_cast<int>(wt.size()));
				GdiFlush();
				std::vector<std::uint32_t> px(static_cast<std::size_t>(w) * h);
				const auto*                src = static_cast<const std::uint8_t*>(bits);
				for (std::size_t i = 0; i < px.size(); ++i) {
					const float ta = (std::max)({ src[i * 4 + 0], src[i * 4 + 1], src[i * 4 + 2] }) / 255.0f;
					const float a = ta + 0.62f * (1.0f - ta);
					const auto  r = static_cast<std::uint32_t>(255.0f * ta + 0.5f);
					const auto  g = static_cast<std::uint32_t>(224.0f * ta + 0.5f);
					const auto  b = static_cast<std::uint32_t>(40.0f * ta + 0.5f);
					px[i] = (static_cast<std::uint32_t>(a * 255.0f + 0.5f) << 24) | (b << 16) | (g << 8) | r;  // R8G8B8A8, premultiplied
				}
				D3D11_TEXTURE2D_DESC td{};
				td.Width = static_cast<UINT>(w);
				td.Height = static_cast<UINT>(h);
				td.MipLevels = 1;
				td.ArraySize = 1;
				td.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
				td.SampleDesc.Count = 1;
				td.Usage = D3D11_USAGE_DEFAULT;
				td.BindFlags = D3D11_BIND_SHADER_RESOURCE;
				D3D11_SUBRESOURCE_DATA sr{ px.data(), static_cast<UINT>(w * 4), 0 };
				ComPtr<ID3D11Texture2D> tex;
				g_srv.Reset();
				if (SUCCEEDED(a_dev->CreateTexture2D(&td, &sr, &tex)) && SUCCEEDED(a_dev->CreateShaderResourceView(tex.Get(), nullptr, &g_srv))) {
					g_texW = static_cast<UINT>(w);
					g_texH = static_cast<UINT>(h);
					ok = true;
				}
				SelectObject(dc, oldBmp);
				DeleteObject(bmp);
			}
			SelectObject(dc, old);
			DeleteObject(font);
			DeleteDC(dc);
			return ok;
		}
	}

	void Set(const std::string& a_text)
	{
		{
			std::lock_guard lk(g_mutex);
			g_text = a_text;
		}
		g_dirty = true;
		logger::info("label: {}", a_text.empty() ? "(cleared)" : a_text);
	}

	void Draw(void* a_swapChain)
	{
		if (g_failed || !a_swapChain) {
			return;
		}
		std::string text;
		{
			std::lock_guard lk(g_mutex);
			text = g_text;
		}
		if (text.empty()) {
			return;
		}
		auto*                       sc = static_cast<IDXGISwapChain*>(a_swapChain);
		ComPtr<ID3D11Device>        dev;
		ComPtr<ID3D11DeviceContext> ctx;
		ComPtr<ID3D11Texture2D>     bb;
		if (FAILED(sc->GetBuffer(0, IID_PPV_ARGS(&bb))) || !bb) {
			return;
		}
		bb->GetDevice(&dev);
		if (!dev) {
			return;
		}
		dev->GetImmediateContext(&ctx);
		D3D11_TEXTURE2D_DESC bd{};
		bb->GetDesc(&bd);
		if (g_dirty.exchange(false) || !g_srv || bd.Width != g_vpW || bd.Height != g_vpH) {
			g_vpW = bd.Width;
			g_vpH = bd.Height;
			if (!MakeStates(dev.Get()) || !MakeTexture(dev.Get(), text, g_vpW, g_vpH) || !CompileShaders(dev.Get(), g_vpW, g_vpH)) {
				logger::error("label: could not build the label, switched off");
				g_failed = true;
				return;
			}
		}
		ComPtr<ID3D11RenderTargetView> rtv;
		if (FAILED(dev->CreateRenderTargetView(bb.Get(), nullptr, &rtv))) {
			return;
		}
		// save what we touch
		ID3D11RenderTargetView*   rtv0[1]{};
		ID3D11DepthStencilView*   dsv0 = nullptr;
		ctx->OMGetRenderTargets(1, rtv0, &dsv0);
		UINT             nvp = 1;
		D3D11_VIEWPORT   vp0{};
		ctx->RSGetViewports(&nvp, &vp0);
		ComPtr<ID3D11BlendState>        bs0;
		FLOAT                           bf0[4]{};
		UINT                            sm0 = 0;
		ctx->OMGetBlendState(&bs0, bf0, &sm0);
		ComPtr<ID3D11DepthStencilState> ds0;
		UINT                            ref0 = 0;
		ctx->OMGetDepthStencilState(&ds0, &ref0);
		ComPtr<ID3D11RasterizerState> rs0;
		ctx->RSGetState(&rs0);
		ComPtr<ID3D11VertexShader> vs0;
		ctx->VSGetShader(&vs0, nullptr, nullptr);
		ComPtr<ID3D11PixelShader> ps0;
		ctx->PSGetShader(&ps0, nullptr, nullptr);
		ComPtr<ID3D11InputLayout> il0;
		ctx->IAGetInputLayout(&il0);
		D3D11_PRIMITIVE_TOPOLOGY topo0{};
		ctx->IAGetPrimitiveTopology(&topo0);
		ID3D11ShaderResourceView* srv0[1]{};
		ctx->PSGetShaderResources(0, 1, srv0);
		ID3D11SamplerState* smp0[1]{};
		ctx->PSGetSamplers(0, 1, smp0);

		ID3D11RenderTargetView* rt = rtv.Get();
		ctx->OMSetRenderTargets(1, &rt, nullptr);
		D3D11_VIEWPORT vp{ 0, 0, static_cast<float>(g_vpW), static_cast<float>(g_vpH), 0.0f, 1.0f };
		ctx->RSSetViewports(1, &vp);
		const FLOAT bf[4]{};
		ctx->OMSetBlendState(g_bs.Get(), bf, 0xFFFFFFFF);
		ctx->OMSetDepthStencilState(g_ds.Get(), 0);
		ctx->RSSetState(g_rs.Get());
		ctx->IASetInputLayout(nullptr);
		ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
		ctx->VSSetShader(g_vs.Get(), nullptr, 0);
		ctx->PSSetShader(g_ps.Get(), nullptr, 0);
		ID3D11ShaderResourceView* s = g_srv.Get();
		ctx->PSSetShaderResources(0, 1, &s);
		ID3D11SamplerState* sm = g_smp.Get();
		ctx->PSSetSamplers(0, 1, &sm);
		ctx->Draw(4, 0);

		// put the state back
		ctx->PSSetSamplers(0, 1, smp0);
		ctx->PSSetShaderResources(0, 1, srv0);
		ctx->PSSetShader(ps0.Get(), nullptr, 0);
		ctx->VSSetShader(vs0.Get(), nullptr, 0);
		ctx->IASetPrimitiveTopology(topo0);
		ctx->IASetInputLayout(il0.Get());
		ctx->RSSetState(rs0.Get());
		ctx->OMSetDepthStencilState(ds0.Get(), ref0);
		ctx->OMSetBlendState(bs0.Get(), bf0, sm0);
		ctx->RSSetViewports(nvp, &vp0);
		ctx->OMSetRenderTargets(1, rtv0, dsv0);
		if (smp0[0]) smp0[0]->Release();
		if (srv0[0]) srv0[0]->Release();
		if (rtv0[0]) rtv0[0]->Release();
		if (dsv0) dsv0->Release();
	}
}
