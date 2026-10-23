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
#include "Hooks.h"
#include "Util.h"

#include <d3d11.h>
#include <d3dcompiler.h>
#include <intrin.h>
#include <wrl/client.h>

// The see-through view (stage S2).
//
// For each placed portal that the main camera can see from its front, the main camera is carried through the pair (entry portal A,
// exit portal B) to a virtual camera, and the world is rendered once more from there with the engine's own pipeline (the same
// recipe the engine uses for its dynamic cubemap: own camera copy, own accumulator, BSCullingProcess::Process, SubmitAccumulator),
// into the engine's main HDR target. That happens at the very start of Main::RenderWorld, before the main view overwrites the
// target; the result is copied to a private texture which the overlay samples inside the portal's oval at screen coordinates.
// Culling is a pyramid through the oval's screen rectangle plus the exit portal's plane as the near plane (engine convention:
// inside when N.P - c >= 0 with c = N.P0). Prototype ids and field offsets: docs/PROTOTYPE-FINDINGS.md.
namespace view
{
	using Microsoft::WRL::ComPtr;
	using RE::NiPoint3;

	namespace
	{
		Params g_params;

		ID3D11Device* Device() { return reinterpret_cast<ID3D11Device*>(RE::BSGraphics::Renderer::GetDevice()); }

		// ---- engine entry points (AE ids, prologues verified in the prototype) ----
		using SetRT_t = void (*)(void*, std::uint32_t, std::int32_t, std::uint32_t, bool);
		using SetDS_t = void (*)(void*, std::int32_t, std::uint32_t, std::uint32_t, bool);
		using UpdateVP_t = void (*)(void*, std::uint32_t, std::uint32_t, bool);
		using ApplyState_t = void (*)(void*, bool);
		using CtxCtor_t = void* (*)(void*);
		using CamUpdate_t = void (*)(void*);
		using Cull_t = void (*)(void*, std::uint64_t, std::uint64_t);
		using Submit_t = void (*)(void*, void*, std::uint64_t);
		using StartAcc_t = void (*)(void*, void*);

		void* MgrAddr() { return reinterpret_cast<void*>(REL::ID(411451).address()); }
		void* RendererAddr() { return reinterpret_cast<void*>(REL::ID(411393).address()); }

		void* SceneStateNode()
		{
			void* p = nullptr;
			ReadSafeT(reinterpret_cast<const void*>(REL::ID(390951).address()), p);
			return p;
		}

		// ---- state ----
		std::uint8_t*             g_cam{ nullptr };  // copy of the world camera
		std::uint8_t*             g_cam2{ nullptr };  // second copy: the real camera, for the body pass
		RE::BSShaderAccumulator*  g_acc{ nullptr };
		RE::BSShaderAccumulator*  g_accPre{ nullptr };  // the depth pre-pass accumulator (view prepass=1)
		bool                      g_disabled{ false };
		int                       g_exceptions{ 0 };
		ComPtr<ID3D11Texture2D>          g_tex[2];
		ComPtr<ID3D11ShaderResourceView> g_srv[2];
		// G306 (view vladd=1, default): the virtual VL is NOT baked into the stored picture; the picture right before its VL (g_vlPre) and right after it (g_vlPost) are kept, g_tex stays without VL, and the overlay adds
		// (post - pre) onto the oval pixels after the engine's own VL composite, as the real frame gets its VL (the engine's glow step before the composite then sees the dim picture, as it does for the real sky)
		ComPtr<ID3D11Texture2D>          g_vlPre[2], g_vlPost[2];
		ComPtr<ID3D11ShaderResourceView> g_vlPreSrv[2], g_vlPostSrv[2];
		bool                              g_vlAddValid[2]{ false, false };
		bool                      g_valid[2]{ false, false };  // this frame's view of portal i is usable
		// What straddles the EXIT portal of the pass for portal i, drawn from the virtual camera of that pass: the part that has gone through the exit's plane, which in the real world
		// is the part emerging in front of portal i. Colour and depth are kept for the overlay to composite over the main view (private copies: the main targets are cleared after).
		// Two pictures per portal, index = portal + 2 * kind: kind 0 is the part that has come out in front of the portal (composited whole), kind 1 is the entity's own front part near the
		// entry plane, redrawn from the real camera (composited only where it lies within the oval's depth-bias band, where the oval's picture has covered it).
		ComPtr<ID3D11Texture2D>          g_p2c[4], g_p2d[4];
		ComPtr<ID3D11ShaderResourceView> g_p2cSrv[4], g_p2dSrv[4];
		bool                      g_p2valid[4]{ false, false, false, false };
		UINT                      g_p2w{ 0 }, g_p2h{ 0 };
		DXGI_FORMAT               g_p2dFmt{ DXGI_FORMAT_UNKNOWN };
		UINT                      g_w{ 0 }, g_h{ 0 };
		DXGI_FORMAT               g_fmt{ DXGI_FORMAT_UNKNOWN };
		double                    g_lastCullMs{ 0 }, g_lastSubmitMs{ 0 }, g_lastTotalMs{ 0 };
		std::uint64_t             g_passes{ 0 };

		// camera used for this frame's views (raw reading = camera of the previous frame; predicted = extrapolated), for the error statistics
		struct Pose
		{
			NiPoint3 pos{}, fwd{}, up{}, right{};
		};
		Pose  g_prevReading, g_usedRaw, g_usedPred;
		bool  g_havePrev{ false }, g_usedThisFrame{ false };
		struct Stats
		{
			int    n{ 0 };
			double rawPos{ 0 }, rawAng{ 0 }, predPos{ 0 }, predAng{ 0 }, rawAngMax{ 0 }, predAngMax{ 0 }, rawPosMax{ 0 }, predPosMax{ 0 };
		};
		Stats g_stats;

		// ---- small vector helpers ----
		float    Dot(const NiPoint3& a, const NiPoint3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
		NiPoint3 Cross(const NiPoint3& a, const NiPoint3& b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
		NiPoint3 Norm(const NiPoint3& a)
		{
			const float l = std::sqrt(Dot(a, a));
			return l > 1e-6f ? a * (1.0f / l) : NiPoint3{ 1, 0, 0 };
		}

		// ---- carrying the camera through the pair ----
		// Entry-local coordinates of a world point, mirrored (a 180-degree turn about the up axis: x -> -x, z -> -z), placed in the exit frame.
		NiPoint3 MapPoint(const portals::Portal& A, const portals::Portal& B, const NiPoint3& p)
		{
			const NiPoint3 d = p - A.c;
			const float    x = Dot(d, A.r), y = Dot(d, A.u), z = Dot(d, A.n);
			return B.c + B.r * (-x) + B.u * y + B.n * (-z);
		}
		NiPoint3 MapDir(const portals::Portal& A, const portals::Portal& B, const NiPoint3& v)
		{
			const float x = Dot(v, A.r), y = Dot(v, A.u), z = Dot(v, A.n);
			return B.r * (-x) + B.u * y + B.n * (-z);
		}

		// screen rectangle (view-space tangents) covered by the portal's bounding rectangle; false when it should be skipped
		bool PortalRect(const portals::Portal& A, const world::CamInfo& cam, float& a_l, float& a_r, float& a_b, float& a_t)
		{
			if (g_params.forceRect || doorway::State(A, cam) != 0) {
				// the camera is at the portal's plane (in front of it, close, or behind it inside the aperture): the portal covers (half of) the screen
				a_l = cam.tanLeft; a_r = cam.tanRight; a_b = cam.tanBottom; a_t = cam.tanTop;
				return true;
			}
			if (Dot(cam.pos - A.c, A.n) <= 0.5f) {
				return false;  // the camera is behind the portal's surface: it is not looking at the front
			}
			float l = 1e9f, r = -1e9f, b = 1e9f, t = -1e9f;
			const float pad = 1.30f;
			// the outline's corners in view space (x right, y up, f forward), clipped against the near plane (Sutherland-Hodgman)
			struct V
			{
				float x, y, f;
			};
			std::vector<V> poly;
			const int      order[4][2] = { { -1, -1 }, { 1, -1 }, { 1, 1 }, { -1, 1 } };
			for (const auto& o : order) {
				const NiPoint3 d = (A.c + A.r * (o[0] * A.hw * pad) + A.u * (o[1] * A.hh * pad)) - cam.pos;
				poly.push_back({ Dot(d, cam.right), Dot(d, cam.up), Dot(d, cam.fwd) });
			}
			const float    fn = cam.nearDist * 1.5f;
			std::vector<V> clipped;
			for (std::size_t i = 0; i < poly.size(); ++i) {
				const V& a = poly[i];
				const V& c = poly[(i + 1) % poly.size()];
				const bool ain = a.f > fn, cin = c.f > fn;
				if (ain) clipped.push_back(a);
				if (ain != cin) {
					const float s = (fn - a.f) / (c.f - a.f);
					clipped.push_back({ a.x + (c.x - a.x) * s, a.y + (c.y - a.y) * s, fn });
				}
			}
			if (clipped.empty()) {
				return false;  // entirely behind the camera
			}
			for (const auto& v : clipped) {
				const float tx = v.x / v.f, ty = v.y / v.f;
				l = (std::min)(l, tx); r = (std::max)(r, tx); b = (std::min)(b, ty); t = (std::max)(t, ty);
			}
			l = (std::max)(l, cam.tanLeft); r = (std::min)(r, cam.tanRight);
			b = (std::max)(b, cam.tanBottom); t = (std::min)(t, cam.tanTop);
			if (l >= r || b >= t) {
				return false;  // off screen
			}
			a_l = l; a_r = r; a_b = b; a_t = t;
			return true;
		}

		bool EnsureTextures(ID3D11Texture2D* a_mainTex)
		{
			D3D11_TEXTURE2D_DESC d{};
			a_mainTex->GetDesc(&d);
			if (g_tex[0] && g_w == d.Width && g_h == d.Height && g_fmt == d.Format) {
				return true;
			}
			g_w = d.Width; g_h = d.Height; g_fmt = d.Format;
			d.BindFlags = D3D11_BIND_SHADER_RESOURCE;
			d.MiscFlags = 0;
			d.MipLevels = 1;
			d.ArraySize = 1;
			d.SampleDesc.Count = 1;
			d.SampleDesc.Quality = 0;
			d.Usage = D3D11_USAGE_DEFAULT;
			d.CPUAccessFlags = 0;
			auto* dev = Device();
			for (int i = 0; i < 2; ++i) {
				g_tex[i].Reset();
				g_srv[i].Reset();
				if (FAILED(dev->CreateTexture2D(&d, nullptr, &g_tex[i])) || FAILED(dev->CreateShaderResourceView(g_tex[i].Get(), nullptr, &g_srv[i]))) {
					logger::error("view: cannot create the private view texture {} ({}x{} fmt {})", i, d.Width, d.Height, static_cast<int>(d.Format));
					return false;
				}
				g_vlPre[i].Reset();
				g_vlPost[i].Reset();
				g_vlPreSrv[i].Reset();
				g_vlPostSrv[i].Reset();
				if (FAILED(dev->CreateTexture2D(&d, nullptr, &g_vlPre[i])) || FAILED(dev->CreateShaderResourceView(g_vlPre[i].Get(), nullptr, &g_vlPreSrv[i])) || FAILED(dev->CreateTexture2D(&d, nullptr, &g_vlPost[i])) || FAILED(dev->CreateShaderResourceView(g_vlPost[i].Get(), nullptr, &g_vlPostSrv[i]))) {
					logger::error("view: cannot create the VL pre/post textures {} ({}x{} fmt {})", i, d.Width, d.Height, static_cast<int>(d.Format));
					return false;
				}
			}
			logger::info("view: private view textures {}x{} format {}", d.Width, d.Height, static_cast<int>(d.Format));
			return true;
		}

		// the engine's state words that it pokes around its own cubemap submit (see the prototype)
		void PokeBeforeBind()
		{
			auto* dirty = reinterpret_cast<std::uint32_t*>(REL::ID(388819).address());
			auto* s88 = reinterpret_cast<std::uint32_t*>(REL::ID(388840).address());
			auto* s8c = reinterpret_cast<std::uint32_t*>(REL::ID(388841).address());
			if (*s88 != 3) {
				*s88 = 3;
				if (*s8c == 3) {
					*dirty &= ~4u;
				} else {
					*dirty |= 4u;
				}
			}
		}
		float             g_biasUnits[2]{ 0.0f, 0.0f };  // G264: the engine's cascade depth biases (cb2[2].y, .z, fractions of the cascade's depth range) in world units, taken from its own mask draw the first time
		bool              g_biasUnitsValid{ false };
		std::atomic<bool> g_sunLogOn{ false };  // view shlog=N (G260): set for the frames that log the sun shadow light (declared early: the mask replay capture logs the engine's constants too)
		bool g_inVirtual{ false };  // true between the two PokeBeforeSubmit calls around one of our own virtual submits (render trace)
		// raw dump of a render target by index: 16-byte header (width, height, DXGI format, bytes per pixel) then tightly packed rows, <log dir>\Riftweaver_<label>_rt<index>.raw
		void DumpTexRaw(ID3D11DeviceContext* a_ctx, ID3D11Texture2D* tex, const std::string& a_name)
		{
			std::filesystem::path dir;
			if (auto ld = logger::log_directory()) dir = *ld;
			if (!tex || dir.empty()) return;
			D3D11_TEXTURE2D_DESC d{};
			tex->GetDesc(&d);
			D3D11_TEXTURE2D_DESC sd = d;
			sd.Usage = D3D11_USAGE_STAGING;
			sd.BindFlags = 0;
			sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
			sd.MiscFlags = 0;
			sd.ArraySize = 1;
			sd.MipLevels = 1;
			ComPtr<ID3D11Texture2D> st;
			if (d.SampleDesc.Count != 1 || FAILED(Device()->CreateTexture2D(&sd, nullptr, &st))) return;
			a_ctx->CopySubresourceRegion(st.Get(), 0, 0, 0, 0, tex, 0, nullptr);
			D3D11_MAPPED_SUBRESOURCE m{};
			if (FAILED(a_ctx->Map(st.Get(), 0, D3D11_MAP_READ, 0, &m))) return;
			std::uint32_t bpp = 4;
			switch (d.Format) {
			case DXGI_FORMAT_R16G16B16A16_FLOAT: case DXGI_FORMAT_R16G16B16A16_UNORM: case DXGI_FORMAT_R32G32_FLOAT: bpp = 8; break;
			case DXGI_FORMAT_R8_UNORM: bpp = 1; break;
			case DXGI_FORMAT_R16_FLOAT: case DXGI_FORMAT_R16_UNORM: case DXGI_FORMAT_R8G8_UNORM: bpp = 2; break;
			default: bpp = 4; break;
			}
			std::vector<std::uint8_t> buf(16 + static_cast<std::size_t>(d.Width) * d.Height * bpp);
			const std::uint32_t hdr[4] = { d.Width, d.Height, static_cast<std::uint32_t>(d.Format), bpp };
			std::memcpy(buf.data(), hdr, 16);
			for (UINT y = 0; y < d.Height; ++y) {
				std::memcpy(buf.data() + 16 + static_cast<std::size_t>(y) * d.Width * bpp, static_cast<const std::uint8_t*>(m.pData) + static_cast<std::size_t>(y) * m.RowPitch, static_cast<std::size_t>(d.Width) * bpp);
			}
			a_ctx->Unmap(st.Get(), 0);
			std::ofstream f(dir / ("Riftweaver_" + a_name + ".raw"), std::ios::binary);
			f.write(reinterpret_cast<const char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
		}
		ID3D11DepthStencilView* g_liveDsv{ nullptr };  // the depth view bound for the current virtual pass
		void DumpRtRaw(ID3D11DeviceContext* a_ctx, int a_index, const std::string& a_label)
		{
			auto* rd2 = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			auto* tex = rd2 ? reinterpret_cast<ID3D11Texture2D*>(rd2->renderTargets[a_index].texture) : nullptr;
			DumpTexRaw(a_ctx, tex, a_label + "_rt" + std::to_string(a_index));
		}
		bool g_rtDumpPending{ false };
		std::vector<int> g_rtListOverride;  // view rtlist=18,1 (G233): dump only these targets (empty = the full list)
		bool             g_rtDepthDump{ true };  // view rtdepth=0: no depth dump with the targets
		std::string      g_engTagPrefix;  // view engtag=xx: prefix of the engine-frame dump labels (xxengA, xxengB, xxengC)
		std::string g_rtDumpTag{ "ssao" };  // file label of the next raw dump (view rttag=...)
		void DumpSsaoInputs(ID3D11DeviceContext* a_ctx, const char* a_stage)
		{
			static const int kList[] = { 1, 2, 3, 4, 5, 6, 7, 18, 46, 47, 49, 50, 62, 63, 64, 65, 66, 67, 85, 112, 0x71 };
			const std::vector<int> all(std::begin(kList), std::end(kList));
			for (int i : (g_rtListOverride.empty() ? all : g_rtListOverride)) {
				if (auto* rd4 = RE::BSGraphics::Renderer::GetRendererDataSingleton(); rd4 && rd4->renderTargets[i].texture && a_stage[1] == 'r') {
					D3D11_TEXTURE2D_DESC dd{};
					reinterpret_cast<ID3D11Texture2D*>(rd4->renderTargets[i].texture)->GetDesc(&dd);
					logger::info("view: render target {}: texture {:p}, {}x{}, format {}, {} mip(s), bind flags {:#x}", i, static_cast<void*>(rd4->renderTargets[i].texture), dd.Width, dd.Height, static_cast<int>(dd.Format), dd.MipLevels, static_cast<unsigned>(dd.BindFlags));
				}
				DumpRtRaw(a_ctx, i, g_rtDumpTag + "_" + a_stage);
			}
			if (g_liveDsv && g_rtDepthDump) {
				ComPtr<ID3D11Resource> dres;
				g_liveDsv->GetResource(&dres);
				ComPtr<ID3D11Texture2D> dtex;
				if (dres && SUCCEEDED(dres.As(&dtex))) {
					DumpTexRaw(a_ctx, dtex.Get(), g_rtDumpTag + "_" + a_stage + "_depth");
				}
			}
			logger::info("view: SSAO inputs dumped ({})", a_stage);
		}
		// Main::RenderWorld's ambient-occlusion step (id 107142, 0x153a034..0x153a1ca): unbind the colour targets, RenderSSAO (compute), the copy/dispatch calls, RenderSSAO (apply), as the engine does between the
		// opaque part of the colour pass and FinishAccumulatingPostResolveDepth. Camera, depth and normals are those of the pass just drawn.
		// The engine's SSAO is temporal: it writes the raw AO into targets 62+(flag+2*parity) (parity = the shader's +0x54, flipped by every compute call) and blends with the pair written the frame before.
		// Run for a virtual view, that history would be the real view's AO (ghost of the real wall in the portal) and the real view would inherit ours. So each view gets its own history pair: before our
		// calls the four raw-AO targets (62..65) are saved, our own history is put in, and afterwards ours is kept and the saved real history and the parity are put back.
		constexpr int kSsaoRts = 9;
		constexpr int kSsaoRtIndex[kSsaoRts] = { 62, 63, 64, 65, 46, 47, 66, 67, 49 };  // raw AO pairs, final AO and its half-res copy, the blur targets
		struct SsaoHistory
		{
			ComPtr<ID3D11Texture2D> mainSave[kSsaoRts];
			ComPtr<ID3D11Texture2D> own[2][kSsaoRts];
			bool                    ownValid[2]{ false, false };
		};
		SsaoHistory g_ssaoHist;
		ID3D11Texture2D* SsaoTarget(int a_i)
		{
			auto* rd2 = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			return rd2 ? reinterpret_cast<ID3D11Texture2D*>(rd2->renderTargets[kSsaoRtIndex[a_i]].texture) : nullptr;
		}
		bool EnsureLike(ComPtr<ID3D11Texture2D>& a_dst, ID3D11Texture2D* a_src)
		{
			if (!a_src) {
				return false;
			}
			D3D11_TEXTURE2D_DESC sd{};
			a_src->GetDesc(&sd);
			if (a_dst) {
				D3D11_TEXTURE2D_DESC dd{};
				a_dst->GetDesc(&dd);
				if (dd.Width == sd.Width && dd.Height == sd.Height && dd.Format == sd.Format) {
					return true;
				}
				a_dst.Reset();
			}
			sd.Usage = D3D11_USAGE_DEFAULT;
			sd.BindFlags = 0;
			sd.CPUAccessFlags = 0;
			sd.MiscFlags = 0;
			sd.ArraySize = 1;
			// (the mip count stays: target 49 is the depth pyramid with 11 mips)
			return SUCCEEDED(Device()->CreateTexture2D(&sd, nullptr, &a_dst));
		}
		// view normclear=1 (A/B of G142): the engine clears the current-frame normal target (4/5, flat 127,127,0, alpha 0) between its pre-pass and its colour Start; do the same for the virtual pass, so the
		// SSAO of the virtual view does not read the previous real frame's normals. Both halves are saved first and put back after the virtual colour pass (the engine clears the current half itself).
		bool                    g_normClear{ true };  // default ON with ssao (see view normclear)
		ComPtr<ID3D11Texture2D> g_normSave[2];
		bool                    g_normSaved{ false };
		bool                    g_ssaoDiff{ false };     // view ssaodiff=1: log the words of the SSAO shader object that our virtual calls change
		bool                    g_ssaoRestore{ false };  // view ssaorestore=1: put them back after our calls (the engine's own call then sees the state it left)
		int                     g_maskStat2{ 0 };  // view maskstat2=N: log the shadow mask target's statistics right after the maskclear and right after our colour pass, for N views
		float                   g_maskClear{ -1.0f };  // view maskclear=v (G150): clear the sun channel (R) of the shadow mask target 18 to v before our colour pass (negative = off): does the virtual picture react to the mask at all?
		bool                    g_mrt{ true };  // view mrt=1 (G143): bind the engine's colour-pass target set before our colour pass: slot 1 = motion vectors (7), slot 2 = the current normal target (id 107177), slot 3 = 112 with the snow setting
		void NormalsSaveAndFlatten(ID3D11DeviceContext* a_ctx)
		{
			g_normSaved = false;
			auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			if (!rd) {
				return;
			}
			ID3D11Texture2D* t[2]{};
			for (int i = 0; i < 2; ++i) {
				t[i] = reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[4 + i].texture);
				if (!t[i]) {
					return;
				}
				D3D11_TEXTURE2D_DESC d{};
				t[i]->GetDesc(&d);
				if (d.Format != DXGI_FORMAT_R8G8B8A8_UNORM || !EnsureLike(g_normSave[i], t[i])) {
					return;
				}
			}
			D3D11_TEXTURE2D_DESC d{};
			t[0]->GetDesc(&d);
			static std::vector<std::uint32_t> s_flat;
			const std::size_t                 n = static_cast<std::size_t>(d.Width) * d.Height;
			if (s_flat.size() != n) {
				s_flat.assign(n, 0x00007F7Fu);  // R = G = 127, B = 0, A = 0
			}
			for (int i = 0; i < 2; ++i) {
				a_ctx->CopyResource(g_normSave[i].Get(), t[i]);
				a_ctx->UpdateSubresource(t[i], 0, nullptr, s_flat.data(), d.Width * 4, 0);
			}
			g_normSaved = true;
		}
		void NormalsRestore(ID3D11DeviceContext* a_ctx)
		{
			if (!g_normSaved) {
				return;
			}
			g_normSaved = false;
			auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			if (!rd) {
				return;
			}
			for (int i = 0; i < 2; ++i) {
				if (auto* t = reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[4 + i].texture); t && g_normSave[i]) {
					a_ctx->CopyResource(t, g_normSave[i].Get());
				}
			}
		}
		void RunVirtualSsao(void* a_mgr, ID3D11DeviceContext* a_ctx, int a_view)
		{
			using RenderSsao_t = void (*)(void*, std::int32_t, std::int32_t);
			using Imagespace3_t = void (*)(void*, std::int32_t, std::int32_t, std::int32_t);
			using Imagespace7_t = void (*)(void*, std::int32_t, std::int32_t, std::int32_t, std::int32_t, std::int32_t, std::int32_t);
			using Copy_t = void (*)(void*, std::int32_t, std::int32_t);
			auto* ism = *reinterpret_cast<std::uint8_t**>(REL::ID(414660).address());
			if (!ism) {
				return;
			}
			// the SSAO reads the depth through the SRV of the main depth target, which is a COPY the engine refreshes after its own pre-pass: refresh it from the live depth of this pass
			{
				ComPtr<ID3D11Resource> srvRes;
				auto* dsData = &RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData().depthStencils[g_params.ds];
				if (auto* srv = reinterpret_cast<ID3D11ShaderResourceView*>(dsData->depthSRV)) {
					srv->GetResource(&srvRes);
				}
				ComPtr<ID3D11Resource> liveRes;
				if (g_liveDsv) {
					g_liveDsv->GetResource(&liveRes);
				}
				static int s_depthLogged = 0;
				if (s_depthLogged++ < 2) {
					logger::info("view: virtual SSAO depth: live depth resource {:p}, depth SRV resource {:p}{}", static_cast<void*>(liveRes.Get()), static_cast<void*>(srvRes.Get()), liveRes.Get() == srvRes.Get() ? " (the same)" : " (a copy)");
				}
				if (srvRes && liveRes && srvRes.Get() != liveRes.Get()) {
					a_ctx->CopyResource(srvRes.Get(), liveRes.Get());
				}
			}
			auto setRt = reinterpret_cast<SetRT_t>(REL::ID(77453).address());
			auto* const parityPtr = reinterpret_cast<std::int32_t*>(*reinterpret_cast<std::uint8_t**>(const_cast<std::uint8_t*>(ism) + 0x1C8) + 0x54);
			const std::int32_t parityBefore = *parityPtr;
			const bool         swapHist = g_params.ssaoHist && a_view >= 0 && a_view < 2;
			bool               histOk = swapHist;
			if (swapHist) {
				static bool s_presentLogged = false;
				if (!s_presentLogged) {
					s_presentLogged = true;
					std::string present;
					for (int i = 0; i < kSsaoRts; ++i) {
						present += std::format(" {}:{}", kSsaoRtIndex[i], SsaoTarget(i) ? "yes" : "no");
					}
					logger::info("view: virtual SSAO history targets present:{}", present);
				}
				for (int i = 0; i < kSsaoRts && histOk; ++i) {
					if (SsaoTarget(i)) {  // some of the pair targets are not created by the engine
						histOk = EnsureLike(g_ssaoHist.mainSave[i], SsaoTarget(i)) && EnsureLike(g_ssaoHist.own[a_view][i], SsaoTarget(i));
					}
				}
				if (histOk) {
					for (int i = 0; i < kSsaoRts; ++i) {
						if (!SsaoTarget(i)) {
							continue;
						}
						a_ctx->CopyResource(g_ssaoHist.mainSave[i].Get(), SsaoTarget(i));
						if (g_params.ssaoClear) {
							// no history at all: the AO of this view is built from this frame's samples only (these targets have no render target view, so they are overwritten with zeros)
							D3D11_TEXTURE2D_DESC td{};
							SsaoTarget(i)->GetDesc(&td);
							const UINT bpp = td.Format == DXGI_FORMAT_R8G8B8A8_UNORM ? 4 : td.Format == DXGI_FORMAT_R8_UNORM ? 1 : td.Format == DXGI_FORMAT_R8G8_UNORM ? 2 : 0;
							static std::vector<std::uint8_t> s_zeros;
							static int                       s_clearLogged = 0;
							if (bpp) {
								if (s_zeros.size() < static_cast<std::size_t>(td.Width) * td.Height * bpp) {
									s_zeros.assign(static_cast<std::size_t>(td.Width) * td.Height * bpp, 0);
								}
								a_ctx->UpdateSubresource(SsaoTarget(i), 0, nullptr, s_zeros.data(), td.Width * bpp, 0);
							} else if (s_clearLogged++ < 4) {
								logger::info("view: virtual SSAO: target {} has format {} - history not cleared", kSsaoRtIndex[i], static_cast<int>(td.Format));
							}
						} else if (g_ssaoHist.ownValid[a_view]) {
							a_ctx->CopyResource(SsaoTarget(i), g_ssaoHist.own[a_view][i].Get());
						}
					}
				}
			}
			const bool dumpNow = g_rtDumpPending && a_view == 0;
			if (dumpNow) {
				DumpSsaoInputs(a_ctx, "pre");
			}
			for (std::uint32_t slot = 0; slot < 4; ++slot) {
				setRt(a_mgr, slot, -1, 3, true);
			}
			const bool snowSetting = *reinterpret_cast<const std::uint8_t*>(REL::ID(391105).address()) != 0;
			if (snowSetting) {
				setRt(a_mgr, 4, -1, 3, true);
				setRt(a_mgr, 5, -1, 3, true);
			}
			auto* const flagSsao = reinterpret_cast<std::uint8_t*>(REL::ID(415071).address());
			auto* const flagCopy = reinterpret_cast<std::uint8_t*>(REL::ID(415072).address());
			*flagSsao = 0;
			void* ssaoShader = *reinterpret_cast<void**>(ism + 0x1C8);
			constexpr std::size_t kObjWords = 0xC0;  // 0x600 bytes of the shader object
			std::array<std::uint64_t, kObjWords> objBefore{};
			const bool                          objWatch = g_ssaoDiff || g_ssaoRestore;
			if (objWatch) {
				for (std::size_t i = 0; i < kObjWords; ++i) {
					ReadSafeT(reinterpret_cast<const std::uint8_t*>(ssaoShader) + i * 8, objBefore[i]);
				}
			}
			reinterpret_cast<RenderSsao_t>(REL::ID(107245).address())(ssaoShader, 1, 1);
			static int s_logged = 0;
			if (s_logged++ < 2) {
				logger::info("view: virtual SSAO: shader {:p}, snow setting {}, snow object {:p} flag {}, copy shader {:p}", ssaoShader, snowSetting, static_cast<const void*>(*reinterpret_cast<const std::uint8_t* const*>(ism + 0x1E8)), (*reinterpret_cast<const std::uint8_t* const*>(ism + 0x1E8)) ? *(*reinterpret_cast<const std::uint8_t* const*>(ism + 0x1E8) + 8) : -1, *reinterpret_cast<void**>(ism + 0x198));
			}
			const std::uint8_t* snowObj = *reinterpret_cast<const std::uint8_t* const*>(ism + 0x1E8);
			const bool          snowActive = snowSetting && snowObj && *(snowObj + 8) != 0;  // RenderWorld: the snow subsurface step only when the setting AND the snow object's flag are set
			if (snowActive) {
				using Snow_t = void (*)(void*, std::int32_t, std::int32_t);
				reinterpret_cast<Snow_t>(REL::ID(107267).address())(*reinterpret_cast<void**>(const_cast<std::uint8_t*>(ism) + 0x1E8), 1, 0x71);
				setRt(a_mgr, 0, -1, 3, true);
			} else {
				void* copyShader = *reinterpret_cast<void**>(ism + 0x198);
				reinterpret_cast<Imagespace3_t>(REL::ID(107219).address())(copyShader, 1, 0x71, 0);
				const auto d1 = *reinterpret_cast<const std::int32_t*>(REL::ID(411453).address());
				const auto d2 = *reinterpret_cast<const std::int32_t*>(REL::ID(411454).address());
				reinterpret_cast<Imagespace7_t>(REL::ID(107221).address())(copyShader, 0, 0, d1, d2, 0, 0);
				reinterpret_cast<Copy_t>(REL::ID(107217).address())(copyShader, 0, 0);
			}
			*flagSsao = 1;
			const std::uint8_t* c0 = *reinterpret_cast<const std::uint8_t* const*>(ism + 0x1C0);
			*flagCopy = c0 ? *(c0 + 0x10) : 0;
			reinterpret_cast<RenderSsao_t>(REL::ID(107245).address())(ssaoShader, 0x71, 1);
			if (objWatch) {
				static int s_diffLogged = 0;
				int        changed = 0;
				for (std::size_t i = 0; i < kObjWords; ++i) {
					std::uint64_t now = 0;
					ReadSafeT(reinterpret_cast<const std::uint8_t*>(ssaoShader) + i * 8, now);
					if (now != objBefore[i]) {
						++changed;
						if (g_ssaoDiff && s_diffLogged < 3 * 80) {
							++s_diffLogged;
							logger::info("view: SSAO shader object word +{:#x} changed {:#018x} -> {:#018x}", i * 8, objBefore[i], now);
						}
						if (g_ssaoRestore) {
							*reinterpret_cast<std::uint64_t*>(reinterpret_cast<std::uint8_t*>(ssaoShader) + i * 8) = objBefore[i];
						}
					}
				}
				static int s_summaryLogged = 0;
				if (s_summaryLogged++ < 6) {
					logger::info("view: SSAO shader object (0x{:x} bytes): {} word(s) changed by this view's calls{}", kObjWords * 8, changed, g_ssaoRestore ? ", put back" : "");
				}
			}
			if (dumpNow) {
				DumpSsaoInputs(a_ctx, "post");
				g_rtDumpPending = false;
			}
			if (swapHist && histOk) {
				for (int i = 0; i < kSsaoRts; ++i) {
					if (!SsaoTarget(i)) {
						continue;
					}
					a_ctx->CopyResource(g_ssaoHist.own[a_view][i].Get(), SsaoTarget(i));
					a_ctx->CopyResource(SsaoTarget(i), g_ssaoHist.mainSave[i].Get());
				}
				g_ssaoHist.ownValid[a_view] = true;
			}
			if (g_params.ssaoHist) {
				*parityPtr = parityBefore;
			}
		}
		// ---- G274 view vl=0|1 (default 0 until verified), view vlpass=<bits>: the engine's volumetric lighting (VL) for the virtual picture ----
		// The real frame's VL (read from SkyrimSE.exe 1.7.104.0, G272/G273): id 107023 (before the world draw: builds the VL volume with compute passes for the camera it is given and the sun cascades of the moment),
		// then in the post function (id 107148) the apply id 107193 (a tiny pixel shader: looks the volume up by screen position and depth, temporal blend with the previous VL through the motion vectors) into the
		// ping-pong target 43/44, the blur id 107206 (compute, temp target 45) and the composite id 107213 (into the main colour target, from the history slot), all on the REAL depth, where the oval holds the portal
		// surface and so gets (almost) no volumetric light. This runs the same steps for a virtual view, after its blended parts: virtual camera, virtual depth, the sun cascades of the virtual shadow pass,
		// the VL targets 43-45 saved and restored around the run, one private temporal history per view.
		// stages: 1 build, 2 depth copies refreshed, 4 apply, 8 blur, 16 composite, 32 per-view history, 64 save/restore of the real VL targets, 128 motion vector target cleared
		std::atomic<int>        g_inVirtualComposite{ 0 };  // G305: the composite hook ignores our own virtual composites
		std::atomic<int>        g_vlRefresh{ 1 };  // G308 view vlrefresh=0|1 (default 1): before every virtual VL run the renderer's dirty bits (compute/pixel resources, samplers, UAVs) are all set: the SECOND VL run of a frame found its compute bindings 'clean' while they were not (G306/G307: its volume stayed empty and its picture only decayed)
		std::atomic<int>        g_vlAdd{ 1 };  // G306 view vladd=0|1 (default 1): 1 = the virtual VL is kept apart and added after the engine's VL composite, 0 = baked into the picture (G281)
		std::atomic<int>        g_vlAddLog{ 8 };  // G307: log how much the kept-apart VL changes each picture (the first N frames)
		bool                    g_vlLastRan{ false };  // the last RunVirtualVl ran all its stages without an exception
		std::atomic<int>        g_vlOff{ 0 };  // view vloff=1 (TEST ONLY): the engine's VL build is told the VL is disabled for every build, 2 (G304): only for the real frame's
		std::atomic<int>        g_vlRun{ 1 };  // G281: portal volumetric lighting ON by default (view vl=0 turns it off)
		std::atomic<int>        g_vlStages{ 0xFF };
		std::atomic<int>        g_vlRestore{ 0 };  // view vlrestore=<bits> (G295/G297): 1 = put the renderer shadow state back as it was before the VL stages and mark everything dirty (G295: not enough), 2 = put the real D3D11 device state back (depth-stencil, blend, rasterizer, viewport, scissor, topology, input layout, shaders, constant buffers, textures, samplers): the VL effects set their own states behind the engine's back
		// G298: the VL of view 0 is DEFERRED until after view 1's pass when both views run in the frame (view 0's VL leaves something behind that breaks the NEXT virtual pass, G293-G297; view 1's does
		// not harm anything). At the end of view 0's pass its depth and its camera object are kept and its picture (pre-VL) stored as always; after view 1's pass RunDeferredVl puts them back, runs the VL
		// and stores the result as view 0's picture.
		struct VlDefer
		{
			bool                    allowed{ false };  // set by RenderViews: both views have their rect on screen
			bool                    want{ false };     // view 0's pass skipped its VL
			bool                    pending{ false };  // depth and camera of view 0 are kept
			ComPtr<ID3D11Texture2D> depth;
			std::vector<std::uint8_t> cam;
		};
		VlDefer                 g_vlDefer;
		std::atomic<int>        g_vlDeferMode{ 2 };  // view vldefer=0|1|2 (default 2): 0 = view 0's VL in its own pass (breaks the next pass, G286), 1 = after view 1's pass (G298: no effect, G302), 2 = in view 1's pass before view 1's own VL (G306)
		std::atomic<int>        g_vlViews{ 3 };  // view vlviews=<mask> (G293): which virtual views run the VL (bit 0 = view 0, bit 1 = view 1)
		std::atomic<int>        g_vlLogLeft{ 8 };
		ComPtr<ID3D11Texture2D> g_vlHist[2], g_vlSave[3];
		bool                    g_vlHistValid[2]{ false, false };
		struct VlSeq
		{
			void*         mgr;
			void*         cam;
			void*         applyObj;
			void*         blurObj;
			void*         compObj;
			std::uint8_t* lensObj;
			std::uint8_t  flag;
			int           hist;
			int           out;
			int           rt;
			int           stages;
		};
		// the engine calls, one guarded call per stage (no C++ objects here: __try); the filter records where an exception happened
		struct VlEx
		{
			unsigned long rip{ 0 };      // rva of the faulting instruction (0 = outside the executable)
			unsigned long long addr{ 0 };  // the address the instruction tried to access
			unsigned long long info0{ 0 };  // 0 read, 1 write, 8 execute
			unsigned long      code{ 0 };
		};
		VlEx        g_vlEx[4];
		char        g_vlExModule[4][128]{};
		unsigned long g_vlExStack[4][14]{};
		unsigned long long g_vlExRipAbs[4]{};
		unsigned long long g_vlExRegs[4][4]{};
		long VlFilter(EXCEPTION_POINTERS* a_p, int a_slot)
		{
			const auto* r = a_p->ExceptionRecord;
			const auto  base = static_cast<std::uintptr_t>(REL::Module::get().base());
			const auto  rip = reinterpret_cast<std::uintptr_t>(r->ExceptionAddress);
			g_vlEx[a_slot].rip = (rip >= base && rip < base + 0x6000000) ? static_cast<unsigned long>(rip - base) : 0;
			g_vlEx[a_slot].code = r->ExceptionCode;
			g_vlEx[a_slot].addr = r->NumberParameters > 1 ? r->ExceptionInformation[1] : 0;
			g_vlEx[a_slot].info0 = r->NumberParameters > 0 ? r->ExceptionInformation[0] : 0;
			g_vlExRipAbs[a_slot] = rip;
			g_vlExRegs[a_slot][0] = a_p->ContextRecord->Rcx;
			g_vlExRegs[a_slot][1] = a_p->ContextRecord->Rdx;
			g_vlExRegs[a_slot][2] = a_p->ContextRecord->R8;
			g_vlExRegs[a_slot][3] = a_p->ContextRecord->R15;
			g_vlExModule[a_slot][0] = 0;
			HMODULE mod = nullptr;
			if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCSTR>(rip), &mod) && mod) {
				char full[MAX_PATH]{};
				GetModuleFileNameA(mod, full, MAX_PATH);
				const char* slash = strrchr(full, '\\');
				strncpy_s(g_vlExModule[a_slot], slash ? slash + 1 : full, _TRUNCATE);
				g_vlExRipAbs[a_slot] = rip - reinterpret_cast<std::uintptr_t>(mod);  // offset inside that module
			}
			void*        fr[14]{};
			const USHORT nfr = CaptureStackBackTrace(0, 14, fr, nullptr);
			for (USHORT q = 0; q < 14; ++q) {
				const auto a = q < nfr ? reinterpret_cast<std::uintptr_t>(fr[q]) : 0;
				g_vlExStack[a_slot][q] = (a >= base && a < base + 0x6000000) ? static_cast<unsigned long>(a - base) : 0;
			}
			return EXCEPTION_EXECUTE_HANDLER;
		}
		unsigned long VlCallBuild(const VlSeq& a)
		{
			__try {
				// G278/G279: id 77507 (called by the build's helper id 107027 for (camera, flag byte 0)) searches the camera entry TABLE (state+0xB8, count +0xC8, stride 0x290, key = camera pointer and the flag byte at +0x284);
				// id 77503 only looks in that table and otherwise fills a scratch entry, so our camera is never found. id 77506 (BuildCameraStateData) finds or ADDS the entry and fills it: register both variants, then make it current as the frame function does
				reinterpret_cast<void (*)(void*, void*, std::uint8_t, std::uint8_t)>(REL::ID(77506).address())(reinterpret_cast<void*>(REL::ID(411479).address()), a.cam, 0, 0);
				reinterpret_cast<void (*)(void*, void*, std::uint8_t, std::uint8_t)>(REL::ID(77506).address())(reinterpret_cast<void*>(REL::ID(411479).address()), a.cam, 1, 0);
				reinterpret_cast<void (*)(void*, void*, std::uint8_t)>(REL::ID(77503).address())(reinterpret_cast<void*>(REL::ID(411479).address()), a.cam, 1);
				reinterpret_cast<void (*)(void*, void*, std::uint8_t)>(REL::ID(107023).address())(a.mgr, a.cam, a.flag);
				return 0;
			} __except (VlFilter(GetExceptionInformation(), 0)) {
				return GetExceptionCode();
			}
		}
		unsigned long VlCallApply(const VlSeq& a)
		{
			__try {
				reinterpret_cast<void (*)(void*, std::int32_t, std::int32_t)>(REL::ID(107193).address())(a.applyObj, a.hist, a.out);
				return 0;
			} __except (VlFilter(GetExceptionInformation(), 1)) {
				return GetExceptionCode();
			}
		}
		unsigned long VlCallBlur(const VlSeq& a)
		{
			__try {
				reinterpret_cast<void (*)(void*, std::int32_t, std::int32_t, std::int32_t)>(REL::ID(107206).address())(a.blurObj, a.out, a.hist, 45);
				return 0;
			} __except (VlFilter(GetExceptionInformation(), 2)) {
				return GetExceptionCode();
			}
		}
		unsigned long VlCallComposite(const VlSeq& a)
		{
			std::uint8_t saved = 0;
			if (a.lensObj) {
				saved = *a.lensObj;
				*a.lensObj = 0;  // the lens flare variant of the composite stays out: the plain volumetric lighting variant
			}
			unsigned long r = 0;
			g_inVirtualComposite.fetch_add(1);
			__try {
				reinterpret_cast<std::int32_t (*)(void*, std::int32_t, std::int32_t)>(REL::ID(107213).address())(a.compObj, a.rt, a.rt);
			} __except (VlFilter(GetExceptionInformation(), 3)) {
				r = GetExceptionCode();
			}
			g_inVirtualComposite.fetch_sub(1);
			if (a.lensObj) {
				*a.lensObj = saved;
			}
			return r;
		}
		// the camera entry table of the camera state (id 411479) moved between runtimes: data pointer +0xB8 / count +0xC8 on 1.7.104, +0xA8 / +0xB8 on 1.6.1170
		// (G417 dump on 1.6.1170: +0xA8 heap pointer, +0xB8 = 7 entries whose keys at stride 0x290 are the engine's cameras and ours); stride and flag offset are the same
		namespace CamTable
		{
			inline bool Old() { static const bool v = REL::Module::get().version() < REL::Version(1, 7, 0, 0); return v; }
			inline std::size_t Data() { return Old() ? 0xA8 : 0xB8; }
			inline std::size_t Count() { return Old() ? 0xB8 : 0xC8; }
		}
		void* SunShadowLight();
		// the camera entry table (state id 411479: data +0xB8, count +0xC8, stride 0x290; key pointer at +0, flag byte at +0x284) and the result of the pure lookup id 77507 for a few keys
		void VlLogTable(const char* a_tag, const void* a_cam)
		{
			const auto*   st = reinterpret_cast<const std::uint8_t*>(REL::ID(411479).address());
			std::uint32_t n = 0;
			std::uint64_t data = 0;
			ReadSafeT(st + CamTable::Count(), n);
			ReadSafeT(st + CamTable::Data(), data);
			std::string keys;
			for (std::uint32_t i = 0; i < n && i < 10; ++i) {
				std::uint64_t k = 0;
				std::uint8_t  f = 0;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(data) + i * 0x290, k);
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(data) + i * 0x290 + 0x284, f);
				keys += std::format(" [{}] {:#x}/{}", i, k, f);
			}
			auto look = [&](const void* c, std::uint8_t f) { return reinterpret_cast<void* (*)(const void*, const void*, std::uint8_t)>(REL::ID(77507).address())(st, c, f); };
			const void* wc = static_cast<const void*>(RE::Main::WorldRootCamera());
			logger::info("view: virtual VL camera table {}: {} entries at {:#x}:{}; lookups: ours (camera {:p}) flag0 {:p} flag1 {:p}, world camera ({:p}) flag0 {:p} flag1 {:p}", a_tag, n, data, keys, a_cam, look(a_cam, 0), look(a_cam, 1), wc, look(wc, 0), look(wc, 1));
		}
		// G290 diagnostic (view vlsnap=N): snapshots of the main colour target around the VL stages of the next N runs (staging copy, 16 x 9 luminance grid, per-pixel comparison): which stage changes the picture?
		std::atomic<int>        g_vlSnap{ 0 };
		ComPtr<ID3D11Texture2D> g_snapStage;
		float HalfToFloat(std::uint16_t h)
		{
			const std::uint32_t s = (h >> 15) & 1u, e = (h >> 10) & 31u, m = h & 1023u;
			float               v;
			if (e == 0) v = std::ldexp(static_cast<float>(m), -24);
			else if (e == 31) v = m ? std::numeric_limits<float>::quiet_NaN() : std::numeric_limits<float>::infinity();
			else v = std::ldexp(static_cast<float>(m | 1024u), static_cast<int>(e) - 25);
			return s ? -v : v;
		}
		struct ColourSnap
		{
			std::vector<std::uint16_t> px;  // RGBA halves
			std::vector<float>         grid;  // 16 x 9 mean luminance
			UINT                       w{ 0 }, h{ 0 };
		};
		bool TakeSnap(ID3D11DeviceContext* a_ctx, ID3D11Texture2D* a_src, ColourSnap& a_out)
		{
			D3D11_TEXTURE2D_DESC d{};
			a_src->GetDesc(&d);
			if (d.Format != DXGI_FORMAT_R16G16B16A16_FLOAT || d.SampleDesc.Count != 1) {
				return false;
			}
			D3D11_TEXTURE2D_DESC sd = d;
			if (g_snapStage) {
				D3D11_TEXTURE2D_DESC cur{};
				g_snapStage->GetDesc(&cur);
				if (cur.Width != d.Width || cur.Height != d.Height) {
					g_snapStage.Reset();
				}
			}
			if (!g_snapStage) {
				sd.Usage = D3D11_USAGE_STAGING;
				sd.BindFlags = 0;
				sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
				sd.MiscFlags = 0;
				sd.ArraySize = 1;
				sd.MipLevels = 1;
				if (FAILED(Device()->CreateTexture2D(&sd, nullptr, &g_snapStage))) {
					return false;
				}
			}
			a_ctx->CopyResource(g_snapStage.Get(), a_src);
			D3D11_MAPPED_SUBRESOURCE m{};
			if (FAILED(a_ctx->Map(g_snapStage.Get(), 0, D3D11_MAP_READ, 0, &m))) {
				return false;
			}
			a_out.w = d.Width;
			a_out.h = d.Height;
			a_out.px.resize(static_cast<std::size_t>(d.Width) * d.Height * 4);
			for (UINT y = 0; y < d.Height; ++y) {
				std::memcpy(&a_out.px[static_cast<std::size_t>(y) * d.Width * 4], static_cast<const std::uint8_t*>(m.pData) + static_cast<std::size_t>(y) * m.RowPitch, static_cast<std::size_t>(d.Width) * 8);
			}
			a_ctx->Unmap(g_snapStage.Get(), 0);
			a_out.grid.assign(16 * 9, 0.0f);
			std::vector<int> cnt(16 * 9, 0);
			for (UINT y = 0; y < d.Height; y += 4) {
				for (UINT x = 0; x < d.Width; x += 4) {
					const auto* p = &a_out.px[(static_cast<std::size_t>(y) * d.Width + x) * 4];
					float       l = 0.2126f * HalfToFloat(p[0]) + 0.7152f * HalfToFloat(p[1]) + 0.0722f * HalfToFloat(p[2]);
					if (!std::isfinite(l)) l = 0.0f;
					const std::size_t gi = static_cast<std::size_t>(y * 9 / d.Height) * 16 + static_cast<std::size_t>(x * 16 / d.Width);
					a_out.grid[gi] += l;
					++cnt[gi];
				}
			}
			for (std::size_t i = 0; i < a_out.grid.size(); ++i) a_out.grid[i] /= static_cast<float>((std::max)(cnt[i], 1));
			return true;
		}
		std::string GridText(const ColourSnap& s)
		{
			std::string t;
			for (int r = 0; r < 9; ++r) {
				for (int c = 0; c < 16; ++c) t += std::format(" {:.2f}", s.grid[static_cast<std::size_t>(r) * 16 + static_cast<std::size_t>(c)]);
				t += " |";
			}
			return t;
		}
		void CompareSnap(const char* a_what, int a_view, const ColourSnap& a, const ColourSnap& b)
		{
			if (a.px.size() != b.px.size() || a.px.empty()) {
				return;
			}
			std::size_t changed = 0, total = a.px.size() / 4;
			double      sum = 0;
			for (std::size_t i = 0; i < total; ++i) {
				bool ch = false;
				for (int c = 0; c < 3; ++c) {
					const float va = HalfToFloat(a.px[i * 4 + c]), vb = HalfToFloat(b.px[i * 4 + c]);
					const float dd = std::fabs(va - vb);
					if (dd > 0.004f + 0.02f * std::fabs(va)) ch = true;
					if (std::isfinite(dd)) sum += dd;
				}
				if (ch) ++changed;
			}
			logger::info("view: vlsnap view {}: {}: {} of {} pixels changed ({:.1f} %), mean abs change per channel {:.4f}", a_view, a_what, changed, total, 100.0 * static_cast<double>(changed) / static_cast<double>(total), sum / static_cast<double>(total * 3));
		}
		// the real D3D11 device state around the VL stages (G297): the image-space effects bind their own depth-stencil / blend / rasterizer states, shaders, textures, samplers and viewport behind the engine's shadow state
		struct D3dSave
		{
			ComPtr<ID3D11DepthStencilState> dss;
			UINT                            dssRef{ 0 };
			ComPtr<ID3D11BlendState>        bs;
			FLOAT                           bf[4]{};
			UINT                            bsMask{ 0xFFFFFFFF };
			ComPtr<ID3D11RasterizerState>   rs;
			UINT                            nvp{ 0 }, nsc{ 0 };
			D3D11_VIEWPORT                  vp[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE]{};
			D3D11_RECT                      sc[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE]{};
			D3D11_PRIMITIVE_TOPOLOGY        topo{};
			ComPtr<ID3D11InputLayout>       il;
			ComPtr<ID3D11VertexShader>      vs;
			ComPtr<ID3D11PixelShader>       ps;
			ComPtr<ID3D11GeometryShader>    gs;
			ComPtr<ID3D11Buffer>            vcb[8], pcb[8];
			ComPtr<ID3D11ShaderResourceView> srv[16];
			ComPtr<ID3D11SamplerState>      smp[16];
			void Take(ID3D11DeviceContext* c)
			{
				c->OMGetDepthStencilState(&dss, &dssRef);
				c->OMGetBlendState(&bs, bf, &bsMask);
				c->RSGetState(&rs);
				nvp = nsc = D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE;
				c->RSGetViewports(&nvp, vp);
				c->RSGetScissorRects(&nsc, sc);
				c->IAGetPrimitiveTopology(&topo);
				c->IAGetInputLayout(&il);
				c->VSGetShader(&vs, nullptr, nullptr);
				c->PSGetShader(&ps, nullptr, nullptr);
				c->GSGetShader(&gs, nullptr, nullptr);
				ID3D11Buffer* b[8]{};
				c->VSGetConstantBuffers(0, 8, b);
				for (int i = 0; i < 8; ++i) vcb[i].Attach(b[i]);
				ID3D11Buffer* b2[8]{};
				c->PSGetConstantBuffers(0, 8, b2);
				for (int i = 0; i < 8; ++i) pcb[i].Attach(b2[i]);
				ID3D11ShaderResourceView* s[16]{};
				c->PSGetShaderResources(0, 16, s);
				for (int i = 0; i < 16; ++i) srv[i].Attach(s[i]);
				ID3D11SamplerState* m[16]{};
				c->PSGetSamplers(0, 16, m);
				for (int i = 0; i < 16; ++i) smp[i].Attach(m[i]);
			}
			void Put(ID3D11DeviceContext* c)
			{
				c->OMSetDepthStencilState(dss.Get(), dssRef);
				c->OMSetBlendState(bs.Get(), bf, bsMask);
				c->RSSetState(rs.Get());
				c->RSSetViewports(nvp, vp);
				c->RSSetScissorRects(nsc, sc);
				c->IASetPrimitiveTopology(topo);
				c->IASetInputLayout(il.Get());
				c->VSSetShader(vs.Get(), nullptr, 0);
				c->PSSetShader(ps.Get(), nullptr, 0);
				c->GSSetShader(gs.Get(), nullptr, 0);
				ID3D11Buffer* b[8]{};
				for (int i = 0; i < 8; ++i) b[i] = vcb[i].Get();
				c->VSSetConstantBuffers(0, 8, b);
				for (int i = 0; i < 8; ++i) b[i] = pcb[i].Get();
				c->PSSetConstantBuffers(0, 8, b);
				ID3D11ShaderResourceView* s[16]{};
				for (int i = 0; i < 16; ++i) s[i] = srv[i].Get();
				c->PSSetShaderResources(0, 16, s);
				ID3D11SamplerState* m[16]{};
				for (int i = 0; i < 16; ++i) m[i] = smp[i].Get();
				c->PSSetSamplers(0, 16, m);
			}
		};
		// G292: what do the engine's VL calls change in our camera object and in the camera state (id 411479)? (view vlsnap=N logs the differing dwords after each stage)
		struct MemWatch
		{
			std::uint8_t cam[0x400]{};
			std::uint8_t st[0x100]{};
			std::uint8_t shadow[0x5e0]{};  // the renderer shadow state (id 388819): cached render targets, viewport, textures, shaders, camera data, position adjust, near/far
		};
		void MemTake(MemWatch& a_w, const void* a_cam)
		{
			std::memcpy(a_w.cam, a_cam, (std::min)(sizeof(a_w.cam), sizeof(RE::NiCamera)));
			std::memcpy(a_w.st, reinterpret_cast<const void*>(REL::ID(411479).address()), sizeof(a_w.st));
			std::memcpy(a_w.shadow, reinterpret_cast<const void*>(REL::ID(388819).address()), sizeof(a_w.shadow));
		}
		void MemDiff(const char* a_what, int a_view, const MemWatch& a_before, const void* a_cam)
		{
			MemWatch now;
			MemTake(now, a_cam);
			std::string txt;
			int         n = 0;
			for (std::size_t i = 0; i + 4 <= (std::min)(sizeof(now.cam), sizeof(RE::NiCamera)); i += 4) {
				if (std::memcmp(a_before.cam + i, now.cam + i, 4) != 0 && n < 40) {
					std::uint32_t o, v;
					std::memcpy(&o, a_before.cam + i, 4);
					std::memcpy(&v, now.cam + i, 4);
					txt += std::format(" cam+{:#x}:{:#x}->{:#x}", i, o, v);
					++n;
				}
			}
			int m = 0;
			std::string st;
			for (std::size_t i = 0; i < sizeof(now.st); i += 4) {
				if (std::memcmp(a_before.st + i, now.st + i, 4) != 0 && m < 40) {
					std::uint32_t o, v;
					std::memcpy(&o, a_before.st + i, 4);
					std::memcpy(&v, now.st + i, 4);
					st += std::format(" state+{:#x}:{:#x}->{:#x}", i, o, v);
					++m;
				}
			}
			int         k = 0;
			std::string sh;
			for (std::size_t i = 0; i < sizeof(now.shadow); i += 4) {
				if (std::memcmp(a_before.shadow + i, now.shadow + i, 4) != 0) {
					if (k < 70) {
						std::uint32_t o, v;
						std::memcpy(&o, a_before.shadow + i, 4);
						std::memcpy(&v, now.shadow + i, 4);
						sh += std::format(" {:#x}:{:#x}->{:#x}", i, o, v);
					}
					++k;
				}
			}
			logger::info("view: vlsnap view {}: {}: camera object changes:{} | camera state changes:{} | renderer shadow state ({} dwords changed):{}", a_view, a_what, n ? txt : " none", m ? st : " none", k, k ? sh : " none");
		}
		void RunVirtualVl(void* a_mgr, ID3D11DeviceContext* a_ctx, int a_view)
		{
			(void)a_mgr;
			g_vlLastRan = false;
			auto* ism = *reinterpret_cast<std::uint8_t**>(REL::ID(414660).address());
			if (!ism || a_view < 0 || a_view > 1) {
				return;
			}
			auto*             applyObj = *reinterpret_cast<std::uint8_t**>(ism + 0x170);
			auto*             blurObj = *reinterpret_cast<std::uint8_t**>(ism + 0x188);
			auto*             compObj = *reinterpret_cast<std::uint8_t**>(ism + 0x190);
			auto*             lensBase = *reinterpret_cast<std::uint8_t**>(ism + 0x180);
			const std::uint8_t g414913 = *reinterpret_cast<const std::uint8_t*>(REL::ID(414913).address());
			auto*             vlm = reinterpret_cast<std::uint8_t* (*)()>(REL::ID(107014).address())();
			auto*             flagObj = reinterpret_cast<std::uint8_t* (*)()>(REL::ID(107121).address())();
			// G304 test only (view vloff=2): the real frame's build has set the apply effect inactive, so that the engine's own VL steps are skipped; ours run with it active and put it back
			struct ForceActive
			{
				std::uint8_t* p;
				bool          on;
				~ForceActive()
				{
					if (on && p) {
						p[8] = 0;
					}
				}
			} forceActive{ applyObj, g_vlOff.load() == 2 && applyObj != nullptr };
			if (forceActive.on) {
				applyObj[8] = 1;
			}
			if (!applyObj || !blurObj || !compObj || !vlm || !flagObj || !g414913 || applyObj[8] == 0) {
				static int s_skipLogged = 0;
				if (s_skipLogged++ < 3) {
					logger::info("view: virtual VL skipped (apply {:p} active {}, blur {:p}, composite {:p}, manager {:p}, global 414913 = {})", static_cast<void*>(applyObj), applyObj ? applyObj[8] : 0, static_cast<void*>(blurObj), static_cast<void*>(compObj), static_cast<void*>(vlm), g414913);
				}
				return;
			}
			if (!SunShadowLight()) {
				return;  // no sun shadow light (interiors, some weathers): the build helper reads the sun light's cascades, so there is no virtual VL either
			}
			if (!(g_vlViews.load() & (1 << a_view))) {
				return;
			}
			const int stages = g_vlStages.load();
			const std::uint32_t x = *reinterpret_cast<const std::uint32_t*>(vlm + 0x88);
			const int           out = 43 + static_cast<int>(x & 1u);
			const int           hist = 43 + static_cast<int>((~x) & 1u);
			auto*               rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			if (!rd) {
				return;
			}
			ID3D11Texture2D* vlTex[3] = { reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[43].texture), reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[44].texture), reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[45].texture) };
			if (!vlTex[0] || !vlTex[1] || !vlTex[2]) {
				return;
			}
			bool ok = true;
			if (stages & 64) {
				for (int i = 0; i < 3 && ok; ++i) {
					ok = EnsureLike(g_vlSave[i], vlTex[i]);
				}
			}
			if (stages & 32) {
				ok = ok && EnsureLike(g_vlHist[a_view], vlTex[hist - 43]);
			}
			if (!ok) {
				static int s_failLogged = 0;
				if (s_failLogged++ < 3) {
					logger::info("view: virtual VL: private copies of the VL targets could not be made");
				}
				return;
			}
			const auto   nowMs = [] { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count(); };
			const double t0 = nowMs();
			if (stages & 64) {
				for (int i = 0; i < 3; ++i) {
					a_ctx->CopyResource(g_vlSave[i].Get(), vlTex[i]);
				}
			}
			if ((stages & 32) && g_vlHistValid[a_view]) {
				a_ctx->CopyResource(vlTex[hist - 43], g_vlHist[a_view].Get());
			}
			if (stages & 2) {
				// the lookup reads the depth through a copy of the depth target (rt116, the depth views): refresh them from the live depth of this pass, as the virtual SSAO does
				ComPtr<ID3D11Resource> liveRes;
				if (g_liveDsv) {
					g_liveDsv->GetResource(&liveRes);
				}
				ComPtr<ID3D11Texture2D> live;
				D3D11_TEXTURE2D_DESC    ld{};
				if (liveRes && SUCCEEDED(liveRes.As(&live))) {
					live->GetDesc(&ld);
				}
				auto&                           dsd = RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData();
				std::array<ID3D11Resource*, 3>  dsts{};
				ComPtr<ID3D11Resource>          r1, r2;
				if (auto* srv1 = reinterpret_cast<ID3D11ShaderResourceView*>(dsd.depthStencils[1].depthSRV)) {
					srv1->GetResource(&r1);
				}
				if (auto* srv2 = reinterpret_cast<ID3D11ShaderResourceView*>(dsd.depthStencils[g_params.ds].depthSRV)) {
					srv2->GetResource(&r2);
				}
				dsts[0] = r1.Get();
				dsts[1] = r2.Get();
				dsts[2] = reinterpret_cast<ID3D11Resource*>(rd->renderTargets[116].texture);
				static int s_dLogged = 0;
				for (std::size_t i = 0; i < dsts.size(); ++i) {
					if (!dsts[i] || !live || dsts[i] == liveRes.Get()) {
						continue;
					}
					bool dup = false;
					for (std::size_t j = 0; j < i; ++j) {
						dup = dup || dsts[j] == dsts[i];
					}
					ComPtr<ID3D11Texture2D> dt;
					D3D11_TEXTURE2D_DESC    dd{};
					if (dup || FAILED(dsts[i]->QueryInterface(IID_PPV_ARGS(&dt)))) {
						continue;
					}
					dt->GetDesc(&dd);
					const bool same = dd.Width == ld.Width && dd.Height == ld.Height && dd.Format == ld.Format && dd.SampleDesc.Count == ld.SampleDesc.Count;
					if (s_dLogged++ < 6) {
						logger::info("view: virtual VL depth copy {}: {}x{} fmt {} against the live depth {}x{} fmt {}: {}", i, dd.Width, dd.Height, static_cast<int>(dd.Format), ld.Width, ld.Height, static_cast<int>(ld.Format), same ? "copied" : "skipped (different)");
					}
					if (same) {
						a_ctx->CopyResource(dsts[i], liveRes.Get());
					}
				}
			}
			if (stages & 128) {
				if (auto* mvRtv = reinterpret_cast<ID3D11RenderTargetView*>(rd->renderTargets[7].RTV)) {
					const FLOAT zero[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
					a_ctx->ClearRenderTargetView(mvRtv, zero);
				}
			}
			if (g_vlLogLeft.load() > 0) {
				std::string f;
				for (int o = 0x30; o <= 0x90; o += 8) {
					std::uint64_t v = 0;
					ReadSafeT(vlm + o, v);
					f += std::format(" +{:#x}={:#x}", o, v);
				}
				logger::info("view: virtual VL manager fields before the build:{}", f);
			}
			if (g_vlRefresh.load() != 0) {
				auto* shadowDirty = reinterpret_cast<std::uint8_t*>(REL::ID(388819).address());
				*reinterpret_cast<std::uint32_t*>(shadowDirty + 0x00) = 0xFFFFFFFFu;  // stateUpdateFlags
				*reinterpret_cast<std::uint32_t*>(shadowDirty + 0x04) = 0xFFFFu;      // PSResourceModifiedBits
				*reinterpret_cast<std::uint32_t*>(shadowDirty + 0x08) = 0xFFFFu;      // PSSamplerModifiedBits
				*reinterpret_cast<std::uint32_t*>(shadowDirty + 0x0C) = 0xFFFFu;      // CSResourceModifiedBits
				*reinterpret_cast<std::uint32_t*>(shadowDirty + 0x10) = 0xFFFFu;      // CSSamplerModifiedBits
				*reinterpret_cast<std::uint32_t*>(shadowDirty + 0x14) = 0xFFu;        // CSUAVModifiedBits
			}
			VlSeq q{};
			q.mgr = vlm;
			q.cam = g_cam;
			q.applyObj = applyObj;
			q.blurObj = blurObj;
			q.compObj = compObj;
			q.lensObj = lensBase ? lensBase + 0x30 : nullptr;
			q.flag = flagObj[0x51];
			q.hist = hist;
			q.out = out;
			q.rt = g_params.rt;
			q.stages = stages;
			auto** const imgCamGlobal = reinterpret_cast<void**>(REL::ID(414580).address());
			void* const  imgCamSaved = *imgCamGlobal;
			*imgCamGlobal = g_cam;  // the post function (id 107148) sets this global to the world camera before its effects; our camera for the virtual steps
			const std::uint32_t camEntriesBefore = *reinterpret_cast<const std::uint32_t*>(REL::ID(411479).address() + CamTable::Count());
			unsigned long ex = 0, exs[4]{ 0, 0, 0, 0 };
			const std::uint32_t tableBefore = *reinterpret_cast<const std::uint32_t*>(REL::ID(411479).address() + CamTable::Count());
			const bool logTable = g_vlLogLeft.load() > 0;
			if (logTable && (stages & 1)) VlLogTable("before registering", g_cam);
			std::uint8_t shadowPre[0x5e0];
			std::memcpy(shadowPre, reinterpret_cast<const void*>(REL::ID(388819).address()), sizeof(shadowPre));
			D3dSave d3dPre;
			if (g_vlRestore.load() & 2) d3dPre.Take(a_ctx);
			ColourSnap snapA, snapB, snapC;
			const bool snapOn = g_vlSnap.load() > 0 && rd->renderTargets[g_params.rt].texture;
			auto*       snapTex = snapOn ? reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[g_params.rt].texture) : nullptr;
			bool        haveA = snapOn && TakeSnap(a_ctx, snapTex, snapA);
			if (haveA) logger::info("view: vlsnap view {} before the VL stages (stages {:#x}) luminance grid:{}", a_view, stages, GridText(snapA));
			MemWatch memW;
			if (snapOn) MemTake(memW, g_cam);
			if (stages & 1) exs[0] = VlCallBuild(q);
			if (snapOn && (stages & 1)) MemDiff("after the build", a_view, memW, g_cam);
			if (logTable && (stages & 1)) VlLogTable("after the build call", g_cam);
			if ((stages & 4) && !exs[0]) exs[1] = VlCallApply(q);
			if (snapOn && (stages & 4)) MemDiff("after build + apply", a_view, memW, g_cam);
			if (haveA && TakeSnap(a_ctx, snapTex, snapB)) CompareSnap("build + apply", a_view, snapA, snapB);
			if ((stages & 8) && !exs[0] && !exs[1]) exs[2] = VlCallBlur(q);
			if ((stages & 16) && !exs[0] && !exs[1] && !exs[2]) exs[3] = VlCallComposite(q);
			ex = exs[0] | exs[1] | exs[2] | exs[3];
			if (haveA && TakeSnap(a_ctx, snapTex, snapC)) {
				CompareSnap("all stages", a_view, snapA, snapC);
				logger::info("view: vlsnap view {} after the VL stages luminance grid:{}", a_view, GridText(snapC));
			}
			if (haveA) --g_vlSnap;
			const std::uint32_t camEntriesAfter = *reinterpret_cast<const std::uint32_t*>(REL::ID(411479).address() + CamTable::Count());
			*imgCamGlobal = imgCamSaved;
			{
				// take our camera's entries out of the table again: only when every added entry is ours (key = our camera pointer)
				auto*         st = reinterpret_cast<std::uint8_t*>(REL::ID(411479).address());
				const auto    nNow = *reinterpret_cast<const std::uint32_t*>(st + CamTable::Count());
				const auto*   dat = *reinterpret_cast<const std::uint8_t* const*>(st + CamTable::Data());
				bool          allOurs = nNow > tableBefore && dat;
				for (std::uint32_t i = tableBefore; allOurs && i < nNow; ++i) {
					std::uint64_t key = 0;
					ReadSafeT(dat + static_cast<std::size_t>(i) * 0x290, key);
					allOurs = key == reinterpret_cast<std::uint64_t>(g_cam);
				}
				if (allOurs) {
					*reinterpret_cast<std::uint32_t*>(st + CamTable::Count()) = tableBefore;
				}
			}
			if (ex && g_vlLogLeft.load() > 0) {
				for (int e = 0; e < 4; ++e) {
					if (exs[e]) {
						std::string stk;
						for (int q = 0; q < 14; ++q) {
							stk += std::format(" {:#x}", g_vlExStack[e][q]);
						}
						logger::info("view: virtual VL stage {} (build, apply, blur, composite) raised {:#x} at rva {:#x} (module '{}' offset {:#x}), access type {} address {:#x}; registers rcx {:#x} rdx {:#x} r8 {:#x} r15 {:#x}; exe stack (rva, 0 = other module):{}", e, exs[e], g_vlEx[e].rip, g_vlExModule[e], g_vlExRipAbs[e], g_vlEx[e].info0, g_vlEx[e].addr, g_vlExRegs[e][0], g_vlExRegs[e][1], g_vlExRegs[e][2], g_vlExRegs[e][3], stk);
					}
				}
			}
			if (g_vlRestore.load() & 2) d3dPre.Put(a_ctx);
			if (g_vlRestore.load() & 1) {
				// the engine's image-space effects leave their render target (none), viewport, textures, samplers and shaders in the renderer shadow state: put it back as it was and set every dirty bit (the all-ones values are the engine's own reset state), so the next virtual pass starts from the state it expects
				auto* shadow = reinterpret_cast<std::uint8_t*>(REL::ID(388819).address());
				std::memcpy(shadow, shadowPre, sizeof(shadowPre));
				*reinterpret_cast<std::uint32_t*>(shadow + 0x00) = 0xFFFFFFFFu;  // stateUpdateFlags
				*reinterpret_cast<std::uint32_t*>(shadow + 0x04) = 0xFFFFu;      // PSResourceModifiedBits
				*reinterpret_cast<std::uint32_t*>(shadow + 0x08) = 0xFFFFu;      // PSSamplerModifiedBits
				*reinterpret_cast<std::uint32_t*>(shadow + 0x0C) = 0xFFFFu;      // CSResourceModifiedBits
				*reinterpret_cast<std::uint32_t*>(shadow + 0x10) = 0xFFFFu;      // CSSamplerModifiedBits
				*reinterpret_cast<std::uint32_t*>(shadow + 0x14) = 0xFFu;        // CSUAVModifiedBits
			}
			if ((stages & 32) && !ex) {
				a_ctx->CopyResource(g_vlHist[a_view].Get(), vlTex[out - 43]);
				g_vlHistValid[a_view] = true;
			}
			if (stages & 64) {
				for (int i = 0; i < 3; ++i) {
					a_ctx->CopyResource(vlTex[i], g_vlSave[i].Get());
				}
			}
			{
				static int s_runs = 0, s_tableLogged = 0;
				if ((++s_runs % 20) == 0 && s_tableLogged < 60) {
					++s_tableLogged;
					logger::info("view: virtual VL run {} (frame {}): camera entry table {} -> {}", s_runs, frame::Count(), camEntriesBefore, camEntriesAfter);
				}
			}
			g_vlLastRan = (ex == 0);
			if (ex) {
				g_vlRun = 0;
				logger::error("view: virtual VL raised {:#x} (stages {:#x}): switched OFF for the rest of the session", ex, stages);
			}
			if (g_vlLogLeft.load() > 0) {
				--g_vlLogLeft;
				logger::info("view: virtual VL for view {}: manager {:p} +0x88 {:#x}, history target {}, output target {}, colour target {}, stages {:#x}, exception {:#x}, camera entries {} -> {}, {:.3f} ms CPU", a_view, static_cast<void*>(vlm), x, hist, out, g_params.rt, stages, ex, camEntriesBefore, camEntriesAfter, nowMs() - t0);
			}
		}
		// ---- G306: view 0's VL inside view 1's pass (vldefer=2) and the VL kept apart from the picture (vladd=1) ----
		ComPtr<ID3D11Texture2D> g_vlTmpColour, g_vlTmpDepth;
		void LogAddStats(ID3D11DeviceContext* a_ctx, int a_view)
		{
			if (g_vlAddLog.load() <= 0) {
				return;
			}
			--g_vlAddLog;
			ColourSnap sa, sb;
			if (TakeSnap(a_ctx, g_vlPre[a_view].Get(), sa) && TakeSnap(a_ctx, g_vlPost[a_view].Get(), sb)) {
				CompareSnap("VL kept apart (picture before the VL against after it)", a_view, sa, sb);
			}
		}
		// view 0's pre-VL picture, depth and camera are put into the main targets, its VL runs in the context in which the VL worked for BOTH views in main-2026-10-20b (inside the pass, before the pass ends), then view 1's own
		// colour, depth and camera go back. The picture before and after goes to g_vlPre/g_vlPost[0] (vladd=1) or the result is baked into g_tex[0] (vladd=0).
		void RunVl0InPass(void* a_mgr, ID3D11DeviceContext* a_ctx, ID3D11Texture2D* a_mainTex)
		{
			if (!g_vlDefer.pending) {
				return;
			}
			g_vlDefer.pending = false;
			auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			auto* dsd = &RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData().depthStencils[g_params.ds];
			if (!rd || !dsd || !g_tex[0] || !g_vlDefer.depth || g_vlDefer.cam.size() != sizeof(RE::NiCamera) || !g_cam || !a_mainTex) {
				return;
			}
			auto*                  dsvT = reinterpret_cast<ID3D11DepthStencilView*>(dsd->views[0]);
			ComPtr<ID3D11Resource> liveRes;
			if (dsvT) {
				dsvT->GetResource(&liveRes);
			}
			ComPtr<ID3D11Texture2D> liveTex;
			if (!liveRes || FAILED(liveRes.As(&liveTex)) || !EnsureLike(g_vlTmpColour, a_mainTex) || !EnsureLike(g_vlTmpDepth, liveTex.Get())) {
				return;
			}
			std::vector<std::uint8_t> cam1(g_cam, g_cam + sizeof(RE::NiCamera));
			a_ctx->CopyResource(g_vlTmpColour.Get(), a_mainTex);
			a_ctx->CopyResource(g_vlTmpDepth.Get(), liveRes.Get());
			a_ctx->CopyResource(a_mainTex, g_tex[0].Get());
			a_ctx->CopyResource(liveRes.Get(), g_vlDefer.depth.Get());
			std::memcpy(g_cam, g_vlDefer.cam.data(), g_vlDefer.cam.size());
			const bool add = g_vlAdd.load() != 0 && g_vlPre[0] && g_vlPost[0];
			if (add) {
				a_ctx->CopyResource(g_vlPre[0].Get(), g_tex[0].Get());
			}
			RunVirtualVl(a_mgr, a_ctx, 0);
			if (g_vlLastRan) {
				if (add) {
					a_ctx->CopyResource(g_vlPost[0].Get(), a_mainTex);
					g_vlAddValid[0] = true;
					LogAddStats(a_ctx, 0);
				} else {
					a_ctx->CopyResource(g_tex[0].Get(), a_mainTex);
				}
			}
			a_ctx->CopyResource(a_mainTex, g_vlTmpColour.Get());
			a_ctx->CopyResource(liveRes.Get(), g_vlTmpDepth.Get());
			std::memcpy(g_cam, cam1.data(), cam1.size());
			static int s_logged = 0;
			if (s_logged++ < 4) {
				logger::info("view: view 0's VL run inside view 1's pass (vldefer=2), ran {}, kept apart {}", g_vlLastRan, add);
			}
		}
		// the VL of a view that runs in its own pass (view 1, or a single view): with vladd=1 the picture before and after is kept and the pass goes on without the VL
		void RunVlInPass(void* a_mgr, ID3D11DeviceContext* a_ctx, int a_view, ID3D11Texture2D* a_mainTex)
		{
			if (a_view == 1 && g_vlDefer.pending && g_vlDeferMode.load() == 2) {
				RunVl0InPass(a_mgr, a_ctx, a_mainTex);
			}
			const bool add = g_vlAdd.load() != 0 && a_mainTex && a_view >= 0 && a_view < 2 && g_vlPre[a_view] && g_vlPost[a_view];
			if (a_view >= 0 && a_view < 2) {
				g_vlAddValid[a_view] = false;
			}
			if (add) {
				a_ctx->CopyResource(g_vlPre[a_view].Get(), a_mainTex);
			}
			RunVirtualVl(a_mgr, a_ctx, a_view);
			if (add && g_vlLastRan) {
				a_ctx->CopyResource(g_vlPost[a_view].Get(), a_mainTex);
				a_ctx->CopyResource(a_mainTex, g_vlPre[a_view].Get());
				g_vlAddValid[a_view] = true;
				LogAddStats(a_ctx, a_view);
			}
		}
		void PokeBeforeSubmit(std::uint32_t a_value)
		{
			g_inVirtual = (a_value == 2);
			auto* dirty = reinterpret_cast<std::uint32_t*>(REL::ID(388819).address());
			auto* s9c = reinterpret_cast<std::uint32_t*>(REL::ID(388845).address());
			if (*s9c != a_value) {
				*s9c = a_value;
				*dirty |= 0x20u;
			}
		}
		// ---- render trace (G118) ----
		// What the engine's own frame does with accumulators (Renderer::SubmitAccumulator id 106436, StartAccumulating 106437, FinishAccumulatingPostResolveDepth 106438: camera, accumulator, flags,
		// the accumulator's render mode +0x150 and equal-depth byte +0x5C) and with every pass (BSBatchRenderer::SetupAndDrawPass id 107644: the alphaTest argument against what the geometry's
		// NiAlphaProperty says (geometry = pass+0x10, alpha property = geometry+0x120, flags +0x30, bit 9 = alpha test enabled)), for the next N frames, ours and the engine's apart.
		std::atomic<bool> g_traceOn{ false };
		std::atomic<int>  g_depthHack{ 0 };  // view depthhack=N (G174): override the depth-stencil state of our colour pass draws: 1 = LESS_EQUAL where a depth test is on, 2 = depth test forced on with LESS_EQUAL, 3 = as 2 plus depth writes on
		std::map<std::pair<void*, int>, ComPtr<ID3D11DepthStencilState>> g_dsHackCache;
		std::atomic<int>  g_jitLogged{ 0 };
		std::atomic<bool> g_logJitter{ false };  // view logjitter=1 (G179): log the engine's jitter data at camera builds
		std::atomic<int>  g_cullFlags{ 3 };  // view cullflags=N: the flags word of our BSCullingProcess context (ctx+0x50); 3 = kIgnoreMultiBounds, as for the cubemap (G173: does it decide the batch order?)
		std::atomic<int>  g_litSeq{ 0 }, g_seqLo{ 0 }, g_seqHi{ 0 };  // view seqrange=lo:hi: draw only our lit passes whose draw-order index (per virtual view) is in [lo, hi) (G170)
		std::atomic<int>  g_skipBitsReal{ 0 };  // view realskipbits=1/2: the same skip test for the ENGINE's own mode-0 lit passes (the real frame)
		std::atomic<int>  g_skipBits{ 0 };  // view skipbits=1: do not draw our lit passes whose technique carries the shadow bits 0x6000; skipbits=2: do not draw those without them (G165: how much of the picture do they cover?)
		std::string       g_lastSig;  // "slot:index" list of the last OMSetRenderTargets while tracing
		// (ours?, shader vtable rva, technique, pass flags, render targets bound when the draw of the pass returned) -> passes
		std::map<std::tuple<bool, std::uintptr_t, std::uint32_t, std::uint32_t, std::string>, int> g_passRt;
		int               g_traceFrames{ 0 };
		int               g_traceLines{ 0 };
		void*             g_curAcc{ nullptr };
		using Fin_t = void (*)(void*, void*, std::uint32_t);
		Fin_t g_origSubmit{ nullptr }, g_origStartAccum{ nullptr }, g_origPostDepth{ nullptr };
		using SetupPass_t = void (*)(void*, std::uint32_t, bool, std::uint32_t);
		SetupPass_t g_origSetupPass{ nullptr };
		struct PassStat
		{
			int calls{ 0 }, argTrue{ 0 }, propTest{ 0 }, propTestArgFalse{ 0 }, forced{ 0 };
		};
		std::map<std::pair<void*, bool>, PassStat> g_passStats;  // (accumulator, ours?)
		std::map<std::uint32_t, int>               g_techNoArg;  // technique id of draws with the alpha test enabled and the argument false, ours
		int                                        g_noArgLogged{ 0 };
		std::atomic<int>                           g_forcedTotal{ 0 };
		struct GroupStat
		{
			int            calls{ 0 }, argTrue{ 0 }, prop{ 0 };
			std::string    name;
			std::uintptr_t geomVt{ 0 }, propVt{ 0 };
		};
		// (accumulator, ours?, shader vtable rva, technique, pass flags) -> draws of that kind
		std::map<std::tuple<void*, bool, std::uintptr_t, std::uint32_t, std::uint32_t>, GroupStat> g_groups;
		using SetRT_t = void (*)(void*, std::uint32_t, std::int32_t, std::uint32_t, bool);
		SetRT_t g_origSetRT{ nullptr };
		std::uintptr_t Rva(const void* a_p)
		{
			return reinterpret_cast<std::uintptr_t>(a_p) - REL::Module::get().base();
		}


		// per geometry: the technique ids of its draws in our pass against the engine's mode-0 passes (is the pass list per geometry or per accumulator?)
		struct GeomTech
		{
			std::set<std::uint32_t> ours, engine;
			std::string             name;
			std::uintptr_t          shaderVt{ 0 };
		};
		std::map<const void*, GeomTech> g_geomTech;
		bool                            g_inCull{ false };  // inside our own BSCullingProcess::Process call
		std::vector<std::uint8_t>       g_engineColourAccCopy;
		void*                           g_engineColourAcc{ nullptr };
		void TraceCall(const char* a_what, void* a_cam, void* a_acc, std::uint32_t a_flags, const void* a_ret)
		{
			if (!g_traceOn.load() || g_traceLines >= 160) {
				return;
			}
			++g_traceLines;
			std::uint32_t mode = 0xFFFF;
			std::uint8_t  eq = 0xFF, fp = 0xFF;
			if (a_acc) {
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_acc) + 0x150, mode);
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_acc) + 0x5C, eq);
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_acc) + 0x128, fp);
			}
			logger::info("trace: {}{} camera {:p} accumulator {:p} flags {:#x} renderMode {} equalDepth {} firstPerson {} caller rva {:#x}", a_what, g_inVirtual ? " [OUR VIRTUAL PASS]" : "", a_cam, a_acc, a_flags, mode, eq, fp, Rva(a_ret));
		}
		// G178: an occlusion query around the draws of every lit colour pass (view occl=1 with capture=1 and trace): how many samples passed depth and stencil per pass, with the depth, stencil, raster and blend state it drew with
		struct OcclRec
		{
			ComPtr<ID3D11Query> q;
			bool                ours{ false };
			std::uint32_t       tech{ 0 }, flags{ 0 };
			std::string         name;
			int                 dsEn{ -1 }, dsFunc{ 0 }, dsWrite{ 0 }, stEn{ 0 }, stFunc{ 0 }, stRef{ 0 }, cull{ 0 }, bias{ 0 }, blend{ 0 };
			int                 rt[4]{ -9, -9, -9, -9 }, dsv{ -9 }, vpW{ 0 }, vpH{ 0 }, scissor{ 0 }, scW{ 0 }, scH{ 0 };  // G193: render target indices bound after the draw (-1 = not an engine target, -9 = nothing), viewport and scissor
			std::uint64_t       samples{ 0 };
		};
		// G194: view probegeom=<name prefix>: for the first few lit colour calls of a geometry (ours and the engine's) copy the main target before and after the call and count the pixels it changed
		std::string                  g_probeName;
		std::atomic<bool>            g_probeAny{ false };  // view probeany=1 (G256): the geometry probe works for any shader (effects, sky) and for the engine's passes with flag 0x40 (its effect passes run with 0x45), and logs the depth and blend state of the call
		std::atomic<int>             g_probeLeftOurs{ 0 }, g_probeLeftEng{ 0 };
		ComPtr<ID3D11Texture2D>      g_probeA, g_probeB;
		bool ProbeCopy(ID3D11DeviceContext* a_c, ComPtr<ID3D11Texture2D>& a_st)
		{
			auto* rdh = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			auto* src = rdh ? reinterpret_cast<ID3D11Texture2D*>(rdh->renderTargets[1].texture) : nullptr;
			if (!src) {
				return false;
			}
			if (!a_st) {
				D3D11_TEXTURE2D_DESC d{};
				src->GetDesc(&d);
				d.Usage = D3D11_USAGE_STAGING;
				d.BindFlags = 0;
				d.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
				d.MiscFlags = 0;
				d.MipLevels = 1;
				d.ArraySize = 1;
				if (d.SampleDesc.Count != 1 || FAILED(Device()->CreateTexture2D(&d, nullptr, &a_st))) {
					return false;
				}
			}
			a_c->CopyResource(a_st.Get(), src);
			return true;
		}
		void ProbeDiff(ID3D11DeviceContext* a_c, bool a_ours, std::uint32_t a_tech, const std::string& a_name)
		{
			D3D11_MAPPED_SUBRESOURCE ma{}, mb{};
			if (FAILED(a_c->Map(g_probeA.Get(), 0, D3D11_MAP_READ, 0, &ma))) {
				return;
			}
			if (FAILED(a_c->Map(g_probeB.Get(), 0, D3D11_MAP_READ, 0, &mb))) {
				a_c->Unmap(g_probeA.Get(), 0);
				return;
			}
			D3D11_TEXTURE2D_DESC d{};
			g_probeA->GetDesc(&d);
			std::size_t changed = 0;
			for (UINT y = 0; y < d.Height; ++y) {
				const auto* ra = reinterpret_cast<const std::uint16_t*>(static_cast<const std::uint8_t*>(ma.pData) + static_cast<std::size_t>(y) * ma.RowPitch);
				const auto* rb = reinterpret_cast<const std::uint16_t*>(static_cast<const std::uint8_t*>(mb.pData) + static_cast<std::size_t>(y) * mb.RowPitch);
				for (UINT x = 0; x < d.Width; ++x) {
					if (ra[x * 4] != rb[x * 4] || ra[x * 4 + 1] != rb[x * 4 + 1] || ra[x * 4 + 2] != rb[x * 4 + 2]) {
						++changed;
					}
				}
			}
			a_c->Unmap(g_probeB.Get(), 0);
			a_c->Unmap(g_probeA.Get(), 0);
			std::string st;
			{
				ComPtr<ID3D11DepthStencilState> dss;
				UINT                            sref = 0;
				a_c->OMGetDepthStencilState(&dss, &sref);
				if (dss) {
					D3D11_DEPTH_STENCIL_DESC dd{};
					dss->GetDesc(&dd);
					st += std::format(" depth enable {} func {} write {} stencil {}", dd.DepthEnable, static_cast<int>(dd.DepthFunc), static_cast<int>(dd.DepthWriteMask), dd.StencilEnable);
				}
				ComPtr<ID3D11BlendState> bs;
				FLOAT                    bf[4]{};
				UINT                     sm = 0;
				a_c->OMGetBlendState(&bs, bf, &sm);
				if (bs) {
					D3D11_BLEND_DESC bd{};
					bs->GetDesc(&bd);
					st += std::format("; blend enable {} src {} dst {} mask {:#x}", bd.RenderTarget[0].BlendEnable, static_cast<int>(bd.RenderTarget[0].SrcBlend), static_cast<int>(bd.RenderTarget[0].DestBlend), static_cast<unsigned>(bd.RenderTarget[0].RenderTargetWriteMask));
				}
			}
			logger::info("probe: {} call of '{}' technique {:#x} changed {} pixels of the main target;{}", a_ours ? "[OURS]" : "(engine)", a_name, a_tech, changed, st);
		}
		std::atomic<bool>    g_occlOn{ false };
		std::vector<OcclRec> g_occl;
		bool                 g_occlOpen{ false };
		void OcclEnd()
		{
			if (!g_occlOpen) {
				return;
			}
			g_occlOpen = false;
			ComPtr<ID3D11DeviceContext> c;
			Device()->GetImmediateContext(&c);
			auto& r = g_occl.back();
			c->End(r.q.Get());
			ComPtr<ID3D11DepthStencilState> ds;
			UINT                            ref = 0;
			c->OMGetDepthStencilState(&ds, &ref);
			if (ds) {
				D3D11_DEPTH_STENCIL_DESC d{};
				ds->GetDesc(&d);
				r.dsEn = d.DepthEnable ? 1 : 0;
				r.dsFunc = static_cast<int>(d.DepthFunc);
				r.dsWrite = d.DepthWriteMask == D3D11_DEPTH_WRITE_MASK_ALL ? 1 : 0;
				r.stEn = d.StencilEnable ? 1 : 0;
				r.stFunc = static_cast<int>(d.FrontFace.StencilFunc);
				r.stRef = static_cast<int>(ref);
			}
			ComPtr<ID3D11RasterizerState> rs;
			c->RSGetState(&rs);
			if (rs) {
				D3D11_RASTERIZER_DESC d{};
				rs->GetDesc(&d);
				r.cull = static_cast<int>(d.CullMode);
				r.bias = d.DepthBias;
				r.scissor = d.ScissorEnable ? 1 : 0;
			}
			{
				ID3D11RenderTargetView* rtvs[4]{};
				ID3D11DepthStencilView* dv = nullptr;
				c->OMGetRenderTargets(4, rtvs, &dv);
				auto* rdh = RE::BSGraphics::Renderer::GetRendererDataSingleton();
				for (int i = 0; i < 4; ++i) {
					if (rtvs[i]) {
						ComPtr<ID3D11Resource> rr;
						rtvs[i]->GetResource(&rr);
						r.rt[i] = -1;
						if (rdh) {
							for (int j = 0; j < 130; ++j) {
								if (rdh->renderTargets[j].texture == reinterpret_cast<void*>(rr.Get())) {
									r.rt[i] = j;
									break;
								}
							}
						}
						rtvs[i]->Release();
					}
				}
				if (dv) {
					ComPtr<ID3D11Resource> dr;
					dv->GetResource(&dr);
					r.dsv = -1;
					if (rdh) {
						for (int j = 0; j < 12; ++j) {
							if (RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData().depthStencils[j].texture == reinterpret_cast<void*>(dr.Get())) {
								r.dsv = j;
								break;
							}
						}
					}
					dv->Release();
				}
				UINT nvp = 1;
				D3D11_VIEWPORT vp{};
				c->RSGetViewports(&nvp, &vp);
				r.vpW = static_cast<int>(vp.Width);
				r.vpH = static_cast<int>(vp.Height);
				UINT       nsc = 1;
				D3D11_RECT sc{};
				c->RSGetScissorRects(&nsc, &sc);
				r.scW = sc.right - sc.left;
				r.scH = sc.bottom - sc.top;
			}
			ComPtr<ID3D11BlendState> bs;
			FLOAT                    bf[4]{};
			UINT                     sm = 0;
			c->OMGetBlendState(&bs, bf, &sm);
			if (bs) {
				D3D11_BLEND_DESC d{};
				bs->GetDesc(&d);
				r.blend = (d.RenderTarget[0].BlendEnable ? 1 : 0) | (static_cast<int>(d.RenderTarget[0].RenderTargetWriteMask) << 4);
			}
		}
		void OcclReport()
		{
			OcclEnd();
			if (g_occl.empty()) {
				return;
			}
			ComPtr<ID3D11DeviceContext> c;
			Device()->GetImmediateContext(&c);
			c->Flush();
			for (auto& r : g_occl) {
				UINT64 n = 0;
				int    tries = 0;
				while (c->GetData(r.q.Get(), &n, sizeof(n), 0) == S_FALSE && ++tries < 5000) {
					c->Flush();
				}
				r.samples = n;
			}
			// totals per side and pass flags, split by the shadow bits
			struct Tot
			{
				std::uint64_t bitS{ 0 }, noS{ 0 };
				int           bitP{ 0 }, noP{ 0 }, zero{ 0 };
			};
			std::map<std::pair<bool, std::uint32_t>, Tot>                                                      tot;
			std::map<std::tuple<bool, bool, int, int, int, int, int, int, int>, std::pair<int, std::uint64_t>> states;
			std::map<std::pair<bool, std::string>, std::uint64_t>                                              byName;
			for (const auto& r : g_occl) {
				const bool bits = (r.tech & 0x6000) == 0x6000;
				auto&      t = tot[{ r.ours, r.flags }];
				(bits ? t.bitS : t.noS) += r.samples;
				++(bits ? t.bitP : t.noP);
				if (r.samples == 0) ++t.zero;
				auto& st = states[{ r.ours, bits, r.dsEn, r.dsFunc, r.dsWrite, r.stEn, r.stFunc, r.cull, r.blend }];
				++st.first;
				st.second += r.samples;
				if (r.ours || r.flags == 0x41) byName[{ r.ours, r.name }] += r.samples;
			}
			{
				std::map<std::tuple<bool, bool, int, int, int, int, int, int, int, int, int>, int> rth;
				for (const auto& r : g_occl) {
					++rth[{ r.ours, (r.tech & 0x6000) == 0x6000, r.rt[0], r.rt[1], r.rt[2], r.rt[3], r.dsv, r.vpW, r.vpH, r.scissor, r.scW }];
				}
				for (const auto& [k, n] : rth) {
					logger::info("occl: {} {} passes drew with targets [{}, {}, {}, {}] depth {} viewport {}x{} scissor {} (width {}): {}", std::get<0>(k) ? "[OURS]" : "(engine)", std::get<1>(k) ? "bit" : "no-bit", std::get<2>(k), std::get<3>(k), std::get<4>(k), std::get<5>(k), std::get<6>(k), std::get<7>(k), std::get<8>(k), std::get<9>(k), std::get<10>(k), n);
				}
			}
			for (const auto& [k, t] : tot) {
				logger::info("occl: {} lit passes with flags {:#x}: with shadow bits {} passes / {} samples, without {} passes / {} samples, {} passes with zero samples", k.first ? "[OURS]" : "(engine)", k.second, t.bitP, t.bitS, t.noP, t.noS, t.zero);
			}
			for (const auto& [k, v] : states) {
				const auto& [ours, bits, en, fn, wr, sen, sfn, cull, bl] = k;
				logger::info("occl: {} {} passes: depth test {} func {} write {}, stencil {} func {}, cull {}, blend/mask {:#x}: {} passes, {} samples", ours ? "[OURS]" : "(engine)", bits ? "bit" : "no-bit", en, fn, wr, sen, sfn, cull, bl, v.first, v.second);
			}
			for (int side = 1; side >= 0; --side) {
				std::vector<const OcclRec*> top;
				for (const auto& r : g_occl) {
					if (r.ours == (side == 1) && (side == 1 || r.flags == 0x41)) top.push_back(&r);
				}
				std::sort(top.begin(), top.end(), [](const OcclRec* a, const OcclRec* b) { return a->samples > b->samples; });
				for (std::size_t i = 0; i < top.size() && i < 25; ++i) {
					const auto& r = *top[i];
					logger::info("occl: top {} {}: tech {:#x} '{}' {} samples, depth {}/{}/{} stencil {}/{} cull {} bias {} blend {:#x}", side ? "[OURS]" : "(engine)", i, r.tech, r.name, r.samples, r.dsEn, r.dsFunc, r.dsWrite, r.stEn, r.stFunc, r.cull, r.bias, r.blend);
				}
			}
			std::set<std::string> names;
			for (const auto& [k, n] : byName) names.insert(k.second);
			std::vector<std::tuple<std::int64_t, std::string, std::uint64_t, std::uint64_t>> diffs;
			for (const auto& nm : names) {
				const auto          io = byName.find({ true, nm }), ie = byName.find({ false, nm });
				const std::uint64_t o = io != byName.end() ? io->second : 0, e = ie != byName.end() ? ie->second : 0;
				diffs.emplace_back(static_cast<std::int64_t>(o) - static_cast<std::int64_t>(e), nm, o, e);
			}
			std::sort(diffs.begin(), diffs.end(), [](const auto& a, const auto& b) { return std::llabs(std::get<0>(a)) > std::llabs(std::get<0>(b)); });
			for (std::size_t i = 0; i < diffs.size() && i < 40; ++i) {
				logger::info("occl: by geometry {} '{}': ours {} samples, engine {} samples", i, std::get<1>(diffs[i]), std::get<2>(diffs[i]), std::get<3>(diffs[i]));
			}
			g_occl.clear();
		}
		void Hook_Submit(void* a_cam, void* a_acc, std::uint32_t a_flags)
		{
			OcclEnd();
			TraceCall("SubmitAccumulator", a_cam, a_acc, a_flags, _ReturnAddress());
			g_curAcc = a_acc;
			g_origSubmit(a_cam, a_acc, a_flags);
		}
		std::atomic<int> g_dumpStage{ 0 };
		std::string      g_dumpStageLabel;
		std::atomic<float> g_realMaskClear{ -1.0f };  // view realmaskclear=v (G157): clear the sun channel of the shadow mask target 18 to v right before the ENGINE's own colour Start (does the real picture use the mask?)
		std::atomic<int> g_engDump{ 0 };  // view engdump=1: raw dumps of the render targets inside the engine's own frame: before its pre-pass Start (engA), before its colour Start (engB = after the pre-pass), after the colour Start (engC)
		void EngineDump(const char* a_tag)
		{
			ComPtr<ID3D11DeviceContext> c;
			Device()->GetImmediateContext(&c);
			const std::string saved = g_rtDumpTag;
			g_rtDumpTag = g_engTagPrefix + a_tag;
			DumpSsaoInputs(c.Get(), "pre");
			g_rtDumpTag = saved;
		}
		void Hook_StartAccum(void* a_cam, void* a_acc, std::uint32_t a_flags)
		{
			OcclEnd();
			TraceCall("StartAccumulating", a_cam, a_acc, a_flags, _ReturnAddress());
			const bool engineCall = Rva(_ReturnAddress()) < 0x10000000;
			const bool engDump = g_engDump.load() != 0 && engineCall && !g_inVirtual;
			if (engDump && a_flags == 0x21) {
				EngineDump("engA");
			}
			if (engineCall && !g_inVirtual && a_flags == 0x41 && g_realMaskClear.load() >= 0.0f) {
				if (auto* rdm = RE::BSGraphics::Renderer::GetRendererDataSingleton(); rdm && rdm->renderTargets[18].texture) {
					auto*                tm = reinterpret_cast<ID3D11Texture2D*>(rdm->renderTargets[18].texture);
					D3D11_TEXTURE2D_DESC md{};
					tm->GetDesc(&md);
					if (md.Format == DXGI_FORMAT_R8G8B8A8_UNORM) {
						ComPtr<ID3D11DeviceContext> cm;
						Device()->GetImmediateContext(&cm);
						static std::vector<std::uint32_t> s_realFill;
						s_realFill.assign(static_cast<std::size_t>(md.Width) * md.Height, g_realMaskClear.load() >= 0.5f ? 0x000000FFu : 0x00000000u);
						cm->UpdateSubresource(tm, 0, nullptr, s_realFill.data(), md.Width * 4, 0);
					}
				}
			}
			if (engDump && a_flags == 0x41) {
				EngineDump("engB");
			}
			if (g_traceOn.load() && a_flags == 0x41 && !g_inVirtual) {
				g_engineColourAcc = a_acc;
			}
			g_curAcc = a_acc;
			g_origStartAccum(a_cam, a_acc, a_flags);
			if (engDump && a_flags == 0x41) {
				EngineDump("engC");
				g_engDump = 0;
			}
			if (g_dumpStage.load() == 1 && a_flags == 0x41 && !g_inVirtual) {
				g_dumpStage = 0;
				diag::DumpMainNow(g_dumpStageLabel + "_s1");
			}
		}
		void Hook_PostDepth(void* a_cam, void* a_acc, std::uint32_t a_flags)
		{
			OcclEnd();
			TraceCall("FinishAccumulatingPostResolveDepth", a_cam, a_acc, a_flags, _ReturnAddress());
			g_curAcc = a_acc;
			if (g_dumpStage.load() == 2 && !g_inVirtual && (a_flags & 0x40)) {
				g_dumpStage = 0;
				diag::DumpMainNow(g_dumpStageLabel + "_s2");
			}
			g_origPostDepth(a_cam, a_acc, a_flags);
		}
		void Hook_SetRT(void* a_mgr, std::uint32_t a_slot, std::int32_t a_target, std::uint32_t a_mode, bool a_flag)
		{
			if (g_traceOn.load() && g_traceLines < 200) {
				++g_traceLines;
				logger::info("trace: SetRenderTarget{} slot {} target {} mode {} flag {} (current accumulator {:p}) caller rva {:#x}", g_inVirtual ? " [OUR VIRTUAL PASS]" : "", a_slot, a_target, a_mode, a_flag, g_curAcc, Rva(_ReturnAddress()));
			}
			g_origSetRT(a_mgr, a_slot, a_target, a_mode, a_flag);
		}
		// ---- G158: the bytecode of every pixel shader the engine creates (ID3D11Device::CreatePixelShader, vtable slot 15), so the exact variant behind a captured pass can be disassembled offline ----
		std::mutex                                                       g_psMutex;
		std::unordered_map<std::uintptr_t, std::vector<std::uint8_t>>    g_psBytes;
		using CreatePS_t = HRESULT(STDMETHODCALLTYPE*)(ID3D11Device*, const void*, SIZE_T, ID3D11ClassLinkage*, ID3D11PixelShader**);
		CreatePS_t g_origCreatePS{ nullptr };
		HRESULT STDMETHODCALLTYPE Hook_CreatePS(ID3D11Device* a_dev, const void* a_bc, SIZE_T a_len, ID3D11ClassLinkage* a_cl, ID3D11PixelShader** a_out)
		{
			const HRESULT hr = g_origCreatePS(a_dev, a_bc, a_len, a_cl, a_out);
			if (SUCCEEDED(hr) && a_out && *a_out && a_bc && a_len && a_len < (1u << 20)) {
				std::lock_guard lk(g_psMutex);
				g_psBytes[reinterpret_cast<std::uintptr_t>(*a_out)].assign(static_cast<const std::uint8_t*>(a_bc), static_cast<const std::uint8_t*>(a_bc) + a_len);
			}
			return hr;
		}
		void EnsureDeviceHooks()
		{
			static bool done = false;
			if (done) {
				return;
			}
			auto* dev = Device();
			if (!dev) {
				return;
			}
			done = true;
			if (g_origCreatePS) {
				return;  // already hooked at device creation
			}
			void** vt = *reinterpret_cast<void***>(dev);
			DWORD  old = 0;
			if (VirtualProtect(&vt[15], sizeof(void*), PAGE_READWRITE, &old)) {
				g_origCreatePS = reinterpret_cast<CreatePS_t>(vt[15]);
				vt[15] = reinterpret_cast<void*>(&Hook_CreatePS);
				VirtualProtect(&vt[15], sizeof(void*), old, &old);
				logger::info("view: CreatePixelShader hooked (device vtable {:p}); pixel shader bytecode is kept for the capture", static_cast<void*>(vt));
			}
		}
		// ---- G159: hook the device at its creation, so the pixel shaders the engine builds at start-up are seen too (IAT patch of D3D11CreateDeviceAndSwapChain, installed in SKSEPluginLoad) ----
		using CreateDevSC_t = HRESULT(WINAPI*)(IDXGIAdapter*, D3D_DRIVER_TYPE, HMODULE, UINT, const D3D_FEATURE_LEVEL*, UINT, UINT, const DXGI_SWAP_CHAIN_DESC*, IDXGISwapChain**, ID3D11Device**, D3D_FEATURE_LEVEL*, ID3D11DeviceContext**);
		CreateDevSC_t g_origCreateDevSC{ nullptr };
		void          HookDeviceCreatePS(ID3D11Device* a_dev)
		{
			if (!a_dev || g_origCreatePS) {
				return;
			}
			void** vt = *reinterpret_cast<void***>(a_dev);
			DWORD  old = 0;
			if (VirtualProtect(&vt[15], sizeof(void*), PAGE_READWRITE, &old)) {
				g_origCreatePS = reinterpret_cast<CreatePS_t>(vt[15]);
				vt[15] = reinterpret_cast<void*>(&Hook_CreatePS);
				VirtualProtect(&vt[15], sizeof(void*), old, &old);
				logger::info("view: CreatePixelShader hooked at device creation (device vtable {:p})", static_cast<void*>(vt));
			}
		}
		HRESULT WINAPI Hook_CreateDevSC(IDXGIAdapter* a1, D3D_DRIVER_TYPE a2, HMODULE a3, UINT a4, const D3D_FEATURE_LEVEL* a5, UINT a6, UINT a7, const DXGI_SWAP_CHAIN_DESC* a8, IDXGISwapChain** a9, ID3D11Device** a10, D3D_FEATURE_LEVEL* a11, ID3D11DeviceContext** a12)
		{
			const HRESULT hr = g_origCreateDevSC(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12);
			if (SUCCEEDED(hr) && a10 && *a10) {
				HookDeviceCreatePS(*a10);
			}
			return hr;
		}
		// ---- G152: what the lit shader sees, engine vs ours: the constant buffers and textures bound when the first lit pass of one geometry has been drawn (view capture=1 with trace) ----
		struct CapBinding
		{
			std::uintptr_t                          ps{ 0 };
			std::array<std::vector<std::uint8_t>, 14> psCb, vsCb;
			std::array<std::string, 16>             srv;
			std::array<std::string, 16>             smp;  // G209: the pixel shader samplers bound (description)
			std::uint32_t                           tech{ 0 }, flags{ 0 };
		};
		bool                              g_capOn{ false };
		const void*                       g_capGeom{ nullptr };
		// the engine's shader vtables (BSLightingShader / BSSkyShader / BSUtilityShader, which draws the sun shadow mask) through the Address Library, so they hold on every
		// supported runtime (0.2: they were the 1.7.104 RVAs 0x1b47420 / 0x1b48078 / 0x1b4ab58)
		namespace ShaderVt
		{
			inline std::uintptr_t Lighting() { static const std::uintptr_t a = RE::VTABLE_BSLightingShader[0].address(); return a; }
			inline std::uintptr_t Sky() { static const std::uintptr_t a = RE::VTABLE_BSSkyShader[0].address(); return a; }
			inline std::uintptr_t Utility() { static const std::uintptr_t a = RE::VTABLE_BSUtilityShader[0].address(); return a; }
		}
		std::uint64_t                     g_capShaderRva{ 0x1b47420 };  // view capshader=<hex rva of the shader's vtable> (G239): 1b47420 = the lighting shader (default), 1b48078 = the sky shader
		bool                              g_capOursValid{ false }, g_capEngineDone{ false };
		CapBinding                        g_capOurs;
		// G202: view capmask=1: the bindings of the first batch-renderer call that draws into the shadow mask target (18), ours (maskpass=1) and the engine's, compared
		std::atomic<bool>                 g_capMaskOn{ false };
		bool                              g_capMaskOursValid{ false }, g_capMaskEngDone{ false };
		bool                              g_inMaskStage{ false };  // true while our own call of the engine's mask stage runs (G202)
		ID3D11ShaderResourceView* FindSunShadowSrv()
		{
			auto&                     dsd = RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData();
			ID3D11ShaderResourceView* found = nullptr;
			static bool               s_logged = false;
			for (int j = 0; j < 12; ++j) {
				auto* tex = reinterpret_cast<ID3D11Texture2D*>(dsd.depthStencils[j].texture);
				if (!tex) continue;
				D3D11_TEXTURE2D_DESC td{};
				tex->GetDesc(&td);
				if (!s_logged) {
					logger::info("view: depth stencil target {}: texture {:p} {}x{} array {} format {} depthSRV {:p}", j, static_cast<void*>(tex), td.Width, td.Height, td.ArraySize, static_cast<int>(td.Format), static_cast<void*>(dsd.depthStencils[j].depthSRV));
				}
				if (!found && td.ArraySize == 2 && td.Width >= 2048) {
					found = reinterpret_cast<ID3D11ShaderResourceView*>(dsd.depthStencils[j].depthSRV);
				}
			}
			s_logged = true;
			return found;
		}
		// G210: statistics of the sun's cascade shadow map array (min, max, mean, zero and full fractions per slice) at a given moment
		void LogShadowMapStats(ID3D11DeviceContext* a_c, const char* a_tag)
		{
			auto&            dsd = RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData();
			ID3D11Texture2D* tex = nullptr;
			for (int j = 0; j < 12 && !tex; ++j) {
				auto* t = reinterpret_cast<ID3D11Texture2D*>(dsd.depthStencils[j].texture);
				D3D11_TEXTURE2D_DESC td{};
				if (t) t->GetDesc(&td);
				if (t && td.ArraySize == 2 && td.Width >= 2048) tex = t;
			}
			if (!tex) {
				logger::info("view: shadow map array {}: not found", a_tag);
				return;
			}
			D3D11_TEXTURE2D_DESC d{};
			tex->GetDesc(&d);
			D3D11_TEXTURE2D_DESC sd = d;
			sd.Usage = D3D11_USAGE_STAGING;
			sd.BindFlags = 0;
			sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
			sd.MiscFlags = 0;
			sd.ArraySize = 1;
			sd.MipLevels = 1;
			ComPtr<ID3D11Texture2D> st;
			if (FAILED(Device()->CreateTexture2D(&sd, nullptr, &st))) {
				logger::info("view: shadow map array {}: staging texture failed", a_tag);
				return;
			}
			for (UINT slice = 0; slice < 2; ++slice) {
				a_c->CopySubresourceRegion(st.Get(), 0, 0, 0, 0, tex, D3D11CalcSubresource(0, slice, d.MipLevels), nullptr);
				D3D11_MAPPED_SUBRESOURCE m{};
				if (FAILED(a_c->Map(st.Get(), 0, D3D11_MAP_READ, 0, &m))) {
					continue;
				}
				std::uint64_t sum = 0, zeros = 0, fulls = 0;
				std::uint16_t mn = 65535, mx = 0;
				for (UINT y = 0; y < d.Height; y += 4) {
					const auto* row = reinterpret_cast<const std::uint16_t*>(static_cast<const std::uint8_t*>(m.pData) + static_cast<std::size_t>(y) * m.RowPitch);
					for (UINT x = 0; x < d.Width; x += 4) {
						const std::uint16_t v = row[x];
						sum += v;
						if (v == 0) ++zeros;
						if (v == 65535) ++fulls;
						mn = (std::min)(mn, v);
						mx = (std::max)(mx, v);
					}
				}
				a_c->Unmap(st.Get(), 0);
				const double n = static_cast<double>((d.Height / 4) * (d.Width / 4));
				logger::info("view: shadow map array {} slice {}: {}x{} format {}: min {} max {} mean {:.1f}, zero fraction {:.3f}, full fraction {:.3f}", a_tag, slice, d.Width, d.Height, static_cast<int>(d.Format), mn, mx, static_cast<double>(sum) / n, static_cast<double>(zeros) / n, static_cast<double>(fulls) / n);
			}
		}
		// G212: raw dump of the texture bound at pixel shader slot t2 (the depth the mask shader reads) and of the main depth target, at the moment of a mask draw
		void DumpMaskDepths(ID3D11DeviceContext* a_c, const char* a_side)
		{
			ID3D11ShaderResourceView* t2 = nullptr;
			a_c->PSGetShaderResources(2, 1, &t2);
			if (t2) {
				ComPtr<ID3D11Resource> r;
				t2->GetResource(&r);
				ComPtr<ID3D11Texture2D> tx;
				if (r && SUCCEEDED(r.As(&tx))) {
					DumpTexRaw(a_c, tx.Get(), std::string("masktex2_") + a_side);
				}
				t2->Release();
			}
			auto* mainDepth = reinterpret_cast<ID3D11Texture2D*>(RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData().depthStencils[0].texture);
			DumpTexRaw(a_c, mainDepth, std::string("maindepth_") + a_side);
		}
		// G214: the fixed-function state after a draw (blend, depth-stencil, raster, viewport, targets, topology, shaders), logged for the mask draws
		void LogDrawState(ID3D11DeviceContext* a_c, const char* a_label)
		{
			ComPtr<ID3D11BlendState> bs;
			FLOAT                    bf[4]{};
			UINT                     sm = 0;
			a_c->OMGetBlendState(&bs, bf, &sm);
			std::string blend = "none";
			if (bs) {
				D3D11_BLEND_DESC d{};
				bs->GetDesc(&d);
				blend = std::format("rt0 enable {} src {} dest {} op {} / alpha {} {} {} mask {:#x}, alpha-to-coverage {}, independent {}; rt1 mask {:#x}; sample mask {:#x}", d.RenderTarget[0].BlendEnable ? 1 : 0, static_cast<int>(d.RenderTarget[0].SrcBlend), static_cast<int>(d.RenderTarget[0].DestBlend), static_cast<int>(d.RenderTarget[0].BlendOp), static_cast<int>(d.RenderTarget[0].SrcBlendAlpha), static_cast<int>(d.RenderTarget[0].DestBlendAlpha), static_cast<int>(d.RenderTarget[0].BlendOpAlpha), static_cast<unsigned>(d.RenderTarget[0].RenderTargetWriteMask), d.AlphaToCoverageEnable ? 1 : 0, d.IndependentBlendEnable ? 1 : 0, static_cast<unsigned>(d.RenderTarget[1].RenderTargetWriteMask), sm);
			}
			ComPtr<ID3D11DepthStencilState> ds;
			UINT                            ref = 0;
			a_c->OMGetDepthStencilState(&ds, &ref);
			std::string dss = "none";
			if (ds) {
				D3D11_DEPTH_STENCIL_DESC d{};
				ds->GetDesc(&d);
				dss = std::format("depth {} func {} write {}, stencil {} func {} ref {}", d.DepthEnable ? 1 : 0, static_cast<int>(d.DepthFunc), static_cast<int>(d.DepthWriteMask), d.StencilEnable ? 1 : 0, static_cast<int>(d.FrontFace.StencilFunc), ref);
			}
			ComPtr<ID3D11RasterizerState> rs;
			a_c->RSGetState(&rs);
			std::string rss = "none";
			if (rs) {
				D3D11_RASTERIZER_DESC d{};
				rs->GetDesc(&d);
				rss = std::format("fill {} cull {} ccw {} bias {} clip {} scissor {} msaa {}", static_cast<int>(d.FillMode), static_cast<int>(d.CullMode), d.FrontCounterClockwise ? 1 : 0, d.DepthBias, d.DepthClipEnable ? 1 : 0, d.ScissorEnable ? 1 : 0, d.MultisampleEnable ? 1 : 0);
			}
			UINT           nvp = 1;
			D3D11_VIEWPORT vp{};
			a_c->RSGetViewports(&nvp, &vp);
			D3D11_PRIMITIVE_TOPOLOGY topo{};
			a_c->IAGetPrimitiveTopology(&topo);
			ComPtr<ID3D11VertexShader> vs;
			a_c->VSGetShader(&vs, nullptr, nullptr);
			ComPtr<ID3D11InputLayout> il;
			a_c->IAGetInputLayout(&il);
			ID3D11RenderTargetView* rtvs[2]{};
			ID3D11DepthStencilView* dv = nullptr;
			a_c->OMGetRenderTargets(2, rtvs, &dv);
			std::string rts;
			auto*       rdh = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			for (int i = 0; i < 2; ++i) {
				int idx = -9;
				if (rtvs[i]) {
					ComPtr<ID3D11Resource> rr;
					rtvs[i]->GetResource(&rr);
					idx = -1;
					for (int j = 0; j < 130 && rdh; ++j) {
						if (rdh->renderTargets[j].texture == reinterpret_cast<void*>(rr.Get())) {
							idx = j;
							break;
						}
					}
					rtvs[i]->Release();
				}
				rts += std::format(" {}", idx);
			}
			const bool hasDsv = dv != nullptr;
			if (dv) dv->Release();
			logger::info("capture {}: state: blend [{}]; depth-stencil [{}]; raster [{}]; viewport {}x{} at ({}, {}) depth {}..{}; targets{}; depth target bound {}; topology {}; VS {:p}; input layout {:p}", a_label, blend, dss, rss, vp.Width, vp.Height, vp.TopLeftX, vp.TopLeftY, vp.MinDepth, vp.MaxDepth, rts, hasDsv ? 1 : 0, static_cast<int>(topo), static_cast<const void*>(vs.Get()), static_cast<const void*>(il.Get()));
		}
		// G215: view t4const=0|1: in our mask stage bind OUR OWN 4x4 three-slice depth array cleared to 0.0 (everything compares as shadowed) or 1.0 (everything lit) at t4: does the stage's output follow the shadow map at all?
		std::atomic<int>                 g_t4Const{ -1 };
		ComPtr<ID3D11Texture2D>          g_t4ConstTex;
		ComPtr<ID3D11ShaderResourceView> g_t4ConstSrv;
		int                              g_t4ConstVal{ -2 };
		ID3D11ShaderResourceView* T4ConstSrv(int a_val)
		{
			if (g_t4ConstSrv && g_t4ConstVal == a_val) {
				return g_t4ConstSrv.Get();
			}
			g_t4ConstSrv.Reset();
			g_t4ConstTex.Reset();
			D3D11_TEXTURE2D_DESC td{};
			td.Width = 4;
			td.Height = 4;
			td.MipLevels = 1;
			td.ArraySize = 3;
			td.Format = DXGI_FORMAT_R16_TYPELESS;
			td.SampleDesc.Count = 1;
			td.Usage = D3D11_USAGE_DEFAULT;
			td.BindFlags = D3D11_BIND_DEPTH_STENCIL | D3D11_BIND_SHADER_RESOURCE;
			if (FAILED(Device()->CreateTexture2D(&td, nullptr, &g_t4ConstTex))) {
				return nullptr;
			}
			D3D11_DEPTH_STENCIL_VIEW_DESC dv{};
			dv.Format = DXGI_FORMAT_D16_UNORM;
			dv.ViewDimension = D3D11_DSV_DIMENSION_TEXTURE2DARRAY;
			dv.Texture2DArray.ArraySize = 3;
			ComPtr<ID3D11DepthStencilView> dsv;
			if (FAILED(Device()->CreateDepthStencilView(g_t4ConstTex.Get(), &dv, &dsv))) {
				return nullptr;
			}
			D3D11_SHADER_RESOURCE_VIEW_DESC sv{};
			sv.Format = DXGI_FORMAT_R16_UNORM;
			sv.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2DARRAY;
			sv.Texture2DArray.MipLevels = 1;
			sv.Texture2DArray.ArraySize = 3;
			if (FAILED(Device()->CreateShaderResourceView(g_t4ConstTex.Get(), &sv, &g_t4ConstSrv))) {
				return nullptr;
			}
			ComPtr<ID3D11DeviceContext> cc;
			Device()->GetImmediateContext(&cc);
			cc->ClearDepthStencilView(dsv.Get(), D3D11_CLEAR_DEPTH, a_val != 0 ? 1.0f : 0.0f, 0);
			g_t4ConstVal = a_val;
			return g_t4ConstSrv.Get();
		}
		// G218: the shadow mask for a virtual view by our OWN full-screen draw (view maskreplay=1): the engine's own mask pixel shader, samplers and constant buffers (copied on the GPU at the engine's own mask draw
		// of the previous frame), our vertex shader, the main depth as t2, the sun's cascade array as t4. The engine's stage cannot be called for another camera (G199-G214: with every input equal it writes a mask
		// that is black for all geometry), a draw of our own removes the hidden state of that stage. Everything we touch is saved and restored.
		struct MaskReplayData
		{
			ComPtr<ID3D11PixelShader>  ps;
			ComPtr<ID3D11Buffer>       cb[14];
			ComPtr<ID3D11SamplerState> smp[16];
			bool                       have{ false };
		};
		MaskReplayData                  g_mr;
		ComPtr<ID3D11VertexShader>      g_mrVs;
		ComPtr<ID3D11RasterizerState>   g_mrRs;
		ComPtr<ID3D11DepthStencilState> g_mrDs;
		ComPtr<ID3D11BlendState>        g_mrBs;
		std::atomic<int>                g_vpAfter{ 1 };  // view vpafter=0|1 (G224): the view-projection recorded for the mask replay is the one AFTER the oblique patch (the matrix the depth buffer was written with)
		std::atomic<int>                g_maskReplay{ 1 };  // default on (G220): the shadow mask of a virtual view by our own full-screen draw
		float                           g_ourVP[16]{};
		bool                            g_ourVPValid{ false };
		ComPtr<ID3D11Buffer>            g_mrRowsBuf;
		std::vector<std::uint8_t> CapReadBuffer(ID3D11DeviceContext* a_ctx, ID3D11Buffer* a_buf);
		void MaskReplayCapture(ID3D11DeviceContext* a_c)
		{
			ComPtr<ID3D11PixelShader> ps;
			a_c->PSGetShader(&ps, nullptr, nullptr);
			if (!ps) {
				return;
			}
			g_mr.ps = ps;
			ID3D11Buffer* cbs[14]{};
			a_c->PSGetConstantBuffers(0, 14, cbs);
			for (int i : { 0, 2, 11, 12 }) {
				if (!cbs[i]) {
					continue;
				}
				D3D11_BUFFER_DESC d{};
				cbs[i]->GetDesc(&d);
				bool make = !g_mr.cb[i];
				if (g_mr.cb[i]) {
					D3D11_BUFFER_DESC o{};
					g_mr.cb[i]->GetDesc(&o);
					make = o.ByteWidth != d.ByteWidth;
				}
				if (make) {
					D3D11_BUFFER_DESC nd{};
					nd.ByteWidth = d.ByteWidth;
					nd.Usage = D3D11_USAGE_DEFAULT;
					nd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
					g_mr.cb[i].Reset();
					Device()->CreateBuffer(&nd, nullptr, &g_mr.cb[i]);
				}
				if (g_mr.cb[i]) {
					a_c->CopyResource(g_mr.cb[i].Get(), cbs[i]);
				}
				if (i == 2 && !g_biasUnitsValid) {
					const auto bytes = CapReadBuffer(a_c, cbs[i]);
					if (bytes.size() >= 25 * 16) {
						const float* f = reinterpret_cast<const float*>(bytes.data());
						std::uint8_t* node = reinterpret_cast<std::uint8_t*>(SceneStateNode());
						std::uint64_t lightp = 0;
						if (node) ReadSafeT(node + 0x210, lightp);
						if (lightp) {
							auto& sr = reinterpret_cast<RE::BSShadowDirectionalLight*>(lightp)->GetRuntimeData();
							int   ci = 0;
							for (auto& dd : sr.shadowmapDescriptors) {
								if (ci < 2 && dd.camera.get()) {
									const auto& fr = dd.camera.get()->GetRuntimeData2().viewFrustum;
									g_biasUnits[ci] = f[2 * 4 + 1 + ci] * (fr.fFar - fr.fNear);
								}
								++ci;
							}
							g_biasUnitsValid = g_biasUnits[0] > 0.0f && g_biasUnits[1] > 0.0f;
							logger::info("view: cascade depth biases of the engine's own mask draw in world units: cascade 0 {:.2f}, cascade 1 {:.2f}", g_biasUnits[0], g_biasUnits[1]);
						}
					}
				}
				if (g_sunLogOn.load() && (i == 0 || i == 2 || i == 12)) {
					const auto bytes = CapReadBuffer(a_c, cbs[i]);
					const float* f = reinterpret_cast<const float*>(bytes.data());
					auto row = [&](int r) { return std::format("({:.6f} {:.6f} {:.6f} {:.6f})", f[r * 4], f[r * 4 + 1], f[r * 4 + 2], f[r * 4 + 3]); };
					if (i == 0 && bytes.size() >= 4 * 16) {
						logger::info("sunlog: ENGINE mask draw cb0 rows 0-3: {} {} {} {}", row(0), row(1), row(2), row(3));
					} else if (i == 2 && bytes.size() >= 25 * 16) {
						logger::info("sunlog: ENGINE mask draw cb2 rows 0-3: {} {} {} {}", row(0), row(1), row(2), row(3));
						logger::info("sunlog: ENGINE mask draw cb2 rows 16-18 (cascade 0): {} {} {}", row(16), row(17), row(18));
						logger::info("sunlog: ENGINE mask draw cb2 rows 19-21 (cascade 1): {} {} {}", row(19), row(20), row(21));
						logger::info("sunlog: ENGINE mask draw cb2 rows 22-24 (cascade 2): {} {} {}", row(22), row(23), row(24));
					} else if (i == 12 && bytes.size() >= 45 * 16) {
						logger::info("sunlog: ENGINE mask draw cb12 rows 40-42 (camera position adjust etc.): {} {} {}", row(40), row(41), row(42));
					}
				}
				static int s_cbLogged = 0;
				if (i == 12 && s_cbLogged < 2) {
					++s_cbLogged;
					const auto bytes = CapReadBuffer(a_c, cbs[i]);
					if (bytes.size() >= 45 * 16) {
						const float* f = reinterpret_cast<const float*>(bytes.data());
						logger::info("view: mask replay: the engine's cb12 rows 32-35: ({:.5f} {:.5f} {:.5f} {:.5f}) ({:.5f} {:.5f} {:.5f} {:.5f}) ({:.5f} {:.5f} {:.5f} {:.5f}) ({:.5f} {:.5f} {:.5f} {:.5f}); camera position cb12[41] ({:.1f}, {:.1f}, {:.1f})", f[128], f[129], f[130], f[131], f[132], f[133], f[134], f[135], f[136], f[137], f[138], f[139], f[140], f[141], f[142], f[143], f[164], f[165], f[166]);
					}
				}
			}
			for (auto* b : cbs) {
				if (b) b->Release();
			}
			ID3D11SamplerState* sm[16]{};
			a_c->PSGetSamplers(0, 16, sm);
			for (int i = 0; i < 16; ++i) {
				g_mr.smp[i].Attach(sm[i]);
			}
			g_mr.have = true;
		}

		// view maskcover=1 (G236): our own transcription of the engine's sun shadow mask pixel shader (shader vtable rva 0x1b4ab58, disassembly in evidence/s7/dis_mask_ps_ours.txt) with one change: where the
		// pixel's position falls outside the cascade's shadow map (the cascades are fitted to the REAL camera's frustum, so a portal view looking elsewhere is mostly outside them) the shadow term fades to
		// LIT over the width of the filter kernel, instead of whatever the comparison sampler returns at the border (all shadowed, G233). Same constant buffers (b0, b2, b11, b12), samplers (s2, s4), t2, t4.
		static const char kMaskCoverPs[] = R"HLSL(
Texture2D<float>        t2 : register(t2);
Texture2DArray<float>   t4 : register(t4);
SamplerState            s2 : register(s2);
SamplerComparisonState  s4 : register(s4);
cbuffer C0  : register(b0)  { float4 cb0[4]; };
cbuffer C2  : register(b2)  { float4 cb2[25]; };
cbuffer C11 : register(b11) { float4 cb11[1]; };
cbuffer C12 : register(b12) { float4 cb12[45]; };
static const float2 icb[16] = {
	float2(0.493393, 0.394269), float2(0.798547, 0.885922), float2(0.247322, 0.926450), float2(0.051454, 0.140782),
	float2(0.831843, 0.009552), float2(0.428632, 0.017151), float2(0.015656, 0.749779), float2(0.758385, 0.496170),
	float2(0.223487, 0.562151), float2(0.011628, 0.406995), float2(0.241462, 0.304636), float2(0.430311, 0.727226),
	float2(0.981811, 0.278359), float2(0.407056, 0.500534), float2(0.123478, 0.463546), float2(0.809534, 0.682272) };
float Pcf(float2 a_uv, float a_slice, float a_ref, float a_kr)
{
	float acc = 0;
	[unroll] for (int i = 0; i < 8; ++i)
	{
		float2 o0 = icb[2 * i] * 2.0 - 1.0;
		float2 o1 = icb[2 * i + 1] * 2.0 - 1.0;
		acc += t4.SampleCmpLevelZero(s4, float3(a_uv + o0 * a_kr, a_slice), a_ref);
		acc += t4.SampleCmpLevelZero(s4, float3(a_uv + o1 * a_kr, a_slice), a_ref);
	}
	return acc * 0.0625;
}
float Cover(float2 a_uv, float a_kr)
{
	float2 e = min(a_uv, 1.0 - a_uv);
	return saturate(min(e.x, e.y) / max(a_kr, 1e-4));
}
float4 main(float4 pos : SV_Position) : SV_Target
{
	float2 uv = pos.xy * cb0[0].xy + cb0[0].zw;
	float depth = t2.SampleLevel(s2, uv, 0).x;
	float4 outc = float4(1, 1, 1, 1);
	if (cb0[2].z >= depth)
	{
		float4 ndc = float4(uv.x * cb12[44].x * 2.0 - 1.0, (1.0 - uv.y * cb12[44].y) * 2.0 - 1.0, depth, 1.0);
		float4 p = float4(dot(cb12[32], ndc), dot(cb12[33], ndc), dot(cb12[34], ndc), dot(cb12[35], ndc));
		float3 pr = p.xyz / p.w;
		float  t = saturate(dot(pr, pr) / cb2[3].z);
		t = t * t; t = t * t;
		float fade = 1.0 - t * t;
		bool  useC1 = cb0[2].x < depth;
		bool  past1 = cb0[2].y < depth;
		bool  useC2 = (2.5 < cb0[2].w) && past1;
		float4 m0 = useC1 ? cb2[19] : cb2[16];
		float4 m1 = useC1 ? cb2[20] : cb2[17];
		float4 m2 = useC1 ? cb2[21] : cb2[18];
		float  bias = useC1 ? cb2[2].z : cb2[2].y;
		float  slice = useC1 ? 1.0 : 0.0;
		if (useC2) { m0 = cb2[22]; m1 = cb2[23]; m2 = cb2[24]; bias = cb2[2].z; slice = 2.0; }
		float4 pw = float4(pr, 1.0);
		float2 suv = float2(dot(m0, pw), dot(m1, pw));
		float  refz = dot(m2, pw) - bias;
		float  kr = cb0[1].z;
		float  shadow = Pcf(suv, slice, refz, kr);
		shadow = lerp(1.0, shadow, Cover(suv, kr));
		if (slice < 1.0 && cb0[3].y < depth)
		{
			float2 suv2 = float2(dot(cb2[19], pw), dot(cb2[20], pw));
			float  ref2 = dot(cb2[21], pw) - cb2[2].z;
			float  sh2 = Pcf(suv2, 1.0, ref2, kr);
			sh2 = lerp(1.0, sh2, Cover(suv2, kr));
			float w = saturate((depth - cb0[3].y) / (cb0[2].x - cb0[3].y));
			w = w * w * (3.0 - 2.0 * w);
			shadow = lerp(shadow, sh2, w);
		}
		float v = 1.0 + fade * (shadow - 1.0);
		outc = float4(v, v, v, v);
	}
	if (outc.w - cb11[0].x < 0.0) discard;
	return outc;
}
)HLSL";
		std::atomic<bool>          g_maskCover{ true };  // default ON since G238: outside the cascade coverage the sun mask fades to lit
		ComPtr<ID3D11PixelShader>  g_mrPsCover;
		bool MaskReplayRun(ID3D11DeviceContext* a_c, const float* a_rows, const float* a_casc = nullptr)
		{
			if (!g_mr.have) {
				return false;
			}
			if (a_rows && g_mr.cb[12]) {
				if (!g_mrRowsBuf) {
					D3D11_BUFFER_DESC bd{};
					bd.ByteWidth = 64;
					bd.Usage = D3D11_USAGE_DEFAULT;
					Device()->CreateBuffer(&bd, nullptr, &g_mrRowsBuf);
				}
				if (g_mrRowsBuf) {
					a_c->UpdateSubresource(g_mrRowsBuf.Get(), 0, nullptr, a_rows, 0, 0);
					a_c->CopySubresourceRegion(g_mr.cb[12].Get(), 0, 32 * 16, 0, 0, g_mrRowsBuf.Get(), 0, nullptr);
				}
			}
			if (a_casc && g_mr.cb[2]) {
				// G264: OUR cascades (view vshadow=3): rows 16-21 of cb2 (u, v, depth per cascade) and the two depth biases cb2[2].y / .z, written over the engine's real-camera cascades in the replay's copy
				static ComPtr<ID3D11Buffer> s_cascBuf;
				if (!s_cascBuf) {
					D3D11_BUFFER_DESC bd{};
					bd.ByteWidth = 9 * 16;
					bd.Usage = D3D11_USAGE_DEFAULT;
					Device()->CreateBuffer(&bd, nullptr, &s_cascBuf);
				}
				if (s_cascBuf) {
					a_c->UpdateSubresource(s_cascBuf.Get(), 0, nullptr, a_casc, 0, 0);
					a_c->CopySubresourceRegion(g_mr.cb[2].Get(), 0, 16 * 16, 0, 0, s_cascBuf.Get(), 0, nullptr);
					static ComPtr<ID3D11Buffer> s_biasBuf;
					if (!s_biasBuf) {
						D3D11_BUFFER_DESC bb{};
						bb.ByteWidth = 16;
						bb.Usage = D3D11_USAGE_DEFAULT;
						Device()->CreateBuffer(&bb, nullptr, &s_biasBuf);
					}
					if (s_biasBuf) {
						a_c->UpdateSubresource(s_biasBuf.Get(), 0, nullptr, a_casc + 36, 0, 0);
						D3D11_BOX box{ 0, 0, 0, 8, 1, 1 };  // two floats: cb2[2].y (cascade 0) and .z (cascade 1 and 2)
						a_c->CopySubresourceRegion(g_mr.cb[2].Get(), 0, 2 * 16 + 4, 0, 0, s_biasBuf.Get(), 0, &box);
					}
				}
			}
			auto* rdh = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			auto* rtv = rdh ? reinterpret_cast<ID3D11RenderTargetView*>(rdh->renderTargets[18].RTV) : nullptr;
			auto* depthSrv = reinterpret_cast<ID3D11ShaderResourceView*>(RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData().depthStencils[0].depthSRV);
			auto* shadowSrv = FindSunShadowSrv();
			if (!rtv || !depthSrv || !shadowSrv || !g_mr.ps) {
				return false;
			}
			if (!g_mrVs) {
				static const char kVs[] = "float4 main(uint id : SV_VertexID) : SV_Position { float2 p = float2((id << 1) & 2, id & 2); return float4(p * float2(2, -2) + float2(-1, 1), 0, 1); }";
				ComPtr<ID3DBlob> blob, err;
				if (FAILED(D3DCompile(kVs, sizeof(kVs) - 1, "mask_vs", nullptr, nullptr, "main", "vs_5_0", 0, 0, &blob, &err)) || FAILED(Device()->CreateVertexShader(blob->GetBufferPointer(), blob->GetBufferSize(), nullptr, &g_mrVs))) {
					logger::info("view: mask replay: the vertex shader could not be created");
					return false;
				}
				D3D11_RASTERIZER_DESC rd{};
				rd.FillMode = D3D11_FILL_SOLID;
				rd.CullMode = D3D11_CULL_NONE;
				rd.DepthClipEnable = TRUE;
				Device()->CreateRasterizerState(&rd, &g_mrRs);
				D3D11_DEPTH_STENCIL_DESC dd{};
				dd.DepthEnable = FALSE;
				Device()->CreateDepthStencilState(&dd, &g_mrDs);
				D3D11_BLEND_DESC bd{};
				bd.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
				Device()->CreateBlendState(&bd, &g_mrBs);
			}
			ID3D11Texture2D*     mt = reinterpret_cast<ID3D11Texture2D*>(rdh->renderTargets[18].texture);
			D3D11_TEXTURE2D_DESC md{};
			mt->GetDesc(&md);
			// save
			ComPtr<ID3D11VertexShader>      sVs;
			ComPtr<ID3D11PixelShader>       sPs;
			ComPtr<ID3D11InputLayout>       sIl;
			ComPtr<ID3D11RasterizerState>   sRs;
			ComPtr<ID3D11BlendState>        sBs;
			ComPtr<ID3D11DepthStencilState> sDs;
			a_c->VSGetShader(&sVs, nullptr, nullptr);
			a_c->PSGetShader(&sPs, nullptr, nullptr);
			a_c->IAGetInputLayout(&sIl);
			D3D11_PRIMITIVE_TOPOLOGY sTopo{};
			a_c->IAGetPrimitiveTopology(&sTopo);
			ID3D11Buffer* sCb[14]{};
			a_c->PSGetConstantBuffers(0, 14, sCb);
			ID3D11ShaderResourceView* sSrv[16]{};
			a_c->PSGetShaderResources(0, 16, sSrv);
			ID3D11SamplerState* sSm[16]{};
			a_c->PSGetSamplers(0, 16, sSm);
			a_c->RSGetState(&sRs);
			FLOAT bf[4]{};
			UINT  sm = 0, sref = 0;
			a_c->OMGetBlendState(&sBs, bf, &sm);
			a_c->OMGetDepthStencilState(&sDs, &sref);
			UINT           nvp = 1;
			D3D11_VIEWPORT svp{};
			a_c->RSGetViewports(&nvp, &svp);
			ID3D11RenderTargetView* sRt[8]{};
			ID3D11DepthStencilView* sDsv = nullptr;
			a_c->OMGetRenderTargets(8, sRt, &sDsv);
			// set up our draw
			ID3D11RenderTargetView* none[8]{};
			a_c->OMSetRenderTargets(8, none, nullptr);
			ID3D11Buffer*             cbs[14];
			ID3D11ShaderResourceView* srvs[16];
			ID3D11SamplerState*       sms[16];
			for (int i = 0; i < 14; ++i) cbs[i] = sCb[i];
			for (int i = 0; i < 16; ++i) srvs[i] = sSrv[i];
			for (int i = 0; i < 16; ++i) sms[i] = sSm[i];
			for (int i : { 0, 2, 11, 12 }) {
				if (g_mr.cb[i]) cbs[i] = g_mr.cb[i].Get();
			}
			srvs[2] = depthSrv;
			srvs[4] = shadowSrv;
			for (int i : { 2, 4 }) {
				if (g_mr.smp[i]) sms[i] = g_mr.smp[i].Get();
			}
			ID3D11ShaderResourceView* noSrv[16]{};
			a_c->PSSetShaderResources(0, 16, noSrv);
			a_c->IASetInputLayout(nullptr);
			a_c->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
			a_c->VSSetShader(g_mrVs.Get(), nullptr, 0);
			ID3D11PixelShader* usePs = g_mr.ps.Get();
			if (g_maskCover.load()) {
				if (!g_mrPsCover) {
					ComPtr<ID3DBlob> bp, err;
					if (FAILED(D3DCompile(kMaskCoverPs, sizeof(kMaskCoverPs) - 1, "mask_cover_ps", nullptr, nullptr, "main", "ps_5_0", 0, 0, &bp, &err)) || FAILED(Device()->CreatePixelShader(bp->GetBufferPointer(), bp->GetBufferSize(), nullptr, &g_mrPsCover))) {
						logger::info("view: mask cover pixel shader failed: {}", err ? static_cast<const char*>(err->GetBufferPointer()) : "?");
						g_maskCover = false;
					}
				}
				if (g_mrPsCover) usePs = g_mrPsCover.Get();
			}
			a_c->PSSetShader(usePs, nullptr, 0);
			a_c->PSSetConstantBuffers(0, 14, cbs);
			a_c->PSSetSamplers(0, 16, sms);
			a_c->RSSetState(g_mrRs.Get());
			a_c->OMSetDepthStencilState(g_mrDs.Get(), 0);
			const FLOAT zero[4]{};
			a_c->OMSetBlendState(g_mrBs.Get(), zero, 0xFFFFFFFF);
			D3D11_VIEWPORT vp{ 0.0f, 0.0f, static_cast<float>(md.Width), static_cast<float>(md.Height), 0.0f, 1.0f };
			a_c->RSSetViewports(1, &vp);
			a_c->OMSetRenderTargets(1, &rtv, nullptr);
			a_c->PSSetShaderResources(0, 16, srvs);
			a_c->Draw(3, 0);
			// restore
			a_c->OMSetRenderTargets(8, sRt, sDsv);
			a_c->PSSetShaderResources(0, 16, noSrv);
			a_c->PSSetShaderResources(0, 16, sSrv);
			a_c->PSSetSamplers(0, 16, sSm);
			a_c->PSSetConstantBuffers(0, 14, sCb);
			a_c->VSSetShader(sVs.Get(), nullptr, 0);
			a_c->PSSetShader(sPs.Get(), nullptr, 0);
			a_c->IASetInputLayout(sIl.Get());
			a_c->IASetPrimitiveTopology(sTopo);
			a_c->RSSetState(sRs.Get());
			a_c->OMSetBlendState(sBs.Get(), bf, sm);
			a_c->OMSetDepthStencilState(sDs.Get(), sref);
			a_c->RSSetViewports(nvp, &svp);
			for (auto* b : sCb) { if (b) b->Release(); }
			for (auto* v : sSrv) { if (v) v->Release(); }
			for (auto* q : sSm) { if (q) q->Release(); }
			for (auto* r : sRt) { if (r) r->Release(); }
			if (sDsv) sDsv->Release();
			static int s_logged = 0;
			if (s_logged++ < 3) {
				logger::info("view: mask replay: drew the full-screen shadow mask pass (pixel shader {:p})", static_cast<void*>(g_mr.ps.Get()));
			}
			return true;
		}
		std::atomic<int>                  g_t4Shadow{ 0 };         // view t4shadow=1 (G204): in our own mask stage call, bind the sun's shadow map array at t4 (the engine's setup leaves it empty there)
		int                               g_maskSrvLogged{ 0 }, g_maskEngLogged{ 0 };
		int                               g_maskCallsOurs{ 0 }, g_maskCallsEng{ 0 };
		CapBinding                        g_capMaskOurs;
		std::map<UINT, ComPtr<ID3D11Buffer>> g_capStaging;
		std::vector<std::uint8_t> CapReadBuffer(ID3D11DeviceContext* a_ctx, ID3D11Buffer* a_buf)
		{
			std::vector<std::uint8_t> out;
			D3D11_BUFFER_DESC d{};
			a_buf->GetDesc(&d);
			if (d.ByteWidth == 0 || d.ByteWidth > 65536) {
				return out;
			}
			auto& st = g_capStaging[d.ByteWidth];
			if (!st) {
				D3D11_BUFFER_DESC sd{};
				sd.ByteWidth = d.ByteWidth;
				sd.Usage = D3D11_USAGE_STAGING;
				sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
				if (FAILED(Device()->CreateBuffer(&sd, nullptr, &st))) {
					return out;
				}
			}
			a_ctx->CopyResource(st.Get(), a_buf);
			D3D11_MAPPED_SUBRESOURCE m{};
			if (SUCCEEDED(a_ctx->Map(st.Get(), 0, D3D11_MAP_READ, 0, &m))) {
				out.assign(static_cast<const std::uint8_t*>(m.pData), static_cast<const std::uint8_t*>(m.pData) + d.ByteWidth);
				a_ctx->Unmap(st.Get(), 0);
			}
			return out;
		}
		CapBinding CaptureBindings(ID3D11DeviceContext* a_ctx, std::uint32_t a_tech, std::uint32_t a_flags)
		{
			CapBinding c;
			c.tech = a_tech;
			c.flags = a_flags;
			ComPtr<ID3D11PixelShader> ps;
			a_ctx->PSGetShader(&ps, nullptr, nullptr);
			c.ps = reinterpret_cast<std::uintptr_t>(ps.Get());
			ID3D11Buffer* cbs[14]{};
			a_ctx->PSGetConstantBuffers(0, 14, cbs);
			for (int i = 0; i < 14; ++i) {
				if (cbs[i]) {
					c.psCb[i] = CapReadBuffer(a_ctx, cbs[i]);
					cbs[i]->Release();
				}
			}
			ID3D11Buffer* vcbs[14]{};
			a_ctx->VSGetConstantBuffers(0, 14, vcbs);
			for (int i = 0; i < 14; ++i) {
				if (vcbs[i]) {
					c.vsCb[i] = CapReadBuffer(a_ctx, vcbs[i]);
					vcbs[i]->Release();
				}
			}
			ID3D11ShaderResourceView* srvs[16]{};
			a_ctx->PSGetShaderResources(0, 16, srvs);
			auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			for (int i = 0; i < 16; ++i) {
				if (!srvs[i]) {
					continue;
				}
				ComPtr<ID3D11Resource> res;
				srvs[i]->GetResource(&res);
				int idx = -1;
				for (int j = 0; j < 130 && rd; ++j) {
					if (rd->renderTargets[j].texture == reinterpret_cast<void*>(res.Get())) {
						idx = j;
						break;
					}
				}
				D3D11_RESOURCE_DIMENSION dim{};
				res->GetType(&dim);
				std::string desc = std::format("res {:p} dim {}", static_cast<void*>(res.Get()), static_cast<int>(dim));
				ComPtr<ID3D11Texture2D> t2;
				if (SUCCEEDED(res.As(&t2))) {
					D3D11_TEXTURE2D_DESC td{};
					t2->GetDesc(&td);
					desc += std::format(" {}x{} fmt {} mips {} arr {} bind {:#x}", td.Width, td.Height, static_cast<int>(td.Format), td.MipLevels, td.ArraySize, static_cast<unsigned>(td.BindFlags));
				}
				c.srv[i] = idx >= 0 ? std::format("rt{} ({})", idx, desc) : desc;
				srvs[i]->Release();
			}
			ID3D11SamplerState* smps[16]{};
			a_ctx->PSGetSamplers(0, 16, smps);
			for (int i = 0; i < 16; ++i) {
				if (!smps[i]) {
					continue;
				}
				D3D11_SAMPLER_DESC sd{};
				smps[i]->GetDesc(&sd);
				c.smp[i] = std::format("filter {} address {}/{}/{} comparison {} lod {}..{} bias {} aniso {} border ({:.2f}, {:.2f}, {:.2f}, {:.2f})", static_cast<int>(sd.Filter), static_cast<int>(sd.AddressU), static_cast<int>(sd.AddressV), static_cast<int>(sd.AddressW), static_cast<int>(sd.ComparisonFunc), sd.MinLOD, sd.MaxLOD, sd.MipLODBias, sd.MaxAnisotropy, sd.BorderColor[0], sd.BorderColor[1], sd.BorderColor[2], sd.BorderColor[3]);
				smps[i]->Release();
			}
			return c;
		}
		void LogCapture(const char* a_label, const CapBinding& a_c)
		{
			logger::info("capture {}: technique {:#x}, pass flags {:#x}, pixel shader {:#x}", a_label, a_c.tech, a_c.flags, a_c.ps);
			for (int i = 0; i < 14; ++i) {
				if (!a_c.psCb[i].empty() || !a_c.vsCb[i].empty()) {
					logger::info("capture {}: slot {}: PS constant buffer {} bytes, VS constant buffer {} bytes", a_label, i, a_c.psCb[i].size(), a_c.vsCb[i].size());
				}
			}
			for (int i = 0; i < 16; ++i) {
				if (!a_c.srv[i].empty()) {
					logger::info("capture {}: PS texture t{}: {}", a_label, i, a_c.srv[i]);
				}
				if (!a_c.smp[i].empty()) {
					logger::info("capture {}: PS sampler s{}: {}", a_label, i, a_c.smp[i]);
				}
			}
		}
		void DumpCaptureFiles(const char* a_label, const CapBinding& a_c)
		{
			std::filesystem::path dir;
			if (auto ld = logger::log_directory()) dir = *ld;
			if (dir.empty()) {
				return;
			}
			{
				std::lock_guard lk(g_psMutex);
				auto            it = g_psBytes.find(a_c.ps);
				if (it != g_psBytes.end()) {
					std::ofstream f(dir / (std::string("Riftweaver_cap_") + a_label + "_ps.dxbc"), std::ios::binary);
					f.write(reinterpret_cast<const char*>(it->second.data()), static_cast<std::streamsize>(it->second.size()));
					logger::info("capture {}: pixel shader {:#x} bytecode {} bytes written", a_label, a_c.ps, it->second.size());
				} else {
					logger::info("capture {}: pixel shader {:#x} was created before the hook (no bytecode)", a_label, a_c.ps);
				}
			}
			for (int i = 0; i < 14; ++i) {
				if (!a_c.psCb[i].empty()) {
					std::ofstream f(dir / (std::string("Riftweaver_cap_") + a_label + "_pscb" + std::to_string(i) + ".bin"), std::ios::binary);
					f.write(reinterpret_cast<const char*>(a_c.psCb[i].data()), static_cast<std::streamsize>(a_c.psCb[i].size()));
				}
			}
		}
		// ---- G250 view ssaocap=N: what the engine's SSAO step (BSImagespaceShader::RenderSSAO, id 107245) leaves bound after each call, for N frames: the pixel shader (bytecode written as Riftweaver_cap_<label>_ps.dxbc),
		// its constant buffers (Riftweaver_cap_<label>_pscb<slot>.bin) and the textures it reads (logged by render target index). Ours (the virtual pass's calls) and the engine's own (the real frame) are labelled
		// ssao_virt_* and ssao_eng_*; the last call of a frame's step is the apply/composite that tints and hazes the picture (G244-G248). ----
		std::atomic<int>  g_ssaoCapLeft{ 0 };
		std::atomic<bool> g_ssaoCapOn{ false };
		bool              g_ssaoCapVirt{ false };  // set around our own RunVirtualSsao
		int               g_ssaoCapN[2]{ 0, 0 };
		using RenderSsaoCap_t = void (*)(void*, std::int32_t, std::int32_t);
		RenderSsaoCap_t        g_origRenderSsaoCap{ nullptr };
		const hooks::Spec      kRenderSsaoCap{ "BSImagespaceShader::RenderSSAO", 107245, { 0x48, 0x8B, 0xC4, 0x44, 0x89, 0x40, 0x18, 0x55, 0x56, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56 } };
		std::atomic<bool>        g_ssaoCapInCall{ false };
		std::int32_t             g_ssaoCapArgA{ 0 };
		int                      g_ssaoCapDumped[2]{ 0, 0 };
		std::vector<std::string> g_ssaoCapSets;
		std::string              g_ssaoCapTag;  // view ssaotag=<name>: prefix of the capture file names
		std::atomic<bool>        g_ssaoCapDump{ true };  // view ssaodump=0|1: raw dumps of the apply's inputs and output
		void InstallDSHook();
		// every PSSetShaderResources inside one RenderSSAO call: which texture (render target index / depth) goes to which slot; in the apply call (0x71) the first one per source is also dumped raw (Riftweaver_ssaoin_<virt|eng>_t<slot>_<what>.raw)
		void SsaoCapNoteSrvs(ID3D11DeviceContext* a_ctx, UINT a_start, UINT a_num, ID3D11ShaderResourceView* const* a_srvs)
		{
			auto*      rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			const int  side = g_ssaoCapVirt ? 0 : 1;
			const bool dump = g_ssaoCapDump.load() && g_ssaoCapArgA == 0x71 && g_ssaoCapDumped[side] == 0;
			for (UINT i = 0; i < a_num && i < 16; ++i) {
				auto* srv = a_srvs[i];
				if (!srv) {
					g_ssaoCapSets.push_back(std::format("t{}: null", a_start + i));
					continue;
				}
				ComPtr<ID3D11Resource> res;
				srv->GetResource(&res);
				int idx = -1;
				for (int j = 0; j < 130 && rd; ++j) {
					if (rd->renderTargets[j].texture == reinterpret_cast<void*>(res.Get())) {
						idx = j;
						break;
					}
				}
				int didx = -1;
				if (idx < 0) {
					auto& dsd = RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData();
					for (int k = 0; k < 3; ++k) {
						if (reinterpret_cast<void*>(dsd.depthStencils[k].depthSRV) == reinterpret_cast<void*>(srv)) {
							didx = k;
							break;
						}
					}
				}
				std::string desc = idx >= 0 ? std::format("rt{}", idx) : didx >= 0 ? std::format("depth{}", didx) : std::string("other");
				ComPtr<ID3D11Texture2D> t2;
				if (SUCCEEDED(res.As(&t2))) {
					D3D11_TEXTURE2D_DESC td{};
					t2->GetDesc(&td);
					g_ssaoCapSets.push_back(std::format("t{}: {} res {:p} {}x{} fmt {}", a_start + i, desc, static_cast<void*>(res.Get()), td.Width, td.Height, static_cast<int>(td.Format)));
					if (dump && td.SampleDesc.Count == 1 && td.Width * td.Height <= 2100000) {
						DumpTexRaw(a_ctx, t2.Get(), std::format("{}ssaoin_{}_t{}_{}", g_ssaoCapTag, side ? "eng" : "virt", a_start + i, desc));
					}
				} else {
					g_ssaoCapSets.push_back(std::format("t{}: {} (not a 2D texture)", a_start + i, desc));
				}
			}
		}
		void Hook_RenderSsaoCap(void* a_shader, std::int32_t a_a, std::int32_t a_b)
		{
			g_ssaoCapSets.clear();
			g_ssaoCapArgA = a_a;
			g_ssaoCapInCall = g_ssaoCapOn.load();
			g_origRenderSsaoCap(a_shader, a_a, a_b);
			g_ssaoCapInCall = false;
			if (!g_ssaoCapOn.load()) {
				return;
			}
			const int side = g_ssaoCapVirt ? 0 : 1;
			if (a_a == 0x71) {
				if (g_ssaoCapDump.load() && g_ssaoCapDumped[side] == 0) {
					ComPtr<ID3D11DeviceContext> dctx;
					Device()->GetImmediateContext(&dctx);
					DumpRtRaw(dctx.Get(), 1, std::format("{}ssaoout_{}", g_ssaoCapTag, side ? "eng" : "virt"));  // the colour target right after the apply
				}
				g_ssaoCapDumped[side] = 1;
			}
			if (g_ssaoCapN[side] >= 8) {
				return;
			}
			for (const auto& s : g_ssaoCapSets) {
				logger::info("ssaocap: {} call {:#x} sets {}", side ? "ENGINE" : "OURS", a_a, s);
			}
			ComPtr<ID3D11DeviceContext> ctx;
			Device()->GetImmediateContext(&ctx);
			const std::string label = std::format("{}ssao_{}_{}_{:x}_{}", g_ssaoCapTag, side ? "eng" : "virt", g_ssaoCapN[side]++, a_a, a_b);
			logger::info("ssaocap: {} RenderSSAO(shader {:p}, {:#x}, {})", side ? "ENGINE (real frame)" : "OURS (virtual view)", a_shader, a_a, a_b);
			const auto c = CaptureBindings(ctx.Get(), 0, 0);
			LogCapture(label.c_str(), c);
			DumpCaptureFiles(label.c_str(), c);
			ID3D11RenderTargetView* rtv[2]{};
			ctx->OMGetRenderTargets(2, rtv, nullptr);
			auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			for (int i = 0; i < 2; ++i) {
				if (!rtv[i]) {
					continue;
				}
				ComPtr<ID3D11Resource> res;
				rtv[i]->GetResource(&res);
				int idx = -1;
				for (int j = 0; j < 130 && rd; ++j) {
					if (rd->renderTargets[j].texture == reinterpret_cast<void*>(res.Get())) {
						idx = j;
						break;
					}
				}
				logger::info("ssaocap: {}: render target slot {} = rt{}", label, i, idx);
				rtv[i]->Release();
			}
			ComPtr<ID3D11BlendState> bs;
			FLOAT                    bf[4]{};
			UINT                     sm = 0;
			ctx->OMGetBlendState(&bs, bf, &sm);
			if (bs) {
				D3D11_BLEND_DESC bd{};
				bs->GetDesc(&bd);
				logger::info("ssaocap: {}: blend enable {} src {} dst {} op {} write mask {:#x}", label, bd.RenderTarget[0].BlendEnable, static_cast<int>(bd.RenderTarget[0].SrcBlend), static_cast<int>(bd.RenderTarget[0].DestBlend), static_cast<int>(bd.RenderTarget[0].BlendOp), static_cast<unsigned>(bd.RenderTarget[0].RenderTargetWriteMask));
			}
		}
		void SsaoCapEnable(int a_frames)
		{
			static bool tried = false;
			if (!tried) {
				tried = true;
				const bool ok = hooks::Install(kRenderSsaoCap, reinterpret_cast<void*>(&Hook_RenderSsaoCap), reinterpret_cast<void**>(&g_origRenderSsaoCap));
				logger::info("view: RenderSSAO capture hook {}", ok ? "installed" : "FAILED");
				if (!ok) {
					return;
				}
			}
			g_ssaoCapDumped[0] = g_ssaoCapDumped[1] = 0;
			InstallDSHook();
			g_ssaoCapLeft = a_frames;
		}
		// which pixel shader each lit pass really ran (view capture=1 with trace): (ours?, shader object) -> passes; the bytecode of the most used ones is written for offline classification
		std::map<std::pair<bool, std::uintptr_t>, int> g_psUse;
		std::vector<std::string>                        g_seqOursNames, g_seqEngineNames;  // "index:technique:geometry name" per lit pass in draw order (only when the dump tag is p1)
		std::string                                     g_orderOurs, g_orderEngine;  // one char per lit pass in draw order: 'B' with the shadow bits, 'n' without, 'L' = Land etc. is not distinguished
		std::map<std::pair<bool, int>, int>             g_sunHist;  // (ours?, bucket of the strongest sun colour component cb2[1].xyz: 0 = zero, 1 < 0.25, 2 < 1, 3 < 2, 4 >= 2) -> lit passes
		std::map<std::tuple<bool, bool, int>, int>      g_t14Hist;  // G186: (ours?, shadow bits?, what is bound at pixel-shader slot t14 after the pass: 0 = nothing, 1 = the shadow mask target 18, 2 = another texture) -> lit passes
		std::map<std::pair<bool, int>, int>             g_nlHist;   // (ours?, bucket of the shader's mask-select cb2[29].x light count 0..7) -> lit passes
		void DumpPsUse()
		{
			std::filesystem::path dir;
			if (auto ld = logger::log_directory()) dir = *ld;
			std::vector<std::tuple<int, bool, std::uintptr_t>> v;
			for (const auto& [k, n] : g_psUse) {
				v.emplace_back(n, k.first, k.second);
			}
			std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return std::get<0>(a) > std::get<0>(b); });
			int ours = 0, eng = 0;
			for (const auto& [n, o, p] : v) {
				(o ? ours : eng) += n;
			}
			logger::info("capture: lit passes by pixel shader: ours {} passes in {} shader(s), engine {} passes in {} shader(s)", ours, std::count_if(v.begin(), v.end(), [](const auto& t) { return std::get<1>(t); }), eng, std::count_if(v.begin(), v.end(), [](const auto& t) { return !std::get<1>(t); }));
			int shown[2]{ 0, 0 };
			for (const auto& [n, o, p] : v) {
				if (shown[o ? 1 : 0] >= 60) {
					continue;
				}
				++shown[o ? 1 : 0];
				logger::info("capture: pixel shader use {} {:#x}: {} pass(es)", o ? "[OURS]" : "(engine)", p, n);
				if (!dir.empty()) {
					std::lock_guard lk(g_psMutex);
					if (auto it = g_psBytes.find(p); it != g_psBytes.end()) {
						std::ofstream f(dir / ("Riftweaver_psuse_" + g_rtDumpTag + "_" + std::string(o ? "ours_" : "engine_") + std::format("{:x}", p) + "_n" + std::to_string(n) + ".dxbc"), std::ios::binary);
						f.write(reinterpret_cast<const char*>(it->second.data()), static_cast<std::streamsize>(it->second.size()));
					}
				}
			}
			for (const auto& [k, n] : g_sunHist) {
				static const char* kB[] = { "sun colour 0", "sun colour < 0.25", "sun colour < 1", "sun colour < 2", "sun colour >= 2" };
				logger::info("capture: {} lit passes with {}: {}", k.first ? "[OURS]" : "(engine)", kB[k.second], n);
			}
			for (const auto& [k, n] : g_nlHist) {
				logger::info("capture: {} lit passes with {} point light(s) in cb2[29].x: {}", k.first ? "[OURS]" : "(engine)", k.second, n);
			}
			for (int side = 0; side < 2; ++side) {
				const std::string& o = side ? g_orderOurs : g_orderEngine;
				std::string        rle;
				for (std::size_t i = 0; i < o.size();) {
					std::size_t j = i;
					while (j < o.size() && o[j] == o[i]) ++j;
					rle += std::format("{}{} ", o[i], j - i);
					i = j;
				}
				logger::info("capture: lit pass order {} ({} passes; B = shadow bits, n = none): {}", side ? "[OURS]" : "(engine)", o.size(), rle.substr(0, 1800));
			}
			for (int side = 0; side < 2; ++side) {
				auto& v = side ? g_seqOursNames : g_seqEngineNames;
				for (std::size_t i = 0; i < v.size() && i < 1320; i += 6) {
					std::string line;
					for (std::size_t j = i; j < i + 6 && j < v.size(); ++j) line += v[j] + " | ";
					logger::info("capture: seq {} {}", side ? "[OURS]" : "(engine)", line);
				}
				v.clear();
			}
			g_orderOurs.clear();
			g_orderEngine.clear();
			g_sunHist.clear();
			for (const auto& [k, n] : g_t14Hist) {
				static const char* kK[] = { "nothing bound at t14", "the shadow mask target 18 bound at t14", "another texture bound at t14" };
				logger::info("capture: {} {} lit passes with {}: {}", std::get<0>(k) ? "[OURS]" : "(engine)", std::get<1>(k) ? "bit" : "no-bit", kK[std::get<2>(k)], n);
			}
			g_t14Hist.clear();
			g_nlHist.clear();
			g_psUse.clear();
		}
		void CompareBuffers(const char* a_what, int a_slot, const std::vector<std::uint8_t>& a_o, const std::vector<std::uint8_t>& a_e)
		{
			if (a_o.size() != a_e.size()) {
				logger::info("capture compare: {} slot {}: sizes differ (ours {} bytes, engine {} bytes)", a_what, a_slot, a_o.size(), a_e.size());
				return;
			}
			int diffs = 0, shown = 0;
			for (std::size_t w = 0; w + 4 <= a_o.size(); w += 4) {
				std::uint32_t o, e;
				std::memcpy(&o, &a_o[w], 4);
				std::memcpy(&e, &a_e[w], 4);
				if (o != e) {
					++diffs;
					if (shown < (a_what[0] == 'P' ? 120 : 16)) {
						++shown;
						float fo, fe;
						std::memcpy(&fo, &o, 4);
						std::memcpy(&fe, &e, 4);
						logger::info("capture compare: {} slot {} word {} (+{:#x}): ours {:#010x} ({:.5g}), engine {:#010x} ({:.5g})", a_what, a_slot, w / 4, w, o, fo, e, fe);
					}
				}
			}
			logger::info("capture compare: {} slot {}: {} bytes, {} differing word(s)", a_what, a_slot, a_o.size(), diffs);
		}
		void CompareCaptures(const CapBinding& a_o, const CapBinding& a_e)
		{
			logger::info("capture compare: pixel shader ours {:#x} engine {:#x}{}; technique ours {:#x} engine {:#x}; pass flags ours {:#x} engine {:#x}", a_o.ps, a_e.ps, a_o.ps == a_e.ps ? " (the same)" : " (DIFFERENT)", a_o.tech, a_e.tech, a_o.flags, a_e.flags);
			for (int i = 0; i < 14; ++i) {
				if (!a_o.psCb[i].empty() || !a_e.psCb[i].empty()) CompareBuffers("PS cb", i, a_o.psCb[i], a_e.psCb[i]);
				if (!a_o.vsCb[i].empty() || !a_e.vsCb[i].empty()) CompareBuffers("VS cb", i, a_o.vsCb[i], a_e.vsCb[i]);
			}
			for (int i = 0; i < 16; ++i) {
				if (a_o.srv[i] != a_e.srv[i]) {
					logger::info("capture compare: PS texture t{} differs: ours [{}] engine [{}]", i, a_o.srv[i], a_e.srv[i]);
				}
			}
		}
		std::atomic<int> g_skipSky{ 0 };  // view skipsky=<bits> (G241): in the virtual passes skip the sky shader's draws: 1 = atmosphere dome (technique 0x5c00005e), 2 = glare (0x5c000061), 4 = cloud layers (0x5c000062), 8 = every other sky technique
		void Hook_SetupPass(void* a_pass, std::uint32_t a_tech, bool a_alpha, std::uint32_t a_flags)
		{
			OcclEnd();
			if (g_skipSky.load() != 0 && g_inVirtual) {
				std::uintptr_t svsk = 0;
				const std::uint8_t* shsk = nullptr;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass), shsk);
				if (shsk) ReadSafeT(shsk, svsk);
				if (svsk && svsk == ShaderVt::Sky()) {
					const int bit = a_tech == 0x5c00005e ? 1 : a_tech == 0x5c000061 ? 2 : a_tech == 0x5c000062 ? 4 : 8;
					if (g_skipSky.load() & bit) {
						return;
					}
				}
			}
			if (g_skipBitsReal.load() != 0 && !g_inVirtual) {
				std::uintptr_t svtr = 0;
				const std::uint8_t* shr = nullptr;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass), shr);
				if (shr) ReadSafeT(shr, svtr);
				std::uint32_t amoder = 0xFFFF;
				if (g_curAcc) ReadSafeT(reinterpret_cast<const std::uint8_t*>(g_curAcc) + 0x150, amoder);
				if (svtr && svtr == ShaderVt::Lighting() && amoder == 0 && a_flags == 0x41) {
					const bool hasBitsR = (a_tech & 0x6000) == 0x6000;
					if ((g_skipBitsReal.load() == 1 && hasBitsR) || (g_skipBitsReal.load() == 2 && !hasBitsR)) {
						return;
					}
				}
			}
			if ((g_skipBits.load() != 0 || g_seqHi.load() > g_seqLo.load()) && g_inVirtual) {
				std::uintptr_t svts = 0;
				const std::uint8_t* shs = nullptr;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass), shs);
				if (shs) ReadSafeT(shs, svts);
				std::uint32_t amodes = 0xFFFF;
				if (g_curAcc) ReadSafeT(reinterpret_cast<const std::uint8_t*>(g_curAcc) + 0x150, amodes);
				if (svts && svts == ShaderVt::Lighting() && amodes == 0) {
					const bool hasBits = (a_tech & 0x6000) == 0x6000;
					if ((g_skipBits.load() == 1 && hasBits) || (g_skipBits.load() == 2 && !hasBits)) {
						return;
					}
					const int seqIdx = g_litSeq.fetch_add(1);
					if (g_seqHi.load() > g_seqLo.load() && (seqIdx < g_seqLo.load() || seqIdx >= g_seqHi.load())) {
						return;
					}
				}
			}
			const bool tracing = g_traceOn.load(), fixing = g_inVirtual && g_params.alphaFix;
			if (tracing || fixing) {
				const std::uint8_t* geom = nullptr;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass) + 0x10, geom);
				const std::uint8_t* ap = nullptr;
				if (geom) {
					ReadSafeT(geom + 0x120, ap);
				}
				std::uint16_t pf = 0;
				if (ap) {
					ReadSafeT(ap + 0x30, pf);
				}
				const bool prop = ap && (pf & 0x200);
				if (tracing) {
					std::uintptr_t svt = 0;
					{
						const std::uint8_t* sh = nullptr;
						ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass), sh);
						if (sh) ReadSafeT(sh, svt);
					}
					auto& gr = g_groups[{ g_curAcc, g_inVirtual, svt ? svt - REL::Module::get().base() : 0, a_tech, a_flags }];
					if (gr.calls++ == 0) {
						const char* nm0 = nullptr;
						if (geom) ReadSafeT(geom + 0x10, nm0);
						gr.name = nm0 ? std::string(nm0, strnlen(nm0, 48)) : std::string("?");
						std::uintptr_t gv = 0, pv2 = 0;
						if (geom) ReadSafeT(geom, gv);
						const std::uint8_t* spr = nullptr;
						if (geom) ReadSafeT(geom + 0x128, spr);
						if (spr) ReadSafeT(spr, pv2);
						gr.geomVt = gv ? gv - REL::Module::get().base() : 0;
						gr.propVt = pv2 ? pv2 - REL::Module::get().base() : 0;
					}
					if (a_alpha) ++gr.argTrue;
					if (prop) ++gr.prop;

					{
						std::uint32_t amode = 0xFFFF;
						if (g_curAcc) ReadSafeT(reinterpret_cast<const std::uint8_t*>(g_curAcc) + 0x150, amode);
						if (g_inVirtual || amode == 0) {
							auto& gt = g_geomTech[geom];
							(g_inVirtual ? gt.ours : gt.engine).insert(a_tech);
							if (gt.name.empty()) {
								const char* gn = nullptr;
								if (geom) ReadSafeT(geom + 0x10, gn);
								gt.name = gn ? std::string(gn, strnlen(gn, 40)) : std::string("?");
								std::uintptr_t sv = 0;
								const std::uint8_t* sh2 = nullptr;
								ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass), sh2);
								if (sh2) ReadSafeT(sh2, sv);
								gt.shaderVt = sv ? sv - REL::Module::get().base() : 0;
							}
						}
					}
					auto& st = g_passStats[{ g_curAcc, g_inVirtual }];
					++st.calls;
					if (a_alpha) ++st.argTrue;
					if (prop) {
						++st.propTest;
						if (!a_alpha) {
							++st.propTestArgFalse;
							if (g_inVirtual) {
								++g_techNoArg[a_tech];
								if (g_noArgLogged < 24) {
									++g_noArgLogged;
									const char* nm = nullptr;
									ReadSafeT(geom + 0x10, nm);
									const std::uint8_t* shader = nullptr;
									ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass), shader);
									std::uintptr_t vt = 0;
									if (shader) ReadSafeT(shader, vt);
									logger::info("trace: [ours] draw of '{}' with the alpha test enabled in its alpha property (flags {:#x}) and the alphaTest argument FALSE: technique {:#x}, pass flags {:#x}, shader vtable rva {:#x}", nm ? nm : "?", pf, a_tech, a_flags, vt ? vt - REL::Module::get().base() : 0);
								}
							}
						}
					}
				}
				if (fixing && prop && !a_alpha) {
					a_alpha = true;
					g_forcedTotal.fetch_add(1);
					if (tracing) {
						++g_passStats[{ g_curAcc, g_inVirtual }].forced;
					}
				}
			}
			bool occlBegun = false;
			if (g_occlOn.load() && tracing && g_capOn) {
				std::uintptr_t svto = 0;
				const std::uint8_t* sho = nullptr;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass), sho);
				if (sho) ReadSafeT(sho, svto);
				std::uint32_t amodeo = 0xFFFF;
				if (g_curAcc) ReadSafeT(reinterpret_cast<const std::uint8_t*>(g_curAcc) + 0x150, amodeo);
				if (svto && svto == ShaderVt::Lighting() && amodeo == 0) {
					OcclRec orc;
					orc.ours = g_inVirtual;
					orc.tech = a_tech;
					orc.flags = a_flags;
					const std::uint8_t* gqo = nullptr;
					ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass) + 0x10, gqo);
					const char* nqo = nullptr;
					if (gqo) ReadSafeT(gqo + 0x10, nqo);
					orc.name = nqo ? std::string(nqo, strnlen(nqo, 40)) : std::string("?");
					D3D11_QUERY_DESC oqd{ D3D11_QUERY_OCCLUSION, 0 };
					ComPtr<ID3D11DeviceContext> cqo;
					Device()->GetImmediateContext(&cqo);
					if (SUCCEEDED(Device()->CreateQuery(&oqd, &orc.q))) {
						cqo->Begin(orc.q.Get());
						g_occl.push_back(std::move(orc));
						g_occlOpen = true;
						occlBegun = true;
					}
				}
			}
			bool        probing = false;
			std::string probeName;
			if (!g_probeName.empty() && (g_inVirtual ? g_probeLeftOurs.load() : g_probeLeftEng.load()) > 0) {
				std::uintptr_t svtp = 0;
				const std::uint8_t* shp = nullptr;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass), shp);
				if (shp) ReadSafeT(shp, svtp);
				std::uint32_t amodep = 0xFFFF;
				if (g_curAcc) ReadSafeT(reinterpret_cast<const std::uint8_t*>(g_curAcc) + 0x150, amodep);
				if (svtp && (g_probeAny.load() || svtp == ShaderVt::Lighting()) && amodep == 0 && (g_inVirtual || (g_probeAny.load() ? (a_flags & 0x40) != 0 : a_flags == 0x41))) {
					const std::uint8_t* gqp = nullptr;
					ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass) + 0x10, gqp);
					const char* nqp = nullptr;
					if (gqp) ReadSafeT(gqp + 0x10, nqp);
					if (nqp) {
						probeName = std::string(nqp, strnlen(nqp, 40));
						if (g_probeAny.load() ? probeName.find(g_probeName) != std::string::npos : probeName.rfind(g_probeName, 0) == 0) {
							ComPtr<ID3D11DeviceContext> cpp;
							Device()->GetImmediateContext(&cpp);
							probing = ProbeCopy(cpp.Get(), g_probeA);
						}
					}
				}
			}
			g_origSetupPass(a_pass, a_tech, a_alpha, a_flags);
			if (probing) {
				ComPtr<ID3D11DeviceContext> cpq;
				Device()->GetImmediateContext(&cpq);
				if (ProbeCopy(cpq.Get(), g_probeB)) {
					(g_inVirtual ? g_probeLeftOurs : g_probeLeftEng).fetch_sub(1);
					ProbeDiff(cpq.Get(), g_inVirtual, a_tech, probeName);
				}
			}
			if (occlBegun) {
				OcclEnd();
			}
			bool maskShaderPass = false;
			if (g_maskReplay.load() && !g_inVirtual && !g_inMaskStage) {
				std::uintptr_t svtm = 0;
				const std::uint8_t* shm2 = nullptr;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass), shm2);
				if (shm2) ReadSafeT(shm2, svtm);
				maskShaderPass = svtm && svtm == ShaderVt::Utility();
			}
			if (maskShaderPass) {
				ComPtr<ID3D11DeviceContext> cmr;
				Device()->GetImmediateContext(&cmr);
				ID3D11RenderTargetView* rrv = nullptr;
				cmr->OMGetRenderTargets(1, &rrv, nullptr);
				bool isM = false;
				if (rrv) {
					ComPtr<ID3D11Resource> rres;
					rrv->GetResource(&rres);
					auto* rdq = RE::BSGraphics::Renderer::GetRendererDataSingleton();
					isM = rdq && rdq->renderTargets[18].texture == reinterpret_cast<void*>(rres.Get());
					rrv->Release();
				}
				if (isM) {
					MaskReplayCapture(cmr.Get());
				}
			}
			if (g_capMaskOn.load()) {
				ComPtr<ID3D11DeviceContext> cmk;
				Device()->GetImmediateContext(&cmk);
				ID3D11RenderTargetView* rk = nullptr;
				cmk->OMGetRenderTargets(1, &rk, nullptr);
				bool isMask = false;
				if (rk) {
					ComPtr<ID3D11Resource> rr;
					rk->GetResource(&rr);
					auto* rdk = RE::BSGraphics::Renderer::GetRendererDataSingleton();
					isMask = rdk && rdk->renderTargets[18].texture == reinterpret_cast<void*>(rr.Get());
					rk->Release();
				}
				if (isMask) {
					const bool oursSide = g_inVirtual || g_inMaskStage;
					++(oursSide ? g_maskCallsOurs : g_maskCallsEng);
					static int s_seqLogged = 0;
					if (s_seqLogged < 24) {
						++s_seqLogged;
						ComPtr<ID3D11PixelShader> psq;
						cmk->PSGetShader(&psq, nullptr, nullptr);
						const std::uint8_t* gqm = nullptr;
						ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass) + 0x10, gqm);
						const char* nqm = nullptr;
						if (gqm) ReadSafeT(gqm + 0x10, nqm);
						std::uintptr_t svm = 0;
						const std::uint8_t* shm = nullptr;
						ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass), shm);
						if (shm) ReadSafeT(shm, svm);
						logger::info("capture: mask-target draw #{} ({}): shader vtable rva {:#x}, technique {:#x}, flags {:#x}, pixel shader {:p}, geometry '{}'", oursSide ? g_maskCallsOurs : g_maskCallsEng, oursSide ? "ours" : "engine", svm ? svm - REL::Module::get().base() : 0, a_tech, a_flags, static_cast<const void*>(psq.Get()), nqm ? std::string(nqm, strnlen(nqm, 32)) : std::string("?"));
					}
					if (oursSide && !g_capMaskOursValid) {
						g_capMaskOurs = CaptureBindings(cmk.Get(), a_tech, a_flags);
						LogShadowMapStats(cmk.Get(), "during our first mask draw");
						DumpMaskDepths(cmk.Get(), "ours");
						LogDrawState(cmk.Get(), "MASK-OURS");
						g_capMaskOursValid = true;
						LogCapture("MASK-OURS", g_capMaskOurs);
						DumpCaptureFiles("mask_ours", g_capMaskOurs);
					} else if (!oursSide && g_capMaskOursValid && !g_capMaskEngDone) {
						auto eng = CaptureBindings(cmk.Get(), a_tech, a_flags);
						LogShadowMapStats(cmk.Get(), "during the engine's first mask draw");
						DumpMaskDepths(cmk.Get(), "engine");
						LogDrawState(cmk.Get(), "MASK-ENGINE");
						LogCapture("MASK-ENGINE", eng);
						DumpCaptureFiles("mask_engine", eng);
						CompareCaptures(g_capMaskOurs, eng);
						g_capMaskEngDone = true;
						logger::info("capture: mask target calls so far: ours {}, engine {}", g_maskCallsOurs, g_maskCallsEng);
					}
				}
			}
			if (tracing) {
				std::uintptr_t svt2 = 0;
				const std::uint8_t* sh = nullptr;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass), sh);
				if (sh) ReadSafeT(sh, svt2);
				++g_passRt[{ g_inVirtual, svt2 ? svt2 - REL::Module::get().base() : 0, a_tech, a_flags, g_lastSig }];
			}
			if (tracing && g_capOn) {
				std::uintptr_t svtu = 0;
				const std::uint8_t* shu = nullptr;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass), shu);
				if (shu) ReadSafeT(shu, svtu);
				std::uint32_t amodeu = 0xFFFF;
				if (g_curAcc) ReadSafeT(reinterpret_cast<const std::uint8_t*>(g_curAcc) + 0x150, amodeu);
				if (svtu && svtu == ShaderVt::Lighting() && amodeu == 0) {
					ComPtr<ID3D11DeviceContext> cu;
					Device()->GetImmediateContext(&cu);
					ComPtr<ID3D11PixelShader> pu;
					cu->PSGetShader(&pu, nullptr, nullptr);
					++g_psUse[{ g_inVirtual, reinterpret_cast<std::uintptr_t>(pu.Get()) }];
					(g_inVirtual ? g_orderOurs : g_orderEngine).push_back((a_tech & 0x6000) == 0x6000 ? 'B' : 'n');
					{
						ID3D11ShaderResourceView* t14 = nullptr;
						cu->PSGetShaderResources(14, 1, &t14);
						int kind = 0;
						if (t14) {
							ComPtr<ID3D11Resource> tr;
							t14->GetResource(&tr);
							auto* rdh = RE::BSGraphics::Renderer::GetRendererDataSingleton();
							kind = (rdh && rdh->renderTargets[RE::RENDER_TARGETS::kSHADOW_MASK].texture == reinterpret_cast<void*>(tr.Get())) ? 1 : 2;
							t14->Release();
						}
						++g_t14Hist[{ g_inVirtual, (a_tech & 0x6000) == 0x6000, kind }];
					}
					if (g_rtDumpTag == "p1") {
						const std::uint8_t* gq = nullptr;
						ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass) + 0x10, gq);
						const char* nq = nullptr;
						if (gq) ReadSafeT(gq + 0x10, nq);
						auto& vq = g_inVirtual ? g_seqOursNames : g_seqEngineNames;
						vq.push_back(std::format("{}:{:x}:{}", vq.size(), a_tech, nq ? std::string(nq, strnlen(nq, 28)) : std::string("?")));
					}
					ID3D11Buffer* cbp[3]{};
					cu->PSGetConstantBuffers(0, 3, cbp);
					if (cbp[2]) {
						const auto bytes = CapReadBuffer(cu.Get(), cbp[2]);
						if (bytes.size() >= 480) {
							float f[8];
							std::memcpy(f, bytes.data(), 32);
							float cnt = 0;
							std::memcpy(&cnt, bytes.data() + 29 * 16, 4);
							const float m = (std::max)({ f[4], f[5], f[6] });
							++g_sunHist[{ g_inVirtual, m <= 0.0f ? 0 : m < 0.25f ? 1 : m < 1.0f ? 2 : m < 2.0f ? 3 : 4 }];
							++g_nlHist[{ g_inVirtual, (std::min)(7, (std::max)(0, static_cast<int>(cnt))) }];
						}
					}
					for (auto* b : cbp) {
						if (b) b->Release();
					}
				}
			}
			if (tracing && g_capOn && !g_capEngineDone) {
				std::uintptr_t svtc = 0;
				const std::uint8_t* shc = nullptr;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass), shc);
				if (shc) ReadSafeT(shc, svtc);
				std::uint32_t amodec = 0xFFFF;
				if (g_curAcc) ReadSafeT(reinterpret_cast<const std::uint8_t*>(g_curAcc) + 0x150, amodec);
				const std::uint8_t* gm = nullptr;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_pass) + 0x10, gm);
				if (svtc && (g_capGeom ? gm == static_cast<const void*>(g_capGeom) : svtc - REL::Module::get().base() == g_capShaderRva) && amodec == 0 && gm) {
					ComPtr<ID3D11DeviceContext> cx;
					Device()->GetImmediateContext(&cx);
					if (g_inVirtual && !g_capGeom) {
						g_capGeom = gm;
						g_capOurs = CaptureBindings(cx.Get(), a_tech, a_flags);
						g_capOursValid = true;
						LogCapture("OURS", g_capOurs);
						DumpCaptureFiles("ours", g_capOurs);
					} else if (!g_inVirtual && g_capOursValid && gm == g_capGeom) {
						auto eng = CaptureBindings(cx.Get(), a_tech, a_flags);
						LogCapture("ENGINE", eng);
						DumpCaptureFiles("engine", eng);
						CompareCaptures(g_capOurs, eng);
						g_capEngineDone = true;
					}
				}
			}
		}
		// the light selection of a geometry (id 107784): which lights, with which shadow-mask channel and which flag, go into a pass
		using LightSel_t = std::uint32_t (*)(void*, void*, std::uint32_t, void*, void*, void*, void*, void*, void*, void*);
		LightSel_t g_origLightSel{ nullptr };
		int        g_lightLogged{ 0 };
		// (ours cull?, lights in the geometry's list, out byte before, out byte after, returned lights, out dword after) -> calls
		std::map<std::tuple<bool, std::uint32_t, int, int, std::uint32_t, std::uint32_t>, int> g_lsHist;
		bool g_lsSeenOurs{ false }, g_lsSeenEngine{ false };
		// G198: the light-mask forcing (view lightmask=1|2) - set the shadow-light bits in a geometry's light mask while our own cull selects its lights. It used to sit behind the trace gate
		// below, so it only acted in TRACED frames (every earlier un-traced 'forced' measurement had no forcing at all).
		void ForceLightMask(void* a1, void* a5)
		{
			const auto*         node = reinterpret_cast<const std::uint8_t*>(a5);
			const std::uint8_t* arr = nullptr;
			std::uint32_t       cnt = 0;
			std::uint64_t       sunShadow = 0;
			ReadSafeT(node + 0x230, arr);
			ReadSafeT(node + 0x240, cnt);
			ReadSafeT(node + 0x210, sunShadow);
			std::uint32_t bits = 0;
			for (std::uint32_t i2 = 0; arr && i2 < cnt && i2 < 32; ++i2) {
				std::uint64_t l = 0;
				ReadSafeT(arr + i2 * 8, l);
				if (l && (g_params.lightMask == 2 || l == sunShadow)) bits |= 1u << i2;
			}
			*reinterpret_cast<std::uint32_t*>(reinterpret_cast<std::uint8_t*>(a1) + 0x1C) |= bits;
		}
		std::uint32_t Hook_LightSel(void* a1, void* a2, std::uint32_t a3, void* a4, void* a5, void* a6, void* a7, void* a8, void* a9, void* a10)
		{
			if (!g_traceOn.load()) {
				if (g_inCull && g_params.lightMask && a1 && a5) {
					ForceLightMask(a1, a5);
				}
				return g_origLightSel(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10);
			}
			std::uint32_t n = 0;
			ReadSafeT(reinterpret_cast<const std::uint8_t*>(a1) + 0x10, n);
			{
				bool& seen = g_inCull ? g_lsSeenOurs : g_lsSeenEngine;
				if (!seen && a5) {
					seen = true;
					const auto*   node = reinterpret_cast<const std::uint8_t*>(a5);
					std::uint64_t sun = 0, cloud = 0, sunShadow = 0;
					std::uint32_t activeN = 0, shadowN = 0;
					ReadSafeT(node + 0x200, sun);
					ReadSafeT(node + 0x208, cloud);
					ReadSafeT(node + 0x210, sunShadow);
					ReadSafeT(node + 0x140, activeN);
					ReadSafeT(node + 0x158, shadowN);
					const std::uint8_t* lights = nullptr;
					ReadSafeT(node + 0x130, lights);
					std::string idx;
					for (std::uint32_t i2 = 0; lights && i2 < activeN && i2 < 8; ++i2) {
						std::uint64_t lp = 0;
						ReadSafeT(lights + i2 * 8, lp);
						idx += std::format(" {:#x}", lp);
					}
					{
						const std::uint8_t* accArr = nullptr;
						std::uint32_t       accN = 0;
						ReadSafeT(node + 0x230, accArr);
						ReadSafeT(node + 0x240, accN);
						std::string ent;
						for (std::uint32_t i2 = 0; accArr && i2 < accN && i2 < 6; ++i2) {
							const std::uint8_t* l = nullptr;
							ReadSafeT(accArr + i2 * 8, l);
							std::uint32_t chn = 0xDEAD;
							if (l) ReadSafeT(l + 0x520, chn);
							ent += std::format(" [{:p} channel {:#x}]", static_cast<const void*>(l), chn);
						}
						logger::info("trace: shadowLightsAccum of that node: {} entr(ies):{}", accN, ent);
					}
					logger::info("trace: shadow scene node {:p} at the first light selection of the {} cull: sunLight {:#x}, cloudLight {:#x}, sunShadowDirLight {:#x}, activeLights {} [{}], activeShadowLights {}", a5, g_inCull ? "OUR" : "engine's", sun, cloud, sunShadow, activeN, idx, shadowN);
				}
			}
			std::uint8_t before = 0xFF, after = 0xFF;
			if (a8) ReadSafeT(reinterpret_cast<const std::uint8_t*>(a8), before);
			if (g_inCull && g_params.lightMask && a1 && a5) {
				ForceLightMask(a1, a5);
			}
			const std::uint32_t ret = g_origLightSel(a1, a2, a3, a4, a5, a6, a7, a8, a9, a10);
			if (a8) ReadSafeT(reinterpret_cast<const std::uint8_t*>(a8), after);
			std::uint32_t out4 = 0xFFFF;
			if (a4) ReadSafeT(reinterpret_cast<const std::uint8_t*>(a4), out4);
			++g_lsHist[{ g_inCull, n, before, after, ret, out4 }];
			if (g_lightLogged < 12 && n > 0) {
				++g_lightLogged;
				const std::uint8_t* data = nullptr;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a1), data);
				std::string desc;
				for (std::uint32_t i2 = 0; i2 < n && i2 < 4; ++i2) {
					const std::uint8_t* light = nullptr;
					if (data) ReadSafeT(data + i2 * 8, light);
					std::uint32_t ch = 0;
					std::uint64_t flg = 0;
					if (light) { ReadSafeT(light + 0x5C, ch); ReadSafeT(light + 0x38, flg); }
					desc += std::format(" [light {:p} channel {:#x} +38 {:#x}]", static_cast<const void*>(light), ch, flg);
				}
				std::uint32_t mask = 0;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(a1) + 0x1C, mask);
				logger::info("trace: light selection{} list {:p} (lights {}, +0x1c {:#x}), max {}, first-person arg {}, a10 {:#x}: out byte {} -> {}, out dword {:#x}, returned {};{}", g_inCull ? " [OUR CULL]" : "", a1, n, mask, a3, reinterpret_cast<std::uintptr_t>(a9) & 0xFF, reinterpret_cast<std::uintptr_t>(a10) & 0xFFFFFFFF, before, after, out4, ret, desc);
			}
			return ret;
		}
		const hooks::Spec kSubmitAcc{ "Renderer::SubmitAccumulator", 106436, { 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57, 0x48, 0x83, 0xEC, 0x20, 0x41 } };
		const hooks::Spec kStartAcc{ "Renderer::StartAccumulating", 106437, { 0x48, 0x85, 0xC9, 0x74, 0x7C, 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57 } };
		const hooks::Spec kPostDepth{ "Renderer::FinishAccumulatingPostResolveDepth", 106438, { 0x48, 0x8B, 0x02, 0x48, 0x8B, 0xCA, 0x41, 0x8B, 0xD0, 0x48, 0xFF, 0xA0, 0x58, 0x01, 0x00, 0x00 } };
		const hooks::Spec kLightSel{ "light selection (id 107784)", 107784, { 0x4C, 0x89, 0x4C, 0x24, 0x20, 0x44, 0x89, 0x44, 0x24, 0x18, 0x48, 0x89, 0x54, 0x24, 0x10, 0x53 } };
		const hooks::Spec kSetRT{ "Renderer::SetRenderTarget", 77453, { 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x6C, 0x24, 0x10, 0x48, 0x89, 0x74, 0x24, 0x18, 0x57 } };
		const hooks::Spec kSetupPass{ "BSBatchRenderer::SetupAndDrawPass", 107644, { 0x48, 0x8B, 0xC4, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x81, 0xEC, 0x80 } };
		// the render target in slot 0 of the last OMSetRenderTargets (by renderTargets[] index), and the stencil/blend state set while the normal target (4/5) is bound
		int g_lastRt0{ -1 };
		struct StBlendHist
		{
			std::map<std::tuple<bool, int, int, int, int, int, int, int, int, int, UINT>, int> stencil;  // ours, depth func, depth write, stencil enable, read mask, write mask, func, pass op, fail op, depth-fail op, ref
			std::map<std::tuple<bool, int, int, int, int, int, int, UINT>, int>               blend;    // ours, rt0 blend enable, rt0 write mask, alpha to coverage, src, dest, rt1 write mask, sample mask
		};
		StBlendHist g_sb;
		// ---- ID3D11DeviceContext::OMSetDepthStencilState (vtable slot 36): which depth tests the engine's and our passes really use ----
		using OMSetDS_t = void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*, ID3D11DepthStencilState*, UINT);
		// G188: view t14over=v (0..255): in our virtual passes the pixel shader's t14 (the shadow mask) is replaced by our own 4x4 texture whose texels are (v/255, 0, 0, 1): does the picture follow t14's content at all?
		// G271 postlog state (declared here: the PSSetShaderResources and OMSetRenderTargets hooks below use it)
		std::atomic<int> g_postLogFrames{ 0 };
		std::atomic<bool> g_postLogOn{ false };
		int              g_postLogDraws = 0;
		std::atomic<long long> g_drawTotal{ 0 }, g_dispTotal{ 0 }, g_rtSetTotal{ 0 };
		std::string PostLogDescribeSrv(ID3D11ShaderResourceView* a_srv);
		using PSSetSRV_t = void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*, UINT, UINT, ID3D11ShaderResourceView* const*);
		PSSetSRV_t                       g_origPSSetSRV{ nullptr };
		std::atomic<int>                 g_t14Over{ -1 };
		std::atomic<int>                 g_t14Hits{ 0 };
		ComPtr<ID3D11ShaderResourceView> g_t14OverSrv;
		int                              g_t14OverVal{ -2 };
		void STDMETHODCALLTYPE Hook_PSSetSRV(ID3D11DeviceContext* a_ctx, UINT a_start, UINT a_num, ID3D11ShaderResourceView* const* a_srvs)
		{
			if (g_postLogOn.load() && a_srvs && a_num && g_postLogDraws < 700) {
				++g_postLogDraws;
				std::string d;
				for (UINT i = 0; i < a_num && i < 16; ++i) {
					d += std::format(" t{}={}", a_start + i, PostLogDescribeSrv(a_srvs[i]));
				}
				logger::info("postlog: PSSetShaderResources{}", d);
			}
			if (g_ssaoCapInCall.load() && a_srvs && a_num) {
				SsaoCapNoteSrvs(a_ctx, a_start, a_num, a_srvs);
			}
			if (g_inMaskStage && a_num >= 1 && a_num <= 16 && g_maskSrvLogged < 60) {
				++g_maskSrvLogged;
				std::string ptrs;
				for (UINT i = 0; i < a_num && i < 16; ++i) ptrs += std::format(" {:p}", a_srvs ? static_cast<const void*>(a_srvs[i]) : nullptr);
				logger::info("view: mask stage PSSetShaderResources start {} count {} srvs{}", a_start, a_num, ptrs);
			}
			if (g_capMaskOn.load() && !g_inMaskStage && a_srvs && a_num == 1 && a_start == 4 && a_srvs[0] && g_maskEngLogged < 6) {
				ComPtr<ID3D11Resource> re;
				a_srvs[0]->GetResource(&re);
				D3D11_SHADER_RESOURCE_VIEW_DESC ve{};
				a_srvs[0]->GetDesc(&ve);
				if (ve.ViewDimension == D3D11_SRV_DIMENSION_TEXTURE2DARRAY) {
					++g_maskEngLogged;
					logger::info("view: (engine) PSSetShaderResources t4: SRV {:p} resource {:p} view dim {} fmt {} mips {}..+{} slices {}..+{}", static_cast<const void*>(a_srvs[0]), static_cast<const void*>(re.Get()), static_cast<int>(ve.ViewDimension), static_cast<int>(ve.Format), ve.Texture2DArray.MostDetailedMip, ve.Texture2DArray.MipLevels, ve.Texture2DArray.FirstArraySlice, ve.Texture2DArray.ArraySize);
				}
			}
			if (g_inMaskStage && g_t4Shadow.load() != 0 && a_srvs && a_num <= 16 && a_start <= 4 && 4 < a_start + a_num) {
				auto* sh = g_t4Const.load() >= 0 ? T4ConstSrv(g_t4Const.load()) : FindSunShadowSrv();
				if (g_maskSrvLogged < 24) {
					++g_maskSrvLogged;
					ComPtr<ID3D11Resource> r0, r1;
					if (a_srvs[4 - a_start]) a_srvs[4 - a_start]->GetResource(&r0);
					if (sh) sh->GetResource(&r1);
					D3D11_SHADER_RESOURCE_VIEW_DESC vd0{}, vd1{};
					if (a_srvs[4 - a_start]) a_srvs[4 - a_start]->GetDesc(&vd0);
					if (sh) sh->GetDesc(&vd1);
					logger::info("view: mask stage t4 call: engine's SRV {:p}, our shadow map depthSRV {:p} (resource {:p}, view dim {} fmt {} mips {}..+{} slices {}..+{})", static_cast<const void*>(a_srvs[4 - a_start]), static_cast<const void*>(sh), static_cast<const void*>(r1.Get()), static_cast<int>(vd1.ViewDimension), static_cast<int>(vd1.Format), vd1.Texture2DArray.MostDetailedMip, vd1.Texture2DArray.MipLevels, vd1.Texture2DArray.FirstArraySlice, vd1.Texture2DArray.ArraySize);
				}
				if (sh) {
					ID3D11ShaderResourceView* tmp[16]{};
					for (UINT i = 0; i < a_num; ++i) tmp[i] = a_srvs[i];
					tmp[4 - a_start] = sh;
					g_origPSSetSRV(a_ctx, a_start, a_num, tmp);
					static int s_chk = 0;
					if (s_chk < 8) {
						++s_chk;
						ID3D11ShaderResourceView* back = nullptr;
						a_ctx->PSGetShaderResources(4, 1, &back);
						ID3D11DepthStencilView* dsvNow = nullptr;
						ID3D11RenderTargetView* rtvNow[1]{};
						a_ctx->OMGetRenderTargets(1, rtvNow, &dsvNow);
						logger::info("view: mask stage t4 set to {:p}; read back {:p}; depth target bound now: {:p}", static_cast<const void*>(sh), static_cast<const void*>(back), static_cast<const void*>(dsvNow));
						if (back) back->Release();
						if (dsvNow) dsvNow->Release();
						if (rtvNow[0]) rtvNow[0]->Release();
					}
					return;
				}
			}
			const int over = g_t14Over.load();
			if (over >= 0 && g_inVirtual && a_srvs && a_num <= 16 && a_start <= 14 && 14 < a_start + a_num) {
				if (!g_t14OverSrv || g_t14OverVal != over) {
					g_t14OverSrv.Reset();
					std::uint32_t px[16];
					for (auto& p : px) p = 0xFF000000u | static_cast<std::uint32_t>(over & 0xFF);
					D3D11_TEXTURE2D_DESC td{};
					td.Width = 4;
					td.Height = 4;
					td.MipLevels = 1;
					td.ArraySize = 1;
					td.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
					td.SampleDesc.Count = 1;
					td.Usage = D3D11_USAGE_IMMUTABLE;
					td.BindFlags = D3D11_BIND_SHADER_RESOURCE;
					D3D11_SUBRESOURCE_DATA sd{ px, 16, 0 };
					ComPtr<ID3D11Texture2D> tt;
					if (SUCCEEDED(Device()->CreateTexture2D(&td, &sd, &tt))) {
						Device()->CreateShaderResourceView(tt.Get(), nullptr, &g_t14OverSrv);
						g_t14OverVal = over;
					}
				}
				if (g_t14OverSrv) {
					ID3D11ShaderResourceView* tmp[16]{};
					for (UINT i = 0; i < a_num; ++i) tmp[i] = a_srvs[i];
					tmp[14 - a_start] = g_t14OverSrv.Get();
					if (g_t14Hits.fetch_add(1) < 3) {
						logger::info("view: t14over {}: replaced the SRV at t14 (call start {} count {}, was {:p})", over, a_start, a_num, static_cast<const void*>(a_srvs[14 - a_start]));
					}
					g_origPSSetSRV(a_ctx, a_start, a_num, tmp);
					return;
				}
			}
			g_origPSSetSRV(a_ctx, a_start, a_num, a_srvs);
		}
		OMSetDS_t g_origOMSetDS{ nullptr };

		// (ours?, accumulator render mode, depth enable, depth func, write mask, stencil enable) -> calls
		std::map<std::tuple<bool, std::uint32_t, int, int, int, int>, int> g_dsHist;
		void STDMETHODCALLTYPE Hook_OMSetDS(ID3D11DeviceContext* a_ctx, ID3D11DepthStencilState* a_state, UINT a_ref)
		{
			if (g_depthHack.load() != 0 && g_inVirtual && a_state && (g_lastRt0 == 1 || g_lastRt0 == 4 || g_lastRt0 == 5)) {
				std::uint32_t hmode = 0xFFFF;
				if (g_curAcc) ReadSafeT(reinterpret_cast<const std::uint8_t*>(g_curAcc) + 0x150, hmode);
				if (hmode == 0) {
					const int hack = g_depthHack.load();
					const auto key = std::make_pair(static_cast<void*>(a_state), hack);
					auto       it = g_dsHackCache.find(key);
					if (it == g_dsHackCache.end()) {
						D3D11_DEPTH_STENCIL_DESC hd{};
						a_state->GetDesc(&hd);
						bool change = false;
						if (hack == 1 && hd.DepthEnable && hd.DepthFunc != D3D11_COMPARISON_LESS_EQUAL) {
							hd.DepthFunc = D3D11_COMPARISON_LESS_EQUAL;
							change = true;
						}
						if (hack >= 2 && (!hd.DepthEnable || hd.DepthFunc != D3D11_COMPARISON_LESS_EQUAL)) {
							hd.DepthEnable = TRUE;
							hd.DepthFunc = D3D11_COMPARISON_LESS_EQUAL;
							change = true;
						}
						if (hack == 3 && hd.DepthWriteMask == D3D11_DEPTH_WRITE_MASK_ZERO) {
							hd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
							change = true;
						}
						ComPtr<ID3D11DepthStencilState> ns;
						static int s_hackLogged = 0;
						if (s_hackLogged++ < 12) {
							D3D11_DEPTH_STENCIL_DESC od{};
							a_state->GetDesc(&od);
							logger::info("view: depthhack {}: state {:p} depth enable {} func {} write {} -> {}", hack, static_cast<void*>(a_state), od.DepthEnable, static_cast<int>(od.DepthFunc), static_cast<int>(od.DepthWriteMask), change ? "CHANGED" : "unchanged");
						}
						if (change) {
							Device()->CreateDepthStencilState(&hd, &ns);
						}
						it = g_dsHackCache.emplace(key, ns).first;
					}
					if (it->second) {
						a_state = it->second.Get();
					}
				}
			}
			if (g_traceOn.load() && a_state) {
				D3D11_DEPTH_STENCIL_DESC d{};
				a_state->GetDesc(&d);
				std::uint32_t mode = 0xFFFF;
				if (g_curAcc) ReadSafeT(reinterpret_cast<const std::uint8_t*>(g_curAcc) + 0x150, mode);
				++g_dsHist[{ g_inVirtual, mode, d.DepthEnable, d.DepthFunc, d.DepthWriteMask, d.StencilEnable }];
				if (g_lastRt0 == 4 || g_lastRt0 == 5) {
					++g_sb.stencil[{ g_inVirtual, d.DepthFunc, d.DepthWriteMask, d.StencilEnable, d.StencilReadMask, d.StencilWriteMask, d.FrontFace.StencilFunc, d.FrontFace.StencilPassOp, d.FrontFace.StencilFailOp, d.FrontFace.StencilDepthFailOp, a_ref }];
				}
			}
			g_origOMSetDS(a_ctx, a_state, a_ref);
		}
		// ---- ID3D11DeviceContext::OMSetRenderTargets (vtable slot 33): which targets (by renderTargets[] index) the engine's and our passes really bind ----
		using OMSetRT_t = void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*, UINT, ID3D11RenderTargetView* const*, ID3D11DepthStencilView*);
		OMSetRT_t g_origOMSetRT{ nullptr };
		// (ours?, accumulator render mode, "slot:index slot:index ...") -> calls
		std::map<std::tuple<bool, std::uint32_t, std::string>, int> g_rtHist;
		void STDMETHODCALLTYPE Hook_OMSetRT(ID3D11DeviceContext* a_ctx, UINT a_n, ID3D11RenderTargetView* const* a_rtvs, ID3D11DepthStencilView* a_dsv)
		{
			++g_rtSetTotal;
			if (g_postLogOn.load() && g_postLogDraws < 700) {
				std::string rts;
				auto*       rdp = RE::BSGraphics::Renderer::GetRendererDataSingleton();
				for (UINT i = 0; i < a_n && i < 8 && a_rtvs; ++i) {
					if (!a_rtvs[i]) continue;
					ComPtr<ID3D11Resource> rr;
					a_rtvs[i]->GetResource(&rr);
					int ix = -1;
					for (int j = 0; j < 130 && rdp; ++j) {
						if (rdp->renderTargets[j].texture == reinterpret_cast<void*>(rr.Get())) { ix = j; break; }
					}
					rts += std::format(" {}:{}", i, ix);
				}
				{
					static std::set<std::uintptr_t> s_seenPs;
					static int                      s_nPs = 0;
					ComPtr<ID3D11PixelShader>       prevPs;
					a_ctx->PSGetShader(&prevPs, nullptr, nullptr);
					if (prevPs && s_nPs < 70 && s_seenPs.insert(reinterpret_cast<std::uintptr_t>(prevPs.Get())).second) {
						++s_nPs;
						const auto  capb = CaptureBindings(a_ctx, 0, 0);
						const auto  label = std::format("post{:02d}", s_nPs);
						logger::info("postlog: a new pass shader {:p} was bound by the pass that ends here (state captured as {})", static_cast<void*>(prevPs.Get()), label);
						LogCapture(label.c_str(), capb);
						DumpCaptureFiles(label.c_str(), capb);
					}
				}
				std::string stack;
				{
					void* fr[12]{};
					const USHORT nfr = CaptureStackBackTrace(1, 8, fr, nullptr);
					for (USHORT q = 0; q < nfr; ++q) {
						stack += std::format(" {:#x}", reinterpret_cast<std::uintptr_t>(fr[q]) - REL::Module::get().base());
					}
				}
				logger::info("postlog: OMSetRenderTargets n {} rts{} dsv {}{}", a_n, rts.empty() ? " none" : rts, a_dsv ? "yes" : "no", stack.empty() ? std::string() : " | stack (rva):" + stack);
			}
			if (!g_traceOn.load() && g_depthHack.load() != 0 && a_n && a_rtvs && a_rtvs[0]) {
				ComPtr<ID3D11Resource> r0;
				a_rtvs[0]->GetResource(&r0);
				int idx0 = -1;
				if (auto* rdh = RE::BSGraphics::Renderer::GetRendererDataSingleton()) {
					for (int j = 0; j < 130; ++j) {
						if (rdh->renderTargets[j].texture == reinterpret_cast<void*>(r0.Get())) {
							idx0 = j;
							break;
						}
					}
				}
				g_lastRt0 = idx0;
			}
			if (g_traceOn.load() && a_n && a_rtvs) {
				std::uint32_t mode = 0xFFFF;
				if (g_curAcc) ReadSafeT(reinterpret_cast<const std::uint8_t*>(g_curAcc) + 0x150, mode);
				std::string s;
				g_lastRt0 = -2;
				auto* rd2 = RE::BSGraphics::Renderer::GetRendererDataSingleton();
				for (UINT i = 0; i < a_n && i < 8; ++i) {
					if (!a_rtvs[i]) continue;
					ComPtr<ID3D11Resource> res;
					a_rtvs[i]->GetResource(&res);
					int idx = -1;
					for (int j = 0; j < 130 && rd2; ++j) {
						if (rd2->renderTargets[j].texture == reinterpret_cast<void*>(res.Get())) {
							idx = j;
							break;
						}
					}
					s += std::format(" {}:{}", i, idx);
					if (i == 0) g_lastRt0 = idx;
				}
				++g_rtHist[{ g_inVirtual, mode, s }];
				g_lastSig = s;
				static int s_seqFrame = -1, s_seqCount = 0;
				if (s_seqFrame != g_traceFrames) {
					s_seqFrame = g_traceFrames;
					s_seqCount = 0;
				}
				if (s_seqCount++ < 500) {
					logger::info("trace: OMSetRenderTargets seq{} mode {} [{} ]{}", g_inVirtual ? " [OURS]" : "", mode, s, a_dsv ? " +depth" : "");
				}
			}
			g_origOMSetRT(a_ctx, a_n, a_rtvs, a_dsv);
		}
		using OMSetBlend_t = void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*, ID3D11BlendState*, const FLOAT*, UINT);
		OMSetBlend_t g_origOMSetBlend{ nullptr };
		void STDMETHODCALLTYPE Hook_OMSetBlend(ID3D11DeviceContext* a_ctx, ID3D11BlendState* a_state, const FLOAT* a_factor, UINT a_mask)
		{
			if (g_traceOn.load() && a_state && (g_lastRt0 == 4 || g_lastRt0 == 5)) {
				D3D11_BLEND_DESC d{};
				a_state->GetDesc(&d);
				++g_sb.blend[{ g_inVirtual, d.RenderTarget[0].BlendEnable, d.RenderTarget[0].RenderTargetWriteMask, d.AlphaToCoverageEnable, d.RenderTarget[0].SrcBlend, d.RenderTarget[0].DestBlend, d.RenderTarget[1].RenderTargetWriteMask, a_mask }];
			}
			g_origOMSetBlend(a_ctx, a_state, a_factor, a_mask);
		}
		using DrawIdx_t = void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*, UINT, UINT, INT);
		using Draw_t = void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*, UINT, UINT);
		using DrawIdxInst_t = void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*, UINT, UINT, UINT, INT, UINT);
		using DrawInst_t = void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*, UINT, UINT, UINT, UINT);
		DrawIdx_t     g_origDrawIndexed{ nullptr };
		Draw_t        g_origDraw{ nullptr };
		DrawIdxInst_t g_origDrawIdxInst{ nullptr };
		DrawInst_t    g_origDrawInst{ nullptr };
		// every draw while tracing: (ours?, accumulator render mode, draw kind, "slot:index" list of the render targets bound at draw time, rt0 write mask, rt1 write mask, depth write) -> (draws, indices)
		std::map<std::tuple<bool, std::uint32_t, int, std::string, int, int, int>, std::pair<int, std::uint64_t>> g_drawSig;
		// ---- G271 view postlog=N: the next N frames, every draw after overlay::Draw until the end of the frame function is logged: render target, pixel shader, the textures bound to t0-t9 (render target index or depth), the draw size ----
		std::string PostLogDescribeSrv(ID3D11ShaderResourceView* a_srv)
		{
			if (!a_srv) {
				return "-";
			}
			ComPtr<ID3D11Resource> res;
			a_srv->GetResource(&res);
			auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			for (int j = 0; j < 130 && rd; ++j) {
				if (rd->renderTargets[j].texture == reinterpret_cast<void*>(res.Get())) {
					ComPtr<ID3D11Texture2D> t;
					D3D11_TEXTURE2D_DESC    td{};
					if (SUCCEEDED(res.As(&t))) {
						t->GetDesc(&td);
					}
					return std::format("rt{}({}x{} f{})", j, td.Width, td.Height, static_cast<int>(td.Format));
				}
			}
			auto& dsd = RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData();
			for (int k = 0; k < 3; ++k) {
				if (reinterpret_cast<void*>(dsd.depthStencils[k].depthSRV) == reinterpret_cast<void*>(a_srv)) {
					return std::format("DEPTH{}", k);
				}
			}
			ComPtr<ID3D11Texture2D> t;
			D3D11_TEXTURE2D_DESC    td{};
			if (SUCCEEDED(res.As(&t))) {
				t->GetDesc(&td);
				return std::format("other({}x{} f{})", td.Width, td.Height, static_cast<int>(td.Format));
			}
			return "other";
		}
		void PostLogDraw(ID3D11DeviceContext* a_ctx, int a_kind, UINT a_count)
		{
			if (g_postLogDraws >= 700) {
				return;
			}
			++g_postLogDraws;
			ID3D11RenderTargetView*        rtvs[2]{};
			ComPtr<ID3D11DepthStencilView> dsv;
			a_ctx->OMGetRenderTargets(2, rtvs, dsv.GetAddressOf());
			auto*       rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			std::string rt;
			for (int i = 0; i < 2; ++i) {
				if (!rtvs[i]) {
					continue;
				}
				ComPtr<ID3D11Resource> res;
				rtvs[i]->GetResource(&res);
				int idx = -1;
				for (int j = 0; j < 130 && rd; ++j) {
					if (rd->renderTargets[j].texture == reinterpret_cast<void*>(res.Get())) {
						idx = j;
						break;
					}
				}
				rt += std::format(" {}:{}", i, idx);
				rtvs[i]->Release();
			}
			if (rt.empty()) {
				rt = " none";
			}
			ComPtr<ID3D11PixelShader> ps;
			a_ctx->PSGetShader(&ps, nullptr, nullptr);
			ID3D11ShaderResourceView* srvs[10]{};
			a_ctx->PSGetShaderResources(0, 10, srvs);
			std::string sr;
			for (int i = 0; i < 10; ++i) {
				if (srvs[i]) {
					sr += std::format(" t{}={}", i, PostLogDescribeSrv(srvs[i]));
					srvs[i]->Release();
				}
			}
			logger::info("postlog: f{} draw {} kind {} n {} rt{} dsv {} ps {:p} |{}", frame::Count(), g_postLogDraws, a_kind, a_count, rt, dsv ? "yes" : "no", static_cast<void*>(ps.Get()), sr);
		}
		void RecordDraw(ID3D11DeviceContext* a_ctx, int a_kind, UINT a_count)
		{
			++g_drawTotal;
			if (g_postLogOn.load()) {
				PostLogDraw(a_ctx, a_kind, a_count);
			}
			if (!g_traceOn.load()) {
				return;
			}
			ID3D11RenderTargetView*        rtvs[8]{};
			ComPtr<ID3D11DepthStencilView> dsv;
			a_ctx->OMGetRenderTargets(8, rtvs, dsv.GetAddressOf());
			auto*       rd2 = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			std::string sig;
			for (int i = 0; i < 8; ++i) {
				if (!rtvs[i]) {
					continue;
				}
				ComPtr<ID3D11Resource> res;
				rtvs[i]->GetResource(&res);
				int idx = -1;
				for (int j = 0; j < 130 && rd2; ++j) {
					if (rd2->renderTargets[j].texture == reinterpret_cast<void*>(res.Get())) {
						idx = j;
						break;
					}
				}
				sig += std::format(" {}:{}", i, idx);
				rtvs[i]->Release();
			}
			if (sig.empty()) {
				sig = " none";
			}
			ComPtr<ID3D11BlendState> bs;
			FLOAT                    bf[4]{};
			UINT                     sm = 0;
			a_ctx->OMGetBlendState(&bs, bf, &sm);
			int wm0 = -1, wm1 = -1;
			if (bs) {
				D3D11_BLEND_DESC d{};
				bs->GetDesc(&d);
				wm0 = d.RenderTarget[0].RenderTargetWriteMask;
				wm1 = d.RenderTarget[1].RenderTargetWriteMask;
			}
			ComPtr<ID3D11DepthStencilState> ds;
			UINT                            ref = 0;
			a_ctx->OMGetDepthStencilState(&ds, &ref);
			int dw = -1;
			if (ds) {
				D3D11_DEPTH_STENCIL_DESC d{};
				ds->GetDesc(&d);
				dw = d.DepthWriteMask;
			}
			std::uint32_t mode = 0xFFFF;
			if (g_curAcc) ReadSafeT(reinterpret_cast<const std::uint8_t*>(g_curAcc) + 0x150, mode);
			auto& e = g_drawSig[{ g_inVirtual, mode, a_kind, sig, wm0, wm1, dw }];
			++e.first;
			e.second += a_count;
		}
		void STDMETHODCALLTYPE Hook_DrawIndexed(ID3D11DeviceContext* a_ctx, UINT a_count, UINT a_start, INT a_base)
		{
			RecordDraw(a_ctx, 0, a_count);
			g_origDrawIndexed(a_ctx, a_count, a_start, a_base);
		}
		void STDMETHODCALLTYPE Hook_Draw(ID3D11DeviceContext* a_ctx, UINT a_count, UINT a_start)
		{
			RecordDraw(a_ctx, 1, a_count);
			g_origDraw(a_ctx, a_count, a_start);
		}
		void STDMETHODCALLTYPE Hook_DrawIdxInst(ID3D11DeviceContext* a_ctx, UINT a_ic, UINT a_inst, UINT a_start, INT a_base, UINT a_sinst)
		{
			RecordDraw(a_ctx, 2, a_ic * a_inst);
			g_origDrawIdxInst(a_ctx, a_ic, a_inst, a_start, a_base, a_sinst);
		}
		void STDMETHODCALLTYPE Hook_DrawInst(ID3D11DeviceContext* a_ctx, UINT a_vc, UINT a_inst, UINT a_start, UINT a_sinst)
		{
			RecordDraw(a_ctx, 3, a_vc * a_inst);
			g_origDrawInst(a_ctx, a_vc, a_inst, a_start, a_sinst);
		}
		void InstallDSHook()
		{
			static bool done = false;
			if (done) return;
			done = true;
			ComPtr<ID3D11DeviceContext> ctx;
			Device()->GetImmediateContext(&ctx);
			void** vt = *reinterpret_cast<void***>(ctx.Get());
			DWORD  old = 0;
			if (VirtualProtect(&vt[36], sizeof(void*), PAGE_READWRITE, &old)) {
				g_origOMSetDS = reinterpret_cast<OMSetDS_t>(vt[36]);
				vt[36] = reinterpret_cast<void*>(&Hook_OMSetDS);
				VirtualProtect(&vt[36], sizeof(void*), old, &old);
				logger::info("view: OMSetDepthStencilState hooked (context vtable {:p})", static_cast<void*>(vt));
			}
			if (VirtualProtect(&vt[8], sizeof(void*), PAGE_READWRITE, &old)) {
				g_origPSSetSRV = reinterpret_cast<PSSetSRV_t>(vt[8]);
				vt[8] = reinterpret_cast<void*>(&Hook_PSSetSRV);
				VirtualProtect(&vt[8], sizeof(void*), old, &old);
			}
			if (VirtualProtect(&vt[33], sizeof(void*), PAGE_READWRITE, &old)) {
				g_origOMSetRT = reinterpret_cast<OMSetRT_t>(vt[33]);
				vt[33] = reinterpret_cast<void*>(&Hook_OMSetRT);
				VirtualProtect(&vt[33], sizeof(void*), old, &old);
			}
			if (VirtualProtect(&vt[35], sizeof(void*), PAGE_READWRITE, &old)) {
				g_origOMSetBlend = reinterpret_cast<OMSetBlend_t>(vt[35]);
				vt[35] = reinterpret_cast<void*>(&Hook_OMSetBlend);
				VirtualProtect(&vt[35], sizeof(void*), old, &old);
			}
			for (auto [slot, hook, orig] : { std::tuple<int, void*, void**>{ 12, reinterpret_cast<void*>(&Hook_DrawIndexed), reinterpret_cast<void**>(&g_origDrawIndexed) },
											 std::tuple<int, void*, void**>{ 13, reinterpret_cast<void*>(&Hook_Draw), reinterpret_cast<void**>(&g_origDraw) },
											 std::tuple<int, void*, void**>{ 20, reinterpret_cast<void*>(&Hook_DrawIdxInst), reinterpret_cast<void**>(&g_origDrawIdxInst) },
											 std::tuple<int, void*, void**>{ 21, reinterpret_cast<void*>(&Hook_DrawInst), reinterpret_cast<void**>(&g_origDrawInst) } }) {
				if (VirtualProtect(&vt[slot], sizeof(void*), PAGE_READWRITE, &old)) {
					*orig = vt[slot];
					vt[slot] = hook;
					VirtualProtect(&vt[slot], sizeof(void*), old, &old);
				}
			}
		}
		// the light selection hook (the shadow forcing) and the batch renderer's per-pass hook (the capture of the engine's mask draw): needed by default, installed once at plugin load (G220)
		void InstallPassHooks()
		{
			static bool done = false;
			if (done) {
				return;
			}
			done = true;
			const bool d = hooks::Install(kSetupPass, reinterpret_cast<void*>(&Hook_SetupPass), reinterpret_cast<void**>(&g_origSetupPass));
			const bool f = hooks::Install(kLightSel, reinterpret_cast<void*>(&Hook_LightSel), reinterpret_cast<void**>(&g_origLightSel));
			logger::info("view: shadow hooks: SetupAndDrawPass {}, light selection {}", d, f);
		}
		void EnsureTraceHooks()
		{
			static bool tried = false;
			if (tried) {
				return;
			}
			tried = true;
			const bool a = hooks::Install(kSubmitAcc, reinterpret_cast<void*>(&Hook_Submit), reinterpret_cast<void**>(&g_origSubmit));
			const bool b = hooks::Install(kStartAcc, reinterpret_cast<void*>(&Hook_StartAccum), reinterpret_cast<void**>(&g_origStartAccum));
			const bool c = hooks::Install(kPostDepth, reinterpret_cast<void*>(&Hook_PostDepth), reinterpret_cast<void**>(&g_origPostDepth));
			InstallPassHooks();
			const bool d = true;
			const bool e = hooks::Install(kSetRT, reinterpret_cast<void*>(&Hook_SetRT), reinterpret_cast<void**>(&g_origSetRT));
			const bool f = true;
			InstallDSHook();
			logger::info("view: render trace hooks: SubmitAccumulator {}, StartAccumulating {}, FinishAccumulatingPostResolveDepth {}, SetupAndDrawPass {}, SetRenderTarget {}, light selection {}", a, b, c, d, e, f);
			{
				// the per-render-mode handler tables: [0x33d4a20] = accumulate (geometry -> passes), [0x33d4b20] = finish/draw (id 414684 / 414716)
				for (int mode = 0; mode < 0x1d; ++mode) {
					std::uint64_t acc = 0, fin = 0;
					ReadSafeT(reinterpret_cast<const std::uint8_t*>(REL::ID(414684).address()) + mode * 8, acc);
					ReadSafeT(reinterpret_cast<const std::uint8_t*>(REL::ID(414716).address()) + mode * 8, fin);
					const auto base = REL::Module::get().base();
					logger::info("view: render mode {} handlers: accumulate rva {:#x}, finish rva {:#x}", mode, acc ? acc - base : 0, fin ? fin - base : 0);
				}
			}
		}
		void TraceFrameBoundary()
		{
			if (g_traceOn.load()) {
				for (const auto& [key, st] : g_passStats) {
					logger::info("trace: passes of accumulator {:p} {}: {} draws, alphaTest argument true {}, alpha test enabled in the geometry's alpha property {} (of them with the argument false {}, forced true {})", key.first, key.second ? "[OUR VIRTUAL PASS]" : "(engine)", st.calls, st.argTrue, st.propTest, st.propTestArgFalse, st.forced);
				}
				{
					int shown = 0;
					for (const auto& [k, calls] : g_lsHist) {
						if (++shown > 120) break;
						logger::info("trace: light selection histogram {}: list of {} light(s), out byte {} -> {}, returned {}, out dword {:#x}: {} call(s)", std::get<0>(k) ? "[OUR CULL]" : "(engine)", std::get<1>(k), std::get<2>(k), std::get<3>(k), std::get<4>(k), std::get<5>(k), calls);
					}
				}
				for (const auto& [k, calls] : g_sb.stencil) {
					logger::info("trace: normal-target draws {}: depth func {}, depth write {}, stencil enable {}, read mask {:#x}, write mask {:#x}, func {}, pass op {}, fail op {}, depth-fail op {}, ref {}: {} call(s)", std::get<0>(k) ? "[OURS]" : "(engine)", std::get<1>(k), std::get<2>(k), std::get<3>(k), std::get<4>(k), std::get<5>(k), std::get<6>(k), std::get<7>(k), std::get<8>(k), std::get<9>(k), std::get<10>(k), calls);
				}
				for (const auto& [k, calls] : g_sb.blend) {
					logger::info("trace: normal-target blend {}: rt0 blend enable {}, rt0 write mask {:#x}, alpha to coverage {}, src {}, dest {}, rt1 write mask {:#x}, sample mask {:#x}: {} call(s)", std::get<0>(k) ? "[OURS]" : "(engine)", std::get<1>(k), std::get<2>(k), std::get<3>(k), std::get<4>(k), std::get<5>(k), std::get<6>(k), std::get<7>(k), calls);
				}
				for (const auto& [k, calls] : g_passRt) {
					const std::string& sg = std::get<4>(k);
					if (sg.find(":4") == std::string::npos && sg.find(":5") == std::string::npos) continue;
					logger::info("trace: passes drawn with the normal target bound {}: shader vtable rva {:#x}, technique {:#x}, pass flags {:#x}, render targets [slot:index]{}: {} pass(es)", std::get<0>(k) ? "[OURS]" : "(engine)", std::get<1>(k), std::get<2>(k), std::get<3>(k), sg, calls);
				}
				for (const auto& [k, v] : g_drawSig) {
					logger::info("trace: draws {} (render mode {}) kind {} (0 indexed, 1 plain, 2 indexed instanced, 3 instanced): render targets [slot:index]{} , rt0 write mask {:#x}, rt1 write mask {:#x}, depth write {}: {} draw(s), {} indices", std::get<0>(k) ? "[OURS]" : "(engine)", std::get<1>(k), std::get<2>(k), std::get<3>(k), std::get<4>(k), std::get<5>(k), std::get<6>(k), v.first, v.second);
				}
				for (const auto& [k, calls] : g_rtHist) {
					logger::info("trace: OMSetRenderTargets {} (render mode {}): targets [slot:index]{}: {} call(s)", std::get<0>(k) ? "[OURS]" : "(engine)", std::get<1>(k), std::get<2>(k), calls);
				}
				for (const auto& [k, calls] : g_dsHist) {
					logger::info("trace: depth-stencil state set {} (render mode {}): depth test {}, function {} (1 never, 2 less, 3 equal, 4 less-equal, 5 greater, 6 not-equal, 7 greater-equal, 8 always), write mask {}, stencil {}: {} call(s)", std::get<0>(k) ? "[OURS]" : "(engine)", std::get<1>(k), std::get<2>(k), std::get<3>(k), std::get<4>(k), std::get<5>(k), calls);
				}
				int groupLines = 0;
				for (const auto& [key, gr] : g_groups) {
					std::uint32_t mode = 0xFFFF;
					std::uint8_t  eq = 0xFF;
					if (std::get<0>(key)) {
						ReadSafeT(reinterpret_cast<const std::uint8_t*>(std::get<0>(key)) + 0x150, mode);
						ReadSafeT(reinterpret_cast<const std::uint8_t*>(std::get<0>(key)) + 0x5C, eq);
					}
					if (mode != 0 && mode != 12) continue;
					if (++groupLines > 700) break;
					logger::info("trace: group {} acc {:p} (mode {} eq {}) shader vtable rva {:#x} technique {:#x} pass flags {:#x}: {} draws, alphaTest arg true {}, alpha-property test {}; first '{}' geometry vtable rva {:#x} shaderProperty vtable rva {:#x}", std::get<1>(key) ? "[OURS]" : "(engine)", std::get<0>(key), mode, eq, std::get<2>(key), std::get<3>(key), std::get<4>(key), gr.calls, gr.argTrue, gr.prop, gr.name, gr.geomVt, gr.propVt);
				}

				{
					int oursAll = 0, oursBits = 0, engAll = 0, engBits = 0;
					for (const auto& [key, gr] : g_groups) {
						if (std::get<2>(key) != 0x1b47420) continue;
						std::uint32_t mode = 0xFFFF;
						if (std::get<0>(key)) ReadSafeT(reinterpret_cast<const std::uint8_t*>(std::get<0>(key)) + 0x150, mode);
						const bool bits = (std::get<3>(key) & 0x6000) == 0x6000;
						if (std::get<1>(key)) { oursAll += gr.calls; if (bits) oursBits += gr.calls; }
						else if (mode == 0) { engAll += gr.calls; if (bits) engBits += gr.calls; }
					}
					logger::info("trace: lighting-shader draws with technique bits 0x6000: ours {} of {}, the engine's mode-0 passes {} of {}", oursBits, oursAll, engBits, engAll);
				}
				std::string t;
				for (const auto& [tech, n] : g_techNoArg) {
					t += std::format("{}{:#x} x{}", t.empty() ? "" : ", ", tech, n);
				}
				if (!t.empty()) {
					logger::info("trace: techniques of our draws with the alpha test enabled and the argument false: {}", t);
				}
				{
					int both = 0, same = 0, differ = 0, onlyOurs = 0, onlyEngine = 0, shown = 0, engBitsOnly = 0, oursBitsOnly = 0, otherDiff = 0;
					int shownOnly[2]{ 0, 0 };
					for (const auto& [gp, gt] : g_geomTech) {
						if (!gt.ours.empty() && !gt.engine.empty()) {
							++both;
							std::set<std::uint32_t> oc, ec;  // the colour-pass techniques only (our pre-pass utility techniques are small numbers)
							for (auto tv : gt.ours) if (tv >= 0x40000000u) oc.insert(tv);
							for (auto tv : gt.engine) if (tv >= 0x40000000u) ec.insert(tv);
							if (oc == ec) {
								++same;
							} else {
								++differ;
								const bool ob = std::any_of(oc.begin(), oc.end(), [](auto t) { return (t & 0x6000) == 0x6000; });
								const bool eb = std::any_of(ec.begin(), ec.end(), [](auto t) { return (t & 0x6000) == 0x6000; });
								if (eb && !ob) ++engBitsOnly;
								else if (ob && !eb) ++oursBitsOnly;
								else ++otherDiff;
								if (shown < 120) {
									++shown;
									std::string a, b;
									for (auto tv : oc) a += std::format(" {:#x}", tv);
									for (auto tv : ec) b += std::format(" {:#x}", tv);
									logger::info("trace: geometry {:p} '{}' shader {:#x}: techniques ours:{} | engine:{}", gp, gt.name, gt.shaderVt, a, b);
								}
							}
						} else if (!gt.ours.empty()) {
							++onlyOurs;
							if (shownOnly[0] < 70) {
								++shownOnly[0];
								std::string a;
								for (auto tv : gt.ours) if (tv >= 0x40000000u) a += std::format(" {:#x}", tv);
								logger::info("trace: geometry only in OUR pass {:p} '{}' shader {:#x}: techniques{}", gp, gt.name, gt.shaderVt, a);
							}
						} else {
							++onlyEngine;
							if (shownOnly[1] < 70) {
								++shownOnly[1];
								std::string b;
								for (auto tv : gt.engine) if (tv >= 0x40000000u) b += std::format(" {:#x}", tv);
								logger::info("trace: geometry only in the ENGINE's pass {:p} '{}' shader {:#x}: techniques{}", gp, gt.name, gt.shaderVt, b);
							}
						}
					}
					logger::info("trace: geometries whose colour technique differs: the engine has the shadow bits and we do not {}, we have them and the engine does not {}, other differences {}", engBitsOnly, oursBitsOnly, otherDiff);
					logger::info("trace: geometries drawn by both our pass and an engine mode-0 pass: {} (identical technique sets {}, different {}); only ours {}; only the engine's {}", both, same, differ, onlyOurs, onlyEngine);
					if (g_engineColourAcc && g_acc) {
						const auto* e = reinterpret_cast<const std::uint8_t*>(g_engineColourAcc);
						const auto* o = reinterpret_cast<const std::uint8_t*>(g_acc);
						int dl = 0;
						for (std::size_t off = 0x10; off < 0x1E0 && dl < 80; off += 8) {
							std::uint64_t ev = 0, ov = 0;
							ReadSafeT(e + off, ev);
							ReadSafeT(o + off, ov);
							if (ev != ov) {
								++dl;
								logger::info("trace: accumulator word +{:#x}: engine colour {:#018x} | ours {:#018x}", off, ev, ov);
							}
						}
					}
				}
				g_traceOn = false;
			}
			if (g_traceFrames > 0) {
				const std::uint8_t* obj = nullptr;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(REL::ID(415031).address()), obj);
				std::uintptr_t ovt = 0;
				std::uint32_t  of = 0;
				if (obj) { ReadSafeT(obj, ovt); ReadSafeT(obj + 0xF4, of); }
				logger::info("trace: render-state object (id 415031) {:p} vtable rva {:#x} +0xF4 = {:#x} at the start of the frame", static_cast<const void*>(obj), ovt ? ovt - REL::Module::get().base() : 0, of);
			}
			g_passStats.clear();
			g_groups.clear();
			g_lsHist.clear();
			g_dsHist.clear();
			g_rtHist.clear();
			g_sb.stencil.clear();
			g_sb.blend.clear();
			g_drawSig.clear();
			g_passRt.clear();
			if (g_capOn) {
				DumpPsUse();
			}
			OcclReport();
			g_capGeom = nullptr;
			g_capOursValid = false;
			g_capEngineDone = false;
			g_lsSeenOurs = g_lsSeenEngine = false;
			g_geomTech.clear();
			g_engineColourAcc = nullptr;
			g_lightLogged = 0;
			g_techNoArg.clear();
			g_noArgLogged = 0;
			g_traceLines = 0;
			if (g_traceFrames > 0) {
				--g_traceFrames;
				g_traceOn = true;
				logger::info("trace: ---- a traced frame begins (our passes run first, then the engine's frame) ----");
			}
		}

		// G124: extra fields written into our accumulators after the defaults: (offset, value, size in bytes)
		struct AccPoke
		{
			std::size_t   off;
			std::uint64_t val;
			int           size;
		};
		std::vector<AccPoke> g_accPokes;
		void SetAccFields(std::uint8_t* a_acc, const NiPoint3& a_pos)
		{
			*reinterpret_cast<std::uint8_t*>(a_acc + 0xB8) = 1;
			*reinterpret_cast<std::uint8_t*>(a_acc + 0x178) = g_params.lightMask ? 1 : 0;  // the shadow-technique gate (default on since G220)
			*reinterpret_cast<void**>(a_acc + 0x148) = SceneStateNode();
			*reinterpret_cast<std::uint32_t*>(a_acc + 0x150) = 0;  // renderMode kNormal
			*reinterpret_cast<float*>(a_acc + 0x16C) = a_pos.x;
			*reinterpret_cast<float*>(a_acc + 0x170) = a_pos.y;
			*reinterpret_cast<float*>(a_acc + 0x174) = a_pos.z;
			for (const auto& p : g_accPokes) {
				std::memcpy(a_acc + p.off, &p.val, static_cast<std::size_t>(p.size));
			}
		}

		// ---- oblique near plane ----
		// BSGraphics::State::BuildCameraStateData fills the engine's per-camera matrices (the shader constants come from them). When it runs for our
		// camera copy we replace the depth column of the view-projection with the exit plane (Lengyel's oblique near-plane clipping, D3D depth 0..1):
		// z' = k * (N.p_rel + d), p_rel = p - camera position (the engine renders camera-relative), k chosen so the far corner keeps depth 1.
		struct Oblique
		{
			bool     active{ false };
			NiPoint3 n{};        // plane normal, pointing to the kept side
			float    dRel{ 0 };  // N.(camera - point on plane) + bias: the plane's value at the camera (negative: camera behind)
			NiPoint3 fwd{}, up{}, right{};
			float    tl{ 0 }, tr{ 0 }, tb{ 0 }, tt{ 0 }, nearD{ 0 }, farD{ 0 };
			float    k{ 0 };           // the depth column scale PatchEntry chose (z' = k * (N.p_rel + dRel) / forward distance)
			float    jx{ 0 }, jy{ 0 };  // the engine's camera jitter of this frame in NDC units (jittered clip x = unjittered clip x + jx * w), measured from VP against the unjittered VPu before the patch (G260b)
			bool     patched{ false };  // a camera entry of this pass really got the oblique column
		};
		Oblique g_obl;
		// the column 2 (the depth column) of the four matrices of every camera entry PatchEntry changed, before and after, so the entries can be put back to the standard projection for a while (G228)
		struct OblSaved
		{
			std::uint8_t* entry{ nullptr };
			float         orig[16]{}, patched[16]{};
		};
		std::vector<OblSaved> g_oblSaved;
		int     g_oblApplied{ 0 };
		int     g_diagN{ 0 };
		int     g_oblLogged{ 0 };

		using Build_t = void (*)(void*, void*, const void*, bool, std::uint64_t);
		Build_t g_origBuild{ nullptr };
		const hooks::Spec kBuild{ "State::BuildCameraStateData", 77520, { 0x48, 0x8B, 0xC4, 0x48, 0x89, 0x58, 0x08, 0x48, 0x89, 0x68, 0x10, 0x48, 0x89, 0x70, 0x18, 0x48 } };

		bool Invert4(const float* m, float* inv)
		{
			double a[4][8];
			for (int r = 0; r < 4; ++r) {
				for (int c = 0; c < 4; ++c) {
					a[r][c] = m[r * 4 + c];
					a[r][4 + c] = (r == c) ? 1.0 : 0.0;
				}
			}
			for (int i = 0; i < 4; ++i) {
				int piv = i;
				for (int k = i + 1; k < 4; ++k) {
					if (std::fabs(a[k][i]) > std::fabs(a[piv][i])) piv = k;
				}
				if (std::fabs(a[piv][i]) < 1e-12) return false;
				for (int c = 0; c < 8; ++c) std::swap(a[i][c], a[piv][c]);
				const double d = a[i][i];
				for (int c = 0; c < 8; ++c) a[i][c] /= d;
				for (int k = 0; k < 4; ++k) {
					if (k == i) continue;
					const double f = a[k][i];
					for (int c = 0; c < 8; ++c) a[k][c] -= f * a[i][c];
				}
			}
			for (int r = 0; r < 4; ++r) {
				for (int c = 0; c < 4; ++c) inv[r * 4 + c] = static_cast<float>(a[r][4 + c]);
			}
			return true;
		}

		void Mul4(const float* a, const float* b, float* out)  // row-vector convention: out = a * b
		{
			for (int r = 0; r < 4; ++r) {
				for (int c = 0; c < 4; ++c) {
					out[r * 4 + c] = a[r * 4 + 0] * b[0 * 4 + c] + a[r * 4 + 1] * b[1 * 4 + c] + a[r * 4 + 2] * b[2 * 4 + c] + a[r * 4 + 3] * b[3 * 4 + c];
				}
			}
		}

		void PatchEntry(std::uint8_t* a_entry)
		{
			float* P = reinterpret_cast<float*>(a_entry + 0x80);
			float* VP = reinterpret_cast<float*>(a_entry + 0xC0);
			float* VPu = reinterpret_cast<float*>(a_entry + 0x140);
			float* Pu = reinterpret_cast<float*>(a_entry + 0x1C0);
			float* V = reinterpret_cast<float*>(a_entry + 0x40);
			const Oblique& o = g_obl;

			// calibration: does [p_rel, 1] * VP reproduce my own projection of the same point? (logged for the first few builds)
			if (g_oblLogged < 3) {
				++g_oblLogged;
				const NiPoint3 pts[3] = { o.fwd * 500.0f + o.right * 100.0f + o.up * 50.0f, o.fwd * 1500.0f - o.right * 300.0f + o.up * 20.0f, o.fwd * 80.0f };
				for (const auto& p : pts) {
					const float x = Dot(p, o.right), y = Dot(p, o.up), f = Dot(p, o.fwd);
					const float mx = 2.0f * (x / f - o.tl) / (o.tr - o.tl) - 1.0f, my = 2.0f * (y / f - o.tb) / (o.tt - o.tb) - 1.0f;
					const float mz = o.farD * (f - o.nearD) / (f * (o.farD - o.nearD));
					float       c[4];
					for (int j = 0; j < 4; ++j) c[j] = p.x * VP[0 * 4 + j] + p.y * VP[1 * 4 + j] + p.z * VP[2 * 4 + j] + VP[3 * 4 + j];
					logger::info("view: oblique calibration: p_rel ({:.0f},{:.0f},{:.0f}) mine ndc ({:.4f},{:.4f},{:.6f}) engine clip/w ({:.4f},{:.4f},{:.6f}) w {:.2f} (f {:.2f})", p.x, p.y, p.z, mx, my, mz, c[0] / c[3], c[1] / c[3], c[2] / c[3], c[3], f);
				}
				logger::info("view: engine VP row0 ({:.4f} {:.4f} {:.4f} {:.4f}) row1 ({:.4f} {:.4f} {:.4f} {:.4f}) row2 ({:.4f} {:.4f} {:.4f} {:.4f}) row3 ({:.4f} {:.4f} {:.4f} {:.4f})",
					VP[0], VP[1], VP[2], VP[3], VP[4], VP[5], VP[6], VP[7], VP[8], VP[9], VP[10], VP[11], VP[12], VP[13], VP[14], VP[15]);
			}

			// the camera jitter: the engine's jittered VP differs from the unjittered one only in the clip x / clip y columns, by jitter * w (the w column is shared)
			{
				int   best = 0;
				float bw = 0.0f;
				for (int r = 0; r < 4; ++r) {
					if (std::fabs(VP[r * 4 + 3]) > std::fabs(bw)) {
						bw = VP[r * 4 + 3];
						best = r;
					}
				}
				if (std::fabs(bw) > 1e-4f) {
					g_obl.jx = (VP[best * 4 + 0] - VPu[best * 4 + 0]) / bw;
					g_obl.jy = (VP[best * 4 + 1] - VPu[best * 4 + 1]) / bw;
				}
			}
			// the far corner of the (full) frustum where the plane is most on the kept side
			float bestVal = -1e30f;
			for (const float tx : { o.tl, o.tr }) {
				for (const float ty : { o.tb, o.tt }) {
					const NiPoint3 q = o.fwd * o.farD + o.right * (o.farD * tx) + o.up * (o.farD * ty);
					bestVal = (std::max)(bestVal, Dot(o.n, q) + o.dRel);
				}
			}
			if (bestVal <= 1e-3f) {
				return;  // the plane faces away from the whole frustum: leave the matrices alone
			}
			const float k = o.farD / bestVal;
			OblSaved    sv{};
			sv.entry = a_entry;
			float* const mats[4] = { P, VP, VPu, Pu };
			for (int m = 0; m < 4; ++m) {
				for (int r = 0; r < 4; ++r) sv.orig[m * 4 + r] = mats[m][r * 4 + 2];
			}
			float       VPn[16];
			std::memcpy(VPn, VP, sizeof(VPn));
			VPn[0 * 4 + 2] = k * o.n.x;
			VPn[1 * 4 + 2] = k * o.n.y;
			VPn[2 * 4 + 2] = k * o.n.z;
			VPn[3 * 4 + 2] = k * o.dRel;
			float Vi[16], Pn[16];
			if (!Invert4(V, Vi)) {
				return;
			}
			Mul4(Vi, VPn, Pn);  // VP = V * P  ->  P = V^-1 * VP
			for (int r = 0; r < 4; ++r) {
				VP[r * 4 + 2] = VPn[r * 4 + 2];
				VPu[r * 4 + 2] = VPn[r * 4 + 2];
				P[r * 4 + 2] = Pn[r * 4 + 2];
				Pu[r * 4 + 2] = Pn[r * 4 + 2];
			}
			++g_oblApplied;
			g_obl.k = k;
			g_obl.patched = true;
			for (int m = 0; m < 4; ++m) {
				for (int r = 0; r < 4; ++r) sv.patched[m * 4 + r] = mats[m][r * 4 + 2];
			}
			bool replaced = false;
			for (auto& e : g_oblSaved) {
				if (e.entry == a_entry) {
					e = sv;
					replaced = true;
				}
			}
			if (!replaced) {
				g_oblSaved.push_back(sv);
			}
		}

		// view deobl=0|1|2 (G228): the engine's screen-space ambient occlusion of the virtual view reads the depth buffer and assumes the standard projection's depth. The oblique near plane
		// writes a different depth (z' = k (N.p_rel + dRel) / w), so the AO of a portal view is wrong (G227: at identity with a synthetic plane the picture is 17 % darker). 1: before the SSAO calls the depth
		// is converted to the standard encoding (full-screen draw, forward distance recovered from the oblique value and the pixel's ray) and put back afterwards for the blended parts; 2: and the camera
		// entries get their standard depth column back during the SSAO calls.
		std::atomic<int> g_deObl{ 1 };
		std::atomic<int> g_blendStd{ 1 };  // view blendstd=0|1 (G260): the oblique near plane hides the exit wall, but with the patched projection the far blended effects (mist banks, cloud shapes: soft-depth/fade shaders) vanish, so after the opaque dispatch and the SSAO calls the depth stays in the STANDARD encoding (the de-oblique conversion is not undone) and the camera is rebuilt unpatched for the blended parts (FinishAccumulatingPostResolveDepth)
		bool             g_inFinishPhase{ false };
		int              g_finishBuilds{ 0 };
		struct DeoRes
		{
			ComPtr<ID3D11Texture2D>          backup;
			ComPtr<ID3D11ShaderResourceView> srv;
			ComPtr<ID3D11VertexShader>       vs;
			ComPtr<ID3D11PixelShader>        ps;
			ComPtr<ID3D11Buffer>             cb;
			ComPtr<ID3D11DepthStencilState>  ds;
			ComPtr<ID3D11RasterizerState>    rs;
			ComPtr<ID3D11BlendState>         bs;
			UINT                             w{ 0 }, h{ 0 };
			DXGI_FORMAT                      fmt{ DXGI_FORMAT_UNKNOWN };
		};
		DeoRes g_deo;
		bool   g_deoActive{ false };
		bool   g_deoSwapped{ false };
		void OblSwap(bool a_toOriginal)
		{
			for (const auto& e : g_oblSaved) {
				float* const mats[4] = { reinterpret_cast<float*>(e.entry + 0x80), reinterpret_cast<float*>(e.entry + 0xC0), reinterpret_cast<float*>(e.entry + 0x140), reinterpret_cast<float*>(e.entry + 0x1C0) };
				const float* src = a_toOriginal ? e.orig : e.patched;
				for (int m = 0; m < 4; ++m) {
					for (int r = 0; r < 4; ++r) mats[m][r * 4 + 2] = src[m * 4 + r];
				}
			}
		}
		void DeObliqueBegin(ID3D11DeviceContext* a_c)
		{
			g_deoActive = false;
			g_deoSwapped = false;
			if (g_deObl.load() <= 0 || !g_obl.active || !g_obl.patched || !g_liveDsv) {
				return;
			}
			ComPtr<ID3D11Resource> liveRes;
			g_liveDsv->GetResource(&liveRes);
			ComPtr<ID3D11Texture2D> live;
			if (!liveRes || FAILED(liveRes.As(&live))) {
				return;
			}
			auto* srvLive = reinterpret_cast<ID3D11ShaderResourceView*>(RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData().depthStencils[g_params.ds].depthSRV);
			if (!srvLive) {
				return;
			}
			D3D11_TEXTURE2D_DESC d{};
			live->GetDesc(&d);
			D3D11_SHADER_RESOURCE_VIEW_DESC sd{};
			srvLive->GetDesc(&sd);
			auto& R = g_deo;
			if (!R.vs) {
				static const char kVs[] = "float4 main(uint id : SV_VertexID) : SV_Position { float2 p = float2((id << 1) & 2, id & 2); return float4(p * float2(2, -2) + float2(-1, 1), 0, 1); }";
				static const char kPs[] =
					"Texture2D<float> depthTex : register(t0);\n"
					"cbuffer C : register(b0) { float4 a; float4 b; float4 c; float4 d; float4 e; float4 g; };\n"
					"float main(float4 pos : SV_Position) : SV_Depth {\n"
					"  float z = depthTex.Load(int3((int2)pos.xy, 0));\n"
					"  if (z >= 0.999999 || z <= 0.0) return z;\n"
					"  float2 uv = pos.xy / g.xy;\n"
					"  uv.x -= g.z * 0.5; uv.y += g.w * 0.5;\n"  // the sample of this pixel was taken at the unjittered position minus the jitter (G260b: the blended parts flickered because the unjittered ray put far depths 1-2 % off)
					"  float tx = lerp(c.w, d.w, uv.x); float ty = lerp(e.y, e.x, uv.y);\n"
					"  float3 ray = b.xyz + c.xyz * tx + d.xyz * ty;\n"
					"  float den = z / a.w - dot(a.xyz, ray);\n"
					"  float f = b.w / den;\n"
					"  if (!(f > e.z)) return z;\n"
					"  f = min(f * 1.00005, e.w);\n"
					"  return e.w * (f - e.z) / (f * (e.w - e.z));\n"
					"}\n";
				ComPtr<ID3DBlob> bv, bp, err;
				if (FAILED(D3DCompile(kVs, sizeof(kVs) - 1, "deo_vs", nullptr, nullptr, "main", "vs_5_0", 0, 0, &bv, &err)) || FAILED(Device()->CreateVertexShader(bv->GetBufferPointer(), bv->GetBufferSize(), nullptr, &R.vs))) {
					logger::info("view: de-oblique: vertex shader failed");
					g_deObl = 0;
					return;
				}
				if (FAILED(D3DCompile(kPs, sizeof(kPs) - 1, "deo_ps", nullptr, nullptr, "main", "ps_5_0", 0, 0, &bp, &err)) || FAILED(Device()->CreatePixelShader(bp->GetBufferPointer(), bp->GetBufferSize(), nullptr, &R.ps))) {
					logger::info("view: de-oblique: pixel shader failed: {}", err ? static_cast<const char*>(err->GetBufferPointer()) : "?");
					g_deObl = 0;
					return;
				}
				D3D11_BUFFER_DESC cbd{};
				cbd.ByteWidth = 6 * 16;
				cbd.Usage = D3D11_USAGE_DEFAULT;
				cbd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
				Device()->CreateBuffer(&cbd, nullptr, &R.cb);
				D3D11_DEPTH_STENCIL_DESC dd{};
				dd.DepthEnable = TRUE;
				dd.DepthWriteMask = D3D11_DEPTH_WRITE_MASK_ALL;
				dd.DepthFunc = D3D11_COMPARISON_ALWAYS;
				Device()->CreateDepthStencilState(&dd, &R.ds);
				D3D11_RASTERIZER_DESC rd{};
				rd.FillMode = D3D11_FILL_SOLID;
				rd.CullMode = D3D11_CULL_NONE;
				rd.DepthClipEnable = FALSE;
				Device()->CreateRasterizerState(&rd, &R.rs);
				D3D11_BLEND_DESC bd{};
				bd.RenderTarget[0].RenderTargetWriteMask = 0;
				Device()->CreateBlendState(&bd, &R.bs);
			}
			if (!R.backup || R.w != d.Width || R.h != d.Height || R.fmt != d.Format) {
				R.backup.Reset();
				R.srv.Reset();
				D3D11_TEXTURE2D_DESC bd = d;
				bd.Usage = D3D11_USAGE_DEFAULT;
				bd.BindFlags = D3D11_BIND_SHADER_RESOURCE;
				bd.CPUAccessFlags = 0;
				bd.MiscFlags = 0;
				if (FAILED(Device()->CreateTexture2D(&bd, nullptr, &R.backup)) || FAILED(Device()->CreateShaderResourceView(R.backup.Get(), &sd, &R.srv))) {
					logger::info("view: de-oblique: backup texture failed (format {})", static_cast<int>(d.Format));
					R.backup.Reset();
					g_deObl = 0;
					return;
				}
				R.w = d.Width;
				R.h = d.Height;
				R.fmt = d.Format;
				logger::info("view: de-oblique: backup depth texture {}x{} format {} (SRV format {})", d.Width, d.Height, static_cast<int>(d.Format), static_cast<int>(sd.Format));
			}
			a_c->CopyResource(R.backup.Get(), live.Get());
			const Oblique& o = g_obl;
			float          cbv[24] = {
				o.n.x, o.n.y, o.n.z, o.k,
				o.fwd.x, o.fwd.y, o.fwd.z, o.dRel,
				o.right.x, o.right.y, o.right.z, o.tl,
				o.up.x, o.up.y, o.up.z, o.tr,
				o.tb, o.tt, o.nearD, o.farD,
				static_cast<float>(d.Width), static_cast<float>(d.Height), o.jx, o.jy
			};
			a_c->UpdateSubresource(R.cb.Get(), 0, nullptr, cbv, 0, 0);
			// save
			ComPtr<ID3D11VertexShader>      sVs;
			ComPtr<ID3D11PixelShader>       sPs;
			ComPtr<ID3D11InputLayout>       sIl;
			ComPtr<ID3D11RasterizerState>   sRs;
			ComPtr<ID3D11BlendState>        sBs;
			ComPtr<ID3D11DepthStencilState> sDs;
			a_c->VSGetShader(&sVs, nullptr, nullptr);
			a_c->PSGetShader(&sPs, nullptr, nullptr);
			a_c->IAGetInputLayout(&sIl);
			D3D11_PRIMITIVE_TOPOLOGY sTopo{};
			a_c->IAGetPrimitiveTopology(&sTopo);
			ID3D11Buffer*             sCb = nullptr;
			ID3D11ShaderResourceView* sSrv = nullptr;
			a_c->PSGetConstantBuffers(0, 1, &sCb);
			a_c->PSGetShaderResources(0, 1, &sSrv);
			a_c->RSGetState(&sRs);
			FLOAT bf[4]{};
			UINT  sm = 0, sref = 0;
			a_c->OMGetBlendState(&sBs, bf, &sm);
			a_c->OMGetDepthStencilState(&sDs, &sref);
			UINT           nvp = 1;
			D3D11_VIEWPORT svp{};
			a_c->RSGetViewports(&nvp, &svp);
			ID3D11RenderTargetView* sRt[8]{};
			ID3D11DepthStencilView* sDsv = nullptr;
			a_c->OMGetRenderTargets(8, sRt, &sDsv);
			// draw: depth only, no colour
			ID3D11ShaderResourceView* noSrv[1]{};
			a_c->PSSetShaderResources(0, 1, noSrv);
			ID3D11RenderTargetView* none[8]{};
			a_c->OMSetRenderTargets(8, none, g_liveDsv);
			a_c->IASetInputLayout(nullptr);
			a_c->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
			a_c->VSSetShader(R.vs.Get(), nullptr, 0);
			a_c->PSSetShader(R.ps.Get(), nullptr, 0);
			ID3D11Buffer* cbs[1] = { R.cb.Get() };
			a_c->PSSetConstantBuffers(0, 1, cbs);
			ID3D11ShaderResourceView* srvs[1] = { R.srv.Get() };
			a_c->PSSetShaderResources(0, 1, srvs);
			a_c->RSSetState(R.rs.Get());
			a_c->OMSetDepthStencilState(R.ds.Get(), 0);
			const FLOAT zero[4]{};
			a_c->OMSetBlendState(R.bs.Get(), zero, 0xFFFFFFFF);
			D3D11_VIEWPORT vp{ 0.0f, 0.0f, static_cast<float>(d.Width), static_cast<float>(d.Height), 0.0f, 1.0f };
			a_c->RSSetViewports(1, &vp);
			a_c->Draw(3, 0);
			// restore
			a_c->PSSetShaderResources(0, 1, noSrv);
			a_c->OMSetRenderTargets(8, sRt, sDsv);
			a_c->PSSetShaderResources(0, 1, &sSrv);
			a_c->PSSetConstantBuffers(0, 1, &sCb);
			a_c->VSSetShader(sVs.Get(), nullptr, 0);
			a_c->PSSetShader(sPs.Get(), nullptr, 0);
			a_c->IASetInputLayout(sIl.Get());
			a_c->IASetPrimitiveTopology(sTopo);
			a_c->RSSetState(sRs.Get());
			a_c->OMSetBlendState(sBs.Get(), bf, sm);
			a_c->OMSetDepthStencilState(sDs.Get(), sref);
			a_c->RSSetViewports(nvp, &svp);
			if (sCb) sCb->Release();
			if (sSrv) sSrv->Release();
			for (auto* r : sRt) { if (r) r->Release(); }
			if (sDsv) sDsv->Release();
			g_deoActive = true;
			if (g_deObl.load() >= 2) {
				g_obl.active = false;  // Hook_Build must not patch again while the SSAO calls rebuild the camera
				OblSwap(true);
				g_deoSwapped = true;
			}
			static int s_logged = 0;
			if (s_logged++ < 3) {
				logger::info("view: de-oblique: depth converted to the standard encoding for the SSAO (level {}, k {:.4f}, dRel {:.2f}, {} camera entries)", g_deObl.load(), o.k, o.dRel, g_oblSaved.size());
			}
		}
		// view vshadow=1 (G235): the sun's shadow cascades are fitted to the world camera's frustum once per frame, so a portal view looking elsewhere has no shadow-map coverage (G233). The engine's own
		// frame does, in order: the shadow stage (id 107132: UpdateCamera of the sun light with the world camera, the caster cull, Accumulate), later the render stage (id 107133: Render of the lights draws the
		// shadow maps), the main draw, and the mask stage (id 107140: the screen-space mask per light, ReturnShadowmaps after each). Both stages read the camera from the scene graph's camera slot (+0x128 of the
		// object at id 415032); here they run for the virtual camera with that slot swapped, before the virtual pass; the engine runs them again for the real camera later in the frame.
		// ---- G260 view shlog=N: the state of the sun's shadow light after a stage, for N frames: the cascade split distances, the sun vector and every shadow-map descriptor (camera frustum, pose, map index,
		// depth target, port, texel size, enabled), ours (after our 107132 + 107133 for the virtual camera) against the engine's own (after its stage and after the sun light's Render) ----
		std::atomic<int>  g_sunLogLeft{ 0 };
		void* SunShadowLight()
		{
			std::uint8_t* node = reinterpret_cast<std::uint8_t*>(SceneStateNode());
			std::uint64_t light = 0;
			if (node) ReadSafeT(node + 0x210, light);
			return reinterpret_cast<void*>(light);
		}
		// ---- G272: the engine's volumetric lighting build (id 107023, called once per frame from the frame function before the world draw with (the light manager, the world camera, a flag)): it sets the
		// ISApplyVolumetricLighting effect inactive when the global disable byte (id 415052) is set. view vloff=1 (TEST ONLY) forces that byte for the call, view vllog=N logs the next N calls ----
		// g_vlOff (view vloff=0|1|2) is declared with g_vlRun above
		std::atomic<int> g_vlLog{ 0 };
		std::atomic<int> g_vlLogLeft2{ 0 };
		using VlBuild_t = void (*)(void*, void*, std::uint8_t);
		VlBuild_t         g_origVlBuild{ nullptr };
		const hooks::Spec kVlBuild{ "volumetric lighting build (id 107023)", 107023, { 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x10, 0x57, 0x48, 0x83, 0xEC, 0x40, 0x48 } };
		void Hook_VlBuild(void* a_mgr, void* a_cam, std::uint8_t a_flag)
		{
			auto* const         dis = reinterpret_cast<std::uint8_t*>(REL::ID(415052).address());
			const std::uint8_t  was = *dis;
			// vloff=1: every build (the real frame's and ours), vloff=2 (G304): only the real frame's, so the portal pictures keep their own VL while the engine's real VL steps are skipped
			if (g_vlOff.load() == 1 || (g_vlOff.load() == 2 && a_cam == static_cast<void*>(RE::Main::WorldRootCamera()))) {
				*dis = 1;
			}
			if (g_vlLog.load() > 0) {
				--g_vlLog;
				logger::info("vl: build call, manager {:p} camera {:p} (world camera {:p}, ours {:p}) flag {} disable byte was {} now {} vloff {} thread {} frame {}", a_mgr, a_cam, static_cast<void*>(RE::Main::WorldRootCamera()), static_cast<void*>(g_cam), a_flag, was, *dis, g_vlOff.load(), GetCurrentThreadId(), frame::Count());
			}
			if (g_vlLogLeft2.load() > 0) {
				--g_vlLogLeft2;
				std::string f;
				for (int o = 0x30; o <= 0x90; o += 8) {
					std::uint64_t v = 0;
					ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_mgr) + o, v);
					f += std::format(" +{:#x}={:#x}", o, v);
				}
				logger::info("vl: manager fields in the ENGINE's own build call (frame {}):{}", frame::Count(), f);
			}
			g_origVlBuild(a_mgr, a_cam, a_flag);
			*dis = was;
		}
		// ---- G305: the engine's own VL composite (id 107213, once per real frame in the post function, after the overlay): hooked only to dump the frame after it (hdrdump) ----
		using VlComposite_t = std::int32_t (*)(void*, std::int32_t, std::int32_t);
		VlComposite_t g_origVlComposite{ nullptr };
		const hooks::Spec kVlComposite{ "volumetric lighting composite (id 107213)", 107213, { 0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x7C, 0x24, 0x10, 0x55, 0x48, 0x8B, 0xEC, 0x48, 0x83 } };
		std::int32_t Hook_VlComposite(void* a_obj, std::int32_t a_rt, std::int32_t a_rt2)
		{
			if (g_inVirtualComposite.load() == 0) {
				diag::SamplePreVl();
				overlay::VlAddBegin();
			}
			const auto r = g_origVlComposite(a_obj, a_rt, a_rt2);
			if (g_inVirtualComposite.load() == 0) {
				static int s_hookLogged = 0;
				if (s_hookLogged++ < 2) {
					logger::info("vl: the engine's real VL composite returned (render targets {} {}); the virtual VL is added now (vladd {})", a_rt, a_rt2, g_vlAdd.load());
				}
				overlay::DrawVlAdd();
				overlay::DrawMoments();
				diag::SamplePostVl();
			}
			return r;
		}
		void VlHookEnable()
		{
			static bool tried = false;
			if (!tried) {
				tried = true;
				const bool ok = hooks::Install(kVlBuild, reinterpret_cast<void*>(&Hook_VlBuild), reinterpret_cast<void**>(&g_origVlBuild));
				logger::info("view: volumetric lighting build hook {}", ok ? "installed" : "FAILED");
			}
		}
		// ---- G260c view cullog=N: BSCullingProcess::Process (id 106921) contexts of the next N calls, with the camera classified (world camera, ours, a sun cascade camera): accumulator, camera, scene, flags and
		// the bytes the engine's shadow caster cull sets (ctx +0x28/+0x38/+0x50/+0x58/+0x5D/+0x5E/+0x5F/+0x60) ----
		std::atomic<int> g_cullLogLeft{ 0 };
		using CullProcess_t = void (*)(void*, std::int32_t, std::int32_t);
		CullProcess_t     g_origCullProcess{ nullptr };
		const hooks::Spec kCullProcess{ "BSCullingProcess::Process", 106921, { 0x48, 0x8B, 0xC4, 0x57, 0x41, 0x54, 0x41, 0x55, 0x41, 0x56, 0x41, 0x57, 0x48, 0x83, 0xEC, 0x40 } };
		void Hook_CullProcess(void* a_ctx, std::int32_t a_a, std::int32_t a_b)
		{
			if (g_cullLogLeft.load() > 0 && g_cullLogLeft.fetch_sub(1) > 0) {
				const auto* c = reinterpret_cast<const std::uint8_t*>(a_ctx);
				std::uint64_t acc = 0, cam = 0, f28 = 0, p38 = 0, scene = 0, f48 = 0;
				std::uint32_t flags = 0, f58 = 0;
				std::uint8_t  b5d = 0, b5e = 0, b5f = 0, b60 = 0;
				ReadSafeT(c + 0x00, acc); ReadSafeT(c + 0x10, cam); ReadSafeT(c + 0x28, f28); ReadSafeT(c + 0x38, p38); ReadSafeT(c + 0x40, scene); ReadSafeT(c + 0x48, f48);
				ReadSafeT(c + 0x50, flags); ReadSafeT(c + 0x58, f58); ReadSafeT(c + 0x5D, b5d); ReadSafeT(c + 0x5E, b5e); ReadSafeT(c + 0x5F, b5f); ReadSafeT(c + 0x60, b60);
				std::string who = "other";
				if (reinterpret_cast<void*>(cam) == static_cast<void*>(RE::Main::WorldRootCamera())) who = "WORLD camera";
				else if (g_cam && reinterpret_cast<void*>(cam) == static_cast<void*>(g_cam)) who = "OURS (virtual camera)";
				else if (void* lp = SunShadowLight()) {
					auto& sr = reinterpret_cast<RE::BSShadowDirectionalLight*>(lp)->GetRuntimeData();
					int   i = 0;
					for (auto& d : sr.shadowmapDescriptors) {
						if (d.camera.get() == reinterpret_cast<RE::NiCamera*>(cam)) who = std::format("sun cascade {}", i);
						++i;
					}
				}
				std::uint32_t amode = 0;
				if (acc) ReadSafeT(reinterpret_cast<const std::uint8_t*>(acc) + 0x150, amode);
				logger::info("cullog: thread {} {} ctx acc {:#x} (mode {}) cam {:#x} +28 {:#x} planes +38 {:#x} scene {:#x} +48 {:#x} flags {:#x} +58 {} +5D {} +5E {} +5F {} +60 {} args ({}, {})", GetCurrentThreadId(), who, acc, amode, cam, f28, p38, scene, f48, flags, f58, b5d, b5e, b5f, b60, a_a, a_b);
			}
			g_origCullProcess(a_ctx, a_a, a_b);
		}
		void CullLogEnable(int a_n)
		{
			static bool tried = false;
			if (!tried) {
				tried = true;
				const bool ok = hooks::Install(kCullProcess, reinterpret_cast<void*>(&Hook_CullProcess), reinterpret_cast<void**>(&g_origCullProcess));
				logger::info("view: BSCullingProcess::Process log hook {}", ok ? "installed" : "FAILED");
				if (!ok) return;
			}
			g_cullLogLeft = a_n;
		}
		void SunShadowLogImpl(const char* a_label)
		{
			if (!g_sunLogOn.load()) {
				return;
			}
			void* lp = SunShadowLight();
			if (!lp) {
				logger::info("sunlog: {}: no sun shadow light", a_label);
				return;
			}
			auto*       L = reinterpret_cast<RE::BSShadowDirectionalLight*>(lp);
			const auto& dr = L->GetShadowDirectionalLightRuntimeData();
			auto&       sr = L->GetRuntimeData();
			logger::info("sunlog: {}: light {:p} sunVector ({:.3f}, {:.3f}, {:.3f}) last ({:.3f}, {:.3f}, {:.3f}) splits start ({:.1f}, {:.1f}, {:.1f}) end ({:.1f}, {:.1f}, {:.1f}) sunUpdate {:.2f} mode {} maskIndex {} accumulatedIndex {} descriptors {} drawFocusShadows {}", a_label, lp, dr.sunVector.x, dr.sunVector.y, dr.sunVector.z, dr.lastSunVector.x, dr.lastSunVector.y, dr.lastSunVector.z,
				dr.startSplitDistances[0], dr.startSplitDistances[1], dr.startSplitDistances[2], dr.endSplitDistances[0], dr.endSplitDistances[1], dr.endSplitDistances[2], dr.sunUpdate, dr.sunUpdateMode, sr.maskIndex, sr.accumulatedIndex, sr.shadowmapDescriptors.size(), sr.drawFocusShadows);
			int i = 0;
			for (auto& d : sr.shadowmapDescriptors) {
				if (i >= 8) break;
				std::string cam = "no camera";
				if (auto* c = d.camera.get()) {
					const auto& fr = c->GetRuntimeData2().viewFrustum;
					cam = std::format("frustum l {:.1f} r {:.1f} t {:.1f} b {:.1f} n {:.1f} f {:.1f} ortho {}, pos ({:.0f}, {:.0f}, {:.0f}), fwd ({:.3f}, {:.3f}, {:.3f})", fr.fLeft, fr.fRight, fr.fTop, fr.fBottom, fr.fNear, fr.fFar, fr.bOrtho, c->world.translate.x, c->world.translate.y, c->world.translate.z, c->world.rotate.entry[0][0], c->world.rotate.entry[1][0], c->world.rotate.entry[2][0]);
				}
				logger::info("sunlog:   descriptor {}: enabled {} map index {} depth target {} cullingMode {} unitsPerTexel {} port (words {}, {}, {}, {}) clear {} | {}", i, d.isEnabled, d.shadowmapIndex, static_cast<int>(d.renderTarget), static_cast<int>(d.cullingMode), d.unitsPerTexel, reinterpret_cast<const std::int32_t*>(&d.port)[0], reinterpret_cast<const std::int32_t*>(&d.port)[1], reinterpret_cast<const std::int32_t*>(&d.port)[2], reinterpret_cast<const std::int32_t*>(&d.port)[3], d.clearRenderTarget, cam);
				{
					const auto* acc = reinterpret_cast<const std::uint8_t*>(d.shaderAccumulator.get());
					const auto* cp = reinterpret_cast<const std::uint8_t*>(d.cullingProcess);
					std::uint32_t amode = 0, a160 = 0, a164 = 0, a168 = 0;
					std::uint8_t  aeq = 0, a12e = 0, c11e = 0, c11f = 0;
					std::uint64_t c30190 = 0;
					if (acc) {
						ReadSafeT(acc + 0x150, amode); ReadSafeT(acc + 0x5C, aeq); ReadSafeT(acc + 0x160, a160); ReadSafeT(acc + 0x164, a164); ReadSafeT(acc + 0x168, a168); ReadSafeT(acc + 0x12E, a12e);
					}
					if (cp) {
						ReadSafeT(cp + 0x11E, c11e); ReadSafeT(cp + 0x11F, c11f); ReadSafeT(cp + 0x30190, c30190);
					}
					std::uint8_t l47 = 0;
					ReadSafeT(reinterpret_cast<const std::uint8_t*>(lp) + 0x47, l47);
					logger::info("sunlog:     accumulator {:p} render mode {} equalDepth {} +160 {} +164 {:#x} +168 {} +12E {} | cullingProcess {:p} +11E {} +11F {} +30190 {:#x} | light +47 {}", static_cast<const void*>(acc), amode, aeq, a160, a164, a168, a12e, static_cast<const void*>(cp), c11e, c11f, c30190, l47);
					{
						const auto* lt = reinterpret_cast<const float*>(&d.lightTransform);
						std::string cr;
						if (auto* c = d.camera.get()) {
							cr = std::format("camera rotate rows ({:.6f} {:.6f} {:.6f}) ({:.6f} {:.6f} {:.6f}) ({:.6f} {:.6f} {:.6f}) translate ({:.4f} {:.4f} {:.4f})", c->world.rotate.entry[0][0], c->world.rotate.entry[0][1], c->world.rotate.entry[0][2], c->world.rotate.entry[1][0], c->world.rotate.entry[1][1], c->world.rotate.entry[1][2], c->world.rotate.entry[2][0], c->world.rotate.entry[2][1], c->world.rotate.entry[2][2], c->world.translate.x, c->world.translate.y, c->world.translate.z);
						}
						logger::info("sunlog:     lightTransform rows ({:.6f} {:.6f} {:.6f} {:.6f}) ({:.6f} {:.6f} {:.6f} {:.6f}) ({:.6f} {:.6f} {:.6f} {:.6f}) ({:.6f} {:.6f} {:.6f} {:.6f}) | {}", lt[0], lt[1], lt[2], lt[3], lt[4], lt[5], lt[6], lt[7], lt[8], lt[9], lt[10], lt[11], lt[12], lt[13], lt[14], lt[15], cr);
					}
				}
				++i;
			}
		}
		// G264: the shader's cascade rows from the descriptors of the cascades we just rendered for the virtual camera. G263 data (the engine's own frame): row j of the engine's cb2 (16 + 3 i + j) =
		// (M[0][j], M[1][j], M[2][j], M[3][j] + M[0][j] Px + M[1][j] Py + M[2][j] Pz) for the descriptor's lightTransform M (row-vector convention, rows u, v, depth) and P = the camera position the shader's
		// positions are relative to (the main camera: the replay's rows 32-35 are made relative to it); the biases are fractions of the cascade's depth range, kept constant in world units
		bool BuildCascadeRows(float* a_out)
		{
			void* lp = SunShadowLight();
			auto* mcam = RE::Main::WorldRootCamera();
			if (!lp || !mcam) {
				return false;
			}
			const auto& P = mcam->world.translate;
			auto&       sr = reinterpret_cast<RE::BSShadowDirectionalLight*>(lp)->GetRuntimeData();
			int         i = 0;
			for (auto& d : sr.shadowmapDescriptors) {
				if (i >= 2) {
					break;
				}
				const auto* M = reinterpret_cast<const float*>(&d.lightTransform);
				for (int j = 0; j < 3; ++j) {
					float* r = a_out + (3 * i + j) * 4;
					r[0] = M[0 * 4 + j];
					r[1] = M[1 * 4 + j];
					r[2] = M[2 * 4 + j];
					r[3] = M[3 * 4 + j] + M[0 * 4 + j] * P.x + M[1 * 4 + j] * P.y + M[2 * 4 + j] * P.z;
				}
				float bias = 0.0045f + 0.0005f * i;
				if (g_biasUnitsValid) {
					if (auto* c = d.camera.get()) {
						const auto& fr = c->GetRuntimeData2().viewFrustum;
						const float range = fr.fFar - fr.fNear;
						if (range > 1.0f) {
							bias = g_biasUnits[i] / range;
						}
					}
				}
				a_out[36 + i] = bias;
				++i;
			}
			if (i < 2) {
				return false;
			}
			a_out[38] = a_out[37];  // the third cascade's bias slot (unused: no third cascade)
			// the third cascade's rows stay zero
			static int s_logged = 0;
			if (s_logged++ < 3) {
				logger::info("view: portal sun shadows: cascade rows for the virtual camera: c0 u ({:.6f} {:.6f} {:.6f} {:.4f}) depth ({:.6f} {:.6f} {:.6f} {:.4f}) bias {:.5f}; c1 depth ({:.6f} {:.6f} {:.6f} {:.4f}) bias {:.5f}", a_out[0], a_out[1], a_out[2], a_out[3], a_out[8], a_out[9], a_out[10], a_out[11], a_out[36], a_out[20], a_out[21], a_out[22], a_out[23], a_out[37]);
			}
			return true;
		}
		bool   g_vShadowActive{ false };
		std::atomic<bool> g_vsTrace{ false };  // view vstrace=1: breadcrumb lines around every step of the virtual shadow sequence (a hang then shows where)
		void   VsCrumb(const char* a_what)
		{
			if (g_vsTrace.load()) {
				logger::info("vstrace: frame {} {}", frame::Count(), a_what);
			}
		}
		double g_vShadowMsStage{ 0 }, g_vShadowMsRender{ 0 };
		void** g_vsSlot{ nullptr };  // the scene graph's camera slot while it is swapped (restored by __finally, and by the exception handler of the view pass)
		void*  g_vsSaved{ nullptr };
		double VsNow() { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
		// no C++ objects in here: __try/__finally
		// view vsfence=1 (G237): a GPU event query after every step of the virtual shadow sequence, polled for up to 400 ms: logs the GPU time of the step, or that the GPU did not finish it (G235/G236: the game then
		// hangs in the driver at the first Map(DISCARD) of the engine's main colour pass; the stack is in evidence/s7/Riftweaver_g236.log). A step that does not finish aborts the sequence (vshadow = 0).
		std::atomic<bool>        g_vsFence{ false };
		std::atomic<int>         g_vsMode{ 16 | 64 | 128 | 4 | 256 };  // G265 default = what `view portalshadows=1` sets (UpdateCamera + own caster cull + Render + ReturnShadowmaps + our cascades in the mask replay); view vsmode=<bits> (G238 bisect): 1 = run the shadow stage 107132, 2 = the render stage 107133, 4 = ReturnShadowmaps at the end, 8 = the engine's mask stage 107141 (else the mask replay)
		ComPtr<ID3D11Query>      g_vsQuery;
		bool VsFence(const char* a_label)
		{
			if (!g_vsFence.load()) {
				return true;
			}
			ComPtr<ID3D11DeviceContext> c;
			Device()->GetImmediateContext(&c);
			if (!g_vsQuery) {
				D3D11_QUERY_DESC qd{};
				qd.Query = D3D11_QUERY_EVENT;
				if (FAILED(Device()->CreateQuery(&qd, &g_vsQuery))) {
					logger::info("vstrace: fence query could not be created");
					g_vsFence = false;
					return true;
				}
			}
			const double t0 = VsNow();
			c->End(g_vsQuery.Get());
			c->Flush();
			BOOL done = FALSE;
			while (VsNow() - t0 < 400.0) {
				if (c->GetData(g_vsQuery.Get(), &done, sizeof(done), 0) == S_OK && done) {
					break;
				}
				Sleep(0);
			}
			const double ms = VsNow() - t0;
			if (!done) {
				logger::error("vstrace: frame {} FENCE after '{}': the GPU did NOT finish within {:.0f} ms - aborting the virtual shadow sequence (vshadow=0)", frame::Count(), a_label, ms);
				g_params.vShadow = 0;
				return false;
			}
			logger::info("vstrace: frame {} fence after '{}': GPU finished, waited {:.3f} ms", frame::Count(), a_label, ms);
			return true;
		}
		void FenceMid() { VsFence("shadow stage 107132"); }
		// G260d: the sun light's Accumulate (slot 9, id 108488) does per cascade descriptor: acc+0x160 = count+1, acc+0x164 = 1 << count, acc+0x168 = index, acc+0x12E = 1, 106596(acc) (clear), then 107602 builds a
		// BSCullingProcess context (acc, camera, planes of the descriptor's culling process) and calls Process(ctx, 2, 0) once per entry of the light's culling-process array = it REPLAYS the caster lists that the
		// job-based caster cull (108492, inside stage 107132) collected for the real camera (G261 cullog: scene 0 in those contexts), so called alone it accumulates stale real-camera casters. This does the same
		// per-descriptor work with a real scene cull for the descriptor's camera (Process(ctx, 0, 0) with the scene root, as our own virtual pass does): the ctx bytes are the ones the engine logged for its cascades
		// (flags 3 (+1 if light byte +0x47), +0x58 = 11 + index, +0x5D = 1 directional, +0x5E/+0x5F from the culling process +0x11E/+0x11F, planes = culling process +0xAC, +0x28 = culling process +0x30190)
		void AccumulateOwn(void* a_light)
		{
			auto*         L = reinterpret_cast<RE::BSShadowDirectionalLight*>(a_light);
			auto&         sr = L->GetRuntimeData();
			void*         scene = SceneStateNode();
			std::uint8_t  l47 = 0;
			ReadSafeT(reinterpret_cast<const std::uint8_t*>(a_light) + 0x47, l47);
			std::uint32_t count = 0;
			std::uint32_t i = 0;
			for (auto& d : sr.shadowmapDescriptors) {
				auto* acc = reinterpret_cast<std::uint8_t*>(d.shaderAccumulator.get());
				auto* cam = d.camera.get();
				auto* cp = reinterpret_cast<std::uint8_t*>(d.cullingProcess);
				if (!acc || !cam) {
					++i;
					continue;
				}
				*reinterpret_cast<std::uint32_t*>(acc + 0x160) = count + 1;
				*reinterpret_cast<std::uint32_t*>(acc + 0x164) = (count <= 0x1e) ? (1u << count) : 0u;
				*reinterpret_cast<std::uint32_t*>(acc + 0x168) = i;
				acc[0x12E] = 1;
				reinterpret_cast<void (*)(void*)>(REL::ID(106596).address())(acc);
				alignas(16) std::uint8_t ctx[0x68]{};
				reinterpret_cast<CtxCtor_t>(REL::ID(106919).address())(ctx);
				*reinterpret_cast<void**>(ctx + 0x00) = acc;
				*reinterpret_cast<void**>(ctx + 0x10) = cam;
				std::uint64_t f28 = 0;
				std::uint8_t  c11e = 0, c11f = 0;
				if (cp) {
					ReadSafeT(cp + 0x30190, f28);
					ReadSafeT(cp + 0x11E, c11e);
					ReadSafeT(cp + 0x11F, c11f);
				}
				*reinterpret_cast<std::uint64_t*>(ctx + 0x28) = f28;
				if (c11f) {
					*reinterpret_cast<void**>(ctx + 0x38) = cp + 0xAC;
				}
				*reinterpret_cast<void**>(ctx + 0x40) = scene;
				*reinterpret_cast<std::uint32_t*>(ctx + 0x50) = 3u + (l47 != 0 ? 1u : 0u);
				*reinterpret_cast<std::uint32_t*>(ctx + 0x58) = 11u + i;
				ctx[0x5D] = 1;
				ctx[0x5E] = c11e;
				ctx[0x5F] = c11f ? 1 : 0;
				ctx[0x60] = 0;
				reinterpret_cast<Cull_t>(REL::ID(106921).address())(ctx, 0, 0);
				++count;
				++i;
			}
		}
		void RunShadowStages(void* a_cam, double* a_t)
		{
			__try {
				a_t[0] = VsNow();
				if (g_vsMode.load() & 1) reinterpret_cast<void (*)(void*)>(REL::ID(107132).address())(a_cam);
				// G260c: the stage's pieces called one by one through the light's own virtual slots (no job-based culls, no stage bookkeeping): 16 = UpdateCamera (slot 0x10, fits the cascades to
				// the camera handed in), 32 = Accumulate (slot 9: per cascade the accumulator is cleared and the scene is culled into it with the cascade camera), 64 = Render (slot 0xA: per cascade
				// the engine's descriptor renderer 107604 draws the shadow map)
				if (g_vsMode.load() & (16 | 32 | 64 | 128)) {
					if (void* light = SunShadowLight()) {
						auto**        vt = *reinterpret_cast<void***>(light);
						std::uint32_t count = 0;
						std::uint32_t index = 0;
						const auto    maskCh = static_cast<std::uint32_t>(reinterpret_cast<RE::BSShadowDirectionalLight*>(light)->GetRuntimeData().maskIndex);
						if (g_vsMode.load() & 16) {
							reinterpret_cast<std::uint8_t (*)(void*, void*)>(vt[0x10])(light, a_cam);
							VsCrumb("slot UpdateCamera (0x10) returned");
							SunShadowLogImpl("OURS: after UpdateCamera(virtual camera)");
							VsFence("slot UpdateCamera");
						}
						if (g_vsMode.load() & 32) {
							reinterpret_cast<void (*)(void*, std::uint32_t*, std::uint32_t, void*, std::uint8_t)>(vt[9])(light, &count, maskCh, SceneStateNode(), 0);
							VsCrumb("slot Accumulate (9) returned");
							SunShadowLogImpl("OURS: after Accumulate");
							VsFence("slot Accumulate");
						}
						if (g_vsMode.load() & 128) {
							AccumulateOwn(light);
							VsCrumb("own caster cull (AccumulateOwn) returned");
							SunShadowLogImpl("OURS: after the own caster cull");
							VsFence("own caster cull");
						}
						if (g_vsMode.load() & 64) {
							reinterpret_cast<void (*)(void*, std::uint32_t*)>(vt[0xA])(light, &index);
							VsCrumb("slot Render (0xA) returned");
							VsFence("slot Render");
						}
					}
				}
				a_t[1] = VsNow();
				FenceMid();
				if (g_vsMode.load() & 2) {
					reinterpret_cast<void (*)(void*, void*, std::uint8_t)>(REL::ID(77503).address())(reinterpret_cast<void*>(REL::ID(411479).address()), a_cam, 1);
					reinterpret_cast<void (*)()>(REL::ID(107133).address())();
				}
				a_t[2] = VsNow();
			} __finally {
				if (g_vsSlot) {
					*g_vsSlot = g_vsSaved;
					g_vsSlot = nullptr;
				}
			}
		}
		void VirtualShadowsAbort();
		bool VirtualShadowsBegin(void* a_cam)
		{
			g_vShadowActive = false;
			if (g_params.vShadow >= 2) {
				static std::uint64_t s_lastFrame = 0;
				if (frame::Count() == s_lastFrame) {
					VsCrumb("Begin skipped (vshadow=2: only the first virtual view of a frame)");
					return false;
				}
				s_lastFrame = frame::Count();
			}
			VsCrumb("Begin");
			std::uint8_t* sg = nullptr;
			ReadSafeT(reinterpret_cast<const void*>(REL::ID(415032).address()), sg);
			if (!sg || !SunShadowLight()) {
				return false;
			}
			g_vsSlot = reinterpret_cast<void**>(sg + 0x128);
			g_vsSaved = *g_vsSlot;
			*g_vsSlot = a_cam;
			double t[3]{};
			VsCrumb("stages: entering 107132 (shadow stage), then 77503, then 107133 (render stage)");
			RunShadowStages(a_cam, t);
			VsCrumb("stages: returned, camera slot restored");
			if (!VsFence("render stage 107133 (shadow maps drawn)")) {
				VirtualShadowsAbort();
				return false;
			}
			g_vShadowMsStage = t[1] - t[0];
			g_vShadowMsRender = t[2] - t[1];
			SunShadowLogImpl("OURS: after our 107132 + 107133 for the virtual camera");
			g_vShadowActive = true;
			static int s_logged = 0;
			if (s_logged++ < 6) {
				logger::info("view: virtual shadows: stage 107132 {:.3f} ms, render stage 107133 {:.3f} ms (CPU), sun light {:p}", g_vShadowMsStage, g_vShadowMsRender, SunShadowLight());
			}
			return true;
		}
		void VirtualShadowsAbort()
		{
			if (g_vsSlot) {
				*g_vsSlot = g_vsSaved;
				g_vsSlot = nullptr;
			}
			g_vShadowActive = false;
		}
		void VirtualShadowsEnd()
		{
			if (!g_vShadowActive) {
				return;
			}
			g_vShadowActive = false;
			VsCrumb("End: ReturnShadowmaps");
			if (void* light = (g_vsMode.load() & 4) ? SunShadowLight() : nullptr) {
				auto** vt = *reinterpret_cast<void***>(light);
				reinterpret_cast<void (*)(void*)>(vt[0xC])(light);  // ReturnShadowmaps: the maps go back to the pool before the engine renders its own
				VsFence("ReturnShadowmaps");
			}
		}
		// view ssaoown=0|1 (G246, default ON): the colour error of the virtual picture at identity comes from the engine's SSAO APPLY step (G244: with SSAO off the virtual/real ratio is a uniform 1.05, with it roof
		// (1.04,1.15,1.27) and ground (0.90,0.93,0.97)); the engine's AO MAP is the same as the real frame's (grey, mean 0.949 against 0.945, G245) and the real result equals the colour before the AO times that map
		// (virtual no-AO colour x AO map against the real frame: 1.00-1.04 in every channel). So the virtual pass copies the colour target before the engine's SSAO calls and afterwards writes colour x AO (render target 46, red)
		// itself, discarding the engine's apply result.
		std::atomic<int> g_ssaoOwn{ 0 };  // 0 = the engine's apply as is, 1 = colour x AO map (G246, bands), 2 = the engine's apply with its tint taken out (G247: the engine's luminance change applied to the colour before the apply)  // default OFF since G246: the own AO apply shows dark bands and foliage speckles in portal views (horrible)
		struct AoRes
		{
			ComPtr<ID3D11Texture2D>          backup, backup2;
			ComPtr<ID3D11ShaderResourceView> srv, srv2;
			ComPtr<ID3D11Buffer>             cb;
			ComPtr<ID3D11VertexShader>       vs;
			ComPtr<ID3D11PixelShader>        ps;
			ComPtr<ID3D11DepthStencilState>  ds;
			ComPtr<ID3D11RasterizerState>    rs;
			ComPtr<ID3D11BlendState>         bs;
			UINT                             w{ 0 }, h{ 0 };
			DXGI_FORMAT                      fmt{ DXGI_FORMAT_UNKNOWN };
		};
		AoRes g_ao;
		bool  g_aoBackedUp{ false };
		void AoOwnBegin(ID3D11DeviceContext* a_c)
		{
			g_aoBackedUp = false;
			if (g_ssaoOwn.load() == 0) {
				return;
			}
			auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			auto* mainTex = rd ? reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[RE::RENDER_TARGETS::kMAIN].texture) : nullptr;
			if (!mainTex) {
				return;
			}
			D3D11_TEXTURE2D_DESC d{};
			mainTex->GetDesc(&d);
			auto& R = g_ao;
			if (!R.backup || R.w != d.Width || R.h != d.Height || R.fmt != d.Format) {
				R.backup.Reset();
				R.srv.Reset();
				D3D11_TEXTURE2D_DESC bd = d;
				bd.Usage = D3D11_USAGE_DEFAULT;
				bd.BindFlags = D3D11_BIND_SHADER_RESOURCE;
				bd.CPUAccessFlags = 0;
				bd.MiscFlags = 0;
				bd.MipLevels = 1;
				if (FAILED(Device()->CreateTexture2D(&bd, nullptr, &R.backup)) || FAILED(Device()->CreateShaderResourceView(R.backup.Get(), nullptr, &R.srv)) || FAILED(Device()->CreateTexture2D(&bd, nullptr, &R.backup2)) || FAILED(Device()->CreateShaderResourceView(R.backup2.Get(), nullptr, &R.srv2))) {
					logger::info("view: ssaoown: backup texture failed (format {})", static_cast<int>(d.Format));
					R.backup.Reset();
					g_ssaoOwn = 0;
					return;
				}
				R.w = d.Width;
				R.h = d.Height;
				R.fmt = d.Format;
			}
			a_c->CopyResource(R.backup.Get(), mainTex);
			g_aoBackedUp = true;
		}
		void AoOwnEnd(ID3D11DeviceContext* a_c)
		{
			if (!g_aoBackedUp) {
				return;
			}
			g_aoBackedUp = false;
			auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			auto* mainRtv = rd ? reinterpret_cast<ID3D11RenderTargetView*>(rd->renderTargets[RE::RENDER_TARGETS::kMAIN].RTV) : nullptr;
			auto* aoSrv = rd ? reinterpret_cast<ID3D11ShaderResourceView*>(rd->renderTargets[46].SRV) : nullptr;
			auto& R = g_ao;
			if (!mainRtv || !aoSrv || !R.backup) {
				return;
			}
			const int mode = g_ssaoOwn.load();
			if (mode == 2) {
				a_c->CopyResource(R.backup2.Get(), reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[RE::RENDER_TARGETS::kMAIN].texture));  // the engine's apply result
			}
			if (!R.vs) {
				static const char kVs[] = "float4 main(uint id : SV_VertexID) : SV_Position { float2 p = float2((id << 1) & 2, id & 2); return float4(p * float2(2, -2) + float2(-1, 1), 0, 1); }";
				static const char kPs[] =
					"Texture2D<float4> col : register(t0);\n"
					"Texture2D<float4> ao : register(t1);\n"
					"Texture2D<float4> eng : register(t2);\n"
					"cbuffer M : register(b0) { float4 mode; };\n"
					"float4 main(float4 pos : SV_Position) : SV_Target {\n"
					"  int3 p = int3((int2)pos.xy, 0);\n"
					"  float4 c = col.Load(p);\n"
					"  if (mode.x < 1.5) { float a = ao.Load(p).r; return float4(c.rgb * a, c.a); }\n"
					"  float4 e = eng.Load(p);\n"
					"  float lc = dot(c.rgb, float3(0.2126, 0.7152, 0.0722));\n"
					"  float le = dot(e.rgb, float3(0.2126, 0.7152, 0.0722));\n"
					"  float f = (lc > 1e-4) ? clamp(le / lc, 0.0, 1.0) : 1.0;\n"
					"  return float4(c.rgb * f, e.a);\n"  // the engine's own alpha: the blended parts after the SSAO step read the destination alpha (G247: keeping the pre-apply alpha produced dark speckles on distant trees)
					"}\n";
				ComPtr<ID3DBlob> bv, bp, err;
				if (FAILED(D3DCompile(kVs, sizeof(kVs) - 1, "ao_vs", nullptr, nullptr, "main", "vs_5_0", 0, 0, &bv, &err)) || FAILED(Device()->CreateVertexShader(bv->GetBufferPointer(), bv->GetBufferSize(), nullptr, &R.vs)) ||
					FAILED(D3DCompile(kPs, sizeof(kPs) - 1, "ao_ps", nullptr, nullptr, "main", "ps_5_0", 0, 0, &bp, &err)) || FAILED(Device()->CreatePixelShader(bp->GetBufferPointer(), bp->GetBufferSize(), nullptr, &R.ps))) {
					logger::info("view: ssaoown: shaders failed: {}", err ? static_cast<const char*>(err->GetBufferPointer()) : "?");
					g_ssaoOwn = 0;
					return;
				}
				D3D11_BUFFER_DESC cbd{};
				cbd.ByteWidth = 16;
				cbd.Usage = D3D11_USAGE_DEFAULT;
				cbd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
				Device()->CreateBuffer(&cbd, nullptr, &R.cb);
				D3D11_DEPTH_STENCIL_DESC dd{};
				dd.DepthEnable = FALSE;
				Device()->CreateDepthStencilState(&dd, &R.ds);
				D3D11_RASTERIZER_DESC rsd{};
				rsd.FillMode = D3D11_FILL_SOLID;
				rsd.CullMode = D3D11_CULL_NONE;
				rsd.DepthClipEnable = FALSE;
				Device()->CreateRasterizerState(&rsd, &R.rs);
				D3D11_BLEND_DESC bd{};
				bd.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_ALL;
				Device()->CreateBlendState(&bd, &R.bs);
			}
			// save
			ComPtr<ID3D11VertexShader>      sVs;
			ComPtr<ID3D11PixelShader>       sPs;
			ComPtr<ID3D11InputLayout>       sIl;
			ComPtr<ID3D11RasterizerState>   sRs;
			ComPtr<ID3D11BlendState>        sBs;
			ComPtr<ID3D11DepthStencilState> sDs;
			a_c->VSGetShader(&sVs, nullptr, nullptr);
			a_c->PSGetShader(&sPs, nullptr, nullptr);
			a_c->IAGetInputLayout(&sIl);
			D3D11_PRIMITIVE_TOPOLOGY sTopo{};
			a_c->IAGetPrimitiveTopology(&sTopo);
			ID3D11ShaderResourceView* sSrv[3]{};
			a_c->PSGetShaderResources(0, 3, sSrv);
			ID3D11Buffer* sCb0 = nullptr;
			a_c->PSGetConstantBuffers(0, 1, &sCb0);
			a_c->RSGetState(&sRs);
			FLOAT bf[4]{};
			UINT  sm = 0, sref = 0;
			a_c->OMGetBlendState(&sBs, bf, &sm);
			a_c->OMGetDepthStencilState(&sDs, &sref);
			UINT           nvp = 1;
			D3D11_VIEWPORT svp{};
			a_c->RSGetViewports(&nvp, &svp);
			ID3D11RenderTargetView* sRt[8]{};
			ID3D11DepthStencilView* sDsv = nullptr;
			a_c->OMGetRenderTargets(8, sRt, &sDsv);
			// draw: colour = backup * AO, no blending, no depth
			ID3D11ShaderResourceView* none2[3]{};
			a_c->PSSetShaderResources(0, 3, none2);
			ID3D11RenderTargetView* rts[1] = { mainRtv };
			a_c->OMSetRenderTargets(1, rts, nullptr);
			a_c->IASetInputLayout(nullptr);
			a_c->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
			a_c->VSSetShader(R.vs.Get(), nullptr, 0);
			a_c->PSSetShader(R.ps.Get(), nullptr, 0);
			ID3D11ShaderResourceView* srvs[3] = { R.srv.Get(), aoSrv, R.srv2.Get() };
			a_c->PSSetShaderResources(0, 3, srvs);
			const float modev[4] = { static_cast<float>(mode), 0.0f, 0.0f, 0.0f };
			a_c->UpdateSubresource(R.cb.Get(), 0, nullptr, modev, 0, 0);
			ID3D11Buffer* cbs0[1] = { R.cb.Get() };
			a_c->PSSetConstantBuffers(0, 1, cbs0);
			a_c->RSSetState(R.rs.Get());
			a_c->OMSetDepthStencilState(R.ds.Get(), 0);
			const FLOAT zero[4]{};
			a_c->OMSetBlendState(R.bs.Get(), zero, 0xFFFFFFFF);
			D3D11_VIEWPORT vp{ 0.0f, 0.0f, static_cast<float>(R.w), static_cast<float>(R.h), 0.0f, 1.0f };
			a_c->RSSetViewports(1, &vp);
			a_c->Draw(3, 0);
			// restore
			a_c->PSSetShaderResources(0, 3, none2);
			a_c->OMSetRenderTargets(8, sRt, sDsv);
			a_c->PSSetShaderResources(0, 3, sSrv);
			a_c->PSSetConstantBuffers(0, 1, &sCb0);
			if (sCb0) sCb0->Release();
			a_c->VSSetShader(sVs.Get(), nullptr, 0);
			a_c->PSSetShader(sPs.Get(), nullptr, 0);
			a_c->IASetInputLayout(sIl.Get());
			a_c->IASetPrimitiveTopology(sTopo);
			a_c->RSSetState(sRs.Get());
			a_c->OMSetBlendState(sBs.Get(), bf, sm);
			a_c->OMSetDepthStencilState(sDs.Get(), sref);
			a_c->RSSetViewports(nvp, &svp);
			for (auto* v : sSrv) { if (v) v->Release(); }
			for (auto* r : sRt) { if (r) r->Release(); }
			if (sDsv) sDsv->Release();
			static int s_logged = 0;
			if (s_logged++ < 3) {
				logger::info("view: ssaoown: colour x AO (render target 46) written over the engine's apply result");
			}
		}
		// a_keepStandard (blendstd): the depth stays converted (standard encoding) and the camera entries stay unpatched, for the blended parts that follow
		bool DeObliqueEnd(ID3D11DeviceContext* a_c, bool a_keepStandard = false)
		{
			const bool converted = g_deoActive && g_liveDsv && g_deo.backup;
			if (a_keepStandard && converted) {
				g_deoActive = false;
				if (g_deoSwapped) {
					g_deoSwapped = false;  // entries already hold the original columns
				} else {
					OblSwap(true);
				}
				g_obl.active = false;  // Hook_Build must not patch the builds of the blended parts
				return true;
			}
			if (g_deoSwapped) {
				OblSwap(false);
				g_obl.active = true;
				g_deoSwapped = false;
			}
			if (!g_deoActive || !g_liveDsv || !g_deo.backup) {
				g_deoActive = false;
				return false;
			}
			g_deoActive = false;
			ComPtr<ID3D11Resource> liveRes;
			g_liveDsv->GetResource(&liveRes);
			if (liveRes) {
				a_c->CopyResource(liveRes.Get(), g_deo.backup.Get());
			}
			return false;
		}

		void Hook_Build(void* a_state, void* a_entry, const void* a_cam, bool a_jitter, std::uint64_t a5)
		{
			g_origBuild(a_state, a_entry, a_cam, a_jitter, a5);
			static int s_buildLogged = 0;
			if (g_inVirtual && a_cam == static_cast<const void*>(g_cam) && s_buildLogged < 24 && g_params.prepass) {
				++s_buildLogged;
				float vp[4]{}, vu[4]{};
				for (int i = 0; i < 4; ++i) {
					ReadSafeT(static_cast<const std::uint8_t*>(a_entry) + 0xC0 + i * 4, vp[i]);
					ReadSafeT(static_cast<const std::uint8_t*>(a_entry) + 0x140 + i * 4, vu[i]);
				}
				logger::info("view: camera build in a virtual submit: jitter {} entry {:p} viewProj row0 ({:.7f}, {:.7f}, {:.7f}, {:.7f}) unjittered row0 ({:.7f}, {:.7f}, {:.7f}, {:.7f}) pre-pass accumulator submitting: {}", a_jitter, a_entry, vp[0], vp[1], vp[2], vp[3], vu[0], vu[1], vu[2], vu[3], g_curAcc == static_cast<void*>(g_accPre));
			}
			{
				// G179: the engine's jitter data at every build of the main camera and of ours (what makes submit flags 0x15 black?): the state's offsets +0x44..+0x50, the TAA enable byte the build tests, the jittered and unjittered viewProj
				const bool mainCam = a_cam == static_cast<const void*>(RE::Main::WorldRootCamera());
				const bool oursCam = g_inVirtual && a_cam == static_cast<const void*>(g_cam);
				if (g_logJitter.load() && (mainCam || oursCam) && g_jitLogged < 120) {
					++g_jitLogged;
					float jo[4]{}, vj[4]{}, vu2[4]{};
					for (int i = 0; i < 4; ++i) {
						ReadSafeT(static_cast<const std::uint8_t*>(a_state) + 0x44 + i * 4, jo[i]);
						ReadSafeT(static_cast<const std::uint8_t*>(a_entry) + 0xC0 + 0x10 + i * 4, vj[i]);
						ReadSafeT(static_cast<const std::uint8_t*>(a_entry) + 0x140 + 0x10 + i * 4, vu2[i]);
					}
					std::uint8_t taaOn = 0xFF;
					const std::uint8_t* o1 = nullptr;
					const std::uint8_t* o2 = nullptr;
					ReadSafeT(reinterpret_cast<const void*>(REL::ID(414660).address()), o1);
					if (o1) ReadSafeT(o1 + 0x1F0, o2);
					if (o2) ReadSafeT(o2 + 0x18, taaOn);
					logger::info("view: jitter build {} jitter arg {} a5 {:#x} taa byte {} state offsets +0x44 ({:.6g}, {:.6g}) +0x4c ({:.6g}, {:.6g}) viewProj row1 ({:.7f}, {:.7f}, {:.7f}, {:.7f}) unjittered row1 ({:.7f}, {:.7f}, {:.7f}, {:.7f}) acc {:p}", mainCam ? "MAIN" : "ours", a_jitter, a5, taaOn, jo[0], jo[1], jo[2], jo[3], vj[0], vj[1], vj[2], vj[3], vu2[0], vu2[1], vu2[2], vu2[3], g_curAcc);
				}
			}
			auto recordVP = [&]() {
				if (a_cam == static_cast<const void*>(g_cam)) {
					for (int i = 0; i < 16; ++i) {
						ReadSafeT(static_cast<const std::uint8_t*>(a_entry) + 0xC0 + i * 4, g_ourVP[i]);
					}
					g_ourVPValid = true;
				}
			};
			if (g_vpAfter.load() == 0) {
				recordVP();
			}
			diag::OnBuild(a_entry, a_cam, a_jitter, a_cam == static_cast<const void*>(RE::Main::WorldRootCamera()), a_cam == static_cast<const void*>(g_cam));
			if (a_cam == static_cast<const void*>(RE::Main::WorldRootCamera())) {
				crossing::RollBuildCheck(reinterpret_cast<const float*>(static_cast<std::uint8_t*>(a_entry) + 0x10));  // camViewData.viewUp
			}
			if (g_inFinishPhase && a_cam == static_cast<const void*>(g_cam)) {
				++g_finishBuilds;
			}
			if (g_obl.active && (a_cam == static_cast<const void*>(g_cam) || a_cam == static_cast<const void*>(g_cam2))) {
				PatchEntry(static_cast<std::uint8_t*>(a_entry));
			}
			if (g_vpAfter.load() != 0) {
				recordVP();
			}
		}

		// ---- the sky follows the camera: for the virtual pass its nodes are shifted by (virtual camera - real camera) and put back afterwards ----
		struct SkySaved
		{
			RE::NiAVObject* o;
			NiPoint3        world, bound;
			float           radius;
		};
		std::vector<SkySaved> g_skySaved;
		std::atomic<float>    g_clearCol[3]{ { 0.02f }, { 0.02f }, { 0.05f } };
		std::atomic<int>      g_skyBig{ 1 };  // default 1 since G259; view skybig=0|1 (G221): while our cull runs the sky nodes get a huge bounding sphere so the custom cull planes of a portal view (cone, exit plane, far) can never reject a cloud layer

		void CollectSky(RE::NiAVObject* a_o, std::vector<RE::NiAVObject*>& a_out, int a_depth)
		{
			if (!a_o || a_depth > 6 || std::find(a_out.begin(), a_out.end(), a_o) != a_out.end()) {
				return;
			}
			a_out.push_back(a_o);
			if (auto* n = a_o->AsNode()) {
				auto& ch = n->GetChildren();
				for (decltype(ch.size()) i = 0; i < ch.size(); ++i) {
					CollectSky(ch[i].get(), a_out, a_depth + 1);
				}
			}
		}

		void ShiftSky(const NiPoint3& a_delta)
		{
			auto* sky = RE::Sky::GetSingleton();
			if (!sky) {
				return;
			}
			std::vector<RE::NiAVObject*> all;
			static int s_skyLogged = 0;
			if (s_skyLogged < 1 && sky->clouds) {
				++s_skyLogged;
				for (std::uint16_t i = 0; i < sky->clouds->numLayers && i < 32; ++i) {
					auto* g = sky->clouds->clouds[i].get();
					if (!g) continue;
					logger::info("view: cloud layer {}: '{}' world translate ({:.0f}, {:.0f}, {:.0f}) bound centre ({:.0f}, {:.0f}, {:.0f}) radius {:.0f} flags {:#x} (main camera at ({:.0f}, {:.0f}, {:.0f}))", i, g->name.c_str() ? g->name.c_str() : "?", g->world.translate.x, g->world.translate.y, g->world.translate.z, g->worldBound.center.x, g->worldBound.center.y, g->worldBound.center.z, g->worldBound.radius, static_cast<unsigned>(g->GetFlags().underlying()), a_delta.x, a_delta.y, a_delta.z);
				}
			}
			CollectSky(sky->root.get(), all, 0);
			CollectSky(sky->moonsRoot.get(), all, 0);
			CollectSky(sky->auroraRoot.get(), all, 0);
			g_skySaved.clear();
			for (auto* o : all) {
				g_skySaved.push_back({ o, o->world.translate, o->worldBound.center, o->worldBound.radius });
				o->world.translate = o->world.translate + a_delta;
				o->worldBound.center = o->worldBound.center + a_delta;
				if (g_skyBig.load() != 0) {
					o->worldBound.radius = (std::max)(o->worldBound.radius, 1.0e7f);
				}
			}
		}

		void RestoreSky()
		{
			for (auto& s : g_skySaved) {
				s.o->world.translate = s.world;
				s.o->worldBound.center = s.bound;
				s.o->worldBound.radius = s.radius;
			}
			g_skySaved.clear();
		}

		std::string g_depthName;
		std::atomic<bool> g_depthPending{ false };

		// Reads the main depth buffer back (what the virtual pass just drew) and reports how much of the screen has geometry, plus a picture of the distances.
		bool GetLiveDepthResource(ComPtr<ID3D11Resource>& a_res)
		{
			if (g_liveDsv) {
				g_liveDsv->GetResource(&a_res);
				return a_res != nullptr;
			}
			auto* dsData = &RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData().depthStencils[g_params.ds];
			auto* depthSRV = reinterpret_cast<ID3D11ShaderResourceView*>(dsData->depthSRV);
			if (!depthSRV) return false;
			depthSRV->GetResource(&a_res);
			return a_res != nullptr;
		}
		// mean / min / max per channel of the screen-space shadow mask target (R8G8B8A8 / R16 / R16G16B16A16 families), every 8th pixel
		void LogMaskStats(ID3D11DeviceContext* a_ctx, const char* a_when)
		{
			auto* rd2 = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			auto* tex = rd2 ? reinterpret_cast<ID3D11Texture2D*>(rd2->renderTargets[RE::RENDER_TARGETS::kSHADOW_MASK].texture) : nullptr;
			if (!tex) {
				logger::info("view: shadow mask {}: no texture", a_when);
				return;
			}
			D3D11_TEXTURE2D_DESC d{};
			tex->GetDesc(&d);
			D3D11_TEXTURE2D_DESC sd = d;
			sd.Usage = D3D11_USAGE_STAGING;
			sd.BindFlags = 0;
			sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
			sd.MiscFlags = 0;
			sd.ArraySize = 1;
			sd.MipLevels = 1;
			ComPtr<ID3D11Texture2D> st;
			if (d.SampleDesc.Count != 1 || FAILED(Device()->CreateTexture2D(&sd, nullptr, &st))) {
				logger::info("view: shadow mask {}: format {} samples {}: not readable", a_when, static_cast<int>(d.Format), d.SampleDesc.Count);
				return;
			}
			a_ctx->CopySubresourceRegion(st.Get(), 0, 0, 0, 0, tex, 0, nullptr);
			D3D11_MAPPED_SUBRESOURCE m{};
			if (FAILED(a_ctx->Map(st.Get(), 0, D3D11_MAP_READ, 0, &m))) {
				return;
			}
			double sum[4]{}, mn[4]{ 1e9, 1e9, 1e9, 1e9 }, mx[4]{ -1e9, -1e9, -1e9, -1e9 };
			std::size_t n = 0;
			int         bytes = 4;
			switch (d.Format) {
			case DXGI_FORMAT_R16G16B16A16_FLOAT:
			case DXGI_FORMAT_R16G16B16A16_UNORM: bytes = 8; break;
			case DXGI_FORMAT_R8_UNORM: bytes = 1; break;
			case DXGI_FORMAT_R16_UNORM:
			case DXGI_FORMAT_R16_FLOAT:
			case DXGI_FORMAT_R8G8_UNORM: bytes = 2; break;
			default: bytes = 4; break;
			}
			for (UINT y = 0; y < d.Height; y += 8) {
				const auto* row = static_cast<const std::uint8_t*>(m.pData) + static_cast<std::size_t>(y) * m.RowPitch;
				for (UINT x = 0; x < d.Width; x += 8) {
					for (int c = 0; c < 4; ++c) {
						double v = 0;
						const std::uint8_t* p = row + x * bytes;
						if (d.Format == DXGI_FORMAT_R16G16B16A16_FLOAT) v = DirectX::PackedVector::XMConvertHalfToFloat(reinterpret_cast<const std::uint16_t*>(p)[c]);
						else if (d.Format == DXGI_FORMAT_R16G16B16A16_UNORM) v = reinterpret_cast<const std::uint16_t*>(p)[c] / 65535.0;
						else if (d.Format == DXGI_FORMAT_R16_UNORM) v = reinterpret_cast<const std::uint16_t*>(p)[0] / 65535.0;
						else if (d.Format == DXGI_FORMAT_R16_FLOAT) v = DirectX::PackedVector::XMConvertHalfToFloat(reinterpret_cast<const std::uint16_t*>(p)[0]);
						else if (d.Format == DXGI_FORMAT_R8_UNORM) v = p[0] / 255.0;
						else v = p[c < bytes ? c : 0] / 255.0;
						sum[c] += v;
						mn[c] = (std::min)(mn[c], v);
						mx[c] = (std::max)(mx[c], v);
					}
					++n;
				}
			}
			a_ctx->Unmap(st.Get(), 0);
			logger::info("view: shadow mask {}: format {} {}x{}: mean ({:.3f}, {:.3f}, {:.3f}, {:.3f}) min ({:.3f}, {:.3f}, {:.3f}, {:.3f}) max ({:.3f}, {:.3f}, {:.3f}, {:.3f})", a_when, static_cast<int>(d.Format), d.Width, d.Height,
				sum[0] / n, sum[1] / n, sum[2] / n, sum[3] / n, mn[0], mn[1], mn[2], mn[3], mx[0], mx[1], mx[2], mx[3]);
		}
		// fraction of the main depth target that holds geometry (below the far value), for the depth-pre-pass tests
		double DepthCoverage(ID3D11DeviceContext* a_ctx)
		{
			ComPtr<ID3D11Resource> res;
			if (!GetLiveDepthResource(res)) {
				return -1;
			}
			ComPtr<ID3D11Texture2D> tex;
			if (FAILED(res.As(&tex))) {
				return -1;
			}
			D3D11_TEXTURE2D_DESC d{};
			tex->GetDesc(&d);
			D3D11_TEXTURE2D_DESC sd = d;
			sd.Usage = D3D11_USAGE_STAGING;
			sd.BindFlags = 0;
			sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
			sd.MiscFlags = 0;
			sd.ArraySize = 1;
			sd.MipLevels = 1;
			ComPtr<ID3D11Texture2D> staging;
			if (d.SampleDesc.Count != 1 || FAILED(Device()->CreateTexture2D(&sd, nullptr, &staging))) {
				return -1;
			}
			a_ctx->CopySubresourceRegion(staging.Get(), 0, 0, 0, 0, tex.Get(), 0, nullptr);
			D3D11_MAPPED_SUBRESOURCE m{};
			if (FAILED(a_ctx->Map(staging.Get(), 0, D3D11_MAP_READ, 0, &m))) {
				return -1;
			}
			std::size_t drawn = 0, total = 0;
			for (UINT y = 0; y < d.Height; y += 4) {
				const auto* row = static_cast<const std::uint8_t*>(m.pData) + static_cast<std::size_t>(y) * m.RowPitch;
				for (UINT x = 0; x < d.Width; x += 4) {
					float z = 1.0f;
					if (d.Format == DXGI_FORMAT_R32G8X24_TYPELESS || d.Format == DXGI_FORMAT_D32_FLOAT_S8X24_UINT) {
						std::memcpy(&z, row + x * 8, 4);
					} else if (d.Format == DXGI_FORMAT_R32_TYPELESS || d.Format == DXGI_FORMAT_D32_FLOAT || d.Format == DXGI_FORMAT_R32_FLOAT) {
						std::memcpy(&z, row + x * 4, 4);
					} else {
						std::uint32_t v;
						std::memcpy(&v, row + x * 4, 4);
						z = (v & 0xFFFFFF) / 16777215.0f;
					}
					++total;
					if (z < 0.999999f) {
						++drawn;
					}
				}
			}
			a_ctx->Unmap(staging.Get(), 0);
			return total ? static_cast<double>(drawn) / static_cast<double>(total) : 0.0;
		}
		int g_coverLogged{ 0 };
		std::vector<float> g_depthPre;
		// every 4th depth value of the main depth target
		bool ReadDepthSamples(ID3D11DeviceContext* a_ctx, std::vector<float>& a_out)
		{
			a_out.clear();
			ComPtr<ID3D11Resource> res;
			if (!GetLiveDepthResource(res)) return false;
			ComPtr<ID3D11Texture2D> tex;
			if (FAILED(res.As(&tex))) return false;
			D3D11_TEXTURE2D_DESC d{};
			tex->GetDesc(&d);
			D3D11_TEXTURE2D_DESC sd = d;
			sd.Usage = D3D11_USAGE_STAGING;
			sd.BindFlags = 0;
			sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
			sd.MiscFlags = 0;
			sd.ArraySize = 1;
			sd.MipLevels = 1;
			ComPtr<ID3D11Texture2D> staging;
			if (d.SampleDesc.Count != 1 || FAILED(Device()->CreateTexture2D(&sd, nullptr, &staging))) return false;
			a_ctx->CopySubresourceRegion(staging.Get(), 0, 0, 0, 0, tex.Get(), 0, nullptr);
			D3D11_MAPPED_SUBRESOURCE m{};
			if (FAILED(a_ctx->Map(staging.Get(), 0, D3D11_MAP_READ, 0, &m))) return false;
			for (UINT y = 0; y < d.Height; y += 4) {
				const auto* row = static_cast<const std::uint8_t*>(m.pData) + static_cast<std::size_t>(y) * m.RowPitch;
				for (UINT x = 0; x < d.Width; x += 4) {
					float z = 1.0f;
					if (d.Format == DXGI_FORMAT_R32G8X24_TYPELESS || d.Format == DXGI_FORMAT_D32_FLOAT_S8X24_UINT) {
						std::memcpy(&z, row + x * 8, 4);
					} else if (d.Format == DXGI_FORMAT_R32_TYPELESS || d.Format == DXGI_FORMAT_D32_FLOAT || d.Format == DXGI_FORMAT_R32_FLOAT) {
						std::memcpy(&z, row + x * 4, 4);
					} else {
						std::uint32_t v;
						std::memcpy(&v, row + x * 4, 4);
						z = (v & 0xFFFFFF) / 16777215.0f;
					}
					a_out.push_back(z);
				}
			}
			a_ctx->Unmap(staging.Get(), 0);
			return true;
		}
		void DumpDepth(int a_index, ID3D11DeviceContext* a_ctx, const world::CamInfo& a_main)
		{
			auto* dsData = &RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData().depthStencils[g_params.ds];
			auto* depthSRV = reinterpret_cast<ID3D11ShaderResourceView*>(dsData->depthSRV);
			if (!depthSRV) {
				logger::warn("view: depth dump: no depth SRV");
				return;
			}
			ComPtr<ID3D11Resource> res;
			depthSRV->GetResource(&res);
			ComPtr<ID3D11Texture2D> tex;
			if (FAILED(res.As(&tex))) {
				return;
			}
			D3D11_TEXTURE2D_DESC d{};
			tex->GetDesc(&d);
			D3D11_TEXTURE2D_DESC sd = d;
			sd.Usage = D3D11_USAGE_STAGING;
			sd.BindFlags = 0;
			sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
			sd.MiscFlags = 0;
			sd.ArraySize = 1;
			sd.MipLevels = 1;
			ComPtr<ID3D11Texture2D> staging;
			if (d.SampleDesc.Count != 1 || FAILED(Device()->CreateTexture2D(&sd, nullptr, &staging))) {
				return;
			}
			a_ctx->CopySubresourceRegion(staging.Get(), 0, 0, 0, 0, tex.Get(), 0, nullptr);
			D3D11_MAPPED_SUBRESOURCE m{};
			if (FAILED(a_ctx->Map(staging.Get(), 0, D3D11_MAP_READ, 0, &m))) {
				return;
			}
			const float n = a_main.nearDist, f = a_main.farDist;
			std::vector<std::uint8_t> bgra(static_cast<std::size_t>(d.Width) * d.Height * 4, 255);
			std::size_t drawn = 0, top = 0, bottom = 0;
			std::vector<float> dist;
			for (UINT y = 0; y < d.Height; ++y) {
				const auto* row = static_cast<const std::uint8_t*>(m.pData) + static_cast<std::size_t>(y) * m.RowPitch;
				for (UINT x = 0; x < d.Width; ++x) {
					float z = 1.0f;
					if (d.Format == DXGI_FORMAT_R32G8X24_TYPELESS || d.Format == DXGI_FORMAT_D32_FLOAT_S8X24_UINT) {
						std::memcpy(&z, row + x * 8, 4);
					} else if (d.Format == DXGI_FORMAT_R32_TYPELESS || d.Format == DXGI_FORMAT_D32_FLOAT || d.Format == DXGI_FORMAT_R32_FLOAT) {
						std::memcpy(&z, row + x * 4, 4);
					} else {
						std::uint32_t v;
						std::memcpy(&v, row + x * 4, 4);
						z = (v & 0xFFFFFF) / 16777215.0f;
					}
					std::uint8_t g = 0;
					if (z < 0.999999f) {
						++drawn;
						(y < d.Height / 2 ? top : bottom)++;
						const float lin = n * f / (f - z * (f - n));
						if ((x & 7) == 0 && (y & 7) == 0) dist.push_back(lin);
						g = static_cast<std::uint8_t>(std::clamp(255.0f - 255.0f * std::log10((std::max)(lin, 10.0f) / 10.0f) / 4.0f, 0.0f, 255.0f));  // 10 units white .. 100000 black
						if (g == 0) g = 1;
					}
					const auto i = (static_cast<std::size_t>(y) * d.Width + x) * 4;
					bgra[i] = bgra[i + 1] = bgra[i + 2] = g;
				}
			}
			a_ctx->Unmap(staging.Get(), 0);
			std::sort(dist.begin(), dist.end());
			auto pct = [&](double p) { return dist.empty() ? 0.0f : dist[(std::min)(dist.size() - 1, static_cast<std::size_t>(p * dist.size()))]; };
			const double total = static_cast<double>(d.Width) * d.Height;
			logger::info("view: depth dump portal {}: depth format {}, {:.1f}% of the pixels have geometry (top half {:.1f}%, bottom half {:.1f}%), distance of those: p10 {:.0f} p50 {:.0f} p90 {:.0f} units",
				a_index, static_cast<int>(d.Format), 100.0 * drawn / total, 200.0 * top / total, 200.0 * bottom / total, pct(0.1), pct(0.5), pct(0.9));
			shot::SaveBgra(std::format("{}_p{}", g_depthName, a_index), d.Width, d.Height, bgra);
		}

		// One virtual view: entry portal A (the one we look at), exit portal B. Returns false when a precondition is missing.
		// The player's own body (third person) drawn into the virtual picture with the REAL camera, keeping what lies behind the entry portal's plane (see Ghost.cpp for why that is the
		// body carried through the pair as the virtual camera sees it). The targets and the accumulator are those of the pass that just ended; nothing is cleared.
		bool RenderBody(const portals::Portal& A, const world::CamInfo& main, std::uint8_t* acc, const RE::NiFrustum& full)
		{
			if (Dot(A.n, main.pos - A.c) <= 0.5f) {
				return false;
			}
			// what is drawn: the player's own body (third person) and every other entity that straddles this portal's plane (props, bodies, NPCs ...): each one's own 3D root
			auto*                        p = RE::PlayerCharacter::GetSingleton();
			RE::NiAVObject*              root = nullptr;  // the player's, when it straddles
			std::vector<RE::NiAVObject*> roots;
			if (ghost::Straddling(A)) {
				root = p ? p->Get3D(false) : nullptr;
				if (root) {
					roots.push_back(root);
				}
			}
			{
				std::vector<travel::Straddler> st;
				travel::Straddlers(st);
				for (const auto& s : st) {
					if (s.entry == A.colour) {
						roots.push_back(s.root);
					}
				}
			}
			auto* mainCam = RE::Main::WorldRootCamera();
			if (roots.empty() || !mainCam) {
				return false;
			}
			if (!g_cam2) {
				g_cam2 = static_cast<std::uint8_t*>(RE::malloc(sizeof(RE::NiCamera)));
			}
			std::memcpy(g_cam2, mainCam, sizeof(RE::NiCamera));
			auto* cam = reinterpret_cast<RE::NiCamera*>(g_cam2);
			*reinterpret_cast<volatile std::uint32_t*>(g_cam2 + 0x08) = 0x40000000u;  // refcount: never freed
			cam->parent = nullptr;
			reinterpret_cast<CamUpdate_t>(REL::ID(70641).address())(g_cam2);

			// the entry plane with the opposite normal keeps what is behind the entry portal (the same depth mapping as the virtual pass's exit plane)
			g_obl = Oblique{}; g_oblSaved.clear();
			if (g_params.oblique) {
				g_obl.active = true;
				g_obl.n = A.n * (ghost::DebugFront() ? 1.0f : -1.0f);
				g_obl.dRel = Dot(g_obl.n, main.pos - A.c) + g_params.planeBias;
				g_obl.fwd = main.fwd; g_obl.up = main.up; g_obl.right = main.right;
				g_obl.tl = full.fLeft; g_obl.tr = full.fRight; g_obl.tb = full.fBottom; g_obl.tt = full.fTop;
				g_obl.nearD = full.fNear; g_obl.farD = full.fFar;
			}
			auto** accVt = *reinterpret_cast<void***>(acc);
			reinterpret_cast<StartAcc_t>(accVt[0x25])(acc, g_cam2);
			SetAccFields(acc, main.pos);
			if (g_params.skinFix) {
				travel::ResetSkinCaches();
			}
			const auto inner = (!root || ghost::DebugFront()) ? std::vector<RE::NiAVObject*>{} : ghost::HideInnerParts(root);
			for (auto* r : roots) {
				alignas(16) std::uint8_t ctx[0x68]{};
				reinterpret_cast<CtxCtor_t>(REL::ID(106919).address())(ctx);
				*reinterpret_cast<void**>(ctx + 0x00) = acc;
				*reinterpret_cast<void**>(ctx + 0x10) = g_cam2;
				*reinterpret_cast<void**>(ctx + 0x18) = nullptr;
				*reinterpret_cast<void**>(ctx + 0x20) = &cam->GetRuntimeData2().viewFrustum;
				*reinterpret_cast<void**>(ctx + 0x28) = nullptr;
				*reinterpret_cast<void**>(ctx + 0x30) = nullptr;
				*reinterpret_cast<void**>(ctx + 0x38) = nullptr;
				*reinterpret_cast<void**>(ctx + 0x40) = r;  // only this entity's 3D (the player's skeleton, a prop, a body ...)
				*reinterpret_cast<void**>(ctx + 0x48) = nullptr;
				*reinterpret_cast<std::uint32_t*>(ctx + 0x50) = 3;
				ctx[0x5E] = g_params.ignorePre ? 1 : 0;  // ignorePreprocess
				ctx[0x5F] = 0;
				ctx[0x60] = 1;
				reinterpret_cast<Cull_t>(REL::ID(106921).address())(ctx, 0, 0);
			}
			ghost::ShowInnerParts(inner);  // the cull has taken what it needs; the real view must see them
			PokeBeforeSubmit(2);
			reinterpret_cast<Submit_t>(REL::ID(106436).address())(g_cam2, acc, g_params.submitFlags);
			PokeBeforeSubmit(1);
			g_obl.active = false;
			ghost::NoteBodyPass();
			static int logged = 0;
			if (logged < 4) {
				++logged;
				logger::info("view: body pass for portal entry (oblique matrices patched so far {})", g_oblApplied);
			}
			return true;
		}

		// Private colour and depth copies for the exit pictures (see g_p2c): same size as the main target; the depth copy has the engine depth texture's own format.
		bool EnsureExitTextures(ID3D11Texture2D* a_mainTex, ID3D11Texture2D* a_depthTex)
		{
			D3D11_TEXTURE2D_DESC cd{}, dd{};
			a_mainTex->GetDesc(&cd);
			a_depthTex->GetDesc(&dd);
			if (g_p2c[0] && g_p2w == cd.Width && g_p2h == cd.Height && g_p2dFmt == dd.Format) {
				return true;
			}
			g_p2w = cd.Width; g_p2h = cd.Height; g_p2dFmt = dd.Format;
			DXGI_FORMAT srvFmt = DXGI_FORMAT_UNKNOWN;
			if (dd.Format == DXGI_FORMAT_R32G8X24_TYPELESS) {
				srvFmt = DXGI_FORMAT_R32_FLOAT_X8X24_TYPELESS;
			} else if (dd.Format == DXGI_FORMAT_R32_TYPELESS) {
				srvFmt = DXGI_FORMAT_R32_FLOAT;
			} else if (dd.Format == DXGI_FORMAT_R24G8_TYPELESS) {
				srvFmt = DXGI_FORMAT_R24_UNORM_X8_TYPELESS;  // the engine's depth texture (format 44, found in G79); the shader reads the depth as .x in 0..1
			}
			if (srvFmt == DXGI_FORMAT_UNKNOWN || dd.SampleDesc.Count != 1) {
				logger::warn("view: the depth texture (format {}, {} samples) cannot be copied for the exit pictures", static_cast<int>(dd.Format), dd.SampleDesc.Count);
				return false;
			}
			for (auto* d : { &cd, &dd }) {
				d->BindFlags = D3D11_BIND_SHADER_RESOURCE;
				d->MiscFlags = 0;
				d->MipLevels = 1;
				d->ArraySize = 1;
				d->SampleDesc.Count = 1;
				d->SampleDesc.Quality = 0;
				d->Usage = D3D11_USAGE_DEFAULT;
				d->CPUAccessFlags = 0;
			}
			D3D11_SHADER_RESOURCE_VIEW_DESC sv{};
			sv.Format = srvFmt;
			sv.ViewDimension = D3D11_SRV_DIMENSION_TEXTURE2D;
			sv.Texture2D.MipLevels = 1;
			auto* dev = Device();
			for (int i = 0; i < 2; ++i) {
				g_p2c[i].Reset(); g_p2cSrv[i].Reset(); g_p2d[i].Reset(); g_p2dSrv[i].Reset();
				if (FAILED(dev->CreateTexture2D(&cd, nullptr, &g_p2c[i])) || FAILED(dev->CreateShaderResourceView(g_p2c[i].Get(), nullptr, &g_p2cSrv[i])) || FAILED(dev->CreateTexture2D(&dd, nullptr, &g_p2d[i])) ||
					FAILED(dev->CreateShaderResourceView(g_p2d[i].Get(), &sv, &g_p2dSrv[i]))) {
					logger::error("view: cannot create the exit picture textures {} ({}x{})", i, cd.Width, cd.Height);
					g_p2c[0].Reset();
					return false;
				}
			}
			for (int i = 2; i < 4; ++i) {  // the front-part pictures (kind 1)
				g_p2c[i].Reset(); g_p2cSrv[i].Reset(); g_p2d[i].Reset(); g_p2dSrv[i].Reset();
				if (FAILED(dev->CreateTexture2D(&cd, nullptr, &g_p2c[i])) || FAILED(dev->CreateShaderResourceView(g_p2c[i].Get(), nullptr, &g_p2cSrv[i])) || FAILED(dev->CreateTexture2D(&dd, nullptr, &g_p2d[i])) ||
					FAILED(dev->CreateShaderResourceView(g_p2d[i].Get(), &sv, &g_p2dSrv[i]))) {
					logger::error("view: cannot create the exit picture textures {} ({}x{})", i, cd.Width, cd.Height);
					g_p2c[0].Reset();
					return false;
				}
			}
			logger::info("view: exit picture textures {}x{}, depth format {}", cd.Width, cd.Height, static_cast<int>(dd.Format));
			return true;
		}

		// What straddles THIS pass's exit portal B, drawn from the pass's virtual camera: the camera stands behind B's plane (the viewer is in front of the entry), so the part of the
		// entity that has gone through B's plane lies on the camera's side, and keeping what is behind B's plane gives exactly that part, which in the real world is the part emerging in
		// front of the entry portal (the same rigid-map argument as for the body pass). Drawn into the main targets (cleared first) and copied out as colour and depth.
		// cost of the extra passes, logged every 240 calls
		struct PassCost
		{
			double sum{ 0 }, max{ 0 };
			int    n{ 0 };
			void   Add(const char* a_name, double a_ms)
			{
				sum += a_ms;
				max = (std::max)(max, a_ms);
				if (++n >= 240) {
					logger::info("view: {} avg {:.3f} ms max {:.3f} ms over {} call(s)", a_name, sum / n, max, n);
					sum = max = 0;
					n = 0;
				}
			}
		};
		PassCost g_costBody, g_costExit;

			// test (G86): GPU pipeline statistics of a section of a pass (did the pass issue any vertices for the entities in it?). `view passstats=N` logs N frames.
			struct StatSlot
			{
				ComPtr<ID3D11Query> q;
				bool                pending{ false };
				int                 frame{ 0 };
				int                 portal{ 0 };
				const char*         tag{ "" };
			};
			constexpr int kStatRing = 16;
			StatSlot      g_stat[kStatRing];
			int           g_statNext{ 0 };
			int           g_statLeft{ 0 };
			bool          g_statOn{ false };
			int           g_statFrame{ 0 };
			int           g_npcBoundsLeft{ 0 };

			int StatBegin(ID3D11DeviceContext* a_c, int a_portal, const char* a_tag)
			{
				if (!g_statOn) {
					return -1;
				}
				auto& s = g_stat[g_statNext];
				if (s.pending) {
					return -1;
				}
				if (!s.q) {
					D3D11_QUERY_DESC qd{ D3D11_QUERY_PIPELINE_STATISTICS, 0 };
					if (FAILED(Device()->CreateQuery(&qd, &s.q))) {
						return -1;
					}
				}
				a_c->Begin(s.q.Get());
				s.frame = g_statFrame;
				s.portal = a_portal;
				s.tag = a_tag;
				const int idx = g_statNext;
				g_statNext = (g_statNext + 1) % kStatRing;
				return idx;
			}
			void StatEnd(ID3D11DeviceContext* a_c, int a_idx)
			{
				if (a_idx >= 0) {
					a_c->End(g_stat[a_idx].q.Get());
					g_stat[a_idx].pending = true;
				}
			}
			void StatPoll()
			{
				bool any = false;
				for (const auto& s : g_stat) {
					any = any || s.pending;
				}
				if (!any) {
					return;
				}
				ComPtr<ID3D11DeviceContext> ctx;
				Device()->GetImmediateContext(&ctx);
				auto* a_c = ctx.Get();
				for (auto& s : g_stat) {
					if (!s.pending) {
						continue;
					}
					D3D11_QUERY_DATA_PIPELINE_STATISTICS d{};
					if (a_c->GetData(s.q.Get(), &d, sizeof(d), D3D11_ASYNC_GETDATA_DONOTFLUSH) == S_OK) {
						s.pending = false;
						logger::info("view: stats frame {} portal {} {}: IA vertices {} prims {}, VS {}, clip in {} out {}, PS {}", s.frame, s.portal, s.tag, d.IAVertices, d.IAPrimitives, d.VSInvocations, d.CInvocations, d.CPrimitives, d.PSInvocations);
					}
				}
			}
		double   NowMs() { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

		bool RenderExit(int a_index, const portals::Portal& A, const portals::Portal& B, const world::CamInfo& main, const NiPoint3& pos, const NiPoint3& f, const NiPoint3& u, const NiPoint3& rgt, const RE::NiFrustum& full,
			std::uint8_t* acc, ID3D11DeviceContext* d3d, ID3D11RenderTargetView* rtv, ID3D11DepthStencilView* dsv, ID3D11Texture2D* mainTex)
		{
			// Two kinds of entity go into this one picture, both composited over the main view after the portal ovals:
			//   back:  what straddles the EXIT portal B, drawn from the virtual camera keeping what is behind B's plane: the part that has come out in front of this (entry) portal
			//   front: what straddles the ENTRY portal A, drawn from the real camera whole: the oval's picture is drawn a few units in front of the wall (to cover the wall's relief), which
			//          also covers the first units of anything that is approaching the plane from the front, and that is put back here (the part behind the plane fails the depth test
			//          against the oval's own depth, which is at the plane's depth)
			auto*                        player = RE::PlayerCharacter::GetSingleton();
			RE::NiAVObject*              playerBack = nullptr;
			RE::NiAVObject*              playerFront = nullptr;
			std::vector<RE::NiAVObject*> back, front;
			if (ghost::Straddling(B)) {
				playerBack = player ? player->Get3D(false) : nullptr;
				if (playerBack) {
					back.push_back(playerBack);
				}
			}
			if (ghost::Straddling(A)) {
				playerFront = player ? player->Get3D(false) : nullptr;
				if (playerFront) {
					front.push_back(playerFront);
				}
			}
			{
				std::vector<travel::Straddler> st;
				travel::Straddlers(st);
				for (const auto& s : st) {
					if (s.entry == B.colour) {
						back.push_back(s.root);
					}
					if (s.entry == A.colour) {
						front.push_back(s.root);
					}
				}
			}
			auto* mainCam = RE::Main::WorldRootCamera();
			auto* dsData = &RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData().depthStencils[g_params.ds];
			auto* depthSRV = reinterpret_cast<ID3D11ShaderResourceView*>(dsData->depthSRV);
			if ((back.empty() && front.empty()) || !mainCam || !depthSRV || !rtv || !dsv) {
				return false;
			}
			ComPtr<ID3D11Resource> res;
			depthSRV->GetResource(&res);
			ComPtr<ID3D11Texture2D> dtex;
			if (FAILED(res.As(&dtex)) || !EnsureExitTextures(mainTex, dtex.Get())) {
				return false;
			}
			const FLOAT black[4] = { 0, 0, 0, 0 };
			d3d->ClearRenderTargetView(rtv, black);
			d3d->ClearDepthStencilView(dsv, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);
			if (!g_cam2) {
				g_cam2 = static_cast<std::uint8_t*>(RE::malloc(sizeof(RE::NiCamera)));
			}
			auto* cam = reinterpret_cast<RE::NiCamera*>(g_cam2);

			// one draw: a camera, an optional oblique plane, some roots
			auto draw = [&](const NiPoint3& a_pos, const NiPoint3& a_f, const NiPoint3& a_u, const NiPoint3& a_r, const NiPoint3* a_obliqueN, const std::vector<RE::NiAVObject*>& a_roots, RE::NiAVObject* a_player) {
				if (a_roots.empty()) {
					return;
				}
				std::memcpy(g_cam2, mainCam, sizeof(RE::NiCamera));
				*reinterpret_cast<volatile std::uint32_t*>(g_cam2 + 0x08) = 0x40000000u;
				cam->parent = nullptr;
				cam->world.translate = a_pos;
				cam->world.rotate.entry[0][0] = a_f.x; cam->world.rotate.entry[0][1] = a_u.x; cam->world.rotate.entry[0][2] = a_r.x;
				cam->world.rotate.entry[1][0] = a_f.y; cam->world.rotate.entry[1][1] = a_u.y; cam->world.rotate.entry[1][2] = a_r.y;
				cam->world.rotate.entry[2][0] = a_f.z; cam->world.rotate.entry[2][1] = a_u.z; cam->world.rotate.entry[2][2] = a_r.z;
				reinterpret_cast<CamUpdate_t>(REL::ID(70641).address())(g_cam2);
				g_obl = Oblique{}; g_oblSaved.clear();
				if (g_params.oblique && a_obliqueN) {
					g_obl.active = true;
					g_obl.n = *a_obliqueN;
					g_obl.dRel = Dot(g_obl.n, a_pos - B.c) + g_params.planeBias;
					g_obl.fwd = a_f; g_obl.up = a_u; g_obl.right = a_r;
					g_obl.tl = full.fLeft; g_obl.tr = full.fRight; g_obl.tb = full.fBottom; g_obl.tt = full.fTop;
					g_obl.nearD = full.fNear; g_obl.farD = full.fFar;
				}
				auto** accVt = *reinterpret_cast<void***>(acc);
				reinterpret_cast<StartAcc_t>(accVt[0x25])(acc, g_cam2);
				SetAccFields(acc, a_pos);
				if (g_params.skinFix) {
					travel::ResetSkinCaches();
				}
				const auto inner = (!a_player || ghost::DebugFront()) ? std::vector<RE::NiAVObject*>{} : ghost::HideInnerParts(a_player);
				for (auto* r : a_roots) {
					alignas(16) std::uint8_t ctx[0x68]{};
					reinterpret_cast<CtxCtor_t>(REL::ID(106919).address())(ctx);
					*reinterpret_cast<void**>(ctx + 0x00) = acc;
					*reinterpret_cast<void**>(ctx + 0x10) = g_cam2;
					*reinterpret_cast<void**>(ctx + 0x18) = nullptr;
					*reinterpret_cast<void**>(ctx + 0x20) = &cam->GetRuntimeData2().viewFrustum;
					*reinterpret_cast<void**>(ctx + 0x28) = nullptr;
					*reinterpret_cast<void**>(ctx + 0x30) = nullptr;
					*reinterpret_cast<void**>(ctx + 0x38) = nullptr;
					*reinterpret_cast<void**>(ctx + 0x40) = r;
					*reinterpret_cast<void**>(ctx + 0x48) = nullptr;
					*reinterpret_cast<std::uint32_t*>(ctx + 0x50) = 3;
					ctx[0x5E] = g_params.ignorePre ? 1 : 0;  // ignorePreprocess
				ctx[0x5F] = 0;
					ctx[0x60] = 1;
					reinterpret_cast<Cull_t>(REL::ID(106921).address())(ctx, 0, 0);
				}
				ghost::ShowInnerParts(inner);
				PokeBeforeSubmit(2);
				reinterpret_cast<Submit_t>(REL::ID(106436).address())(g_cam2, acc, g_params.submitFlags);
				PokeBeforeSubmit(1);
				g_obl.active = false;
			};
			const NiPoint3 keepBehindB = B.n * -1.0f;
			if (!front.empty()) {
				draw(main.pos, main.fwd, main.up, main.right, nullptr, front, playerFront);  // the real camera, nothing clipped
				d3d->CopyResource(g_p2c[a_index + 2].Get(), mainTex);
				d3d->CopyResource(g_p2d[a_index + 2].Get(), dtex.Get());
				g_p2valid[a_index + 2] = true;
				if (!back.empty()) {
					d3d->ClearRenderTargetView(rtv, black);
					d3d->ClearDepthStencilView(dsv, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);
				}
			}
			if (!back.empty()) {
				draw(pos, f, u, rgt, &keepBehindB, back, playerBack);  // the virtual camera, keeping what is behind the exit's plane
				d3d->CopyResource(g_p2c[a_index].Get(), mainTex);
				d3d->CopyResource(g_p2d[a_index].Get(), dtex.Get());
				g_p2valid[a_index] = true;
			}
			static int logged = 0;
			if (logged < 6) {
				++logged;
				logger::info("view: exit picture for portal {}: {} root(s) in front of the entry plane (real camera), {} behind the exit plane (virtual camera)", a_index, front.size(), back.size());
			}
			return true;
		}

		bool RunPass(int a_index, const portals::Portal& A, const portals::Portal& B, const world::CamInfo& main, float a_l, float a_r, float a_b, float a_t, std::string& a_why)
		{
			const double t0 = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
			auto*        mainCam = RE::Main::WorldRootCamera();
			void*        scene = SceneStateNode();
			if (!mainCam || !scene || *reinterpret_cast<std::uintptr_t*>(scene) != RE::VTABLE_ShadowSceneNode[0].address()) {
				a_why = "no world camera or shadow scene node";
				return false;
			}
			auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			if (!rd || !rd->renderTargets[g_params.rt].texture) {
				a_why = "no main render target";
				return false;
			}
			auto* mainTex = reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[g_params.rt].texture);
			if (!EnsureTextures(mainTex)) {
				a_why = "no private texture";
				return false;
			}

			// --- virtual camera: the main camera carried through the pair ---
			const NiPoint3 pos = g_params.identity ? NiPoint3(main.pos.x + g_params.offX, main.pos.y + g_params.offY, main.pos.z + g_params.offZ) : MapPoint(A, B, main.pos);
			auto           rotYaw = [](const NiPoint3& v, float deg) {
				const float a = deg * 0.017453292f;
				return NiPoint3(v.x * std::cos(a) + v.y * std::sin(a), -v.x * std::sin(a) + v.y * std::cos(a), v.z);
			};
			const NiPoint3 f = g_params.identity ? rotYaw(main.fwd, g_params.offYaw) : Norm(MapDir(A, B, main.fwd));
			const NiPoint3 u = g_params.identity ? rotYaw(main.up, g_params.offYaw) : Norm(MapDir(A, B, main.up));
			const NiPoint3 rgt = g_params.identity ? rotYaw(main.right, g_params.offYaw) : Norm(MapDir(A, B, main.right));
			if (g_params.diag && (g_diagN++ % 120) == 0) {
				auto probe = [&](const char* a_name, const NiPoint3& a_p, bool a_fwdToo) {
					const auto up = world::Raycast(a_p, NiPoint3{ 0, 0, 1 }, 3000.0f, RE::COL_LAYER::kLOS);
					const auto dn = world::Raycast(a_p, NiPoint3{ 0, 0, -1 }, 3000.0f, RE::COL_LAYER::kLOS);
					const auto top = world::Raycast(NiPoint3{ a_p.x, a_p.y, a_p.z + 1500.0f }, NiPoint3{ 0, 0, -1 }, 3000.0f, RE::COL_LAYER::kLOS);
					const float surfaceZ = top.hit ? a_p.z + 1500.0f - top.dist : -1e9f;
					logger::info("view: diag {}: ({:.0f},{:.0f},{:.0f}) ray up {} {:.0f} [{}], ray down {} {:.0f} [{}], top surface over it z {:.0f} ({:+.0f} vs camera) [{}]", a_name, a_p.x, a_p.y, a_p.z,
						up.hit, up.dist, up.hit ? world::LayerNameOf(up.layer) : "-", dn.hit, dn.dist, dn.hit ? world::LayerNameOf(dn.layer) : "-", surfaceZ, surfaceZ - a_p.z, top.hit ? world::LayerNameOf(top.layer) : "-");
					(void)a_fwdToo;
				};
				probe("main camera   ", main.pos, false);
				probe("virtual camera", pos, true);
			}
			if (!g_cam) {
				g_cam = static_cast<std::uint8_t*>(RE::malloc(sizeof(RE::NiCamera)));
			}
			std::memcpy(g_cam, mainCam, sizeof(RE::NiCamera));
			auto* cam = reinterpret_cast<RE::NiCamera*>(g_cam);
			*reinterpret_cast<volatile std::uint32_t*>(g_cam + 0x08) = 0x40000000u;  // refcount: never freed
			cam->parent = nullptr;
			cam->world.translate = pos;
			cam->world.rotate.entry[0][0] = f.x; cam->world.rotate.entry[0][1] = u.x; cam->world.rotate.entry[0][2] = rgt.x;
			cam->world.rotate.entry[1][0] = f.y; cam->world.rotate.entry[1][1] = u.y; cam->world.rotate.entry[1][2] = rgt.y;
			cam->world.rotate.entry[2][0] = f.z; cam->world.rotate.entry[2][1] = u.z; cam->world.rotate.entry[2][2] = rgt.z;
			const auto full = cam->GetRuntimeData2().viewFrustum;  // full frustum is kept for rendering: the image stays aligned with the main view's screen
			reinterpret_cast<CamUpdate_t>(REL::ID(70641).address())(g_cam);
			if (g_params.vShadow) {
				VirtualShadowsBegin(g_cam);
			}

			// --- accumulator ---
			if (!g_acc) {
				g_acc = RE::BSShaderAccumulator::Create(0x63);
				if (!g_acc) {
					a_why = "accumulator creation failed";
					return false;
				}
				*reinterpret_cast<volatile std::uint32_t*>(reinterpret_cast<std::uint8_t*>(g_acc) + 0x08) = 0x40000000u;
			}
			auto* acc = reinterpret_cast<std::uint8_t*>(g_acc);

			// --- render targets, as the engine does for a cubemap face: the main HDR colour and the main depth ---
			ComPtr<ID3D11DeviceContext> d3d;
			Device()->GetImmediateContext(&d3d);
			auto* mgr = MgrAddr();
			auto* rend = RendererAddr();
			reinterpret_cast<SetRT_t>(REL::ID(77453).address())(mgr, 0, g_params.rt, g_params.mode, false);
			for (std::uint32_t slot = 1; slot < 8; ++slot) {
				reinterpret_cast<SetRT_t>(REL::ID(77453).address())(mgr, slot, -1, g_params.mode, false);
			}
			reinterpret_cast<UpdateVP_t>(REL::ID(77240).address())(rend, 0, 0, true);
			reinterpret_cast<SetDS_t>(REL::ID(77454).address())(mgr, g_params.ds, g_params.mode, 0, false);
			reinterpret_cast<ApplyState_t>(REL::ID(77247).address())(rend, false);
			PokeBeforeBind();
			ComPtr<ID3D11RenderTargetView> rtv;
			ComPtr<ID3D11DepthStencilView> dsv;
			d3d->OMGetRenderTargets(1, &rtv, &dsv);
			g_liveDsv = dsv.Get();
			if (g_params.maskStat > 0) {
				--g_params.maskStat;
				LogMaskStats(d3d.Get(), "at the start of the virtual pass");
			}
			if (!rtv || !dsv) {
				a_why = "engine did not bind a colour or depth target";
				return false;
			}
			const FLOAT col[4] = { g_clearCol[0].load(), g_clearCol[1].load(), g_clearCol[2].load(), 1.0f };  // view clearcol=r,g,b (G240): the colour the virtual pass starts from (it shows through any blended sky pixel)
			d3d->ClearRenderTargetView(rtv.Get(), col);
			d3d->ClearDepthStencilView(dsv.Get(), D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);

			// --- oblique near plane: armed before the engine builds the camera matrices ---
			// Camera behind the entry plane (doorway): the mapped camera is in front of the exit plane, looking away from it, so nothing needs clipping
			NiPoint3 oblN = B.n;
			float    oblD = Dot(B.n, pos - B.c) + g_params.planeBias;
			bool     exitClip = Dot(B.n, pos - B.c) <= 0.5f;
			if (g_params.oblForce > 0.0f) {  // synthetic plane for the identity test: the kept side is beyond it, its normal 20 degrees off the view axis
				oblN = Norm(f * 0.9397f + rgt * 0.3420f);
				oblD = -g_params.oblForce * 0.9397f;
				exitClip = true;
			}
			g_obl = Oblique{}; g_oblSaved.clear();
			if (g_params.oblique && exitClip) {
				g_obl.active = true;
				g_obl.n = oblN;
				g_obl.dRel = oblD;
				g_obl.fwd = f; g_obl.up = u; g_obl.right = rgt;
				g_obl.tl = full.fLeft; g_obl.tr = full.fRight; g_obl.tb = full.fBottom; g_obl.tt = full.fTop;
				g_obl.nearD = full.fNear; g_obl.farD = full.fFar;
			}
			const int applied0 = g_oblApplied;

			// --- start accumulating with our camera ---
			auto** accVt = *reinterpret_cast<void***>(acc);
			reinterpret_cast<StartAcc_t>(accVt[0x25])(acc, g_cam);
			SetAccFields(acc, pos);
			if (g_params.eqz >= 0) {
				acc[0x5C] = static_cast<std::uint8_t>(g_params.eqz);
			}
			if (g_params.diag && g_diagN < 3) {
				logger::info("view: diag accumulator bytes: +5C (equal depth test) {} +B8 {} +B9 {} +BA {} +128 {} +129 {} +12A {} +12B {} +12C (decals) {} +12D {} +12E {}; State::useEarlyZ {}",
					acc[0x5C], acc[0xB8], acc[0xB9], acc[0xBA], acc[0x128], acc[0x129], acc[0x12A], acc[0x12B], acc[0x12C], acc[0x12D], acc[0x12E], RE::BSGraphics::State::GetSingleton()->GetUseEarlyZ());
			}

			// --- lighting inputs of the virtual view that the engine computes later in the frame for the main view ---
			if (g_params.shadowMask >= 0) {
				if (auto* rtvMask = reinterpret_cast<ID3D11RenderTargetView*>(rd->renderTargets[RE::RENDER_TARGETS::kSHADOW_MASK].RTV)) {
					const float v = static_cast<float>(g_params.shadowMask);
					const FLOAT c[4] = { v, v, v, v };
					d3d->ClearRenderTargetView(rtvMask, c);
				}
			}
			if (g_params.skyMove) {
				ShiftSky(pos - main.pos);
			}

			// --- custom cull planes: the pyramid through the portal's screen rectangle, and the exit portal's plane as the near plane ---
			alignas(16) std::uint8_t planeBuf[0x70]{};
			const bool               useCustom = g_params.cone || g_params.exitPlane;
			if (useCustom) {
				float l = full.fLeft, rr = full.fRight, b = full.fBottom, t = full.fTop;
				if (g_params.cone) {
					const float mx = 0.04f * (a_r - a_l), my = 0.04f * (a_t - a_b);
					l = a_l - mx; rr = a_r + mx; b = a_b - my; t = a_t + my;
				}
				// local (x forward, y up, z right) inside-normals: left (-l,0,1), right (rr,0,-1), bottom (-b,1,0), top (t,-1,0)
				auto toWorld = [&](float x, float y, float z) { return Norm(f * x + u * y + rgt * z); };
				NiPoint3 nrm[6] = { B.n, f * -1.0f, toWorld(-l, 0, 1), toWorld(rr, 0, -1), toWorld(t, -1, 0), toWorld(-b, 1, 0) };
				float    d[6];
				for (int i = 0; i < 6; ++i) {
					d[i] = Dot(nrm[i], pos);
				}
				if (g_params.exitPlane && exitClip) {
					d[0] = Dot(B.n, B.c) - g_params.planeBias;  // exit plane (a few units into the surface)
				} else {
					d[0] = Dot(f, pos) + full.fNear;  // plain near plane
					nrm[0] = f;
				}
				d[1] -= full.fFar;
				for (int i = 0; i < 6; ++i) {
					const float rec[4] = { nrm[i].x, nrm[i].y, nrm[i].z, d[i] };
					std::memcpy(planeBuf + i * 16, rec, 16);
				}
				const std::uint32_t active = 0x01 | (g_params.cone ? (0x04 | 0x08 | 0x10 | 0x20) : 0);  // near, left, right, top, bottom (no far)
				std::memcpy(planeBuf + 0x60, &active, 4);
				std::memcpy(planeBuf + 0x64, &active, 4);
			}

			if (g_npcBoundsLeft > 0) {
				--g_npcBoundsLeft;
				travel::LogNpcBounds(a_index == 0 ? "virtual pass of portal 0" : "virtual pass of portal 1", useCustom ? reinterpret_cast<const float*>(planeBuf) : nullptr, pos);
			}
			if (g_params.skinFix) {
				travel::ResetSkinCaches();
			}
			// --- cull the shadow scene node (it holds the sky, the weather, LOD and all loaded cells) into an accumulator ---
			auto cullInto = [&](void* a_acc) {
				alignas(16) std::uint8_t ctx[0x68]{};
				reinterpret_cast<CtxCtor_t>(REL::ID(106919).address())(ctx);
				*reinterpret_cast<void**>(ctx + 0x00) = a_acc;
				*reinterpret_cast<void**>(ctx + 0x10) = g_cam;
				*reinterpret_cast<void**>(ctx + 0x18) = nullptr;
				*reinterpret_cast<void**>(ctx + 0x20) = &cam->GetRuntimeData2().viewFrustum;
				*reinterpret_cast<void**>(ctx + 0x28) = nullptr;
				*reinterpret_cast<void**>(ctx + 0x30) = nullptr;
				*reinterpret_cast<void**>(ctx + 0x38) = useCustom ? planeBuf : nullptr;
				*reinterpret_cast<void**>(ctx + 0x40) = scene;
				*reinterpret_cast<void**>(ctx + 0x48) = nullptr;
				*reinterpret_cast<std::uint32_t*>(ctx + 0x50) = static_cast<std::uint32_t>(g_cullFlags.load());  // 3 = kIgnoreMultiBounds, as for the cubemap
				ctx[0x5E] = g_params.ignorePre ? 1 : 0;             // ignorePreprocess: actors are tested like any other node, not by the main camera's visibility flag
				ctx[0x5F] = useCustom ? 1 : 0;                      // doCustomCullPlanes
				ctx[0x60] = 1;                                      // cameraRelatedUpdates
				g_inCull = true;
				reinterpret_cast<Cull_t>(REL::ID(106921).address())(ctx, 0, 0);
				g_inCull = false;
			};
			const double tc = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
			std::uint32_t* lightFlagPtr = nullptr;
			std::uint32_t  lightFlagOld = 0;
			if (g_params.lightFlag) {
				const std::uint8_t* obj = nullptr;
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(REL::ID(415031).address()), obj);
				if (obj) {
					lightFlagPtr = reinterpret_cast<std::uint32_t*>(const_cast<std::uint8_t*>(obj) + 0xF4);
					lightFlagOld = *lightFlagPtr;
					*lightFlagPtr = lightFlagOld | 1u;
				}
			}
			// --- depth pre-pass (G120): the engine draws the world twice, first into depth only with a second accumulator in render mode 12 (Start flags 0x21, Submit flags 3: the utility
			// shader, alpha tests included; the tree LOD cards and the grass get their depth techniques there), then the colour pass with the equal-depth test ---
			bool prepassDone = false;
			if (g_params.prepass) {
				if (!g_accPre) {
					g_accPre = RE::BSShaderAccumulator::Create(0x63);
					if (g_accPre) {
						*reinterpret_cast<volatile std::uint32_t*>(reinterpret_cast<std::uint8_t*>(g_accPre) + 0x08) = 0x40000000u;
					}
				}
				if (g_accPre) {
					auto*  pre = reinterpret_cast<std::uint8_t*>(g_accPre);
					auto** preVt = *reinterpret_cast<void***>(pre);
					if (g_params.preStart != 0) {
						reinterpret_cast<Submit_t>(REL::ID(106437).address())(g_cam, pre, g_params.preStart);
					} else {
						reinterpret_cast<StartAcc_t>(preVt[0x25])(pre, g_cam);
					}
					SetAccFields(pre, pos);
					*reinterpret_cast<std::uint32_t*>(pre + 0x150) = g_params.preMode;
					pre[0x5C] = 0;
					cullInto(pre);
					if (g_params.preUnbind) {
						for (std::uint32_t slot = 0; slot < 3; ++slot) {
							reinterpret_cast<SetRT_t>(REL::ID(77453).address())(mgr, slot, -1, g_params.mode, true);
						}
					}
					PokeBeforeSubmit(2);
					const int stPre = StatBegin(d3d.Get(), a_index, "virtual depth pre-pass");
					reinterpret_cast<Submit_t>(REL::ID(106436).address())(g_cam, pre, g_params.preFlags);
					StatEnd(d3d.Get(), stPre);
					PokeBeforeSubmit(1);
					if (g_coverLogged < 2) {
						ReadDepthSamples(d3d.Get(), g_depthPre);
						logger::info("view: pre-pass (start {:#x}, submit {:#x}, unbind {}, restore {}): depth target holds geometry in {:.3f} of the screen", g_params.preStart, g_params.preFlags, g_params.preUnbind, g_params.preRestore, DepthCoverage(d3d.Get()));
					}
					// back to the colour set-up of this pass (the same calls as before the clear)
					if (g_params.preRestore) {
					reinterpret_cast<SetRT_t>(REL::ID(77453).address())(mgr, 0, g_params.rt, g_params.mode, false);
					for (std::uint32_t slot = 1; slot < 8; ++slot) {
						reinterpret_cast<SetRT_t>(REL::ID(77453).address())(mgr, slot, -1, g_params.mode, false);
					}
					reinterpret_cast<UpdateVP_t>(REL::ID(77240).address())(rend, 0, 0, true);
					reinterpret_cast<SetDS_t>(REL::ID(77454).address())(mgr, g_params.ds, g_params.mode, 0, false);
					reinterpret_cast<ApplyState_t>(REL::ID(77247).address())(rend, false);
					PokeBeforeBind();
					}
					if (g_params.preRebind) {
						reinterpret_cast<SetRT_t>(REL::ID(77453).address())(mgr, 0, g_params.rt, 4, false);
						reinterpret_cast<ApplyState_t>(REL::ID(77247).address())(rend, false);
					}
					d3d->ClearRenderTargetView(rtv.Get(), col);  // whatever the pre-pass drew into the colour target goes; its depth stays
					if (g_params.maskPass) {
						// the engine's own shadow-mask stage (id 107140) with the virtual camera: clear value, camera state, current camera, the mask draw, state finish
						auto* rendererObj = reinterpret_cast<void*>(REL::ID(411393).address());
						const bool lowQuality = *reinterpret_cast<const std::uint8_t*>(REL::ID(415040).address()) != 0;
						using ClearCol_t = void (*)(void*, float, float, float, float);
						using SetCam_t = void (*)(void*, void*, std::uint8_t);
						using Void_t = void (*)();
						using Fin_t2 = void (*)(void*);
						if (g_params.maskStat > 0) {
							LogMaskStats(d3d.Get(), "before the virtual mask pass");
						}
						reinterpret_cast<ClearCol_t>(REL::ID(77249).address())(rendererObj, lowQuality ? 0.0f : 1.0f, 0.0f, 0.0f, 0.0f);
						reinterpret_cast<SetCam_t>(REL::ID(77503).address())(reinterpret_cast<void*>(REL::ID(411479).address()), g_cam, 0);
						void** curCam = reinterpret_cast<void**>(REL::ID(414580).address());
						void*  savedCam = *curCam;
						*curCam = g_cam;
						static int s_maskLogged = 0;
						const bool maskOn = *reinterpret_cast<const std::uint8_t*>(REL::ID(390933).address()) != 0;
						if (s_maskLogged++ < 3) {
							logger::info("view: mask pass: settings byte id390933 = {}, id415035 = {}, low quality byte id415040 = {}", *reinterpret_cast<const std::uint8_t*>(REL::ID(390933).address()), *reinterpret_cast<const std::int32_t*>(REL::ID(415035).address()), lowQuality);
						}
						if (g_params.maskStat > 0) {
							LogMaskStats(d3d.Get(), "after the clear, before the mask stage");
						}
						if (g_maskReplay.load() && !(g_vShadowActive && (g_vsMode.load() & 8))) {
							float rows[16]{};
							bool  haveRows = false;
							if (g_ourVPValid) {
								float inv[16]{};
								if (Invert4(g_ourVP, inv)) {
									for (int k = 0; k < 4; ++k) {
										for (int j = 0; j < 4; ++j) rows[k * 4 + j] = inv[j * 4 + k];  // row k of the shader's matrix = column k of the row-vector inverse
									}
									if (auto* mcam = RE::Main::WorldRootCamera()) {
										const float d[3]{ pos.x - mcam->world.translate.x, pos.y - mcam->world.translate.y, pos.z - mcam->world.translate.z };
										for (int k = 0; k < 3; ++k) {
											for (int j = 0; j < 4; ++j) rows[k * 4 + j] += d[k] * rows[12 + j];  // positions are relative to the MAIN camera in the cascade matrices
										}
									}
									haveRows = true;
									static int s_rowsLogged = 0;
									if (s_rowsLogged++ < 4) {
										logger::info("view: mask replay rows 32-35 for the virtual camera: ({:.5f} {:.5f} {:.5f} {:.5f}) ({:.5f} {:.5f} {:.5f} {:.5f}) ({:.5f} {:.5f} {:.5f} {:.5f}) ({:.5f} {:.5f} {:.5f} {:.5f})", rows[0], rows[1], rows[2], rows[3], rows[4], rows[5], rows[6], rows[7], rows[8], rows[9], rows[10], rows[11], rows[12], rows[13], rows[14], rows[15]);
									}
								}
							}
							float casc[40]{};  // 9 rows of cb2 (16-24) + the biases
							bool  haveCasc = false;
							if (g_vShadowActive && g_params.vShadow >= 3 && (g_vsMode.load() & 256) && !(g_vsMode.load() & 8)) {
								haveCasc = BuildCascadeRows(casc);
							}
							const bool okr = MaskReplayRun(d3d.Get(), haveRows ? rows : nullptr, haveCasc ? casc : nullptr);
							static int s_rl = 0;
							if (s_rl++ < 6) {
								logger::info("view: mask replay {}", okr ? "ran" : "did not run (no captured engine mask draw yet or a missing input)");
							}
						} else if (maskOn) {
							VsCrumb("mask stage 107141 entering");
							g_inMaskStage = true;
							reinterpret_cast<Void_t>(REL::ID(107141).address())();
							g_inMaskStage = false;
							VsCrumb("mask stage 107141 returned");
							VsFence("mask stage 107141 (virtual mask drawn)");
						}
						*curCam = savedCam;
						reinterpret_cast<Fin_t2>(REL::ID(77250).address())(rendererObj);
						VirtualShadowsEnd();
						VsCrumb("End done (mask block)");
						if (g_params.maskStat > 0) {
							LogMaskStats(d3d.Get(), "after the virtual mask pass");
						}
						// the mask stage bound its own targets: colour and depth back without clearing
						reinterpret_cast<SetRT_t>(REL::ID(77453).address())(mgr, 0, g_params.rt, 4, false);
						for (std::uint32_t slot = 1; slot < 8; ++slot) {
							reinterpret_cast<SetRT_t>(REL::ID(77453).address())(mgr, slot, -1, 4, false);
						}
						reinterpret_cast<UpdateVP_t>(REL::ID(77240).address())(rend, 0, 0, true);
						reinterpret_cast<SetDS_t>(REL::ID(77454).address())(mgr, g_params.ds, 4, 0, false);
						reinterpret_cast<ApplyState_t>(REL::ID(77247).address())(rend, false);
					}
					prepassDone = true;
					acc[0x5C] = static_cast<std::uint8_t>(g_params.eqz >= 0 ? g_params.eqz : 1);  // the colour pass tests depth EQUAL against the pre-pass
				}
			}
			if (g_maskClear >= 0.0f) {
				if (auto* rdm = RE::BSGraphics::Renderer::GetRendererDataSingleton(); rdm && rdm->renderTargets[18].texture) {
					auto*                tm = reinterpret_cast<ID3D11Texture2D*>(rdm->renderTargets[18].texture);
					D3D11_TEXTURE2D_DESC md{};
					tm->GetDesc(&md);
					static int s_maskLogged = 0;
					if (s_maskLogged++ < 3) {
						logger::info("view: maskclear {}: shadow mask target 18 {}x{} format {}, render target view {}", g_maskClear, md.Width, md.Height, static_cast<int>(md.Format), rdm->renderTargets[18].RTV ? "present" : "MISSING");
					}
					if (md.Format == DXGI_FORMAT_R8G8B8A8_UNORM) {
						static std::vector<std::uint32_t> s_maskFill;
						s_maskFill.assign(static_cast<std::size_t>(md.Width) * md.Height, g_maskClear >= 0.5f ? 0x000000FFu : 0x00000000u);  // R = 1 or 0, the other channels 0
			if (g_maskClear >= 1.5f) {  // maskclear=2: a pattern, R = 0 on the left half of the screen, 1 on the right half (vertical bands of 120 px beyond the middle)
				for (UINT y = 0; y < md.Height; ++y) {
					for (UINT x = 0; x < md.Width; ++x) {
						s_maskFill[static_cast<std::size_t>(y) * md.Width + x] = (x < md.Width / 2) ? 0x00000000u : 0x000000FFu;
					}
				}
			}
						d3d->UpdateSubresource(tm, 0, nullptr, s_maskFill.data(), md.Width * 4, 0);
					}
				}
			}
			if (g_maskStat2 > 0) {
				LogMaskStats(d3d.Get(), "after the maskclear, before our colour pass");
			}
			if (g_normClear) {
				NormalsSaveAndFlatten(d3d.Get());
			}
			cullInto(acc);
			// (G85: culling each actor's own root into this pass drew nothing either, and the game then hung after the guard was moved: removed.)
			const double ts = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
			if (g_mrt) {
				using GetNormIdx_t = std::int32_t (*)();
				const std::int32_t nIdx = reinterpret_cast<GetNormIdx_t>(REL::ID(107177).address())();
				auto               setRtMrt = reinterpret_cast<SetRT_t>(REL::ID(77453).address());
				setRtMrt(mgr, 1, 7, 3, true);
				setRtMrt(mgr, 2, nIdx, 3, true);
				const bool snowOn = *reinterpret_cast<const std::uint8_t*>(REL::ID(391105).address()) != 0;
				if (snowOn) {
					setRtMrt(mgr, 3, 0x70, 3, true);
				}
				static int s_mrtLogged = 0;
				if (s_mrtLogged++ < 3) {
					logger::info("view: colour pass with the engine's target set: slot 1 = 7, slot 2 = {}, slot 3 = {}", nIdx, snowOn ? 112 : -1);
				}
			}
			g_litSeq = 0;
			PokeBeforeSubmit(2);
			const int stWorld = StatBegin(d3d.Get(), a_index, "virtual world pass");
			if (g_params.ssao) {
				reinterpret_cast<Submit_t>(REL::ID(106437).address())(g_cam, acc, g_params.submitFlags);  // Renderer::StartAccumulating: camera, accumulator start, the opaque dispatch
				DeObliqueBegin(d3d.Get());
				AoOwnBegin(d3d.Get());
				g_ssaoCapVirt = true;
				RunVirtualSsao(mgr, d3d.Get(), a_index);
				g_ssaoCapVirt = false;
				const bool blendStd = DeObliqueEnd(d3d.Get(), g_blendStd.load() != 0);
				AoOwnEnd(d3d.Get());
				g_inFinishPhase = true;
				g_finishBuilds = 0;
				if (blendStd) {
					// make sure the constants of the blended parts come from an UNPATCHED build of our camera (the entries were put back above, g_obl.active is false): the same call the shadow stages use
					reinterpret_cast<void (*)(void*, void*, std::uint8_t)>(REL::ID(77503).address())(reinterpret_cast<void*>(REL::ID(411479).address()), g_cam, (g_params.submitFlags & 1u) ? 1 : 0);
				}
				reinterpret_cast<Submit_t>(REL::ID(106438).address())(g_cam, acc, g_params.submitFlags);  // FinishAccumulatingPostResolveDepth: the blended parts
				g_inFinishPhase = false;
				if (g_vlRun.load() != 0) {
					if (a_index == 0 && g_vlDefer.allowed && g_vlDeferMode.load() != 0) {
						g_vlDefer.want = true;  // after view 1's pass (RunDeferredVl)
					} else {
						RunVlInPass(mgr, d3d.Get(), a_index, mainTex);
						d3d->OMSetRenderTargets(1, rtv.GetAddressOf(), dsv.Get());
					}
				}
				static int s_blendLogged = 0;
				if (s_blendLogged++ < 4) {
					logger::info("view: blended parts {} ({} camera build(s) during the Finish call)", blendStd ? "ran with the standard projection and the standard depth encoding" : "ran as before (oblique projection, oblique depth)", g_finishBuilds);
				}
			} else {
				reinterpret_cast<Submit_t>(REL::ID(106436).address())(g_cam, acc, g_params.submitFlags);
			}
			StatEnd(d3d.Get(), stWorld);
			NormalsRestore(d3d.Get());
			if (g_maskStat2 > 0) {
				--g_maskStat2;
				LogMaskStats(d3d.Get(), "after our colour pass (Start, SSAO, Finish)");
			}
			if (g_mrt) {
				for (std::uint32_t slot = 1; slot < 4; ++slot) {
					reinterpret_cast<SetRT_t>(REL::ID(77453).address())(mgr, slot, -1, g_params.mode, false);
				}
			}
			PokeBeforeSubmit(1);
			if (prepassDone && g_coverLogged < 2) {
				++g_coverLogged;
				logger::info("view: after the colour pass of a pre-pass view: depth target holds geometry in {:.3f} of the screen", DepthCoverage(d3d.Get()));
				std::vector<float> after;
				if (!g_depthPre.empty() && ReadDepthSamples(d3d.Get(), after) && after.size() == g_depthPre.size()) {
					std::size_t changed = 0, closer = 0, farther = 0, drawnPre = 0;
					double maxDiff = 0, sumRel = 0;
					for (std::size_t i = 0; i < after.size(); ++i) {
						if (g_depthPre[i] < 0.999999f) ++drawnPre;
						if (after[i] != g_depthPre[i]) {
							++changed;
							(after[i] < g_depthPre[i] ? closer : farther)++;
							maxDiff = (std::max)(maxDiff, static_cast<double>(std::fabs(after[i] - g_depthPre[i])));
							sumRel += std::fabs(after[i] - g_depthPre[i]) / (std::max)(1e-9f, g_depthPre[i]);
						}
					}
					logger::info("view: depth of the colour pass against the pre-pass over {} samples ({} with geometry in the pre-pass): {} changed ({} closer, {} farther), max change {:.3e}, mean relative change of the changed {:.3e}", after.size(), drawnPre, changed, closer, farther, maxDiff, changed ? sumRel / static_cast<double>(changed) : 0.0);
				}
			}
			if (prepassDone) {
				acc[0x5C] = static_cast<std::uint8_t>(g_params.eqz >= 0 ? g_params.eqz : 0);  // the body and exit passes draw with this accumulator without a pre-pass
			}
			if (lightFlagPtr) {
				*lightFlagPtr = lightFlagOld;
			}
			g_obl.active = false;
			if (exitClip) {
				travel::LogSkin("after the virtual world pass");
				const double tb0 = NowMs();
				const int    stBody = StatBegin(d3d.Get(), a_index, "body pass");
				if (RenderBody(A, main, acc, full)) {
					g_costBody.Add("body pass into the virtual picture", NowMs() - tb0);
				}
				StatEnd(d3d.Get(), stBody);
			}
			const double te = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();

			if (g_depthPending.load()) {
				DumpDepth(a_index, d3d.Get(), main);
			}

			RestoreSky();
			VsCrumb("colour pass finished");
			VirtualShadowsEnd();
			VsFence("virtual view colour pass finished");

			if (g_vlDefer.want && a_index == 0) {
				g_vlDefer.want = false;
				ComPtr<ID3D11Resource> liveRes;
				if (g_liveDsv) g_liveDsv->GetResource(&liveRes);
				ComPtr<ID3D11Texture2D> live;
				D3D11_TEXTURE2D_DESC    ld{};
				if (liveRes && SUCCEEDED(liveRes.As(&live))) {
					live->GetDesc(&ld);
					D3D11_TEXTURE2D_DESC cd{};
					if (g_vlDefer.depth) g_vlDefer.depth->GetDesc(&cd);
					if (!g_vlDefer.depth || cd.Width != ld.Width || cd.Height != ld.Height || cd.Format != ld.Format) {
						D3D11_TEXTURE2D_DESC nd = ld;
						nd.BindFlags = 0;
						nd.MiscFlags = 0;
						g_vlDefer.depth.Reset();
						Device()->CreateTexture2D(&nd, nullptr, &g_vlDefer.depth);
					}
					if (g_vlDefer.depth) {
						d3d->CopyResource(g_vlDefer.depth.Get(), liveRes.Get());
						g_vlDefer.cam.assign(g_cam, g_cam + sizeof(RE::NiCamera));
						g_vlDefer.pending = true;
					}
				}
				if (!g_vlDefer.pending) {
					// the copies could not be made: run the VL here after all
					RunVlInPass(mgr, d3d.Get(), a_index, mainTex);
					d3d->OMSetRenderTargets(1, rtv.GetAddressOf(), dsv.Get());
				}
			}
			// --- keep the picture before the main view overwrites the target ---
			d3d->CopyResource(g_tex[a_index].Get(), mainTex);
			if (exitClip) {
				const double te0 = NowMs();
				const int    stExit = StatBegin(d3d.Get(), a_index, "exit pass");
				if (RenderExit(a_index, A, B, main, pos, f, u, rgt, full, acc, d3d.Get(), rtv.Get(), dsv.Get(), mainTex)) {
					g_costExit.Add("exit picture pass (real-camera front part + virtual-camera back part + copies)", NowMs() - te0);
				}
				StatEnd(d3d.Get(), stExit);
			}
			travel::LogSkin("after the body and exit passes");
			g_lastCullMs = ts - tc;
			g_lastSubmitMs = te - ts;
			g_lastTotalMs = te - t0;
			++g_passes;
			if (g_passes <= 6 || (g_passes % 600) == 0) {
				logger::info("view: pass {} portal {} virtual cam ({:.1f},{:.1f},{:.1f}) fwd ({:.3f},{:.3f},{:.3f}) rect l{:.3f} r{:.3f} b{:.3f} t{:.3f}; cull {:.3f} ms submit {:.3f} ms total {:.3f} ms; oblique matrices patched {} time(s) this pass",
					g_passes, a_index, pos.x, pos.y, pos.z, f.x, f.y, f.z, a_l, a_r, a_b, a_t, g_lastCullMs, g_lastSubmitMs, g_lastTotalMs, g_oblApplied - applied0);
			}
			return true;
		}

		struct SafeArgs
		{
			int                    index;
			const portals::Portal* A;
			const portals::Portal* B;
			const world::CamInfo*  cam;
			float                  l, r, b, t;
			bool                   ok;
			std::string            why;
		};

		void RunPassPlain(SafeArgs* a)
		{
			a->ok = RunPass(a->index, *a->A, *a->B, *a->cam, a->l, a->r, a->b, a->t, a->why);
		}

		LONG LogException(EXCEPTION_POINTERS* a_ep)
		{
			const auto base = REL::Module::get().base();
			logger::error("view: EXCEPTION {:#x} at {:#x} (exe rva {:#x}), access address {:#x}", a_ep->ExceptionRecord->ExceptionCode,
				reinterpret_cast<std::uintptr_t>(a_ep->ExceptionRecord->ExceptionAddress), reinterpret_cast<std::uintptr_t>(a_ep->ExceptionRecord->ExceptionAddress) - base,
				a_ep->ExceptionRecord->NumberParameters > 1 ? static_cast<std::uint64_t>(a_ep->ExceptionRecord->ExceptionInformation[1]) : 0);
			return EXCEPTION_EXECUTE_HANDLER;
		}

		bool RunPassGuarded(SafeArgs* a)
		{
			__try {
				RunPassPlain(a);
				return true;
			} __except (LogException(GetExceptionInformation())) {
				RestoreSky();
				VirtualShadowsAbort();
				return false;
			}
		}

		// view 0's VL, run after view 1's pass (see VlDefer)
		void RunDeferredVl()
		{
			if (!g_vlDefer.pending) {
				return;
			}
			g_vlDefer.pending = false;
			auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			auto* dsd = &RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData().depthStencils[g_params.ds];
			if (!rd || !dsd || !g_tex[0] || !g_vlDefer.depth || g_vlDefer.cam.size() != sizeof(RE::NiCamera) || !g_cam) {
				return;
			}
			auto*                          mainTex = reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[g_params.rt].texture);
			auto*                          rtvT = reinterpret_cast<ID3D11RenderTargetView*>(rd->renderTargets[g_params.rt].RTV);
			auto*                          dsvT = reinterpret_cast<ID3D11DepthStencilView*>(dsd->views[0]);
			if (!mainTex || !rtvT || !dsvT) {
				return;
			}
			ComPtr<ID3D11DeviceContext> d3d;
			Device()->GetImmediateContext(&d3d);
			ComPtr<ID3D11Resource> liveRes;
			dsvT->GetResource(&liveRes);
			if (!liveRes) {
				return;
			}
			// view 0's pre-VL picture, depth and camera back in place
			d3d->CopyResource(mainTex, g_tex[0].Get());
			d3d->CopyResource(liveRes.Get(), g_vlDefer.depth.Get());
			std::memcpy(g_cam, g_vlDefer.cam.data(), g_vlDefer.cam.size());
			g_liveDsv = dsvT;
			auto* mgr = MgrAddr();
			auto* rend = RendererAddr();
			reinterpret_cast<SetRT_t>(REL::ID(77453).address())(mgr, 0, g_params.rt, g_params.mode, false);
			for (std::uint32_t slot = 1; slot < 8; ++slot) {
				reinterpret_cast<SetRT_t>(REL::ID(77453).address())(mgr, slot, -1, g_params.mode, false);
			}
			reinterpret_cast<UpdateVP_t>(REL::ID(77240).address())(rend, 0, 0, true);
			reinterpret_cast<SetDS_t>(REL::ID(77454).address())(mgr, g_params.ds, g_params.mode, 0, false);
			reinterpret_cast<ApplyState_t>(REL::ID(77247).address())(rend, false);
			d3d->OMSetRenderTargets(1, &rtvT, dsvT);
			PokeBeforeSubmit(2);
			const bool addD = g_vlAdd.load() != 0 && g_vlPre[0] && g_vlPost[0];
			if (addD) {
				d3d->CopyResource(g_vlPre[0].Get(), g_tex[0].Get());
			}
			RunVirtualVl(mgr, d3d.Get(), 0);
			PokeBeforeSubmit(1);
			d3d->OMSetRenderTargets(1, &rtvT, dsvT);
			if (g_vlLastRan) {
				if (addD) {
					d3d->CopyResource(g_vlPost[0].Get(), mainTex);
					g_vlAddValid[0] = true;
				} else {
					d3d->CopyResource(g_tex[0].Get(), mainTex);  // view 0's picture, now with its volumetric lighting
				}
			}
			static int s_logged = 0;
			if (s_logged++ < 4) {
				logger::info("view: deferred VL of view 0 run after view 1's pass");
			}
		}
	}

	Params& P()
	{
		return g_params;
	}

	bool Install()
	{
		const bool ok = hooks::Install(kBuild, reinterpret_cast<void*>(&Hook_Build), reinterpret_cast<void**>(&g_origBuild));
		hooks::Install(kVlComposite, reinterpret_cast<void*>(&Hook_VlComposite), reinterpret_cast<void**>(&g_origVlComposite));
		InstallPassHooks();
		return ok;
	}

	void RequestStageDump(const std::string& a_label, int a_stage)
	{
		EnsureTraceHooks();
		g_dumpStageLabel = a_label;
		g_dumpStage = a_stage;
	}

	void ResetFrame()
	{
		g_valid[0] = g_valid[1] = false;
		g_vlAddValid[0] = g_vlAddValid[1] = false;
		g_p2valid[0] = g_p2valid[1] = g_p2valid[2] = g_p2valid[3] = false;
	}

	void InstallEarlyDeviceHook()
	{
		auto* base = reinterpret_cast<std::uint8_t*>(GetModuleHandleW(nullptr));
		auto* dos = reinterpret_cast<IMAGE_DOS_HEADER*>(base);
		auto* nt = reinterpret_cast<IMAGE_NT_HEADERS64*>(base + dos->e_lfanew);
		const auto& dir = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
		if (!dir.VirtualAddress) {
			logger::warn("view: no import table, the early device hook is not installed");
			return;
		}
		for (auto* imp = reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR*>(base + dir.VirtualAddress); imp->Name; ++imp) {
			if (_stricmp(reinterpret_cast<const char*>(base + imp->Name), "d3d11.dll") != 0) {
				continue;
			}
			auto* oft = reinterpret_cast<IMAGE_THUNK_DATA64*>(base + imp->OriginalFirstThunk);
			auto* ft = reinterpret_cast<IMAGE_THUNK_DATA64*>(base + imp->FirstThunk);
			for (; oft->u1.AddressOfData; ++oft, ++ft) {
				if (oft->u1.Ordinal & IMAGE_ORDINAL_FLAG64) {
					continue;
				}
				auto* ibn = reinterpret_cast<IMAGE_IMPORT_BY_NAME*>(base + oft->u1.AddressOfData);
				if (std::strcmp(ibn->Name, "D3D11CreateDeviceAndSwapChain") != 0) {
					continue;
				}
				DWORD old = 0;
				if (VirtualProtect(&ft->u1.Function, sizeof(void*), PAGE_READWRITE, &old)) {
					g_origCreateDevSC = reinterpret_cast<CreateDevSC_t>(ft->u1.Function);
					ft->u1.Function = reinterpret_cast<ULONGLONG>(&Hook_CreateDevSC);
					VirtualProtect(&ft->u1.Function, sizeof(void*), old, &old);
					logger::info("view: D3D11CreateDeviceAndSwapChain import patched (original {:p})", reinterpret_cast<void*>(g_origCreateDevSC));
					return;
				}
			}
		}
		logger::warn("view: D3D11CreateDeviceAndSwapChain is not in the import table of the executable; the early device hook is not installed");
	}

	void LogSunShadow(const char* a_label)
	{
		SunShadowLogImpl(a_label);
	}

	void PostLogBegin()
	{
		if (g_postLogFrames.load() > 0) {
			--g_postLogFrames;
			g_postLogDraws = 0;
			g_postLogOn = true;
			logger::info("postlog: ===== frame {}: after overlay::Draw, the engine's later draws follow (totals so far: {} draws, {} render target sets)", frame::Count(), g_drawTotal.load(), g_rtSetTotal.load());
			{
				const std::uint8_t* ismp = nullptr;
				ReadSafeT(reinterpret_cast<const void*>(REL::ID(414660).address()), ismp);
				const std::uint8_t* vlm = reinterpret_cast<const std::uint8_t* (*)()>(REL::ID(107014).address())();
				std::uint32_t       m88 = 0;
				std::uint8_t        applyAct = 0, compAct = 0, g414913 = 0, g391215 = 0, g415052 = 0;
				const std::uint8_t *pa = nullptr, *pc = nullptr, *pb = nullptr;
				if (ismp) {
					ReadSafeT(ismp + 0x170, pa);
					ReadSafeT(ismp + 0x190, pc);
					ReadSafeT(ismp + 0x188, pb);
					if (pa) ReadSafeT(pa + 8, applyAct);
					if (pc) ReadSafeT(pc + 0x18, compAct);
				}
				if (vlm) ReadSafeT(vlm + 0x88, m88);
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(REL::ID(414913).address()), g414913);
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(REL::ID(391215).address()), g391215);
				ReadSafeT(reinterpret_cast<const std::uint8_t*>(REL::ID(415052).address()), g415052);
				logger::info("postlog: VL state: manager {:p} +0x88 = {:#x}; ISM apply shader {:p} active {}; composite shader {:p} flag(+0x18) {}; blur shader {:p}; globals 414913 = {}, 391215 = {}, 415052 = {}", static_cast<const void*>(vlm), m88, static_cast<const void*>(pa), applyAct, static_cast<const void*>(pc), compAct, static_cast<const void*>(pb), g414913, g391215, g415052);
			}
		}
	}
	// 0.1 for 1.6.1170: the camera state (id 411479) as raw qwords, to find its camera entry table (data pointer, count, entry stride) on a runtime whose layout differs
	void DumpCamState()
	{
		const auto* st = reinterpret_cast<const std::uint8_t*>(REL::ID(411479).address());
		std::string line;
		for (int o = 0; o < 0x140; o += 8) {
			std::uint64_t q = 0;
			ReadSafeT(st + o, q);
			line += std::format(" +{:#x}={:#x}", o, q);
		}
		logger::info("camstate: id 411479 at {:p}:{}", static_cast<const void*>(st), line);
		const void* wc = static_cast<const void*>(RE::Main::WorldRootCamera());
		logger::info("camstate: world root camera {:p}, our virtual camera {:p}", wc, static_cast<const void*>(g_cam));
		for (int o = 0x80; o < 0x120; o += 8) {
			std::uint64_t p = 0;
			ReadSafeT(st + o, p);
			if (p < 0x10000 || p > 0x7FFFFFFFFFFF) continue;
			for (int stride : { 0x270, 0x280, 0x290, 0x2A0, 0x2B0 }) {
				std::string ks;
				for (int i = 0; i < 6; ++i) {
					std::uint64_t k = 0;
					if (!ReadSafeT(reinterpret_cast<const std::uint8_t*>(p) + i * stride, k)) break;
					ks += std::format(" {:#x}", k);
				}
				logger::info("camstate: +{:#x} -> {:#x} stride {:#x}: first qwords{}", o, p, stride, ks);
			}
		}
	}

	void PostLogEnd()
	{
		if (g_postLogOn.load()) {
			g_postLogOn = false;
			logger::info("postlog: ===== Present: end after {} logged draws (totals now: {} draws, {} render target sets)", g_postLogDraws, g_drawTotal.load(), g_rtSetTotal.load());
		}
	}

	void RenderViews()
	{
		g_valid[0] = g_valid[1] = false;
		g_vlDefer.pending = false;
		g_vlDefer.want = false;
		g_vlDefer.allowed = false;
		g_p2valid[0] = g_p2valid[1] = g_p2valid[2] = g_p2valid[3] = false;
		++g_statFrame;
		EnsureDeviceHooks();
		TraceFrameBoundary();
		StatPoll();
		g_statOn = g_statLeft > 0;
		if (g_statLeft > 0) {
			--g_statLeft;
		}
		g_sunLogOn = g_sunLogLeft.load() > 0;
		if (g_sunLogOn) {
			--g_sunLogLeft;
		}
		g_ssaoCapOn = g_ssaoCapLeft.load() > 0;
		g_ssaoCapN[0] = g_ssaoCapN[1] = 0;
		if (g_ssaoCapOn) {
			--g_ssaoCapLeft;
		}
		travel::LogSkin("frame start");
		if (!g_params.enabled || g_disabled) {
			return;
		}
		const auto pb = portals::Get(0), po = portals::Get(1);
		if (!pb.valid || !po.valid) {
			return;
		}
		world::CamInfo cam;
		if (!world::CameraFull(cam)) {
			return;
		}
		// The camera read at the start of the frame is the one the previous frame was drawn with. The main view of this frame will use a camera one step
		// further along: extrapolate the last two readings (position linearly, orientation by repeating the last frame-to-frame rotation).
		const Pose raw{ cam.pos, cam.fwd, cam.up, cam.right };
		Pose       pred = raw;
		if (g_params.predict && g_havePrev) {
			const NiPoint3 dp = raw.pos - g_prevReading.pos;
			const float    cosA = std::clamp(Dot(raw.fwd, g_prevReading.fwd), -1.0f, 1.0f);
			if (std::sqrt(Dot(dp, dp)) < 300.0f && cosA > 0.866f) {  // not a teleport or a cut: below 30 degrees
				pred.pos = raw.pos + dp;
				// rotation delta D with D * M_prev = M_raw, M = [fwd up right] as columns: apply it once more
				const NiPoint3 c0[3] = { raw.fwd, raw.up, raw.right }, p0[3] = { g_prevReading.fwd, g_prevReading.up, g_prevReading.right };
				// D = sum_k c0[k] * p0[k]^T ; pred_axis = D * c0[axis] = sum_k c0[k] * (p0[k] . c0[axis])
				NiPoint3 out[3];
				for (int a = 0; a < 3; ++a) {
					NiPoint3 v{ 0, 0, 0 };
					for (int k = 0; k < 3; ++k) v = v + c0[k] * Dot(p0[k], c0[a]);
					out[a] = v;
				}
				pred.fwd = Norm(out[0]);
				pred.up = Norm(out[1] - pred.fwd * Dot(out[1], pred.fwd));
				pred.right = Cross(pred.fwd, pred.up);
			}
		}
		g_prevReading = raw;
		g_havePrev = true;
		g_usedRaw = raw;
		g_usedPred = pred;
		g_usedThisFrame = false;
		cam.pos = pred.pos; cam.fwd = pred.fwd; cam.up = pred.up; cam.right = pred.right;
		bool passRan = false;
		{
			// both views will run in this frame: the VL of view 0 is deferred behind view 1's pass
			float l0, r0, b0, t0, l1, r1, b1, t1;
			g_vlDefer.allowed = PortalRect(pb, cam, l0, r0, b0, t0) && PortalRect(po, cam, l1, r1, b1, t1);
		}
		// see yourself through a portal (09:30Z): in first person the engine draws only the arms; for the portal pictures the third-person body is shown and the
		// first-person arms hidden, both put back right after the passes (the real view draws later in the frame)
		struct SelfView
		{
			RE::NiAVObject* body{ nullptr };
			RE::NiAVObject* arms{ nullptr };
			bool            bodyHidden{ false }, armsHidden{ false };
			bool            on{ false };
		} self;
		if (g_params.selfView) {
			auto* pl = RE::PlayerCharacter::GetSingleton();
			auto* pcam = RE::PlayerCamera::GetSingleton();
			if (pl && pcam && pcam->IsInFirstPerson()) {
				self.body = pl->Get3D(false);
				self.arms = pl->Get3D(true);
				if (self.body && self.arms && self.body != self.arms) {
					self.bodyHidden = self.body->flags.any(RE::NiAVObject::Flag::kHidden);
					self.armsHidden = self.arms->flags.any(RE::NiAVObject::Flag::kHidden);
					static int logged = 0;
					if (logged++ < 3) {
						logger::info("view: self view: third-person body flags {:#x} (hidden {}), first-person arms flags {:#x}", static_cast<std::uint32_t>(self.body->flags.get()), self.bodyHidden,
							static_cast<std::uint32_t>(self.arms->flags.get()));
					}
					self.body->flags.reset(RE::NiAVObject::Flag::kHidden);
					self.arms->flags.set(RE::NiAVObject::Flag::kHidden);
					self.on = true;
				}
			}
		}
		for (int i = 0; i < 2; ++i) {
			const portals::Portal& A = i == 0 ? pb : po;
			const portals::Portal& B = i == 0 ? po : pb;
			float                  l, r, b, t;
			if (!PortalRect(A, cam, l, r, b, t)) {
				continue;
			}
			SafeArgs sa{ i, &A, &B, &cam, l, r, b, t, false, {} };
			passRan = true;
			if (!RunPassGuarded(&sa)) {
				++g_exceptions;
				g_disabled = g_exceptions >= 2;
				logger::error("view: pass raised an exception ({} so far){}", g_exceptions, g_disabled ? "; views disabled for this session" : "");
				return;
			}
			if (!sa.ok) {
				static int warned = 0;
				if (warned++ < 5) {
					logger::warn("view: pass for portal {} not run: {}", i, sa.why);
				}
				continue;
			}
			g_valid[i] = true;
			g_usedThisFrame = true;
		}
		if (self.on) {
			if (self.bodyHidden) self.body->flags.set(RE::NiAVObject::Flag::kHidden);
			if (!self.armsHidden) self.arms->flags.reset(RE::NiAVObject::Flag::kHidden);
		}
		RunDeferredVl();
		g_depthPending = false;
		if (g_params.fpSkin && (g_valid[0] || g_valid[1] || passRan)) {
			const int lg = g_params.fpSkinLog;
			if (lg > 0) {
				g_params.fpSkinLog = lg - 1;
			}
			travel::ResetFirstPersonSkins(lg);  // the real first-person draw later in this frame must build its own bone matrices
		}
		if (g_params.skinFix && (g_valid[0] || g_valid[1])) {
			travel::ResetSkinCaches();  // the main view must rebuild the bone matrices for the real camera
		}
		if ((g_valid[0] || g_valid[1]) && g_params.clearAfter) {
			// the main view expects cleared colour and depth: our passes left their picture in the same targets. Cleared through the engine's own
			// target table (the targets bound after SubmitAccumulator are not necessarily the ones we asked for).
			ComPtr<ID3D11DeviceContext> d3d;
			Device()->GetImmediateContext(&d3d);
			auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
			auto* ds = &RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData().depthStencils[g_params.ds];
			auto* rtvT = rd ? reinterpret_cast<ID3D11RenderTargetView*>(rd->renderTargets[g_params.rt].RTV) : nullptr;
			auto* dsvT = ds ? reinterpret_cast<ID3D11DepthStencilView*>(ds->views[0]) : nullptr;
			static int logged = 0;
			if (logged < 2) {
				++logged;
				ComPtr<ID3D11RenderTargetView> rtv;
				ComPtr<ID3D11DepthStencilView> dsv;
				d3d->OMGetRenderTargets(1, &rtv, &dsv);
				logger::info("view: clear-after: bound RTV {:p} (table {:p}) bound DSV {:p} (table {:p})", static_cast<void*>(rtv.Get()), static_cast<void*>(rtvT), static_cast<void*>(dsv.Get()), static_cast<void*>(dsvT));
			}
			const FLOAT black[4] = { 0, 0, 0, 0 };
			if (rtvT) d3d->ClearRenderTargetView(rtvT, black);
			if (dsvT) d3d->ClearDepthStencilView(dsvT, D3D11_CLEAR_DEPTH | D3D11_CLEAR_STENCIL, 1.0f, 0);
		}
	}

	void ReportActual()
	{
		if (!g_usedThisFrame) {
			return;
		}
		g_usedThisFrame = false;
		world::CamInfo a;
		if (!world::CameraFull(a)) {
			return;
		}
		auto angDeg = [](const NiPoint3& x, const NiPoint3& y) { return std::acos(std::clamp(Dot(x, y), -1.0f, 1.0f)) * 57.29578f; };
		auto dist = [](const NiPoint3& x, const NiPoint3& y) { const NiPoint3 d = x - y; return std::sqrt(Dot(d, d)); };
		const double ra = angDeg(a.fwd, g_usedRaw.fwd), pa = angDeg(a.fwd, g_usedPred.fwd);
		const double rp = dist(a.pos, g_usedRaw.pos), pp = dist(a.pos, g_usedPred.pos);
		auto& s = g_stats;
		++s.n;
		s.rawAng += ra; s.predAng += pa; s.rawPos += rp; s.predPos += pp;
		s.rawAngMax = (std::max)(s.rawAngMax, ra); s.predAngMax = (std::max)(s.predAngMax, pa);
		s.rawPosMax = (std::max)(s.rawPosMax, rp); s.predPosMax = (std::max)(s.predPosMax, pp);
		if (s.n >= 240) {
			LogStats();
		}
	}

	void LogStats()
	{
		auto& s = g_stats;
		if (s.n > 0) {
			logger::info("view: camera error over {} frames (predict={}): stale camera mean {:.3f} deg max {:.3f} deg, pos mean {:.2f} max {:.2f} | predicted mean {:.3f} deg max {:.3f} deg, pos mean {:.2f} max {:.2f}",
				s.n, g_params.predict, s.rawAng / s.n, s.rawAngMax, s.rawPos / s.n, s.rawPosMax, s.predAng / s.n, s.predAngMax, s.predPos / s.n, s.predPosMax);
		}
		s = Stats{};
	}

	void RefreshCamera(void* a_niCamera)
	{
		// the engine's own update, not the hooked one (the hook would roll the camera a second time)
		auto* orig = crossing::CamUpdateOriginal();
		reinterpret_cast<CamUpdate_t>(orig ? orig : reinterpret_cast<void*>(REL::ID(70641).address()))(a_niCamera);
	}

	void RequestDepthDump(const std::string& a_name)
	{
		g_depthName = a_name;
		g_depthPending = true;
	}

	void* VlPreSRV(int a_portal)
	{
		return (a_portal >= 0 && a_portal < 2 && g_valid[a_portal] && g_vlAddValid[a_portal]) ? static_cast<void*>(g_vlPreSrv[a_portal].Get()) : nullptr;
	}

	void* VlPostSRV(int a_portal)
	{
		return (a_portal >= 0 && a_portal < 2 && g_valid[a_portal] && g_vlAddValid[a_portal]) ? static_cast<void*>(g_vlPostSrv[a_portal].Get()) : nullptr;
	}

	bool VlAddValid(int a_portal)
	{
		return a_portal >= 0 && a_portal < 2 && g_valid[a_portal] && g_vlAddValid[a_portal];
	}

	void* ViewSRV(int a_portal)
	{
		return (a_portal >= 0 && a_portal < 2 && g_valid[a_portal]) ? static_cast<void*>(g_srv[a_portal].Get()) : nullptr;
	}

	bool ExitCopy(int a_portal, int a_kind, void** a_colour, void** a_depth)
	{
		const int idx = a_portal + 2 * a_kind;
		if (a_portal < 0 || a_portal > 1 || a_kind < 0 || a_kind > 1 || !g_p2valid[idx]) {
			return false;
		}
		*a_colour = static_cast<void*>(g_p2cSrv[idx].Get());
		*a_depth = static_cast<void*>(g_p2dSrv[idx].Get());
		return *a_colour && *a_depth;
	}

	void* ViewTexture(int a_portal)
	{
		return (a_portal >= 0 && a_portal < 2) ? static_cast<void*>(g_tex[a_portal].Get()) : nullptr;
	}

	bool GpuFence(const char* a_label)
	{
		return VsFence(a_label);
	}

	void Configure(const std::string& a_args)
	{
		std::istringstream in(a_args);
		for (std::string tok; in >> tok;) {
			const auto eq = tok.find('=');
			if (eq == std::string::npos) {
				if (tok == "on") g_params.enabled = true;
				else if (tok == "off") g_params.enabled = false;
				continue;
			}
			const auto k = tok.substr(0, eq);
			const auto v = tok.substr(eq + 1);
			try {
				if (k == "cone") g_params.cone = std::stoi(v) != 0;
				else if (k == "exitplane") g_params.exitPlane = std::stoi(v) != 0;
				else if (k == "bias") g_params.planeBias = std::stof(v);
				else if (k == "oblique") g_params.oblique = std::stoi(v) != 0;
				else if (k == "skinfix") g_params.skinFix = std::stoi(v) != 0;
				else if (k == "fpskin") g_params.fpSkin = std::stoi(v) != 0;
				else if (k == "selfview") g_params.selfView = std::stoi(v) != 0;
				else if (k == "fpskinlog") g_params.fpSkinLog = std::stoi(v);
				else if (k == "ignorepre") g_params.ignorePre = std::stoi(v) != 0;
				else if (k == "passstats") g_statLeft = std::stoi(v);
				else if (k == "npcbounds") g_npcBoundsLeft = std::stoi(v);
				else if (k == "clearafter") g_params.clearAfter = std::stoi(v) != 0;
				else if (k == "when") g_params.when = std::stoi(v);
				else if (k == "eqz") g_params.eqz = std::stoi(v);
				else if (k == "trace") { EnsureTraceHooks(); g_traceFrames = std::stoi(v); }
				else if (k == "alphafix") { EnsureTraceHooks(); g_params.alphaFix = std::stoi(v) != 0; }
				else if (k == "prepass") { g_params.prepass = std::stoi(v) != 0; g_coverLogged = 0; }
				else if (k == "preflags") { g_params.preFlags = static_cast<std::uint32_t>(std::stoul(v, nullptr, 0)); g_coverLogged = 0; }
				else if (k == "premode") { g_params.preMode = static_cast<std::uint32_t>(std::stoul(v, nullptr, 0)); g_coverLogged = 0; }
				else if (k == "preunbind") { g_params.preUnbind = std::stoi(v) != 0; g_coverLogged = 0; }
				else if (k == "prerestore") g_params.preRestore = std::stoi(v) != 0;
				else if (k == "lightmask") { EnsureTraceHooks(); g_params.lightMask = std::stoi(v); }
				else if (k == "ssao") g_params.ssao = std::stoi(v) != 0;
				else if (k == "rtdump") g_rtDumpPending = std::stoi(v) != 0;
				else if (k == "rtlist") {
					g_rtListOverride.clear();
					std::istringstream ls(v);
					for (std::string t; std::getline(ls, t, ',');) g_rtListOverride.push_back(std::stoi(t));
				}
				else if (k == "rtdepth") g_rtDepthDump = std::stoi(v) != 0;
				else if (k == "engtag") g_engTagPrefix = v;
				else if (k == "rttag") g_rtDumpTag = v;
				else if (k == "normclear") g_normClear = std::stoi(v) != 0;
				else if (k == "mrt") g_mrt = std::stoi(v) != 0;
				else if (k == "maskclear") g_maskClear = std::stof(v);
				else if (k == "maskstat2") g_maskStat2 = std::stoi(v);
				else if (k == "skipbits") { EnsureTraceHooks(); g_skipBits = std::stoi(v); }
				else if (k == "cullflags") g_cullFlags = std::stoi(v, nullptr, 0);
				else if (k == "depthhack") { EnsureTraceHooks(); g_depthHack = std::stoi(v); }
				else if (k == "logjitter") { g_logJitter = std::stoi(v) != 0; g_jitLogged = 0; }
				else if (k == "t14over") { EnsureTraceHooks(); g_t14Over = std::stoi(v); g_t14Hits = 0; }
				else if (k == "probeany") g_probeAny = std::stoi(v) != 0;
				else if (k == "probegeom") { EnsureTraceHooks(); g_probeName = v; g_probeLeftOurs = 6; g_probeLeftEng = 6; }
				else if (k == "capmask") { EnsureTraceHooks(); g_capMaskOn = std::stoi(v) != 0; g_capMaskOursValid = false; g_capMaskEngDone = false; g_maskCallsOurs = g_maskCallsEng = 0; }
				else if (k == "t4const") { EnsureTraceHooks(); g_t4Const = std::stoi(v); }
				else if (k == "maskreplay") { EnsureTraceHooks(); g_maskReplay = std::stoi(v); }
				else if (k == "t4shadow") { EnsureTraceHooks(); g_t4Shadow = std::stoi(v); g_maskSrvLogged = 0; }
				else if (k == "occl") { EnsureTraceHooks(); g_occlOn = std::stoi(v) != 0; }
				else if (k == "seqrange") { EnsureTraceHooks(); const auto c = v.find(':'); g_seqLo = std::stoi(v.substr(0, c)); g_seqHi = std::stoi(v.substr(c + 1)); }
				else if (k == "realskipbits") { EnsureTraceHooks(); g_skipBitsReal = std::stoi(v); }
				else if (k == "realmaskclear") { EnsureTraceHooks(); g_realMaskClear = std::stof(v); }
				else if (k == "capture") { EnsureTraceHooks(); g_capOn = std::stoi(v) != 0; }
				else if (k == "capshader") g_capShaderRva = std::stoull(v, nullptr, 16);
				else if (k == "ssaodiff") g_ssaoDiff = std::stoi(v) != 0;
				else if (k == "ssaorestore") g_ssaoRestore = std::stoi(v) != 0;
				else if (k == "engdump") { EnsureTraceHooks(); g_engDump = std::stoi(v); }
				else if (k == "ssaohist") g_params.ssaoHist = std::stoi(v) != 0;
				else if (k == "ssaoclear") g_params.ssaoClear = std::stoi(v) != 0;
				else if (k == "maskpass") g_params.maskPass = std::stoi(v) != 0;
				else if (k == "maskstat") g_params.maskStat = std::stoi(v);
				else if (k == "prerebind") g_params.preRebind = std::stoi(v) != 0;
				else if (k == "accpoke") {
					g_accPokes.clear();
					if (v != "clear") {
						std::istringstream ps(v);
						for (std::string item; std::getline(ps, item, ',');) {
							std::istringstream is(item);
							std::string a, b, c;
							std::getline(is, a, ':');
							std::getline(is, b, ':');
							std::getline(is, c, ':');
							g_accPokes.push_back({ static_cast<std::size_t>(std::stoul(a, nullptr, 0)), std::stoull(b, nullptr, 0), c.empty() ? 1 : std::stoi(c) });
						}
					}
					logger::info("view: accumulator pokes: {} entries", g_accPokes.size());
				}
				else if (k == "prestart") { g_params.preStart = static_cast<std::uint32_t>(std::stoul(v, nullptr, 0)); g_coverLogged = 0; }
				else if (k == "lightflag") g_params.lightFlag = std::stoi(v) != 0;
				else if (k == "identity") g_params.identity = std::stoi(v) != 0;
				else if (k == "forcerect") g_params.forceRect = std::stoi(v) != 0;
				else if (k == "oblforce") g_params.oblForce = std::stof(v);
				else if (k == "offyaw") g_params.offYaw = std::stof(v);
				else if (k == "offx") g_params.offX = std::stof(v);
				else if (k == "offy") g_params.offY = std::stof(v);
				else if (k == "offz") g_params.offZ = std::stof(v);
				else if (k == "sky") g_params.skyMove = std::stoi(v) != 0;
				else if (k == "skybig") g_skyBig = std::stoi(v);
				else if (k == "animfix") g_params.animFix = std::stoi(v);
				else if (k == "ssaoown") g_ssaoOwn = std::stoi(v);
				else if (k == "blendstd") g_blendStd = std::stoi(v);
				else if (k == "cullog") CullLogEnable(std::stoi(v));
				else if (k == "vloff") { VlHookEnable(); g_vlOff = std::stoi(v); }
				else if (k == "vl") { g_vlRun = std::stoi(v); g_vlLogLeft = 8; }
				else if (k == "vlpass") g_vlStages = std::stoi(v);
				else if (k == "vlsnap") g_vlSnap = std::stoi(v);
				else if (k == "vlviews") g_vlViews = std::stoi(v);
				else if (k == "vlrestore") g_vlRestore = std::stoi(v);
				else if (k == "vldefer") g_vlDeferMode = std::stoi(v);
				else if (k == "vladd") g_vlAdd = std::stoi(v);
				else if (k == "vlrefresh") g_vlRefresh = std::stoi(v);
				else if (k == "vlsub") overlay::SetVlSub(std::stoi(v));
				else if (k == "vllog") { VlHookEnable(); g_vlLog = std::stoi(v); g_vlLogLeft2 = std::stoi(v); }
				else if (k == "postlog") { InstallDSHook(); g_postLogFrames = std::stoi(v); }
				else if (k == "shlog") { g_sunLogLeft = std::stoi(v); frame::ShadowTrace(0); }
				else if (k == "ssaotag") g_ssaoCapTag = v + "_";
				else if (k == "ssaodump") g_ssaoCapDump = std::stoi(v) != 0;
				else if (k == "ssaocap") SsaoCapEnable(std::stoi(v));
				else if (k == "skipsky") g_skipSky = std::stoi(v);
				else if (k == "clearcol") {
					std::istringstream cs(v);
					std::string        t;
					for (int i = 0; i < 3 && std::getline(cs, t, ','); ++i) g_clearCol[i] = std::stof(t);
				}
				else if (k == "vshadow") g_params.vShadow = std::stoi(v);
				else if (k == "vstrace") g_vsTrace = std::stoi(v) != 0;
				else if (k == "vsfence") g_vsFence = std::stoi(v) != 0;
				else if (k == "vsmode") g_vsMode = std::stoi(v);
				else if (k == "portalshadows") { g_params.vShadow = std::stoi(v) ? 3 : 0; g_vsMode = 16 | 64 | 128 | 4 | 256; }
				else if (k == "shfence") frame::ShadowFence(std::stoi(v) != 0);
				else if (k == "watchdog") frame::SetWatchdog(std::stod(v));
				else if (k == "maskcover") g_maskCover = std::stoi(v) != 0;
				else if (k == "shtrace") frame::ShadowTrace(std::stoi(v));
				else if (k == "deobl") g_deObl = std::stoi(v);
				else if (k == "vpafter") g_vpAfter = std::stoi(v);
				else if (k == "shadowmask") g_params.shadowMask = std::stoi(v);
				else if (k == "diag") { g_params.diag = std::stoi(v) != 0; g_diagN = 0; }
				else if (k == "subflags") g_params.submitFlags = static_cast<std::uint32_t>(std::stoul(v, nullptr, 0));
				else if (k == "predict") g_params.predict = std::stoi(v) != 0;
				else if (k == "rt") g_params.rt = std::stoi(v);
				else if (k == "ds") g_params.ds = std::stoi(v);
				else if (k == "mode") g_params.mode = static_cast<std::uint32_t>(std::stoi(v));
				else logger::warn("view: unknown parameter '{}'", k);
			} catch (...) {
				logger::warn("view: bad value '{}' for '{}'", v, k);
			}
		}
		logger::info("view: enabled={} cone={} exitplane={} bias={} oblique={} clearafter={} when={} rt={} ds={} mode={} subflags={:#x} sky={} shadowmask={}", g_params.enabled, g_params.cone, g_params.exitPlane, g_params.planeBias, g_params.oblique, g_params.clearAfter, g_params.when, g_params.rt, g_params.ds, g_params.mode, g_params.submitFlags, g_params.skyMove, g_params.shadowMask);
	}
}
