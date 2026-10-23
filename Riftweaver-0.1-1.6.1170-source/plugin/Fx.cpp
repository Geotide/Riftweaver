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
#include <wrl/client.h>

// Portal particles, drawn by the plugin itself (no mesh, no texture file, no engine particle system): slow miasma that curls off the rim of each oval, a burst when a portal opens, and a small
// wisp at the casting hand while one of the portal spells charges. Called from overlay::Draw (Overlay.cpp) after the ovals, with the overlay's render state bound (main HDR colour + depth, the
// viewport, a rasterizer without culling, triangle strip, no input layout); the overlay restores everything it saved, so only what is outside its save list is restored here.
//   Simulation: a plain CPU particle list (at most kMax), stepped with the frame time.
//   Look: camera-facing soft sprites from one procedural R8 atlas (three wisp shapes + a mote), alpha blended, depth tested against the engine's depth (no depth write), sorted far to near.
//   Colour: teal (#3FD9C4 family) and violet (#9A6BFF family), each with a deep shade for the miasma and a pale one for the motes.
// Everything is guarded: a shader compile failure only disables the particles (the ovals are unaffected), an exception disables them for the session (logged once).
namespace fx
{
	using Microsoft::WRL::ComPtr;

	namespace
	{
		constexpr int kMax = 520;       // live particles
		constexpr int kBatch = 256;     // instances per draw (the constant buffer holds this many)
		// A portal's particles never reach farther than this many units from the oval's rim: a sprite is shrunk so that its far edge stays inside the band, and is gone at the band's edge
		// (the overlay's halo is zero out there too), so the wall keeps its own colour and no straight edge of a sprite or of the geometry in front of it can show as a tinted patch.
		constexpr float kReach = 30.0f;

		std::atomic<bool> g_on{ true };
		bool              g_ready{ false };
		bool              g_failed{ false };
		std::atomic<int>  g_count{ 0 };
		std::atomic<float> g_reach{ kReach };   // `fx reach <units>` (tests): a large value = no limit

		const char* kHlsl = R"HLSL(
cbuffer CB : register(b0)
{
	float4 camPos;
	float4 camFwd;
	float4 camUp;
	float4 camRight;
	float4 frustum;   // tanLeft, tanRight, tanBottom, tanTop
	float4 depthP;    // near, far, mode (0 std, 1 reversed, 2 none), unused
	float4 P0[256];   // xyz centre, half size in units
	float4 P1[256];   // rgb colour (HDR), alpha
	float4 P2[256];   // rotation, atlas tile, unused, unused
};
Texture2D<float> spriteTex : register(t2);

struct VSOut
{
	float4 pos : SV_Position;
	float2 q   : TEXCOORD0;
	float4 col : TEXCOORD1;
	nointerpolation float tile : TEXCOORD2;
};

VSOut VSMain(uint id : SV_VertexID, uint inst : SV_InstanceID)
{
	float2 q = float2((id & 1) ? 1.0 : -1.0, (id & 2) ? 1.0 : -1.0);
	float4 a = P0[inst];
	float4 c = P1[inst];
	float4 r = P2[inst];
	float cs = cos(r.x), sn = sin(r.x);
	float2 qr = float2(q.x * cs - q.y * sn, q.x * sn + q.y * cs);
	float3 wp = a.xyz + (camRight.xyz * qr.x + camUp.xyz * qr.y) * a.w;
	float3 d = wp - camPos.xyz;
	float f = dot(d, camFwd.xyz);
	float u = dot(d, camUp.xyz);
	float rr = dot(d, camRight.xyz);
	float fs = max(f, 0.01);
	float nx = 2.0 * (rr / fs - frustum.x) / (frustum.y - frustum.x) - 1.0;
	float ny = 2.0 * (u / fs - frustum.z) / (frustum.w - frustum.z) - 1.0;
	float n = depthP.x, fa = depthP.y;
	float zq = (depthP.z < 0.5) ? fa * (fs - n) / (fs * (fa - n)) : n * (fa - fs) / (fs * (fa - n));
	VSOut o;
	o.pos = (f > 0.5) ? float4(nx * f, ny * f, saturate(zq) * f, f) : float4(0, 0, -1, 1);
	o.q = q;
	o.col = float4(c.rgb, c.a * smoothstep(20.0, 120.0, f));   // particles fade out as they get close to the camera (they must not wash out the screen at close range)
	o.tile = r.y;
	return o;
}

float4 PSMain(VSOut i) : SV_Target
{
	float2 uv = saturate(i.q * 0.5 + 0.5) * 126.0;
	int2 i0 = (int2)floor(uv);
	float2 f = frac(uv);
	int t = (int)(i.tile + 0.5);
	int2 base = int2((t & 1) * 128, (t >> 1) * 128) + i0;
	float s00 = spriteTex.Load(int3(base, 0));
	float s10 = spriteTex.Load(int3(base + int2(1, 0), 0));
	float s01 = spriteTex.Load(int3(base + int2(0, 1), 0));
	float s11 = spriteTex.Load(int3(base + int2(1, 1), 0));
	float s = lerp(lerp(s00, s10, f.x), lerp(s01, s11, f.x), f.y);
	float a = s * i.col.a;
	if (a < 0.002) discard;
	return float4(i.col.rgb, a);
}
)HLSL";

		struct CB
		{
			float camPos[4], camFwd[4], camUp[4], camRight[4], frustum[4], depthP[4];
			float p0[kBatch][4], p1[kBatch][4], p2[kBatch][4];
		};

		ComPtr<ID3D11VertexShader>      g_vs;
		ComPtr<ID3D11PixelShader>       g_ps;
		ComPtr<ID3D11Buffer>            g_cb;
		ComPtr<ID3D11BlendState>        g_blend;
		ComPtr<ID3D11Texture2D>         g_tex;
		ComPtr<ID3D11ShaderResourceView> g_srv;

		// ---- the procedural sprite atlas: 2 x 2 tiles of 128 px, R8: three wisp shapes and a soft mote ----
		float Hash(int x, int y, int s)
		{
			std::uint32_t h = static_cast<std::uint32_t>(x) * 374761393u + static_cast<std::uint32_t>(y) * 668265263u + static_cast<std::uint32_t>(s) * 2246822519u;
			h = (h ^ (h >> 13)) * 1274126177u;
			h ^= h >> 16;
			return static_cast<float>(h & 0xFFFFFF) / 16777215.0f;
		}

		float VNoise(float x, float y, int s)
		{
			const int   ix = static_cast<int>(std::floor(x)), iy = static_cast<int>(std::floor(y));
			const float fx0 = x - static_cast<float>(ix), fy0 = y - static_cast<float>(iy);
			const float ux = fx0 * fx0 * (3.0f - 2.0f * fx0), uy = fy0 * fy0 * (3.0f - 2.0f * fy0);
			const float a = Hash(ix, iy, s), b = Hash(ix + 1, iy, s), c = Hash(ix, iy + 1, s), d = Hash(ix + 1, iy + 1, s);
			return (a + (b - a) * ux) + ((c + (d - c) * ux) - (a + (b - a) * ux)) * uy;
		}

		float Fbm(float x, float y, int s)
		{
			float v = 0.0f, amp = 0.5f;
			for (int k = 0; k < 4; ++k) {
				v += amp * VNoise(x, y, s + k);
				x = x * 2.03f + 17.1f;
				y = y * 2.03f - 9.3f;
				amp *= 0.5f;
			}
			return v;
		}

		std::vector<std::uint8_t> MakeAtlas()
		{
			std::vector<std::uint8_t> px(256 * 256, 0);
			for (int tile = 0; tile < 4; ++tile) {
				const int ox = (tile & 1) * 128, oy = (tile >> 1) * 128;
				for (int y = 0; y < 128; ++y) {
					for (int x = 0; x < 128; ++x) {
						const float nx = (static_cast<float>(x) + 0.5f) / 64.0f - 1.0f, ny = (static_cast<float>(y) + 0.5f) / 64.0f - 1.0f;
						const float r2 = nx * nx + ny * ny;
						float       v = 0.0f;
						if (r2 < 1.0f) {
							if (tile < 3) {
								const float fall = std::pow(1.0f - r2, 1.6f);
								const float nz = Fbm(nx * 1.9f + static_cast<float>(tile) * 7.3f, ny * 1.9f - static_cast<float>(tile) * 3.9f, tile * 11);
								const float w = std::clamp((nz - 0.28f) * 2.4f, 0.0f, 1.0f);
								v = fall * (0.18f + 0.82f * w);
							} else {
								v = std::exp(-r2 * 5.5f) * (1.0f - std::clamp((std::sqrt(r2) - 0.7f) / 0.3f, 0.0f, 1.0f));
							}
						}
						px[static_cast<std::size_t>(oy + y) * 256 + static_cast<std::size_t>(ox + x)] = static_cast<std::uint8_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f);
					}
				}
			}
			return px;
		}

		bool Init(ID3D11Device* a_dev)
		{
			ComPtr<ID3DBlob> vsb, psb, err;
			auto             compile = [&](const char* a_entry, const char* a_target, ComPtr<ID3DBlob>& a_out) {
                err.Reset();
                const HRESULT hr = D3DCompile(kHlsl, std::strlen(kHlsl), "portal_fx", nullptr, nullptr, a_entry, a_target, D3DCOMPILE_OPTIMIZATION_LEVEL2, 0, &a_out, &err);
                if (FAILED(hr)) {
                    logger::error("fx: shader compile {} failed: {}", a_entry, err ? static_cast<const char*>(err->GetBufferPointer()) : "?");
                    return false;
                }
                return true;
			};
			if (!compile("VSMain", "vs_5_0", vsb) || !compile("PSMain", "ps_5_0", psb)) {
				return false;
			}
			if (FAILED(a_dev->CreateVertexShader(vsb->GetBufferPointer(), vsb->GetBufferSize(), nullptr, &g_vs)) || FAILED(a_dev->CreatePixelShader(psb->GetBufferPointer(), psb->GetBufferSize(), nullptr, &g_ps))) {
				logger::error("fx: CreateShader failed");
				return false;
			}
			D3D11_BUFFER_DESC bd{};
			bd.ByteWidth = sizeof(CB);
			bd.Usage = D3D11_USAGE_DYNAMIC;
			bd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
			bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
			if (FAILED(a_dev->CreateBuffer(&bd, nullptr, &g_cb))) {
				logger::error("fx: CreateBuffer failed");
				return false;
			}
			D3D11_BLEND_DESC bl{};
			bl.RenderTarget[0].BlendEnable = TRUE;
			bl.RenderTarget[0].SrcBlend = D3D11_BLEND_SRC_ALPHA;
			bl.RenderTarget[0].DestBlend = D3D11_BLEND_INV_SRC_ALPHA;
			bl.RenderTarget[0].BlendOp = D3D11_BLEND_OP_ADD;
			bl.RenderTarget[0].SrcBlendAlpha = D3D11_BLEND_ZERO;
			bl.RenderTarget[0].DestBlendAlpha = D3D11_BLEND_ONE;
			bl.RenderTarget[0].BlendOpAlpha = D3D11_BLEND_OP_ADD;
			bl.RenderTarget[0].RenderTargetWriteMask = D3D11_COLOR_WRITE_ENABLE_RED | D3D11_COLOR_WRITE_ENABLE_GREEN | D3D11_COLOR_WRITE_ENABLE_BLUE;
			if (FAILED(a_dev->CreateBlendState(&bl, &g_blend))) {
				logger::error("fx: CreateBlendState failed");
				return false;
			}
			const auto           atlas = MakeAtlas();
			D3D11_TEXTURE2D_DESC td{};
			td.Width = td.Height = 256;
			td.MipLevels = td.ArraySize = 1;
			td.Format = DXGI_FORMAT_R8_UNORM;
			td.SampleDesc.Count = 1;
			td.Usage = D3D11_USAGE_IMMUTABLE;
			td.BindFlags = D3D11_BIND_SHADER_RESOURCE;
			D3D11_SUBRESOURCE_DATA sd{};
			sd.pSysMem = atlas.data();
			sd.SysMemPitch = 256;
			if (FAILED(a_dev->CreateTexture2D(&td, &sd, &g_tex)) || FAILED(a_dev->CreateShaderResourceView(g_tex.Get(), nullptr, &g_srv))) {
				logger::error("fx: sprite atlas creation failed");
				return false;
			}
			logger::info("fx: particle shaders, blend state and the procedural sprite atlas (256x256 R8) created");
			return true;
		}

		// ---- simulation ----
		struct Particle
		{
			float pos[3]{}, vel[3]{};
			float age{ 0 }, life{ 1 };
			float size0{ 10 }, size1{ 30 };
			float rot{ 0 }, rotVel{ 0 };
			float alpha{ 0.2f };
			float drag{ 0.3f };
			float swirl{ 1.0f };
			float rgb[3]{ 1, 1, 1 };
			int   tile{ 0 };
			float flicker{ 0.0f };   // Embers: how deeply the spark's brightness flickers
			int   portal{ -1 };   // 0 teal, 1 violet: the particle is kept within kReach of this portal's rim; -1 (the casting hand) is not
			int   hand{ -1 };     // 0 left, 1 right: the particle moves with that hand (wisps and release sparks); -1 world space
			bool  local{ false }; // pos/vel are an offset from the hand (drawn at the hand of the frame)
			bool  camAxes{ false };  // a local offset in the camera's axes (x right, y up, z forward: first person) instead of world axes
		};
		std::vector<Particle> g_parts;
		std::uint32_t         g_rng{ 0x9E3779B9u };
		double                g_lastNow{ 0 };
		double                g_seenPlaced[2]{ 0, 0 };
		float                 g_accWisp[2]{ 0, 0 }, g_accMote[2]{ 0, 0 }, g_accHand[2]{ 0, 0 }, g_accEmber[2]{ 0, 0 };
		double                g_seenGhost[2]{ 0, 0 };   // G310: the replaced portal whose closing sparks have started (its closedAtSec)
		bool                  g_popDone[2]{ false, false };
		bool                  g_burstWaiting[2]{ false, false };  // G310: the soft opening burst is held back until the tear has widened (a burst over a sliver is a white column)
		double                g_seenCross{ 0 };         // G310: the walk through a portal whose mist puff has been made
		RE::NiPoint3          g_handPrev[2]{};          // the hand positions of the last frame, for carrying the hand particles
		std::atomic<bool>     g_handCarry{ true };      // test switch (fx handcarry off = the old world-space hand particles, for before/after)
		int                   g_handLogTick{ 0 }, g_handLogged{ 0 };
		// try 3: the hand as the arms are drawn this frame (camera + camera-relative node); the axis convention of the camera-relative node is picked at runtime
		int                   g_anchorMode{ -1 };       // -1 = start-of-frame position rebuilt with this frame's camera; 0..2 = camera + node in convention 0..2
		double                g_modeErr[3]{ 0, 0, 0 };
		int                   g_modeN{ 0 };
		RE::NiPoint3          g_anchorRel[2]{};         // anchor - camera, for the overlay's ring and glow
		std::uint64_t         g_anchorFrame[2]{ 0, 0 };
		RE::NiPoint3          g_prevCamPos{}, g_prevCamFwd{};
		bool                  g_prevCamOk{ false };
		RE::NiPoint3          g_prevPlayer{};
		double                g_prevPlayerT{ 0 };
		std::atomic<bool>     g_handFxLog{ false };     // Riftweaver.ini HandFxLog=1
		std::atomic<bool>     g_handWisps{ false };     // Riftweaver.ini HandWisps (default 0): the smoke wisps and release sparks at the casting hand (0.1: off; they wander 2-4 units
		                                                // and the hand is 35-50 units from the eye, so they 'danced around the screen' while moving, 2026-10-22)
		int                   g_hfxTick{ 0 };
		float                 g_screenW{ 1920.0f };
		RE::NiPoint3          g_handDraw[2]{};          // the hand the local particles are drawn at this frame (kept when the hand goes down, so they fade where it was)
		float Len3(const RE::NiPoint3& a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }
		bool                  g_handPrevOk[2]{ false, false };

		float Rf()
		{
			g_rng ^= g_rng << 13;
			g_rng ^= g_rng >> 17;
			g_rng ^= g_rng << 5;
			return static_cast<float>(g_rng & 0xFFFFFF) / 16777215.0f;
		}
		float Rr(float a, float b) { return a + (b - a) * Rf(); }

		// palette: [colour][0] miasma body, [1] deep shade, [2] pale mote
		const float kPal[2][3][3] = {
			{ { 0.10f, 0.62f, 0.52f }, { 0.03f, 0.26f, 0.30f }, { 0.55f, 1.00f, 0.90f } },   // teal  (#3FD9C4 family)
			{ { 0.40f, 0.22f, 0.90f }, { 0.13f, 0.06f, 0.40f }, { 0.80f, 0.66f, 1.00f } },   // violet (#9A6BFF family)
		};

		void SetColour(Particle& p, int a_colour, bool a_mote)
		{
			const auto& pal = kPal[a_colour & 1];
			if (a_mote) {
				for (int k = 0; k < 3; ++k) p.rgb[k] = pal[2][k] * Rr(1.2f, 2.2f);
			} else {
				const float m = Rf();
				for (int k = 0; k < 3; ++k) p.rgb[k] = (pal[0][k] * m + pal[1][k] * (1.0f - m)) * Rr(1.0f, 1.5f);
			}
		}

		void Add(const Particle& p)
		{
			if (static_cast<int>(g_parts.size()) < kMax) {
				g_parts.push_back(p);
			}
		}

		// one miasma wisp on the rim of portal a_p
		void EmitRim(const portals::Portal& a_p, int a_index, bool a_mote)
		{
			const float ang = Rr(0.0f, 6.2831853f);
			const float ca = std::cos(ang), sa = std::sin(ang);
			Particle    p;
			const RE::NiPoint3 inPlane = a_p.r * ca + a_p.u * sa;  // outward in the portal's plane
			const float        off = Rr(-4.0f, 10.0f);              // along the normal: a little in front of the surface
			const RE::NiPoint3 c = a_p.c + a_p.r * (ca * a_p.hw) + a_p.u * (sa * a_p.hh) + a_p.n * (off + 8.0f);
			p.pos[0] = c.x; p.pos[1] = c.y; p.pos[2] = c.z;
			const float out = a_mote ? Rr(3.0f, 16.0f) : Rr(4.0f, 13.0f);
			const float tanv = Rr(-7.0f, 7.0f);
			const RE::NiPoint3 tang = a_p.r * (-sa) + a_p.u * ca;
			const float        nrm = a_mote ? Rr(2.0f, 12.0f) : Rr(1.0f, 7.0f);
			const float        rise = a_mote ? Rr(6.0f, 22.0f) : Rr(4.0f, 11.0f);
			p.vel[0] = inPlane.x * out + tang.x * tanv + a_p.n.x * nrm;
			p.vel[1] = inPlane.y * out + tang.y * tanv + a_p.n.y * nrm;
			p.vel[2] = inPlane.z * out + tang.z * tanv + a_p.n.z * nrm + rise;
			p.life = a_mote ? Rr(1.6f, 3.4f) : Rr(3.2f, 6.0f);
			p.size0 = a_mote ? Rr(0.9f, 1.8f) : Rr(14.0f, 24.0f);
			p.size1 = a_mote ? Rr(0.6f, 1.2f) : Rr(40.0f, 64.0f);
			p.alpha = a_mote ? Rr(0.55f, 0.95f) : Rr(0.09f, 0.18f);
			p.drag = a_mote ? 0.15f : 0.35f;
			p.rot = Rr(0.0f, 6.28f);
			p.rotVel = Rr(-0.25f, 0.25f);
			p.swirl = Rr(0.6f, 1.4f);
			p.tile = a_mote ? 3 : static_cast<int>(Rf() * 2.999f);
			p.portal = a_index;
			SetColour(p, a_p.colour, a_mote);
			Add(p);
		}

		// a spark that rises off the rim of portal a_p (Embers)
		void EmitEmber(const portals::Portal& a_p, int a_index)
		{
			const float ang = Rr(0.0f, 6.2831853f);
			const float ca = std::cos(ang), sa = std::sin(ang);
			Particle    p;
			const RE::NiPoint3 inPlane = a_p.r * ca + a_p.u * sa;
			const RE::NiPoint3 c = a_p.c + a_p.r * (ca * a_p.hw) + a_p.u * (sa * a_p.hh) + a_p.n * Rr(2.0f, 8.0f);
			p.pos[0] = c.x; p.pos[1] = c.y; p.pos[2] = c.z;
			const float out = Rr(1.0f, 7.0f), nrm = Rr(1.0f, 6.0f), rise = Rr(12.0f, 30.0f);
			p.vel[0] = inPlane.x * out + a_p.n.x * nrm;
			p.vel[1] = inPlane.y * out + a_p.n.y * nrm;
			p.vel[2] = inPlane.z * out + a_p.n.z * nrm + rise;
			p.life = Rr(1.2f, 2.6f);
			p.size0 = Rr(1.4f, 2.6f);
			p.size1 = Rr(0.9f, 1.6f);
			p.alpha = Rr(0.75f, 1.0f);
			p.drag = 0.35f;
			p.rot = Rr(0.0f, 6.28f);
			p.rotVel = Rr(-1.2f, 1.2f);
			p.swirl = 0.8f;
			p.tile = 3;
			p.portal = a_index;
			p.flicker = 0.6f;
			const auto& pal = kPal[a_p.colour & 1];
			for (int k = 0; k < 3; ++k) {
				p.rgb[k] = pal[2][k] * Rr(2.6f, 4.0f);
			}
			Add(p);
		}

		// the opening burst: a ring of wisps and motes thrown out of the oval
		void Burst(const portals::Portal& a_p, int a_index)
		{
			for (int k = 0; k < 46; ++k) {
				const bool         mote = (k % 3) == 0;
				const float        ang = Rr(0.0f, 6.2831853f), rad = std::sqrt(Rf());
				const float        ca = std::cos(ang), sa = std::sin(ang);
				const RE::NiPoint3 dir = a_p.r * (ca * a_p.hw) + a_p.u * (sa * a_p.hh);
				const RE::NiPoint3 c = a_p.c + dir * (rad * 0.9f) + a_p.n * Rr(6.0f, 22.0f);
				const RE::NiPoint3 out = (a_p.r * ca + a_p.u * sa) * Rr(25.0f, 95.0f) * (0.4f + 0.6f * rad) + a_p.n * Rr(30.0f, 110.0f);
				Particle           p;
				p.pos[0] = c.x; p.pos[1] = c.y; p.pos[2] = c.z;
				p.vel[0] = out.x; p.vel[1] = out.y; p.vel[2] = out.z + Rr(0.0f, 16.0f);
				p.life = mote ? Rr(0.8f, 1.5f) : Rr(1.0f, 1.9f);
				p.size0 = mote ? Rr(1.2f, 2.4f) : Rr(16.0f, 28.0f);
				p.size1 = mote ? Rr(0.8f, 1.6f) : Rr(46.0f, 78.0f);
				p.alpha = mote ? Rr(0.7f, 1.0f) : Rr(0.22f, 0.40f);
				p.drag = mote ? 1.6f : 2.1f;
				p.rot = Rr(0.0f, 6.28f);
				p.rotVel = Rr(-0.6f, 0.6f);
				p.swirl = 0.6f;
				p.tile = mote ? 3 : static_cast<int>(Rf() * 2.999f);
				p.portal = a_index;
				SetColour(p, a_p.colour, mote);
				Add(p);
			}
		}

		// G310: a ring of sparks thrown out of the oval as it opens
		void SparkRing(const portals::Portal& a_p)
		{
			const auto& pal = kPal[a_p.colour & 1];
			for (int k = 0; k < 44; ++k) {
				const float        ang = 6.2831853f * (static_cast<float>(k) + Rf()) / 44.0f;
				const float        ca = std::cos(ang), sa = std::sin(ang);
				const RE::NiPoint3 c = a_p.c + a_p.r * (ca * a_p.hw) + a_p.u * (sa * a_p.hh) + a_p.n * Rr(2.0f, 8.0f);
				const RE::NiPoint3 v = (a_p.r * ca + a_p.u * sa) * Rr(150.0f, 330.0f) + a_p.n * Rr(20.0f, 90.0f);
				Particle           p;
				p.pos[0] = c.x; p.pos[1] = c.y; p.pos[2] = c.z;
				p.vel[0] = v.x; p.vel[1] = v.y; p.vel[2] = v.z;
				p.life = Rr(0.22f, 0.45f);
				p.size0 = Rr(1.6f, 2.8f);
				p.size1 = Rr(0.5f, 1.0f);
				p.alpha = 1.0f;
				p.drag = 3.2f;
				p.swirl = 0.0f;
				p.tile = 3;
				for (int j = 0; j < 3; ++j) p.rgb[j] = pal[2][j] * Rr(3.0f, 4.5f);
				Add(p);
			}
		}

		// G310: the closing implosion: sparks drawn in from the rim to the centre in the time the oval takes to collapse
		void Implode(const portals::Portal& a_p)
		{
			const auto& pal = kPal[a_p.colour & 1];
			for (int k = 0; k < 40; ++k) {
				const float        ang = Rr(0.0f, 6.2831853f);
				const RE::NiPoint3 c = a_p.c + a_p.r * (std::cos(ang) * a_p.hw * 1.05f) + a_p.u * (std::sin(ang) * a_p.hh * 1.05f) + a_p.n * Rr(2.0f, 8.0f);
				const RE::NiPoint3 v = (a_p.c + a_p.n * 6.0f - c) * (1.0f / 0.19f);
				Particle           p;
				p.pos[0] = c.x; p.pos[1] = c.y; p.pos[2] = c.z;
				p.vel[0] = v.x; p.vel[1] = v.y; p.vel[2] = v.z;
				p.life = 0.19f;
				p.size0 = Rr(1.8f, 3.2f);
				p.size1 = 0.8f;
				p.alpha = 1.0f;
				p.drag = 0.0f;
				p.swirl = 0.0f;
				p.tile = 3;
				for (int j = 0; j < 3; ++j) p.rgb[j] = pal[2][j] * Rr(2.5f, 4.0f);
				Add(p);
			}
		}

		// G310: the pop at the end of the closing: sparks and a few wisps thrown out of the point the oval collapsed into
		void Pop(const portals::Portal& a_p, int a_index)
		{
			const auto& pal = kPal[a_p.colour & 1];
			for (int k = 0; k < 52; ++k) {
				const bool         wisp = k >= 46;
				const float        ang = Rr(0.0f, 6.2831853f);
				const RE::NiPoint3 v = (a_p.r * std::cos(ang) + a_p.u * std::sin(ang)) * Rr(110.0f, 300.0f) + a_p.n * Rr(40.0f, 160.0f);
				Particle           p;
				const RE::NiPoint3 c = a_p.c + a_p.n * 6.0f;
				p.pos[0] = c.x; p.pos[1] = c.y; p.pos[2] = c.z;
				p.vel[0] = v.x * (wisp ? 0.25f : 1.0f); p.vel[1] = v.y * (wisp ? 0.25f : 1.0f); p.vel[2] = v.z * (wisp ? 0.25f : 1.0f);
				p.life = wisp ? Rr(0.6f, 0.9f) : Rr(0.25f, 0.5f);
				p.size0 = wisp ? Rr(14.0f, 22.0f) : Rr(1.6f, 2.8f);
				p.size1 = wisp ? Rr(40.0f, 60.0f) : Rr(0.5f, 1.0f);
				p.alpha = wisp ? Rr(0.18f, 0.30f) : 1.0f;
				p.drag = wisp ? 2.5f : 3.0f;
				p.rot = Rr(0.0f, 6.28f);
				p.rotVel = Rr(-0.6f, 0.6f);
				p.swirl = wisp ? 0.6f : 0.0f;
				p.tile = wisp ? static_cast<int>(Rf() * 2.999f) : 3;
				p.portal = wisp ? a_index : -1;
				if (wisp) SetColour(p, a_p.colour, false);
				else for (int j = 0; j < 3; ++j) p.rgb[j] = pal[2][j] * Rr(3.0f, 4.5f);
				Add(p);
			}
		}

		// G310: the puff of mist at the portal the player has just come out of: wisps thrown out of the oval along its normal, where the player now stands
		void Puff(const portals::Portal& a_p, int a_index, float a_strength)
		{
			const int n = static_cast<int>(34.0f * (std::min)(a_strength, 2.0f));
			for (int k = 0; k < n; ++k) {
				const bool         mote = (k % 4) == 0;
				const float        ang = Rr(0.0f, 6.2831853f), rad = std::sqrt(Rf());
				const RE::NiPoint3 c = a_p.c + a_p.r * (std::cos(ang) * a_p.hw * rad * 0.85f) + a_p.u * (std::sin(ang) * a_p.hh * rad * 0.85f) + a_p.n * Rr(4.0f, 18.0f);
				const RE::NiPoint3 v = a_p.n * Rr(40.0f, 120.0f) + (a_p.r * std::cos(ang) + a_p.u * std::sin(ang)) * Rr(10.0f, 40.0f);
				Particle           p;
				p.pos[0] = c.x; p.pos[1] = c.y; p.pos[2] = c.z;
				p.vel[0] = v.x; p.vel[1] = v.y; p.vel[2] = v.z;
				p.life = mote ? Rr(0.7f, 1.3f) : Rr(0.9f, 1.5f);
				p.size0 = mote ? Rr(1.2f, 2.4f) : Rr(16.0f, 26.0f);
				p.size1 = mote ? Rr(0.8f, 1.6f) : Rr(46.0f, 74.0f);
				p.alpha = mote ? Rr(0.7f, 1.0f) : Rr(0.20f, 0.34f);
				p.drag = 2.0f;
				p.rot = Rr(0.0f, 6.28f);
				p.rotVel = Rr(-0.6f, 0.6f);
				p.swirl = 0.6f;
				p.tile = mote ? 3 : static_cast<int>(Rf() * 2.999f);
				p.portal = a_index;
				SetColour(p, a_p.colour, mote);
				Add(p);
			}
		}

		// G310: sparks thrown off the casting hand when the spell lands
		void ReleaseSparks(int a_colour, int a_hand, bool a_camAxes)
		{
			const auto& pal = kPal[a_colour & 1];
			for (int k = 0; k < 26; ++k) {
				const float ct = Rr(-1.0f, 1.0f), st = std::sqrt(1.0f - ct * ct), ph = Rr(0.0f, 6.2831853f);
				const float sp = Rr(25.0f, 60.0f);
				Particle    p;
				p.local = true;
				p.camAxes = a_camAxes;
				p.vel[0] = st * std::cos(ph) * sp; p.vel[1] = st * std::sin(ph) * sp; p.vel[2] = ct * sp;
				p.life = Rr(0.2f, 0.4f);
				p.size0 = Rr(0.8f, 1.6f);
				p.size1 = Rr(0.3f, 0.7f);
				p.alpha = 1.0f;
				p.drag = 6.0f;
				p.swirl = 0.0f;
				p.tile = 3;
				for (int j = 0; j < 3; ++j) p.rgb[j] = pal[2][j] * Rr(3.0f, 4.5f);
				p.hand = a_hand;
				Add(p);
			}
		}

		// a wisp at the casting hand
		void EmitHand(int a_colour, bool a_mote, int a_hand, bool a_camAxes)
		{
			Particle p;
			p.local = true;
			p.camAxes = a_camAxes;
			const int up = a_camAxes ? 1 : 2;  // camera axes: y is up; world axes: z is up
			for (int j = 0; j < 3; ++j) {
				p.pos[j] = Rr(-1.0f, 1.0f);
				p.vel[j] = Rr(-1.5f, 1.5f);
			}
			p.vel[up] = Rr(2.0f, 6.0f);
			p.life = a_mote ? Rr(0.5f, 1.0f) : Rr(0.6f, 1.1f);
			p.size0 = a_mote ? Rr(0.5f, 0.9f) : Rr(2.2f, 3.6f);
			p.size1 = a_mote ? Rr(0.3f, 0.6f) : Rr(6.0f, 10.0f);
			p.alpha = a_mote ? Rr(0.6f, 0.95f) : Rr(0.16f, 0.30f);
			p.drag = 0.8f;
			p.rot = Rr(0.0f, 6.28f);
			p.rotVel = Rr(-0.8f, 0.8f);
			p.swirl = 0.0f;
			p.tile = a_mote ? 3 : static_cast<int>(Rf() * 2.999f);
			SetColour(p, a_colour, a_mote);
			p.hand = a_hand;
			Add(p);
		}

		void Step(double a_now, const world::CamInfo& a_cam, const portals::Portal* a_portal[2], const spells::Style& a_style)
		{
			float dt = g_lastNow > 0.0 ? static_cast<float>(a_now - g_lastNow) : 0.0f;
			g_lastNow = a_now;
			dt = std::clamp(dt, 0.0f, 0.1f);
			const float t = static_cast<float>(std::fmod(a_now, 1000.0));
			// the hands now (the magic node as the engine has it while the overlay draws) and how far each moved since the last frame: its particles move with it
			spells::HandFx hands[2];
			spells::GetHandFx(hands);
			RE::NiPoint3 handNow[2]{};
			bool         handOk[2]{ false, false };
			RE::NiPoint3 handMove[2]{};
			bool         carry[2]{ false, false };
			for (int h = 0; h < 2; ++h) {
				// G412: here (before the engine draws the first-person arms) the first-person node is not in world space (it read ~21,860 units off, near the origin), so in
				// first person the hand is its position relative to the camera, rebuilt with this frame's camera; in third person the third-person node is in world space
				if (!hands[h].active) {
					g_handPrevOk[h] = false;
					continue;
				}
				if (hands[h].firstPerson) {
					const RE::NiPoint3 ref = a_cam.pos + a_cam.right * hands[h].camRel.x + a_cam.up * hands[h].camRel.y + a_cam.fwd * hands[h].camRel.z;
					handNow[h] = ref;
					RE::NiPoint3 node;
					const bool   rel = spells::HandWorldNow(h, node) && Len3(node) < 1000.0f && Len3(node - a_cam.pos) > 1000.0f;  // camera-relative only while the arms are not drawn yet
					if (rel) {
						const RE::NiPoint3 cand[3] = { a_cam.pos + node, a_cam.pos + a_cam.fwd * node.x + a_cam.up * node.y + a_cam.right * node.z,
							a_cam.pos + a_cam.right * node.x + a_cam.fwd * node.y + a_cam.up * node.z };
						const bool still = g_prevCamOk && Len3(a_cam.pos - g_prevCamPos) < 0.3f &&
						                   a_cam.fwd.x * g_prevCamFwd.x + a_cam.fwd.y * g_prevCamFwd.y + a_cam.fwd.z * g_prevCamFwd.z > 0.99999f;
						if (still) {
							// standing still the start-of-frame position is exact: it tells which convention the camera-relative node uses
							for (int m = 0; m < 3; ++m) g_modeErr[m] += Len3(cand[m] - ref);
							if (++g_modeN >= 20) {
								int best = 0;
								for (int m = 1; m < 3; ++m) if (g_modeErr[m] < g_modeErr[best]) best = m;
								const int    pick = g_modeErr[best] / g_modeN < 4.0 ? best : -1;
								if (pick != g_anchorMode) {
									logger::warn("handfx: anchor {} (mean errors against the start-of-frame hand over {} still frames: {:.1f} / {:.1f} / {:.1f} units)", pick < 0 ? std::string("start-of-frame") : std::format("camera + node, convention {}", pick),
										g_modeN, g_modeErr[0] / g_modeN, g_modeErr[1] / g_modeN, g_modeErr[2] / g_modeN);
									g_anchorMode = pick;
								}
								g_modeErr[0] = g_modeErr[1] = g_modeErr[2] = 0.0;
								g_modeN = 0;
							}
						}
					}
				} else if (!spells::HandWorldNow(h, handNow[h])) {
					handNow[h] = hands[h].pos;
				}
				handOk[h] = true;
				g_anchorRel[h] = handNow[h] - a_cam.pos;
				g_anchorFrame[h] = frame::Count();
				g_handDraw[h] = handNow[h];
				if (handOk[h] && g_handPrevOk[h] && g_handCarry.load()) {
					handMove[h] = handNow[h] - g_handPrev[h];
					// a jump (a walk through a portal, a load) is not a hand movement: the particles stay where they are
					carry[h] = handMove[h].x * handMove[h].x + handMove[h].y * handMove[h].y + handMove[h].z * handMove[h].z < 150.0f * 150.0f;
				}
				g_handPrev[h] = handNow[h];
				g_handPrevOk[h] = handOk[h];
			}
			if (g_handFxLog.load() && (hands[0].active || hands[1].active) && (++g_hfxTick & 1) == 0) {
				auto*              pl = RE::PlayerCharacter::GetSingleton();
				const RE::NiPoint3 pp = pl ? pl->GetPosition() : RE::NiPoint3{};
				const double       pdt = a_now - g_prevPlayerT;
				RE::NiPoint3       v{};
				if (g_prevPlayerT > 0.0 && pdt > 1e-4 && pdt < 0.5) v = (pp - g_prevPlayer) * static_cast<float>(1.0 / pdt);
				const float vf = v.x * a_cam.fwd.x + v.y * a_cam.fwd.y, vs = v.x * a_cam.right.x + v.y * a_cam.right.y;
				const float pxPerUnitAt1 = g_screenW / (std::max)(a_cam.tanRight - a_cam.tanLeft, 0.01f);
				for (int h = 0; h < 2; ++h) {
					if (!hands[h].active || !handOk[h]) continue;
					const RE::NiPoint3 d0 = handNow[h] - a_cam.pos;
					const float        depth = (std::max)(d0.x * a_cam.fwd.x + d0.y * a_cam.fwd.y + d0.z * a_cam.fwd.z, 1.0f);
					const float        oldOff = Len3(hands[h].pos - handNow[h]);
					int                n = 0;
					RE::NiPoint3       cen{};
					float              mxOff = 0.0f;
					for (const auto& p : g_parts) {
						if (p.hand != h || !p.local) continue;
						cen.x += p.pos[0]; cen.y += p.pos[1]; cen.z += p.pos[2];
						mxOff = (std::max)(mxOff, std::sqrt(p.pos[0] * p.pos[0] + p.pos[1] * p.pos[1] + p.pos[2] * p.pos[2]));
						++n;
					}
					if (n) cen = cen * (1.0f / n);
					const float wispOff = n ? Len3(cen) : 0.0f;  // the offsets are hand-local: the frame's movement cannot enter this
					logger::warn("handfx: t {:.3f} {} hand {} anchor {} speed fwd {:.0f} side {:.0f} u/s | arms hand ({:.1f},{:.1f},{:.1f}) depth {:.0f} | ring+glow at the anchor | start-of-frame pos off {:.1f} u = {:.0f} px | wisps {} (hand-local) centre off {:.1f} u = {:.0f} px, farthest {:.1f} u = {:.0f} px",
						a_now, hands[h].firstPerson ? "1st" : "3rd", h, g_anchorMode, vf, vs, handNow[h].x, handNow[h].y, handNow[h].z, depth, oldOff, oldOff / depth * pxPerUnitAt1, n, wispOff, wispOff / depth * pxPerUnitAt1, mxOff, mxOff / depth * pxPerUnitAt1);
				}
				g_prevPlayer = pp;
				g_prevPlayerT = a_now;
			}
			g_prevCamPos = a_cam.pos;
			g_prevCamFwd = a_cam.fwd;
			g_prevCamOk = true;
			// age, drift, retire
			for (auto& p : g_parts) {
				p.age += dt;
				const float damp = std::exp(-p.drag * dt);
				p.vel[0] *= damp; p.vel[1] *= damp; p.vel[2] *= damp;
				if (p.local) {
					p.pos[0] += p.vel[0] * dt; p.pos[1] += p.vel[1] * dt; p.pos[2] += p.vel[2] * dt;
					p.rot += p.rotVel * dt;
					continue;
				}
				// slow curl: a smooth, position-dependent push that makes the smoke meander
				const float k = 0.014f * p.swirl;
				p.vel[0] += 9.0f * std::sin(p.pos[1] * k + t * 0.7f + p.rot) * dt;
				p.vel[1] += 9.0f * std::sin(p.pos[2] * k + t * 0.9f + p.rot * 1.7f) * dt;
				p.vel[2] += 6.0f * std::sin(p.pos[0] * k + t * 0.5f + p.rot * 2.3f) * dt;
				if (p.portal >= 0 && p.portal < 2 && a_portal[p.portal] && a_portal[p.portal]->valid && a_style.mistSwirl > 0.001f) {
					// MistSwirl: the mist turns around the portal's axis, teal one way and violet the other
					const auto&        pt = *a_portal[p.portal];
					const RE::NiPoint3 rel{ p.pos[0] - pt.c.x, p.pos[1] - pt.c.y, p.pos[2] - pt.c.z };
					const float        sx = rel.x * pt.r.x + rel.y * pt.r.y + rel.z * pt.r.z;
					const float        sy = rel.x * pt.u.x + rel.y * pt.u.y + rel.z * pt.u.z;
					const float        rr = (std::max)(std::sqrt(sx * sx + sy * sy), 1.0f);
					const float        sp = ((pt.colour == 0) ? 1.0f : -1.0f) * 22.0f * a_style.mistSwirl * p.swirl * dt;
					p.vel[0] += (pt.r.x * (-sy / rr) + pt.u.x * (sx / rr)) * sp;
					p.vel[1] += (pt.r.y * (-sy / rr) + pt.u.y * (sx / rr)) * sp;
					p.vel[2] += (pt.r.z * (-sy / rr) + pt.u.z * (sx / rr)) * sp;
				}
				p.pos[0] += p.vel[0] * dt; p.pos[1] += p.vel[1] * dt; p.pos[2] += p.vel[2] * dt;
				if (p.hand >= 0 && p.hand < 2 && carry[p.hand]) {
					p.pos[0] += handMove[p.hand].x; p.pos[1] += handMove[p.hand].y; p.pos[2] += handMove[p.hand].z;
				}
				p.rot += p.rotVel * dt;
			}
			g_parts.erase(std::remove_if(g_parts.begin(), g_parts.end(), [](const Particle& p) { return p.age >= p.life; }), g_parts.end());
			// portals: bursts on a new placement, a steady drift off the rim
			for (int i = 0; i < 2; ++i) {
				const auto* pt = a_portal[i];
				if (!pt || !pt->valid) {
					g_seenPlaced[i] = 0.0;
					g_burstWaiting[i] = false;
					continue;
				}
				if (pt->placedAtSec != g_seenPlaced[i]) {
					g_seenPlaced[i] = pt->placedAtSec;
					if (a_style.openAnim > 0.001f) {
						g_burstWaiting[i] = true;
						SparkRing(*pt);
					} else {
						Burst(*pt, i);
					}
					if (a_style.releaseFlash > 0.001f) {
						for (int h = 0; h < 2 && g_handWisps.load(); ++h) {
							if (hands[h].colour == pt->colour && a_now - hands[h].at < 1.0) {
								ReleaseSparks(pt->colour, h, hands[h].firstPerson);
							}
						}
					}
				}
				if (g_burstWaiting[i] && portals::OpenFraction(i, a_now) >= 0.5f) {
					g_burstWaiting[i] = false;
					Burst(*pt, i);
				}
				const RE::NiPoint3 dv = pt->c - a_cam.pos;
				if (dv.x * dv.x + dv.y * dv.y + dv.z * dv.z > 6000.0f * 6000.0f) {
					continue;
				}
				const float open = std::clamp(static_cast<float>((a_now - pt->placedAtSec) / 0.6), 0.0f, 1.0f);
				g_accWisp[i] += 14.0f * open * dt * a_style.density;
				g_accMote[i] += 22.0f * open * dt * a_style.density;
				for (; g_accWisp[i] >= 1.0f; g_accWisp[i] -= 1.0f) EmitRim(*pt, i, false);
				for (; g_accMote[i] >= 1.0f; g_accMote[i] -= 1.0f) EmitRim(*pt, i, true);
				g_accEmber[i] += 9.0f * open * dt * a_style.embers * a_style.density;
				for (; g_accEmber[i] >= 1.0f; g_accEmber[i] -= 1.0f) EmitEmber(*pt, i);
			}
			// G310: the closing of a portal a recast replaced (sparks drawn in, then a pop) and the mist puff at the exit of a walk through
			for (int i = 0; i < 2; ++i) {
				const auto g = portals::GetGhost(i);
				if (!g.p.valid) {
					continue;
				}
				if (g.closedAtSec != g_seenGhost[i]) {
					g_seenGhost[i] = g.closedAtSec;
					g_popDone[i] = false;
					if (a_style.closeAnim > 0.001f) {
						Implode(g.p);
					}
				}
				if (!g_popDone[i] && a_style.closeAnim > 0.001f && g.progress >= 0.75f) {
					g_popDone[i] = true;
					Pop(g.p, i);
				}
			}
			{
				const auto cross = portals::LastCross();
				if (cross.at != g_seenCross) {
					g_seenCross = cross.at;
					if (cross.at > 0.0 && a_style.exitPuff > 0.001f && a_portal[cross.exitColour & 1] && a_portal[cross.exitColour & 1]->valid) {
						Puff(*a_portal[cross.exitColour & 1], cross.exitColour & 1, a_style.exitPuff);
					}
				}
			}
			// hands: the wisps, emitted where the hand is now
			for (int h = 0; h < 2; ++h) {
				if (!g_handWisps.load() || !hands[h].active || hands[h].colour < 0 || a_now - hands[h].at > 0.25) {
					g_accHand[h] = 0.0f;
					continue;
				}
				g_accHand[h] += 60.0f * dt;
				for (; g_accHand[h] >= 1.0f; g_accHand[h] -= 1.0f) EmitHand(hands[h].colour, (static_cast<int>(g_accHand[h] * 7.0f) & 1) != 0, h, hands[h].firstPerson);
			}
			if ((hands[0].active || hands[1].active) && ++g_handLogTick % 30 == 0 && g_handLogged < 300) {
				for (int h = 0; h < 2; ++h) {
					if (!hands[h].active || !handOk[h]) continue;
					int   n = 0;
					float sum = 0.0f, mx = 0.0f;
					for (const auto& p : g_parts) {
						if (p.hand != h && !(p.hand < 0 && p.portal < 0)) continue;
						const float dx = p.pos[0] - handNow[h].x, dy = p.pos[1] - handNow[h].y, dz = p.pos[2] - handNow[h].z;
						const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
						++n;
						sum += d;
						mx = (std::max)(mx, d);
					}
					++g_handLogged;
					logger::info("fx: hand {} particles {} mean {:.1f} max {:.1f} units from the hand (carry {}, hand moved {:.1f} units this frame)", h, n, n ? sum / n : 0.0f, mx, g_handCarry.load(), std::sqrt(handMove[h].x * handMove[h].x + handMove[h].y * handMove[h].y + handMove[h].z * handMove[h].z));
				}
			}
			g_count = static_cast<int>(g_parts.size());
		}

		// How far a particle's centre is from the rim of its portal, in units: the distance outside the oval in the oval's plane (same signed-distance approximation as the overlay shader),
		// plus half of the height above the plane (seen at an angle, a sprite in front of the wall lands next to where it is projected onto it).
		float RimDistance(const Particle& a_p, const portals::Portal& a_pt)
		{
			const RE::NiPoint3 rel{ a_p.pos[0] - a_pt.c.x, a_p.pos[1] - a_pt.c.y, a_p.pos[2] - a_pt.c.z };
			const float        x = rel.x * a_pt.r.x + rel.y * a_pt.r.y + rel.z * a_pt.r.z;
			const float        y = rel.x * a_pt.u.x + rel.y * a_pt.u.y + rel.z * a_pt.u.z;
			const float        z = rel.x * a_pt.n.x + rel.y * a_pt.n.y + rel.z * a_pt.n.z;
			const float        hw = (std::max)(a_pt.hw, 1.0f), hh = (std::max)(a_pt.hh, 1.0f);
			const float        nx = x / hw, ny = y / hh;
			const float        k0 = std::sqrt(nx * nx + ny * ny);
			const float        gx = x / (hw * hw), gy = y / (hh * hh);
			const float        k1 = (std::max)(std::sqrt(gx * gx + gy * gy), 1e-5f);
			const float        out = (k0 > 0.02f) ? k0 * (k0 - 1.0f) / k1 : -(std::min)(hw, hh);
			return (std::max)(out, 0.0f) + 0.5f * std::fabs(z);
		}

		// where a particle is drawn: a hand-local one at the hand of the frame plus its offset (camera axes in first person)
		RE::NiPoint3 WorldPos(const Particle& a_p, const world::CamInfo& a_cam)
		{
			if (!a_p.local || a_p.hand < 0 || a_p.hand > 1) {
				return { a_p.pos[0], a_p.pos[1], a_p.pos[2] };
			}
			const RE::NiPoint3& b = g_handDraw[a_p.hand];
			if (a_p.camAxes) {
				return b + a_cam.right * a_p.pos[0] + a_cam.up * a_p.pos[1] + a_cam.fwd * a_p.pos[2];
			}
			return { b.x + a_p.pos[0], b.y + a_p.pos[1], b.z + a_p.pos[2] };
		}

		bool RenderImpl(const Frame& a_f, const world::CamInfo& a_cam, const portals::Portal& a_teal, const portals::Portal& a_violet)
		{
			auto* dev = static_cast<ID3D11Device*>(a_f.dev);
			auto* ctx = static_cast<ID3D11DeviceContext*>(a_f.ctx);
			if (!dev || !ctx) {
				return false;
			}
			if (!g_ready) {
				g_ready = Init(dev);
				if (!g_ready) {
					g_failed = true;
					return false;
				}
			}
			const portals::Portal* pts[2] = { &a_teal, &a_violet };
			const auto style = spells::GetStyle();
			if (a_f.width > 0) g_screenW = static_cast<float>(a_f.width);
			Step(a_f.now, a_cam, pts, style);
			if (g_parts.empty()) {
				return true;
			}
			// far to near
			struct Item
			{
				float dist;
				const Particle* p;
			};
			std::vector<Item> items;
			items.reserve(g_parts.size());
			for (const auto& p : g_parts) {
				const RE::NiPoint3 w = WorldPos(p, a_cam);
				const float        d = (w.x - a_cam.pos.x) * a_cam.fwd.x + (w.y - a_cam.pos.y) * a_cam.fwd.y + (w.z - a_cam.pos.z) * a_cam.fwd.z;
				if (d > 2.0f) {
					items.push_back({ d, &p });
				}
			}
			if (items.empty()) {
				return true;
			}
			std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.dist > b.dist; });

			const FLOAT bf[4] = { 0, 0, 0, 0 };
			ctx->OMSetBlendState(g_blend.Get(), bf, 0xFFFFFFFF);
			ctx->OMSetDepthStencilState(static_cast<ID3D11DepthStencilState*>(a_f.dssTest), 0);
			ctx->VSSetShader(g_vs.Get(), nullptr, 0);
			ctx->PSSetShader(g_ps.Get(), nullptr, 0);
			ID3D11Buffer* cb = g_cb.Get();
			ctx->VSSetConstantBuffers(0, 1, &cb);
			ctx->PSSetConstantBuffers(0, 1, &cb);
			ID3D11ShaderResourceView* srv = g_srv.Get();
			ctx->PSSetShaderResources(2, 1, &srv);

			const double now = a_f.now;
			(void)now;
			static CB c;  // too large for the stack of a render-thread callback frame
			auto put = [](float* a_dst, const RE::NiPoint3& a_v, float a_w) { a_dst[0] = a_v.x; a_dst[1] = a_v.y; a_dst[2] = a_v.z; a_dst[3] = a_w; };
			put(c.camPos, a_cam.pos, 0);
			put(c.camFwd, a_cam.fwd, 0);
			put(c.camUp, a_cam.up, 0);
			put(c.camRight, a_cam.right, 0);
			c.frustum[0] = a_cam.tanLeft; c.frustum[1] = a_cam.tanRight; c.frustum[2] = a_cam.tanBottom; c.frustum[3] = a_cam.tanTop;
			c.depthP[0] = a_cam.nearDist; c.depthP[1] = a_cam.farDist; c.depthP[2] = static_cast<float>(a_f.depthMode); c.depthP[3] = 0.0f;
			for (std::size_t first = 0; first < items.size(); first += kBatch) {
				const std::size_t n = (std::min<std::size_t>)(kBatch, items.size() - first);
				for (std::size_t k = 0; k < n; ++k) {
					const Particle& p = *items[first + k].p;
					const float     t = std::clamp(p.age / p.life, 0.0f, 1.0f);
					const float     env = std::clamp(t / 0.18f, 0.0f, 1.0f) * (1.0f - std::clamp((t - 0.55f) / 0.45f, 0.0f, 1.0f));
					const float     sm = env * env * (3.0f - 2.0f * env);
					float           size = p.size0 + (p.size1 - p.size0) * std::sqrt(t);
					if (p.portal >= 0 && p.portal < 2) {
						// kept within kReach of the rim: the far edge of the sprite stops there, and the sprite is gone when its centre gets there
						const portals::Portal& pt = *pts[p.portal];
						size = pt.valid ? std::clamp(g_reach.load() - RimDistance(p, pt), 0.0f, size) : 0.0f;
					}
					const RE::NiPoint3 w = WorldPos(p, a_cam);
					c.p0[k][0] = w.x; c.p0[k][1] = w.y; c.p0[k][2] = w.z; c.p0[k][3] = size;
					c.p1[k][0] = p.rgb[0]; c.p1[k][1] = p.rgb[1]; c.p1[k][2] = p.rgb[2]; c.p1[k][3] = p.alpha * sm * style.alpha * (1.0f - p.flicker * (0.5f + 0.5f * std::sin(p.age * 21.0f + p.rot * 7.0f)));
					c.p2[k][0] = p.rot; c.p2[k][1] = static_cast<float>(p.tile); c.p2[k][2] = 0.0f; c.p2[k][3] = 0.0f;
				}
				D3D11_MAPPED_SUBRESOURCE m{};
				if (FAILED(ctx->Map(g_cb.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &m))) {
					break;
				}
				std::memcpy(m.pData, &c, sizeof(c));
				ctx->Unmap(g_cb.Get(), 0);
				ctx->DrawInstanced(4, static_cast<UINT>(n), 0, 0);
			}
			ID3D11ShaderResourceView* none = nullptr;
			ctx->PSSetShaderResources(2, 1, &none);
			return true;
		}

		void LogFailure()
		{
			logger::error("fx: an exception was raised while simulating or drawing the particles; the particles are disabled for this session");
		}
	}

	bool Active()
	{
		if (!g_on.load() || g_failed || !spells::ParticlesEnabled()) {
			return false;
		}
		if (g_count.load() > 0) {
			return true;
		}
		spells::HandFx hands[2];
		spells::GetHandFx(hands);
		return hands[0].active || hands[1].active;
	}

	void Render(const Frame& a_frame, const world::CamInfo& a_cam, const portals::Portal& a_teal, const portals::Portal& a_violet)
	{
		if (!g_on.load() || g_failed || !spells::ParticlesEnabled()) {
			return;
		}
		__try {
			RenderImpl(a_frame, a_cam, a_teal, a_violet);
		} __except (EXCEPTION_EXECUTE_HANDLER) {
			g_failed = true;
			LogFailure();
		}
	}

	void SetEnabled(bool a_on)
	{
		g_on = a_on;
		logger::info("fx: particles {}", a_on ? "ON" : "off");
	}

	void SetHandCarry(bool a_on)
	{
		g_handCarry = a_on;
		logger::info("fx: hand particles {}", a_on ? "move with the hand" : "stay in the world (old)");
	}

	void SetHandWisps(bool a_on)
	{
		if (g_handWisps.exchange(a_on) != a_on) {
			logger::info("fx: hand wisps {}", a_on ? "on" : "off");
		}
	}

	void SetHandFxLog(bool a_on)
	{
		if (g_handFxLog.exchange(a_on) != a_on) {
			logger::warn("handfx: position log {}", a_on ? "ON (Riftweaver.ini HandFxLog=1)" : "off");
		}
	}

	bool HandAnchor(int a_hand, const world::CamInfo& a_cam, RE::NiPoint3& a_out)
	{
		if (a_hand < 0 || a_hand > 1 || !g_handCarry.load() || g_anchorFrame[a_hand] != frame::Count()) {
			return false;
		}
		a_out = a_cam.pos + g_anchorRel[a_hand];
		return true;
	}

	void SetReach(float a_units)
	{
		g_reach = (std::max)(a_units, 0.0f);
		logger::info("fx: particles reach {:.0f} units from the rim", g_reach.load());
	}

	void LogStatus()
	{
		logger::info("fx: on {} failed {} ready {} live particles {} (max {})", g_on.load(), g_failed, g_ready, g_count.load(), kMax);
	}
}
