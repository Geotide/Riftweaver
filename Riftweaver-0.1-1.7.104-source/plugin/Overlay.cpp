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

// Draws the placed portals as procedural ovals into the engine's main HDR target, after the world and before post-processing,
// with the hardware depth test against the engine's depth buffer (so anything in front of a portal hides it). The oval also WRITES its depth, a few units in
// front of the wall it sits on: the engine's post-processing (fog, ambient occlusion, depth of field) reads that buffer, and the gaps between the wall's
// boards (which hold the far background's depth) used to show through the picture as pale vertical lines.
// Camera projection is done by hand from the main camera (basis + frustum tangents) so nothing depends on engine matrices.
// The engine keeps a shadow copy of the D3D11 state, so every piece of state we touch is saved and restored exactly.
namespace overlay
{
	using Microsoft::WRL::ComPtr;

	namespace
	{
		constexpr int kMainRT = 1;  // engine render target table: 1 = float16 HDR main (verified in the prototype)

		std::atomic<bool> g_enabled{ true };
		std::atomic<std::uint64_t> g_momentsFrame{ 0 };  // G310: the last frame DrawMoments ran from the engine's VL composite (after the first-person hands); when that stops, Draw runs it itself
		std::atomic<int>  g_depthMode{ 0 };  // 0 standard (0 near .. 1 far; verified in game), 1 reversed, 2 none
		std::atomic<bool> g_probe{ false };
		std::atomic<bool> g_depthFar{ false };
		std::atomic<float> g_depthBias{ 24.0f };  // units. The oval sits on the front-most point the placement rays found, but the wall's relief between the rays
		                                           // (boards, rails, nail heads) can stand several units in front of it; those bits must be covered by the picture.
		bool              g_ready{ false };
		bool              g_failed{ false };

		ComPtr<ID3D11VertexShader>      g_vs, g_vsDoor, g_vsExit;
		ComPtr<ID3D11PixelShader>       g_ps, g_psDoor, g_psExit, g_psLight, g_psHand, g_psRipple, g_psRune;
		ComPtr<ID3D11Buffer>            g_cb;
		ComPtr<ID3D11BlendState>        g_blend, g_blendAdd;
		ComPtr<ID3D11RasterizerState>   g_rs;
		ComPtr<ID3D11DepthStencilState> g_dss[3][2];  // [depth mode: standard, reversed, off][write depth]

		const char* kHlsl = R"HLSL(
cbuffer CB : register(b0)
{
	float4 camPos;
	float4 camFwd;
	float4 camUp;
	float4 camRight;
	float4 frustum;   // tanLeft, tanRight, tanBottom, tanTop
	float4 depthP;    // near, far, mode (0 std, 1 reversed, 2 none), depth bias in units
	float4 pc;        // portal centre, half width
	float4 pn;        // normal, half height
	float4 pu;        // up
	float4 pr;        // right
	float4 colA;      // rim colour (HDR)
	float4 colB;      // interior tint
	float4 misc;      // time, open amount 0..1, has picture, pass (0 = body: oval and rim, writes depth; 1 = halo, no depth write)
	float4 style;     // Riftweaver.ini: x = halo strength (Halo), y = rim strength (Rim)
	float4 style2;    // G309 phase 1: x RimFlow, y RimGlints, z Ripple, w Sheen
	float4 lightP;    // G309: xyz the light's position, w its radius (PSLight)
	float4 lightC;    // G309: rgb the light's colour times its intensity
	float4 scr;       // G309: viewport width, height
	float4 fx;        // G310: x = closing fraction of a replaced portal (0 = open); PSHand: y ring unfold 0..1, z release age (>= 1 none), w strength; PSRipple: unused
};
// the quad extends this many units beyond the oval on every side; the halo is faded to nothing long before it (see Shade)
static const float kPad = 56.0;

Texture2D<float4> viewTex   : register(t1);   // the picture seen through this portal (virtual camera), screen-aligned
Texture2D<float4> vlPre    : register(t4);   // G306: the picture right before its virtual VL ran (style.z > 0.5: the VL add pass after the engine's VL composite)
Texture2D<float4> vlPost   : register(t5);   // G306: the same picture right after it
Texture2D<float4> compBefore : register(t6);  // G308: the main HDR target right before the engine's VL composite
Texture2D<float4> compAfter  : register(t7);  // G308: right after it (the difference is what the engine itself added at the pixel)

struct VSOut
{
	float4 pos  : SV_Position;
	float2 uv   : TEXCOORD0;   // portal-local, in units of half size (1 = the oval edge)
	float  dist : TEXCOORD1;   // view-space forward distance of the quad point
};

VSOut VSMain(uint id : SV_VertexID)
{
	float2 q = float2((id & 1) ? 1.0 : -1.0, (id & 2) ? 1.0 : -1.0);
	// the quad extends 64 units beyond the oval on every side (rim, glow and haze are sized in units, so a tall oval and a narrow one look alike)
	const float padU = kPad;
	float ax = pc.w + padU, ay = pn.w + padU;
	float3 wp = pc.xyz + pr.xyz * (q.x * ax) + pu.xyz * (q.y * ay);
	float3 d = wp - camPos.xyz;
	float f = dot(d, camFwd.xyz);
	float u = dot(d, camUp.xyz);
	float r = dot(d, camRight.xyz);
	VSOut o;
	float fs = max(f, 0.01);
	float nx = 2.0 * (r / fs - frustum.x) / (frustum.y - frustum.x) - 1.0;
	float ny = 2.0 * (u / fs - frustum.z) / (frustum.w - frustum.z) - 1.0;
	// behind the camera: push out of the clip volume
	o.pos = (f > 0.05) ? float4(nx * f, ny * f, 0.5 * f, f) : float4(0, 0, -1, 1);
	o.uv = float2(q.x * ax / pc.w, q.y * ay / pn.w);
	o.dist = f;
	return o;
}

struct PSOut
{
	float4 col   : SV_Target;
	float  depth : SV_Depth;
};

// ---- miasma: slow, domain-warped value noise that drifts outward from the rim ----
float h21(float2 p)
{
	p = frac(p * float2(123.34, 456.21));
	p += dot(p, p + 45.32);
	return frac(p.x * p.y);
}

float vnoise(float2 p)
{
	float2 i = floor(p);
	float2 f = frac(p);
	float2 u = f * f * (3.0 - 2.0 * f);
	float a = h21(i), b = h21(i + float2(1, 0)), c = h21(i + float2(0, 1)), d = h21(i + float2(1, 1));
	return lerp(lerp(a, b, u.x), lerp(c, d, u.x), u.y);
}

float fbm3(float2 p)
{
	float v = 0.0, a = 0.5;
	[unroll] for (int k = 0; k < 3; ++k)
	{
		v += a * vnoise(p);
		p = p * 2.03 + 17.1;
		a *= 0.5;
	}
	return v;
}

// 0..1 smoke density at w (units in the oval's plane); dirOut = unit vector away from the oval's centre
float Mist(float2 w, float2 dirOut, float t)
{
	float2 p = w / 34.0 - dirOut * (t * 0.16) + float2(0.0, -t * 0.07);
	float2 q = float2(fbm3(p + 3.1), fbm3(p + 7.7));
	return fbm3(p + 2.0 * q + float2(t * 0.03, 0.0));
}

// One pixel of the portal surface. w = position in the oval's plane (units from the centre), dist = forward distance of that point from the camera.
// full: the camera is behind the plane, the whole pixel shows the other side (no oval, no rim).
// G307: the depth of the VL add pass: a few more units toward the camera than the oval's own (equal depths would let float noise between the overlay's frame time and the engine's VL composite reject pixels)
float ZNear(float dist)
{
	float fb = max(dist - depthP.w - 6.0, depthP.x * 1.001);
	return (depthP.z < 0.5) ? depthP.y * (fb - depthP.x) / (fb * (depthP.y - depthP.x)) : depthP.x * (depthP.y - fb) / (fb * (depthP.y - depthP.x));
}

// G362: Sparks into violet threw the whole frame white, then black, about twice a second (the engine's bloom and eye adaptation reacting to the picture): a
// pixel of the virtual picture that is not a finite colour, or absurdly bright, must not reach the main HDR target
float3 SafeHdr(float3 c)
{
	return all(isfinite(c)) ? clamp(c, 0.0, 32.0) : float3(0.0, 0.0, 0.0);
}

PSOut Shade(float2 w, float dist, float4 svpos, bool full)
{
	// The depth of the portal surface: the quad moved toward the camera by the bias (the wall's own relief must not poke through it), in the engine's
	// depth convention (standard: 0 near .. 1 far; reversed: 1 near .. 0 far)
	float fb = max(dist - depthP.w, depthP.x * 1.001);
	float zq = (depthP.z < 0.5) ? depthP.y * (fb - depthP.x) / (fb * (depthP.y - depthP.x)) : depthP.x * (depthP.y - fb) / (fb * (depthP.y - depthP.x));
	if (pu.w > 0.5) zq = (depthP.z < 0.5) ? 1.0 : 0.0;   // test: write the far depth
	PSOut o;
	if (full)
	{
		if (misc.w > 0.5) discard;
		if (style.z > 0.5)
		{
			int3 lp = int3((int2)svpos.xy, 0);
			float3 vadd = max(vlPost.Load(lp).rgb - vlPre.Load(lp).rgb, 0.0);
			float3 eadd = (style.w > 0.5) ? max(compAfter.Load(lp).rgb - compBefore.Load(lp).rgb, 0.0) : float3(0.0, 0.0, 0.0);
			o.col = float4(SafeHdr(max(vadd - eadd, 0.0)), 1.0);
			o.depth = ZNear(dist);
			return o;
		}
		float3 vc = SafeHdr((misc.z > 0.5) ? viewTex.Load(int3((int2)svpos.xy, 0)).rgb : colB.rgb * 0.5);
		o.col = float4(vc, 1.0);
		o.depth = zq;
		return o;
	}
	// G310 moments: opening (misc.y 0..1 over 0.25 s) = a thin tear that lengthens, then widens like an iris; closing (fx.x 0..1 over 0.25 s, a portal a recast replaced) = the sides come in to a slit that shrinks to a point, then a pop
	float oT = saturate(misc.y);
	float cT = saturate(fx.x);
	float sx = 1.0;
	float sy = 1.0;
	if (cT > 0.0)
	{
		float ct = saturate(cT / 0.75);
		sx = 1.0 - smoothstep(0.0, 0.7, ct);
		sy = 1.0 - smoothstep(0.3, 1.0, ct);
	}
	else if (oT < 1.0)
	{
		float e1 = saturate(oT / 0.45);
		sy = 1.0 - (1.0 - e1) * (1.0 - e1) * (1.0 - e1);
		sx = lerp(0.05, 1.0, smoothstep(0.25, 1.0, oT));
	}
	sx = max(sx, 0.002);
	sy = max(sy, 0.002);
	float2 wq = float2(w.x / sx, w.y / sy);
	float2 nq = float2(wq.x / pc.w, wq.y / pn.w);
	float f = dot(nq, nq) - 1.0;
	// signed distance to the oval's edge in units (positive outside): k0 (k0 - 1) / k1 is exact for a circle and close for an ellipse, also far from the edge (f / |grad| halves it out there)
	float k0 = length(nq);
	float k1 = max(length(float2(w.x / (sx * sx * pc.w * pc.w), w.y / (sy * sy * pn.w * pn.w))), 1e-5);
	float dist2 = (k0 > 0.02) ? (k0 * (k0 - 1.0) / k1) : (-min(pc.w * sx, pn.w * sy));
	float dd = length(nq);                              // normalised radius, for the swirl and the tint
	float ang = atan2(nq.y, nq.x);
	float t = misc.x;
	float wobU = 0.6 * sin(ang * 7.0 + t * 3.1) + 0.4 * sin(ang * 13.0 - t * 4.3);
	float edge = dist2 + wobU;
	float fill = 1.0 - smoothstep(-0.8, 0.6, edge);
	if (style.z > 0.5)
	{
		// G306: only the virtual VL of the picture ((post - pre), added by the blend) inside the oval, after the engine's VL composite
		if (fill <= 0.003) discard;
		int3 lp = int3((int2)svpos.xy, 0);
		float3 vadd = max(vlPost.Load(lp).rgb - vlPre.Load(lp).rgb, 0.0);
		float3 eadd = (style.w > 0.5) ? max(compAfter.Load(lp).rgb - compBefore.Load(lp).rgb, 0.0) : float3(0.0, 0.0, 0.0);
		o.col = float4(SafeHdr(max(vadd - eadd, 0.0)), fill);
		o.depth = ZNear(dist);
		return o;
	}
	// faded, semi-transparent look: a soft rim band, a wide glow and a drifting haze of miasma around the edge, all tinted by the portal's colour (colA) and its deep shade (colB)
	float pulse = 0.92 + 0.08 * sin(t * 1.3 + ang * 2.0);
	float rq = (edge + 2.0) / 6.5;
	float rim = exp(-rq * rq) * pulse;
	float glow = exp(-max(edge, 0.0) / 9.0) * step(0.0, edge);
	float2 dirOut = nq / max(length(nq), 1e-3);
	float m = Mist(w, dirOut, t);
	// the haze: a soft glow hugging the rim that is exactly zero 30 units out (and long before the quad edge, kPad = 56, so no hard rectangle), dimmed when the camera is close (it must not tint the whole screen), scaled by the ini
	float haze = glow * (1.0 - smoothstep(8.0, 30.0, edge)) * (0.25 + 0.75 * smoothstep(0.25, 0.75, m));
	float qm = min(pc.w + kPad - abs(w.x), pn.w + kPad - abs(w.y));
	haze *= smoothstep(0.0, 14.0, qm) * lerp(0.15, 1.0, smoothstep(50.0, 260.0, dist)) * style.x;
	float swirl = 0.5 + 0.5 * sin(ang * 3.0 + dd * 9.0 - t * 2.2);
	float3 inner = colB.rgb * (0.35 + 0.55 * swirl) * (1.0 - 0.6 * dd * dd);
	if (misc.z > 0.5)
	{
		// G309 Ripple: just inside the rim the picture is sampled through rings that travel inward from the rim (strongest at the rim, gone about 20 units in)
		float2 pix = svpos.xy;
		float band = 0.0;
		float wv = 0.0;
		if (style2.z > 0.01)
		{
			band = smoothstep(-0.5, 2.0, -edge) * exp(-max(-edge, 0.0) / 9.0);
			wv = sin(-edge * 0.75 + t * 3.4 + 3.0 * m);
			float2 rdir = float2(cos(ang + 0.6 * wv), sin(ang + 0.6 * wv));
			float amp = 3.2 * clamp(300.0 / max(dist, 40.0), 0.5, 2.0) * style2.z;
			pix += rdir * wv * band * amp;
		}
		uint tw, th;
		viewTex.GetDimensions(tw, th);
		int2 ip = clamp((int2)pix, int2(0, 0), int2((int)tw - 1, (int)th - 1));
		float3 vc = SafeHdr(viewTex.Load(int3(ip, 0)).rgb);
		vc *= 1.0 + 0.10 * wv * band;
		inner = vc;   // no edge tint (removed G227, 13:31Z): the picture is shown as rendered up to the rim
		// G309 Sheen: close up, a soft light sweep slides over the surface
		float closeS = smoothstep(280.0, 70.0, dist);
		float bandS = 0.5 + 0.5 * sin(dot(w / max(pc.w, 1.0), float2(0.8, 1.3)) * 2.2 - t * 0.9);
		inner += lerp(colA.rgb, float3(1.0, 1.0, 1.0), 0.55) * (closeS * bandS * bandS * bandS * 0.22 * style2.w);
	}
	// G310 moments: the opening's flash inside the oval, wall flash and spark ring; the closing's brightening and pop
	float aFx = 0.0;
	float closeK = smoothstep(0.0, 0.75, cT);
	float3 fxCol = lerp(colA.rgb, float3(1.0, 1.0, 1.0), 0.70) * 4.0;
	float qmF = smoothstep(0.0, 14.0, qm);
	if (cT <= 0.0 && oT < 1.0)
	{
		float oi = 1.0 - oT;
		float ringR = 46.0 * (1.0 - oi * oi * oi);
		float ringE = edge - ringR;
		aFx = (exp(-(ringE * ringE) / 7.0) * oi * oi + exp(-max(edge, 0.0) / 10.0) * oi * oi * 0.08) * qmF;
		inner = lerp(inner, fxCol * 0.30, oi * oi * 0.6 * fill);
	}
	else if (cT > 0.0)
	{
		float pq = saturate((cT - 0.75) / 0.25);
		float rr = length(w);
		float popRing = exp(-((rr - 64.0 * pq) * (rr - 64.0 * pq)) / 12.0) * (1.0 - pq) * step(0.75, cT);
		float popCore = exp(-(rr * rr) / (40.0 + 300.0 * pq)) * (1.0 - pq) * (1.0 - pq) * step(0.75, cT);
		aFx = (popRing * 0.8 + popCore * 0.7) * qmF;
	}
	float aRim = 0.55 * rim * style.y * (1.0 + 0.6 * closeK);
	float aMist = 0.55 * haze;
	float3 rimCol = colA.rgb * 2.0;
	float3 mistCol = lerp(colB.rgb * 1.4, colA.rgb * 1.1, 0.30 + 0.45 * m);
	// G309 RimFlow: a narrow white-hot core of the rim whose brightness flows around the oval; RimGlints: a few bright points that travel along it
	float coreW = exp(-((edge + 0.4) * (edge + 0.4)) / 1.6);
	float fl1 = 0.5 + 0.5 * sin(ang * 3.0 - t * 2.8 + 2.2 * m);
	float fl2 = pow(0.5 + 0.5 * sin(ang * 2.0 + t * 1.7), 3.0);
	float flowI = 0.30 + 0.70 * max(fl1 * fl1, fl2);
	float aCore = coreW * flowI * style2.x * 0.70 * (1.0 + 0.8 * closeK);
	float3 coreCol = lerp(colA.rgb, float3(1.0, 1.0, 1.0), 0.60) * 3.2;
	float glint = 0.0;
	float rAvg = 0.5 * (pc.w + pn.w);
	for (int gk = 0; gk < 3; ++gk)
	{
		float a0 = frac(t * (0.07 + 0.03 * gk) + 0.31 * gk) * 6.2831853;
		float da = ang - a0;
		da = atan2(sin(da), cos(da));
		float arc = abs(da) * rAvg;
		float tw2 = 0.55 + 0.45 * sin(t * (11.0 + 3.0 * gk) + gk * 2.3);
		glint += exp(-(arc * arc) / 6.0) * exp(-((edge + 0.2) * (edge + 0.2)) / 5.0) * tw2;
	}
	float aGl = glint * style2.y;
	float3 glintCol = lerp(colA.rgb, float3(1.0, 1.0, 1.0), 0.80) * 6.0;
	float3 premul = inner * fill + rimCol * aRim + mistCol * aMist + coreCol * aCore + glintCol * aGl + fxCol * aFx;
	float a = saturate(fill * (misc.z > 0.5 ? 1.0 : 0.92) + aRim + aMist + 0.5 * aCore + 0.3 * saturate(aGl) + 0.6 * saturate(aFx));
	float3 col = premul / max(a, 1e-3);
	if (misc.w < 0.5) { if (edge > 4.0) discard; } else { if (edge <= 4.0) discard; }
	if (a < 0.003) discard;
	o.col = float4(SafeHdr(col), a);
	o.depth = zq;
	return o;
}

PSOut PSMain(VSOut i)
{
	float2 w = float2(i.uv.x * pc.w, i.uv.y * pn.w);    // position in the oval's plane, units from the centre
	return Shade(w, i.dist, i.pos, false);
}

// ---- doorway path: the camera is close to the plane; a full-screen pass with the ray of every pixel intersected with the plane ----
struct VSDoorOut
{
	float4 pos : SV_Position;
	float3 dir : TEXCOORD0;   // world-space ray direction, not normalised: dot(dir, camFwd) = 1
};

VSDoorOut VSDoor(uint id : SV_VertexID)
{
	float2 q = float2((id & 1) ? 1.0 : -1.0, (id & 2) ? 1.0 : -1.0);
	float tx = lerp(frustum.x, frustum.y, 0.5 * (q.x + 1.0));
	float ty = lerp(frustum.z, frustum.w, 0.5 * (q.y + 1.0));
	VSDoorOut o;
	o.pos = float4(q.x, q.y, 0.5, 1.0);
	o.dir = camFwd.xyz + camRight.xyz * tx + camUp.xyz * ty;
	return o;
}

PSOut PSDoor(VSDoorOut i)
{
	float3 d = i.dir;
	float dn = dot(d, pn.xyz);
	float dcam = dot(camPos.xyz - pc.xyz, pn.xyz);      // signed distance of the camera from the plane (positive in front)
	if (dn >= -1e-5) discard;                            // only rays that head into the plane (from the front) or away from its front (from behind)
	if (dcam >= 0.0)
	{
		float t = -dcam / dn;                            // forward distance to the plane along this ray
		float3 rel = camPos.xyz + d * t - pc.xyz;
		return Shade(float2(dot(rel, pr.xyz), dot(rel, pu.xyz)), t, i.pos, false);
	}
	return Shade(float2(0.0, 0.0), depthP.x * 1.01, i.pos, true);
}

// MSVC limits one string literal to about 16 KB: the shader source continues in a second literal
)HLSL" R"HLSL(// ---- the exit picture: what has gone through the other portal's plane, drawn from the virtual camera into private colour and depth (PortalView.cpp RenderExit) ----
Texture2D<float4> exitCol   : register(t2);
Texture2D<float2> exitDepth : register(t3);

struct VSExitOut { float4 pos : SV_Position; };

VSExitOut VSExit(uint id : SV_VertexID)
{
	float2 q = float2((id & 1) ? 1.0 : -1.0, (id & 2) ? 1.0 : -1.0);
	VSExitOut o;
	o.pos = float4(q.x, q.y, 0.5, 1.0);
	return o;
}

PSOut PSExit(VSExitOut i)
{
	int3 p = int3((int2)i.pos.xy, 0);
	float z = exitDepth.Load(p).x;
	if (depthP.z < 0.5 ? (z >= 0.999999) : (z <= 0.000001)) discard;   // nothing was drawn here
	float n = depthP.x, f = depthP.y;
	float lin = (depthP.z < 0.5) ? n * f / (f - z * (f - n)) : n * f / (n + z * (f - n));
	if (misc.x > 0.5)
	{
		// the entity's own front part redrawn from the real camera: only where the oval's picture has covered it, within the depth bias in front of the plane
		uint tw, th;
		exitCol.GetDimensions(tw, th);
		float2 uv = i.pos.xy / float2(tw, th);
		float tx = lerp(frustum.x, frustum.y, uv.x);
		float ty = lerp(frustum.w, frustum.z, uv.y);
		float3 wp = camPos.xyz + (camFwd.xyz + camRight.xyz * tx + camUp.xyz * ty) * lin;
		float hgt = dot(wp - pc.xyz, pn.xyz);
		if (hgt > depthP.w + 2.0 || hgt < -2.0) discard;
	}
	lin = max(lin - depthP.w, n * 1.001);   // as near as the oval's own depth, so the oval does not hide the first units of what comes out of it
	float zq = (depthP.z < 0.5) ? f * (lin - n) / (lin * (f - n)) : n * (f - lin) / (lin * (f - n));
	PSOut o;
	o.col = float4(SafeHdr(exitCol.Load(p).rgb), 1.0);
	o.depth = zq;
	return o;
}

// ---- G309 the coloured light: a dynamic light at the portal, per pixel from the depth buffer (world position) and the screen-space normal; additive, no shadows ----
Texture2D<float> lightDepth : register(t8);

float4 PSLight(VSExitOut i) : SV_Target
{
	int3 p = int3((int2)i.pos.xy, 0);
	float z = lightDepth.Load(p);
	if (depthP.z < 0.5 ? (z >= 0.999999) : (z <= 0.000001)) discard;
	float n = depthP.x, f = depthP.y;
	float lin = (depthP.z < 0.5) ? n * f / (f - z * (f - n)) : n * f / (n + z * (f - n));
	float2 uv = (i.pos.xy + 0.5) / scr.xy;
	float tx = lerp(frustum.x, frustum.y, uv.x);
	float ty = lerp(frustum.w, frustum.z, uv.y);
	float3 wp = camPos.xyz + (camFwd.xyz + camRight.xyz * tx + camUp.xyz * ty) * lin;
	float3 L = lightP.xyz - wp;
	float d = length(L);
	if (d >= lightP.w) discard;
	float3 N = cross(ddx(wp), ddy(wp));
	float nl = length(N);
	N = (nl > 1e-6) ? N / nl : float3(0.0, 0.0, 1.0);
	if (dot(N, camPos.xyz - wp) < 0.0) N = -N;
	float ndl = saturate(dot(N, L / max(d, 1e-3)));
	float att = saturate(1.0 - d / lightP.w);
	att *= att;
	return float4(SafeHdr(lightC.rgb * att * (0.30 + 0.70 * ndl)), 1.0);
}

// ---- G310 the casting hand: a glow, a ring of runes and the release flash, in screen space around the hand's magic node, drawn after the first-person arms (additive, no depth test) ----
float hash11(float x)
{
	return frac(sin(x * 127.1 + 311.7) * 43758.5453);
}

float4 PSHand(VSExitOut i) : SV_Target
{
	// lightP.xy = the hand in pixels, lightP.z = the ring's radius in pixels, lightC.rgb = the spell's colour, misc.x = time, fx.y = how far the ring has unfolded (0..1), fx.z = release age 0..1 (>= 1: no flash), fx.w = strength
	float2 p = i.pos.xy - lightP.xy;
	float R = lightP.z;
	float d = length(p);
	if (d > R * 6.5) discard;
	float t = misc.x;
	float ramp = fx.y;
	float3 hot = lerp(lightC.rgb, float3(1.0, 1.0, 1.0), 0.55);
	float3 col = float3(0.0, 0.0, 0.0);
	float gl = exp(-(d * d) / (R * R * 0.55)) * (0.55 + 0.15 * sin(t * 9.0));
	col += lightC.rgb * gl * 1.6 * ramp;
	float2 q = p * float2(1.0, 1.28);
	float rr = length(q);
	float ang = atan2(q.y, q.x) + t * 0.9;
	float rad = R * min(ramp, 1.0);
	float wr = max(1.2, R * 0.02);
	float ringO = exp(-((rr - rad) * (rr - rad)) / (2.0 * wr * wr));
	float ringI = exp(-((rr - rad * 0.74) * (rr - rad * 0.74)) / (2.0 * wr * wr)) * 0.6;
	// the runes: 14 cells around the band between the two rings, each a small stroke pattern chosen by a hash
	float cell = (ang / 6.2831853 + 0.5) * 14.0;
	float ci = floor(cell);
	float u = frac(cell) - 0.5;
	float v = (rr - rad * 0.74) / max(rad * 0.26, 1.0) - 0.5;
	float hsh = hash11(ci + 3.0);
	float stroke = 0.0;
	if (hsh < 0.33) stroke = step(abs(u), 0.07) * step(abs(v), 0.30);
	else if (hsh < 0.66) stroke = max(step(abs(u - v * 0.7), 0.07), step(abs(u + v * 0.7), 0.07)) * step(abs(v), 0.28);
	else stroke = step(abs(length(float2(u * 1.4, v)) - 0.2), 0.055) + step(abs(v + 0.28), 0.05) * step(abs(u), 0.18);
	stroke = saturate(stroke) * step(rad * 0.76, rr) * step(rr, rad * 0.98);
	float flick = 0.7 + 0.3 * sin(t * 6.0 + ci * 1.7);
	col += hot * (ringO * 2.6 + ringI * 1.7 + stroke * 2.2 * flick) * ramp;
	float ra = fx.z;
	if (ra < 1.0)
	{
		float e = 1.0 - ra;
		float sh = R * (1.0 + 1.6 * (1.0 - e * e));
		float wf = R * 0.10;
		float wave = exp(-((d - sh) * (d - sh)) / (2.0 * wf * wf)) * e;
		float core = exp(-(d * d) / (R * R * 1.1)) * e * e;
		col += hot * (wave * 1.4 + core * 1.0) * fx.w;
	}
	return float4(col, 1.0);
}

// ---- G310 walking through: the picture (a copy of the frame in t7) is drawn back with a ring wave travelling out from the centre, and a tint of the exit's colour at the edges ----
float4 PSRipple(VSExitOut i) : SV_Target
{
	// lightP.x = age 0..1 (over 0.3 s), lightP.y = strength, lightC.rgb = the exit portal's colour, scr = viewport
	int2 ip = (int2)i.pos.xy;
	float2 uv = (i.pos.xy + 0.5) / scr.xy;
	float2 dp = (uv - 0.5) * scr.xy;
	float2 dirPix = dp / max(length(dp), 1e-3);
	float2 dv = (uv - 0.5) * float2(scr.x / scr.y, 1.0);
	float r = length(dv);
	float a = saturate(lightP.x);
	float fade = (1.0 - a) * (1.0 - a);
	float R = 0.05 + 1.1 * a;
	float band = exp(-((r - R) * (r - R)) / 0.018);
	float wave = sin((r - R) * 38.0);
	float2 off = dirPix * wave * (16.0 * lightP.y * fade * band);
	uint tw, th;
	compAfter.GetDimensions(tw, th);
	int2 mx = int2((int)tw - 1, (int)th - 1);
	float3 c;
	c.r = compAfter.Load(int3(clamp(ip + (int2)round(off * 1.12), int2(0, 0), mx), 0)).r;
	c.g = compAfter.Load(int3(clamp(ip + (int2)round(off), int2(0, 0), mx), 0)).g;
	c.b = compAfter.Load(int3(clamp(ip + (int2)round(off * 0.88), int2(0, 0), mx), 0)).b;
	c *= 1.0 + 0.10 * wave * band * fade * lightP.y;
	float edgeT = smoothstep(0.55, 1.05, r);
	c += lightC.rgb * (edgeT * fade * 0.45 * lightP.y);
	return float4(c, 1.0);
}
)HLSL" R"HLSL(
// ---- 2026-10-22 the rune glyph ring at the exit ("Draw our own"): a rune cast through a portal lands at the exit, where the engine's own glyph decal does not show
// (G104-G109); this original ring is drawn flat on whatever surface is there, per pixel from the depth buffer like PSLight: centre lightP.xyz, radius lightP.w, surface
// normal pn.xyz, colour lightC.rgb, misc.x time, misc.y fade (0..1) ----
float RuneHash(float n)
{
	return frac(sin(n * 91.345 + 47.853) * 43758.5453);
}

float RuneLine(float2 p, float2 a, float2 b, float w)
{
	float2 pa = p - a, ba = b - a;
	float  h = saturate(dot(pa, ba) / max(dot(ba, ba), 1e-5));
	return 1.0 - smoothstep(w * 0.5, w, length(pa - ba * h));
}

// one glyph in a cell (u across 0..1, v up 0..1): a stem and two strokes picked by the cell's hash
float RuneGlyph(float2 uv, float id)
{
	float h1 = RuneHash(id), h2 = RuneHash(id + 17.0), h3 = RuneHash(id + 31.0);
	float g = RuneLine(uv, float2(0.5, 0.12), float2(0.5, 0.88), 0.10);
	float2 a = float2(0.5, lerp(0.25, 0.75, h1));
	g = max(g, RuneLine(uv, a, float2(h2 < 0.5 ? 0.18 : 0.82, a.y + (h3 < 0.5 ? 0.22 : -0.22)), 0.09));
	if (h3 > 0.35)
	{
		float2 b2 = float2(0.5, lerp(0.75, 0.25, h2));
		g = max(g, RuneLine(uv, b2, float2(h1 < 0.5 ? 0.82 : 0.18, b2.y + 0.18), 0.09));
	}
	return g;
}

float4 PSRune(VSExitOut i) : SV_Target
{
	int3 p = int3((int2)i.pos.xy, 0);
	float z = lightDepth.Load(p);
	if (depthP.z < 0.5 ? (z >= 0.999999) : (z <= 0.000001)) discard;
	float n = depthP.x, f = depthP.y;
	float lin = (depthP.z < 0.5) ? n * f / (f - z * (f - n)) : n * f / (n + z * (f - n));
	float2 uvs = (i.pos.xy + 0.5) / scr.xy;
	float tx = lerp(frustum.x, frustum.y, uvs.x);
	float ty = lerp(frustum.w, frustum.z, uvs.y);
	float3 wp = camPos.xyz + (camFwd.xyz + camRight.xyz * tx + camUp.xyz * ty) * lin;
	float3 N = normalize(pn.xyz);
	float3 d = wp - lightP.xyz;
	float  hgt = dot(d, N);
	if (abs(hgt) > 18.0) discard;
	float3 q = d - N * hgt;
	float  R = lightP.w;
	float  rn = length(q) / R;
	if (rn > 1.12) discard;
	float3 t1 = normalize(abs(N.z) < 0.9 ? cross(N, float3(0.0, 0.0, 1.0)) : cross(N, float3(1.0, 0.0, 0.0)));
	float3 t2 = cross(N, t1);
	float2 pl = float2(dot(q, t1), dot(q, t2)) / R;
	float  t = misc.x;
	float  ang = atan2(pl.y, pl.x);
	// two rings and a faint fill
	float outer = exp(-pow((rn - 1.0) / 0.030, 2.0));
	float inner = exp(-pow((rn - 0.76) / 0.020, 2.0));
	float fill = 0.10 * (1.0 - smoothstep(0.70, 1.0, rn));
	// a band of glyphs between the rings, turning slowly
	const float cells = 14.0;
	float seg = (ang / 6.2831853 + 0.5) * cells + t * 0.12;
	float id = floor(seg);
	float bandV = (rn - 0.79) / 0.18;
	float glyph = (bandV > 0.0 && bandV < 1.0) ? RuneGlyph(float2(frac(seg), bandV), fmod(id + cells * 4.0, cells)) : 0.0;
	// a hexagram inside the inner ring, turning the other way
	float star = 0.0;
	float rot = -t * 0.08;
	[unroll] for (int k = 0; k < 6; ++k)
	{
		float th = rot + 0.5235988 + k * 1.0471976;
		float2 nk = float2(cos(th), sin(th));
		star = max(star, 1.0 - smoothstep(0.010, 0.022, abs(dot(pl, nk) - 0.37)));
	}
	star *= 1.0 - smoothstep(0.70, 0.74, rn);
	float pulse = 0.78 + 0.22 * sin(t * 2.6);
	float flicker = 0.93 + 0.07 * sin(t * 17.0 + ang * 3.0);
	float a = (1.0 * outer + 0.75 * inner + 0.85 * glyph + 0.55 * star + fill) * pulse * flicker;
	a *= saturate(misc.y) * (1.0 - smoothstep(8.0, 18.0, abs(hgt)));
	return float4(SafeHdr(lightC.rgb * a), 1.0);
}
)HLSL";

		struct CB
		{
			float camPos[4], camFwd[4], camUp[4], camRight[4], frustum[4], depthP[4];
			float pc[4], pn[4], pu[4], pr[4], colA[4], colB[4], misc[4], style[4], style2[4], lightP[4], lightC[4], scr[4], fx[4];
		};

		ID3D11Device* Device() { return reinterpret_cast<ID3D11Device*>(RE::BSGraphics::Renderer::GetDevice()); }

		// G310: how far a portal's opening animation has got (0..1 over kOpenSec; 1 when OpenAnim is off)
		float OpenFrac(const portals::Portal& a_p, double a_now)
		{
			return portals::OpenFraction(a_p.colour, a_now);
		}

		// the rune rings at the exit landings (game thread adds and keeps them alive, render thread draws)
		struct RuneRing
		{
			RE::NiPoint3 pos, n;
			int          element{ 0 };
			float        radius{ 80.0f };
			double       born{ 0 }, seen{ 0 }, endAt{ -1 };
		};
		std::mutex                         g_runeMutex;
		std::map<std::uint32_t, RuneRing>  g_runeRings;

		bool Init(ID3D11Device* a_dev)
		{
			auto compile = [](const char* a_entry, const char* a_target, ComPtr<ID3DBlob>& a_out) {
				ComPtr<ID3DBlob> err;
				const HRESULT    hr = D3DCompile(kHlsl, std::strlen(kHlsl), "portal_overlay", nullptr, nullptr, a_entry, a_target, D3DCOMPILE_OPTIMIZATION_LEVEL2, 0, &a_out, &err);
				if (FAILED(hr)) {
					logger::error("overlay: shader compile {} failed: {}", a_entry, err ? static_cast<const char*>(err->GetBufferPointer()) : "?");
					return false;
				}
				return true;
			};
			ComPtr<ID3DBlob> vsb, psb, vsd, psd, vse, pse, psl, psh, psr, psu;
			if (!compile("VSMain", "vs_5_0", vsb) || !compile("PSMain", "ps_5_0", psb) || !compile("VSDoor", "vs_5_0", vsd) || !compile("PSDoor", "ps_5_0", psd) || !compile("VSExit", "vs_5_0", vse) || !compile("PSExit", "ps_5_0", pse) || !compile("PSLight", "ps_5_0", psl) ||
				!compile("PSHand", "ps_5_0", psh) || !compile("PSRipple", "ps_5_0", psr) || !compile("PSRune", "ps_5_0", psu)) {
				return false;
			}
			if (FAILED(a_dev->CreateVertexShader(vsb->GetBufferPointer(), vsb->GetBufferSize(), nullptr, &g_vs)) ||
				FAILED(a_dev->CreatePixelShader(psb->GetBufferPointer(), psb->GetBufferSize(), nullptr, &g_ps)) ||
				FAILED(a_dev->CreateVertexShader(vsd->GetBufferPointer(), vsd->GetBufferSize(), nullptr, &g_vsDoor)) ||
				FAILED(a_dev->CreatePixelShader(psd->GetBufferPointer(), psd->GetBufferSize(), nullptr, &g_psDoor)) ||
				FAILED(a_dev->CreateVertexShader(vse->GetBufferPointer(), vse->GetBufferSize(), nullptr, &g_vsExit)) ||
				FAILED(a_dev->CreatePixelShader(pse->GetBufferPointer(), pse->GetBufferSize(), nullptr, &g_psExit)) ||
				FAILED(a_dev->CreatePixelShader(psl->GetBufferPointer(), psl->GetBufferSize(), nullptr, &g_psLight)) ||
				FAILED(a_dev->CreatePixelShader(psh->GetBufferPointer(), psh->GetBufferSize(), nullptr, &g_psHand)) ||
				FAILED(a_dev->CreatePixelShader(psr->GetBufferPointer(), psr->GetBufferSize(), nullptr, &g_psRipple)) ||
				FAILED(a_dev->CreatePixelShader(psu->GetBufferPointer(), psu->GetBufferSize(), nullptr, &g_psRune))) {
				logger::error("overlay: CreateShader failed");
				return false;
			}
			D3D11_BUFFER_DESC bd{};
			bd.ByteWidth = sizeof(CB);
			bd.Usage = D3D11_USAGE_DYNAMIC;
			bd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
			bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
			if (FAILED(a_dev->CreateBuffer(&bd, nullptr, &g_cb))) {
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
				return false;
			}
			bl.RenderTarget[0].DestBlend = D3D11_BLEND_ONE;  // G306: the virtual VL is light added to what the engine's composite left
			if (FAILED(a_dev->CreateBlendState(&bl, &g_blendAdd))) {
				return false;
			}
			D3D11_RASTERIZER_DESC rs{};
			rs.FillMode = D3D11_FILL_SOLID;
			rs.CullMode = D3D11_CULL_NONE;
			rs.DepthClipEnable = FALSE;
			if (FAILED(a_dev->CreateRasterizerState(&rs, &g_rs))) {
				return false;
			}
			for (int mode = 0; mode < 3; ++mode) {
				for (int wr = 0; wr < 2; ++wr) {
					D3D11_DEPTH_STENCIL_DESC ds{};
					ds.DepthEnable = mode < 2 ? TRUE : FALSE;
					ds.DepthWriteMask = (mode < 2 && wr) ? D3D11_DEPTH_WRITE_MASK_ALL : D3D11_DEPTH_WRITE_MASK_ZERO;
					ds.DepthFunc = mode == 1 ? D3D11_COMPARISON_GREATER_EQUAL : D3D11_COMPARISON_LESS_EQUAL;
					ds.StencilEnable = FALSE;
					if (FAILED(a_dev->CreateDepthStencilState(&ds, &g_dss[mode][wr]))) {
						return false;
					}
				}
			}
			logger::info("overlay: shaders and states created");
			return true;
		}

		float LinearizeCpu(float a_z, int a_mode, float a_n, float a_f)
		{
			if (a_mode == 0) return a_n * a_f / (a_f - a_z * (a_f - a_n));
			return a_n * a_f / (a_n + a_z * (a_f - a_n));
		}

		// Reads the depth buffer at 5 pixels and compares with raycasts through the same pixels.
		void Probe(ID3D11Device* a_dev, ID3D11DeviceContext* a_ctx, ID3D11ShaderResourceView* a_depthSRV)
		{
			ComPtr<ID3D11Resource> res;
			a_depthSRV->GetResource(&res);
			ComPtr<ID3D11Texture2D> tex;
			if (FAILED(res.As(&tex))) {
				logger::warn("overlay: probe: depth SRV is not a 2D texture");
				return;
			}
			D3D11_TEXTURE2D_DESC d{};
			tex->GetDesc(&d);
			D3D11_SHADER_RESOURCE_VIEW_DESC sv{};
			a_depthSRV->GetDesc(&sv);
			logger::info("overlay: probe: depth texture {}x{} format {} samples {} bind {:#x}; SRV format {} dim {}", d.Width, d.Height, static_cast<int>(d.Format), d.SampleDesc.Count, d.BindFlags, static_cast<int>(sv.Format), static_cast<int>(sv.ViewDimension));
			D3D11_TEXTURE2D_DESC sd = d;
			sd.Usage = D3D11_USAGE_STAGING;
			sd.BindFlags = 0;
			sd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
			sd.MiscFlags = 0;
			sd.ArraySize = 1;
			sd.MipLevels = 1;
			ComPtr<ID3D11Texture2D> staging;
			if (d.SampleDesc.Count != 1 || FAILED(a_dev->CreateTexture2D(&sd, nullptr, &staging))) {
				logger::warn("overlay: probe: cannot create a staging copy of the depth texture");
				return;
			}
			a_ctx->CopySubresourceRegion(staging.Get(), 0, 0, 0, 0, tex.Get(), 0, nullptr);
			D3D11_MAPPED_SUBRESOURCE m{};
			if (FAILED(a_ctx->Map(staging.Get(), 0, D3D11_MAP_READ, 0, &m))) {
				logger::warn("overlay: probe: map failed");
				return;
			}
			world::CamInfo cam;
			if (!world::CameraFull(cam)) {
				a_ctx->Unmap(staging.Get(), 0);
				return;
			}
			std::vector<double> err0, err1;
			const float fx[5] = { 0.5f, 0.25f, 0.75f, 0.5f, 0.5f };
			const float fy[5] = { 0.5f, 0.5f, 0.5f, 0.25f, 0.75f };
			for (int k = 0; k < 5; ++k) {
				const UINT px = static_cast<UINT>(fx[k] * d.Width), py = static_cast<UINT>(fy[k] * d.Height);
				const auto* row = static_cast<const std::uint8_t*>(m.pData) + static_cast<std::size_t>(py) * m.RowPitch;
				float z = 0;
				if (d.Format == DXGI_FORMAT_R32G8X24_TYPELESS || d.Format == DXGI_FORMAT_D32_FLOAT_S8X24_UINT) {
					std::memcpy(&z, row + px * 8, 4);
				} else if (d.Format == DXGI_FORMAT_R32_TYPELESS || d.Format == DXGI_FORMAT_D32_FLOAT || d.Format == DXGI_FORMAT_R32_FLOAT) {
					std::memcpy(&z, row + px * 4, 4);
				} else {
					std::uint32_t v;
					std::memcpy(&v, row + px * 4, 4);
					z = (v & 0xFFFFFF) / 16777215.0f;  // D24S8
				}
				// ray through this pixel: view-space direction from the frustum tangents
				const float tx = cam.tanLeft + (cam.tanRight - cam.tanLeft) * fx[k];
				const float ty = cam.tanTop + (cam.tanBottom - cam.tanTop) * fy[k];  // pixel y grows downwards
				RE::NiPoint3 dir = cam.fwd + cam.right * tx + cam.up * ty;
				const float  dl = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
				dir = dir * (1.0f / dl);
				const auto h = world::Raycast(cam.pos, dir, 20000.0f, RE::COL_LAYER::kLOS);
				float fRay = -1;
				if (h.hit) {
					const RE::NiPoint3 dv = h.pos - cam.pos;
					fRay = dv.x * cam.fwd.x + dv.y * cam.fwd.y + dv.z * cam.fwd.z;
				}
				const float lStd = LinearizeCpu(z, 0, cam.nearDist, cam.farDist);
				const float lRev = LinearizeCpu(z, 1, cam.nearDist, cam.farDist);
				logger::info("overlay: probe px ({:.2f},{:.2f}) raw depth {:.6f} | linear std {:.1f} rev {:.1f} | raycast forward distance {:.1f} layer {}", fx[k], fy[k], z, lStd, lRev, fRay, h.layer);
				if (fRay > 0 && fRay < 15000.0f) {
					err0.push_back(std::fabs(lStd - fRay) / fRay);
					err1.push_back(std::fabs(lRev - fRay) / fRay);
				}
			}
			a_ctx->Unmap(staging.Get(), 0);
			if (!err0.empty()) {
				auto median = [](std::vector<double> v) {
					std::sort(v.begin(), v.end());
					return v[v.size() / 2];
				};
				const double e0 = median(err0), e1 = median(err1);  // median: a ray can hit collision that is not drawn, which is an outlier
				logger::info("overlay: probe: median relative error standard {:.4f} reversed {:.4f} over {} pixels (near {:.2f} far {:.1f})", e0, e1, err0.size(), cam.nearDist, cam.farDist);
				const int mode = ((std::min)(e0, e1) < 0.08) ? (e0 <= e1 ? 0 : 1) : 2;
				g_depthMode = mode;
				logger::info("overlay: probe: depth mode set to {} ({})", mode, mode == 0 ? "standard" : mode == 1 ? "reversed" : "no match, depth test off");
			}
		}
	}

	void AddRuneRing(std::uint32_t a_id, const RE::NiPoint3& a_pos, const RE::NiPoint3& a_normal, int a_element, float a_radius)
	{
		const double    now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
		std::lock_guard lk(g_runeMutex);
		g_runeRings[a_id] = RuneRing{ a_pos, a_normal, a_element, a_radius, now, now, -1.0 };
		logger::info("overlay: rune ring {:08X} (element {}) at ({:.1f},{:.1f},{:.1f}) normal ({:.2f},{:.2f},{:.2f}) radius {:.0f}", a_id, a_element, a_pos.x, a_pos.y, a_pos.z, a_normal.x, a_normal.y, a_normal.z, a_radius);
	}

	bool LastRuneRing(RE::NiPoint3& a_pos, RE::NiPoint3& a_normal)
	{
		std::lock_guard lk(g_runeMutex);
		const RuneRing* last = nullptr;
		for (const auto& [id, r] : g_runeRings) {
			if (!last || r.born > last->born) {
				last = &r;
			}
		}
		if (!last) {
			return false;
		}
		a_pos = last->pos;
		a_normal = last->n;
		return true;
	}

	void KeepRuneRings(const std::set<std::uint32_t>& a_alive)
	{
		const double    now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
		std::lock_guard lk(g_runeMutex);
		for (auto& [id, r] : g_runeRings) {
			if (r.endAt >= 0.0) {
				continue;
			}
			if (a_alive.contains(id)) {
				r.seen = now;
			} else {
				r.endAt = now;
				logger::info("overlay: rune ring {:08X} fades: its rune is gone", id);
			}
		}
	}

	void Enable(bool a_on)
	{
		g_enabled = a_on;
		logger::info("overlay: {}", a_on ? "on" : "off");
	}

	void SetDepthBias(float a_units)
	{
		g_depthBias = a_units;
		logger::info("overlay: depth bias {:.1f} units", a_units);
	}

	void SetDepthMode(int a_mode)
	{
		g_depthMode = a_mode;
		logger::info("overlay: depth mode {}", a_mode);
	}

	void SetDepthFar(bool a_on)
	{
		g_depthFar = a_on;
		logger::info("overlay: far-depth test {}", a_on ? "ON" : "off");
	}

	void RequestProbe()
	{
		g_probe = true;
	}

	void SetShotTag(const std::string&) {}

	void Draw()
	{
		if (!g_enabled || g_failed) {
			return;
		}
		audio::Tick();  // G310 phase 3: the portal sounds follow the game state every rendered frame
		travel::LogSkin("overlay (after the main world render)");
		const auto pb = portals::Get(0), po = portals::Get(1);
		const bool probe = g_probe.exchange(false);
		if (!pb.valid && !po.valid && !probe && !fx::Active()) {
			return;
		}
		auto* dev = Device();
		auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
		if (!dev || !rd) {
			return;
		}
		auto* rtv = reinterpret_cast<ID3D11RenderTargetView*>(rd->renderTargets[kMainRT].RTV);
		auto* dsData = &RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData().depthStencils[0];
		auto* depthSRV = reinterpret_cast<ID3D11ShaderResourceView*>(dsData->depthSRV);
		if (!rtv || !depthSRV) {
			static bool once = false;
			if (!once) {
				once = true;
				logger::warn("overlay: main RTV {:p} or depth SRV {:p} missing", static_cast<void*>(rtv), static_cast<void*>(depthSRV));
			}
			return;
		}
		ComPtr<ID3D11DeviceContext> ctx;
		dev->GetImmediateContext(&ctx);
		if (!g_ready) {
			g_ready = Init(dev);
			if (!g_ready) {
				g_failed = true;
				return;
			}
		}
		if (probe) {
			Probe(dev, ctx.Get(), depthSRV);
		}
		world::CamInfo cam;
		if (!world::CameraFull(cam)) {
			return;
		}
		ComPtr<ID3D11Resource> rtRes;
		rtv->GetResource(&rtRes);
		ComPtr<ID3D11Texture2D> rtTex;
		if (FAILED(rtRes.As(&rtTex))) {
			return;
		}
		D3D11_TEXTURE2D_DESC rd2{};
		rtTex->GetDesc(&rd2);

		// ---- save state ----
		ID3D11RenderTargetView*  oldRTV[D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT] = {};
		ID3D11DepthStencilView*  oldDSV = nullptr;
		ctx->OMGetRenderTargets(D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, oldRTV, &oldDSV);
		UINT           nvp = D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE;
		D3D11_VIEWPORT oldVP[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE];
		ctx->RSGetViewports(&nvp, oldVP);
		ID3D11RasterizerState* oldRS = nullptr;
		ctx->RSGetState(&oldRS);
		ID3D11BlendState* oldBS = nullptr;
		FLOAT             oldBF[4];
		UINT              oldSM = 0;
		ctx->OMGetBlendState(&oldBS, oldBF, &oldSM);
		ID3D11DepthStencilState* oldDS = nullptr;
		UINT                     oldRef = 0;
		ctx->OMGetDepthStencilState(&oldDS, &oldRef);
		ID3D11VertexShader* oldVS = nullptr;
		ID3D11PixelShader*  oldPS = nullptr;
		ID3D11GeometryShader* oldGS = nullptr;
		ctx->VSGetShader(&oldVS, nullptr, nullptr);
		ctx->PSGetShader(&oldPS, nullptr, nullptr);
		ctx->GSGetShader(&oldGS, nullptr, nullptr);
		ID3D11InputLayout*       oldIL = nullptr;
		D3D11_PRIMITIVE_TOPOLOGY oldTopo{};
		ctx->IAGetInputLayout(&oldIL);
		ctx->IAGetPrimitiveTopology(&oldTopo);
		ID3D11Buffer* oldVCB = nullptr;
		ID3D11Buffer* oldPCB = nullptr;
		ctx->VSGetConstantBuffers(0, 1, &oldVCB);
		ctx->PSGetConstantBuffers(0, 1, &oldPCB);
		ID3D11ShaderResourceView* oldSRV[2] = {};
		ctx->PSGetShaderResources(0, 2, oldSRV);
		ID3D11ShaderResourceView* oldSRV23[2] = {};
		ctx->PSGetShaderResources(2, 2, oldSRV23);

		// ---- draw ----
		auto* dsvMain = reinterpret_cast<ID3D11DepthStencilView*>(dsData->views[0]);
		const int depthMode = (std::clamp)(g_depthMode.load(), 0, 2);
		ctx->OMSetRenderTargets(1, &rtv, depthMode < 2 ? dsvMain : nullptr);
		D3D11_VIEWPORT vp{ 0.0f, 0.0f, static_cast<float>(rd2.Width), static_cast<float>(rd2.Height), 0.0f, 1.0f };
		ctx->RSSetViewports(1, &vp);
		ctx->RSSetState(g_rs.Get());
		const FLOAT bf[4] = { 0, 0, 0, 0 };
		ctx->OMSetBlendState(g_blend.Get(), bf, 0xFFFFFFFF);

		ctx->IASetInputLayout(nullptr);
		ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
		ctx->VSSetShader(g_vs.Get(), nullptr, 0);
		ctx->GSSetShader(nullptr, nullptr, 0);
		ctx->PSSetShader(g_ps.Get(), nullptr, 0);
		ID3D11Buffer* cb = g_cb.Get();
		ctx->VSSetConstantBuffers(0, 1, &cb);
		ctx->PSSetConstantBuffers(0, 1, &cb);

		// ---- G310 the moments: how far each portal's 0.25 s opening has got, and the portals a recast has just replaced (they keep drawing, collapsing, for 0.25 s) ----
		struct Entry
		{
			const portals::Portal* p;
			float                  open;   // 0..1
			float                  close;  // 0 = a live portal; > 0 = a replaced one, how far its closing has got
		};
		Entry                entries[4];
		int                  nEntries = 0;
		const double         nowE = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
		const portals::Ghost ghosts[2] = { portals::GetGhost(0), portals::GetGhost(1) };
		for (const auto* p : { &pb, &po }) {
			if (p->valid) {
				entries[nEntries++] = Entry{ p, OpenFrac(*p, nowE), 0.0f };
				static int s_openLog = 0;
				if (entries[nEntries - 1].open < 1.0f && s_openLog++ < 60) {
					logger::info("overlay: frame {} portal {} opening {:.3f} (placed {:.3f} s ago)", frame::Count(), p->colour, entries[nEntries - 1].open, nowE - p->placedAtSec);
				}
			}
		}
		for (const auto& g : ghosts) {
			if (g.p.valid && nEntries < 4) {
				entries[nEntries++] = Entry{ &g.p, 1.0f, (std::max)(g.progress, 0.002f) };
			}
		}
		// ---- G309 the coloured light: one additive full-screen pass per portal (PSLight), world position from the depth buffer, normal from the screen-space derivatives, before the ovals are drawn ----
		{
			const auto stL = spells::GetStyle();
			if (stL.light && stL.lightIntensity > 0.001f && g_psLight && depthSRV && nEntries > 0) {
				const double              nowL = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
				ID3D11ShaderResourceView* oldD8 = nullptr;
				ctx->PSGetShaderResources(8, 1, &oldD8);
				ctx->OMSetRenderTargets(1, &rtv, nullptr);  // the depth is read as a texture, so it must not be bound as the depth-stencil at the same time
				ctx->PSSetShaderResources(8, 1, &depthSRV);
				ctx->OMSetBlendState(g_blendAdd.Get(), bf, 0xFFFFFFFF);
				ctx->OMSetDepthStencilState(g_dss[2][0].Get(), 0);
				ctx->VSSetShader(g_vsExit.Get(), nullptr, 0);
				ctx->PSSetShader(g_psLight.Get(), nullptr, 0);
				for (int ei = 0; ei < nEntries; ++ei) {
					const auto* p = entries[ei].p;
					const float entryOpen = entries[ei].open, entryClose = entries[ei].close;
					if (!p->valid) {
						continue;
					}
					CB   c{};
					auto put = [](float* a_dst, const RE::NiPoint3& a_v, float a_w) { a_dst[0] = a_v.x; a_dst[1] = a_v.y; a_dst[2] = a_v.z; a_dst[3] = a_w; };
					put(c.camPos, cam.pos, 0);
					put(c.camFwd, cam.fwd, 0);
					put(c.camUp, cam.up, 0);
					put(c.camRight, cam.right, 0);
					c.frustum[0] = cam.tanLeft; c.frustum[1] = cam.tanRight; c.frustum[2] = cam.tanBottom; c.frustum[3] = cam.tanTop;
					c.depthP[0] = cam.nearDist; c.depthP[1] = cam.farDist; c.depthP[2] = static_cast<float>(g_depthMode.load()); c.depthP[3] = g_depthBias.load();
					const float tealC[3] = { 0.05f, 0.70f, 0.56f }, violetC[3] = { 0.33f, 0.15f, 1.00f };
					const float* lc = p->colour == 0 ? tealC : violetC;
					const float  openL = entryOpen;
					const float  pulseL = 0.92f + 0.08f * std::sin(static_cast<float>(std::fmod(nowL, 1000.0)) * 1.3f + static_cast<float>(p->colour) * 2.0f);
					float        rampL = 0.4f + 0.6f * openL;                                               // G310: the light punches at the cast and settles over the 0.25 s opening
					float        flashL = 1.0f + 1.0f * (1.0f - openL) * (1.0f - openL);
					if (entryClose > 0.0f) {                                                                 // a collapsing portal: its light dies away, with a bright pop at the end
						const float pq = std::clamp((entryClose - 0.75f) / 0.25f, 0.0f, 1.0f);
						rampL = (1.0f - entryClose) * (1.0f - entryClose) + (entryClose >= 0.75f ? 1.2f * (1.0f - pq) * (1.0f - pq) : 0.0f);
						flashL = 1.0f;
					}
					const float  kL = 0.5f * stL.lightIntensity * rampL * flashL * pulseL;
					put(c.lightP, p->c + p->n * 26.0f, stL.lightRadius);
					c.lightC[0] = lc[0] * kL; c.lightC[1] = lc[1] * kL; c.lightC[2] = lc[2] * kL; c.lightC[3] = 1.0f;
					c.scr[0] = static_cast<float>(rd2.Width); c.scr[1] = static_cast<float>(rd2.Height);
					D3D11_MAPPED_SUBRESOURCE m{};
					if (SUCCEEDED(ctx->Map(g_cb.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &m))) {
						std::memcpy(m.pData, &c, sizeof(c));
						ctx->Unmap(g_cb.Get(), 0);
						ctx->Draw(4, 0);
					}
				}
				ID3D11ShaderResourceView* nullD8 = nullptr;
				ctx->PSSetShaderResources(8, 1, &nullD8);
				ctx->OMSetRenderTargets(1, &rtv, depthMode < 2 ? dsvMain : nullptr);
				if (oldD8) {
					oldD8->Release();
				}
				ctx->OMSetBlendState(g_blend.Get(), bf, 0xFFFFFFFF);
				ctx->VSSetShader(g_vs.Get(), nullptr, 0);
				ctx->PSSetShader(g_ps.Get(), nullptr, 0);
			}
		}
		// ---- the rune rings (PSRune): additive, flat on the surface at each exit landing, from the depth buffer like the light ----
		{
			const double          nowR = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
			std::vector<RuneRing> rings;
			{
				std::lock_guard lk(g_runeMutex);
				for (auto it = g_runeRings.begin(); it != g_runeRings.end();) {
					auto& r = it->second;
					if (r.endAt < 0.0 && nowR - r.seen > 2.0) {
						r.endAt = nowR;  // no longer reported alive (portals gone or the rune removed)
					}
					if (r.endAt >= 0.0 && nowR - r.endAt > 0.5) {
						it = g_runeRings.erase(it);
						continue;
					}
					rings.push_back(r);
					++it;
				}
			}
			if (!rings.empty() && g_psRune && depthSRV) {
				ID3D11ShaderResourceView* oldD8 = nullptr;
				ctx->PSGetShaderResources(8, 1, &oldD8);
				ctx->OMSetRenderTargets(1, &rtv, nullptr);
				ctx->PSSetShaderResources(8, 1, &depthSRV);
				ctx->OMSetBlendState(g_blendAdd.Get(), bf, 0xFFFFFFFF);
				ctx->OMSetDepthStencilState(g_dss[2][0].Get(), 0);
				ctx->VSSetShader(g_vsExit.Get(), nullptr, 0);
				ctx->PSSetShader(g_psRune.Get(), nullptr, 0);
				static const float kRuneCol[3][3] = { { 1.00f, 0.42f, 0.08f }, { 0.55f, 0.80f, 1.00f }, { 0.82f, 0.72f, 1.00f } };  // fire orange, frost pale blue, lightning violet-white
				for (const auto& r : rings) {
					CB   c{};
					auto put = [](float* a_dst, const RE::NiPoint3& a_v, float a_w) { a_dst[0] = a_v.x; a_dst[1] = a_v.y; a_dst[2] = a_v.z; a_dst[3] = a_w; };
					put(c.camPos, cam.pos, 0);
					put(c.camFwd, cam.fwd, 0);
					put(c.camUp, cam.up, 0);
					put(c.camRight, cam.right, 0);
					c.frustum[0] = cam.tanLeft; c.frustum[1] = cam.tanRight; c.frustum[2] = cam.tanBottom; c.frustum[3] = cam.tanTop;
					c.depthP[0] = cam.nearDist; c.depthP[1] = cam.farDist; c.depthP[2] = static_cast<float>(g_depthMode.load()); c.depthP[3] = g_depthBias.load();
					put(c.lightP, r.pos, r.radius);
					put(c.pn, r.n, 0);
					const float  fadeIn = static_cast<float>(std::clamp((nowR - r.born) / 0.4, 0.0, 1.0));
					const float  fadeOut = r.endAt >= 0.0 ? static_cast<float>(std::clamp(1.0 - (nowR - r.endAt) / 0.5, 0.0, 1.0)) : 1.0f;
					const float* col = kRuneCol[std::clamp(r.element, 0, 2)];
					const float  k = 2.2f;
					c.lightC[0] = col[0] * k; c.lightC[1] = col[1] * k; c.lightC[2] = col[2] * k; c.lightC[3] = 1.0f;
					c.misc[0] = static_cast<float>(std::fmod(nowR - r.born, 1000.0));
					c.misc[1] = fadeIn * fadeOut;
					c.scr[0] = static_cast<float>(rd2.Width); c.scr[1] = static_cast<float>(rd2.Height);
					D3D11_MAPPED_SUBRESOURCE m{};
					if (SUCCEEDED(ctx->Map(g_cb.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &m))) {
						std::memcpy(m.pData, &c, sizeof(c));
						ctx->Unmap(g_cb.Get(), 0);
						ctx->Draw(4, 0);
					}
				}
				static int s_runeDrawLog = 0;
				if (s_runeDrawLog++ % 600 == 0) {
					logger::info("overlay: {} rune ring(s) drawn at the exit landings", rings.size());
				}
				ID3D11ShaderResourceView* nullD8 = nullptr;
				ctx->PSSetShaderResources(8, 1, &nullD8);
				ctx->OMSetRenderTargets(1, &rtv, depthMode < 2 ? dsvMain : nullptr);
				if (oldD8) {
					oldD8->Release();
				}
				ctx->OMSetBlendState(g_blend.Get(), bf, 0xFFFFFFFF);
				ctx->VSSetShader(g_vs.Get(), nullptr, 0);
				ctx->PSSetShader(g_ps.Get(), nullptr, 0);
			}
		}
		const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
		for (int ei = 0; ei < nEntries; ++ei) {
			const auto* p = entries[ei].p;
			const float entryOpen = entries[ei].open, entryClose = entries[ei].close;
			if (!p->valid) {
				continue;
			}
			auto* viewSrv = entryClose > 0.0f ? nullptr : static_cast<ID3D11ShaderResourceView*>(view::ViewSRV(p->colour));  // a collapsing portal has no picture of its own (the live one of its colour shows the new portal's)
			ctx->PSSetShaderResources(1, 1, &viewSrv);
			const int door = entryClose > 0.0f ? 0 : doorway::State(*p, cam);  // 0 quad, 1 per-pixel in front of the plane, 2 per-pixel behind it
			ctx->VSSetShader(door ? g_vsDoor.Get() : g_vs.Get(), nullptr, 0);
			ctx->PSSetShader(door ? g_psDoor.Get() : g_ps.Get(), nullptr, 0);
			static int lastDoor[2] = { -1, -1 };
			if (entryClose <= 0.0f && lastDoor[p->colour & 1] != door) {
				lastDoor[p->colour & 1] = door;
				logger::info("overlay: portal {} drawn by the {} path", p->colour, door == 0 ? "quad" : (door == 1 ? "doorway (camera in front of the plane)" : "doorway (camera behind the plane)"));
			}
			CB c{};
			auto put = [](float* a_dst, const RE::NiPoint3& a_v, float a_w) { a_dst[0] = a_v.x; a_dst[1] = a_v.y; a_dst[2] = a_v.z; a_dst[3] = a_w; };
			put(c.camPos, cam.pos, 0);
			put(c.camFwd, cam.fwd, 0);
			put(c.camUp, cam.up, 0);
			put(c.camRight, cam.right, 0);
			c.frustum[0] = cam.tanLeft; c.frustum[1] = cam.tanRight; c.frustum[2] = cam.tanBottom; c.frustum[3] = cam.tanTop;
			c.depthP[0] = cam.nearDist; c.depthP[1] = cam.farDist; c.depthP[2] = static_cast<float>(g_depthMode.load()); c.depthP[3] = g_depthBias.load();
			put(c.pc, p->c, p->hw);
			put(c.pn, p->n, p->hh);
			put(c.pu, p->u, g_depthFar.load() ? 1.0f : 0.0f);
			put(c.pr, p->r, 0);
			// the portal spells' colours (linear HDR): colA = the rim / glow, colB = the deep shade of the haze and of the oval without a picture
			//   0 teal (#3FD9C4 family, "Rift: Tidal Weave"), 1 violet (#9A6BFF family, "Rift: Umbral Weave")
			const float tealA[4] = { 0.05f, 0.70f, 0.56f, 1 }, tealB[4] = { 0.02f, 0.30f, 0.32f, 1 };
			const float violetA[4] = { 0.33f, 0.15f, 1.00f, 1 }, violetB[4] = { 0.14f, 0.06f, 0.42f, 1 };
			const float* colA = p->colour == 0 ? tealA : violetA;
			const float* colB = p->colour == 0 ? tealB : violetB;
			for (int i = 0; i < 4; ++i) {
				c.colA[i] = colA[i];
				c.colB[i] = colB[i];
			}
			{
				const auto st = spells::GetStyle();
				c.style[0] = st.halo;
				c.style[1] = st.rim;
				c.style2[0] = st.rimFlow;
				c.style2[1] = st.rimGlints;
				c.style2[2] = st.ripple;
				c.style2[3] = st.sheen;
			}
			c.misc[0] = static_cast<float>(std::fmod(now, 1000.0));
			c.misc[1] = entryOpen;
			c.fx[0] = entryClose;
			c.misc[2] = viewSrv ? 1.0f : 0.0f;
			for (int pass = 0; pass < 2; ++pass) {
				if (door == 2 && pass == 1) {
					break;  // behind the plane the whole picture is the body pass
				}
				c.misc[3] = static_cast<float>(pass);
				D3D11_MAPPED_SUBRESOURCE m{};
				if (SUCCEEDED(ctx->Map(g_cb.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &m))) {
					std::memcpy(m.pData, &c, sizeof(c));
					ctx->Unmap(g_cb.Get(), 0);
					// the body writes depth, the halo only tests; behind the plane the picture covers its pixels outright (no depth test, no write)
					ctx->OMSetDepthStencilState(g_dss[door == 2 ? 2 : depthMode][pass == 0 ? 1 : 0].Get(), 0);
					ctx->Draw(4, 0);
				}
			}
		}

		// ---- the exit pictures: the part of what straddles the other portal's plane that has come out in front of this one, over the main view with the depth test ----
		if (depthMode < 2) {
			for (const auto* p : { &pb, &po }) {
				for (int kind = 0; kind < 2; ++kind) {
					void* col = nullptr;
					void* dep = nullptr;
					if (!p->valid || !view::ExitCopy(p->colour, kind, &col, &dep)) {
						continue;
					}
					ID3D11ShaderResourceView* srvs[2] = { static_cast<ID3D11ShaderResourceView*>(col), static_cast<ID3D11ShaderResourceView*>(dep) };
					ctx->PSSetShaderResources(2, 2, srvs);
					ctx->VSSetShader(g_vsExit.Get(), nullptr, 0);
					ctx->PSSetShader(g_psExit.Get(), nullptr, 0);
					CB   c{};
					auto put = [](float* a_dst, const RE::NiPoint3& a_v, float a_w) { a_dst[0] = a_v.x; a_dst[1] = a_v.y; a_dst[2] = a_v.z; a_dst[3] = a_w; };
					put(c.camPos, cam.pos, 0);
					put(c.camFwd, cam.fwd, 0);
					put(c.camUp, cam.up, 0);
					put(c.camRight, cam.right, 0);
					c.frustum[0] = cam.tanLeft; c.frustum[1] = cam.tanRight; c.frustum[2] = cam.tanBottom; c.frustum[3] = cam.tanTop;
					c.depthP[0] = cam.nearDist; c.depthP[1] = cam.farDist; c.depthP[2] = static_cast<float>(g_depthMode.load()); c.depthP[3] = g_depthBias.load();
					put(c.pc, p->c, p->hw);
					put(c.pn, p->n, p->hh);
					c.misc[0] = static_cast<float>(kind);  // 1 = the entity's front part: composite only within the oval's depth band
					D3D11_MAPPED_SUBRESOURCE m{};
					if (SUCCEEDED(ctx->Map(g_cb.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &m))) {
						std::memcpy(m.pData, &c, sizeof(c));
						ctx->Unmap(g_cb.Get(), 0);
						ctx->OMSetDepthStencilState(g_dss[depthMode][1].Get(), 0);
						ctx->Draw(4, 0);
					}
					static int logged = 0;
					if (logged < 4) {
						++logged;
						logger::info("overlay: exit picture (kind {}) composited for portal {}", kind, p->colour);
					}
				}
			}
			ID3D11ShaderResourceView* nul23[2] = {};
			ctx->PSSetShaderResources(2, 2, nul23);
		}

		// ---- the particles: miasma off the rims, the opening burst, the wisp at the casting hand (Fx.cpp; its own shaders and blend state, the overlay's restore below puts everything back) ----
		if (fx::Active()) {
			fx::Frame ff;
			ff.dev = dev;
			ff.ctx = ctx.Get();
			ff.dssTest = g_dss[depthMode][0].Get();
			ff.depthMode = depthMode;
			ff.now = now;
			ff.width = static_cast<int>(rd2.Width);
			fx::Render(ff, cam, pb, po);
		}

		// ---- restore state ----
		ID3D11ShaderResourceView* nullSRVs[2] = {};
		ctx->PSSetShaderResources(0, 2, nullSRVs);
		ctx->PSSetShaderResources(0, 2, oldSRV);
		ctx->PSSetShaderResources(2, 2, oldSRV23);
		ctx->VSSetConstantBuffers(0, 1, &oldVCB);
		ctx->PSSetConstantBuffers(0, 1, &oldPCB);
		ctx->IASetInputLayout(oldIL);
		ctx->IASetPrimitiveTopology(oldTopo);
		ctx->VSSetShader(oldVS, nullptr, 0);
		ctx->PSSetShader(oldPS, nullptr, 0);
		ctx->GSSetShader(oldGS, nullptr, 0);
		ctx->OMSetDepthStencilState(oldDS, oldRef);
		ctx->OMSetBlendState(oldBS, oldBF, oldSM);
		ctx->RSSetState(oldRS);
		ctx->RSSetViewports(nvp, oldVP);
		ctx->OMSetRenderTargets(D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, oldRTV, oldDSV);
		for (auto* r : oldRTV) {
			if (r) r->Release();
		}
		auto rel = [](IUnknown* a_o) { if (a_o) a_o->Release(); };
		rel(oldDSV); rel(oldRS); rel(oldBS); rel(oldDS); rel(oldVS); rel(oldPS); rel(oldGS); rel(oldIL); rel(oldVCB); rel(oldPCB); rel(oldSRV[0]); rel(oldSRV[1]); rel(oldSRV23[0]); rel(oldSRV23[1]);
		// G310: the engine's VL composite (the hook that runs DrawMoments after the first-person hands) does not run in every scene; then the moments are drawn from here, before the hands
		if (frame::Count() > g_momentsFrame.load() + 3) {
			DrawMoments(false);
		}
	}

	// G308: the main HDR target right before and right after the engine's own VL composite (copies); what the engine added at an oval pixel is not added a second time by the virtual VL (view vlsub=1)
	namespace
	{
		ComPtr<ID3D11Texture2D>          g_compTex[2];
		ComPtr<ID3D11ShaderResourceView> g_compSrv[2];
		bool                              g_compHave{ false };
		std::atomic<int>                 g_vlSub{ 1 };
		bool EnsureComp(ID3D11Texture2D* a_main)
		{
			D3D11_TEXTURE2D_DESC d{};
			a_main->GetDesc(&d);
			if (g_compTex[0]) {
				D3D11_TEXTURE2D_DESC c{};
				g_compTex[0]->GetDesc(&c);
				if (c.Width == d.Width && c.Height == d.Height && c.Format == d.Format) {
					return true;
				}
			}
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
				g_compTex[i].Reset();
				g_compSrv[i].Reset();
				if (FAILED(dev->CreateTexture2D(&d, nullptr, &g_compTex[i])) || FAILED(dev->CreateShaderResourceView(g_compTex[i].Get(), nullptr, &g_compSrv[i]))) {
					g_compTex[0].Reset();
					return false;
				}
			}
			return true;
		}
	}

	void SetVlSub(int a_on)
	{
		g_vlSub = a_on;
	}

	void VlAddBegin()
	{
		g_compHave = false;
		if (!g_enabled || g_failed || !g_ready || g_vlSub.load() == 0) {
			return;
		}
		if (!(view::VlAddValid(0) || view::VlAddValid(1))) {
			return;
		}
		auto* dev = Device();
		auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
		auto* mainTex = rd ? reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[kMainRT].texture) : nullptr;
		if (!dev || !mainTex || !EnsureComp(mainTex)) {
			return;
		}
		ComPtr<ID3D11DeviceContext> ctx;
		dev->GetImmediateContext(&ctx);
		ctx->CopyResource(g_compTex[0].Get(), mainTex);
		g_compHave = true;
	}

	// G306: after the engine's real VL composite: the virtual VL of each portal picture (the picture after its VL minus the picture before it, kept apart by PortalView: view vladd=1) is ADDED onto the oval's pixels,
	// the way the real frame gets its VL after the engine's glow step. Same oval geometry as Draw (body pass only), depth test without writes so anything in front of the oval (the hands) is left alone.
	// G308: minus what the engine's own composite has just added at that pixel (the real VL of the air in front of the window, view vlsub=1), so the two are not both counted.
	void DrawVlAdd()
	{
		if (!g_enabled || g_failed || !g_ready) {
			return;
		}
		const auto pb = portals::Get(0), po = portals::Get(1);
		bool       any = false;
		for (const auto* p : { &pb, &po }) {
			any = any || (p->valid && view::VlAddValid(p->colour));
		}
		if (!any) {
			return;
		}
		auto* dev = Device();
		auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
		if (!dev || !rd) {
			return;
		}
		auto* rtv = reinterpret_cast<ID3D11RenderTargetView*>(rd->renderTargets[kMainRT].RTV);
		auto* mainTexA = reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[kMainRT].texture);
		auto* dsData = &RE::BSGraphics::Renderer::GetSingleton()->GetDepthStencilData().depthStencils[0];
		auto* dsvMain = reinterpret_cast<ID3D11DepthStencilView*>(dsData->views[0]);
		world::CamInfo cam;
		if (!rtv || !mainTexA || !world::CameraFull(cam)) {
			return;
		}
		ComPtr<ID3D11DeviceContext> ctx;
		dev->GetImmediateContext(&ctx);
		const bool sub = g_vlSub.load() != 0 && g_compHave && EnsureComp(mainTexA);
		if (sub) {
			ctx->CopyResource(g_compTex[1].Get(), mainTexA);
		}
		g_compHave = false;
		D3D11_TEXTURE2D_DESC rd2{};
		mainTexA->GetDesc(&rd2);
		// ---- save state ----
		ID3D11RenderTargetView* oldRTV[D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT] = {};
		ID3D11DepthStencilView* oldDSV = nullptr;
		ctx->OMGetRenderTargets(D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, oldRTV, &oldDSV);
		UINT           nvp = D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE;
		D3D11_VIEWPORT oldVP[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE];
		ctx->RSGetViewports(&nvp, oldVP);
		ID3D11RasterizerState* oldRS = nullptr;
		ctx->RSGetState(&oldRS);
		ID3D11BlendState* oldBS = nullptr;
		FLOAT             oldBF[4];
		UINT              oldSM = 0;
		ctx->OMGetBlendState(&oldBS, oldBF, &oldSM);
		ID3D11DepthStencilState* oldDS = nullptr;
		UINT                     oldRef = 0;
		ctx->OMGetDepthStencilState(&oldDS, &oldRef);
		ID3D11VertexShader*   oldVS = nullptr;
		ID3D11PixelShader*    oldPS = nullptr;
		ID3D11GeometryShader* oldGS = nullptr;
		ctx->VSGetShader(&oldVS, nullptr, nullptr);
		ctx->PSGetShader(&oldPS, nullptr, nullptr);
		ctx->GSGetShader(&oldGS, nullptr, nullptr);
		ID3D11InputLayout*       oldIL = nullptr;
		D3D11_PRIMITIVE_TOPOLOGY oldTopo{};
		ctx->IAGetInputLayout(&oldIL);
		ctx->IAGetPrimitiveTopology(&oldTopo);
		ID3D11Buffer* oldVCB = nullptr;
		ID3D11Buffer* oldPCB = nullptr;
		ctx->VSGetConstantBuffers(0, 1, &oldVCB);
		ctx->PSGetConstantBuffers(0, 1, &oldPCB);
		ID3D11ShaderResourceView* oldSRV[4] = {};
		ctx->PSGetShaderResources(4, 4, oldSRV);
		// ---- draw ----
		const int depthMode = (std::clamp)(g_depthMode.load(), 0, 2);
		ctx->OMSetRenderTargets(1, &rtv, depthMode < 2 ? dsvMain : nullptr);
		D3D11_VIEWPORT vp{ 0.0f, 0.0f, static_cast<float>(rd2.Width), static_cast<float>(rd2.Height), 0.0f, 1.0f };
		ctx->RSSetViewports(1, &vp);
		ctx->RSSetState(g_rs.Get());
		const FLOAT bf[4] = { 0, 0, 0, 0 };
		ctx->OMSetBlendState(g_blendAdd.Get(), bf, 0xFFFFFFFF);
		ctx->IASetInputLayout(nullptr);
		ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
		ctx->GSSetShader(nullptr, nullptr, 0);
		ID3D11Buffer* cb = g_cb.Get();
		ctx->VSSetConstantBuffers(0, 1, &cb);
		ctx->PSSetConstantBuffers(0, 1, &cb);
		for (const auto* p : { &pb, &po }) {
			if (!p->valid || !view::VlAddValid(p->colour)) {
				continue;
			}
			ID3D11ShaderResourceView* srvs[4] = { static_cast<ID3D11ShaderResourceView*>(view::VlPreSRV(p->colour)), static_cast<ID3D11ShaderResourceView*>(view::VlPostSRV(p->colour)), sub ? g_compSrv[0].Get() : nullptr, sub ? g_compSrv[1].Get() : nullptr };
			ctx->PSSetShaderResources(4, 4, srvs);
			const int door = doorway::State(*p, cam);
			ctx->VSSetShader(door ? g_vsDoor.Get() : g_vs.Get(), nullptr, 0);
			ctx->PSSetShader(door ? g_psDoor.Get() : g_ps.Get(), nullptr, 0);
			CB   c{};
			auto put = [](float* a_dst, const RE::NiPoint3& a_v, float a_w) { a_dst[0] = a_v.x; a_dst[1] = a_v.y; a_dst[2] = a_v.z; a_dst[3] = a_w; };
			put(c.camPos, cam.pos, 0);
			put(c.camFwd, cam.fwd, 0);
			put(c.camUp, cam.up, 0);
			put(c.camRight, cam.right, 0);
			c.frustum[0] = cam.tanLeft; c.frustum[1] = cam.tanRight; c.frustum[2] = cam.tanBottom; c.frustum[3] = cam.tanTop;
			c.depthP[0] = cam.nearDist; c.depthP[1] = cam.farDist; c.depthP[2] = static_cast<float>(g_depthMode.load()); c.depthP[3] = g_depthBias.load();
			put(c.pc, p->c, p->hw);
			put(c.pn, p->n, p->hh);
			put(c.pu, p->u, g_depthFar.load() ? 1.0f : 0.0f);
			put(c.pr, p->r, 0);
			const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
			c.misc[0] = static_cast<float>(std::fmod(now, 1000.0));
			c.misc[1] = OpenFrac(*p, now);  // the opening animation of Draw: the add follows the same oval
			c.misc[2] = 1.0f;
			c.misc[3] = 0.0f;
			c.style[2] = 1.0f;  // the VL add mode of Shade
			c.style[3] = sub ? 1.0f : 0.0f;  // minus what the engine's composite added
			D3D11_MAPPED_SUBRESOURCE m{};
			if (SUCCEEDED(ctx->Map(g_cb.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &m))) {
				std::memcpy(m.pData, &c, sizeof(c));
				ctx->Unmap(g_cb.Get(), 0);
				ctx->OMSetDepthStencilState(g_dss[door == 2 ? 2 : depthMode][0].Get(), 0);
				ctx->Draw(4, 0);
			}
		}
		static int s_logged = 0;
		if (s_logged++ < 3) {
			logger::info("overlay: virtual VL added after the engine's VL composite (portal 0 {}, portal 1 {}, minus the engine's own add {})", view::VlAddValid(0), view::VlAddValid(1), sub);
		}
		// ---- restore state ----
		ID3D11ShaderResourceView* nullSRVs[4] = {};
		ctx->PSSetShaderResources(4, 4, nullSRVs);
		ctx->PSSetShaderResources(4, 4, oldSRV);
		ctx->VSSetConstantBuffers(0, 1, &oldVCB);
		ctx->PSSetConstantBuffers(0, 1, &oldPCB);
		ctx->IASetInputLayout(oldIL);
		ctx->IASetPrimitiveTopology(oldTopo);
		ctx->VSSetShader(oldVS, nullptr, 0);
		ctx->PSSetShader(oldPS, nullptr, 0);
		ctx->GSSetShader(oldGS, nullptr, 0);
		ctx->OMSetDepthStencilState(oldDS, oldRef);
		ctx->OMSetBlendState(oldBS, oldBF, oldSM);
		ctx->RSSetState(oldRS);
		ctx->RSSetViewports(nvp, oldVP);
		ctx->OMSetRenderTargets(D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, oldRTV, oldDSV);
		for (auto* r : oldRTV) {
			if (r) r->Release();
		}
		auto rel = [](IUnknown* a_o) { if (a_o) a_o->Release(); };
		rel(oldDSV); rel(oldRS); rel(oldBS); rel(oldDS); rel(oldVS); rel(oldPS); rel(oldGS); rel(oldIL); rel(oldVCB); rel(oldPCB); rel(oldSRV[0]); rel(oldSRV[1]); rel(oldSRV[2]); rel(oldSRV[3]);
	}

	// ---- G310 the moments that need the first-person hands in the frame: the glow and rune ring at a casting hand, the release flash when a spell lands, the screen ripple and edge tint when the player walks through a portal ----
	namespace
	{
		struct HandMo
		{
			bool         up{ false };   // the hand is holding / casting one of our spells
			double       upAt{ 0 }, lastUp{ 0 };
			RE::NiPoint3 pos{};
			RE::NiPoint3 camRel{};      // first person: the hand relative to the camera (right, up, forward), rebuilt with the drawing frame's camera
			bool         firstPerson{ false };
			int          colour{ 0 };
			double       flashAt{ 0 };  // the release flash: when it started (0 = none) and where
			RE::NiPoint3 flashPos{};
			int          flashColour{ 0 };
		};
		HandMo g_handMo[2];
		double g_seenPlacedMo[2]{ 0, 0 };

		// saves the pipeline state the engine had bound and puts it back
		struct StateGuard
		{
			ComPtr<ID3D11DeviceContext> ctx;
			ID3D11RenderTargetView*     rtvs[D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT] = {};
			ID3D11DepthStencilView*     dsv = nullptr;
			UINT                        nvp = D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE;
			D3D11_VIEWPORT              vps[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE];
			ID3D11RasterizerState*      rs = nullptr;
			ID3D11BlendState*           bs = nullptr;
			FLOAT                       bf[4]{};
			UINT                        sm = 0;
			ID3D11DepthStencilState*    ds = nullptr;
			UINT                        ref = 0;
			ID3D11VertexShader*         vs = nullptr;
			ID3D11PixelShader*          ps = nullptr;
			ID3D11GeometryShader*       gs = nullptr;
			ID3D11InputLayout*          il = nullptr;
			D3D11_PRIMITIVE_TOPOLOGY    topo{};
			ID3D11Buffer*               vcb = nullptr;
			ID3D11Buffer*               pcb = nullptr;
			ID3D11ShaderResourceView*   srv[4] = {};

			explicit StateGuard(ID3D11DeviceContext* a_ctx) :
				ctx(a_ctx)
			{
				ctx->OMGetRenderTargets(D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, rtvs, &dsv);
				ctx->RSGetViewports(&nvp, vps);
				ctx->RSGetState(&rs);
				ctx->OMGetBlendState(&bs, bf, &sm);
				ctx->OMGetDepthStencilState(&ds, &ref);
				ctx->VSGetShader(&vs, nullptr, nullptr);
				ctx->PSGetShader(&ps, nullptr, nullptr);
				ctx->GSGetShader(&gs, nullptr, nullptr);
				ctx->IAGetInputLayout(&il);
				ctx->IAGetPrimitiveTopology(&topo);
				ctx->VSGetConstantBuffers(0, 1, &vcb);
				ctx->PSGetConstantBuffers(0, 1, &pcb);
				ctx->PSGetShaderResources(4, 4, srv);
			}
			~StateGuard()
			{
				ID3D11ShaderResourceView* nullSRVs[4] = {};
				ctx->PSSetShaderResources(4, 4, nullSRVs);
				ctx->PSSetShaderResources(4, 4, srv);
				ctx->VSSetConstantBuffers(0, 1, &vcb);
				ctx->PSSetConstantBuffers(0, 1, &pcb);
				ctx->IASetInputLayout(il);
				ctx->IASetPrimitiveTopology(topo);
				ctx->VSSetShader(vs, nullptr, 0);
				ctx->PSSetShader(ps, nullptr, 0);
				ctx->GSSetShader(gs, nullptr, 0);
				ctx->OMSetDepthStencilState(ds, ref);
				ctx->OMSetBlendState(bs, bf, sm);
				ctx->RSSetState(rs);
				ctx->RSSetViewports(nvp, vps);
				ctx->OMSetRenderTargets(D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT, rtvs, dsv);
				for (auto* r : rtvs) {
					if (r) r->Release();
				}
				auto rel = [](IUnknown* a_o) { if (a_o) a_o->Release(); };
				rel(dsv); rel(rs); rel(bs); rel(ds); rel(vs); rel(ps); rel(gs); rel(il); rel(vcb); rel(pcb); rel(srv[0]); rel(srv[1]); rel(srv[2]); rel(srv[3]);
			}
		};
	}

	void DrawMoments(bool a_fromComposite)
	{
		if (!g_enabled || g_failed || !g_ready) {
			return;
		}
		if (a_fromComposite) {
			g_momentsFrame = frame::Count();
		}
		const auto     st = spells::GetStyle();
		const double   now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
		spells::HandFx hands[2];
		spells::GetHandFx(hands);
		const portals::Portal ps[2] = { portals::Get(0), portals::Get(1) };
		const auto            cross = portals::LastCross();
		// a portal was just placed: the release flash at the hand that cast it
		for (int ci = 0; ci < 2; ++ci) {
			if (!ps[ci].valid) {
				g_seenPlacedMo[ci] = 0.0;
				continue;
			}
			if (ps[ci].placedAtSec != g_seenPlacedMo[ci]) {
				g_seenPlacedMo[ci] = ps[ci].placedAtSec;
				for (int h = 0; h < 2; ++h) {
					if (hands[h].colour == ci && now - hands[h].at < 1.0) {
						g_handMo[h].flashAt = now;
						g_handMo[h].flashPos = hands[h].pos;
						g_handMo[h].flashColour = ci;
					}
				}
			}
		}
		float ramp[2] = { 0.0f, 0.0f }, flashAge[2] = { 1.0f, 1.0f };
		bool  anyHand = false;
		for (int h = 0; h < 2; ++h) {
			auto&      m = g_handMo[h];
			const bool fresh = hands[h].active && hands[h].colour >= 0 && now - hands[h].at < 0.25;
			if (fresh) {
				if (!m.up) {
					m.up = true;
					m.upAt = now;
				}
				m.lastUp = now;
				m.pos = hands[h].pos;
				m.camRel = hands[h].camRel;
				m.firstPerson = hands[h].firstPerson;
				m.colour = hands[h].colour;
			} else {
				m.up = false;
			}
			if (st.handRunes > 0.001f) {
				ramp[h] = m.up ? static_cast<float>(std::clamp((now - m.upAt) / 0.18, 0.0, 1.0)) : static_cast<float>(std::clamp(1.0 - (now - m.lastUp) / 0.15, 0.0, 1.0)) * (m.lastUp > 0.0 ? 1.0f : 0.0f);
			}
			if (st.releaseFlash > 0.001f && m.flashAt > 0.0) {
				flashAge[h] = static_cast<float>((now - m.flashAt) / 0.25);
			}
			if (ramp[h] > 0.001f || flashAge[h] < 1.0f) {
				anyHand = true;
			}
		}
		const double rippleAge = (st.walkRipple > 0.001f && cross.at > 0.0) ? (now - cross.at) / 0.3 : 2.0;
		const bool   ripple = rippleAge >= 0.0 && rippleAge < 1.0;
		if (!ripple && !anyHand) {
			return;
		}
		auto* dev = Device();
		auto* rd = RE::BSGraphics::Renderer::GetRendererDataSingleton();
		if (!dev || !rd) {
			return;
		}
		auto* rtv = reinterpret_cast<ID3D11RenderTargetView*>(rd->renderTargets[kMainRT].RTV);
		auto* mainTexA = reinterpret_cast<ID3D11Texture2D*>(rd->renderTargets[kMainRT].texture);
		world::CamInfo cam;
		if (!rtv || !mainTexA || !g_psHand || !g_psRipple || !world::CameraFull(cam)) {
			return;
		}
		ComPtr<ID3D11DeviceContext> ctx;
		dev->GetImmediateContext(&ctx);
		D3D11_TEXTURE2D_DESC rd2{};
		mainTexA->GetDesc(&rd2);
		const float W = static_cast<float>(rd2.Width), H = static_cast<float>(rd2.Height);
		StateGuard  guard(ctx.Get());
		D3D11_VIEWPORT vp{ 0.0f, 0.0f, W, H, 0.0f, 1.0f };
		ctx->RSSetViewports(1, &vp);
		ctx->RSSetState(g_rs.Get());
		ctx->IASetInputLayout(nullptr);
		ctx->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
		ctx->GSSetShader(nullptr, nullptr, 0);
		ID3D11Buffer* cb = g_cb.Get();
		ctx->VSSetConstantBuffers(0, 1, &cb);
		ctx->PSSetConstantBuffers(0, 1, &cb);
		ctx->OMSetRenderTargets(1, &rtv, nullptr);
		ctx->OMSetDepthStencilState(g_dss[2][0].Get(), 0);
		ctx->VSSetShader(g_vsExit.Get(), nullptr, 0);
		const FLOAT bf[4] = { 0, 0, 0, 0 };
		auto        run = [&](ID3D11PixelShader* a_ps, ID3D11BlendState* a_bs, const CB& a_c) {
			D3D11_MAPPED_SUBRESOURCE m{};
			if (FAILED(ctx->Map(g_cb.Get(), 0, D3D11_MAP_WRITE_DISCARD, 0, &m))) {
				return;
			}
			std::memcpy(m.pData, &a_c, sizeof(a_c));
			ctx->Unmap(g_cb.Get(), 0);
			ctx->OMSetBlendState(a_bs, bf, 0xFFFFFFFF);
			ctx->PSSetShader(a_ps, nullptr, 0);
			ctx->Draw(4, 0);
		};
		const float tealC[3] = { 0.05f, 0.70f, 0.56f }, violetC[3] = { 0.33f, 0.15f, 1.00f };
		const float timeNow = static_cast<float>(std::fmod(now, 1000.0));
		// the ripple: the frame so far, drawn back distorted by a ring wave from the centre, with the exit colour tinting the edges
		if (ripple && EnsureComp(mainTexA)) {
			ctx->CopyResource(g_compTex[1].Get(), mainTexA);
			ID3D11ShaderResourceView* src = g_compSrv[1].Get();
			ctx->PSSetShaderResources(7, 1, &src);
			CB          c{};
			const float* ec = (cross.exitColour == 0) ? tealC : violetC;
			c.lightP[0] = static_cast<float>(rippleAge);
			c.lightP[1] = st.walkRipple;
			c.lightC[0] = ec[0]; c.lightC[1] = ec[1]; c.lightC[2] = ec[2]; c.lightC[3] = 1.0f;
			c.scr[0] = W; c.scr[1] = H;
			run(g_psRipple.Get(), g_blend.Get(), c);
			ID3D11ShaderResourceView* none = nullptr;
			ctx->PSSetShaderResources(7, 1, &none);
		}
		// the hands: glow and rune ring around the magic node, the release flash
		for (int h = 0; h < 2 && anyHand; ++h) {
			if (ramp[h] <= 0.001f && flashAge[h] >= 1.0f) {
				continue;
			}
			const auto&        m = g_handMo[h];
			const bool         flashOnly = !(ramp[h] > 0.001f);
			const bool   useFlash = (flashOnly || flashAge[h] < 1.0f) && !m.up;
			RE::NiPoint3 nowPos;
			const bool   haveNow = spells::HandWorldNow(h, nowPos);  // the release flash follows the hand too (it used to stay where the spell left the hand)
			RE::NiPoint3 anchor;
			const bool   haveAnchor = fx::HandAnchor(h, cam, anchor);  // try 3: the hand as the arms are drawn this frame (the same point the wisps use)
			const RE::NiPoint3 wp = haveAnchor ? anchor : haveNow ? nowPos : useFlash ? m.flashPos : m.firstPerson ? cam.pos + cam.right * m.camRel.x + cam.up * m.camRel.y + cam.fwd * m.camRel.z : m.pos;
			static int s_handLog = 0;
			if (!useFlash && s_handLog < 30 && (static_cast<int>(timeNow * 4.0f) % 2) == 0) {
				++s_handLog;
				logger::info("overlay: hand {} ring at ({:.1f},{:.1f},{:.1f}) (node now {}, frame start ({:.1f},{:.1f},{:.1f})), camera ({:.1f},{:.1f},{:.1f}) fwd ({:.2f},{:.2f},{:.2f})", h, wp.x, wp.y, wp.z, haveNow ? "read" : "missing",
					m.pos.x, m.pos.y, m.pos.z, cam.pos.x, cam.pos.y, cam.pos.z, cam.fwd.x, cam.fwd.y, cam.fwd.z);
			}
			const RE::NiPoint3 d = wp - cam.pos;
			const float        f = d.x * cam.fwd.x + d.y * cam.fwd.y + d.z * cam.fwd.z;
			if (f < 2.0f) {
				continue;
			}
			// the first-person arms are drawn with the camera's first-person field of view, not the world's (2026-10-21: the ring drifted off the hand and grew huge;
			// his log showed the ring exactly at the hand's magic node, so the projection was the fault): a point of the arms is projected with tangents scaled by
			// tan(firstPersonFOV / 2) / tan(worldFOV / 2)
			float fovK = 1.0f;
			if (!useFlash && m.firstPerson) {
				if (auto* pcam = RE::PlayerCamera::GetSingleton()) {
					const float wf = pcam->GetRuntimeData2().worldFOV, ff = pcam->GetRuntimeData2().firstPersonFOV;
					if (wf > 1.0f && ff > 1.0f && wf < 179.0f && ff < 179.0f) {
						fovK = std::tan(ff * 0.5f * 0.017453292f) / std::tan(wf * 0.5f * 0.017453292f);
					}
				}
			}
			const float tr = (d.x * cam.right.x + d.y * cam.right.y + d.z * cam.right.z) / f / fovK;
			const float tu = (d.x * cam.up.x + d.y * cam.up.y + d.z * cam.up.z) / f / fovK;
			const float px = (tr - cam.tanLeft) / (cam.tanRight - cam.tanLeft) * W;
			const float py = (cam.tanTop - tu) / (cam.tanTop - cam.tanBottom) * H;
			const float rpx = std::clamp(5.0f / f / fovK / (cam.tanRight - cam.tanLeft) * W, 22.0f, 170.0f);
			static int s_fovLog = 0;
			if (!useFlash && s_fovLog < 6) {
				++s_fovLog;
				const auto* pc2 = RE::PlayerCamera::GetSingleton();
				logger::info("overlay: hand {} ring projected with fov factor {:.3f} (world fov {:.1f}, first-person fov {:.1f}, first person {}): screen ({:.0f},{:.0f}) radius {:.0f}", h, fovK, pc2 ? pc2->GetRuntimeData2().worldFOV : 0.0f,
					pc2 ? pc2->GetRuntimeData2().firstPersonFOV : 0.0f, m.firstPerson, px, py, rpx);
			}
			const int   colour = (flashOnly || flashAge[h] < 1.0f) && !m.up ? m.flashColour : m.colour;
			const float* hc = (colour == 0) ? tealC : violetC;
			CB          c{};
			c.lightP[0] = px; c.lightP[1] = py; c.lightP[2] = rpx; c.lightP[3] = 1.0f;
			c.lightC[0] = hc[0]; c.lightC[1] = hc[1]; c.lightC[2] = hc[2]; c.lightC[3] = 1.0f;
			c.misc[0] = timeNow;
			c.fx[1] = ramp[h] * st.handRunes;
			c.fx[2] = flashAge[h];
			c.fx[3] = st.releaseFlash;
			c.scr[0] = W; c.scr[1] = H;
			run(g_psHand.Get(), g_blendAdd.Get(), c);
		}
		static int s_logged = 0;
		if (s_logged++ < 3) {
			logger::info("overlay: moments drawn (ripple {}, hands {}) from {}", ripple, anyHand, a_fromComposite ? "the engine's VL composite" : "Draw");
		}
	}
}
