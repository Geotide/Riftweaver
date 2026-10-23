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

#pragma once

// Module interfaces of the Riftweaver plugin. Each namespace lives in one .cpp of the same name.

namespace frame  // hook of the engine's per-frame function (not called at the main menu)
{
	bool          Install();
	std::uint64_t Count();  // frames seen since start
	void          SetWatchdog(double a_seconds);  // G236: when the frame counter has not moved for that many seconds (0 = off): capture every thread's stack into the log and terminate the game process
	void          ShadowFence(bool a_on);  // G238: GPU event fences (view vsfence=1 must be on too) after the engine's OWN shadow stage and sun Render, and before its main world draw, to find the first command the GPU never finishes
	void          ShadowTrace(int a_frames);  // G234: log the calls the engine makes to the sun shadow light's virtual functions (Accumulate 9, Render 0xA, ReturnShadowmaps 0xC, UpdateCamera 0x10), to the shadow stage (id 107132) and to the shadow-map cull (id 108492) for the next frames, with return addresses and timing
	// A job run at the start of every frame on the game thread until it returns false (a_dt = seconds since the previous frame).
	void          SetPerFrame(std::function<bool(double)> a_job);
}

namespace shot  // screenshots of the final composited frame, taken at Present
{
	bool Init();
	bool SaveResource(void* a_d3dTexture, const std::string& a_name);  // ID3D11Texture2D* -> <SKSE log dir>\Riftweaver_<name>.png (HDR crudely tonemapped)
	void Request(const std::string& a_name);  // saved as <SKSE log dir>\Riftweaver_<name>.png at the next Present
	bool Busy();                              // a request is still waiting for its Present
	bool SaveBgra(const std::string& a_name, unsigned a_w, unsigned a_h, const std::vector<std::uint8_t>& a_bgra);  // <SKSE log dir>/Riftweaver_<name>.png from 8-bit BGRA pixels
	// A ring of the last a_pre + a_post presented frames (GPU copies, no stall while capturing). TriggerRing() starts the countdown of a_post more frames, then all frames
	// are written as <tag>_00.png ... in order and the ring disarms. For crossings: consecutive frames around the teleport.
	void ArmRing(const std::string& a_tag, int a_pre, int a_post);
	void TriggerRing();
	void DisarmRing();
}

namespace world  // engine queries: camera, raycast, player measurements, loading doors
{
	struct CamInfo
	{
		RE::NiPoint3 pos{}, fwd{}, up{}, right{};  // Skyrim camera: local X = forward, Y = up, Z = right
		float        tanLeft{ -1 }, tanRight{ 1 }, tanBottom{ -1 }, tanTop{ 1 };  // frustum direction ratios (view-space x/f and y/f at the edges)
		float        nearDist{ 1 }, farDist{ 1000 };
	};
	bool CameraFull(CamInfo& a_out);
	const char* LayerNameOf(std::uint32_t a_layer);

	struct Hit
	{
		bool                hit{ false };
		RE::NiPoint3        pos{};
		RE::NiPoint3        normal{};
		float               dist{ 0 };
		std::uint32_t       layer{ 0 };
		RE::TESObjectREFR*  ref{ nullptr };
		const void*         collidable{ nullptr };  // the hkpCollidable the ray hit
	};

	bool   Camera(RE::NiPoint3& a_pos, RE::NiPoint3& a_forward);
	Hit    Raycast(const RE::NiPoint3& a_from, const RE::NiPoint3& a_dir, float a_maxDist, RE::COL_LAYER a_layer);
	void   LogStatus();
	void   LogMeasure();
	void   LogRayFan(const std::string& a_arg);
	void   LogDoors(float a_radius);
	void   KeepActorsAnimated(float a_radius);  // G249: for every actor with loaded 3D within the radius of either portal: clear the 'not visible' flag and set the 'accumulated' flag on its root, so the engine's per-frame update treats it as seen by the camera (G247: a horse seen only through a portal had zero skeleton motion, its root carried kNotVisible and not kAccumulated)
	void   ProbeFlags(const char* a_label);  // G250: while animtrack runs, log the tracked actor's root flags at this point of the frame (to see who sets/clears the 'not visible' bit and when)
	void   FindNodes(const std::string& a_substr, int a_max);  // G254: depth-first search of the world scene graph (from the top of the player's 3D) for nodes whose name contains the text: log their flags, position, parents, controllers (type, flags, last time) and, for effect geometry, the material alpha
	void   NodeTrack(const std::string& a_substr, int a_frames);  // G254: per frame, flags / controller times / alpha of the nearest matching node
	void   AnimTrack(float a_radius, int a_frames, const std::string& a_name);  // G248: per frame, how much the skeleton of the nearest actor (name filter) moves (sum of world translation changes of up to 300 nodes), its process level and its root flags: is the actor's 3D animation updated at all?
	float  LogActors(float a_radius);  // every loaded actor within the radius, nearest first (guards marked); returns the nearest distance (or -1)
	bool   Ready();  // player and cell loaded, no loading screen
	// An id for "the area the player is in": the interior cell, or the worldspace for the outdoors (walking between outdoor cells stays the same area). 0 while loading.
	std::uint32_t CurrentArea();
	void   LogCenterRay();  // what the ray through the screen centre hits (layer, reference, distance)
	// Console lines that put the player where a camera would see the portal's picture directly: in front of the exit portal, mirrored from the virtual camera.
	bool   PoseTestLines(int a_colour, std::vector<std::string>& a_out);
	// Console lines that put the player a_dist units in front of a placed portal, a_lateral units to the side of its centre, facing the portal.
	bool   FaceLines(int a_colour, float a_dist, float a_lateral, std::vector<std::string>& a_out);
	bool   FaceBackLines(int a_colour, float a_dist, float a_lateral, std::vector<std::string>& a_out);  // G225: the player BEHIND the portal's plane, facing out through it: the real camera at the pose the virtual camera has when the player stands the same distance in front of the other portal (equal-pose reference)
	// Console lines that put the player's feet a_height units above (along the normal of) a placed portal, to fall into it.
	bool   DropLines(int a_colour, float a_height, std::vector<std::string>& a_out);
	// Console lines that stand the player 200 units south of a ground point, looking at it (to place a floor portal there).
	bool   AimGroundLines(float a_x, float a_y, float a_z, std::vector<std::string>& a_out);
	bool   SaveSpot(const std::string& a_name);
	bool   SpotCommands(const std::string& a_name, std::vector<std::string>& a_out);  // console lines that teleport to a saved spot
}

namespace portals  // portal state and placement (stage S1)
{
	struct Portal
	{
		bool         valid{ false };
		int          colour{ 0 };  // 0 = teal (the old blue), 1 = violet (the old orange)
		RE::NiPoint3 c{}, n{}, u{}, r{};  // centre on the surface (already pushed to the front-most sample), normal out of the surface, up, right
		float        hw{ 44.0f }, hh{ 96.0f };  // half width / half height in units
		double       placedAtSec{ 0 };
		std::uint32_t cell{ 0 }, worldspace{ 0 };
		std::uint32_t area{ 0 };  // where it was placed: the interior cell, or the worldspace (portals do not follow the player through a loading door)
		std::string  fitMode;
	};

	Portal Get(int a_colour);
	void   Clear();
	constexpr double kOpenSec = 0.25;   // G310: the opening animation of a portal, seconds
	constexpr double kCloseSec = 0.25;  // G310: the closing animation, seconds
	struct Ghost
	{
		Portal p;                 // the portal that a recast has just replaced: it keeps drawing, collapsing, for kCloseSec of animation time (valid false = none or finished)
		double closedAtSec{ 0 };
		float  progress{ 0 };     // 0..1 of the closing animation: starts at the first rendered frame that asks, advances once per rendered frame by that frame's time (at most 40 ms)
	};
	Ghost  GetGhost(int a_colour);
	float  OpenFraction(int a_colour, double a_now);  // 0..1 of the opening animation of the portal of that colour (same clock rules; 1 when OpenAnim is off or the portal is not new)
	struct CrossInfo
	{
		double at{ 0 };           // steady-clock seconds of the last walk through a portal
		int    exitColour{ -1 };  // the portal the player came out of
	};
	void      NoteCross(int a_exitColour);
	CrossInfo LastCross();
	void   SetSize(float a_halfW, float a_halfH);       // overrides the size derived from the player (tests)
	void   SetFactors(float a_widthFactor, float a_heightFactor);  // multiples of the player's bound-box width and height (default 2.0 and 2.5)
	void   SetVerbose(bool a_on);              // log the ray map of every accepted placement
	void   SetMargin(float a_units);           // free space required around the oval, in the plane of the surface (default 24)
	// Casts from the camera along its view direction. On success stores the portal and returns true; otherwise a_why says why not.
	bool   PlaceFromCamera(int a_colour, std::string& a_why);
	// The same placement from any pose; with a_commit false nothing is stored (a dry run that only logs what it would do).
	bool   PlaceFrom(const world::CamInfo& a_cam, int a_colour, std::string& a_why, bool a_commit);
	// Dry-run placements in 36 directions x 3 pitches from the camera: where around here would a portal of the current size fit? (logs the hits)
	void   Scan();
	void   SetUnits(float a_width, float a_height);  // absolute size in units (0 = use the multiple of the player's bound box for that axis)
}

namespace doorway  // the camera is close to a portal's plane (stage S4b): the portal is drawn as a window by a per-pixel plane test instead of a quad
{
	constexpr float kFront = 120.0f;    // units in front of the plane
	constexpr float kBehind = 100.0f;   // units behind the plane (the player's body is inside the collision hole while the camera is there)
	constexpr float kLateral = 400.0f;  // distance from the portal's centre along the plane
	inline std::atomic<bool> g_enabled{ true };
	// 0 = not in the doorway; 1 = in front of the plane, close; 2 = behind the plane, inside the oval's aperture
	inline int State(const portals::Portal& p, const world::CamInfo& cam)
	{
		const float rx = cam.pos.x - p.c.x, ry = cam.pos.y - p.c.y, rz = cam.pos.z - p.c.z;
		const float d = rx * p.n.x + ry * p.n.y + rz * p.n.z;
		const float lx = rx * p.r.x + ry * p.r.y + rz * p.r.z, ly = rx * p.u.x + ry * p.u.y + rz * p.u.z;
		if (!g_enabled.load()) {
			return 0;
		}
		if (d >= 0.0f) {
			return (d < kFront && lx * lx + ly * ly < kLateral * kLateral) ? 1 : 0;
		}
		return (d > -kBehind && (lx / p.hw) * (lx / p.hw) + (ly / p.hh) * (ly / p.hh) <= 1.44f) ? 2 : 0;
	}
}

namespace view  // the see-through view: a second world render from the virtual camera (stage S2)
{
	struct Params
	{
		bool          enabled{ true };
		bool          cone{ true };        // cull with a pyramid through the portal's screen rectangle
		bool          exitPlane{ true };   // cull with the exit portal's plane as the near plane
		float         planeBias{ -2.0f };  // units into the exit surface that the clip plane is moved (negative = in front of the oval's plane, so the exit wall's relief is clipped)
		std::int32_t  rt{ 1 };             // engine render target table: 1 = main HDR colour
		std::int32_t  ds{ 0 };             // engine depth target table: 0 = main depth
		std::uint32_t mode{ 3 };           // the engine's render target "mode" (3 is what it passes for its own passes)
		bool          oblique{ true };     // oblique near plane: patch the exit plane into the projection of the virtual camera (hides the exit wall). With 'oblique off' (G259) the mist banks and clouds showed but the wall behind the exit portal did too (16:32Z), so it is back ON and the BLENDED phase (clouds, mist, particles: Finish 106438) runs with the standard projection and the standard depth encoding instead (view blendstd=1, default): see MODLOG G260
		bool          ignorePre{ true };   // the passes' culling context sets ignorePreprocess (ctx +0x5E): the engine decides which pre-processed nodes (actors) are drawn from a visibility flag it sets for the MAIN camera (kNotVisible), so an actor the main camera cannot see is skipped by every other pass (G86)
		bool          skinFix{ false };    // test: before every extra draw (and after the last) the skin caches of the actors near the player are reset, so each camera builds its own bone matrices
		bool          fpSkin{ true };      // G284 (view fpskin=0|1): after the virtual passes the first-person skeleton's skin caches are reset, so the arms / bow / spell hands do not draw with matrices built for a virtual camera
		std::int32_t  fpSkinLog{ 0 };      // view fpskinlog=N: log the first-person skin cache state before the reset for the next N frames
		bool          selfView{ true };    // view selfview=0|1: in first person the portal pictures show the player's third-person body (the arms hidden for them)
		bool          clearAfter{ true };  // re-clear the main colour and depth after the virtual views, before the main view
		bool          predict{ false };    // measured: the camera read at the start of the frame equals the one the main view uses (0.002 deg), so no prediction is needed     // extrapolate the (one frame old) camera to where it will be when the main view is drawn
		std::int32_t  eqz{ -1 };           // force the accumulator's "equal depth test" byte (+0x5C): -1 leave it, 0 off, 1 on
		bool          forceRect{ false };  // test: run the virtual pass over the full screen whether or not the portal is in view (with identity: sky and scenery parity anywhere)
		std::int32_t  animFix{ 2 };        // G249/G250 (view animfix=<bits>): actors near the portals stay animated while only a portal shows them; 1 = mark them visible at the start of the frame (before our views), 2 = after the engine's frame function
		std::int32_t  vShadow{ 3 };  // G265: 3 = portal sun shadows ON by default (own per-portal cascades fitted to the virtual camera, `view portalshadows=0` turns them off); 0 off, 1 every virtual view, 2 only the first virtual view of a frame (G236).     // G235: render the sun's shadow cascades for each virtual camera (the engine's shadow stage 107132 + render stage 107133 with the world camera slot swapped to ours), then the engine's own mask stage; false = the mask replay with the real camera's (stale) cascades
		float         oblForce{ 0.0f };    // G227 test (with identity=1): a synthetic oblique plane this many units ahead of the camera, tilted 20 degrees sideways, replaces the exit plane: the real picture's own depth encoding without any portal geometry
		float         offYaw{ 0.0f };      // G232 test (with identity=1): the virtual camera is turned about the vertical axis by this many degrees (heading increases clockwise seen from above, like player.setangle z)
		float         offX{ 0.0f }, offY{ 0.0f }, offZ{ 0.0f };  // G229 test (with identity=1): the virtual camera is the main camera moved by this world-space offset (same orientation): the real frame at the moved pose is the ground truth for everything that depends on the camera's position
		bool          identity{ false };   // test: the virtual camera is the main camera itself (no mapping through the portals)
		bool          diag{ false };       // log what surrounds the virtual camera (rays up, down, forward, and the surface height over it)
		bool          skyMove{ true };    // move the sky (dome, clouds, sun, moons) to the virtual camera for the pass: it is centred on the real camera otherwise
		std::int32_t  shadowMask{ -1 };    // clear the screen-space sun shadow mask before the pass: -1 leave it, 1 = fully lit, 0 = fully shadowed (the engine recomputes it for the main view later)
		std::uint32_t submitFlags{ 0x51 };    // G253: default 0x51 (was 0x15). The 0x04 bit (kApplyAlphaProperty) made the lit shaders fog IN the geometry, and the engine's SSAO apply then fogged the picture a second time (G251/G252: virtual pre-apply colour = engine's colour + fog; 0x41, 0x51 and 0x11 give the engine's unfogged colour: t0 error 0.007-0.012 against 0.042, the blue cast and the lost haze contrast are gone). The engine's own colour pass uses 0x41. Old text: render flags of the virtual view's submit: 0x01 = the engine's camera jitter (default since G182: the engine's own TAA then anti-aliases the portal picture like the real one; it must match preFlags, a jittered colour pass against an unjittered pre-pass fails the equal-depth test everywhere = black picture, G179) + 0x04 kApplyAlphaProperty (required: at frame start the device holds stale blend/alpha state, without it most of the world is not drawn) + 0x10 kNoMotionVectors (keeps the extra draw from emptying the main view's motion vectors; fixes the ghosting)
		bool          alphaFix{ false };   // G118 test: during the virtual passes every draw whose geometry's NiAlphaProperty enables the alpha test is drawn with the batch renderer's alphaTest argument TRUE (the alpha reference is then set for it)
		bool          prepass{ true };    // G120 test: draw the world twice like the engine does: a depth pre-pass with a second accumulator in render mode `preMode` (the engine's mode 12: utility shader, alpha tests, tree LOD and grass depth techniques), then the colour pass with the equal-depth test
		std::uint32_t preFlags{ 0x35 };    // render flags of the pre-pass submit (the engine's own is 3); 0x01 = jitter, must match submitFlags (G179/G180)
		std::uint32_t preMode{ 12 };       // render mode written to the pre-pass accumulator (+0x150)
		bool          preUnbind{ false };   // unbind the colour targets (slots 0 to 2) for the pre-pass, as the engine does
		std::uint32_t preStart{ 0x21 };    // flags of Renderer::StartAccumulating for the pre-pass accumulator (the engine's own: 0x21 = jitter + depth pre-pass); 0 = only the accumulator's own start, as before
		std::int32_t  lightMask{ 1 };       // G127 test: while our cull runs, set the shadow-light bits (1 = the sun's shadow light, 2 = all accumulated shadow lights) in every geometry's light mask (+0x1C of its light list), which the engine fills in its own cull
		bool          ssaoClear{ false };  // G134 test: clear the AO history targets to zero before our SSAO calls (no temporal history for the virtual views)
		bool          ssaoHist{ true };    // keep a private temporal AO history per virtual view (and put the real view's history and the SSAO parity back afterwards)
		bool          ssao{ true };        // default ON since G148 ("On, keep fixing"; needs mrt and normclear below). G132: split the colour pass like Main::RenderWorld does (StartAccumulating = opaque part, then the engine's two RenderSSAO calls, then FinishAccumulatingPostResolveDepth) so the virtual picture gets the ambient occlusion the real one has
		bool          maskPass{ true };   // G129 test: after the pre-pass run the engine's shadow-mask routine (Main::RenderShadowMasks, ids 77249/77503/107141/77250 as id 107140 calls them) with the VIRTUAL camera: the screen-space sun shadow mask for the virtual view
		int           maskStat{ 0 };       // log the shadow-mask target's statistics (mean per channel) at the start of this many virtual passes and after the mask pass
		bool          preRebind{ false };  // G124: after the pre-pass bind the colour target (slot 0) again, mode 4 (no clear): the colour pass with flags 0x14 does not bind its targets itself
		bool          preRestore{ false }; // G121: after the pre-pass re-bind the colour target and depth as at the start of the pass (the engine does not; with mode 3 the depth was restored from a stale copy)
		bool          lightFlag{ false };  // G121 test: set bit 0 of the byte at +0xF4 of the render-state object (AE id 415031) that Main::RenderWorld sets for the main world pass, while the virtual passes accumulate and draw
		std::int32_t  when{ 0 };           // 0 = at the start of the frame function; 3 = right after the engine's shadow maps of the frame (the light and shadow set-up is done, the main cull has not started: the only window in which our culling does not disturb the main accumulators' pass lists); 1 and 2 hang the game
	};
	bool    Install();                   // hooks BuildCameraStateData so the oblique plane reaches the shader constants
	Params& P();
	void    Configure(const std::string& a_args);
	void    RequestStageDump(const std::string& a_label, int a_stage);  // dump the main HDR target at a stage of the engine's frame: 1 = after the colour pass's opaque part (StartAccumulating of the main accumulator returns), 2 = just before FinishAccumulatingPostResolveDepth (SSAO done)
	void    RenderViews();               // start of Main::RenderWorld, render thread
	void    LogSunShadow(const char* a_label);  // G260: with view shlog=N: the sun shadow light's cascade state (splits, sun vector, every descriptor), logged under the caller's label
	void    PostLogBegin();  // G271 (view postlog=N): from here to PostLogEnd (the end of the frame function) every draw is logged with its target, pixel shader and bound textures: which post passes run after the overlay and what they read
	void    PostLogEnd();
	bool    GpuFence(const char* a_label);  // G238: with view vsfence=1: flush, wait up to 400 ms for the GPU to finish everything queued so far, log the wait; false = it did not finish
	void InstallEarlyDeviceHook();  // SKSEPluginLoad: patch the executable's D3D11CreateDeviceAndSwapChain import so the device's CreatePixelShader is seen from the start (shader bytecode for the capture)
	void ResetFrame();  // forget this frame's virtual views (when=3: RenderViews may not be reached)
	void    ReportActual();              // end of the world pass: compares the camera the views were built from with the real one (statistics)
	void    LogStats();
	void    RequestDepthDump(const std::string& a_name);  // after the next virtual pass: log the depth buffer's coverage and save a picture of it (<name>_p<portal>)
	void*   VlPreSRV(int a_portal);    // G306: the portal picture right before its virtual VL ran (null unless VlAddValid)
	void*   VlPostSRV(int a_portal);   // G306: the same picture right after its virtual VL ran
	bool    VlAddValid(int a_portal);    // G306: this frame's VL of that view is kept apart from its picture (view vladd=1): the overlay adds (post - pre) after the engine's VL composite
	void*   ViewSRV(int a_portal);       // ID3D11ShaderResourceView* of this frame's view of that portal, or null
	bool    ExitCopy(int a_portal, int a_kind, void** a_colour, void** a_depth);  // kind 0: the part of what straddles the other portal's plane that has come out in front of this portal; kind 1: what straddles this portal's own plane, redrawn from the real camera; colour and depth SRVs, when there is one this frame
	void*   ViewTexture(int a_portal);   // ID3D11Texture2D* (last copy), for screenshots
}

namespace diag  // engine-state diagnostics for the ghosting investigation
{
	void SetChain(bool a_on);     // log whether the main camera's "previous frame" matrices still chain from the last frame while virtual views are built
	void OnBuild(void* a_entry, const void* a_cam, bool a_jitter, bool a_main, bool a_virtual);  // BuildCameraStateData hook, render thread
	void LogChain();
	void RequestMv(const std::string& a_label, int a_frames);  // read the motion-vector target back at the end of the next world passes
	void RequestParity(const std::string& a_label, int a_frames);  // compare the HDR picture of the virtual view with the real frame (meaningful with view identity=1): mean colour and ratio
	void SampleParity();          // end of the world pass, render thread (before the overlay)
	void SamplePreVl();           // G308: right BEFORE the engine's real VL composite: the HDR frame as <label>_prevl.f16 (only after an hdrdump request of the same frame)
	void SamplePostVl();          // G305: right after the engine's real VL composite: the HDR frame with the real VL added, as <label>_postvl.f16 (only after an hdrdump request of the same frame)
	void SamplePost();            // G224: after the overlay: the HDR frame with the portal pictures composited, as <label>_post.f16 (only after an hdrdump request of the same frame)
	bool DumpMainNow(const std::string& a_label);  // G131: write the main HDR target right now (render thread) as <label>_main.f16
	void RequestHdrDump(const std::string& a_label);  // G120: at the end of the next world pass write the real HDR frame (main target) and each virtual view's private texture as raw half-float files <log dir>\Riftweaver_<label>_main.f16 / _v0.f16 / _v1.f16 (16-byte header: 'F16H', width, height, DXGI format; then width*8 bytes per row)
	void LogViewMeans(const std::string& a_label);  // mean HDR colour of each virtual view's private texture (render thread, at the end of the world pass)
	void SampleMv();              // end of the world pass, render thread
}

namespace crashlog  // logs fatal-looking exceptions (module + offset, registers, stack words) before the game dies
{
	void Install();
}

namespace travel  // things that fly or fall through a portal (stage S7)
{
	void Tick(double a_dt);       // every frame on the game thread: sweeps the active projectiles against the portals and carries them through
	void SetEnabled(bool a_on);
	void SetVerbose(bool a_on);   // log the first projectiles' positions and velocities
	void SetLook(float a_frames); // how many frames ahead the sweep looks (default 1.25)
	void SetRelaunch(bool a_on);  // test: carry projectiles by launching a new one at the exit instead of moving the original (default off)
	void LogStats();
	bool Throw(const std::string& a_what, float a_speed);
	bool KillNpc();                                      // test: kill the spawned actor (it becomes a ragdoll)
	bool FlingNpc(float a_speed, int a_colour);          // test: give every ragdoll body of the (dead) spawned actor a velocity toward that portal's centre
	void LogNpcBodies();  // test: the collision objects of the spawned actor's 3D
	void LogNpc();  // test: the spawned actor's state (3D, controller, process)
	bool SpawnNpc(const std::string& a_name);            // test: spawn a loose actor (by name, e.g. "Chicken") in front of the player
	bool PutNpc(int a_colour, float a_dist, float a_yawDeg = 0.0f);  // test: put the spawned actor that far in front of that portal on its axis, facing it (turned by a_yawDeg degrees)
	bool PutNpcOnRay(float a_dist);  // test (G110): put the spawned actor on the cast's ray after the exit, that far in front of the exit plane
	bool NpcConsole(const std::string& a_cmd);  // test: run a console command on the spawned actor
	bool HoldNpc();  // test (G109): restrain the spawned actor and give it a large health pool
	void LogNpcHealth(const std::string& a_tag);  // test (G109): its health and the magic effects active on it
	bool DriveNpc(float a_speed, int a_colour);          // test: make the last spawned actor walk straight at the centre of that portal (the hole and the crossing do the rest)  // test: spawn a loose prop (ingot, apple or a hex form id) in front of the player and throw it along the view direction
	bool Cast(const std::string& a_name, float a_seconds);  // test: a real cast by the player (the caster's state machine), finished after a_seconds
	bool DriveProp(float a_speed, int a_colour, float a_seconds, float a_startDist = -1.0f);  // test: push the last thrown prop at a constant velocity toward that portal's centre (and on, mapped, after the carry) for a_seconds; a_startDist > 0 first puts it that far in front of the portal, on its axis
	void AutoShot(const std::string& a_tag, int a_count, int a_every);  // test: screenshots '<tag>_NN' every a_every frames while an entity straddles a portal's plane
	void ActorRoots(std::vector<RE::NiAVObject*>& a_out);   // the 3D roots of the loaded actors near the player (not the player)
	void SetSkinLog(int a_calls);                           // test: log the test NPC's first skinned geometry (skin instance frame id, cached bone matrix) at the next a_calls named moments of the frame
	void LogSkin(const char* a_tag);
	void HideNpcGeom(const std::string& a_sub, bool a_hide);  // test (G92): kHidden on the geometries of the test NPC whose name contains a_sub
	void HideNpcExcept(const std::string& a_sub);  // test (G94): hide every geometry of the test NPC except those whose name contains a_sub
	bool SetNpcPos(float a_x, float a_y, float a_z, float a_yawDeg);    // test (G98): put the test actor at a position, facing a heading (degrees)
	bool TestNpcCentre(RE::NiPoint3& a_out);                            // test: the test actor's centre (its ragdoll's centroid once dead), for aimthrough
	void NoteImpact(RE::Projectile* a_p, RE::TESObjectREFR* a_ref);     // from the AddImpact hook: the engine hit this actor with this projectile (the twin hit test skips it)
	void SetPushScale(float a_scale);  // test: scale of the push a carried shout/stagger gives at the exit (default 1)
	void SetConeCarry(bool a_on);   // test switch: cones (Ice Storm) carried like a missile (default) or copied at the exit
	void SetConeOrigin(bool a_on);  // test switch: a cone twin's origin (Ice Storm) moved to the exit with it (default) or left to the engine
	void NoteDroppedImpact(RE::Projectile* a_p, const RE::NiPoint3& a_hit);  // from the AddImpact hook: this projectile's wall impact inside an oval was dropped (it is carried if the sweep misses it)
	void SetTrapdoor(bool a_on);                                        // a living actor standing over a floor portal drops into it (Riftweaver.ini Trapdoor, default on)
	void WalkNpcTo(float a_x, float a_y, float a_speed, float a_seconds);  // test (G98): walk the test actor toward a point (no portal needed), one step per frame
	void WatchNpc(float a_seconds);                                      // test (G98): log every change of the test actor's state (3D, disabled, deleted, dead, process) for a_seconds, with the callers of Disable()
	void HideNpc(bool a_hide);  // test (G86): set or clear the hidden flag on the test NPC's 3D root (no pass can draw a hidden root)
	void LogNpcBounds(const char* a_tag, const float* a_planes, const RE::NiPoint3& a_camPos);  // test (G86): the test NPC's geometries (flags, world bound, skin) and their distance inside each of a pass's 6 cull planes (4 floats each: normal, constant; inside when N.P - c >= 0)
	void ResetSkinCaches();                                 // make the next draw of every skinned actor near the player rebuild its bone matrices for its own camera
	int  ResetFirstPersonSkins(int a_log);                  // G284: zero the skin frame ids under the player's first-person root (the real first-person draw then builds its own bone matrices); a_log > 0 logs how many were built this frame first
	void SetClipDraw(bool a_on);                             // test: hand the straddling entities to the clip pass (default on)
	void SetPassage(bool a_on);                              // gradual passage: rigid bodies near a portal may enter the wall inside the oval, are drawn cut while they straddle the plane, and are carried when their centre crosses it
	struct Straddler
	{
		RE::NiAVObject* root;    // the entity's 3D
		int             entry;   // the portal (colour index) whose plane it crosses
		float           radius;
	};
	void Straddlers(std::vector<Straddler>& a_out);          // entities that straddle a portal's plane this frame, for the clip pass
	void ScanSpells();                                       // test: log the kinds of spell in the game data (casting type, delivery, projectile type) with examples
	void SetTwinMode(const std::string& a_mode);             // test: how a beam or flame twin avoids the engine reusing the caster's own projectile: "other" (the other hand's casting source) or "shatter" (the launch data's new-projectile flag)
	void SetTwinPick(bool a_on, float a_startOut = 4.0f);  // G113/G114 test: a flame or beam twin runs its own target pick (off by default) and starts that far out of the exit plane
	bool EquipSpell(const std::string& a_name);              // test: put a named spell in the player's right hand and draw (a mouse press is then a real cast, as in play)
	bool Shoot(const std::string& a_what, float a_speed);  // test: an arrow (given speed, units/s) or a named spell from the player along the view direction
}

namespace ghost  // the player's body carried through a portal for the see-through pass (stage S6)
{
	void SetEnabled(bool a_on);
	bool Straddling(const portals::Portal& a_entry);  // third person, and part of the player's body is inside the entry portal's oval
	void LogStats();
	void NoteBodyPass();  // statistics
	// The pieces of the body that are seen from inside when the camera looks at a cut body from behind (eyes, mouth, teeth, tongue): hidden for the body pass, then shown again.
	std::vector<RE::NiAVObject*> HideInnerParts(RE::NiAVObject* a_root);
	void ShowInnerParts(const std::vector<RE::NiAVObject*>& a_hidden);
	void SetDebugFront(bool a_on);  // test: keep the part IN FRONT of the entry plane instead (the visible half of the body appears in the picture, to check the pass)
	bool DebugFront();
}

namespace hole  // the narrow collision exception (stage S3): the player's contacts with the portal's own surface inside the oval are dropped
{
	bool Install();                      // patches the player-controller listener's ProcessConstraintsCallback
	void SetEnabled(bool a_on);
	void SetFlight(bool a_on, float a_vx, float a_vy);  // overwrite the player's horizontal solver velocity inside the physics step (game units/s)
	void Tick();                         // every frame on the game thread: remembers the player's collision object and centre for the physics thread
	struct Body
	{
		const void*  coll;    // the actor's character-controller collidable
		RE::NiPoint3 centre;  // its capsule centre now
	};
	void        SetExtra(const std::vector<Body>& a_bodies);  // other actors that may pass through a portal (set every frame by travel): their contacts with the portal's surface are dropped too
	const void* CollidableOf(RE::Actor* a_actor);             // the collidable of an actor's character controller (null for controllers that are not the proxy kind)
	void        Drive(RE::Actor* a_actor, float a_vx, float a_vy);  // test: overwrite that actor's horizontal controller velocity inside the physics step (null = off)
	void Update(int a_index, const portals::Portal& a_p, const std::vector<const void*>& a_surfaces);
	// Rigid bodies (props, ragdoll bones, grenades, the capsule body of an NPC or animal): the contacts of a body that is near a portal with the surfaces the portal sits on are dropped
	// by a contact listener added to the body, under the same conditions as for the player (the contact inside the oval's volume, the entity's centre inside the oval prism).
	void SetBody(RE::hkpRigidBody* a_rb, const RE::NiPoint3& a_centre, float a_radius);  // add the body (first call) and refresh its entity's centre and size (every frame while it is near)
	void RemoveBody(RE::hkpRigidBody* a_rb, bool a_detach);                             // stop tracking it; a_detach = also remove the listener (false when the body may already be gone)
	int  TrackedCount();
	// Projectiles: the engine records a projectile's wall hit through the base AddImpact (id 44203); a hit on the surface a portal sits on, inside the oval, while the projectile's centre is in
	// the oval's column, is not recorded, so the projectile flies on into the wall and the travel sweep carries it when its centre crosses the plane
	bool InstallProjectileHook();       // once (safe to call again); false when the prologue check failed
	void SetProjectileHole(bool a_on);  // test switch (on after a successful install)
	bool ProjectileHoleOn();
	void LogStats();
}

namespace crossing  // walking through a portal (stage S3)
{
	void Tick(double a_dt);       // start of every frame, game thread
	void SetEnabled(bool a_on);
	int  Count();                 // how many crossings so far
	void SetFlightMode(int a_mode);  // 0 off, 1 keep the horizontal velocity after a fast exit
	// Camera roll (stage S4): after a crossing the mapped camera can be rolled (a wall portal to a floor portal turns the world on its side). The player's own
	// heading and pitch cannot express a roll, so the camera is rolled by the mapped amount and eased back to upright. Called every frame, start of the frame function,
	// after Tick and before the virtual views are built (they read the rolled camera).
	void SetMapEye(bool a_on);    // test switch (default off): map the camera exactly instead of the capsule centre
	bool Install();               // hooks the engine's camera update (id 70641): every rebuild of the world camera comes out rolled while a roll is active
	void* CamUpdateOriginal();    // the engine's own camera update, bypassing the hook (null before Install)
	void RollBuildCheck(const float* a_viewUp);  // BuildCameraStateData for the world camera: does the matrix set it just built carry the roll?
	void ApplyRoll(double a_dt);
	// The engine rebuilds the camera object once or twice per frame after our write (found with RollCheck: before the main world draw in some frames, between the world
	// draw and the post-world pass in others), so the current roll is written again at the start of the main world draw and at the start of Main::RenderWorld.
	void ReapplyRoll(const char* a_where);
	void RollCheck(const char* a_where);  // while a roll is active: log whether the camera object still holds the roll we wrote (called later in the frame)
	void SetRoll(bool a_on, double a_seconds, const std::string& a_shotTag);  // a_seconds <= 0: keep the current duration (default 0.35 s); a_shotTag: save screenshots while it eases
}
namespace view
{
	void RefreshCamera(void* a_niCamera);  // recompute the camera's derived matrices after its world transform was edited
}

namespace label  // a big on-screen label for interactive tests (G283): harness `label <text>` / `label off`
{
	void Set(const std::string& a_text);
	void Draw(void* a_swapChain);  // from the Present hook
}

namespace record  // G310: records the game's own output (the swap chain's back buffer + this process's own audio via WASAPI process loopback) for the final video (Record.cpp; harness `record start <name> [fps] [ffmpeg.exe]` / `record stop`)
{
	bool Start(const std::string& a_name, const std::string& a_ffmpeg, int a_fps);
	void Stop();
	bool Active();
	void OnPresent(void* a_swapChain);  // from the Present hook (Shot.cpp)
}

namespace audio  // G310 phase 3: the portal sounds, procedural WAVs from Data\SKSE\Plugins\RiftweaverSounds mixed by XAudio2 (Audio.cpp)
{
	void Tick();  // every rendered frame (from overlay::Draw): cast loop while a hand holds the spell, release + opening on a new portal, closing on a replaced one, walk-through, ambient hum within about 600 units
}

namespace killcam  // through-portal killcam: slow time and ride behind a carried projectile that will kill (the game's own killcam never starts through a portal)
{
	void OnCarried(RE::Projectile* a_p, const RE::NiPoint3& a_exitPos, const RE::NiPoint3& a_dir);  // from the projectile carry: starts when the path out of the exit meets an actor the hit should kill
	void Tick();                                                                                      // every frame (camera follow, hold, end)
	bool Active();
	void SetEnabled(bool a_on);  // Riftweaver.ini Killcam (default on)
}

namespace overlay  // draws the placed portals into the frame (after the world, before post-processing)
{
	void Draw();                  // called at the end of Main::RenderWorld on the render thread
	void VlAddBegin();            // G308: right before the engine's real VL composite: keeps the main HDR target (what the engine's VL composite adds is then known and not added a second time)
	void SetVlSub(int a_on);      // G308: view vlsub=0|1 (default 1): the virtual VL add is reduced by what the engine's own composite just added at the oval's pixels
	void DrawVlAdd();              // G306: called right after the engine's real VL composite: adds the virtual VL (post - pre) of each portal picture onto the oval pixels
	void DrawMoments(bool a_fromComposite = true);  // G310: called right after DrawVlAdd (the first-person hands are in the frame by then): the walk-through ripple, the glow and rune ring at the casting hands, the release flash
	void AddRuneRing(std::uint32_t a_id, const RE::NiPoint3& a_pos, const RE::NiPoint3& a_normal, int a_element, float a_radius);  // our glyph ring where a rune landed after the exit (element 0 fire, 1 frost, 2 lightning)
	void KeepRuneRings(const std::set<std::uint32_t>& a_alive);
	bool LastRuneRing(RE::NiPoint3& a_pos, RE::NiPoint3& a_normal);  // test: the newest rune ring  // the runes still armed (each tick); the other rings fade out
	void Enable(bool a_on);
	void SetDepthBias(float a_units);  // how far in front of the oval's plane a nearer surface may be and still be painted over (the wall's own relief)
	void SetDepthFar(bool a_on);    // test: the portal picture writes FAR depth (the engine's depth-based post effects, such as distance haze, then treat it as far scenery)
	void SetDepthMode(int a_mode);  // 0 standard depth, 1 reversed, 2 no depth test
	void RequestProbe();            // next Draw: read back the depth buffer at 5 pixels, compare with raycasts, pick the depth convention
	void SetShotTag(const std::string&);  // unused for now
}

namespace input  // test input through the game's own input queue (the harness casts the portal spells like a player)
{
	void InjectJump();               // the game's own Jump control through the input queue (tests of jumping onto a floor portal)
	void InjectShout(int a_holdFrames = 6);  // test: press the Shout key through the input queue, held a_holdFrames (a tap = the first word; ~90 = all three)
	void InjectMouseMove(int a_dx, int a_dy, int a_frames);  // mouse look through the input queue, dx/dy per frame (tests: pan with a cast held)
	void Tick();                     // every frame from the frame hook: pushes the pending mouse look
	void InjectQuicksave();          // the game's own quicksave: the F5 key event through the input queue (test and setup tool: writes a real Quicksave file)
	void InjectMouse(const std::string& a_button, int a_hold = 8);  // a click through the game's own input queue (left or right) held for a_hold frames, for tests: SendInput does not reach the game's DirectInput mouse. The left button is the RIGHT hand, the right button the LEFT hand.
}

namespace spells  // the two portal spells (Riftweaver.esp): casting one places that colour's portal where the camera aims (replaces the portal gun)
{
	// colour index everywhere: 0 = teal (Rift: Tidal Weave, the old blue), 1 = violet (Rift: Umbral Weave, the old orange)
	bool Install();                      // the spell-cast and effect-apply event sinks (two independent triggers; the per-frame poll in Tick() is the third)
	void RemoveAllForTest();              // test: take both spells away from the player (spellgift fresh)
	void OnLoad(bool a_newGame);         // a save was loaded / a new game started: settings from Riftweaver.ini, retired gun removed, the two spells added when missing
	void Tick();                         // every frame on the game thread: the third trigger (magicka paid while our spell charges) and the hand state for the effects
	bool GiveAll();                      // test and gift: add both spells to the player
	bool Equip(int a_colour, bool a_rightHand);  // test: add the spell if needed, put it in a hand and draw the hands
	void LogStatus();
	int  ColourOfSpell(std::uint32_t a_formId);  // 0/1 when the form is one of our spells or effects, else -1
	struct HandFx
	{
		bool         active{ false };  // our spell is being readied/charged/cast in this hand
		int          colour{ -1 };
		RE::NiPoint3 pos{};            // world position of the hand's magic node
		RE::NiPoint3 camRel{};         // first person: the same point relative to the camera read at the same moment (right, up, forward)
		bool         firstPerson{ false };
		double       at{ 0 };          // steady-clock seconds of the last update
	};
	void GetHandFx(HandFx a_out[2]);     // render thread: copy of the latest hand state
	bool HandWorldNow(int a_hand, RE::NiPoint3& a_out);  // render thread: the hand's magic node position as the engine has it now (first-person arms in first person)
	bool ParticlesEnabled();             // Riftweaver.ini Particles=1 (default)
	// The look, from Riftweaver.ini (re-read while the game runs whenever the file changes) and the harness `style` command:
	//   Particles (0/1), ParticleDensity (x the emission rates), ParticleAlpha (x the particle opacity), Halo (strength of the haze around the rim, 0 = none), Rim (strength of the faded rim band, 0 = none)
	struct Style
	{
		bool  particles{ true };
		float density{ 1.0f };
		float alpha{ 1.0f };
		float halo{ 0.7f };
		float rim{ 1.0f };
		// G309 premium portal look (phase 1); every one is switchable in Riftweaver.ini (0 = the old look), hot-reloaded
		float rimFlow{ 1.0f };      // RimFlow: a white-hot core of the rim that flows around the oval
		float rimGlints{ 1.0f };    // RimGlints: bright glints that travel along the rim
		float ripple{ 1.0f };       // Ripple: a surface ripple just inside the rim
		float sheen{ 1.0f };        // Sheen: a light sweep over the surface when close
		float mistSwirl{ 1.0f };    // MistSwirl: the mist turns around the portal's axis
		float embers{ 1.0f };       // Embers: sparks that rise off the rim
		bool  light{ true };        // Light: a coloured dynamic light at each portal
		float lightRadius{ 250.0f };      // LightRadius (units)
		float lightIntensity{ 1.0f };     // LightIntensity
		// G310 premium moments (phase 2); every one is switchable in Riftweaver.ini (0 = off), hot-reloaded
		float openAnim{ 1.0f };      // OpenAnim: the 0.25 s opening (tear, iris, spark ring, wall flash)
		float closeAnim{ 1.0f };     // CloseAnim: the 0.25 s closing (implosion and a pop) when a portal is replaced by a recast
		float handRunes{ 1.0f };     // HandRunes: a glow and a rune ring in front of the casting hand
		float releaseFlash{ 1.0f };  // ReleaseFlash: a flash at the hand when the spell lands
		float walkRipple{ 1.0f };    // WalkRipple: a 0.3 s screen ripple and an edge tint when walking through
		float exitPuff{ 1.0f };      // ExitPuff: a puff of mist at the exit portal when walking through
		// G310 phase 3: the portal sounds (audio namespace)
		float sounds{ 1.0f };        // Sounds: volume of all portal sounds (0 = off)
		float humVolume{ 1.0f };     // HumVolume: the ambient hum of an open portal (0 = none)
	};
	Style GetStyle();
	void  SetStyle(const Style& a_style);  // test: overrides what the ini said until the ini changes again
}

namespace fx  // portal particles drawn by the plugin: miasma drifting off the oval's rim, an opening burst, a wisp at the casting hand (Fx.cpp, called from the overlay)
{
	struct Frame
	{
		void*        dev{ nullptr };       // ID3D11Device*
		void*        ctx{ nullptr };       // ID3D11DeviceContext* (the overlay's states are bound: main HDR target + depth, viewport, rasterizer without culling, triangle strip)
		void*        dssTest{ nullptr };   // ID3D11DepthStencilState*: depth test without write (or none when the depth mode is off)
		int          depthMode{ 0 };       // 0 standard, 1 reversed, 2 none
		double       now{ 0 };             // steady-clock seconds
		int          width{ 0 };           // the render target's width in pixels (for the HandFxLog pixel offsets)
	};
	bool Active();                                            // something to simulate or draw (portals placed, hand wisps, live particles)
	void Render(const Frame& a_frame, const world::CamInfo& a_cam, const portals::Portal& a_teal, const portals::Portal& a_violet);  // simulate and draw (SEH-guarded; disables itself on an exception)
	void SetEnabled(bool a_on);
	void SetHandWisps(bool a_on);                             // Riftweaver.ini HandWisps (default 0): the plugin's own smoke wisps and release sparks at the casting hand (the spells' Casting Art NIF replaces them)
	void SetHandFxLog(bool a_on);                             // Riftweaver.ini HandFxLog=1: log the hand effects' positions while a Rift cast is held
	bool HandAnchor(int a_hand, const world::CamInfo& a_cam, RE::NiPoint3& a_out);  // the hand as the arms are drawn this frame (set by the particles' step; false when not fresh)
	void SetHandCarry(bool a_on);                             // tests: the hand particles move with the hand (default) or stay in the world (old, for before/after)
	void SetReach(float a_units);                             // tests: how far from the oval's rim the particles may reach (default 30 units; a large value = no limit, the pre-G300 look)
	void LogStatus();
}

namespace camera  // the player's view mode (tests of the gun's model in the hand)
{
	void SetThirdPerson(bool a_on);
}

namespace spike  // stage S3 experiments
{
	void PressMouse(const std::string& a_button);  // a real click ("left" or "right") after bringing the game window to the front
	void FocusGame();
	void LogController();
	void LogWall();
	bool BlockingMenuOpen(std::string* a_what);  // a dialogue, inventory, map, message box ... is open, or the player's movement controls are disabled
	void PressKey(const std::string& a_name);    // a real key press (esc, tab, enter, space) after bringing the game window to the front
	void Walk(double a_seconds, unsigned short a_scan = 0x11);  // holds the W key (or A 0x1E / D 0x20 to strafe; a real key press) for that long while recording the player's track
	void Track(double a_seconds); // logs position, velocity and health a few times a second
}

namespace harness  // command file, ordered queue, waits
{
	void Start();
	void Mute(bool a_on);  // test runs: all top-level sound categories and our portal sounds to 0, in memory only
}
