# Riftweaver - Seamless Portals for Skyrim: source code

This is the corresponding source code of `SKSE\Plugins\Riftweaver.dll` in **Riftweaver-0.1-1.6.1170** (plugin version 0.1.0), released under the GNU General Public License v3.0 or later (`LICENSE.txt`). Third-party code and licenses: `THIRD-PARTY-NOTICES.txt`.

Contents:
- `plugin/` - the plugin's source (C++23). Every file is compiled into the DLL; the test-harness files (Harness.cpp, Record.cpp, Shot.cpp, Label.cpp, Diag.cpp, Spike.cpp, Ghost.cpp) are compiled too, but the release build (`RW_RELEASE`) never starts them.
- `xmake.lua` - the build file. The release DLL is the target `RiftweaverNexus`.
- `lib/commonlibsse-ng/` - CommonLibSSE NG 12.2.0 (commit 67b201b6fe45241ac5c1e4630ecf847523775b66), unmodified.
- `third_party/minhook/` - MinHook 1.3.4, unmodified.

The sounds, meshes, texture and Riftweaver.esp of the mod are not part of this source release (see THIRD-PARTY-NOTICES.txt).

## Build

Requirements: Windows 10/11 x64, Visual Studio 2022 Build Tools (MSVC v143 with the C++ workload and a Windows 10/11 SDK), xmake 3.0 or newer (https://xmake.io), internet access on the first build (xmake downloads spdlog 1.16.0, DirectXTK 24.2.0, DirectXMath 2024.02 and Xbyak 7.06).

```
xmake f -m releasedbg -y
xmake build -y RiftweaverNexus
```

The DLL is written to `build/windows/x64/releasedbg/RiftweaverNexus.dll`. Rename it to `Riftweaver.dll` and put it in `Data/SKSE/Plugins/` (the plugin finds `Riftweaver.ini` by its own file name). The `SkyrimPortalProto` target in `xmake.lua` is an old prototype whose sources are not included; build `RiftweaverNexus` only.

Source commit: 45df893. The shipped DLL was built from commit 2cdb0f9; this commit only adds the copyright and license comment at the top of each source file (so line numbers in log messages are 15 higher).
