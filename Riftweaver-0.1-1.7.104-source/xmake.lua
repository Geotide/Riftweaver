-- Riftweaver - Copyright (C) 2026 Ph3nixxSTRise
--
-- This program is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program.  If not, see <https://www.gnu.org/licenses/>.

set_xmakever("3.0.0")
set_project("SkyrimPortalProto")
set_version("0.1.0")
set_arch("x64")
set_languages("c++23")
set_warnings("allextra")
set_encodings("utf-8")

add_rules("mode.debug", "mode.releasedbg")

-- keep the local build folder out of the binaries (__FILE__ / source_location strings)
add_cxxflags("cl::/d1trimfile:$(projectdir)\\", { force = true })

-- Only Skyrim SE/AE 1.7.104 matters for this prototype. AE-only keeps the struct layout flat
-- and the CommonLib source build small. skse_xbyak is needed later for trampolines/codegen.
set_config("skyrim_se", false)
set_config("skyrim_ae", true)
set_config("skyrim_vr", false)
set_config("skse_xbyak", true)

includes("lib/commonlibsse-ng")

-- MinHook 1.3.4 (BSD-2-Clause), vendored in third_party/minhook, used for function-entry detours.
target("minhook", function()
    set_kind("static")
    set_languages("c11")
    set_warnings("none")
    add_files("third_party/minhook/src/*.c", "third_party/minhook/src/hde/hde64.c")
    add_includedirs("third_party/minhook/include", "third_party/minhook/src", { public = false })
    add_includedirs("third_party/minhook/include", { public = true })
end)

target("SkyrimPortalProto", function()
    add_deps("commonlibsse-ng", "minhook")
    add_rules("commonlibsse-ng.plugin", {
        name = "SkyrimPortalProto",
        author = "Ph3nixxSTRise",
        description = "Throwaway prototype: second world render with a custom view cone",
        options = { address_library = true }
    })
    add_files("src/*.cpp")
    add_includedirs("src")
    set_pcxxheader("src/PCH.h")
    add_syslinks("d3d11", "dxgi", "d3dcompiler", "user32", "windowscodecs", "ole32", "uuid")
end)

-- The full mod. src/ above stays as the throwaway prototype (tag prototype-final); plugin/ is the clean rewrite.
target("Riftweaver", function()
    add_deps("commonlibsse-ng", "minhook")
    add_rules("commonlibsse-ng.plugin", {
        name = "Riftweaver",
        author = "Ph3nixxSTRise",
        description = "Riftweaver - Seamless Portals for Skyrim",
        options = { address_library = true }
    })
    add_files("plugin/*.cpp")
    add_includedirs("plugin")
    set_pcxxheader("plugin/PCH.h")
    add_syslinks("d3d11", "dxgi", "d3dcompiler", "user32", "gdi32", "windowscodecs", "ole32", "uuid", "xaudio2", "mmdevapi")
end)

-- The Nexus release (0.1.0, Nexus version "0.1"): the same sources with RW_RELEASE (no test harness, mute flag, Present hook, bAlwaysActive override or retired-gun code; warnings-only
-- logging after start-up; refuses to load on runtimes other than 1.7.104). The DLL is renamed to Riftweaver.dll when packaged (the ini is found by the DLL's own name).
-- /d1trimfile and /PDBALTPATH keep the local build paths out of the shipped DLL; /OPT:REF drops the code only the test harness used.
local function riftweaver_release(name, smoke)
    target(name, function()
        add_deps("commonlibsse-ng", "minhook")
        set_version("0.1.0")
        add_rules("commonlibsse-ng.plugin", {
            name = "Riftweaver",
            author = "Ph3nixxSTRise",
            description = "Riftweaver - Seamless Portals for Skyrim",
            options = { address_library = true }
        })
        add_files("plugin/*.cpp")
        add_includedirs("plugin")
        set_pcxxheader("plugin/PCH.h")
        add_defines("RW_RELEASE", "RW_VERSION_STRING=\"0.1.0\"")
        if smoke then
            add_defines("RW_TEST_HOOKS")  -- smoke-test twin only: the harness, the mute flag and the Present hook are back; never shipped
        end
        add_shflags("/OPT:REF", "/OPT:ICF", "/PDBALTPATH:Riftweaver.pdb", { force = true })  -- shflags: the link flags of a shared library
        add_syslinks("d3d11", "dxgi", "d3dcompiler", "user32", "gdi32", "windowscodecs", "ole32", "uuid", "xaudio2", "mmdevapi")
    end)
end
riftweaver_release("RiftweaverNexus", false)
riftweaver_release("RiftweaverSmoke", true)
