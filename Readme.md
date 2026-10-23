@'
# Riftweaver

An SKSE plugin for Skyrim Special Edition (1.6.1170 and 1.7.104) that adds two Alteration spells, Rift: Umbral Weave (violet) and Rift: Tidal Weave (teal). Each spell opens one end of a linked pair of seamless rifts. You see straight through to the other side, live, and you can walk, fall and fight through them.

What goes through: you (first and third person, momentum kept), arrows, spells, summons, NPCs, ragdolls (Fus Ro Dah them through), trapdoors under enemies' feet, a through-rift killcam, and you can even see and shoot yourself.

How it works: the plugin renders the world a second time with the game's own pipeline from a virtual camera behind the other rift, clipped to the rift's plane, and draws the rim glow and particles itself.

Players: download the mod from its Nexus Mods page (Riftweaver). This repo is the source code only.

## Requirements
- Skyrim SE 1.6.1170 (SKSE64 2.2.6) or 1.7.104 (SKSE64 2.3.1)
- Address Library for SKSE Plugins

## Building
See BUILD.md. Branch `main` is the 1.6.1170 build; branch `release-0.1-1.7.104` is the 1.7.104 build.

## License
The code is GPL-3.0-or-later (see LICENSE.txt), because it is built with CommonLibSSE-NG. Third-party notices are in THIRD-PARTY-NOTICES.txt. The mod's sounds, meshes, texture and ESP ship on Nexus and may be reused with credit.

## Credits
- Created by Ph3nixxSTRise (Phoenix Studios Rise): idea, design, testing and direction.
- Built with help from Claude (AI) on the coding side.
- Thanks to the SKSE team and the makers of Address Library and CommonLibSSE-NG.
- Inspired by Valve's Portal games. Not affiliated with or endorsed by Valve.
'@ | Set-Content -Encoding utf8 README.md
@'
build/
.xmake/
vsxmake*/
*.dll
*.pdb
*.log
'@ | Set-Content -Encoding utf8 .gitignore
git add -A
git commit -m "Riftweaver 0.1 source (SE 1.6.1170)"
git tag v0.1-1.6.1170