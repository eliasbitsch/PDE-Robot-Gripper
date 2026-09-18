---
workflow: general-video
flow: automation
storyboard: no
message: "The plan is set - tell me what you'd change"
destination: file
aspect: 1920x1080
language: en
audience: "Viktoriia, project partner, mechanical engineering student"
length: 87s
angle: plan-pitch
---

## Intent

Pitch video for Viktoriia. In under a minute and a half she should see: what we
grip and where, why a 2-finger four-bar gripper with a servo, which tool does
what, which two dates are hard, what each package is worth, who takes what, and
the route from September to January.

Tone: collegial and concrete, no marketing. It is a proposal, not a decision -
the ending explicitly invites her to push back. Technical, dark, precise. No
voiceover, no music bed: she will most likely watch it muted on a phone.

Per the author: **timeline and task split are the core scenes.** The toolchain
is the surprise, not the main content. Points per package must be visible so it
is clear what is worth what.

## Assets

- `assets/part/part_b.png` - part B, isometric, tessellated from Bauteil_B.STEP via build123d
- `assets/part/part_b_title.png` - same view, used on the title scene
- `assets/part/part_b_grip.png` - top view, both grip faces marked, jaw arrows, 70 mm dimension
- `assets/part/linkage.png` - four-bar schematic drawn from the real geometry (90 mm links, +-4.46 deg, 0.27 mm scrub)
- `assets/part/robotiq.jpg` - Robotiq 2F-85 / 2F-140 product photo, design reference. Copyright Robotiq Inc., credited on screen; source note in shared/references/SOURCES.md
- `assets/shots/onshape.png` - Onshape landing page, advertises FeatureScript MCP itself
- `assets/shots/mujoco.png` - mujoco.org
- `assets/logos/onshape.png`, `mujoco.png`, `latex.svg`, `github.svg` - official marks
- `assets/work/fem_konvergenz.png`, `ablauf_4_transport.png` - own images from the preliminary work

Only assets actually referenced by index.html are kept in the repo.

## Customizations

- Real logos and real website screenshots instead of rebuilt tiles - explicitly requested.
- Own working images included so it is visible that the chain already runs.
- The two red deadlines (26 Nov / 15 Jan) must read as the hardest thing on screen.
- Points shown per package, matching the Moodle breakdown exactly.
- Ending with the repo link and an explicit invitation to disagree.

## Notes

- English throughout, including the repo name.
- No invented numbers. Points, dates and figures come from `moodle/` and `PLAN.md`.
- Viktoriia is in course group BB-2, Elias in BB-1 - the video does not claim
  the date question is settled, it names it as open.
