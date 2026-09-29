# CLAUDE.md

Guidance for Claude Code working in this repository.

> **Read `/home/ubuntu/PROJECT-NOTES.md` first.** It has the cross-repo context:
> why this branch exists and how it relates to FrogPilot's other branches, how the
> comma device is reached and deployed to, and the traps that have caused real
> outages on the car.

## What this is

A FrogPilot fork of openpilot on branch `viktor-mapd-2026`, rooted on **openpilot
0.10.3**. It is paired with the `mapd` Go daemon in `/home/ubuntu/mapd` (branch
`speed-bump-v2`) to add speed-bump detection. The mapd **binary is committed here**
at `frogpilot/navigation/mapd`; there is no runtime download on this branch.

## Before you push anything

**Build it.** openpilot compiles on this x86 box:

```sh
source .venv/bin/activate && scons -j8
```

Two local-only edits are required for the build to link here, and must **not** be
committed: `#include <QPainterPath>` in `frogpilot/ui/qt/onroad/frogpilot_annotated_camera.cc`,
and arch-gating `OmxCore` + the screen recorder in `selfdrive/ui/SConscript`.

**Check params typing.** `Params.get()` takes no `encoding=` and returns the type
registered in `common/params_keys.h`. `Params.put()` is strictly typed with no
coercion — not even `int` into a FLOAT param. Three separate on-car outages came
from code ported from an older branch where the API differed. Do not copy params
code between branches without re-checking the API.

**Do not `git add -A` after a build.** scons writes `moc_*.cc` and `.qm` files next
to the sources; they have been committed by accident before (they are gitignored now).

## Deploying

`git pull` on the device swaps the mapd binary on disk but does **not** restart the
running daemon — reboot, or the old one keeps running. Map tiles are not in git and
must be copied separately; see PROJECT-NOTES.
