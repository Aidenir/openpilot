# CLAUDE.md

Guidance for Claude Code working in this repository.

> **Read the project notes first.** They live in the private mapd repo (`Aidenir/mapd`, branch `speed-bump-v2`) at
> `docs/project/PROJECT-NOTES.md`; on the dev server `~/PROJECT-NOTES.md` links to it. They have the cross-repo
> context: why this branch exists and how it relates to FrogPilot's other branches, how the comma device is reached
> and deployed to, and the traps that have caused real outages on the car. Setting up a new server:
> `docs/project/NEW-SERVER.md` there.

## What this is

A FrogPilot fork of openpilot on branch `viktor-mapd-2026`, rooted on **openpilot
0.10.3**. It is paired with the `mapd` Go daemon in `/home/ubuntu/mapd` (branch
`speed-bump-v2`) to add speed-bump detection. The mapd **binary is committed here**
at `frogpilot/navigation/mapd`; there is no runtime download on this branch.

## Before you push anything

**Build it.** openpilot compiles on an x86 box as checked out:

```sh
tools/install_ubuntu_dependencies.sh && tools/install_python_dependencies.sh   # once
source .venv/bin/activate && scons -j$(nproc)
python -m pytest frogpilot/controls/tests frogpilot/system/tests
```

Off the device the UI links without the Qualcomm OpenMAX screen recorder: `selfdrive/ui/SConscript` builds
`frogpilot/ui/screenrecorder/screenrecorder_x86_stub.cc` instead (only when not `larch64`). In a worktree
(`~/wt/op-<name>`), `ln -s ~/openpilot/.venv .venv` first.

**Check params typing.** `Params.get()` takes no `encoding=` and returns the type
registered in `common/params_keys.h`. `Params.put()` is strictly typed with no
coercion — not even `int` into a FLOAT param. Three separate on-car outages came
from code ported from an older branch where the API differed. Do not copy params
code between branches without re-checking the API.

**Do not `git add -A` after a build.** scons writes `moc_*.cc` and `.qm` files next
to the sources; they have been committed by accident before (they are gitignored now).

**This repo is public.** Device addresses, tokens and notes about the car go in the mapd repo's
`docs/project/`, not here.

## Deploying

Push to `viktor-mapd-2026`, then on the car `git pull` in `/data/openpilot` and reboot (the updater is unreliable: on
2026-10-28 it had not fetched since 10-17). `git pull` swaps the mapd binary on disk but does **not** restart the
running daemon — reboot, or the old one keeps running. Map tiles are not in git and must be copied separately; see
the project notes.
