---
description: Review the current changes against the project invariants (maths, URDF, firmware, docs)
allowed-tools: Bash(git diff:*), Bash(git status), Bash(python3 -m pytest:*), Bash(python3 tools/cad_to_urdf/step4_validate_urdf.py:*), Bash(make -C firmware/test:*), Read, Grep, Glob
---
Review the pending changes (`git diff` plus untracked files) for this robotics project. $ARGUMENTS

1. Run `git status` and `git diff` to see what changed.
2. Check the invariants in `CLAUDE.md` (SI units, zero = home, working mode, guard on every command, generated files untouched, `legacy/` untouched).
3. Run the matching checks:
   - `pm3rrr_core` changed → `python3 -m pytest -q ros2_ws/src/pm3rrr_core/test`
   - URDF/xacro/yaml changed → `python3 tools/cad_to_urdf/step4_validate_urdf.py`
   - firmware changed → `make -C firmware/test`
4. Consistency: are numbers quoted in `README.md`, `docs/learning`, `docs/scenarios` still true? Are new topics documented in lesson 12?
5. Use the `code-reviewer` agent for a deeper pass if the diff touches more than one package.

Report: blocking issues first (with file:line), then suggestions. Do not edit files unless asked.
