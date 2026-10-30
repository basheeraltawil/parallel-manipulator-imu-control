#!/usr/bin/env bash
# PostToolUse hook (Edit/Write): fast consistency checks after an edit.
#  * robot model or geometry edited  -> validate URDF against the kinematics
#  * pm3rrr_core edited              -> run the core unit tests
#  * firmware kinematics/protocol    -> run the firmware host tests
# Exit 2 sends the failure back to Claude so it fixes it immediately.
set -uo pipefail

file=$(python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("tool_input",{}).get("file_path",""))' 2>/dev/null || true)
root="${CLAUDE_PROJECT_DIR:-.}"
cd "$root" || exit 0

run() {
  local out
  if ! out=$("$@" 2>&1); then
    echo "post-edit check failed: $*" >&2
    echo "$out" | tail -n 25 >&2
    exit 2
  fi
}

case "$file" in
  *pm3rrr_description/urdf/*|*pm3rrr_description/config/robot_params.yaml|*pm3rrr_description/config/inertials.yaml)
    run python3 tools/cad_to_urdf/step4_validate_urdf.py --samples 50 ;;
  *pm3rrr_core/pm3rrr_core/*.py)
    run python3 -m pytest -q -x ros2_ws/src/pm3rrr_core/test ;;
  *firmware/pm3rrr_firmware/kinematics.*|*firmware/pm3rrr_firmware/protocol.*|*firmware/pm3rrr_firmware/config.h)
    run make -s -C firmware/test ;;
esac
exit 0
