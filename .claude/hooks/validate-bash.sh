#!/usr/bin/env bash
# PreToolUse hook (Bash): block dangerous or project-breaking commands.
# Input: tool call as JSON on stdin. Exit 2 = block (stderr is shown to Claude).
set -euo pipefail

cmd=$(python3 -c 'import json,sys; print(json.load(sys.stdin).get("tool_input",{}).get("command",""))' 2>/dev/null || true)

block() { echo "BLOCKED by .claude/hooks/validate-bash.sh: $1" >&2; exit 2; }

# Only inspect real shell commands: drop heredoc bodies (file contents written by the
# command, e.g. docs mentioning "legacy/") and split on ; && || | into simple commands.
cmds=$(printf '%s\n' "$cmd" | awk '
  inhd { if ($0 == tag) inhd = 0; next }
  { line = $0
    if (match(line, /<<-?[[:space:]]*["\047]?[A-Za-z_]+["\047]?/)) {
      t = substr(line, RSTART, RLENGTH); gsub(/<<-?[[:space:]]*|["\047]/, "", t); tag = t; inhd = 1 }
    print line }' | sed -E 's/(\&\&|\|\||;|\|)/\n/g' | sed -E 's/^[[:space:]]+//')

while IFS= read -r c; do
  # 1. Destructive file-system / git operations
  [[ "$c" =~ ^(sudo[[:space:]]+)?rm[[:space:]]+-[a-zA-Z]*r[a-zA-Z]*f?[[:space:]]+(/|~|\$HOME|\.)([[:space:]]|$) ]] && block "recursive delete of a root/home/project directory"
  [[ "$c" =~ ^git[[:space:]]+push([[:space:]].*)?[[:space:]](--force|-f)([[:space:]]|$) ]] && block "force push; use --force-with-lease on your own branch, after asking"
  [[ "$c" =~ ^git[[:space:]]+reset[[:space:]]+--hard ]] && block "git reset --hard discards work; ask the user first"

  # 2. Never rewrite the historical record
  [[ "$c" =~ ^(rm|mv|sed[[:space:]]+-i|truncate)[[:space:]].*(^|[[:space:]/])legacy/ ]] && block "legacy/ is read-only (historical record)"

  # 3. Generated files must be regenerated, not hand-edited by shell
  [[ "$c" =~ ^sed[[:space:]]+-i.*(inertials\.yaml|pm3rrr\.srdf|meshes/) ]] && block "generated file: rerun tools/cad_to_urdf/step2/step3 instead"
done <<< "$cmds"

# 4. Flashing hardware requires the firmware host tests to pass first
if [[ "$cmd" =~ arduino-cli[[:space:]]+upload ]]; then
  root="${CLAUDE_PROJECT_DIR:-.}"
  make -s -C "$root/firmware/test" >/dev/null 2>&1 || block "firmware host tests fail (make -C firmware/test); fix before uploading"
fi

exit 0
