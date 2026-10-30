---
description: Fix a GitHub issue end-to-end (reproduce, test, fix, document)
argument-hint: <issue number or description>
allowed-tools: Bash, Read, Edit, Write, Grep, Glob
---
Fix issue: $ARGUMENTS

1. Understand: read the issue (use the GitHub MCP server if it is a number) and find the code involved (`CLAUDE.md` → "Where things are").
2. Reproduce: write a **failing test** first (pytest in `pm3rrr_core/test`, `firmware/test`, or a scenario run).
3. Fix: smallest correct change in the new code (never in `legacy/`). Respect the invariants in `CLAUDE.md`.
4. Verify: the new test passes, plus the checks for every touched area (see `/review`).
5. Document: update the affected lesson or scenario numbers, and regenerate figures or GIFs if their content changed.
6. Commit on a branch `fix/<short-name>` with an imperative message that references the issue.
