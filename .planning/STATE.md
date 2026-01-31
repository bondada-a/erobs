# Project State

## Project Reference

See: .planning/PROJECT.md

**Core value:** Reliable, autonomous sample manipulation — scientists write simple commands, system works consistently
**Current focus:** Phase 1 — DSSI Handoff Prep

## Current Position

Phase: 1 of 4 (DSSI Handoff Prep)
Status: In progress — codebase cleanup on `refactor/codebase-cleanup` branch
Last activity: 2026-01-31 — Comprehensive codebase review, cleanup commits

Progress: ██░░░░░░░░ 20%

## What's Done

- ✅ Phase 0: Communication Research (see phases/01-communication-research/01-RESEARCH.md)
- ✅ Root-level file organization
- ✅ Gitignore updates
- ✅ License declarations
- ✅ Debug print fix, typo fix
- ✅ Comprehensive 534-file review (~97 issues documented)
- 🔄 README rewrite (Claude Code session in progress)

## What's Next

Immediate (Phase 1 blockers):
1. Fix critical bugs (HandE mass, UR3e timeout, obstacle z-values)
2. Enable tests in CI
3. Update old repo URLs in Dockerfiles
4. Test containers on ws2
5. Document container usage for DSSI

## Blockers/Concerns

- Phase 4 blocked on DSSI delivering the communication bridge
- Phase 4 blocked on beamline IT input for security constraints
- Need to schedule DSSI handoff meeting after Phase 1 complete

## Session Continuity

Last session: 2026-01-31
Stopped at: Reviewing CLEANUP.md with Rocky, updating roadmap
Active branch: refactor/codebase-cleanup
Claude Code session: erobs-readme (README rewrite)
