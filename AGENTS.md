# AGENTS.md

## Project context

This repository contains a private development fork for a 2025 Hyundai Sonata AWD (Canadian market, non-HDA II target) using comma four and Hyundai A harness.

Primary upstream is sunnypilot `ccnc-port`.

Read these before making changes:

- `docs/SONATA_PRIVATE_ROADMAP.md`
- `docs/SAFETY_GUARDRAILS.md`

## Branch policy

- `sonata-base`: clean mirror/reference of upstream `ccnc-port`; do not add project features.
- `sonata-dev`: integration branch for reviewed development.
- `sonata-stable`: last-known-good daily-driver branch.
- `sonata-radar-lab`: isolated radar research.
- Create feature branches for substantial work.

Do not commit directly to `sonata-stable` for new behavior.

## Required behavior for coding agents

Before editing vehicle-control code:

1. identify the exact subsystem and files involved
2. explain expected vehicle behavior
3. identify safety implications
4. identify whether the feature can be tested passively or through replay first
5. identify relevant upstream implementation/reference
6. keep the change minimal and reversible

## Restricted areas

Do not modify any of the following without explicit human approval:

- panda safety logic or safety model
- steering torque/rate/angle limits
- longitudinal acceleration or braking safety limits
- brake/CANCEL disengagement behavior
- driver-monitoring enforcement
- AEB/FCW/FCA behavior
- CAN safety filters
- actuator-limit checks
- automatic re-engagement after cancellation
- safety-critical alert suppression

If a desired feature appears blocked by one of these restrictions, stop and document the blocker rather than bypassing it.

## Autonomous lane planning

The long-term project goal includes supervised autonomous highway lane selection.

Development order is mandatory:

1. passive recommendation logging
2. offline/replay evaluation
3. driver-confirmed recommendation mode
4. supervised autonomous execution only after indicator control and adjacent-lane sensing are validated

Do not implement silent lane changes. An active turn signal is a prerequisite to automated lateral lane-change execution.

## Radar work

Initial radar changes must remain read-only:

read -> parse -> log -> visualize

Do not connect experimental radar tracks directly to braking/acceleration without separate approval and validation.

## Testing expectations

For material vehicle behavior changes, provide:

- tests or reason tests are not possible
- replay plan/result when applicable
- exact changed files
- exact safety implications
- expected rollback commit
- route/test notes for any physical validation

## Feature harvesting

Features may be researched from openpilot, sunnypilot, FrogPilot, OPKR, StarPilot or other credible forks.

Do not copy code blindly. For every imported feature:

- record source repo/branch/commit
- confirm license compatibility
- compare with current sunnypilot implementation
- minimize carried divergence
- adapt specifically to this Sonata only where required

## Code quality

Prefer small feature flags and isolated modules over invasive modifications.
Preserve upstream style and architecture where practical.
Avoid project-specific hacks in generic upstream code when a vehicle/platform-specific extension is possible.
Document non-obvious CAN assumptions and firmware dependencies.
