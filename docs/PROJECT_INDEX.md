# Sonata comma four Project Index

## Vehicle

```text
2025 Hyundai Sonata AWD
Canadian market
non-HDA II
ccNC / CAN-FD
Hyundai A harness
comma four
```

## Project intent

Create a carefully tested, best-of-breed Sonata-specific sunnypilot fork that begins with the stable ccNC community port and progressively adds stronger observability, radar perception, lane-planning intelligence, route awareness and supervised automation without weakening the existing safety envelope.

The long-term highway target is substantially more automated than ordinary blinker-triggered lane change:

```text
understand traffic/route
        |
select useful lane
        |
validate target lane
        |
BSM + perception + radar gates
        |
activate real turn signal
        |
revalidate
        |
execute lane change
        |
pass / position for route
        |
return when appropriate
```

The project intentionally does not begin at the bottom of this diagram. It earns each layer through data and testing.

---

# Core documentation

- `SONATA_PRIVATE_ROADMAP.md` — long-term goals and phases.
- `FEATURE_INTAKE.md` — detailed feature catalogue, references and risk levels.
- `BRANCH_AUDIT.md` — inherited ccNC/Sonata branch archaeology and harvest decisions.
- `VEHICLE_PROFILE.md` — sanitized facts about the exact car; complete after delivery.
- `DELIVERY_DAY_CHECKLIST.md` — Friday commissioning procedure.
- `TEST_MATRIX.md` — repeatable commissioning/regression tests.
- `DEVELOPMENT_ENVIRONMENT.md` — local Git/build/replay/Codex workflow.
- `SAFETY_GUARDRAILS.md` — non-negotiable safety boundaries.
- `AUTONOMOUS_LANE_PLANNER.md` — flagship lane-selection specification.
- `RADAR_RESEARCH.md` — passive-first radar-track program.
- `INDICATOR_CONTROL_RESEARCH.md` — electronic exterior-turn-signal research.
- root `AGENTS.md` — rules for Codex/AI agents working in this repository.

---

# Branch map

```text
upstream sunnypilot/ccnc-port
            |
            v
       sonata-base
            |
            +------------------+
            |                  |
            v                  v
       sonata-dev       sonata-radar-lab
            |
            v
      feature branches
            |
      review / replay /
     controlled testing
            |
            v
      sonata-stable
```

`sonata-base` should stay clean. `sonata-stable` should stay boring.

---

# Current backlog

## Commissioning / exact vehicle

- #1 Commission comma four on 2025 Sonata AWD
- #2 Capture exact Sonata firmware, fingerprints, and CAN architecture
- #9 Validate MADS on 2025 Sonata after baseline
- #11 Characterize Sonata longitudinal and factory AEB/FCA architecture

## Development infrastructure

- #3 Set up local build, replay, and regression environment
- #4 Harvest live developer UI from ccnc-port-live
- #12 Build Sonata route manifest, event markers, and driving metrics

## Perception and autonomy

- #5 Build passive Sonata radar-track pipeline
- #6 Build passive Autonomous Lane Planner
- #7 Research electronic turn-signal control on non-HDA2 ccNC Sonata
- #10 Research navigation and route-aware lane positioning

## Best-of-breed research

- #8 Maintain best-of-breed fork feature audit

---

# Phase gates

## Gate 0: Repository foundation

Status: substantially complete.

- dedicated fork
- clean Sonata branches
- roadmap
- safety policy
- test matrix
- feature intake
- branch audit
- radar plan
- ALP spec
- indicator research plan
- issue backlog

## Gate 1: Known-good physical baseline

Requires #1 and core parts of #2.

No project-specific actuation before this gate.

## Gate 2: Observability

Requires local replay environment, route manifests and live tooling.

This gate comes before serious feature tuning because the project must be able to measure what changed.

## Gate 3: Passive perception/planning

Radar parser and ALP can run/log without controlling the car.

## Gate 4: Existing automation validation

MADS, timed/nudgeless ALC, BSM gating and other baseline sunnypilot features validated one at a time.

## Gate 5: Recommendation-level intelligence

ALP can recommend but not independently initiate maneuvers.

## Gate 6: Automatic indicator + supervised lane selection

Requires a proven real exterior-indicator control path plus mature planner, BSM and gap validation.

## Gate 7: Route-aware highway automation

Overtake/return, exit preparation, lane-drop/merge reasoning.

## Gate 8: Longitudinal experimentation

Only after exact AEB/FCA/SCC architecture is understood and passive radar/vision work is mature.

## Gate 9: Future compute/model expansion

Chestnut/larger models/other hardware only when the current platform has measurable compute-bound limitations that justify it.

---

# Immediate next work after delivery

1. Close #1 only after clean baseline tests pass.
2. Populate exact sanitized vehicle data under #2.
3. Preserve repeatable baseline routes.
4. Set up #3 locally.
5. Port/evaluate #4 live tooling.
6. Start #12 metrics/event infrastructure.
7. Begin #5 radar and #6 ALP as passive-only efforts.

This sequence is intentional: visibility before automation, passive evidence before actuation.
