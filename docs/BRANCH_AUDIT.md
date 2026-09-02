# Inherited Branch Audit

## Purpose

The fork contains many sunnypilot development branches in addition to our Sonata branches. This document records what each branch appears to contain and whether it should be treated as a source, a research reference, or something to avoid.

The objective is to prevent two common failures:

1. forgetting useful work that already exists; and
2. switching the car to an old experimental branch merely because one interesting feature lives there.

Our source of truth remains current `ccnc-port`. The audit is an intake map, not an endorsement of every branch.

---

# Classification

- `BASE`: current source of truth.
- `HARVEST`: contains a focused feature worth porting selectively.
- `RESEARCH`: useful design/history but not clean enough to cherry-pick wholesale.
- `FUTURE`: relevant only after another dependency/hardware stage.
- `HOLD`: not useful for current goals or too invasive/stale.

---

# Branches

## `ccnc-port`

**Class:** BASE

Current ccNC source and upstream tracking base for this project.

Project relationship:

```text
ccnc-port
   |
   +--> sonata-base
           |
           +--> sonata-dev
           +--> sonata-radar-lab
           +--> eventually sonata-stable
```

`sonata-base` was verified identical to `ccnc-port` when project setup began.

## `ccnc-port-prebuilt`

**Class:** BASE / DEPLOYMENT

Generated/prebuilt community distribution used for delivery-day commissioning and known-good rollback.

Do not use it as the source branch for custom feature development.

## `sonata-base`

**Class:** PROJECT BASE

Must remain a clean mirror/reference point for ccNC. Do not add Sonata custom features here.

## `sonata-dev`

**Class:** PROJECT DEVELOPMENT

Documentation, integration and approved development land here after feature-level review/testing. It is not automatically the daily-driving branch.

## `sonata-stable`

**Class:** PROJECT PRODUCTION

Eventually contains only changes validated through the test/replay process. No direct experimental work.

## `sonata-radar-lab`

**Class:** PROJECT LAB

Dedicated radar and perception research. Radar changes do not merge to stable merely because they parse successfully.

---

# Inherited Sonata/history branches

## `sonata-hev-2024-port`

**Class:** RESEARCH / ARCHAEOLOGY

Finding:

- extremely stale relative to current ccNC
- branch history is thousands of commits behind current ccNC
- direct branch delta is primarily an old opendbc submodule pointer

Use:

- investigate old firmware/platform assumptions if a specific 2024 Sonata issue appears
- compare historical fingerprints if useful

Do not:

- base the 2025 project on it
- merge it into current ccNC

Current ccNC opendbc already explicitly defines `HYUNDAI_SONATA_2024` as `Hyundai Sonata (without HDA II) 2024-26` using Hyundai A, making this old branch much less important for basic platform support.

---

# Radar branches

## `ccnc-port-radar-tracks`

**Class:** HARVEST / RESEARCH

High-value historical radar work, but heavily diverged from current ccNC.

Historical opendbc SHA:

```text
c662942dcbe18b85c726af363b86e5fceb82b283
```

Current ccNC opendbc SHA at audit:

```text
0819b0e8e06119c5d5853e79de3f55bb7c4a3214
```

High-value radar concepts include:

- multiple radar/object families
- automatic address-family discovery
- multi-bus detection
- family-specific message sizes/cadence
- state/motion/age parsing
- `LONG_DIST`, `LAT_DIST`, `REL_SPEED`, `REL_ACCEL`
- dynamic parser discovery

Most interesting candidate family for ccNC research:

```text
RADAR_3A5_3C4
0x3A5-0x3C4
32 x 24-byte messages
20 Hz
```

Use:

- reimplement/port selected parser abstractions against current opendbc
- harvest DBC knowledge/tests

Do not:

- install the whole old branch as our current daily-driving stack
- inherit old platform/safety/longitudinal assumptions blindly

See `RADAR_RESEARCH.md`.

---

# Developer tooling branches

## `ccnc-port-live`

**Class:** HARVEST, HIGH PRIORITY

A focused developer-tooling branch that adds live tooling rather than changing the driving controller.

Notable files inherited from the branch audit:

```text
openpilot/tools/live/README.md
openpilot/tools/live/ui.py
```

Why it matters:

- useful for live state inspection while commissioning
- low control risk
- ideal early feature to study/port into our dev workflow

Plan:

Create a dedicated feature branch later and port only the live tooling after checking compatibility with current ccNC.

## `ccnc-port-experiments`

**Class:** HARVEST / RESEARCH

Adds developer/route-analysis tooling including:

```text
openpilot/tools/clustermaps/README.md
openpilot/tools/clustermaps/clusters.py
openpilot/tools/clustermaps/navigator.py
openpilot/tools/clustermaps/replay.py
openpilot/tools/live/README.md
openpilot/tools/live/ui.py
```

Why it matters:

- geographic clustering of route events could be useful for repeatable steering/intervention analysis
- replay helpers may accelerate regression investigation

Plan:

Harvest tooling individually after local environment is established.

## `ccnc-port-wgpu`

**Class:** RESEARCH

Contains WGPU-based tooling/visualization but also touches model/process/UI behavior.

Potentially useful:

- PC/GPU visualization
- developer scene rendering

Caution:

Do not cherry-pick the branch wholesale. Separate standalone tools from runtime/model changes first.

---

# Device/power branches

## `ccnc-port-auxpowersave`

**Class:** HARVEST LATER

Relatively current branch with power-management work. Notable addition:

```text
openpilot/common/hardware/aux_power.py
```

and hardwared integration, plus model-manager related changes.

Potential value:

- parked/off-road power behavior
- battery protection/developer availability tradeoff

Policy:

Do not import before observing real comma-four power behavior on this Sonata. When evaluated, isolate the power feature from unrelated model changes.

## `ccnc-port-chestnut-show-with-mads`

**Class:** FUTURE

Relevant if Chestnut is purchased later. Contains MADS/Chestnut display/model-manager related changes.

No need to port now.

## `ccnc-port-ui-engaged`

**Class:** HARVEST LATER

Small comma-four engaged-state HUD/UI refinement.

Useful cosmetic/UX candidate after core driving baseline is mature.

---

# Control/experimental branches

## `ccnc-port-custom-button`

**Class:** RESEARCH

Interesting custom-button concept, but branch also touches longitudinal planner, Dynamic Experimental Control and tests/UI.

Use:

- study the interaction/button architecture if we later want a safe custom control for planner recommendation/confirmation

Do not:

- cherry-pick whole branch just to gain a button

## `ccnc-port-testing-damp`

**Class:** HOLD / RESEARCH

Touches a broad set of hardware/process/spec/runtime files. Not a clean Sonata feature branch.

Only investigate if a specific damping/steering question requires it.

## `ccnc-port-lite`

**Class:** HOLD

Broadly removes/reduces functionality and diverges from our development goals. Not a project base.

## `ccnc-port-alert-dedup`

**Class:** RESEARCH LATER

Potential alert/UX cleanup. Low strategic priority until baseline alerts are observed in the real car.

## `ccnc-port-buckets`

**Class:** RESEARCH

Do not adopt until a concrete problem maps to it. Branch name alone is not sufficient reason to port anything.

## `ccnc-port-sync-20260902`

**Class:** RESEARCH / UPSTREAM SYNC HISTORY

Treat as sync/history branch. Do not base custom development on it unless comparing upstream synchronization changes.

---

# Indicator-control code audit

Current opendbc contains a generic CAN-FD blinker-control pathway:

- flag: `HyundaiFlags.CANFD_ENABLE_BLINKERS`
- interface logic can disable ECU `0x7B1` when the flag is active
- controller can send `SPAS1`/`SPAS2`
- `SPAS2.BLINKER_CONTROL` values are used for left/right requests

However, current execution code only sends those SPAS messages when:

```text
CANFD_LKA_STEER_MSG
AND
CANFD_ENABLE_BLINKERS
```

Current `HYUNDAI_SONATA_2024` static platform flags show `CCNC` only, and our non-HDA2 architecture is expected to use direct LFA steering rather than the HDA2/LKA-steering path.

Therefore:

**There is no current evidence that the existing generic SPAS blinker command is directly enabled for our non-HDA2 Sonata.**

This is useful because it narrows the research question. The codebase already knows how some CAN-FD Hyundais can command real blinkers, but we must determine whether the 2025 non-HDA2 Sonata has an equivalent accessible request path.

Also note that ccNC HUD creation accepts `left_blinker`/`right_blinker` to draw lane-change arrows/icons. That is not proof of exterior lamp actuation.

This distinction is mandatory in Autonomous Lane Planner research.

---

# Harvest order

Recommended order after delivery-day baseline:

1. `ccnc-port-live` developer UI
2. route manifest/event-bookmark tooling
3. `ccnc-port-experiments` replay/clustermap tooling where useful
4. passive radar-track parser work
5. passive Autonomous Lane Planner
6. indicator-control research
7. lane-planner recommendation mode
8. radar-assisted gap evaluation
9. navigation context
10. longitudinal experimentation
11. UI/Chestnut/power refinements as needed

The order deliberately prioritizes visibility and testing infrastructure before increased automation.
