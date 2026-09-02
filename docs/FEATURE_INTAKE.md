# Sonata Feature Intake Catalogue

## Purpose

This is the long term intake list for the 2025 Hyundai Sonata AWD comma four project. It is intentionally broader than the current sprint. A feature can stay here for months before implementation. The objective is to avoid losing useful ideas while preventing unreviewed features from reaching the car.

The project baseline is the current `sunnypilot/ccnc-port` source. Features from other branches or forks are references, not automatic dependencies.

## Status vocabulary

- `BASELINE`: already present in the chosen ccNC/sunnypilot baseline.
- `RESEARCH`: worth studying, no implementation decision yet.
- `PLANNED`: accepted as a project goal, implementation not started.
- `PASSIVE-FIRST`: must first run without actuation.
- `LAB`: isolated experimental branch only.
- `HOLD`: useful idea but currently too risky, stale, or unnecessary.
- `PROHIBITED`: must not be introduced without explicit human approval and a separate safety review.

## Risk vocabulary

- `L0`: documentation, UI, analytics, developer tooling, no vehicle-control impact.
- `L1`: settings or observability around existing behavior.
- `L2`: lateral behavior using existing safety envelope.
- `L3`: longitudinal planning, perception fusion, autonomous maneuver decisions, or body-control commands.
- `L4`: safety limits, panda changes, steering authority increases, brake/AEB behavior, CAN safety bypasses.

---

# A. Vehicle compatibility and platform foundation

| Feature | Source/reference | Status | Risk | Project note |
|---|---|---:|---:|---|
| 2024-2026 non-HDA2 ccNC Sonata support | `sunnypilot/sunnypilot:ccnc-port` | BASELINE | L1 | Core platform source for this project. |
| comma four prebuilt baseline | `sunnypilot/sunnypilot:ccnc-port-prebuilt` | BASELINE | L1 | Delivery-day known-good community build. Do not make first commissioning drive on our custom code. |
| Hyundai A harness | ccNC Sonata compatibility guidance | BASELINE | L0 | Ordered hardware. Connector still must be visually verified before forcing any plug. |
| Automatic firmware fingerprinting | sunnypilot/opendbc Hyundai interface | PLANNED | L1 | Exact 2025 Canadian AWD camera/radar firmware will be captured after installation. |
| 2024 Sonata HEV historical port | inherited branch `sonata-hev-2024-port` | RESEARCH | L2 | Very stale relative to current ccNC. Branch is thousands of commits behind current ccNC and only directly differs through its opendbc pointer. Treat as archaeology, not a base. |

# B. Lateral control and steering

| Feature | Source/reference | Status | Risk | Project note |
|---|---|---:|---:|---|
| Neural lane centering | sunnypilot/openpilot | BASELINE | L2 | Validate straight road centering, curves, highway speed, and driver override before tuning. |
| MADS / separated lateral control | sunnypilot Hyundai CAN FD support | PLANNED | L2 | High priority after baseline. Useful for steering assistance while the driver controls speed. |
| Always On Lateral comparison | FrogPilot `Always On Lateral`; OPKR `Full time lateral control` | RESEARCH | L2 | Compare behavior and semantics with sunnypilot MADS. Prefer the least invasive implementation. |
| Desired-vs-actual steering telemetry | openpilot steering measurement tooling plus our telemetry | PLANNED | L0 | Required before any Sonata-specific lateral tuning. |
| Steering saturation detection | openpilot control state | PLANNED | L0 | Log when Hyundai EPS cannot achieve requested path. |
| Lane-center bias measurement | our route analytics | PLANNED | L0 | Quantify consistent left/right bias rather than tuning by feel. |
| Curve-specific offset research | OPKR `LeftCurv Offset`, `RightCurv Offset` | RESEARCH | L2 | Interesting only if repeatable bias is measured. No blind port. |
| Multi-lateral controller comparison | OPKR `Multi-lateral control`; sunnypilot control stack | RESEARCH | L2 | Compare algorithms only after stock ccNC performance is characterized. |
| Lane-change completion-rate tuning | OPKR `LaneChange Time(km/h: value)` | RESEARCH | L2 | Potentially useful for more natural lane-change trajectories. Requires replay and controlled road tests. |
| Increased steering torque | FrogPilot select vehicles; legacy HKG hacks | HOLD | L4 | Do not implement during normal development. Hyundai actuator limits must be treated as safety boundaries. |
| CAN-FD torque hacks | experimental HKG forks | PROHIBITED | L4 | No implementation without an explicit separate safety decision. |

# C. Existing lane-change assistance

| Feature | Source/reference | Status | Risk | Project note |
|---|---|---:|---:|---|
| Nudge lane change | sunnypilot `AutoLaneChangeController` | BASELINE | L2 | Driver signals and nudges. Baseline test first. |
| Nudgeless lane change | sunnypilot `AutoLaneChangeController` | BASELINE | L2 | Driver still signals, system begins automatically. |
| Timed lane change | sunnypilot `AutoLaneChangeController`: 0.5/1/2/3 s | BASELINE | L2 | Initial preferred production setting likely 0.5 or 1.0 s after testing. |
| BSM-aware delay | sunnypilot `AutoLaneChangeBsmDelay` | BASELINE | L2 | Uses Hyundai blind-spot state to delay initiation. Driver remains responsible for checking. |
| One change per signal sequence | sunnypilot `prev_lane_change` state | BASELINE | L2 | Confirm behavior on real Sonata stalk semantics. |
| Target lane line visibility gating | sunnypilot lane-change logic | BASELINE | L2 | Verify with the 2025 Sonata and comma four model. |
| Road-edge/curb protection | FrogPilot lane-change safety behavior | RESEARCH | L2 | Compare implementation against sunnypilot model road-edge/lane state. |
| Adjacent-lane lead awareness during maneuver | FrogPilot human-like lane-change ideas | RESEARCH | L3 | Potential input to longitudinal planning while changing lanes. Not first-wave. |

# D. Autonomous Lane Planner, flagship project goal

The final goal is lane selection without the driver initiating the turn signal. This is distinct from current blinker-triggered auto lane change.

| Feature | Source/reference | Status | Risk | Project note |
|---|---|---:|---:|---|
| Passive lane-change proposals | our `Autonomous Lane Planner` | PLANNED / PASSIVE-FIRST | L0 | First implementation must only log what it would do. |
| Slow-lead overtake detection | our planner, model lead + later radar | PLANNED | L3 | Decide when current lead is sufficiently slower to justify passing. |
| Target-lane validity | model lane lines + road edges + map context | PLANNED | L3 | Must reject shoulder, curb, lane ending, ambiguous target lane, construction uncertainty. |
| BSM gating | Hyundai native BSM | PLANNED | L3 | BSM occupied means no autonomous maneuver proposal/execution. |
| Radar gap assessment | ccNC radar tracks + our fusion | PLANNED | L3 | Use range and relative velocity where trustworthy. |
| Recommendation mode | our planner | PLANNED | L1 | System proposes lane change; driver confirms. Intermediate validation stage. |
| Driver acceptance/rejection analytics | our planner telemetry | PLANNED | L0 | Learn whether proposals match the driver's real decisions. |
| Automatic return after passing | our planner | PLANNED | L3 | Only after overtaking logic is stable. Respect keep-right behavior where applicable. |
| Merge/lane-drop response | navigation/model/map context | PLANNED | L3 | Later highway feature. |
| Route/exit lane positioning | navigation integration | PLANNED | L3 | Navigation should override unnecessary overtakes near exits. |
| Automatic indicator command | vehicle CAN/body-control research | RESEARCH / PASSIVE-FIRST | L3 | Must determine whether non-HDA2 Canadian Sonata exposes a safe electronic turn-signal request. No silent lane changes. |
| Autonomous supervised lane execution | our planner + existing ALC | PLANNED | L3 | Only after passive, recommendation, indicator, radar/BSM, and replay gates pass. |

## Autonomous Lane Planner rollout gates

1. Passive proposal only.
2. Offline precision review over many normal drives.
3. Recommendation UI, no autonomous actuation.
4. Driver-confirmed execution.
5. Automatic indicator command if the vehicle safely supports it.
6. Supervised autonomous highway lane selection.
7. Automatic overtaking and return.
8. Route-aware lane positioning and highway exits.

Do not skip gates to accelerate the project.

# E. Radar and perception

| Feature | Source/reference | Status | Risk | Project note |
|---|---|---:|---:|---|
| ccNC radar-track archaeology | inherited `ccnc-port-radar-tracks` | RESEARCH | L1 | Branch has diverged substantially from current ccNC. Harvest targeted logic, never replace the whole current stack with the old branch. |
| Sonata radar ECU identification | real car firmware query/logs | PLANNED | L0 | Required before assuming 2024 radar grouping applies to exact 2025 AWD. |
| Passive raw radar logging | CAN-FD logs | PLANNED | L0 | No actuation. |
| Radar track parser | historical ccNC radar track work + current opendbc | PLANNED | L1 | Decode object ID, distance, relative velocity, lateral position, validity, and persistence. |
| Radar visualization | our developer UI | PLANNED | L0 | Show tracks alongside model lead for validation. |
| Vision/radar lead comparison | our analytics | PLANNED | L0 | Measure disagreements and false/ghost targets before fusion. |
| Adjacent-lane radar tracking | radar tracks | RESEARCH | L3 | Potentially valuable for autonomous lane planner if track geometry is reliable. |
| Cut-in detection | radar + model | PLANNED | L3 | Later longitudinal and lane-planning feature. |
| Radar/vision fusion | current openpilot concepts + Sonata data | PLANNED | L3 | Must prove improvement offline before controlling speed. |
| Radar-directed braking | none initially | HOLD | L3/L4 | Not a first-stage objective. Hyundai stock SCC remains longitudinal baseline. |
| ESCC physical radar module | legacy non-CAN-FD HKG approaches | HOLD | L4 | Not the intended architecture for this ccNC CAN-FD Sonata. |

# F. Longitudinal control

| Feature | Source/reference | Status | Risk | Project note |
|---|---|---:|---:|---|
| Hyundai stock SCC | factory vehicle | BASELINE | L1 | Initial and early-development longitudinal controller. |
| Alpha Longitudinal | sunnypilot | RESEARCH | L3 | Do not enable until exact AEB/FCW/FCA behavior on this platform is documented. |
| Dynamic Experimental Control | sunnypilot DEC | RESEARCH | L3 | Compare against FrogPilot CEM rather than adding both. |
| Conditional Experimental Mode | FrogPilot CEM | RESEARCH | L3 | FrogPilot switches based on curves, slower/stopped leads, speed, and predicted stops. Compare architecture to DEC. |
| Human-like acceleration | FrogPilot | RESEARCH | L3 | Later tuning candidate only after our own baseline metrics. |
| Human-like braking | FrogPilot | RESEARCH | L3 | Same. Avoid subjective tuning without route metrics. |
| Custom following distance personalities | FrogPilot Driving Personalities | RESEARCH | L2/L3 | Candidate once longitudinal control is ours. |
| Stop-and-go improvements | sunnypilot/FrogPilot/OPKR ideas | RESEARCH | L3 | Stock SCC remains comparator. |
| OPKR SCC button-spam speed control | OPKR | HOLD | L3 | Clever legacy path for cars retaining stock longitudinal. Probably unnecessary unless current platform has a specific need. |
| Curve speed control | sunnypilot/FrogPilot/OPKR | PLANNED | L3 | Compare vision curve, model planned path, OSM/map-based inputs. |
| Speed-limit controller | FrogPilot SLC, OPKR OSM speed control | RESEARCH | L3 | Map accuracy and sign source confidence required. |

# G. Navigation and map intelligence

| Feature | Source/reference | Status | Risk | Project note |
|---|---|---:|---:|---|
| Navigation context service | sunnypilot future work / openpilot history | RESEARCH | L0/L1 | First expose route context to planner without actuation. |
| Offline maps | FrogPilot OSM map downloads | RESEARCH | L0 | Useful for speed limits, road classification, exits, curvature and offline resilience. |
| OSM speed limits | FrogPilot/OPKR | RESEARCH | L1 | Accuracy validation needed. |
| Learning curve speed controller | StarPilot | RESEARCH | L3 | Candidate comparator for our curve-speed work. |
| Route-aware lane selection | our planner | PLANNED | L3 | Key Tesla-like highway goal. |
| Fork/exit awareness | historical Navigate on openpilot concepts | PLANNED | L3 | Initially recommendation-only. |
| Destination-to-destination city automation | none | HOLD | L3+ | Not an early objective and limited by sensor coverage/integration. |

# H. Developer tooling and observability

| Feature | Source/reference | Status | Risk | Project note |
|---|---|---:|---:|---|
| Live data UI | inherited `ccnc-port-live`, files `openpilot/tools/live/README.md`, `ui.py` | RESEARCH / HIGH PRIORITY | L0 | Excellent candidate for early harvesting. Branch is only one feature commit relative to its base and does not alter driving behavior. |
| Cluster/map analytics | inherited `ccnc-port-experiments`, `tools/clustermaps/*` | RESEARCH / HIGH PRIORITY | L0 | Useful for route/event analysis and geographic clustering. Harvest tooling only, not branch state. |
| WGPU developer visualization | inherited `ccnc-port-wgpu`, `openpilot/tools/wgpu/*` | RESEARCH | L0/L1 | Potential PC/GPU visualization path. Branch also changes model/UI processes, so isolate tooling before adoption. |
| Custom button | inherited `ccnc-port-custom-button` | RESEARCH | L1/L3 | Useful concept, but branch also changes longitudinal planner and DEC tests. Never cherry-pick the branch wholesale. |
| High quality recording | FrogPilot/StarPilot | PLANNED | L0 | Useful for development if storage/heat tradeoff is acceptable. |
| Automatic version backups | FrogPilot | PLANNED | L0 | Strong developer convenience and rollback feature. |
| Drive event bookmarks | existing comma/sunnypilot UI concepts | PLANNED | L0 | Driver should be able to mark an event for later analysis without remembering timestamp. |
| Route manifest | our project | PLANNED | L0 | Save route ID, git SHA, model, settings, test IDs, weather/road notes. |
| Intervention detector | our project | PLANNED | L0 | Quantify steering/brake/cancel overrides. |
| Steering saturation report | our project | PLANNED | L0 | Frequency, speed, curvature, duration. |
| A/B model comparisons | model manager + replay | PLANNED | L0 | Same routes, same metrics, controlled comparison. |
| Regression replay suite | comma/openpilot replay tools | PLANNED | L0 | Required before vehicle-control PRs reach stable. |
| StarPilot desktop/cross-build workflow | StarPilot developer features | RESEARCH | L0 | StarPilot documents PC UI and cross compilation for C4. Useful reference for local dev workflow. |
| Remote configuration portal concepts | sunnylink; StarPilot Galaxy | RESEARCH | L0/L1 | Prefer sunnylink unless a missing capability justifies custom tooling. |

# I. Device and power behavior

| Feature | Source/reference | Status | Risk | Project note |
|---|---|---:|---:|---|
| Aux power management | inherited `ccnc-port-auxpowersave` | RESEARCH | L1 | Current branch is directly ahead of ccNC and adds `aux_power.py` plus hardwared integration. Evaluate after real off-road power behavior is observed. |
| comma four model fallback changes | inherited auxpower/chestnut branches | RESEARCH | L1 | These branches also carry model manager/fallback changes. Keep separate from power feature. |
| Chestnut visibility with MADS | inherited `ccnc-port-chestnut-show-with-mads` | RESEARCH | L0/L1 | Relevant only if Chestnut is later purchased. |
| Engaged-state UI refinement | inherited `ccnc-port-ui-engaged` | RESEARCH | L0 | Small C4 HUD-only delta. Nice candidate after core commissioning. |

# J. Model and compute roadmap

| Feature | Source/reference | Status | Risk | Project note |
|---|---|---:|---:|---|
| Model selector | sunnypilot/FrogPilot/StarPilot | PLANNED | L1/L2 | Compare models using repeatable route metrics rather than novelty. |
| Model manager/fallback hardening | recent ccNC feature branches | RESEARCH | L1 | Evaluate current upstream state before importing old work. |
| Chestnut external GPU | comma ecosystem | FUTURE | L1/L2 | Hardware compute expansion after current C4 integration is mature. |
| Larger driving models | comma/Chestnut ecosystem | FUTURE | L2 | Must preserve deterministic rollback and route comparisons. |

# K. Safety and features we intentionally reject

The following are not ordinary feature candidates:

- Disabling or weakening driver monitoring.
- Removing brake or CANCEL disengagement behavior.
- Suppressing safety-critical alerts so a fault appears to be gone.
- Bypassing CAN safety filters.
- Raising steering torque/rate limits merely to make a corner work.
- Automatically re-engaging after a deliberate driver cancellation without an independently reviewed design.
- Disabling factory AEB/FCA without explicitly understanding and accepting the consequence.
- Direct public-road deployment of AI-generated actuator code.
- Silent lane changes without a real turn indication.

These require explicit human approval outside normal feature intake. See `SAFETY_GUARDRAILS.md` and `AGENTS.md`.

---

# External reference inventory

## sunnypilot

- Repository: `sunnypilot/sunnypilot`
- Project base: `ccnc-port`
- Initial device build: `ccnc-port-prebuilt`
- Radar archaeology: `ccnc-port-radar-tracks`
- Useful inherited research branches: `ccnc-port-live`, `ccnc-port-experiments`, `ccnc-port-wgpu`, `ccnc-port-custom-button`, `ccnc-port-auxpowersave`, `ccnc-port-ui-engaged`, `ccnc-port-chestnut-show-with-mads`
- Auto lane change implementation: `openpilot/sunnypilot/selfdrive/controls/lib/auto_lane_change.py`

## FrogPilot

- Repository: `FrogAi/FrogPilot`
- Baseline reference branch: `FrogPilot`
- Relevant documented features: Always On Lateral, Conditional Experimental Mode, Automatic Lane Changes, Driving Model Selector, Speed Limit Controller, automatic version backups, high-quality recordings, human-like acceleration/braking, driving personalities, increased steering torque on select vehicles.
- Use as a feature source, not a replacement base.

## OPKR

- Repository: `openpilotkr/openpilot`
- Branch: `OPKR`
- HKG-specific reference features include lane-change speed/delay/time, BSM display, curve offsets, multi-lateral control, OSM speed/curve behavior, live tune, full-time lateral, and numerous legacy Hyundai-specific control techniques.
- OPKR is an older architectural generation. Extract ideas and algorithms selectively, not whole modules.

## StarPilot

- Repository: `firestar5683/StarPilot`
- Branch: `StarPilot`
- Built on FrogPilot. Relevant references include comma four support, model switching, Galaxy remote configuration, learning curve speed controller, enhanced CEM, high-quality recordings, desktop UI/cross compilation and build tooling.

---

# Intake rule

No external feature reaches `sonata-stable` because it is interesting. It reaches stable only after we can answer:

1. What problem does it solve on this exact Sonata?
2. What source commit/file inspired it?
3. What code path does it touch?
4. Does it affect perception, planning, actuation, or safety?
5. Can it be validated passively or with replay first?
6. What test IDs cover it?
7. What is the rollback commit?
8. Is it materially better than the current ccNC behavior on repeatable routes?
