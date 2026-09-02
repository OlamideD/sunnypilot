# 2025 Hyundai Sonata AWD + comma four Private Roadmap

Status: Internal project roadmap
Vehicle: 2025 Hyundai Sonata AWD, Canadian market, non-HDA II target configuration
Primary hardware: comma four + Hyundai A harness
Primary upstream: sunnypilot/sunnypilot
Baseline branch: `ccnc-port`
Daily development branch: `sonata-dev`
Stable branch: `sonata-stable`
Clean upstream mirror: `sonata-base`
Radar research branch: `sonata-radar-lab`

## Project objective

Build a disciplined, maintainable, safety-conscious Sonata-specific fork that starts from the current sunnypilot ccNC port and progressively integrates the best compatible ideas from sunnypilot, openpilot, FrogPilot, OPKR, StarPilot and other credible community work.

The end goal is not merely to add features. The goal is to create a Tesla-like supervised driving experience on the 2025 Sonata while retaining Hyundai's existing safety systems wherever possible, preserving rollback paths, maintaining upstream compatibility, and validating every material change with logs, replay, passive testing and controlled road testing.

This roadmap deliberately includes long-term items that may not be attempted for weeks or months. They are recorded here so project goals are not lost.

---

# 1. Non-negotiable engineering principles

- [ ] Keep `sonata-base` clean and aligned to upstream `ccnc-port`.
- [ ] Do daily development only on `sonata-dev` or feature branches.
- [ ] Promote only validated changes to `sonata-stable`.
- [ ] Keep radar work isolated in `sonata-radar-lab` until explicitly promoted.
- [ ] Never deploy an unreviewed Codex/AI-generated vehicle-control change directly to the car.
- [ ] Prefer passive observation before actuation for any new perception or planner feature.
- [ ] Preserve factory AEB/FCW/FCA behavior unless a specific change is understood, documented and explicitly accepted.
- [ ] Never weaken panda safety limits, driver monitoring, brake disengagement, steering-rate limits or CAN safety checks merely to make a feature work.
- [ ] Every imported feature must record source repository, source branch/commit, license, dependencies, expected benefit and validation status.
- [ ] Every road-tested build must have an exact Git commit SHA and route ID.
- [ ] Always retain a known-good rollback path to official/community baseline and fully OEM vehicle wiring.

---

# 2. Phase 0: stock vehicle and installation baseline

## 2.1 Stock vehicle documentation

- [ ] Photograph dashboard with no warnings before comma installation.
- [ ] Record exact trim, drivetrain and market.
- [ ] Confirm factory Smart Cruise Control.
- [ ] Confirm Lane Following Assist.
- [ ] Confirm Lane Keeping Assist.
- [ ] Confirm Blind Spot Monitoring.
- [ ] Confirm Forward Collision Avoidance / AEB status.
- [ ] Record infotainment software version.
- [ ] Record any Hyundai OTA update status.
- [ ] Photograph camera housing and OEM connector before harness installation.

## 2.2 Hardware installation

- [ ] Inspect comma four contents.
- [ ] Inspect Hyundai A harness.
- [ ] Verify physical connector keying before insertion.
- [ ] Install harness without forcing connectors.
- [ ] Mount comma four high and near windshield centerline.
- [ ] Verify driver-facing camera view.
- [ ] Verify all OEM ADAS systems remain fault-free after harness installation.
- [ ] Document reversible OEM wiring state.

## 2.3 Initial software baseline

Initial install target:

`Sunnypilot/ccnc-port-prebuilt`

- [ ] Install current ccNC prebuilt.
- [ ] Pair comma connect.
- [ ] Pair sunnylink.
- [ ] Record dongle ID.
- [ ] Record AGNOS version.
- [ ] Record sunnypilot build/version and commit.
- [ ] Verify automatic fingerprint.
- [ ] Record detected Hyundai platform.
- [ ] Record camera ECU firmware.
- [ ] Record radar ECU firmware.
- [ ] Record CAN-FD bus layout visible to comma.
- [ ] Calibrate camera.

Reference: upstream `sunnypilot/sunnypilot` branch `ccnc-port-prebuilt` for installable baseline and branch `ccnc-port` for source development.

---

# 3. Phase 1: known-good driving baseline

The first successful configuration should deliberately use Hyundai longitudinal control and sunnypilot lateral control.

- [ ] Factory Hyundai SCC handles acceleration/braking.
- [ ] sunnypilot handles lateral steering.
- [ ] Experimental longitudinal OFF.
- [ ] Alpha longitudinal OFF.
- [ ] Radar-track actuation OFF.
- [ ] Steering torque modifications OFF.
- [ ] Default driver monitoring ON.

## Baseline tests

- [ ] Straight-road lane centering.
- [ ] Gentle curves.
- [ ] Moderate curves.
- [ ] Steering override.
- [ ] Brake disengagement.
- [ ] CANCEL disengagement.
- [ ] RESUME behavior.
- [ ] BSM left/right visibility.
- [ ] Turn signal state visibility.
- [ ] No EPS warnings.
- [ ] No camera warnings.
- [ ] No CAN faults.
- [ ] No unexpected dashboard alerts.
- [ ] Driver monitoring works in daylight.
- [ ] Driver monitoring works at night.

Goal: accumulate enough uneventful driving that the baseline becomes boring and trusted before adding anything experimental.

---

# 4. Existing sunnypilot features we intend to use

## 4.1 Auto Lane Change by Blinker

Current sunnypilot already contains automatic lane-change execution after driver turn-signal intent.

Modes to validate:

- [ ] Nudge confirmation.
- [ ] Nudgeless.
- [ ] 0.5 second delay.
- [ ] 1 second delay.
- [ ] 2 second delay.
- [ ] 3 second delay.
- [ ] BSM delay.
- [ ] Lane-line visibility gating.
- [ ] Driver-attention gating.
- [ ] One change per lane-change sequence.

Primary source reference:

- `openpilot/sunnypilot/selfdrive/controls/lib/auto_lane_change.py`
- `docs/features/steering/auto-lane-change.md` in sunnypilot user docs

Preferred daily setting after validation: likely 0.5-1.0 second timed lane change + BSM delay.

## 4.2 MADS

Goal: allow lateral assistance independently of adaptive cruise.

- [ ] Validate MADS on CAN-FD Sonata.
- [ ] Confirm normal accelerator/brake manual behavior.
- [ ] Confirm steering remains predictable at urban speeds.
- [ ] Define preferred city-driving behavior.

## 4.3 Dynamic Experimental Control

Longer-term comparison target against FrogPilot Conditional Experimental Mode.

- [ ] Understand sunnypilot DEC decision logic.
- [ ] Compare to FrogPilot CEM.
- [ ] Determine which planner transitions are smoother and safer.
- [ ] Do not enable until longitudinal baseline is understood.

## 4.4 Driver monitoring

- [ ] Preserve stock sunnypilot/openpilot attention enforcement.
- [ ] Validate glasses/sunglasses.
- [ ] Validate night performance.
- [ ] Validate phone-distraction detection if present in current model.
- [ ] Do not weaken attention thresholds simply for convenience.

---

# 5. Radar integration roadmap

Primary research reference: upstream/community `ccnc-port-radar-tracks` and Hyundai/Kia/Genesis radar-tracks work.

The project should not begin by allowing radar changes to command braking. Radar work starts as observation only.

## 5.1 Radar inventory

- [ ] Identify exact radar ECU.
- [ ] Record firmware.
- [ ] Identify bus.
- [ ] Identify relevant CAN-FD messages.
- [ ] Identify object/track IDs.
- [ ] Determine track refresh rate.
- [ ] Decode longitudinal distance.
- [ ] Decode relative velocity.
- [ ] Decode lateral position.
- [ ] Decode object validity/status.
- [ ] Determine track persistence behavior.

Community reference previously associated with facelift Sonata: radar group `RADAR_3A5_3C4` and `HYUNDAI_SONATA_2024`. Exact compatibility must be confirmed from this vehicle's logs, not assumed.

## 5.2 Passive radar logger

- [ ] Read radar.
- [ ] Parse radar.
- [ ] Log radar.
- [ ] Visualize radar.
- [ ] Never affect vehicle actuation in first implementation.

## 5.3 Vision vs radar evaluation

- [ ] Compare vision lead vs radar lead.
- [ ] Evaluate cut-ins.
- [ ] Evaluate stopped leads.
- [ ] Evaluate adjacent-lane traffic.
- [ ] Evaluate false targets.
- [ ] Evaluate phantom braking scenarios without changing control.
- [ ] Measure target-lane traffic for lane-planner development.

## 5.4 Radar/vision fusion

- [ ] Design fusion confidence model.
- [ ] Prefer radar for range/range-rate where validated.
- [ ] Prefer vision/model for semantic/lane context.
- [ ] Track adjacent-lane vehicles.
- [ ] Track future cut-ins.
- [ ] Feed fused state to passive autonomous lane planner first.

## 5.5 Longitudinal integration

Only after extensive passive validation:

- [ ] Determine whether factory AEB remains active with sunnypilot longitudinal.
- [ ] Determine whether FCW remains active.
- [ ] Determine whether radar can improve lead control without disabling factory safety.
- [ ] Validate acceleration limits.
- [ ] Validate braking limits.
- [ ] Validate stop-and-go.
- [ ] Validate resume behavior.
- [ ] Validate cut-ins.
- [ ] Validate lead loss.
- [ ] Validate false-positive resistance.

---

# 6. Autonomous Lane Planner, long-term flagship feature

Current sunnypilot automates the maneuver after the driver signals. Our long-term goal is Tesla-like supervised highway lane selection where the system can decide that a lane change is appropriate without requiring the driver to touch the stalk.

## 6.1 Planner goals

Potential lane-change reasons:

- [ ] Overtake slower lead vehicle.
- [ ] Return from passing lane.
- [ ] Prepare for navigation exit.
- [ ] Move away from ending lane.
- [ ] Handle merge/lane-drop context.
- [ ] Position for route continuation.
- [ ] Avoid blocked/slow lane where appropriate.

## 6.2 Required inputs

- [ ] Current lane geometry.
- [ ] Adjacent lane geometry.
- [ ] Lane line confidence.
- [ ] Road-edge confidence.
- [ ] Current speed.
- [ ] Lead speed.
- [ ] Lead closing rate.
- [ ] Hyundai BSM.
- [ ] Radar target-lane traffic.
- [ ] Vision target-lane traffic.
- [ ] Driver-attention state.
- [ ] Navigation route context.
- [ ] Steering authority/curve context.
- [ ] Construction/poor-marking confidence if detectable.

## 6.3 Stage A, current behavior

Driver signals -> sunnypilot executes lane change.

- [ ] Validate thoroughly first.

## 6.4 Stage B, passive proposal mode

The planner decides what it WOULD do but never actuates.

Example logged output:

`PROPOSE LEFT | reason=slow_lead | confidence=.94 | BSM=clear | radar=clear | lane=valid`

- [ ] Implement planner output message.
- [ ] Log every recommendation.
- [ ] Compare planner recommendation to driver's actual choice.
- [ ] Build false-positive dataset.
- [ ] Build false-negative dataset.
- [ ] Tune policy offline.

## 6.5 Stage C, recommendation mode

- [ ] Display proposed lane change.
- [ ] Driver confirms using blinker or explicit control.
- [ ] Track acceptance/rejection rate.
- [ ] Learn preferred passing thresholds.

## 6.6 Stage D, supervised autonomous lane selection

Target behavior:

system decides -> system signals -> confirms BSM/radar/vision -> changes lane -> stabilizes in target lane.

- [ ] Identify whether 2025 non-HDA2 Sonata permits electronic indicator command.
- [ ] If yes, decode and reproduce legitimate indicator request safely.
- [ ] If no, do NOT allow silent automatic lane changes.
- [ ] Require turn signal to be physically/electronically active before lateral lane-change execution.
- [ ] Require driver monitoring active.
- [ ] Require sufficient lane confidence.
- [ ] Require target-lane clearance.
- [ ] Require minimum safe gap.
- [ ] Permit immediate driver cancellation.

## 6.7 Passing intelligence

Possible user-configurable policy after validation:

- [ ] Slow-lead trigger delta.
- [ ] Minimum pass speed benefit.
- [ ] Minimum target-lane lead gap.
- [ ] Minimum target-lane rear gap.
- [ ] Return-after-pass behavior.
- [ ] Keep-right preference.
- [ ] Navigation priority over overtaking.
- [ ] Conservative/Normal assertiveness profiles.

Reference inspiration: Tesla-style highway lane selection concept, FrogPilot human-like lane-change work, sunnypilot ALC infrastructure. We will implement only what can be validated on the Sonata's actual sensors and actuator interfaces.

---

# 7. Automatic turn-signal / indicator control research

This is a dependency for truly autonomous lane selection.

- [ ] Capture CAN during left blinker activation.
- [ ] Capture CAN during right blinker activation.
- [ ] Determine whether messages are simply state broadcasts or commands.
- [ ] Identify BCM involvement.
- [ ] Identify steering-column/stalk ECU involvement.
- [ ] Search HDA2/non-HDA2 Hyundai code for electronic indicator commands.
- [ ] Determine whether camera harness bus can reach required ECU.
- [ ] Determine whether panda safety allows the relevant message.
- [ ] Do not modify safety policy until command path and consequences are fully understood.
- [ ] If legitimate electronic indicator command is unavailable, keep autonomous lane-change execution disabled.

---

# 8. Navigation integration roadmap

Long-term objective: route-aware lane positioning, highway exits and eventually richer route-informed driving.

Sources to study:

- historical Navigate on openpilot work
- sunnypilot navigation proof-of-concept/future work
- FrogPilot navigation/offline maps
- StarPilot navigation work

## 8.1 Navigation data foundation

- [ ] Determine available route provider/API.
- [ ] Obtain current route polyline.
- [ ] Obtain maneuver distance.
- [ ] Obtain target road/lane direction where possible.
- [ ] Cache maps/offline data where useful.
- [ ] Keep navigation data separate from actuation initially.

## 8.2 Passive route planner

- [ ] Log lane recommendation for upcoming exits.
- [ ] Log fork preference.
- [ ] Log required lane changes.
- [ ] Compare recommendations to driver actions.

## 8.3 Route-informed lane selection

- [ ] Prepare for highway exits.
- [ ] Select correct fork.
- [ ] Avoid unnecessary passing maneuver shortly before exit.
- [ ] Combine navigation priority with autonomous lane planner.

## 8.4 Future urban routing research

- [ ] Intersection context.
- [ ] Left/right turn intent.
- [ ] Roundabout intent.
- [ ] Multi-lane approach positioning.

Urban autonomous turning is not an early project goal and requires substantially more confidence than highway lane selection.

---

# 9. FrogPilot feature-harvest candidates

Every candidate must be compared against current sunnypilot before import. Do not copy features that sunnypilot already implements better.

## High-interest

- [ ] Lane-edge/curb protection during lane change.
- [ ] Human-like lane-change behavior.
- [ ] Adjacent-lane lead tracking during lane changes.
- [ ] Conditional Experimental Mode, compare with sunnypilot DEC.
- [ ] High-quality logging/recording enhancements.
- [ ] Backup/restore conveniences.
- [ ] Offline maps/navigation pieces.
- [ ] Useful developer overlays.
- [ ] Better drive statistics.

## Research-only / high-risk

- [ ] Increased steering torque concepts.
- [ ] Hyundai/Kia/Genesis CAN-FD torque hacks.
- [ ] Any modified panda safety limits.

These are never imported merely because they provide stronger steering. Exact actuator and safety implications must be understood first.

Reference repository: FrogPilot/FrogAi community fork and releases.

---

# 10. OPKR / Hyundai-specific feature-harvest candidates

OPKR has historically contained Hyundai/Kia-specific tuning and UX ideas.

Candidates:

- [ ] Lane-change minimum-speed options.
- [ ] Lane-change delay/timing.
- [ ] Lane-change completion speed/timing.
- [ ] Hyundai-specific steering-controller ideas.
- [ ] BSM visualization.
- [ ] Hyundai button/control integrations.
- [ ] Vehicle-specific diagnostic information.

High-risk OPKR control changes must be isolated and replay-tested before road testing.

Reference repository: openpilotkr/openpilot, OPKR branch.

---

# 11. StarPilot and other fork research

- [ ] Navigation work.
- [ ] comma four compatibility improvements.
- [ ] UI ideas that remain practical on comma four's small display.
- [ ] Any novel model/planner integration.
- [ ] Compare against sunnypilot before harvesting.

Reference repository: StarPilot community fork.

---

# 12. Steering quality and lateral-control roadmap

Do not pursue maximum steering authority first. Pursue smoothness, predictability and measured accuracy.

## Measurements

- [ ] Desired vs actual steering angle/torque.
- [ ] Steering saturation events.
- [ ] Curve failure points.
- [ ] High-speed oscillation.
- [ ] Lane-center bias.
- [ ] Wind/cross-slope behavior.
- [ ] Driver interventions.

## Possible future tuning

- [ ] Lateral controller tuning specific to 2025 AWD Sonata.
- [ ] Curve-entry smoothness.
- [ ] Lane-change trajectory smoothness.
- [ ] Reduced oscillation.
- [ ] Better recovery after driver override.
- [ ] Model/controller compatibility testing.

## Explicitly restricted

- [ ] No steering-torque-limit increase without separate technical review.
- [ ] No panda safety bypass.
- [ ] No EPS spoofing beyond well-understood upstream patterns.

---

# 13. Longitudinal-control roadmap

Early daily driving remains Hyundai SCC.

Long-term research goals:

- [ ] Compare Hyundai SCC to sunnypilot longitudinal.
- [ ] Measure following smoothness.
- [ ] Measure stop-and-go smoothness.
- [ ] Measure cut-in response.
- [ ] Measure lead-loss response.
- [ ] Curve-speed behavior.
- [ ] Speed-limit behavior.
- [ ] Traffic-control behavior in experimental model.
- [ ] Radar-enhanced longitudinal planning.

Before any daily deployment:

- [ ] Factory AEB/FCW behavior documented.
- [ ] Driver override documented.
- [ ] Brake disengagement verified.
- [ ] Fault behavior verified.
- [ ] Rollback tested.

---

# 14. Speed, road-context and comfort intelligence

Possible future features:

- [ ] Curve-aware speed control.
- [ ] Speed-limit assistance.
- [ ] User-selected offset above/below detected limit.
- [ ] Smooth speed transitions.
- [ ] Construction-zone conservative profile.
- [ ] Weather-aware conservatism if reliable external/local inputs become available.
- [ ] Driver-selectable Comfort / Normal profiles.

Any automated speed control is longitudinal behavior and should follow the same validation rules.

---

# 15. Models and compute roadmap

- [ ] Record baseline driving model.
- [ ] Support model A/B comparisons.
- [ ] Record model SHA/version with every route.
- [ ] Measure interventions by model.
- [ ] Measure lane-change quality by model.
- [ ] Measure curve behavior by model.
- [ ] Measure driver-monitoring performance where applicable.

Future optional compute:

- [ ] Evaluate comma Chestnut only after normal comma-four integration is stable.
- [ ] Determine measurable benefit for Sonata use case.
- [ ] Do not add compute hardware merely for theoretical capability.

---

# 16. Telemetry, replay and engineering observability

This is an early high-priority area because it makes every later feature safer and easier to develop.

## Route metadata

- [ ] Route ID.
- [ ] Segment.
- [ ] Git commit SHA.
- [ ] Software version.
- [ ] Model version.
- [ ] Vehicle fingerprint.
- [ ] ECU firmware set.
- [ ] Relevant settings snapshot.

## Automatic event logging

- [ ] Driver intervention.
- [ ] Steering saturation.
- [ ] Disengagement reason.
- [ ] Lane-change start/end.
- [ ] BSM block.
- [ ] Planner lane-change proposal.
- [ ] Radar lead change.
- [ ] CAN fault.
- [ ] EPS fault.
- [ ] Calibration fault.

## Offline analytics

- [ ] Intervention count per 100 km.
- [ ] Engaged distance percentage.
- [ ] Lane-change acceptance rate.
- [ ] Planner false-positive rate.
- [ ] Planner false-negative rate.
- [ ] Steering saturation frequency.
- [ ] Lead cut-in performance.
- [ ] Model-to-model comparison.

## Replay

- [ ] Establish reproducible route replay environment.
- [ ] Build replay smoke test.
- [ ] Run selected regressions before merging vehicle-control changes.
- [ ] Maintain a small set of representative Sonata routes as regression fixtures where privacy permits.

---

# 17. UI and driver-experience roadmap

comma four has a small display, so avoid bloated UI.

Potential improvements:

- [ ] Clear engagement state.
- [ ] Compact blind-spot state.
- [ ] Radar/vision lead debug overlay for lab mode.
- [ ] Planner recommendation indicator.
- [ ] Reason for proposed lane change.
- [ ] Current autonomy mode.
- [ ] Current build SHA shorthand.
- [ ] Fault/rollback guidance.
- [ ] Remote configuration through sunnylink where practical.

Do not turn the driving UI into a debugging dashboard for normal daily use. Debug overlays should be toggleable.

---

# 18. Autonomy-mode concept

Potential future user-selectable modes:

1. Manual
   - Standard driver control.

2. Assist
   - Driver signals, system executes lane change.

3. Recommend
   - System proposes lane changes, driver confirms.

4. Auto Highway
   - System selects and executes validated highway lane changes with supervised attention.

5. Auto Navigation
   - Route context can request lane positioning and exits.

6. Experimental
   - Lab-only features, explicitly not daily stable.

- [ ] Define exact behavior of each mode.
- [ ] Ensure mode changes never weaken core driver-monitoring/safety constraints.

---

# 19. Safety and rollback roadmap

Every feature needs a rollback story before activation.

Known-good states:

A. OEM Sonata, comma physically removed.

B. comma four + upstream/community `ccnc-port-prebuilt`.

C. last-known-good `sonata-stable` commit.

Required procedures:

- [ ] Document software reinstall procedure.
- [ ] Document factory reflash procedure.
- [ ] Document harness removal procedure.
- [ ] Record last-known-good commit after every stable promotion.
- [ ] Tag stable field-tested releases.
- [ ] Preserve settings backups.

---

# 20. Feature intake template

Every harvested or original feature should be documented with:

- Feature name
- Problem it solves
- Source repository
- Source branch
- Source commit SHA
- License compatibility
- Files changed upstream
- Dependencies
- Sonata applicability
- Safety risk: Low / Medium / High / Critical
- Does it affect lateral actuation?
- Does it affect longitudinal actuation?
- Does it affect driver monitoring?
- Does it affect panda safety?
- Does it affect factory AEB/FCW?
- Passive test possible?
- Replay test plan
- Road test plan
- Rollback commit
- Current status

---

# 21. Proposed development priority

## Tier 0: before hardware arrives

- [ ] Repository structure.
- [ ] Private roadmap.
- [ ] Safety guardrails.
- [ ] AGENTS/Codex instructions.
- [ ] Vehicle profile template.
- [ ] Test matrix.
- [ ] Feature intake template.
- [ ] Radar research notes.
- [ ] Autonomous lane-planner specification.
- [ ] Local development environment.

## Tier 1: delivery + first week

- [ ] Physical installation.
- [ ] Vehicle fingerprint.
- [ ] Baseline ccNC prebuilt.
- [ ] Calibration.
- [ ] Factory SCC + sunnypilot lateral.
- [ ] Basic ALC.
- [ ] BSM-aware ALC.
- [ ] Collect routes.
- [ ] Establish SSH/dev access.
- [ ] Do not deploy custom actuation code.

## Tier 2: weeks 2-4 after stable baseline

- [ ] MADS.
- [ ] Telemetry improvements.
- [ ] Automated route annotations.
- [ ] Replay environment.
- [ ] Passive radar logger.
- [ ] Passive autonomous lane planner.
- [ ] Indicator CAN research.

## Tier 3: later

- [ ] Radar/vision fusion.
- [ ] Recommendation-mode autonomous lane planner.
- [ ] Navigation context.
- [ ] Improved lane-change trajectory.
- [ ] Longitudinal experiments.

## Tier 4: advanced / only after strong evidence

- [ ] Fully supervised autonomous highway lane selection.
- [ ] Automatic indicator command if safely supported.
- [ ] Automatic pass/return.
- [ ] Navigation-driven lane positioning.
- [ ] Route-aware highway exits.
- [ ] Advanced longitudinal/radar fusion.
- [ ] Larger model/Chestnut evaluation.

## Tier 5: research frontier

- [ ] Urban route-aware lane selection.
- [ ] Intersection/roundabout intent.
- [ ] Traffic-control integration improvements.
- [ ] More Tesla-like end-to-end supervised behavior where hardware permits.

---

# 22. Things explicitly out of scope until separately approved

- Disabling driver monitoring.
- Silent lane changes without an active turn signal.
- Removing brake disengagement.
- Removing panda safety constraints.
- Raising steering torque merely for convenience.
- Disabling factory safety features without a documented replacement and explicit decision.
- Deploying AI-generated actuation code without review/replay/testing.
- Using public-road testing as the first test of new control logic.

---

# 23. Immediate next repository tasks

- [ ] Add `docs/SAFETY_GUARDRAILS.md`.
- [ ] Add root `AGENTS.md` for Codex/AI instructions.
- [ ] Add `docs/VEHICLE_PROFILE.md`.
- [ ] Add `docs/TEST_MATRIX.md`.
- [ ] Add `docs/FEATURE_INTAKE.md`.
- [ ] Add `docs/RADAR_RESEARCH.md`.
- [ ] Add `docs/AUTONOMOUS_LANE_PLANNER.md`.
- [ ] Verify `sonata-base` remains identical to `ccnc-port` before development begins.
- [ ] Create first project issue for delivery-day baseline validation.

---

# 24. Reference map

Primary code/reference sources to track:

- `commaai/openpilot` for upstream openpilot architecture, safety and model/control changes.
- `sunnypilot/sunnypilot` `ccnc-port` for our primary Sonata ccNC source.
- `sunnypilot/sunnypilot` `ccnc-port-prebuilt` for initial install baseline.
- `sunnypilot/sunnypilot` `ccnc-port-radar-tracks` for experimental ccNC radar work.
- sunnypilot user documentation for MADS, lane change, longitudinal and settings behavior.
- FrogPilot repositories/releases for feature-harvest candidates.
- `openpilotkr/openpilot` OPKR branch for Hyundai/Kia-specific ideas.
- StarPilot repository for navigation and additional feature research.

The source reference does not imply automatic adoption. Every imported feature must pass the project intake process.

---

# 25. Definition of success

A successful project is not the fork with the most toggles. It is a Sonata that:

- drives smoothly and predictably,
- substantially reduces highway workload,
- can be trusted to remain within known limits,
- preserves driver supervision,
- makes intelligent use of Hyundai's existing radar and BSM,
- eventually performs supervised autonomous highway lane selection where hardware permits,
- remains easy to roll back,
- can be updated from upstream without becoming unmaintainable,
- and has enough telemetry that we can prove whether a change made the car better or worse.
