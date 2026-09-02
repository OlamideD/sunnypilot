# Autonomous Lane Planner

## Project objective

Build a supervised highway lane-selection layer for the 2025 Hyundai Sonata AWD that can eventually decide when a lane change is useful, signal the maneuver through the vehicle if a safe electronic indicator command is available, verify the target lane, execute through the existing lane-change controller, and return to an appropriate cruising lane after passing.

This is not the same feature as sunnypilot Auto Lane Change. Existing ALC automates the maneuver after the driver activates the turn signal. This project adds the decision layer above it.

The planner must be developed passive-first. It must prove that its decisions are consistently sensible before it receives any authority to initiate a lane change.

## Non-negotiable constraints

1. Driver monitoring remains active.
2. The driver remains responsible and must be able to override at all times.
3. No silent autonomous lane changes. If the driver does not operate the stalk, the vehicle must produce a real external turn indication before lateral movement begins.
4. BSM occupied means the maneuver is blocked.
5. Unknown or low-confidence target-lane geometry means the maneuver is blocked.
6. Construction, ambiguous lane topology, poor visibility, or perception disagreement must bias toward staying in lane.
7. The planner must not alter panda safety, steering torque limits, steering-rate limits, brake disengagement, CANCEL behavior, AEB/FCA, or CAN safety filters.
8. No AI-generated planner or actuator change goes directly to public-road actuation.
9. Every autonomous lane decision must be reconstructable from logs.
10. Navigation requests do not override an unsafe target lane.

---

# 1. Development phases

## Phase ALP-0: Data collection

No planner output yet.

Collect normal drives containing:

- highway cruising behind slower traffic
- driver-initiated overtakes
- driver choosing not to overtake
- left and right lane changes
- BSM occupied events
- vehicles approaching from behind in adjacent lanes
- merges
- passing-lane return maneuvers
- highway exits
- lane drops where encountered naturally

For each manual lane change, preserve sufficient context before and after the maneuver for offline analysis.

## Phase ALP-1: Passive proposal engine

The planner runs but cannot initiate or request a maneuver.

Example log output:

```text
ALP PROPOSAL
route: <route-id>
timestamp: <monotonic-time>
direction: LEFT
reason: SLOW_LEAD_PASS
confidence: 0.93
current_speed: 108.2 km/h
lead_speed_est: 91.4 km/h
lead_distance: 54.1 m
target_lane_valid: true
left_bsm: false
radar_gap_confidence: unknown
navigation_need: none
result: PASSIVE_ONLY
```

The objective is to measure planner quality against what the driver actually does.

## Phase ALP-2: Recommendation mode

The UI may present a non-actuating recommendation:

```text
Lane change left recommended
Reason: slower lead
```

The driver may confirm through an approved mechanism. Initially the existing turn signal is the preferred confirmation because it exercises the proven sunnypilot ALC path.

## Phase ALP-3: Confirmed execution

Planner proposes. Driver confirms. Existing lane-change controller executes after all normal lane-change safety gates pass.

This phase validates the handoff between our planner and the current lateral lane-change state machine without autonomous initiation.

## Phase ALP-4: Automatic indicator research

Only after vehicle CAN analysis proves a legitimate, reliable, reversible way to request the Sonata's turn indicator.

First tests are non-driving or stationary where technically valid and safe, followed by controlled validation. The command must operate the real exterior indicator and cluster indication. Fake UI indication is not acceptable.

If a safe body-control path does not exist through the available vehicle networks, fully autonomous lane initiation remains blocked. We do not work around this by moving laterally without signaling.

## Phase ALP-5: Supervised autonomous highway lane selection

Allowed only after ALP-1 through ALP-4 meet their acceptance criteria.

The planner can request:

- pass slower lead
- return from pass
- establish route-required lane position

Initial operational domain should be conservative:

- divided controlled-access highway
- same-direction multilane traffic
- clear lane topology
- adequate lane confidence
- no active construction classification where detectable
- no intersection/urban-turn logic

## Phase ALP-6: Advanced highway behavior

Later work:

- merge cooperation
- lane-drop avoidance
- route/exit preparation
- pass-and-return sequences
- traffic-aware lane choice
- navigation-vs-overtake arbitration
- speed/efficiency preference

---

# 2. Planner architecture

```text
                         Model / vision
                         lane geometry
                              |
                              v
Navigation/context ---> Scene State <--- Hyundai BSM
                              ^
                              |
                    Radar / track state
                              |
                              v
                     Lane Opportunity
                         Evaluator
                              |
              +---------------+---------------+
              |                               |
           reject                         candidate
                                              |
                                              v
                                     Safety/Confidence
                                          Gate
                                              |
              +-------------------------------+------------------+
              |                                                  |
            block                                             propose
                                                                  |
                                                                  v
                                                       Maneuver Arbiter
                                                                  |
                                               +------------------+----------------+
                                               |                                   |
                                         PASSIVE/RECOMMEND                  EXECUTION GATE
                                                                                  |
                                                                    existing lane-change
                                                                          controller
```

The planner should not become a second steering controller. It decides whether and where to change lanes. Existing proven lateral-control infrastructure remains responsible for executing the lane-change trajectory.

---

# 3. Inputs

Inputs are grouped by trust level. Trust values are not static. They must be validated on real Sonata data.

## 3.1 Vehicle state

Required:

- vehicle speed
- acceleration
- steering angle/rate
- accelerator state
- brake state
- cruise state
- lateral-active state
- gear
- driver steering override
- current turn-signal state

## 3.2 Driver monitoring

Required:

- driver attentive state
- monitoring availability/validity
- recent distraction warning state

Planner execution must fail closed when driver monitoring is invalid or the driver is not attentive.

## 3.3 Current-lane perception

Candidate inputs:

- lane-line probabilities
- lane width estimate
- road edges
- model path
- curvature
- lane-change model probabilities/desires
- lead vehicle state

## 3.4 Target-lane perception

Need to derive or validate:

- target lane exists
- target lane direction matches current travel
- target lane width plausible
- target lane is not shoulder/road edge
- target lane does not terminate immediately
- lane marking boundary is sufficiently visible or topology otherwise sufficiently certain

## 3.5 Hyundai BSM

Required before autonomous execution:

- left blind spot occupied
- right blind spot occupied
- validity/freshness of BSM state if available

BSM state is a blocker, not proof that the entire target lane is clear.

## 3.6 Radar tracks

Future high-value inputs:

- object track ID
- longitudinal range
- relative longitudinal velocity
- lateral position
- track validity
- track age/persistence
- association confidence

Radar must initially be treated as experimental perception. Until validated, `radar_clear = unknown`, not `true`.

## 3.7 Navigation/map context

Future inputs:

- route-required exit
- distance to exit
- fork direction
- lane count/topology where reliable
- road class
- speed limit
- merge/lane-drop information

Navigation assists planning. It cannot make an unsafe lane safe.

---

# 4. Scene-state model

Proposed internal data model:

```text
SceneState
  timestamp
  vehicle
    speed
    accel
    lat_active
    cruise_active
    brake_pressed
    gas_pressed
    steering_override
  driver
    attentive
    dm_valid
  current_lane
    valid
    confidence
    width
    curvature
    road_edge_distance_left
    road_edge_distance_right
  left_lane
    exists
    confidence
    bsm_occupied
    front_gap
    rear_gap
    front_relative_speed
    rear_closing_speed
    radar_confidence
  right_lane
    exists
    confidence
    bsm_occupied
    front_gap
    rear_gap
    front_relative_speed
    rear_closing_speed
    radar_confidence
  lead
    valid
    distance
    relative_speed
    estimated_absolute_speed
  navigation
    valid
    next_maneuver
    distance
    required_direction
```

Fields may evolve as real logs show what is actually available and reliable.

---

# 5. Maneuver reasons

Use explicit reason codes, not free text inside control logic.

Initial enum proposal:

```text
NONE
SLOW_LEAD_PASS
RETURN_FROM_PASS
NAV_POSITION
EXIT_PREP
LANE_DROP
MERGE
TRAFFIC_BALANCE
DRIVER_REQUEST
```

Only the first three should be considered for early autonomous highway work.

---

# 6. Decision layers

## 6.1 Opportunity detector

Answers only:

> Would another lane plausibly be better than the current lane?

Examples:

- lead materially slower than preferred cruise speed
- navigation requires a different lane soon
- pass is complete and cruising lane is available

It must not initiate anything.

## 6.2 Target-lane evaluator

Answers:

> Does the target lane appear to exist and be usable?

Reject when:

- lane confidence below threshold
- shoulder/road edge ambiguity
- lane width implausible
- target lane direction uncertain
- lane ends too soon where known
- geometry inconsistent between frames

## 6.3 Occupancy/gap evaluator

Combines:

- BSM
- model perception
- validated radar tracks when available
- temporal persistence

A single-frame `clear` observation is insufficient for autonomous execution.

## 6.4 Maneuver arbiter

Selects between competing goals.

Suggested priority concept:

1. Safety block
2. Route-required positioning
3. Return from completed pass when appropriate
4. Slow-lead pass
5. Traffic/efficiency optimization

Example: do not initiate a left pass shortly before a right-side highway exit if it creates unnecessary route risk.

---

# 7. Slow-lead passing logic

No hard production thresholds are defined before real-data analysis.

Variables to study:

- difference between set/preferred speed and lead speed
- time-to-catch lead
- lead distance
- duration the slower-lead condition persists
- target-lane traffic speed
- rear closing speed
- proximity of route maneuver
- road curvature
- weather/visibility state if available

The passive planner should initially emit all relevant values so thresholds can be selected from data rather than guesses.

---

# 8. Return-from-pass logic

This is a separate maneuver reason, not simply the inverse of overtaking.

Need to establish:

- passed vehicle is sufficiently behind
- target cruising lane is valid
- target cruising lane is clear
- BSM clear
- no immediate need to remain in passing lane for another obstacle/vehicle
- no route reason to stay in current lane

Add hysteresis to avoid weaving between lanes.

The planner should prefer fewer well-justified lane changes over constant optimization.

---

# 9. Automatic indicator control

## Research questions

1. What CAN/CAN-FD messages represent physical stalk state?
2. Is that state merely an input or is there a separate BCM/ADAS request path?
3. Do HDA2 vehicles use a body-control lane-change indicator request unavailable on non-HDA2 cars?
4. Is the relevant network accessible through Hyundai A / comma four?
5. Can the actual exterior lamps be commanded without spoofing unrelated safety messages?
6. Does the cluster reflect the state correctly?
7. What happens if the driver operates the stalk while an electronic request is active?
8. Can the request be immediately cancelled?
9. Does it introduce diagnostic trouble codes or persistent faults?

## Acceptance criteria

Automatic indicator control is only considered valid if it:

- drives the actual exterior turn indicator
- produces normal vehicle indication/feedback
- is immediately cancellable
- does not interfere with hazard lamps
- does not suppress driver stalk input
- does not create persistent faults
- remains inside ordinary vehicle-network safety policy

---

# 10. Passive planner metrics

Every proposal should be evaluated against driver behavior and scene outcome.

## Precision-oriented metrics

- proposals per 100 km
- accepted by driver / total proposals
- false or undesirable proposal count
- BSM-blocked proposals
- low-confidence rejections
- navigation-conflict rejections
- target-lane ambiguity rejections
- duplicate proposal rate

## Timing metrics

- time from slow-lead detection to proposal
- proposal lead time before driver manually signals
- pass completion time
- time until return-to-lane proposal

## Quality labels

For offline review:

```text
GOOD
GOOD_BUT_EARLY
GOOD_BUT_LATE
UNNECESSARY
WRONG_DIRECTION
UNSAFE_GAP
BAD_LANE_GEOMETRY
ROUTE_CONFLICT
UNKNOWN
```

This labeling can later support threshold tuning or a learned proposal model, but initial versions should remain transparent and rule-based enough to inspect easily.

---

# 11. Execution safety gates

Before an autonomous maneuver can move from proposal to execution, require all applicable conditions:

```text
planner enabled
AND approved operational domain
AND lateral active
AND driver attentive
AND target lane valid
AND target lane confidence sufficient
AND BSM clear
AND gap evaluator clear
AND no brake input
AND no steering override
AND no recent driver cancellation
AND no system fault
AND real turn indicator active
AND existing lane-change controller ready
```

Any gate becoming false before lane-change start should cancel the initiation.

During the actual maneuver, existing sunnypilot/openpilot lane-change and driver-override behavior remains authoritative.

---

# 12. Kill switches and rollback

At minimum:

- one persistent setting to disable ALP entirely
- execution feature flag separate from passive planner flag
- automatic-indicator feature flag separate from lane execution
- easy return to `ccnc-port-prebuilt`
- stable branch does not auto-merge planner changes

Recommended configuration separation:

```text
ALPPassiveEnabled
ALPRecommendationEnabled
ALPExecutionEnabled
ALPAutoIndicatorEnabled
ALPRadarGapEnabled
ALPNavigationEnabled
```

Only `ALPPassiveEnabled` should exist in the first implementation.

---

# 13. Testing map

Relevant test IDs should be maintained in `TEST_MATRIX.md`.

Expected groups:

- ALP-P: passive planner unit/replay tests
- ALP-R: recommendation UI tests
- ALP-I: indicator research tests
- ALP-E: execution integration tests
- ALP-N: navigation arbitration tests
- ALP-RAD: radar-gap tests

Before any execution-capable PR merges to `sonata-stable`, its PR description must link the exact route replays and test IDs used for validation.

---

# 14. What success looks like

Early success is not autonomous lane changing. Early success is a passive planner that, over a large set of normal highway driving, consistently proposes the same lane changes a careful driver would choose and reliably refuses questionable ones.

Later success is a supervised highway experience where the driver can maintain attention while the system handles routine lane selection, signaling, overtaking, and return-to-lane behavior without creating unnecessary maneuvers or weakening the vehicle's existing safety systems.
