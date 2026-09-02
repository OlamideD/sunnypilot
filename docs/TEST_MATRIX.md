# Sonata Test Matrix

Use PASS / FAIL / NOT TESTED / NOT APPLICABLE.
Record route ID, segment, commit SHA, model version and settings for all road tests.

## Pre-install OEM baseline

| ID | Test | Status | Notes |
|---|---|---|---|
| T00 | No dashboard ADAS warnings | NOT TESTED | |
| T01 | Smart Cruise Control works | NOT TESTED | |
| T02 | Lane Following Assist works | NOT TESTED | |
| T03 | Lane Keeping Assist works | NOT TESTED | |
| T04 | Blind Spot Monitoring left/right works | NOT TESTED | |
| T05 | No existing camera/radar faults | NOT TESTED | |

## Hardware installation

| ID | Test | Status | Notes |
|---|---|---|---|
| T10 | Hyundai A connectors match without force | NOT TESTED | |
| T11 | OEM camera reconnects through harness | NOT TESTED | |
| T12 | comma four powers normally | NOT TESTED | |
| T13 | No new persistent dashboard faults | NOT TESTED | |
| T14 | Driver camera view clear | NOT TESTED | |
| T15 | Road cameras unobstructed | NOT TESTED | |

## Software / fingerprint

| ID | Test | Status | Notes |
|---|---|---|---|
| T20 | `ccnc-port-prebuilt` installs | NOT TESTED | |
| T21 | Vehicle automatically fingerprints | NOT TESTED | |
| T22 | Expected Sonata ccNC platform identified | NOT TESTED | |
| T23 | Camera calibration completes | NOT TESTED | |
| T24 | comma connect pairing works | NOT TESTED | |
| T25 | sunnylink pairing works | NOT TESTED | |
| T26 | SSH keys/dev access configured | NOT TESTED | |

## Baseline driving, factory longitudinal + sunnypilot lateral

| ID | Test | Status | Notes |
|---|---|---|---|
| T30 | Drive normally with sunnypilot disengaged | NOT TESTED | |
| T31 | Factory SCC set speed | NOT TESTED | |
| T32 | Factory SCC following distance | NOT TESTED | |
| T33 | Factory SCC stop-and-go | NOT TESTED | |
| T34 | Brake disengages/cancels as expected | NOT TESTED | |
| T35 | CANCEL behavior correct | NOT TESTED | |
| T36 | RESUME behavior correct | NOT TESTED | |
| T37 | sunnypilot lateral engages cleanly | NOT TESTED | |
| T38 | Straight-road lane centering | NOT TESTED | |
| T39 | Gentle curve tracking | NOT TESTED | |
| T40 | Moderate curve tracking | NOT TESTED | |
| T41 | Manual steering override | NOT TESTED | |
| T42 | No high-speed oscillation | NOT TESTED | |
| T43 | No EPS fault during engagement | NOT TESTED | |
| T44 | Driver monitoring daylight | NOT TESTED | |
| T45 | Driver monitoring night | NOT TESTED | |

## Lane change

| ID | Test | Status | Notes |
|---|---|---|---|
| T50 | Left blinker detected | NOT TESTED | |
| T51 | Right blinker detected | NOT TESTED | |
| T52 | Left BSM detected | NOT TESTED | |
| T53 | Right BSM detected | NOT TESTED | |
| T54 | Nudge lane change | NOT TESTED | |
| T55 | Timed 1-second lane change | NOT TESTED | |
| T56 | BSM delays lane change | NOT TESTED | |
| T57 | Lane change cancels safely | NOT TESTED | |
| T58 | One lane change per sequence | NOT TESTED | |
| T59 | Nudgeless mode | NOT TESTED | Test only after previous lane-change tests pass |

## MADS

| ID | Test | Status | Notes |
|---|---|---|---|
| T60 | Lateral active without SCC | NOT TESTED | |
| T61 | Manual accelerator interaction | NOT TESTED | |
| T62 | Manual brake interaction | NOT TESTED | |
| T63 | Urban low-speed behavior | NOT TESTED | |

## Radar research, passive only

| ID | Test | Status | Notes |
|---|---|---|---|
| T70 | Radar ECU identified | NOT TESTED | |
| T71 | Radar firmware captured | NOT TESTED | |
| T72 | Radar message group identified | NOT TESTED | |
| T73 | Object tracks parse | NOT TESTED | |
| T74 | Relative distance plausible | NOT TESTED | |
| T75 | Relative velocity plausible | NOT TESTED | |
| T76 | Lateral position plausible | NOT TESTED | |
| T77 | Track persistence characterized | NOT TESTED | |
| T78 | Vision/radar lead comparison | NOT TESTED | |
| T79 | No actuation from radar research code | NOT TESTED | Required |

## Autonomous lane planner, passive stages

| ID | Test | Status | Notes |
|---|---|---|---|
| T80 | Passive planner generates no actuation | NOT TESTED | Required |
| T81 | Slow-lead left proposal | NOT TESTED | |
| T82 | No proposal when BSM occupied | NOT TESTED | |
| T83 | No proposal when target lane invalid | NOT TESTED | |
| T84 | Return-from-pass proposal | NOT TESTED | |
| T85 | Recommendation acceptance/rejection logged | NOT TESTED | |
| T86 | False positives reviewed offline | NOT TESTED | |
| T87 | False negatives reviewed offline | NOT TESTED | |

## Indicator control research

| ID | Test | Status | Notes |
|---|---|---|---|
| T90 | Left stalk CAN capture | NOT TESTED | |
| T91 | Right stalk CAN capture | NOT TESTED | |
| T92 | BCM/stalk message path identified | NOT TESTED | |
| T93 | Determine state-vs-command semantics | NOT TESTED | |
| T94 | Electronic indicator command possible | NOT TESTED | Do not assume |
| T95 | Indicator command activates real exterior lamps | NOT TESTED | Controlled test only |
| T96 | Immediate cancellation works | NOT TESTED | |

## Longitudinal experimental tests, locked until later review

| ID | Test | Status | Notes |
|---|---|---|---|
| T100 | Alpha longitudinal technical review | NOT TESTED | Prerequisite |
| T101 | Factory AEB/FCW behavior documented | NOT TESTED | Prerequisite |
| T102 | Controlled acceleration behavior | NOT TESTED | |
| T103 | Controlled braking behavior | NOT TESTED | |
| T104 | Stop-and-go | NOT TESTED | |
| T105 | Cut-in handling | NOT TESTED | |
| T106 | Lead loss | NOT TESTED | |

## Regression metadata template

For every meaningful test session record:

- Date/time:
- Git branch:
- Git SHA:
- Software version:
- Model/version:
- Route ID:
- Segment(s):
- Vehicle fingerprint:
- Relevant settings:
- Weather/road notes:
- Driver interventions:
- Dashboard warnings:
- Result summary:
