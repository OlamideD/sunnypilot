# 2025 Sonata Radar Research Plan

## Objective

Understand and safely expose the 2025 Canadian Hyundai Sonata AWD's native front-radar data to our development stack, first for observability and later as one input to lane planning and, only if justified, longitudinal perception.

The first radar milestone is not braking control. It is a trustworthy passive data pipeline:

```text
vehicle radar CAN-FD
        |
        v
raw frames
        |
        v
track decoder
        |
        v
validated object tracks
        |
        +--> developer visualization
        +--> route/replay analytics
        +--> vision comparison
        +--> future lane planner
```

Hyundai factory SCC remains the longitudinal baseline during early work.

---

# 1. Current code-state findings

## Current ccNC base

Project source branch: `sunnypilot/sunnypilot:ccnc-port`.

At the current project audit, `ccnc-port` points its `opendbc_repo` submodule at:

```text
sunnypilot/opendbc
0819b0e8e06119c5d5853e79de3f55bb7c4a3214
```

The current Hyundai radar interface is comparatively conservative. It primarily supports a classic Mando-style family beginning at `0x500`, 32 messages, when a radar DBC is associated with the platform. It also exposes sunnypilot's radar-interface extension path when the stock parser is unavailable.

## Historical ccNC radar branch

Research branch: `sunnypilot/sunnypilot:ccnc-port-radar-tracks`.

That branch is substantially diverged from current ccNC and must not be used wholesale as our current daily-driving base.

Its historical `opendbc_repo` points at:

```text
sunnypilot/opendbc
c662942dcbe18b85c726af363b86e5fceb82b283
```

The historical Hyundai `radar_interface.py` is much more ambitious than current ccNC. It defines and can discover multiple radar/object families on buses 0, 1 and 2:

```text
RADAR_500_53F   start 0x500, up to 64 messages
RADAR_210_21F   start 0x210, 16 messages
RADAR_235_248   start 0x235, 20 messages, camera-object source
RADAR_3A5_3C4   start 0x3A5, 32 messages
RADAR_602_617   start 0x602, 16 messages
```

Of particular interest for the ccNC-generation Sonata is `RADAR_3A5_3C4`, which the historical parser models as 24-byte messages at 20 Hz with:

```text
STATE
MOTION_STATE
AGE
LONG_DIST
LAT_DIST
REL_SPEED
REL_ACCEL
```

The branch contains auto-detection logic that looks for complete expected address ranges and message sizes in the CAN fingerprint before creating parsers. This design is valuable because it avoids assuming that every Hyundai platform uses the same radar family.

## Important conclusion

The radar branch contains reusable ideas and likely reusable decoding work, but the correct integration strategy is:

```text
current ccNC + current opendbc
             |
             + selected radar-track concepts/DBC work
             |
             v
       sonata-radar-lab
```

not:

```text
replace current ccNC with the old radar branch
```

---

# 2. Exact Sonata platform state

Current ccNC opendbc explicitly defines:

```text
HYUNDAI_SONATA_2024
Hyundai Sonata (without HDA II) 2024-26
Harness: Hyundai A
Platform type: HyundaiCanFDPlatformConfig
Static flag: CCNC
```

This is encouraging because our exact generation is no longer merely an inferred community cousin of the 2024 car. The current platform definition explicitly covers 2024-26 non-HDA II Sonata.

However, the exact 2025 Canadian AWD firmware and radar traffic still need to be captured from the real car. A model-year platform declaration does not prove that every trim/market radar ECU emits the same object family.

---

# 3. Research phases

## RAD-0: Inventory and baseline

Before enabling any radar-specific experimental mode:

Capture:

- car fingerprint
- exact detected platform
- camera ECU firmware
- radar ECU firmware
- other ADAS/forward safety ECU firmware exposed by firmware query
- bus layout detected by `CanBus`
- whether stock `CarParams.radarUnavailable` is true or false
- whether a radar DBC is selected for `Bus.radar`
- BSM availability
- SCC source architecture on this car

Expected output is a vehicle-specific radar profile committed as data/documentation, not control code.

## RAD-1: Passive raw CAN capture

Collect representative normal-driving routes with no radar-actuation changes.

Routes should naturally include:

- highway following
- stationary and moving leads
- stop-and-go traffic
- vehicles cutting in/out
- adjacent-lane traffic
- overtakes
- BSM occupied events

Do not create dangerous scenarios to generate data.

Search the recorded buses for candidate families, particularly:

```text
0x3A5-0x3C4
0x500+
0x210+
0x235+
0x602+
```

but do not limit discovery to these historical ranges.

## RAD-2: Decoder validation

For every candidate object family, establish:

- message length
- bus
- frequency/cadence
- sequence behavior
- valid/invalid state enum
- object lifetime/age
- longitudinal distance
- lateral distance or azimuth
- relative longitudinal velocity
- relative lateral velocity if available
- acceleration if available
- object ID semantics if available

A signal is not considered validated merely because its numerical range looks plausible.

Use temporal/physical tests such as:

- lead distance should shrink while closing
- relative speed sign should match closing/opening behavior
- lateral position should move appropriately during adjacent-lane passes
- object age should increase/persist coherently
- track should disappear or invalidate when target is gone

## RAD-3: Passive track service

Create a normalized developer-only track object, for example:

```text
SonataRadarTrack
  source_family
  source_bus
  source_address
  track_id
  valid
  age
  d_rel_m
  y_rel_m
  v_rel_mps
  a_rel_mps2
  motion_state
  timestamp_ns
  confidence
```

This service must not feed vehicle control at first.

## RAD-4: Visualization

Add a developer visualization showing:

- ego lane/path
- current model lead(s)
- raw radar tracks
- selected radar candidate lead
- BSM state
- object ID/source family

The purpose is to make parser mistakes visually obvious.

## RAD-5: Vision-versus-radar comparison

Measure:

- lead range difference
- lead relative-speed difference
- target switching
- ghost/false tracks
- missed vision leads
- missed radar leads
- cut-in timing
- stopped-object handling
- track persistence

Do not fuse until we understand disagreement modes.

## RAD-6: Adjacent-lane gap research

Only after lateral-position data is trustworthy, test whether radar tracks can materially improve the Autonomous Lane Planner.

Potential derived fields:

```text
left_front_gap
left_rear_gap
left_rear_closing_speed
right_front_gap
right_rear_gap
right_rear_closing_speed
```

These values require lane association. Raw `yRel` alone is not sufficient on curves.

BSM remains a hard blocker for autonomous lane execution even if radar says clear.

## RAD-7: Perception fusion

Candidate fusion inputs:

- model lead
- model lane topology
- radar tracks
- ego motion
- BSM

The fusion output should have explicit confidence and source attribution.

First consumers should be analytics and passive lane planning.

## RAD-8: Longitudinal evaluation

Only after the passive pipeline is mature should we evaluate whether radar/vision fusion improves longitudinal behavior enough to justify enabling sunnypilot longitudinal control.

Before any such test, separately document:

- factory FCA/AEB behavior with stock SCC
- what is retained/lost under Alpha Longitudinal for this exact platform
- radar/ADAS ECU disable behavior
- diagnostic faults
- emergency disengagement behavior

---

# 4. Current-vs-historical parser comparison

## Current ccNC radar interface

The current parser:

- expects `RADAR_START_ADDR = 0x500`
- expects 32 track messages when a radar DBC is available
- decodes `STATE`, `AZIMUTH`, `LONG_DIST`, `REL_SPEED`
- constructs `dRel`, `yRel`, `vRel`
- otherwise falls back to sunnypilot radar-extension behavior

## Historical radar-track branch

The experimental parser adds:

- multiple radar families
- multiple buses
- automatic family discovery by fingerprint/address length
- partial/full-range handling
- motion state
- age
- lateral distance on relevant families
- relative acceleration
- multiple track-prefix formats
- run-time parser discovery
- selectable radar modes in sunnypilot extensions

## Harvest policy

Potentially harvest:

- `HyundaiRadarTrackSpec` abstraction
- address-family discovery logic
- normalized decoder functions
- family-specific DBC definitions
- passive parser/test fixtures

Do not blindly harvest:

- old platform fingerprints
- old safety behavior
- old longitudinal assumptions
- old submodule state
- branch-wide process/model/UI changes

Every selected radar patch must be rebased conceptually against the current `0819b0e8...` opendbc state and current ccNC branch.

---

# 5. Friday collection checklist

After the comma is commissioned on known-good `ccnc-port-prebuilt`:

1. Save detected platform string.
2. Save dongle ID privately.
3. Save software/version SHA.
4. Save `CarParams` and `CarParamsSP` where accessible.
5. Save firmware query output.
6. Confirm BSM state is seen correctly.
7. Confirm factory SCC remains functional.
8. Record a normal highway-follow route.
9. Record a normal multilane route with naturally occurring left/right lane changes.
10. Preserve route IDs and timestamps for:
   - a stable lead
   - a vehicle cutting in
   - a vehicle leaving the lane
   - left BSM occupied
   - right BSM occupied
   - an overtake
11. Do not enable old radar branches on the road merely to gather the data.

---

# 6. Success criteria for passive radar support

Before radar data is allowed to influence the lane planner, require:

- correct object sign/direction semantics
- stable track lifetimes
- physically plausible range and relative speed
- known bus/frequency
- low false-track rate on representative routes
- no CAN validity regressions
- replay tests
- no effect on stock SCC/AEB behavior
- documented failure modes

Before radar is allowed to influence longitudinal actuation, require a separate acceptance gate.

---

# 7. Open questions

- Which candidate radar family does the 2025 Canadian AWD emit?
- Does the radar publish tracks by default or require a diagnostic enable request?
- Are useful tracks present on a bus exposed by Hyundai A/comma four without extra hardware?
- Does ccNC camera/object data at `0x235-0x248` provide useful supplementary perception?
- Can adjacent-lane objects be associated robustly enough for autonomous gap assessment?
- Does platform firmware differ materially from 2024 community captures?
- What exact AEB/FCA architecture remains active if Alpha Longitudinal is later enabled?

These are data-collection questions, not assumptions to encode before delivery.
