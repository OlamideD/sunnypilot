# Electronic Turn-Signal Control Research

## Why this exists

The long-term Autonomous Lane Planner goal requires the car to initiate its own real turn signal before an autonomous lane change. Current sunnypilot Auto Lane Change is blinker-triggered; it does not remove the driver's need to initiate the signal.

The project rule is strict:

> If the driver does not touch the stalk, the vehicle must still activate the real exterior turn indicator before lateral movement begins.

A comma/sunny HUD arrow alone is not a turn signal.

---

# 1. Current ccNC code findings

Current project opendbc audit SHA:

```text
0819b0e8e06119c5d5853e79de3f55bb7c4a3214
```

## Generic Hyundai CAN-FD support exists

Current Hyundai values define:

```text
HyundaiFlags.CANFD_ENABLE_BLINKERS
```

Current Hyundai interface contains logic that, when this flag is enabled, disables ECU `0x7B1` using communication-control requests.

Current Hyundai CAN-FD controller also has a real SPAS message generator:

```text
create_spas_messages(...)
```

which creates:

```text
SPAS1
SPAS2
```

and writes `SPAS2.BLINKER_CONTROL` as:

```text
0 = none
3 = left
4 = right
```

This is strong evidence that openpilot's Hyundai stack knows how to request real blinker behavior on at least some CAN-FD Hyundai/Kia/Genesis architectures.

## But the pathway is gated

The current car controller only sends these SPAS messages when both conditions are true:

```text
CANFD_LKA_STEER_MSG
AND
CANFD_ENABLE_BLINKERS
```

That is important because current ccNC explicitly defines our generation as:

```text
HYUNDAI_SONATA_2024
Hyundai Sonata (without HDA II) 2024-26
HyundaiCanFDPlatformConfig
flags = CCNC
```

The non-HDA2 ccNC Sonata is expected to use direct LFA steering rather than the LKA-steering/HDA2 path. The platform definition does not statically include `CANFD_ENABLE_BLINKERS`.

Therefore we must **not assume** that the existing SPAS implementation can be enabled on our car simply by setting a flag.

---

# 2. ccNC HUD blinker inputs are not proof of lamp control

Current non-HDA2 ccNC HUD generation accepts:

```text
CC.leftBlinker
CC.rightBlinker
```

and uses them to draw/modify:

- lane-change arrows
- LCA icons
- lane-line display state

inside `CCNC_0x161` / related HUD messages.

That means the code can visually represent a lane-change request in the cluster/ADAS UI.

It does **not** prove that those fields activate the exterior indicator lamps.

The project must keep these two concepts separate:

```text
HUD lane-change indication
!=
physical exterior turn signal
```

---

# 3. Research hypotheses

## Hypothesis A: the non-HDA2 Sonata has an accessible BCM/body request message

Best-case outcome.

We identify a message that legitimately requests left/right turn indication and is accessible through a bus exposed by the Hyundai A harness/comma four.

Then our future path could be:

```text
ALP request
  |
  v
body/indicator request
  |
  v
real exterior indicator confirmed
  |
  v
existing ALC execution gate
```

## Hypothesis B: SPAS pathway exists but requires different topology/ECU handling

The hardware may support a similar command but current generic implementation is only wired for LKA-steering/HDA2 cars.

This would require careful reverse engineering and comparison against:

- HDA2 CAN-FD platforms where SPAS control is known
- physical stalk behavior
- ECU `0x7B1`
- network routing differences between LKA- and LFA-steering cars

We must not enable the flag experimentally on a public road to see what happens.

## Hypothesis C: no safe electronic request path is available through our exposed networks

Then fully autonomous lane-change initiation remains blocked unless a safe vehicle-side integration is later discovered.

The fallback remains:

```text
planner recommends
    |
driver signals
    |
existing ALC executes
```

We do not solve this limitation by changing lanes silently.

---

# 4. Data collection plan

## IND-0: physical stalk capture

Record normal left/right signal use while logging CAN-FD traffic.

Capture separate examples of:

- left momentary/tap behavior if applicable
- left latched behavior
- right momentary/tap behavior
- right latched behavior
- cancellation by steering return
- cancellation by stalk
- hazard lamps

Do not infer signal semantics from one message/frame.

## IND-1: message correlation

Find messages/signals that correlate strongly with:

```text
left stalk input
right stalk input
left exterior lamp state
right exterior lamp state
hazard state
cluster arrow state
```

Input and output state may be different messages.

## IND-2: topology comparison

Compare our fingerprint/firmware with Hyundai CAN-FD platforms that set or use:

```text
CANFD_LKA_STEER_MSG
CANFD_ENABLE_BLINKERS
SPAS1
SPAS2
```

Determine whether ECU `0x7B1` is present on our E-CAN and what role it has.

Passive firmware identification comes before communication-control experiments.

## IND-3: stationary/lab command validation

Only if a plausible documented command path exists.

First test should be in a safe stationary context where exterior indicators can be visually confirmed.

Requirements:

- real left/right exterior lamps respond
- cluster/vehicle feedback normal
- hazards unaffected
- driver stalk overrides/cancels normally
- no persistent DTCs/warnings
- request can be stopped immediately

## IND-4: integration with ALP

Only after the physical command is proven independently.

ALP must never treat a sent CAN frame as proof that the indicator is actually operating if a feedback state can be observed.

Preferred execution gate:

```text
request indicator
      |
      v
observe/confirm active state
      |
      v
wait configured pre-maneuver interval
      |
      v
recheck BSM/gap/lane/driver
      |
      v
begin lane change
```

---

# 5. Safety/behavior requirements

Automatic indicator control must:

- activate the actual exterior lamps
- preserve normal driver stalk operation
- preserve hazard-lamp behavior
- allow immediate driver cancellation
- stop when ALP cancels a pending maneuver
- not require weakening panda safety
- not hide DTCs or suppress safety alerts
- fail toward no autonomous maneuver

If the indicator request fails, the lane change does not begin.

---

# 6. Open questions for the real car

- Is ECU `0x7B1` present/reachable on the non-HDA2 Sonata E-CAN?
- Does it identify as a parking/ADAS/SPAS-related ECU?
- Does physical stalk state live on the camera/ECAN/another body network accessible through Hyundai A?
- Is there a distinct lamp-state feedback signal?
- Are SPAS1/SPAS2 present naturally in any operating state?
- Does the Canadian AWD trim differ from other 2024-26 non-HDA2 ccNC examples?
- Can the body-control function be requested without disabling another ECU?
- Is an equivalent ccNC-specific command hidden in the 0x161/0x162 family or elsewhere, or are those strictly display/status messages?

No answer should be hard-coded until the vehicle logs support it.
