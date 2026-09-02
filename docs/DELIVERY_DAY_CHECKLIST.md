# Delivery Day Commissioning Checklist

## 2025 Hyundai Sonata AWD + comma four + Hyundai A

This checklist is the delivery-day procedure. The goal is to establish a clean, reproducible baseline before any custom Sonata code is introduced.

Initial software target:

```text
sunnypilot/ccnc-port-prebuilt
```

Initial longitudinal control:

```text
Hyundai factory Smart Cruise Control
```

Do not begin delivery day on our custom branch. A clean community baseline is essential for diagnosis.

---

# 0. Stop conditions

Stop the procedure and return to the previous known-good state if any of these occur:

- connector does not fit naturally
- bent/damaged pins
- persistent forward-camera/ADAS warning after reseating
- persistent CAN/steering/SCC fault
- vehicle fails to return to normal after comma is disengaged
- wrong or uncertain vehicle fingerprint
- unexpected longitudinal takeover
- brake/CANCEL no longer behaves normally

Never force a connector or continue road testing merely because the software appears to boot.

---

# 1. Before opening the box

- [ ] Park on level ground in a safe location.
- [ ] Vehicle in Park.
- [ ] Parking brake applied.
- [ ] Photograph stock dashboard with ignition on and no warning lights.
- [ ] Photograph steering-wheel cruise/LFA controls.
- [ ] Photograph stock mirror/camera housing.
- [ ] Confirm factory SCC works normally on a prior/initial stock drive.
- [ ] Confirm LFA/LKA works normally.
- [ ] Confirm BSM left/right works normally.
- [ ] Confirm no current forward-safety warning.

Record date/time and current odometer for the commissioning log.

---

# 2. Unbox and inventory

Photograph all components before installation.

Expected:

- [ ] comma four
- [ ] comma four windshield mount(s)
- [ ] Hyundai A car harness
- [ ] harness box / interface components supplied with car harness
- [ ] appropriate comma cable/power components from the order

Inspect for shipping damage.

Record visible hardware revisions/labels where present.

---

# 3. Expose factory camera connector

- [ ] Ignition off.
- [ ] Allow vehicle electronics to settle briefly.
- [ ] Use plastic trim tools, not metal tools where avoidable.
- [ ] Remove/loosen camera/mirror trim without pulling on wiring.
- [ ] Photograph connector before unplugging.
- [ ] Photograph connector front/keying.
- [ ] Photograph wire side.
- [ ] Photograph Hyundai A matching connector.

## Gate C1: connector verification

Proceed only if the Hyundai A connector physically/keyingly matches the factory connection.

Do not use force to overcome a mismatch.

---

# 4. Install Hyundai A harness

- [ ] Release OEM camera connector using its latch.
- [ ] Pull connector body, not wires.
- [ ] Insert OEM connector into matching Hyundai A harness socket.
- [ ] Insert Hyundai A vehicle-side connector into camera.
- [ ] Confirm retaining clips fully latch.
- [ ] Arrange harness so trim will not pinch wires.
- [ ] Temporarily leave enough access for troubleshooting.

Do not permanently close trim until the electrical sanity check passes.

---

# 5. Mount comma four

- [ ] Clean windshield with appropriate alcohol/glass-prep method.
- [ ] Locate mount high and near vehicle centerline.
- [ ] Ensure road cameras will not be blocked by factory housing/tint/dots.
- [ ] Ensure driver-facing camera can see normal driving position.
- [ ] Ensure device can still be removed from mount.
- [ ] Apply mount firmly and squarely.
- [ ] Connect comma cable fully.
- [ ] Slide comma four fully onto mount.

If uncertain about exact centering, prefer measuring/photographing before committing the adhesive.

---

# 6. First power/electrical sanity check

Start/awaken the car as required.

Observe vehicle first, software second.

Verify no persistent:

- [ ] Check Forward Safety System
- [ ] Check Lane Keeping Assist
- [ ] Check Driver Assistance System
- [ ] SCC unavailable
- [ ] EPS/steering fault
- [ ] camera fault
- [ ] abnormal warning-lamp cluster

## If persistent errors appear

1. Turn vehicle off safely.
2. Power down/disconnect comma where appropriate.
3. Reseat both harness connectors.
4. Recheck latch/keying.
5. Retry.

If the problem remains, restore OEM camera cable directly to the OEM camera.

## Gate C2: OEM rollback

- [ ] If harness/comma removed, vehicle returns to the original warning-free condition.

If not, stop installation and diagnose before proceeding.

---

# 7. Initial software installation

Connect comma four to reliable Wi-Fi/hotspot.

Select custom software and install:

```text
sunnypilot/ccnc-port-prebuilt
```

- [ ] Internet stable.
- [ ] Download completes.
- [ ] Install/build completes.
- [ ] Device reboots normally.
- [ ] No install loop.

If the known custom-build installer compatibility error appears, document the exact text before trying a workaround.

Do not randomly flash other forks as a troubleshooting strategy.

---

# 8. Vehicle identification gate

The desired platform is current ccNC's explicit platform:

```text
HYUNDAI_SONATA_2024
Hyundai Sonata (without HDA II) 2024-26
Hyundai A
```

Record:

- [ ] detected platform string
- [ ] whether fingerprint was automatic or manually forced
- [ ] software version
- [ ] branch/build name
- [ ] commit/SHA where exposed
- [ ] AGNOS/system version

## Gate C3: fingerprint

Preferred result: automatic recognition as the 2024-26 non-HDA2 Sonata platform.

If the vehicle is unrecognized:

- do not force a 2023 Sonata just to make controls available
- collect fingerprint/firmware/log information
- remain in non-actuating/dashcam behavior
- fix recognition first

---

# 9. Preserve firmware/platform information

As soon as practical, save privately:

- [ ] camera ECU firmware
- [ ] radar ECU firmware
- [ ] ADAS/forward-safety firmware returned by query
- [ ] full car fingerprint
- [ ] `CarParams`
- [ ] `CarParamsSP`
- [ ] BSM detected true/false
- [ ] CAN-FD/LFA/LKA architecture flags
- [ ] alpha-longitudinal availability flag
- [ ] radarUnavailable state
- [ ] selected DBC names
- [ ] opendbc SHA

Do not commit VIN, dongle secrets or private tokens to the public repository.

---

# 10. Driver camera and calibration

- [ ] Open driver-camera preview.
- [ ] Confirm normal seating position is visible.
- [ ] Sunglasses/night testing can wait.
- [ ] Start normal road-camera calibration.
- [ ] Do not move the mount during calibration.
- [ ] Complete enough normal driving for calibration state to become valid.

Record calibration status before first active lateral test.

---

# 11. Initial settings

Use conservative baseline settings.

| Setting | Delivery-day state |
|---|---|
| Alpha Longitudinal | OFF |
| Experimental longitudinal behavior | OFF |
| Old radar-track branch | OFF / not installed |
| MADS | OFF initially |
| Autonomous Lane Planner | nonexistent/passive only later |
| Nudgeless ALC | OFF initially |
| Timed ALC | OFF until base lateral is validated |
| Driver monitoring | default/active |
| Steering tune | default ccNC |
| Factory SCC | ON / baseline controller |

Do not maximize automation on the first drive.

---

# 12. Static/control checks before road test

With vehicle safely stationary where applicable:

- [ ] device starts/stops with expected vehicle state
- [ ] steering-wheel buttons are represented normally
- [ ] left blinker state is detected
- [ ] right blinker state is detected
- [ ] BSM state can be observed when naturally available later
- [ ] brake state is detected
- [ ] accelerator state is detected
- [ ] cruise main/set/cancel state behaves normally

Do not attempt automatic exterior-indicator CAN commands on delivery day.

---

# 13. First drive test order

Use a familiar low-complexity road in daylight/dry conditions if possible.

## T30-series: passive/stock behavior

- [ ] Drive with sunnypilot not actively steering.
- [ ] Confirm vehicle behaves normally.
- [ ] Engage factory SCC.
- [ ] Confirm following-distance behavior.
- [ ] Confirm CANCEL.
- [ ] Confirm brake disengagement.
- [ ] Confirm RESUME where normally expected.

## T40-series: basic lateral

On a suitable straight road:

- [ ] engage sunnypilot lateral
- [ ] hands immediately ready
- [ ] confirm clean engagement
- [ ] confirm reasonable center position
- [ ] confirm no oscillation
- [ ] confirm manual steering override
- [ ] confirm no EPS warning

Then gently expand to:

- [ ] mild curves
- [ ] moderate highway curves after confidence is established

Do not deliberately seek the sharpest curve to test maximum steering authority.

---

# 14. First lane-change tests

Only after basic lateral behavior is predictable.

## ALC-1: nudge

- [ ] use a clear multilane road
- [ ] manually verify mirrors/target lane
- [ ] activate indicator
- [ ] confirm system enters expected pre-lane-change state
- [ ] use required steering nudge
- [ ] verify smooth completion
- [ ] verify no repeated unintended second change

## ALC-2: timed

After ALC-1 passes:

- [ ] configure a conservative timed value, preferably 1 second initially
- [ ] activate indicator
- [ ] confirm system waits
- [ ] verify maneuver starts only under expected conditions

## ALC-3: BSM delay

Observe only during naturally occurring adjacent traffic.

- [ ] BSM occupied prevents/delays initiation
- [ ] maneuver does not start merely because timer expires while BSM remains occupied

Nudgeless/immediate mode can wait until these are boringly reliable.

---

# 15. Baseline route collection

Preserve at least:

## R-A: straight/calibration route

- [ ] stable lane centering
- [ ] manual override event

## R-B: multilane highway

- [ ] factory SCC following
- [ ] left manual/assisted lane change
- [ ] right manual/assisted lane change
- [ ] BSM event if naturally encountered

## R-C: curves

- [ ] representative gentle/moderate curves
- [ ] note any steering-limit alert

## R-D: stop-and-go

- [ ] factory SCC behavior
- [ ] no custom longitudinal actuation

## R-E: radar-rich traffic

- [ ] stable lead
- [ ] cut-in/out if naturally encountered
- [ ] adjacent-lane pass

Record route IDs and useful timestamps.

---

# 16. Delivery-day completion criteria

Commissioning is considered successful when:

- [ ] car is automatically recognized as the correct 2024-26 Sonata platform
- [ ] no persistent vehicle faults
- [ ] driver monitoring works
- [ ] calibration valid
- [ ] factory SCC works normally
- [ ] brake and CANCEL work normally
- [ ] basic lateral works predictably
- [ ] driver can immediately override steering
- [ ] BSM is visible to the stack
- [ ] at least one clean baseline route is preserved
- [ ] current software/SHA/platform information is documented
- [ ] OEM rollback has been proven conceptually/physically available

A successful Friday does not require radar-track decoding, Alpha Longitudinal, autonomous lane selection or any custom project feature.

The best delivery-day result is a boring, stable, well-recorded baseline.

---

# 17. After successful commissioning

Next development order:

1. clone/setup local development environment
2. inspect Friday logs/firmware
3. update `VEHICLE_PROFILE.md` with sanitized exact platform data
4. create permanent route manifest
5. evaluate/port live developer UI
6. begin passive radar research
7. begin passive Autonomous Lane Planner
8. enable/test MADS separately
9. investigate automatic indicator path only after CAN data is understood

No custom actuation feature is required before these observability steps.
