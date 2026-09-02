# Sonata Safety Guardrails

These rules apply to every human or AI contributor working on this fork.

## Core rule

No convenience feature is important enough to justify weakening a safety mechanism without a separate, explicit engineering review.

## Changes prohibited without explicit human approval and dedicated review

- panda safety limits
- steering torque limits
- steering-rate limits
- actuator command bounds
- brake disengagement
- CANCEL disengagement
- driver monitoring enforcement
- CAN safety filters
- AEB/FCW/FCA behavior
- longitudinal acceleration/braking limits
- automatic re-engagement after driver cancellation
- silent lane changes without an active turn signal
- suppression of safety-critical alerts

## AI/Codex rules

AI-generated control changes must never be deployed directly to `sonata-stable` or a public-road test.

Required order:

1. feature branch
2. code review
3. static/unit tests
4. replay where possible
5. passive/no-actuation validation where possible
6. controlled road validation
7. multiple uneventful normal routes
8. promotion through `sonata-dev`
9. explicit approval before `sonata-stable`

## Baseline preservation

Always preserve these rollback states:

- OEM Sonata with comma removed
- upstream/community `ccnc-port-prebuilt`
- last-known-good `sonata-stable`

## Per-feature review questions

Every feature affecting vehicle control must answer:

- What actuator can this influence?
- What is the maximum commanded authority?
- What driver action immediately overrides it?
- What happens when CAN messages disappear?
- What happens when a sensor becomes invalid?
- What factory safety feature changes behavior?
- Can the feature be tested passively first?
- Can it be replay-tested?
- What exact commit is the rollback target?

## Autonomous lane planner rule

The planner may recommend a lane change before it is permitted to execute one.

Autonomous execution requires all of the following to be validated on this exact vehicle:

- target lane validity
- adjacent-lane clearance
- Hyundai BSM input
- radar/vision checks where available
- driver attention
- immediate cancellation path
- active turn signal
- legal electronic indicator command if the driver does not operate the stalk

If electronic indicator control cannot be safely implemented, the planner remains recommendation/driver-confirmation only.

## Radar rule

Radar integration begins read-only:

read -> parse -> log -> visualize

No radar-derived control command is introduced until track quality and failure modes are characterized from real Sonata routes.

## Steering rule

The project optimizes steering smoothness and accuracy before steering authority. Increased torque is not an early project objective.
