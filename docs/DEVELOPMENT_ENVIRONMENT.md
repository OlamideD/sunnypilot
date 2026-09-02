# Development and Replay Environment

## Objective

Standardize how the Sonata fork is developed, tested, reviewed and deployed so that a change can be reproduced later and does not depend on one laptop or one comma-four filesystem state.

The development model is:

```text
GitHub fork
   |
   +--> local Linux/WSL development
   |       |
   |       +--> static/unit tests
   |       +--> replay tests
   |       +--> analytics
   |
   +--> feature branch
           |
           +--> PR / review
                   |
                   +--> sonata-dev
                           |
                           +--> controlled validation
                                   |
                                   +--> sonata-stable
```

The comma four is a test/deployment target, not the canonical place where source code lives.

---

# 1. Recommended host environment

Preferred:

- native Linux, or
- Windows 11 with WSL2 + current Ubuntu LTS.

macOS can be useful for general Git/Python work, but Linux should remain the reference environment for openpilot/sunnypilot build and replay tooling.

Suggested machine resources:

- 16 GB RAM minimum; 32 GB preferable for comfortable development
- modern x86-64 CPU
- at least 100 GB free SSD space once routes/build caches begin accumulating
- reliable broadband

A discrete GPU is helpful for some visualization/model experiments but not required for initial repository work.

---

# 2. Base packages

Install or confirm:

```text
git
git-lfs
openssh-client
curl
build-essential
python3
python3-venv
python3-pip
```

Then follow the current upstream/sunnypilot bootstrap requirements rather than pinning old package versions in this document.

Do not run random community setup scripts without reviewing what they change.

---

# 3. Git remotes

Recommended clone/remotes:

```bash
git clone git@github.com:OlamideD/sunnypilot.git
cd sunnypilot

git remote rename origin origin
git remote add upstream https://github.com/sunnypilot/sunnypilot.git

git remote -v
```

Expected logical state:

```text
origin   -> OlamideD/sunnypilot
upstream -> sunnypilot/sunnypilot
```

Fetch all refs:

```bash
git fetch origin --prune
git fetch upstream --prune
```

Never force-update `sonata-stable` merely to make it match an upstream branch.

---

# 4. SSH distinction

There are two separate SSH concepts in this project.

## GitHub SSH

Used by the development computer to clone/push Git repositories.

This can be configured before the comma arrives.

Typical validation:

```bash
ssh -T git@github.com
```

## comma-four SSH

Used to log into the physical comma device. This naturally waits until the device arrives, is registered/configured and has your GitHub public key available through the device setup.

Do not confuse the two. The project can proceed without comma-device SSH until delivery.

---

# 5. Branch discipline

## `sonata-base`

Clean current ccNC reference. No project features.

## `sonata-dev`

Integration branch for reviewed development.

## `sonata-stable`

Daily-driver candidate after validation.

## `sonata-radar-lab`

Radar/perception lab only.

## Feature branches

Create from the correct project branch, for example:

```bash
git switch sonata-dev
git pull --ff-only origin sonata-dev
git switch -c feat/live-developer-ui
```

Examples:

```text
feat/live-developer-ui
feat/route-manifest
feat/passive-autonomous-lane-planner
feat/radar-track-parser
feat/radar-visualization
feat/indicator-control-research
feat/navigation-context
fix/sonata-fingerprint
fix/highway-lateral-oscillation
```

Do not develop unrelated features on one branch merely because they are convenient to test together.

---

# 6. Codex / AI-agent workflow

The root `AGENTS.md` is authoritative for repository agents.

AI agents may be used aggressively for:

- code search
- architecture mapping
- unit tests
- route analytics
- replay tooling
- visualization
- documentation
- refactoring with unchanged semantics
- feature comparisons
- upstream merge analysis
- DBC/parser analysis

AI agents must not independently deploy or normalize changes that weaken safety boundaries.

Before asking Codex to work on a vehicle-control feature, include:

- exact branch
- exact goal
- explicit non-goals
- risk level from `FEATURE_INTAKE.md`
- relevant test IDs
- whether the requested work must be passive-only

Recommended prompt pattern:

```text
Work on feat/<name> from sonata-dev.
Read AGENTS.md, SAFETY_GUARDRAILS.md, FEATURE_INTAKE.md and the feature spec first.
Do not change vehicle actuation or panda safety unless the task explicitly requires it.
Run the relevant tests and report every changed file plus unresolved assumptions.
```

---

# 7. Upstream synchronization

Do not mix upstream synchronization with feature development when avoidable.

Recommended process:

```bash
git fetch upstream

git switch sonata-base
# update sonata-base to reviewed/current upstream ccNC state

# then create an explicit sync PR/merge path into sonata-dev
```

Before updating the daily-driving branch, record:

- previous ccNC SHA
- new ccNC SHA
- opendbc submodule change
- model/version changes
- significant Hyundai/CAN-FD changes
- test/replay results

For this project, an upstream update is itself a software change that deserves validation.

---

# 8. Build/test layers

Use the lightest validation that can disprove a change before moving to a more expensive layer.

## Layer 0: source review

- diff inspection
- formatting/lint
- type/static checks where available
- no unexpected submodule movement

## Layer 1: unit tests

Run targeted tests around changed code.

Examples:

- lane planner decision tests
- radar parser tests
- Hyundai interface tests
- settings tests

## Layer 2: process/replay tests

Replay known routes where possible.

A feature that changes planning/control should be tested against:

- a known-good straight/highway route
- at least one relevant edge case route
- a regression route for any bug it claims to fix

## Layer 3: passive vehicle test

Preferred first physical test for new perception/planning logic. Observe/log without actuation.

## Layer 4: controlled actuation test

Only after previous layers pass and the feature's risk class permits it.

## Layer 5: normal-road validation

Repeated ordinary drives, not a single successful demonstration.

---

# 9. Route manifest

Every route selected for development should have metadata similar to:

```yaml
route_id: "..."
date: "..."
vehicle: "2025 Hyundai Sonata AWD Canada"
software:
  branch: sonata-dev
  sha: "..."
  upstream_ccnc_sha: "..."
  opendbc_sha: "..."
model: "..."
settings:
  alpha_longitudinal: false
  mads: false
  auto_lane_change: "1s"
conditions:
  road: highway
  weather: dry
  lighting: daylight
markers:
  - time: "00:12:31"
    tag: manual_left_lane_change
  - time: "00:18:03"
    tag: bsm_left_occupied
notes: "..."
```

Do not commit private home-location route data to a public repository.

---

# 10. Test route privacy

The fork is currently public. Therefore:

Never commit:

- VIN in plain text
- dongle credentials/tokens
- home address
- private route URLs with access tokens
- API keys
- private SSH keys
- comma/sunnylink secrets
- exact personal trip histories unless intentionally sanitized

Use hashes/redaction where a stable identifier is needed.

If route data must be shared publicly, choose segments that do not expose home/work locations or other private patterns.

---

# 11. Device deployment policy

During initial commissioning, install community `ccnc-port-prebuilt`, not our branch.

When our fork is ready for device testing:

- deploy only a named feature/dev/stable branch
- record its SHA before driving
- keep a known-good rollback branch/install route
- do not edit source directly on the comma without immediately reproducing the change in Git
- avoid `git reset --hard` or branch switching on the device while the car is in motion

A device-local hotfix must be copied into a Git branch before it is considered part of the project.

---

# 12. Regression baseline

After Friday's commissioning, designate a small set of routes as permanent regression references:

```text
R-A: calibration / straight lateral
R-B: multilane highway / lane changes
R-C: gentle-to-moderate curves
R-D: stop-and-go / factory SCC behavior
R-E: radar-rich highway traffic
```

The route set can grow, but existing reference routes should not be discarded simply because a new build performs poorly on them.

---

# 13. Definition of done for a software feature

A feature is not done when it works once.

For normal L0-L2 changes, done generally means:

1. source identified and license compatible if harvested
2. feature branch clean
3. tests pass
4. replay/passive validation where applicable
5. no unrelated changes
6. rollback known
7. docs/test matrix updated
8. PR reviewed
9. repeated vehicle validation if it affects driving

For L3/L4 changes, additional feature-specific safety gates apply.
