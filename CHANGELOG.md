# Changelog
All notable changes to this project are recorded here.
Tagged milestones correspond to recoverable Git states.

## [Unreleased] — Phase 3 (toward v0.4-4wd-capable)
### Added
- Step 14 — Plant: double-track four-corner model (doubleTrackModel.m). Per-corner
  slip angles and static per-corner Fz; longitudinal-force yaw term written in but
  zeroed (forward-compatible slot for step 16). PlantBus unchanged; reproduces the
  v0.3 baseline yaw rate to machine precision (0.31286831 rad/s).
- Step 15 — Plant: per-wheel saturating Magic-Formula tyre, Fy = mu*Fz*sin(C*atan(B*alpha)),
  with B derived from the validated linear cornering stiffness (small-slip continuity).
  Lateral load transfer dFz = m*ay*h/tr via a one-step-delayed ay (breaks the Fz<->ay
  algebraic loop, X0=0). Corners resolve independently: 194.6 N transfer at the test
  steer, outer wheels gaining grip headroom.
- Step 16a — PlantBus extended with per-wheel vertical loads (Fz_fl/fr/rl/rr); surfaced
  from the plant for the allocator. Bus 6->10 elements, plant 8->12 outputs,
  PlantBusCreator 6->10 inputs. Verified side-effect-free (yaw rate unchanged, Fx zero).
- Step 16 — Torque-vectoring loop CLOSED. Allocator swapped to a topology-aware
  pseudo-inverse (allocatePseudoInverse.m, unweighted); correct per-rung scaling
  (RWD_twin auto-doubles the rear differential). Plant Fx now driven by delivered
  torque (Tact/Rw); p extended to 11 (appends Rw). Powertrain made an ideal passthrough
  with one-step actuator-lag Unit Delays, breaking the closed-loop algebraic loop. Loop
  verified stable, correctly-signed, and torque demonstrably affects yaw.
### Deferred (open items going into step 17)
- PI gains untuned for closed-loop operation: authority is low at gentle steer (the
  integral winds too slowly to correct within the manoeuvre). To be tuned against the
  step-17 manoeuvre library.
- Allocator Fz-weighting: ran unweighted this step to de-risk loop closure. The per-wheel
  Fz is already on PlantBus (step 16a), ready to weight the pseudo-inverse next.

## [v0.3-baseline-loop] — 2026-06-23
### Added
- Plant: linear single-track bicycle model (bicycleModel.m); verified ~0.313 rad/s.
- Reference: understeer target with friction limit (referenceGen.m); 0.2972 rad/s.
- Controller: PI via native PID block, gains from cfg, saturation + anti-windup.
- Allocator: baseline moment-to-torque split (allocateBaseline.m) with config-ladder
  topology (RWD_single / RWD_twin / AWD_4); verified numerically.
- DriverBus interface contract (delta); Manoeuvre generator (Manoeuvre.slx).
- Single steering source feeds Plant + Reference; coherent closed loop.
- Metric harness (metrics.m): RMSE / IAE / peak yaw-rate error + steady-state checks.
- Phase 2 complete: thesis reproduced inside the modular platform.

## [v0.2-skeleton-runs] — 2026-06-21
### Added
- Five Simulink.Bus interface contracts (defineBuses.m).
- vehicleConfig.m single source of truth + startup_project.m loader.
- Five modular referenced models wired into a closed-loop walking skeleton.

## [v0.1-skeleton] — 2026-06-21
### Added
- MATLAB Project under Git; private GitHub repo; .gitignore/.gitattributes.
- Long-path support and model auto-merge. Phase 0 complete.