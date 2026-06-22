# Changelog

All notable changes to this project are recorded here.
Tagged milestones correspond to recoverable Git states.

## [Unreleased] — Phase 2 (toward v0.3)
### Added
- Plant: linear single-track bicycle model (bicycleModel.m) in a MATLAB Function
  block with external integrators; step-steer response verified (~0.31 rad/s).
- Reference: understeer-based target yaw rate with friction limit (referenceGen.m);
  steady-state 0.2972 rad/s, matches hand calculation.
- Controller: PI via native PID block, gains from cfg, saturation + anti-windup;
  first block to consume its bus inputs (RefBus + PlantBus -> error).
- Allocator: baseline moment-to-torque split (allocateBaseline.m) with config-ladder
  topology (RWD_single / RWD_twin / AWD_4); verified numerically for all three.
- DriverBus interface contract (delta) added to defineBuses.m.
- Manoeuvre generator (Manoeuvre.slx) producing step steer as a DriverBus.
- Steering routing: single source feeds both Plant and Reference via DriverIn inports;
  removed the temporary internal step-steers. Full closed loop is coherent and runs.
### Pending for v0.3
- Metric harness (metrics.m): IAE / RMSE / peak; log reference vs. actual yaw rate.

## [v0.2-skeleton-runs] — 2026-06-21
### Added
- Five Simulink.Bus interface contracts (defineBuses.m).
- vehicleConfig.m single source of truth + startup_project.m loader.
- Five modular referenced models wired into a closed-loop walking skeleton.
- YawTV_4WD.slx simulates end-to-end. Phase 1 complete.

## [v0.1-skeleton] — 2026-06-21
### Added
- MATLAB Project under Git; private GitHub repo; .gitignore/.gitattributes.
- Long-path support and model auto-merge. Phase 0 complete.
