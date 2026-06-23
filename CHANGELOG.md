# Changelog

All notable changes to this project are recorded here.
Tagged milestones correspond to recoverable Git states.

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
- Phase 2 complete: thesis reproduced inside the modular platform. FIRST REAL SUMMIT.

## [v0.2-skeleton-runs] — 2026-06-21
### Added
- Five Simulink.Bus interface contracts (defineBuses.m).
- vehicleConfig.m single source of truth + startup_project.m loader.
- Five modular referenced models wired into a closed-loop walking skeleton.

## [v0.1-skeleton] — 2026-06-21
### Added
- MATLAB Project under Git; private GitHub repo; .gitignore/.gitattributes.
- Long-path support and model auto-merge. Phase 0 complete.
2026-06-23  Phase 3 step 14: double-track four-corner plant (per-corner slip + static Fz, zeroed Fx slot for step 16); PlantBus unchanged, reproduces v0.3 baseline (0.31286831 rad/s).
