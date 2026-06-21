function cfg = vehicleConfig()
%VEHICLECONFIG  Single source of truth for the vehicle and architecture.
%   cfg = vehicleConfig() returns a struct describing the car, its actuator
%   configuration (the ladder rung), and the chosen control architecture.
%   Every block reads parameters from this struct -- nothing is hard-coded
%   inside a Simulink block.
%
%   Parameters tagged *** MEASURE *** or *** ESTIMATE *** are not yet trusted
%   values and are tracked in the validation backlog (spec section 6).

    % ===== Geometry =====
    cfg.geom.L   = 1.59;       % wheelbase [m]
    cfg.geom.a   = 0.816;      % CG to front axle [m]
    cfg.geom.b   = cfg.geom.L - cfg.geom.a;  % CG to rear axle [m] (derived)
    cfg.geom.tr  = 1.23;       % track width [m] (front=rear for now)
    cfg.geom.h   = 0.30;       % CG height [m]   *** MEASURE ***
    cfg.geom.Rw  = 0.1995;     % effective wheel radius [m]

    % ===== Mass / inertia =====
    cfg.mass.m   = 255;        % total mass incl. driver [kg]
    cfg.mass.Iz  = 100;        % yaw inertia [kg*m^2]   *** ESTIMATE ***
    cfg.mass.g   = 9.81;       % gravity [m/s^2]

    % ===== Tyres =====
    cfg.tyre.model = "linear";  % "linear" | "nonlinear" (later)
    cfg.tyre.Cf  = 46000;      % front axle cornering stiffness [N/rad]
    cfg.tyre.Cr  = 50000;      % rear axle cornering stiffness [N/rad]
    cfg.tyre.mu  = 1.3;        % peak friction coefficient   *** ESTIMATE (fit from TTC) ***

    % ===== Reference generator =====
    cfg.ref.Ku    = 0.52*(pi/180)/9.81;  % understeer gradient [rad/(m/s^2)]
    cfg.ref.Crl   = 0.8;       % friction-limit conservatism on r_max [-]
    cfg.ref.vMin  = 1.0;       % min speed for reference division [m/s]

    % ===== Controller (PI baseline) =====
    cfg.ctrl.Kp     = 200;     % proportional gain
    cfg.ctrl.Ki     = 40;      % integral gain
    cfg.ctrl.MzMax  = 212.5;   % yaw-moment saturation [N*m]

    % ===== Actuator configuration (the ladder) =====
    %   "RWD_single" : team car today  (1 motor, rear axle)
    %   "RWD_twin"   : data-source car  (2 rear hub motors)
    %   "AWD_4"      : target           (4 hub motors)
    cfg.actuator = "AWD_4";

    % ===== Control architecture (which strategy to run) =====
    cfg.arch.controller = "PI";            % "PI" | "Fuzzy" | "MPC" | "SMC"
    cfg.arch.allocator  = "pseudo_inverse"; % "rule" | "pseudo_inverse" | "optimisation"
    cfg.arch.topology   = "hierarchical";   % "hierarchical" | "centralised"

    % ===== Powertrain limits (placeholder, refined in Phase 4) =====
    cfg.pwr.TmaxWheel = 250;   % max torque per wheel [N*m] (placeholder)
    cfg.pwr.tau       = 0.02;  % actuator lag time constant [s] (placeholder)
end
