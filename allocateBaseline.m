function [T_fl, T_fr, T_rl, T_rr] = allocateBaseline(MzDemand, FxDemand, p)
%ALLOCATEBASELINE  Simplest yaw-moment-to-wheel-torque allocation.
%   Converts a demanded yaw moment into a left/right differential torque,
%   split evenly front/rear. Longitudinal demand (FxDemand) is shared equally
%   across all four wheels. This is the baseline; Phase 3 replaces it with a
%   constraint-aware allocator (pseudo-inverse / optimisation).
%
%   MzDemand : yaw moment demand        [N*m]
%   FxDemand : longitudinal force demand[N]
%   p        : parameter vector [tr; Rw; actuator]   (actuator: 1=RWD_single,2=RWD_twin,3=AWD_4)

    tr  = p(1);   % track width [m]
    Rw  = p(2);   % wheel radius [m]
    act = p(3);   % actuator configuration code

    % ----- yaw moment -> differential wheel torque -----
    % A positive Mz needs more torque on the right than the left.
    % Mz ~= (T_right - T_left)/Rw * (tr/2)*2  -> dT = Mz*Rw/tr per side pair
    dT = MzDemand * Rw / tr;     % torque magnitude added right / removed left [N*m]

    % ----- longitudinal demand -> even tractive torque per wheel -----
    Tx = FxDemand * Rw / 4;      % share Fx equally across 4 wheels [N*m]

    % ----- base per-wheel torques (split differential evenly front/rear) -----
    T_fl = Tx - dT/2;   T_fr = Tx + dT/2;
    T_rl = Tx - dT/2;   T_rr = Tx + dT/2;

    % ----- apply actuator topology (zero undriven wheels) -----
    if act == 1            % RWD_single: lump rear drive, no torque vectoring
        T_fl = 0; T_fr = 0;
        T_rl = Tx; T_rr = Tx;            % no differential (single motor)
    elseif act == 2        % RWD_twin: rear hub motors, TV on rear only
        T_fl = 0; T_fr = 0;
        T_rl = Tx - dT/2; T_rr = Tx + dT/2;
    end
    % act == 3 (AWD_4): all four wheels as computed above
end
