function [T_fl, T_fr, T_rl, T_rr] = allocatePseudoInverse(MzDemand, FxDemand, p)
%ALLOCATEPSEUDOINVERSE  Pseudo-inverse control allocation - step 16 (unweighted).
%==========================================================================
% PURPOSE
%   Map a vehicle-level demand [Mz; Fx] to four wheel torques using a
%   least-effort pseudo-inverse over the DRIVEN wheels. This closes the
%   torque-vectoring loop: for the first time the controller's demand
%   actually produces wheel torques that move the plant.
%
%   This is the UNWEIGHTED form (equal cost per wheel). The grip-weighted
%   form - leaning torque onto the wheels with more vertical load, using the
%   per-wheel Fz now carried on PlantBus (step 16a) - is the NEXT step. The
%   weighting slot is marked below so it drops in without restructuring.
%
% EFFECTIVENESS MATRIX
%   Each wheel torque becomes a contact force Fx = T/Rw. The plant makes yaw
%   from the left/right Fx difference and net Fx from the sum:
%       Mz = (tr/2)/Rw * (-T_fl + T_fr - T_rl + T_rr)
%       Fx = (1/Rw)    * ( T_fl + T_fr + T_rl + T_rr)
%   => [Mz; Fx] = B*T. B is TOPOLOGY-AWARE via the drive mask: undriven
%   wheels are removed, so the inverse self-scales per rung (rear-only drive
%   automatically needs twice the per-wheel differential for the same Mz).
%
%   MzDemand : yaw moment demand               [N*m]
%   FxDemand : total longitudinal force demand [N]
%   p        : [tr; Rw; actuator]   actuator: 1=RWD_single,2=RWD_twin,3=AWD_4
%==========================================================================
    tr  = p(1);   % track width  [m]
    Rw  = p(2);   % wheel radius [m]
    act = p(3);   % actuator configuration code

    d = [MzDemand; FxDemand];      % demand vector [Mz; Fx]

% ---- per-wheel drive mask from topology --------------------------------
%   1 = driven, 0 = undriven. RWD configs drive the rear pair only.
    if act == 3                    % AWD_4
        mask = [1 1 1 1];
    else                           % RWD_single / RWD_twin: rear axle only
        mask = [0 0 1 1];
    end

% ---- effectiveness matrix (per-wheel columns) --------------------------
%   Row 1 (Mz): -,+,-,+ for fl,fr,rl,rr (right wheels make +yaw).
%   Row 2 (Fx): all + (each wheel pushes forward).
    kMz = (tr/2)/Rw;
    kFx = 1/Rw;
    Bfull = [ -kMz, +kMz, -kMz, +kMz; ...
              +kFx, +kFx, +kFx, +kFx ];

% ---- per-wheel cost weights --------------------------------------------
%   UNWEIGHTED this step: equal cost on every driven wheel. Undriven wheels
%   get effectively infinite cost so they are never used.
%   *** NEXT STEP (grip-weighting): replace the driven-wheel cost of 1 with
%       1/Fz_i (from PlantBus), so loaded wheels take more torque. The four
%       Fz inputs return to the signature then. ***
    cost = ones(1,4);
    cost(mask == 0) = 1e9;         % undriven -> infinite cost

% ---- weighted-least-effort pseudo-inverse: T = Winv B' (B Winv B')^-1 d --
    Winv = diag(1 ./ cost);        % 4x4; = identity on driven wheels now
    M    = Bfull * Winv * Bfull';   % 2x2
    T4   = Winv * Bfull' * (M \ d); % 4x1 wheel torques

% ---- enforce topology exactly (zero undriven wheels) -------------------
    T4 = T4 .* mask(:);

    T_fl = T4(1);  T_fr = T4(2);  T_rl = T4(3);  T_rr = T4(4);
end