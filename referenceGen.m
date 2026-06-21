function [yawRateRef, vxRef] = referenceGen(delta, vx, p)
%REFERENCEGEN  Understeer-based reference yaw rate with friction limit.
%   Faithful port of the Year-4 thesis reference model. Produces the target
%   yaw rate a neutral driver expects for the given steer and speed, capped
%   at what tyre friction can physically deliver.
%
%   delta : front road-wheel steer angle [rad]
%   vx    : longitudinal speed           [m/s]
%   p     : parameter vector [L; Ku; mu; g; Crl; vMin]

    % ----- unpack parameters -----
    L    = p(1);   Ku  = p(2);   mu = p(3);
    g    = p(4);   Crl = p(5);   vMin = p(6);

    % guard against divide-by-zero at very low speed
    vxs = max(vx, vMin);

    % ----- kinematic / understeer target yaw rate -----
    r_target = vxs * delta / (L + Ku * vxs^2);

    % ----- friction-limited ceiling -----
    r_max = Crl * mu * g / vxs;

    % ----- clamp target to +/- r_max -----
    yawRateRef = max(min(r_target, r_max), -r_max);

    % ----- pass speed straight through as the speed reference -----
    vxRef = vx;
end
