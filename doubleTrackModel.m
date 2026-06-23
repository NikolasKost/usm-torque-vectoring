function [yawRate, vx_out, vy, ax, ay, beta, vy_dot, r_dot] = doubleTrackModel(delta, vy, r, p)
%DOUBLETRACKMODEL  Double-track (four-corner) lateral dynamics - Step 14.
%   Drop-in replacement for bicycleModel: identical port contract (same
%   inputs, same 8 outputs, same parameter-vector style). Resolves four
%   corners with independent slip angles and static per-corner Fz. Linear
%   tyres (Fy = C*alpha) this step; saturation is Step 15. The longitudinal
%   yaw term is written in but ZEROED (torque not yet consumed) so Step 16
%   drops in without a plant edit. Reproduces bicycleModel.m exactly now.
%
%   p : [m; Iz; a; b; Cf; Cr; vx; tr]   (tr appended at slot 8)
    m  = p(1);  Iz = p(2);  a = p(3);  b = p(4);
    Cf = p(5);  Cr = p(6);  vx = p(7);  tr = p(8);
    g  = 9.81;
% per-corner cornering stiffness (split each axle across 2 tyres)
    C_fl = Cf/2;  C_fr = Cf/2;  C_rl = Cr/2;  C_rr = Cr/2;
% per-corner slip angles (vx in denominator -> sums back to single-track)
    alpha_fl = delta - (vy + a*r)/vx;
    alpha_fr = delta - (vy + a*r)/vx;
    alpha_rl =       - (vy - b*r)/vx;
    alpha_rr =       - (vy - b*r)/vx;
% linear lateral tyre forces per corner
    Fy_fl = C_fl*alpha_fl;  Fy_fr = C_fr*alpha_fr;
    Fy_rl = C_rl*alpha_rl;  Fy_rr = C_rr*alpha_rr;
% static per-corner vertical loads (NO transfer yet: Step 15) - INTERNAL
    L    = a + b;
    Fz_f = m*g*(b/L)/2;  Fz_r = m*g*(a/L)/2;
    Fz_fl = Fz_f;  Fz_fr = Fz_f;  Fz_rl = Fz_r;  Fz_rr = Fz_r;   %#ok<NASGU>
% longitudinal forces: ZEROED this step (torque not yet consumed)
    Fx_fl = 0;  Fx_fr = 0;  Fx_rl = 0;  Fx_rr = 0;
% equations of motion
    Fy_tot = Fy_fl + Fy_fr + Fy_rl + Fy_rr;
    Mz = a*(Fy_fl + Fy_fr) - b*(Fy_rl + Fy_rr) ...
       + (tr/2)*((Fx_fr + Fx_rr) - (Fx_fl + Fx_rl));   % long. term == 0 now
    vy_dot = Fy_tot/m - vx*r;
    r_dot  = Mz / Iz;
% outputs (identical PlantBus contract to v0.3)
    yawRate = r;
    vx_out  = vx;
    vy      = vy;
    ax      = 0;
    ay      = vy_dot + vx*r;
    beta    = atan2(vy, vx);
end