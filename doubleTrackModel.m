function [yawRate, vx_out, vy, ax, ay, beta, vy_dot, r_dot, Fz_fl, Fz_fr, Fz_rl, Fz_rr] = doubleTrackModel(delta, vy, r, p, ay_prev, Tact_fl, Tact_fr, Tact_rl, Tact_rr)
%DOUBLETRACKMODEL  Double-track (four-corner) lateral plant - Steps 14-16.
%==========================================================================
% PURPOSE
%   Four-corner lateral vehicle model. Each corner has its own slip angle,
%   its own vertical load (static + lateral transfer), and its own tyre
%   force from a saturating Magic-Formula curve. The per-corner vertical
%   loads are now SURFACED as outputs (step 16) so the allocator can weight
%   torque by available grip - closing the loop deferred since step 14.
%
% PORT CONTRACT:
%   IN : delta, vy, r, p, ay_prev   (as steps 14-15)
%   OUT: yawRate vx_out vy ax ay beta vy_dot r_dot   (PlantBus state + derivs)
%        Fz_fl Fz_fr Fz_rl Fz_rr     (per-corner vertical loads -> PlantBus,
%                                      NEW in step 16, consumed by allocator)
%
% HISTORY
%   Step 14: linear tyres, static Fz, Fx yaw slot zeroed.
%   Step 15: Magic-Formula saturating tyre + lateral load transfer.
%   Step 16: per-corner Fz surfaced to the bus (this change). Fx still zero
%            here until the plant Fx wiring of sub-step 16c.
%==========================================================================

% ---- UNPACK PARAMETER VECTOR -------------------------------------------
%   p = [m; Iz; a; b; Cf; Cr; vx; tr; h; mu]
    m   = p(1);   Iz  = p(2);   a = p(3);   b = p(4);
    Cf  = p(5);   Cr  = p(6);   vx = p(7);  tr = p(8);
    h   = p(9);   mu  = p(10);  g  = 9.81;
    Rw  = p(11);  % wheel radius [m] - converts delivered torque to Fx
% ---- PER-CORNER LINEAR CORNERING STIFFNESS (half the axle each) --------
    C_corner_f = Cf/2;   C_corner_r = Cr/2;
% ---- PER-CORNER SLIP ANGLES (small-angle linear region) ----------------
    alpha_f = delta - (vy + a*r)/vx;
    alpha_r =       - (vy - b*r)/vx;
% ---- STATIC PER-CORNER VERTICAL LOADS ----------------------------------
    L    = a + b;
    Fz_f_static = m*g*(b/L)/2;   Fz_r_static = m*g*(a/L)/2;
% ---- LATERAL LOAD TRANSFER (delayed ay breaks the Fz<->ay loop) ---------
    dFz_f = m*ay_prev*h/tr / 2;   dFz_r = m*ay_prev*h/tr / 2;
%   Right (outer in +ay) gains load, left (inner) loses; floor at zero.
    Fz_fl = max(Fz_f_static - dFz_f, 0);   Fz_fr = max(Fz_f_static + dFz_f, 0);
    Fz_rl = max(Fz_r_static - dFz_r, 0);   Fz_rr = max(Fz_r_static + dFz_r, 0);
% ---- MAGIC-FORMULA SATURATING TYRE per corner --------------------------
    C_shape = 1.30;
    Fy_fl = mf_tyre(alpha_f, Fz_fl, C_corner_f, C_shape, mu);
    Fy_fr = mf_tyre(alpha_f, Fz_fr, C_corner_f, C_shape, mu);
    Fy_rl = mf_tyre(alpha_r, Fz_rl, C_corner_r, C_shape, mu);
    Fy_rr = mf_tyre(alpha_r, Fz_rr, C_corner_r, C_shape, mu);
% ---- LONGITUDINAL FORCES: still ZEROED (wired live in sub-step 16c) -----
    Fx_fl = Tact_fl/Rw;  Fx_fr = Tact_fr/Rw;  Fx_rl = Tact_rl/Rw;  Fx_rr = Tact_rr/Rw;
% ---- EQUATIONS OF MOTION -----------------------------------------------
    Fy_tot = Fy_fl + Fy_fr + Fy_rl + Fy_rr;
    Mz = a*(Fy_fl + Fy_fr) - b*(Fy_rl + Fy_rr) ...
       + (tr/2)*((Fx_fr + Fx_rr) - (Fx_fl + Fx_rl));   % long. term now LIVE (torque vectoring)
    vy_dot = Fy_tot/m - vx*r;
    r_dot  = Mz / Iz;
% ---- OUTPUTS (PlantBus state + derivatives + per-corner Fz) -------------
    yawRate = r;
    vx_out  = vx;
    vy      = vy;
    ax      = 0;
    ay      = vy_dot + vx*r;
    beta    = atan2(vy, vx);
%   Fz_fl..rr already computed above; returned as outputs 9-12 -> PlantBus.
end

% ========================================================================
function Fy = mf_tyre(alpha, Fz, C_corner, C_shape, mu)
%MF_TYRE  Simplified Magic-Formula lateral force; B derived so the origin
%   slope equals the linear per-corner stiffness. Zero for a no-load wheel.
    if Fz <= 0
        Fy = 0;
        return;
    end
    B  = C_corner / (C_shape*mu*Fz);
    Fy = mu*Fz*sin(C_shape*atan(B*alpha));
end