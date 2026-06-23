function [yawRate, vx_out, vy, ax, ay, beta, vy_dot, r_dot] = doubleTrackModel(delta, vy, r, p, ay_prev)
%DOUBLETRACKMODEL  Double-track (four-corner) lateral plant - Steps 14-15.
%==========================================================================
% PURPOSE
%   Four-corner lateral vehicle model. Each corner has its own slip angle,
%   its own vertical load (static + lateral transfer), and its own tyre
%   force from a saturating Magic-Formula curve. This is the fidelity that
%   lets left and right wheels behave differently - the prerequisite for any
%   torque-vectoring study (spec sec 1, "no per-wheel tyre behaviour").
%
% PORT CONTRACT (unchanged outputs - PlantBus identical to v0.3/Step 14):
%   IN : delta   front road-wheel steer angle           [rad]
%        vy      body lateral velocity (state)          [m/s]
%        r       yaw rate (state)                        [rad/s]
%        p       parameter vector (see UNPACK below)
%        ay_prev lateral accel from PREVIOUS time step   [m/s^2]
%                (delayed to break the Fz<->ay algebraic loop - see
%                 LOAD TRANSFER note. Deliberate design choice: error is
%                 O(step*d(ay)/dt), exactly zero in steady state, and
%                 negligible at fixed-step ms solver rates. Do NOT replace
%                 with an implicit solve unless adding fast suspension
%                 dynamics in a future higher-fidelity plant.)
%   OUT: yawRate vx_out vy ax ay beta vy_dot r_dot  (PlantBus + derivatives)
%
% HISTORY
%   Step 14: linear tyres Fy=C*alpha, static Fz, Fx yaw slot zeroed.
%   Step 15: Magic-Formula saturating tyre + lateral load transfer (this).
%==========================================================================

% ---- UNPACK PARAMETER VECTOR -------------------------------------------
%   p = [m; Iz; a; b; Cf; Cr; vx; tr; h; mu]
%   Slots 1-8 fixed in Steps 0-14; slots 9-10 APPENDED for Step 15 so that
%   no existing index shifts (same forward-compatible pattern as tr in 14).
    m   = p(1);   % vehicle mass                                   [kg]
    Iz  = p(2);   % yaw inertia about CG vertical axis             [kg m^2]
    a   = p(3);   % CG -> FRONT axle longitudinal distance         [m]
    b   = p(4);   % CG -> REAR  axle longitudinal distance         [m]
    Cf  = p(5);   % FRONT AXLE linear cornering stiffness          [N/rad]
    Cr  = p(6);   % REAR  AXLE linear cornering stiffness          [N/rad]
    vx  = p(7);   % forward speed (held constant, lateral model)   [m/s]
    tr  = p(8);   % track width (left-right wheel separation)      [m]
    h   = p(9);   % CG height above ground (ESTIMATE - to measure) [m]
    mu  = p(10);  % tyre-road friction coefficient                 [-]
    g   = 9.81;   % gravitational acceleration                     [m/s^2]
    L   = a + b;  % wheelbase                                      [m]

% ---- PER-CORNER LINEAR CORNERING STIFFNESS -----------------------------
%   Each axle stiffness is shared equally by its two tyres, so one corner
%   carries half the axle value. These set the LOW-SLIP slope of the tyre
%   curve below, preserving continuity with the Step-14 linear model.
    C_corner_f = Cf/2;   % per front corner [N/rad]
    C_corner_r = Cr/2;   % per rear  corner [N/rad]

% ---- STATIC PER-CORNER VERTICAL LOADS ----------------------------------
%   Weight on an axle is set by the OPPOSITE lever arm over the wheelbase
%   (front axle carries more when CG is forward, i.e. uses CG->REAR dist b).
%   Divided by 2 for the two corners of that axle. No transfer yet here.
    Fz_f_static = m*g*(b/L)/2;   % static load, each FRONT corner [N]
    Fz_r_static = m*g*(a/L)/2;   % static load, each REAR  corner [N]

% ---- LATERAL LOAD TRANSFER (Step 15) -----------------------------------
%   Cornering throws load onto the OUTER tyres and unloads the INNER ones.
%   Quasi-static transfer magnitude per axle (no suspension/roll split yet):
%        dFz = m * ay * h / tr        [N]   (total left<->right shift)
%   Reference: Lund dissertation eq 2.18/2.19 (the m*ay*h/(2*tr) per-corner
%   form); ETH/AMZ slide 20 (steady-state weight transfer about CG).
%   We use ay_prev (previous step) to avoid an algebraic loop, since the
%   transfer changes Fz, which changes tyre force, which changes ay.
%   Split equally front/rear for now (no roll-stiffness distribution yet -
%   that is a future higher-fidelity upgrade per spec sec 8).
%   SIGN CONVENTION: positive ay (left turn, accel points +y / leftward)
%   loads the RIGHT (outer) wheels. With our axis (vy,+y left), a left turn
%   has ay>0, outer = right. So right corners GAIN, left corners LOSE.
    dFz_f = m*ay_prev*h/tr / 2;   % half the axle transfer -> per corner [N]
    dFz_r = m*ay_prev*h/tr / 2;   % (front/rear split equally for now)   [N]

%   Apply transfer: right (outer in +ay) gains, left (inner) loses.
%   max(.,0) floors load at zero - a lifted inner wheel carries no load,
%   it cannot pull NEGATIVE load (which would invert the tyre force sign).
    Fz_fl = max(Fz_f_static - dFz_f, 0);   % front-left  [N]
    Fz_fr = max(Fz_f_static + dFz_f, 0);   % front-right [N]
    Fz_rl = max(Fz_r_static - dFz_r, 0);   % rear-left   [N]
    Fz_rr = max(Fz_r_static + dFz_r, 0);   % rear-right  [N]

% ---- PER-CORNER SLIP ANGLES --------------------------------------------
%   Small-angle linear-region slip estimate (vx in denominator, as Step 14)
%   so that at low slip the four corners still sum to the single-track
%   result. Front corners are steered by delta; rear corners are not.
%   (Left/right share an axle slip angle here; per-corner kinematic
%    differences from yaw*half-track are second-order and deferred.)
    alpha_f = delta - (vy + a*r)/vx;   % both front corners [rad]
    alpha_r =       - (vy - b*r)/vx;   % both rear  corners [rad]

% ---- MAGIC-FORMULA SATURATING TYRE (Step 15) ---------------------------
%   Simplified Pacejka lateral form, per corner:
%        Fy = mu * Fz * sin( C_shape * atan( B * alpha ) )
%   - Saturates at mu*Fz (the friction-circle lateral limit) as alpha grows.
%   - C_shape is the Pacejka shape factor; ~1.3 for lateral tyre behaviour
%     (standard value; controls how the curve falls past the peak).
%   - B (stiffness factor) is DERIVED, not guessed, so the curve slope at
%     alpha=0 equals the validated linear stiffness C_corner:
%        d(Fy)/d(alpha)|_0 = B*C_shape*mu*Fz  ==  C_corner
%        =>  B = C_corner / (C_shape*mu*Fz)
%     This guarantees small-slip behaviour matches Step 14 (continuity
%     check) while large-slip saturates - the new discriminating physics.
%   B uses the per-corner Fz (load-dependent), so a more heavily loaded
%   outer tyre is both stiffer AND has a higher force ceiling - the grip
%   asymmetry a good allocator will later exploit.
    C_shape = 1.30;   % Pacejka lateral shape factor [-]

%   Guard Fz=0 (lifted wheel) to avoid divide-by-zero in B; a zero-load
%   tyre makes zero force regardless, so set Fy=0 directly in that case.
    Fy_fl = mf_tyre(alpha_f, Fz_fl, C_corner_f, C_shape, mu);
    Fy_fr = mf_tyre(alpha_f, Fz_fr, C_corner_f, C_shape, mu);
    Fy_rl = mf_tyre(alpha_r, Fz_rl, C_corner_r, C_shape, mu);
    Fy_rr = mf_tyre(alpha_r, Fz_rr, C_corner_r, C_shape, mu);

% ---- LONGITUDINAL FORCES: still ZEROED (torque consumed at Step 16) ----
    Fx_fl = 0; Fx_fr = 0; Fx_rl = 0; Fx_rr = 0;

% ---- EQUATIONS OF MOTION -----------------------------------------------
    Fy_tot = Fy_fl + Fy_fr + Fy_rl + Fy_rr;   % total lateral force [N]
%   Yaw moment = front lever * front lateral - rear lever * rear lateral
%   + differential-longitudinal term (==0 now; the torque-vectoring slot).
    Mz = a*(Fy_fl + Fy_fr) - b*(Fy_rl + Fy_rr) ...
       + (tr/2)*((Fx_fr + Fx_rr) - (Fx_fl + Fx_rl));   % long. term == 0

    vy_dot = Fy_tot/m - vx*r;   % body-frame lateral accel eqn [m/s^2]
    r_dot  = Mz / Iz;           % yaw angular accel             [rad/s^2]

% ---- OUTPUTS (PlantBus contract identical to v0.3) ---------------------
    yawRate = r;
    vx_out  = vx;
    vy      = vy;
    ax      = 0;                % no longitudinal dynamics yet
    ay      = vy_dot + vx*r;    % inertial lateral accel (sensor sense)
%   NOTE: this ay feeds NEXT step's load transfer via the canvas unit delay.
    beta    = atan2(vy, vx);    % body sideslip angle [rad]
end

% ========================================================================
function Fy = mf_tyre(alpha, Fz, C_corner, C_shape, mu)
%MF_TYRE  Simplified Magic-Formula lateral tyre force for one corner.
%   Fy = mu*Fz*sin(C_shape*atan(B*alpha)), with B derived so the origin
%   slope equals the linear per-corner stiffness C_corner (continuity with
%   the Step-14 linear model). Returns 0 for a zero/negative-load wheel.
    if Fz <= 0
        Fy = 0;
        return;
    end
    B  = C_corner / (C_shape*mu*Fz);   % stiffness factor [1/rad], load-dep.
    Fy = mu*Fz*sin(C_shape*atan(B*alpha));
end