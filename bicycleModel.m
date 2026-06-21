function [yawRate, vx_out, vy, ax, ay, beta, vy_dot, r_dot] = bicycleModel(delta, vy, r, p)
%BICYCLEMODEL  Linear single-track (bicycle) lateral dynamics.
%   Faithful port of the Year-4 thesis 2-DOF model. Inputs are the steering
%   angle and the two integrated states (vy, r); outputs are the state
%   derivatives (to be integrated) plus the measured PlantBus signals.
%
%   delta : front road-wheel steer angle [rad]
%   vy    : body lateral velocity   [m/s]  (state, integrated externally)
%   r     : yaw rate                [rad/s](state, integrated externally)
%   p     : parameter vector [m; Iz; a; b; Cf; Cr; vx]

    % ----- unpack parameters -----
    m  = p(1);  Iz = p(2);  a = p(3);  b = p(4);
    Cf = p(5);  Cr = p(6);  vx = p(7);

    % ----- slip angles (small-angle linear region) -----
    alpha_f = delta - (vy + a*r)/vx;   % front slip angle [rad]
    alpha_r =       - (vy - b*r)/vx;   % rear  slip angle [rad]

    % ----- linear lateral tyre forces -----
    Fyf = Cf * alpha_f;   % front axle lateral force [N]
    Fyr = Cr * alpha_r;   % rear  axle lateral force [N]

    % ----- equations of motion (state derivatives) -----
    vy_dot = (Fyf + Fyr)/m - vx*r;     % lateral accel in body frame
    r_dot  = (a*Fyf - b*Fyr)/Iz;       % yaw acceleration

    % ----- outputs for the PlantBus -----
    yawRate = r;
    vx_out  = vx;                      % speed held constant (lateral model)
    vy      = vy;                      % pass current lateral velocity
    ax      = 0;                       % no longitudinal dynamics yet
    ay      = vy_dot + vx*r;           % lateral acceleration (sensor sense)
    beta    = atan2(vy, vx);           % sideslip angle [rad]
end
