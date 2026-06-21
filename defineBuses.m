function defineBuses()
%DEFINEBUSES  Create the Simulink.Bus interface contracts for the platform.
%   Run this to populate the base workspace with the bus objects that define
%   every block-to-block interface in the control architecture. These buses
%   ARE the interface contracts: any block honouring a bus can be swapped
%   without disturbing its neighbours.
%
%   Boundaries (see spec section 4.1):
%     Reference  -> Controller   : RefBus
%     Controller -> Allocator    : CtrlBus
%     Allocator  -> Powertrain   : AllocBus
%     Powertrain -> Plant        : PwrBus
%     Plant      -> Sensors/Ctrl : PlantBus

% ----- helper: build one bus from a list of {name, description} pairs -----
    makeBus = @(elemSpecs) localBuildBus(elemSpecs);

    % ----- 1. RefBus : Reference -> Controller (targets to chase) -----
    RefBus = makeBus({ ...
        'yawRateRef', 'Target yaw rate [rad/s]'; ...
        'vxRef',      'Target longitudinal speed [m/s]'});

    % ----- 2. CtrlBus : Controller -> Allocator (vehicle-level demand) -----
    CtrlBus = makeBus({ ...
        'MzDemand', 'Yaw moment demand [N*m]'; ...
        'FxDemand', 'Total longitudinal force demand [N]'});

    % ----- 3. AllocBus : Allocator -> Powertrain (per-wheel requests) -----
    AllocBus = makeBus({ ...
        'T_fl', 'Front-left wheel torque request [N*m]'; ...
        'T_fr', 'Front-right wheel torque request [N*m]'; ...
        'T_rl', 'Rear-left wheel torque request [N*m]'; ...
        'T_rr', 'Rear-right wheel torque request [N*m]'});

    % ----- 4. PwrBus : Powertrain -> Plant (actual delivered torques) -----
    PwrBus = makeBus({ ...
        'Tact_fl', 'Actual front-left wheel torque [N*m]'; ...
        'Tact_fr', 'Actual front-right wheel torque [N*m]'; ...
        'Tact_rl', 'Actual rear-left wheel torque [N*m]'; ...
        'Tact_rr', 'Actual rear-right wheel torque [N*m]'});

    % ----- 5. PlantBus : Plant -> Sensors/Controller (vehicle state) -----
    PlantBus = makeBus({ ...
        'yawRate', 'Measured yaw rate [rad/s]'; ...
        'vx',      'Body-frame longitudinal velocity [m/s]'; ...
        'vy',      'Body-frame lateral velocity [m/s]'; ...
        'ax',      'Longitudinal acceleration [m/s^2]'; ...
        'ay',      'Lateral acceleration [m/s^2]'; ...
        'beta',    'Sideslip angle [rad]'});

    % ----- publish all buses to the base workspace -----
    assignin('base', 'RefBus',   RefBus);
    assignin('base', 'CtrlBus',  CtrlBus);
    assignin('base', 'AllocBus', AllocBus);
    assignin('base', 'PwrBus',   PwrBus);
    assignin('base', 'PlantBus', PlantBus);

    fprintf('defineBuses: created RefBus, CtrlBus, AllocBus, PwrBus, PlantBus.\n');
end

% ========================================================================
function bus = localBuildBus(elemSpecs)
%LOCALBUILDBUS  Build a Simulink.Bus from an Nx2 cell of {name, description}.
    n = size(elemSpecs, 1);
    elems = Simulink.BusElement.empty(0, 1);
    for k = 1:n
        e = Simulink.BusElement;
        e.Name           = elemSpecs{k, 1};
        e.DataType       = 'double';   % all signals double for now
        e.Dimensions     = 1;          % scalar signals
        e.Description    = elemSpecs{k, 2};
        elems(k, 1) = e;  %#ok<AGROW>
    end
    bus = Simulink.Bus;
    bus.Elements = elems;
end
