% startup_project.m  -- runs on project open.
% Builds interface buses and loads the vehicle config into the base workspace.
defineBuses();
assignin('base', 'cfg', vehicleConfig());
fprintf('Project startup: buses defined, cfg loaded.\n');
