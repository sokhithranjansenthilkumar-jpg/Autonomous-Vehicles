function state = initialControllerState()
%INITIALCONTROLLERSTATE Initialise longitudinal integrator memory.
state = struct('integralSpeedError', 0, 'previousSteeringRad', 0);
end
