function egoNext = updateBicycle(ego, command, dt, cfg)
%UPDATEBICYCLE Advance simplified kinematic bicycle dynamics one step.
values = [ego.positionWorldM ego.headingRad ego.speedMps ...
    command.steeringRad command.accelerationMps2 dt];
assert(all(isfinite(values)) && dt > 0, 'PathWeaver:InvalidDynamicsInput', ...
    'Bicycle inputs must be finite and dt must be positive.');
steering = max(-cfg.ego.maxSteeringRad, ...
    min(cfg.ego.maxSteeringRad, command.steeringRad));
acceleration = max(cfg.ego.minAccelerationMps2, ...
    min(cfg.ego.maxAccelerationMps2, command.accelerationMps2));
egoNext = ego;
egoNext.positionWorldM = ego.positionWorldM + ...
    ego.speedMps*[cos(ego.headingRad) sin(ego.headingRad)]*dt;
egoNext.headingRad = pathweaver.core.wrapAngle(ego.headingRad + ...
    ego.speedMps*tan(steering)/cfg.ego.wheelbaseM*dt);
egoNext.speedMps = max(0, ego.speedMps + acceleration*dt);
egoNext.accelerationMps2 = (egoNext.speedMps-ego.speedMps)/dt;
egoNext.steeringRad = steering;
egoNext.timestampS = ego.timestampS + dt;
pathweaver.core.validateEgoState(egoNext);
end
