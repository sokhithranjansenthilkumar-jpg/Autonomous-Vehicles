function ego = makeEgoState(positionWorldM, headingRad, speedMps, timestampS)
%MAKEEGOSTATE Construct and validate an EgoState structure.
ego = struct('positionWorldM', reshape(positionWorldM, 1, 2), ...
    'headingRad', headingRad, 'speedMps', speedMps, ...
    'accelerationMps2', 0, 'steeringRad', 0, 'timestampS', timestampS);
pathweaver.core.validateEgoState(ego);
end
