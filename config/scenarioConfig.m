function sc = scenarioConfig(preset)
%SCENARIOCONFIG Geometry and actors for deterministic village_crossing.
arguments
    preset (1,1) string {mustBeMember(preset,["nominal","challenging","emergency"])}="nominal"
end
sc.name = "village_crossing";
sc.preset=preset;
sc.roadLengthM = 135;
sc.nominalWidthM = 7.0;
sc.boundarySampleM = 1.0;
sc.egoInitial = struct('positionWorldM', [5 0], 'headingRad', 0, ...
    'speedMps', 8, 'accelerationMps2', 0, 'steeringRad', 0, 'timestampS', 0);
sc.pedestrian = struct('id', 101, 'class', "pedestrian", ...
    'positionWorldM', [49 -4.2], 'velocityWorldMps', [0 0], ...
    'headingRad', pi/2, 'collisionRadiusM', 0.42, ...
    'positionCovarianceWorldM2', diag([0.10 0.10]), 'timestampS', 0, ...
    'crossingStartS', 3.65, 'crossingSpeedMps', 1.25);
sc.twoWheeler = struct('id', 202, 'class', "two_wheeler", ...
    'positionWorldM', [103 2.3], 'velocityWorldMps', [-5.2 0], ...
    'headingRad', pi, 'collisionRadiusM', 0.75, ...
    'positionCovarianceWorldM2', diag([0.12 0.08]), 'timestampS', 0);
sc.pothole = struct('id', 301, 'geometryType', "circle", ...
    'positionWorldM', [72 -2.15], 'geometry', struct('radiusM', 0.85), ...
    'obstacleType', "pothole");
sc.goalPositionWorldM = [128 0];
sc.longitudinalJitterStdM = 0.6;
switch preset
    case "challenging"
        sc.egoInitial.speedMps=9;
        sc.pedestrian.crossingStartS=3.0;
        sc.pedestrian.crossingSpeedMps=1.65;
        sc.pedestrian.positionWorldM(1)=47;
    case "emergency"
        sc.pedestrian.positionWorldM=[25 -3.4];
        sc.pedestrian.crossingStartS=1.0;
        sc.pedestrian.crossingSpeedMps=2.4;
end
end
