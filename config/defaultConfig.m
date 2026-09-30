function cfg = defaultConfig()
%DEFAULTCONFIG Deterministic configuration for PathWeaver v0.1.
cfg.seed = 26037;
cfg.mode = "risk-aware";
cfg.dt = 0.05;
cfg.replanInterval = 0.15;
cfg.horizon = 3.0;
cfg.planDt = 0.20;
cfg.maxSimulationTime = 32;
cfg.goalToleranceM = 3;
cfg.latencyWarmupPlans = 2;

cfg.ego.lengthM = 4.2;
cfg.ego.widthM = 1.8;
cfg.ego.wheelbaseM = 2.7;
cfg.ego.maxSteeringRad = deg2rad(30);
cfg.ego.maxSteeringRateRadps = deg2rad(50);
cfg.ego.maxAccelerationMps2 = 2.8;
cfg.ego.minAccelerationMps2 = -5.5;
cfg.ego.maxCurvaturePerM = tan(cfg.ego.maxSteeringRad)/cfg.ego.wheelbaseM;

cfg.prediction.varianceFloorM2 = 0.04;
cfg.prediction.pedestrian = [0.25 0.45];
cfg.prediction.two_wheeler = [0.18 0.28];
cfg.prediction.car = [0.10 0.16];
cfg.prediction.animal = [0.30 0.55];
cfg.prediction.uncertaintyBeta = 0.15;

cfg.planner.lateralTargetsM = [-2.1 -1.05 0 1.05 2.1];
cfg.planner.speedFractions = [1.0 0.65 0.25 0.0];
cfg.planner.stopDurationsS = [2.2 2.6];
cfg.planner.routeSpeedMps = 8.0;
cfg.planner.maxJerkMps3 = 8.0;
cfg.planner.staticMarginM = 0.35;
cfg.planner.dynamicMarginM = 1.25;
cfg.planner.weights = struct( ...
    'dynamicRisk', 28, 'staticObstacle', 12, 'boundary', 8, ...
    'smoothness', 0.25, 'curvature', 2.0, 'jerk', 0.15, ...
    'progress', 20.0, 'routeDeviation', 0.35, 'time', 5.0);

cfg.behavior.emergencyTtcS = 0.75;
cfg.behavior.yieldTtcS = 3.5;
cfg.behavior.followDistanceM = 14;
cfg.behavior.riskYield = 0.9;
cfg.behavior.riskClear = 0.45;
cfg.behavior.clearDwellS = 0.5;
cfg.behavior.clearTtcMarginS = 0.8;

cfg.control.lookaheadBaseM = 2.5;
cfg.control.lookaheadGainS = 0.35;
cfg.control.speedKp = 1.3;
cfg.control.speedKi = 0.15;
cfg.control.integralLimit = 5;

cfg.visualization.enabled = true;
cfg.visualization.video = false;
cfg.visualization.playbackSpeed = 1;
cfg.visualization.startPaused = false;
cfg.visualization.showCandidates = true;
cfg.visualization.showUncertainty = true;
cfg.outputDirectory = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'artifacts');
cfg.scenario = scenarioConfig();
end
