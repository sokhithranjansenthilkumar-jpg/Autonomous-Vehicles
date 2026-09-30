function candidate = scoreCandidate(candidate, predictions, scenario, cfg)
%SCORECANDIDATE Apply hard constraints and compute inspectable cost terms.
p = candidate.positionsWorldM;
finiteSignals = [p candidate.headingsRad candidate.speedsMps ...
    candidate.accelerationsMps2 candidate.curvaturesPerM];
if any(~isfinite(finiteSignals), 'all')
    candidate = reject(candidate, "nonfinite trajectory"); return
end
[roadClearance,boundaryClearance]=pathweaver.core.boundaryClearance(p,candidate.headingsRad,cfg);
if roadClearance<=0
    candidate = reject(candidate, "drivable boundary violation"); return
end
if any(candidate.accelerationsMps2 > cfg.ego.maxAccelerationMps2 + 1e-6) || ...
        any(candidate.accelerationsMps2 < cfg.ego.minAccelerationMps2 - 1e-6)
    candidate = reject(candidate, "acceleration limit"); return
end
if any(abs(candidate.curvaturesPerM) > cfg.ego.maxCurvaturePerM)
    candidate = reject(candidate, "curvature limit"); return
end
jerk = gradient(candidate.accelerationsMps2, cfg.planDt);
steering=atan(cfg.ego.wheelbaseM*candidate.curvaturesPerM);
if any(abs(diff(steering)./diff(candidate.timestampsS))>cfg.ego.maxSteeringRateRadps+1e-8)
    candidate=reject(candidate,"steering rate limit"); return
end
% Emergency braking retains physical constraints; comfort jerk is soft only.
if ~candidate.isEmergencyBraking && any(abs(jerk) > cfg.planner.maxJerkMps3)
    candidate = reject(candidate, "jerk limit"); return
end

obstacle = scenario.staticObstacles;
[centres,egoRadius]=pathweaver.core.egoFootprint(p,candidate.headingsRad,cfg);
clearance=inf(size(p,1),1);
for j=1:3
    clearance=min(clearance,vecnorm(centres(:,:,j)-obstacle.positionWorldM,2,2)-obstacle.geometry.radiusM-egoRadius);
end
if pathweaver.core.sweptClearance(p,candidate.headingsRad,obstacle.positionWorldM, ...
        obstacle.geometry.radiusM,cfg)<=cfg.planner.staticMarginM
    candidate = reject(candidate, "static obstacle collision"); return
end
[dynamicRisk, dynamicCollision] = pathweaver.planning.dynamicRisk(p, predictions, cfg, ...
    candidate.timestampsS,candidate.headingsRad);
if dynamicCollision
    candidate = reject(candidate, "dynamic occupancy collision"); return
end

terms = candidate.costTerms;
terms.dynamicRisk = dynamicRisk;
terms.staticObstacle = trapz(candidate.timestampsS, 1./max(clearance, 0.1).^2);
terms.boundary = trapz(candidate.timestampsS, 1./max(boundaryClearance, 0.1).^2);
terms.smoothness = trapz(candidate.timestampsS, candidate.accelerationsMps2.^2);
terms.curvature = trapz(candidate.timestampsS, candidate.curvaturesPerM.^2);
terms.jerk = trapz(candidate.timestampsS, jerk.^2);
terms.progress = max(0, scenario.goalPositionWorldM(1) - p(end,1))/cfg.scenario.roadLengthM;
terms.routeDeviation = trapz(candidate.timestampsS, p(:,2).^2)/cfg.horizon;
terms.time = 1/max(candidate.speedsMps(end), 0.25);
candidate.costTerms = terms;

names = fieldnames(terms);
total = 0;
for k = 1:numel(names)
    name = names{k};
    weighted = cfg.planner.weights.(name)*terms.(name);
    candidate.weightedCostTerms.(name) = weighted;
    total = total + weighted;
end
candidate.totalCost = total;
end

function candidate = reject(candidate, reason)
candidate.isFeasible = false;
candidate.rejectionReason = reason;
candidate.totalCost = inf;
end
