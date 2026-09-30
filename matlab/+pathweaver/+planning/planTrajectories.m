function output = planTrajectories(world, predictions, scenario, cfg)
%PLANTRAJECTORIES Generate, score, and deterministically select candidates.
timer = tic;
lateralTargets = unique([cfg.planner.lateralTargetsM world.ego.positionWorldM(2)], 'stable');
targetSpeeds = cfg.planner.routeSpeedMps*cfg.planner.speedFractions;
durations=[repmat(cfg.horizon,size(targetSpeeds)) cfg.planner.stopDurationsS];
targetSpeeds=[targetSpeeds zeros(size(cfg.planner.stopDurationsS))];
count = numel(lateralTargets)*numel(targetSpeeds)+1;
candidate = pathweaver.planning.generateCandidate(world.ego, 0, 0, cfg);
candidates = repmat(candidate, 1, count);
index = 0;
for i = 1:numel(targetSpeeds)
    for j = 1:numel(lateralTargets)
        index = index + 1;
        candidate = pathweaver.planning.generateCandidate(world.ego, ...
            lateralTargets(j), targetSpeeds(i), cfg,durations(i));
        candidates(index) = pathweaver.planning.scoreCandidate( ...
            candidate, predictions, scenario, cfg);
    end
end
braking=pathweaver.planning.emergencyTrajectory(world.ego,cfg);
braking.isFeasible=true; braking.rejectionReason="";
candidates(end)=pathweaver.planning.scoreCandidate(braking,predictions,scenario,cfg);
feasible = find([candidates.isFeasible]);
if isempty(feasible)
    selected = pathweaver.planning.emergencyTrajectory(world.ego,cfg);
    riskScore = pathweaver.planning.dynamicRisk(selected.positionsWorldM,predictions,cfg, ...
        selected.timestampsS,selected.headingsRad);
    emergency = true;
else
    costs = [candidates(feasible).totalCost];
    terminalX = arrayfun(@(c) c.positionsWorldM(end,1), candidates(feasible));
    ordering = sortrows([costs(:), -terminalX(:), feasible(:)], [1 2 3]);
    selectedIndex = ordering(1,3);
    selected = candidates(selectedIndex);
    riskScore = selected.costTerms.dynamicRisk;
    emergency = selected.isEmergencyBraking;
end
output = struct('selectedTrajectory', selected, ...
    'candidateTrajectories', candidates, 'planningLatencyS', toc(timer), ...
    'riskScore', riskScore, ...
    'minimumTtcS', pathweaver.planning.minimumTtc(world.ego, world.agents, cfg.horizon,cfg,scenario.staticObstacles), ...
    'behaviorState', "CRUISE", 'emergencyFlag', emergency);
end
