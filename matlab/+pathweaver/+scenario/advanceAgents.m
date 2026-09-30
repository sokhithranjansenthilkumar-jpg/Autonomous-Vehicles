function scenario = advanceAgents(scenario, ~, dt, cfg)
%ADVANCEAGENTS Exogenous clock-driven actors, independent of ego/planner mode.
assert(isfinite(dt) && dt>=0,'PathWeaver:Timing','Actor timestep must be nonnegative.');
start=scenario.timeS; finish=start+dt;
onset=cfg.scenario.pedestrian.crossingStartS;
for k=1:numel(scenario.agents)
    if scenario.agents(k).id==cfg.scenario.pedestrian.id
        activeDuration=max(0,finish-max(start,onset));
        velocity=[0 cfg.scenario.pedestrian.crossingSpeedMps];
        scenario.agents(k).positionWorldM=scenario.agents(k).positionWorldM+velocity*activeDuration;
        if finish>=onset-1e-10
            scenario.agents(k).velocityWorldMps=velocity;
            scenario.pedestrianTriggered=true;
        end
    else
        scenario.agents(k).positionWorldM=scenario.agents(k).positionWorldM+scenario.agents(k).velocityWorldMps*dt;
    end
    scenario.agents(k).timestampS=finish;
end
scenario.timeS=finish;
end
