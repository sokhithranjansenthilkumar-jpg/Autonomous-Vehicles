function tests=TestPresets
tests=functiontests(localfunctions);
end
function testAllPresetsCompleteAndRepeat(t)
for preset=["nominal","challenging","emergency"]
    cfg=defaultConfig(); cfg.scenario=scenarioConfig(preset);
    a=pathweaver.simulation.runSimulation(cfg); b=pathweaver.simulation.runSimulation(cfg);
    verifyEqual(t,a.metrics.status,"goal_reached",char(preset));
    verifyFalse(t,a.collision,char(preset));
    fields={'timeS','xWorldM','yWorldM','speedMps','accelerationMps2','steeringRad','riskScore','minimumTtcS','behavior'};
    verifyEqual(t,a.log(:,fields),b.log(:,fields));
    if preset=="emergency", verifyGreaterThan(t,a.metrics.emergencyBrakingCount,0); end
end
end
function testPairedPhysicalConditionsAndRandomIsolation(t)
cfg=defaultConfig(); original=rng;
a=pathweaver.scenario.createVillageCrossing(cfg); verifyEqual(t,rng,original);
cfg.mode="baseline"; b=pathweaver.scenario.createVillageCrossing(cfg);
verifyEqual(t,a,b);
cfg.prediction.pedestrian=5*cfg.prediction.pedestrian;
verifyEqual(t,a,pathweaver.scenario.createVillageCrossing(cfg));
egoA=cfg.scenario.egoInitial; egoB=egoA; egoB.positionWorldM=[100 0];
for k=1:100
    a=pathweaver.scenario.advanceAgents(a,egoA,cfg.dt,cfg);
    b=pathweaver.scenario.advanceAgents(b,egoB,cfg.dt,cfg);
end
verifyEqual(t,a.agents,b.agents);
cfg.seed=cfg.seed+1; c=pathweaver.scenario.createVillageCrossing(cfg);
verifyNotEqual(t,a.parameters,c.parameters);
end
function testIntrusionBetweenActorSteps(t)
cfg=defaultConfig(); cfg.scenario.pedestrian.crossingStartS=.025;
s=pathweaver.scenario.createVillageCrossing(cfg); y=s.agents(1).positionWorldM(2);
s=pathweaver.scenario.advanceAgents(s,cfg.scenario.egoInitial,.05,cfg);
verifyEqual(t,s.agents(1).positionWorldM(2),y+.025*cfg.scenario.pedestrian.crossingSpeedMps,'AbsTol',1e-12);
end
