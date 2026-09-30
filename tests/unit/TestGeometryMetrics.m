function tests=TestGeometryMetrics
tests=functiontests(localfunctions);
end
function testStationarySeparated(t)
verifyEqual(t,pathweaver.core.circleEncounter([10 0],[0 0],2,3),Inf);
end
function testEqualVelocitySeparated(t)
v=[8 1]; verifyEqual(t,pathweaver.core.circleEncounter([6 0],v-v,2,3),Inf);
end
function testMovingApart(t)
verifyEqual(t,pathweaver.core.circleEncounter([10 0],[5 0],2,3),Inf);
end
function testHeadOn(t)
verifyEqual(t,pathweaver.core.circleEncounter([10 0],[-4 0],2,3),2,'AbsTol',1e-12);
verifyEqual(t,pathweaver.core.circleEncounter([10 0],[-4 0],2,1),Inf);
end
function testSimultaneousCrossing(t)
verifyEqual(t,pathweaver.core.circleEncounter([5 -5],[-5 5],1,3), ...
    1-1/sqrt(50),'AbsTol',1e-12);
end
function testDifferentArrivalTimes(t)
[ttc,clearance,closest]=pathweaver.core.circleEncounter([5 -12],[-5 5],1,3);
verifyEqual(t,ttc,Inf); verifyGreaterThan(t,clearance,0); verifyEqual(t,closest,1.7,'AbsTol',1e-12);
end
function testInitialOverlap(t)
verifyEqual(t,pathweaver.core.circleEncounter([.5 0],[3 0],1,3),0);
end
function testFastBetweenSamples(t)
verifyEqual(t,pathweaver.core.circleEncounter([5 0],[-100 0],1,3),.04,'AbsTol',1e-12);
cfg=defaultConfig();
verifyLessThan(t,pathweaver.core.sweptClearance([20 0;20 0],[0;0],[20 -5;20 5],.4,cfg),0);
end
function testInvalidUnavailable(t)
verifyTrue(t,isnan(pathweaver.core.circleEncounter([NaN 0],[0 0],1,3)));
cfg=defaultConfig(); s=pathweaver.scenario.createVillageCrossing(cfg);
s.agents(1).timestampS=.05;
verifyTrue(t,isnan(pathweaver.planning.minimumTtc(cfg.scenario.egoInitial,s.agents,3,cfg)));
verifyError(t,@()pathweaver.prediction.predictAgents(s.agents,0,cfg),'PathWeaver:InvalidAgent');
end
function testCoveringFootprint(t)
cfg=defaultConfig(); [centres,r]=pathweaver.core.egoFootprint([0 0],0,cfg);
corners=[-2.1 -.9;-2.1 .9;2.1 -.9;2.1 .9];
for k=1:4
    verifyLessThanOrEqual(t,min(vecnorm(squeeze(centres)'-corners(k,:),2,2)),r+1e-12);
end
end
function testNoSidewaysMotionAtStandstill(t)
cfg=defaultConfig(); ego=cfg.scenario.egoInitial; ego.speedMps=0;
c=pathweaver.planning.generateCandidate(ego,2,0,cfg);
verifyEqual(t,c.positionsWorldM,repmat(ego.positionWorldM,numel(c.timestampsS),1),'AbsTol',1e-12);
end
function testBoundedEmergencyRollout(t)
cfg=defaultConfig(); ego=cfg.scenario.egoInitial; ego.steeringRad=.15;
c=pathweaver.planning.emergencyTrajectory(ego,cfg);
verifyFalse(t,c.isFeasible); verifyGreaterThanOrEqual(t,c.speedsMps,0);
verifyGreaterThanOrEqual(t,c.accelerationsMps2,cfg.ego.minAccelerationMps2-1e-10);
verifyEqual(t,c.speedsMps(end),0); verifyEqual(t,c.accelerationsMps2(end),0);
end
function testBehaviorEmergencyDwell(t)
cfg=defaultConfig(); s=pathweaver.scenario.createVillageCrossing(cfg);
w=pathweaver.scenario.worldState(s,cfg.scenario.egoInitial);
p=pathweaver.prediction.predictAgents(w.agents,0,cfg);
o=pathweaver.planning.planTrajectories(w,p,s,cfg); state=pathweaver.behavior.initialState();
o.emergencyFlag=true; [state,~]=pathweaver.behavior.updateState(state,o,w,s,false,cfg);
o.emergencyFlag=false; o.minimumTtcS=Inf; o.riskScore=0;
for time=[.05 .25 .50]
    w.timestampS=time; [state,~]=pathweaver.behavior.updateState(state,o,w,s,false,cfg);
    verifyEqual(t,state.name,"EMERGENCY_BRAKE");
end
w.timestampS=.6; [state,~]=pathweaver.behavior.updateState(state,o,w,s,false,cfg);
verifyNotEqual(t,state.name,"EMERGENCY_BRAKE");
end
function testEveryCandidateInfeasible(t)
cfg=defaultConfig(); s=pathweaver.scenario.createVillageCrossing(cfg);
s.staticObstacles.positionWorldM=cfg.scenario.egoInitial.positionWorldM;
w=pathweaver.scenario.worldState(s,cfg.scenario.egoInitial);
p=pathweaver.prediction.predictAgents(w.agents,0,cfg);
o=pathweaver.planning.planTrajectories(w,p,s,cfg);
verifyFalse(t,any([o.candidateTrajectories.isFeasible]));
verifyFalse(t,o.selectedTrajectory.isFeasible); verifyTrue(t,o.emergencyFlag);
verifyTrue(t,contains(o.selectedTrajectory.rejectionReason,'safety not guaranteed'));
verifyEqual(t,o.minimumTtcS,0);
end
function testClockAlignmentAndActuatorBounds(t)
cfg=defaultConfig(); cfg.maxSimulationTime=1;
r=pathweaver.simulation.runSimulation(cfg);
verifyEqual(t,r.lastFrame.world.timestampS,r.finalEgo.timestampS,'AbsTol',1e-12);
verifyEqual(t,[r.scenario.agents.timestampS],repmat(r.finalEgo.timestampS,1,2),'AbsTol',1e-12);
verifyLessThanOrEqual(t,abs(diff(r.log.steeringRad))/cfg.dt,cfg.ego.maxSteeringRateRadps+1e-10);
verifyEqual(t,r.log.accelerationMps2(2:end),diff(r.log.speedMps)/cfg.dt,'AbsTol',1e-10);
verifyEqual(t,r.metrics.integratedAbsoluteJerkMps2,sum(abs(diff(r.log.accelerationMps2))),'AbsTol',1e-10);
verifyEqual(t,height(r.planningLog),r.metrics.planningCalls);
end
