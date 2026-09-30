function tests = TestCore
%TESTCORE Unit coverage for geometry, prediction, planning, behaviour, control.
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
testCase.TestData.root = setupPath();
testCase.TestData.cfg = defaultConfig();
end

function testGeometryPrimitives(testCase)
verifyTrue(testCase,pathweaver.core.circleOverlap([0 0],1,[1.5 0],0.5));
verifyFalse(testCase,pathweaver.core.circleOverlap([0 0],1,[2 0],0.5));
verifyEqual(testCase,pathweaver.core.wrapAngle(3*pi),-pi,'AbsTol',1e-12);
end

function testDrivableBoundary(testCase)
cfg=testCase.TestData.cfg;
verifyTrue(testCase,pathweaver.core.isDrivable([20 0],0.9,cfg.scenario));
verifyFalse(testCase,pathweaver.core.isDrivable([20 4],0.9,cfg.scenario));
end

function testCollisionDetection(testCase)
cfg=testCase.TestData.cfg; s=pathweaver.scenario.createVillageCrossing(cfg);
ego=cfg.scenario.egoInitial; ego.positionWorldM=s.staticObstacles.positionWorldM;
[collision,reason]=pathweaver.simulation.detectCollision(ego,s,cfg);
verifyTrue(testCase,collision); verifyEqual(testCase,reason,"static obstacle");
end

function testDeterministicSeed(testCase)
cfg=testCase.TestData.cfg;
a=pathweaver.scenario.createVillageCrossing(cfg);
b=pathweaver.scenario.createVillageCrossing(cfg);
verifyEqual(testCase,[a.agents.positionWorldM],[b.agents.positionWorldM]);
cfg.seed=cfg.seed+1; c=pathweaver.scenario.createVillageCrossing(cfg);
verifyNotEqual(testCase,[a.agents.positionWorldM],[c.agents.positionWorldM]);
end

function testPredictionMeanAndCovariance(testCase)
cfg=testCase.TestData.cfg; s=pathweaver.scenario.createVillageCrossing(cfg);
p=pathweaver.prediction.predictAgents(s.agents,0,cfg);
tau=cfg.horizon;
expected=s.agents(1).positionWorldM+tau*s.agents(1).velocityWorldMps;
verifyEqual(testCase,p(1).expectedPositionsWorldM(end,:),expected,'AbsTol',1e-12);
for k=1:size(p(1).positionCovariancesWorldM2,3)
    verifyGreaterThan(testCase,eig(p(1).positionCovariancesWorldM2(:,:,k)),0);
end
end

function testClassConditionedGrowth(testCase)
cfg=testCase.TestData.cfg; s=pathweaver.scenario.createVillageCrossing(cfg);
animal=s.agents(1); animal.class="animal"; car=animal; car.class="car";
p=pathweaver.prediction.predictAgents([animal car],0,cfg);
verifyGreaterThan(testCase,trace(p(1).positionCovariancesWorldM2(:,:,end)), ...
    trace(p(2).positionCovariancesWorldM2(:,:,end)));
end

function testRiskMonotonicity(testCase)
cfg=testCase.TestData.cfg; s=pathweaver.scenario.createVillageCrossing(cfg);
p=pathweaver.prediction.predictAgents(s.agents(1),0,cfg);
near=p.expectedPositionsWorldM;
far=near+[20*ones(size(near,1),1) zeros(size(near,1),1)];
[nearRisk,nearOverlap]=pathweaver.planning.dynamicRisk(near,p,cfg);
[farRisk,farOverlap]=pathweaver.planning.dynamicRisk(far,p,cfg);
verifyGreaterThan(testCase,nearRisk,farRisk);
verifyTrue(testCase,nearOverlap); verifyFalse(testCase,farOverlap);
pWide=p; pWide.positionCovariancesWorldM2=4*p.positionCovariancesWorldM2;
[wideRisk,~,~]=pathweaver.planning.dynamicRisk(near,pWide,cfg);
verifyGreaterThan(testCase,wideRisk,nearRisk);
end

function testTrajectoryConstraintsAndCost(testCase)
cfg=testCase.TestData.cfg; s=pathweaver.scenario.createVillageCrossing(cfg);
ego=cfg.scenario.egoInitial; w=pathweaver.scenario.worldState(s,ego);
p=pathweaver.prediction.predictAgents(w.agents,0,cfg);
o=pathweaver.planning.planTrajectories(w,p,s,cfg);
verifyGreaterThanOrEqual(testCase,numel(o.candidateTrajectories),20);
verifyTrue(testCase,o.selectedTrajectory.isFeasible);
names=fieldnames(o.selectedTrajectory.costTerms); total=0;
for k=1:numel(names), total=total+o.selectedTrajectory.weightedCostTerms.(names{k}); end
verifyEqual(testCase,total,o.selectedTrajectory.totalCost,'RelTol',1e-12);
bad=pathweaver.planning.generateCandidate(ego,10,8,cfg);
bad=pathweaver.planning.scoreCandidate(bad,p,s,cfg);
verifyFalse(testCase,bad.isFeasible);
end

function testEmergencyBraking(testCase)
cfg=testCase.TestData.cfg; ego=cfg.scenario.egoInitial;
t=pathweaver.planning.generateCandidate(ego,0,8,cfg);
state=pathweaver.control.initialControllerState();
[command,~]=pathweaver.control.trackTrajectory(ego,t,true,state,cfg);
verifyEqual(testCase,command.accelerationMps2,cfg.ego.minAccelerationMps2);
end

function testBehaviourTransitions(testCase)
cfg=testCase.TestData.cfg; s=pathweaver.scenario.createVillageCrossing(cfg);
ego=cfg.scenario.egoInitial; w=pathweaver.scenario.worldState(s,ego);
p=pathweaver.prediction.predictAgents(w.agents,0,cfg);
planner=pathweaver.planning.planTrajectories(w,p,s,cfg);
planner.emergencyFlag=true; state=pathweaver.behavior.initialState();
[state,transition]=pathweaver.behavior.updateState(state,planner,w,s,false,cfg);
verifyEqual(testCase,state.name,"EMERGENCY_BRAKE"); verifyTrue(testCase,transition.occurred);
end

function testBicycleUpdate(testCase)
cfg=testCase.TestData.cfg; ego=cfg.scenario.egoInitial;
command=struct('steeringRad',0,'accelerationMps2',1);
next=pathweaver.control.updateBicycle(ego,command,0.1,cfg);
verifyEqual(testCase,next.positionWorldM,[5.8 0],'AbsTol',1e-12);
verifyEqual(testCase,next.speedMps,8.1,'AbsTol',1e-12);
end

function testBaselineConfiguration(testCase)
cfg=testCase.TestData.cfg; s=pathweaver.scenario.createVillageCrossing(cfg);
risk=pathweaver.prediction.predictAgents(s.agents,0,cfg);
cfg.mode="baseline"; base=pathweaver.prediction.predictAgents(s.agents,0,cfg);
verifyGreaterThan(testCase,trace(risk(1).positionCovariancesWorldM2(:,:,end)), ...
    trace(base(1).positionCovariancesWorldM2(:,:,end)));
verifyEqual(testCase,base(1).positionCovariancesWorldM2(:,:,1), ...
    base(1).positionCovariancesWorldM2(:,:,end),'AbsTol',1e-12);
end
