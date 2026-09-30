function tests=TestStateflowSequences
tests=functiontests(localfunctions);
end
function setupOnce(t)
setupPath(); assumeFalse(t,isempty(ver('stateflow')));
t.TestData.model=buildPathWeaverStateflowModel();
end
function testOrderedTransitionsOscillationAndRecovery(t)
times=(0:.05:4)'; inputs=repmat([0 Inf 0 0 0 0 100 1],numel(times),1);
inputs(times>=.1 & times<.3,7)=10;
inputs(times>=.3 & times<.5,6)=1;
inputs(times>=.5 & times<.75,1)=.91;
indices=find(times>=.75 & times<1); inputs(indices,1)=.89+.02*mod((1:numel(indices))',2);
inputs(times>=1 & times<1.1,3)=1;
inputs(times>=1.35 & times<1.4,1)=.46;
inputs(times>=1.6 & times<2,2)=4.3;
inputs(abs(times-2.7)<1e-9,2)=NaN;
inputs(times>=3.4,4)=1; inputs(times>=3.6,5)=1;
actual=verifySequence(t,times,inputs,0,NaN);
anchors=[.15 .35 .6 1.05 2.5 2.7 3.4 3.6]; expected=[1 3 2 4 0 4 5 5]';
indices=round(anchors/.05)+1;
verifyEqual(t,actual.stateCode(indices),expected);
end
function testExplicitEmergencyMemoryAndTerminalStates(t)
times=(0:.05:1)'; inputs=repmat([0 Inf 0 0 0 0 100 1],numel(times),1);
verifySequence(t,times,inputs,4,NaN);
verifySequence(t,times,inputs,4,-.25);
verifySequence(t,times,inputs,5,NaN);
verifySequence(t,times,inputs,6,NaN);
inputs(:,4:5)=1; verifySequence(t,times,inputs,0,NaN);
end
function actual=verifySequence(t,times,inputs,initial,clearSince)
initialMemory=clearSince;
cfg=defaultConfig(); scenario=pathweaver.scenario.createVillageCrossing(cfg);
world=pathweaver.scenario.worldState(scenario,cfg.scenario.egoInitial);
predictions=pathweaver.prediction.predictAgents(world.agents,0,cfg);
planner=pathweaver.planning.planTrajectories(world,predictions,scenario,cfg);
names=["CRUISE","FOLLOW","YIELD","AVOID","EMERGENCY_BRAKE","GOAL_REACHED","COLLISION"];
state=pathweaver.behavior.initialState(); state.name=names(initial+1); state.clearSinceS=clearSince;
expected=zeros(numel(times),1); memory=nan(size(expected)); reasons=zeros(size(expected)); numeric=initial;
for k=1:numel(times)
    row=inputs(k,:); localWorld=world; localWorld.timestampS=times(k);
    localWorld.agents=world.agents(1); localWorld.agents.positionWorldM=world.ego.positionWorldM+[row(7) 0];
    localScenario=scenario; if row(4), localScenario.goalPositionWorldM=world.ego.positionWorldM; end
    planner.riskScore=row(1); planner.minimumTtcS=row(2); planner.emergencyFlag=logical(row(3));
    planner.selectedTrajectory.positionsWorldM(end,2)=world.ego.positionWorldM(2)+row(6);
    planner.selectedTrajectory.isFeasible=logical(row(8));
    [state,~]=pathweaver.behavior.updateState(state,planner,localWorld,localScenario,logical(row(5)),cfg);
    expected(k)=pathweaver.behavior.stateCode(state.name); memory(k)=state.clearSinceS;
    [numeric,clearSince,reasons(k)]=pathweaver.behavior.stepDecision(numeric,clearSince,times(k), ...
        row(1),row(2),row(3),row(8),row(4),row(5),row(6),row(7),pathweaver.behavior.decisionLimits(cfg));
end
actual=pathweaver.behavior.runStateflowSequence(t.TestData.model,times,inputs,initial,initialMemory);
verifyEqual(t,actual.stateCode,expected); verifyEqual(t,actual.clearSinceS,memory,'AbsTol',1e-10);
verifyEqual(t,actual.reasonCode,reasons);
end
