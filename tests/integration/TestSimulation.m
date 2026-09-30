function tests = TestSimulation
%TESTSIMULATION Closed-loop and conditional Simulink integration tests.
tests = functiontests(localfunctions);
end

function setupOnce(~)
setupPath();
end

function testShortClosedLoop(testCase)
cfg=defaultConfig(); cfg.maxSimulationTime=1;
r=pathweaver.simulation.runSimulation(cfg);
verifyGreaterThan(testCase,height(r.log),10);
verifyFalse(testCase,any(ismissing(r.log(:,1:7)),'all'));
verifyFalse(testCase,r.collision);
end

function testDefaultCompletes(testCase)
cfg=defaultConfig(); r=pathweaver.simulation.runSimulation(cfg);
verifyTrue(testCase,r.completed);
verifyFalse(testCase,r.collision);
end

function testSimulinkModelWhenAvailable(testCase)
assumeFalse(testCase,isempty(ver('simulink')));
modelPath=buildPathWeaverModel();
verifyTrue(testCase,isfile(modelPath));
load_system(modelPath); cleanup=onCleanup(@()close_system('pathweaver_v0',0));
set_param('pathweaver_v0','SimulationCommand','update');
clear cleanup
end

function testStateflowMatchesMatlabDecisions(testCase)
assumeFalse(testCase,isempty(ver('stateflow')));
modelPath=buildPathWeaverStateflowModel();
cfg=defaultConfig(); scenario=pathweaver.scenario.createVillageCrossing(cfg);
ego=cfg.scenario.egoInitial; world=pathweaver.scenario.worldState(scenario,ego);
predictions=pathweaver.prediction.predictAgents(world.agents,0,cfg);
basePlanner=pathweaver.planning.planTrajectories(world,predictions,scenario,cfg);
base=struct('riskScore',0,'minimumTtcS',10,'emergencyFlag',0, ...
    'goalReached',0,'collisionFlag',0,'lateralAvoid',0,'leadDistanceM',100);
cases={base,base,base,base,base,base,base};
cases{2}.leadDistanceM=10;
cases{3}.minimumTtcS=2;
cases{4}.lateralAvoid=1;
cases{5}.emergencyFlag=1;
cases{6}.goalReached=1;
cases{7}.collisionFlag=1;
expected=["CRUISE","FOLLOW","YIELD","AVOID","EMERGENCY_BRAKE","GOAL_REACHED","COLLISION"];
for k=1:numel(cases)
    planner=basePlanner;
    planner.riskScore=cases{k}.riskScore;
    planner.minimumTtcS=cases{k}.minimumTtcS;
    planner.emergencyFlag=logical(cases{k}.emergencyFlag);
    localWorld=world; localScenario=scenario;
    if cases{k}.lateralAvoid
        planner.selectedTrajectory.positionsWorldM(end,2)=world.ego.positionWorldM(2)+1;
    end
    if cases{k}.leadDistanceM<14
        localWorld.agents(1).positionWorldM=world.ego.positionWorldM+[10 0];
    end
    if cases{k}.goalReached
        localWorld.ego.positionWorldM=scenario.goalPositionWorldM;
    end
    [matlabState,~]=pathweaver.behavior.updateState( ...
        pathweaver.behavior.initialState(),planner,localWorld,localScenario, ...
        logical(cases{k}.collisionFlag),cfg);
    verifyEqual(testCase,matlabState.name,expected(k));
    chartCode=pathweaver.behavior.runStateflowCase(modelPath,cases{k});
    verifyEqual(testCase,chartCode,pathweaver.behavior.stateCode(matlabState.name));
end
end
