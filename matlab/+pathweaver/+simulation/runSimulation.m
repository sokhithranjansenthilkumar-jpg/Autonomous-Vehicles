function result = runSimulation(cfg, callbacks)
%RUNSIMULATION Fixed-step closed loop; display never advances simulation time.
arguments
    cfg (1,1) struct
    callbacks (1,1) struct = struct('onStep', [])
end
assert(cfg.maxSimulationTime>=cfg.dt && cfg.dt>0 && ...
    abs(cfg.planDt/cfg.dt-round(cfg.planDt/cfg.dt))<1e-8, ...
    'PathWeaver:Timing','Require positive duration and planDt divisible by dt.');
scenario=pathweaver.scenario.createVillageCrossing(cfg); initialScenario=scenario;
ego=cfg.scenario.egoInitial; behavior=pathweaver.behavior.initialState();
controller=pathweaver.control.initialControllerState();
maxSteps=floor(cfg.maxSimulationTime/cfg.dt); replanSteps=max(1,round(cfg.replanInterval/cfg.dt));
names={'timeS','xWorldM','yWorldM','speedMps','accelerationMps2','steeringRad', ...
    'curvaturePerM','riskScore','minimumTtcS','planningLatencyS', ...
    'selectedEndXWorldM','selectedEndYWorldM','minimumClearanceM','headingRad'};
data=nan(maxSteps+1,numel(names)); behaviorName=strings(maxSteps+1,1);
transitions=struct('occurred',{},'timestampS',{},'from',{},'to',{},'reason',{});
planningTimes=zeros(0,2); planner=[]; runTimer=tic;
[collision,collisionReason,clearance]=pathweaver.simulation.detectCollision(ego,scenario,cfg);
ttc=pathweaver.planning.minimumTtc(ego,scenario.agents,cfg.horizon,cfg,scenario.staticObstacles);
data(1,:)=[ego.timestampS ego.positionWorldM ego.speedMps ego.accelerationMps2 ...
    ego.steeringRad 0 NaN ttc NaN NaN NaN clearance ego.headingRad];
behaviorName(1)=behavior.name; completed=false;
for step=1:maxSteps
    scenario=pathweaver.scenario.advanceAgents(scenario,ego,0,cfg);
    world=pathweaver.scenario.worldState(scenario,ego);
    if isempty(planner) || mod(step-1,replanSteps)==0
        timer=tic;
        predictions=pathweaver.prediction.predictAgents(world.agents,world.timestampS,cfg);
        planner=pathweaver.planning.planTrajectories(world,predictions,scenario,cfg);
        planner.planningLatencyS=toc(timer);
        planner.timestampS=world.timestampS;
        planningTimes(end+1,:)=[world.timestampS planner.planningLatencyS]; %#ok<AGROW>
    end
    planner.minimumTtcS=pathweaver.planning.minimumTtc(ego,world.agents,cfg.horizon,cfg,scenario.staticObstacles);
    [behavior,transition]=pathweaver.behavior.updateState(behavior,planner,world,scenario,collision,cfg);
    if transition.occurred, transitions(end+1)=transition; end %#ok<AGROW>
    emergency=planner.emergencyFlag || behavior.name=="EMERGENCY_BRAKE";
    [command,controller]=pathweaver.control.trackTrajectory(ego,planner.selectedTrajectory,emergency,controller,cfg);
    previousEgo=ego; previousScenario=scenario;
    ego=pathweaver.control.updateBicycle(ego,command,cfg.dt,cfg);
    scenario=pathweaver.scenario.advanceAgents(scenario,previousEgo,cfg.dt,cfg);
    [collision,collisionReason,clearance]=pathweaver.simulation.detectCollision(ego,scenario,cfg,previousEgo,previousScenario);
    world=pathweaver.scenario.worldState(scenario,ego);
    planner.minimumTtcS=pathweaver.planning.minimumTtc(ego,world.agents,cfg.horizon,cfg,scenario.staticObstacles);
    completed=~collision && norm(ego.positionWorldM-scenario.goalPositionWorldM)<=cfg.goalToleranceM;
    if collision || completed
        [behavior,transition]=pathweaver.behavior.updateState(behavior,planner,world,scenario,collision,cfg);
        if transition.occurred, transitions(end+1)=transition; end %#ok<AGROW>
    end
    planner.behaviorState=behavior.name;
    data(step+1,:)=[ego.timestampS ego.positionWorldM ego.speedMps ego.accelerationMps2 ...
        ego.steeringRad tan(ego.steeringRad)/cfg.ego.wheelbaseM planner.riskScore ...
        planner.minimumTtcS planner.planningLatencyS planner.selectedTrajectory.positionsWorldM(end,:) ...
        clearance ego.headingRad];
    behaviorName(step+1)=behavior.name;
    frame=struct('scenario',scenario,'world',world,'ego',ego,'predictions',predictions, ...
        'planner',planner,'behavior',behavior,'collision',collision, ...
        'collisionReason',collisionReason,'completed',completed,'clearanceM',clearance,'config',cfg, ...
        'command',command,'emergencyOverride',emergency);
    if ~isempty(callbacks.onStep), callbacks.onStep(frame); end
    if collision || completed, break; end
end
log=array2table(data(1:step+1,:),'VariableNames',names); log.behavior=behaviorName(1:step+1);
planningLog=array2table(planningTimes,'VariableNames',{'timeS','latencyS'});
metrics=pathweaver.evaluation.computeMetrics(log,completed,collision,planningLog,cfg);
metrics.runtimeS=toc(runTimer);
result=struct('config',cfg,'scenario',scenario,'initialScenario',initialScenario,'finalEgo',ego, ...
    'log',log,'planningLog',planningLog,'transitions',transitions,'metrics',metrics, ...
    'completed',completed,'collision',collision,'collisionReason',collisionReason,'lastFrame',frame);
end
