function modelPath=buildPathWeaverStateflowModel()
%BUILDPATHWEAVERSTATEFLOWMODEL Execute the shared stateful kernel in Stateflow.
root=setupPath(); cfg=defaultConfig();
assert(~isempty(ver('stateflow')),'PathWeaver:StateflowUnavailable','Stateflow is unavailable.');
model='pathweaver_behavior'; directory=fullfile(root,'simulink','models');
if ~exist(directory,'dir'), mkdir(directory); end
modelPath=fullfile(directory,[model '.slx']);
if bdIsLoaded(model), close_system(model,0); end
new_system(model); load_system(model);
cleanup=onCleanup(@()close_system(model,0));
set_param(model,'Solver','FixedStepDiscrete','FixedStep','0.05','StopTime','0.1');
workspace=get_param(model,'ModelWorkspace');
workspace.assignin('samples',[(0:.05:.1)' repmat([0 Inf 0 0 0 0 100 1],3,1)]);
workspace.assignin('limits',pathweaver.behavior.decisionLimits(cfg));
workspace.assignin('initialBehaviorCode',0); workspace.assignin('initialClearSinceS',NaN);
add_block('sflib/Chart',[model '/PathWeaver Behaviour'],'Position',[340 95 620 340]);
chart=find(sfroot,'-isa','Stateflow.Chart','Path',[model '/PathWeaver Behaviour']);
chart=chart(1); chart.ActionLanguage='MATLAB'; chart.ChartUpdate='DISCRETE'; chart.SampleTime='0.05';
names={'riskScore','minimumTtcS','emergencyFlag','goalReached','collisionFlag','lateralAvoid','leadDistanceM','feasible'};
for k=1:numel(names)
    data=Stateflow.Data(chart); data.Name=names{k}; data.Scope='Input'; data.Port=k;
    add_block('simulink/Sources/From Workspace',[model '/' names{k}], ...
        'VariableName',sprintf('samples(:,[1 %d])',k+1),'Interpolate','off', ...
        'OutputAfterFinalValue','Holding final value', ...
        'Position',[35 20+43*k 190 43+43*k]);
    add_line(model,[names{k} '/1'],['PathWeaver Behaviour/' num2str(k)],'autorouting','on');
end
data=Stateflow.Data(chart); data.Name='timeS'; data.Scope='Input'; data.Port=9;
add_block('simulink/Sources/Digital Clock',[model '/Simulation clock'], ...
    'SampleTime','0.05','Position',[35 425 190 450]);
add_line(model,'Simulation clock/1','PathWeaver Behaviour/9','autorouting','on');
for name={'limits','initialBehaviorCode','initialClearSinceS'}
    data=Stateflow.Data(chart); data.Name=name{1}; data.Scope='Parameter';
end
outputNames={'stateCode','clearSinceS','reasonCode'};
initials={'0','NaN','0'};
logNames={'pathweaverBehaviorState','pathweaverClearSince','pathweaverReasonCode'};
for k=1:3
    data=Stateflow.Data(chart); data.Name=outputNames{k}; data.Scope='Output'; data.Port=k;
    data.Props.InitialValue=initials{k};
    add_block('simulink/Sinks/To Workspace',[model '/' outputNames{k}], ...
        'VariableName',logNames{k},'SaveFormat','Array','Position',[735 80+80*k 900 110+80*k]);
    add_line(model,['PathWeaver Behaviour/' num2str(k)],[outputNames{k} '/1'],'autorouting','on');
end
state=Stateflow.State(chart); state.Name='ExecuteReference'; state.Position=[60 50 620 180];
step=sprintf(['[stateCode, clearSinceS, reasonCode] = pathweaver.behavior.stepDecision( ...\n' ...
    'stateCode, clearSinceS, timeS, riskScore, minimumTtcS, emergencyFlag, feasible, ...\n' ...
    'goalReached, collisionFlag, lateralAvoid, leadDistanceM, limits);']);
state.LabelString=sprintf(['ExecuteReference\nentry:\n' ...
    'stateCode = initialBehaviorCode;\nclearSinceS = initialClearSinceS;\n%s\nduring:\n%s'],step,step);
transition=Stateflow.Transition(chart); transition.Destination=state; transition.DestinationOClock=0;
note=Simulink.Annotation(model,sprintf(['SIMULATION-TIME BEHAVIOUR EXECUTION, 0.05 s\n' ...
    'Stateful shared MATLAB kernel compiled inside Stateflow; seven numeric behaviour codes.\n' ...
    'Test-input harness only: no perception, planner, controller or vehicle dynamics execute here.']));
note.Position=[40 485 930 555];
set_param(model,'SimulationCommand','update');
out=sim(model);
assert(~isempty(out.pathweaverBehaviorState),'PathWeaver:EmptyStateflowRun','No behaviour output.');
save_system(model,modelPath);
clear cleanup
fprintf('Built and ran behaviour execution harness: %s\n',modelPath);
end
