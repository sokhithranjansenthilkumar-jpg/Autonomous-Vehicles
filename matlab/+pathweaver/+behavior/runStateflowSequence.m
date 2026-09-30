function output=runStateflowSequence(modelPath,times,inputs,initialCode,initialClearSince)
%RUNSTATEFLOWSEQUENCE Explicit fixed-step stimuli; no base-workspace variables.
if nargin<4, initialCode=0; end
if nargin<5, initialClearSince=NaN; end
assert(size(inputs,2)==8 && size(inputs,1)==numel(times) && times(1)==0 && ...
    all(abs(diff(times)-.05)<1e-9),'PathWeaver:SequenceTiming','Use N-by-8 inputs and a 0.05 s grid starting at zero.');
[~,model]=fileparts(modelPath); load_system(modelPath);
cleanup=onCleanup(@()close_system(model,0));
request=Simulink.SimulationInput(model);
request=request.setVariable('samples',[times(:) inputs],Workspace=model);
request=request.setVariable('initialBehaviorCode',initialCode,Workspace=model);
request=request.setVariable('initialClearSinceS',initialClearSince,Workspace=model);
request=request.setModelParameter(StopTime=num2str(times(end),17));
run=sim(request);
output=table(times(:),run.pathweaverBehaviorState(:),run.pathweaverClearSince(:), ...
    run.pathweaverReasonCode(:),'VariableNames',{'timeS','stateCode','clearSinceS','reasonCode'});
clear cleanup
end
