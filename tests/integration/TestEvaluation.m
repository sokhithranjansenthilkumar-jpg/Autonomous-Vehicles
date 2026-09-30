function tests=TestEvaluation
tests=functiontests(localfunctions);
end
function testExportsIncludeTimeoutsAndProvenance(t)
root=setupPath(); output=string(tempname(fullfile(root,'artifacts')));
[results,summary]=runPathWeaverEvaluation(seeds=27001,presets="nominal", ...
    maximumSimulationTime=.1,outputDirectory=output);
verifyEqual(t,height(results),2); verifyEqual(t,results.status,repmat("timeout",2,1));
verifyEqual(t,sum(summary.timeouts),2); verifyTrue(t,all(isnan(results.timeToGoalS)));
verifyTrue(t,isfile(fullfile(output,'evaluation_results.csv')));
verifyTrue(t,isfile(fullfile(output,'evaluation_summary.png')));
saved=load(fullfile(output,'evaluation_results.mat'));
verifyEqual(t,saved.results,results);
a=load(fullfile(output,results.runFile(1))); b=load(fullfile(output,results.runFile(2)));
verifyEqual(t,a.run.initialScenario,b.run.initialScenario);
verifyEqual(t,results.revision,repmat(saved.metadata.revision,2,1));
end
