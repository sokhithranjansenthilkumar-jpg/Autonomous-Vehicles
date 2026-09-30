function [results,summary] = runPathWeaverEvaluation(options)
%RUNPATHWEAVEREVALUATION Paired encounters; retain all runs and their provenance.
arguments
    options.seeds (1,:) double {mustBeInteger,mustBeNonnegative} = 27001:27010
    options.presets (1,:) string {mustBeMember(options.presets,["nominal","challenging","emergency"])} = ["nominal","challenging","emergency"]
    options.maximumSimulationTime (1,1) double {mustBePositive} = 32
    options.outputDirectory (1,1) string = ""
end
root=setupPath();
assert(~isempty(options.seeds) && ~isempty(options.presets),'PathWeaver:EmptyBatch','Select at least one seed and preset.');
assert(numel(unique(options.seeds))==numel(options.seeds),'PathWeaver:DuplicateSeeds','Seed labels must be unique.');
if strlength(options.outputDirectory)==0
    outputDirectory=fullfile(root,'artifacts','evaluation');
else
    outputDirectory=char(options.outputDirectory);
end
if ~exist(outputDirectory,'dir'), mkdir(outputDirectory); end
runDirectory=fullfile(outputDirectory,'runs'); if ~exist(runDirectory,'dir'), mkdir(runDirectory); end
metadata=pathweaver.evaluation.environmentInfo();
modes=["baseline","risk-aware"]; rows=cell(numel(options.presets)*numel(options.seeds)*2,1); index=0;
metricNames={'collision','completed','minimumTtcS','ttcHorizonS','ttcUnavailableSamples', ...
    'minimumClearanceM','timeToGoalS','durationS','pathLengthM','averagePlannerLatencyS', ...
    'p95PlannerLatencyS','maximumPlannerLatencyS','planningCalls','latencyWarmupExcluded', ...
    'integratedAbsoluteJerkMps2','maximumCurvaturePerM','emergencyBrakingCount','runtimeS'};
for preset=options.presets
    for seed=options.seeds
        % Alternate execution order to avoid always warming one planner first.
        order=modes; if mod(seed,2)==0, order=fliplr(order); end
        pairedConditions=[];
        for mode=order
            cfg=defaultConfig(); cfg.mode=mode; cfg.seed=seed; cfg.scenario=scenarioConfig(preset);
            cfg.maxSimulationTime=options.maximumSimulationTime;
            cfg.visualization.enabled=false; cfg.visualization.video=false;
            scenario=pathweaver.scenario.createVillageCrossing(cfg);
            if isempty(pairedConditions), pairedConditions=scenario;
            else, assert(isequaln(scenario,pairedConditions),'PathWeaver:UnpairedRun','Physical conditions differ.'); end
            row=struct('preset',preset,'seed',seed,'mode',mode,'revision',metadata.revision, ...
                'dirty',metadata.dirty,'status',"invalid",'errorIdentifier',"",'errorMessage',"");
            params=fieldnames(scenario.parameters);
            for k=1:numel(params), row.(params{k})=scenario.parameters.(params{k}); end
            for k=1:numel(metricNames), row.(metricNames{k})=NaN; end
            runFile=sprintf('%s_%s_seed_%d.mat',preset,mode,seed);
            row.runFile=string(fullfile('runs',runFile)); timer=tic;
            try
                run=pathweaver.simulation.runSimulation(cfg);
                row.status=run.metrics.status;
                for k=1:numel(metricNames), row.(metricNames{k})=double(run.metrics.(metricNames{k})); end
                save(fullfile(runDirectory,runFile),'run','metadata');
            catch exception
                row.runtimeS=toc(timer); row.errorIdentifier=string(exception.identifier);
                row.errorMessage=string(exception.message);
                failure=struct('identifier',row.errorIdentifier,'message',row.errorMessage);
                save(fullfile(runDirectory,runFile),'cfg','scenario','failure','metadata');
            end
            index=index+1; rows{index}=struct2table(row);
            results=vertcat(rows{1:index});
            % Checkpoint after every run so interruption cannot erase failures.
            writetable(results,fullfile(outputDirectory,'evaluation_results.csv'));
            save(fullfile(outputDirectory,'evaluation_results.mat'),'results','metadata','options');
            fprintf('%s | %s | seed %d: %s, clearance %.3f m\n',preset,mode,seed,row.status,row.minimumClearanceM);
        end
    end
end
summary=pathweaver.evaluation.aggregateResults(results);
writetable(summary,fullfile(outputDirectory,'evaluation_summary.csv'));
save(fullfile(outputDirectory,'evaluation_results.mat'),'results','summary','metadata','options');
pathweaver.evaluation.plotComparison(results,outputDirectory);
disp(summary);
fprintf('Evidence: %s\nRevision: %s (dirty=%d)\n',outputDirectory,metadata.revision,metadata.dirty);
end
