function result = runPathWeaverDemo(options)
%RUNPATHWEAVERDEMO Run the deterministic PathWeaver vertical slice.
arguments
    options.seed (1,1) double = 26037
    options.preset (1,1) string {mustBeMember(options.preset,["nominal","challenging","emergency"])} = "nominal"
    options.plannerMode (1,1) string {mustBeMember(options.plannerMode,["risk-aware","baseline"])} = "risk-aware"
    options.visualization (1,1) logical = true
    options.video (1,1) logical = false
    options.playbackSpeed (1,1) double {mustBeMember(options.playbackSpeed,[.5 1 2 4])} = 1
    options.startPaused (1,1) logical = false
    options.showCandidates (1,1) logical = true
    options.showUncertainty (1,1) logical = true
    options.outputDirectory (1,1) string = ""
    options.maximumSimulationTime (1,1) double = 32
end
root=setupPath(); cfg=defaultConfig(); cfg.seed=options.seed; cfg.mode=options.plannerMode;
cfg.scenario=scenarioConfig(options.preset);
cfg.maxSimulationTime=options.maximumSimulationTime;
cfg.visualization.enabled=options.visualization; cfg.visualization.video=options.video;
cfg.visualization.playbackSpeed=options.playbackSpeed; cfg.visualization.startPaused=options.startPaused;
cfg.visualization.showCandidates=options.showCandidates; cfg.visualization.showUncertainty=options.showUncertainty;
assert(~options.video || options.visualization,'PathWeaver:VideoNeedsFigure','Video export requires visualization=true.');
if strlength(options.outputDirectory)>0
    cfg.outputDirectory=char(options.outputDirectory);
else
    cfg.outputDirectory=fullfile(root,'artifacts');
end
if ~exist(cfg.outputDirectory,'dir'), mkdir(cfg.outputDirectory); end
provenance=pathweaver.evaluation.environmentInfo();
while true
    if cfg.visualization.enabled
        view=pathweaver.visualization.TechnicalView(cfg);
        cleanup=onCleanup(@()view.close());
        callbacks.onStep=@(frame)view.update(frame);
    else
        callbacks.onStep=[];
    end
    try
        result=pathweaver.simulation.runSimulation(cfg,callbacks);
        break
    catch exception
        if strcmp(exception.identifier,'PathWeaver:ResetRequested')
            view.dispose(); clear cleanup; continue
        elseif strcmp(exception.identifier,'PathWeaver:DemoStopped')
            view.dispose(); clear cleanup;
            result=struct('cancelled',true); fprintf('Demo stopped; no completed-run metrics exported.\n'); return
        end
        rethrow(exception)
    end
end
result.provenance=provenance;
save(fullfile(cfg.outputDirectory,sprintf('pathweaver_%s_%s_seed_%d.mat',options.preset,cfg.mode,cfg.seed)),'result');
fprintf('\nPathWeaver %s | %s | seed %d\n',options.preset,cfg.mode,cfg.seed);
fprintf('%s | time %.2f s | clearance %.3f m | min TTC: %s\n', ...
    result.metrics.status,result.log.timeS(end),result.metrics.minimumClearanceM, ...
    pathweaver.planning.ttcLabel(result.metrics.minimumTtcS,cfg.horizon));
fprintf('planner latency mean/p95 = %.2f / %.2f ms (%d warm-up plans excluded)\n', ...
    result.metrics.averagePlannerLatencyS*1000,result.metrics.p95PlannerLatencyS*1000,cfg.latencyWarmupPlans);
if exist('cleanup','var'), clear cleanup; end
end
