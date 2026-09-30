function result=recordPathWeaverEvidence(options)
%RECORDPATHWEAVEREVIDENCE Record an actual run plus revision and frame metadata.
arguments
    options.teamName (1,1) string = "Team PathWeaver"
    options.preset (1,1) string {mustBeMember(options.preset,["nominal","challenging","emergency"])} = "nominal"
    options.seed (1,1) double = 26037
    options.plannerMode (1,1) string {mustBeMember(options.plannerMode,["baseline","risk-aware"])} = "risk-aware"
    options.outputDirectory (1,1) string = ""
end
root=setupPath(); output=options.outputDirectory;
if strlength(output)==0, output=string(fullfile(root,'artifacts','release','backup',options.preset)); end
result=runPathWeaverDemo(preset=options.preset,seed=options.seed,plannerMode=options.plannerMode, ...
    visualization=true,video=true,playbackSpeed=4,outputDirectory=output);
if isfield(result,'cancelled'), return; end
video=VideoReader(fullfile(output,'pathweaver_demo.mp4'));
provenance=result.provenance;
provenance.team=options.teamName; provenance.preset=options.preset;
provenance.seed=options.seed; provenance.mode=options.plannerMode;
provenance.simulatedDurationS=result.metrics.durationS;
provenance.encodedDurationS=video.Duration; provenance.frameRate=video.FrameRate;
provenance.width=video.Width; provenance.height=video.Height;
provenance.note="Fixed simulated-time frame cadence; live playback speed does not change encoded timing.";
save(fullfile(output,'recording_provenance.mat'),'provenance');
fprintf('Backup: %s\n%s | seed %d | %s | revision %s | dirty=%d\n', ...
    output,options.preset,options.seed,options.plannerMode,provenance.revision,provenance.dirty);
end
