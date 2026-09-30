function outputDirectory = generatePathWeaverPitchAssets(options)
%GENERATEPATHWEAVERPITCHASSETS Export reproducible 16:9 pitch frames.
arguments
    options.teamName (1,1) string = "Team PathWeaver"
    options.outputDirectory (1,1) string = ""
    options.refreshEvaluation (1,1) logical = false
    options.evaluationDirectory (1,1) string = ""
end
root=setupPath();
if strlength(options.outputDirectory)==0
    outputDirectory=fullfile(root,'artifacts','pitch');
else
    outputDirectory=char(options.outputDirectory);
end
if ~exist(outputDirectory,'dir'),mkdir(outputDirectory);end
colors=palette();

fig=newSlide(colors); logoPath=fullfile(root,'media','pathweaver-logo.png');
logoAxes=axes(fig,'Position',[0.16 0.48 0.68 0.38]); axis(logoAxes,'off');
[logo,~,alpha]=imread(logoPath); image(logoAxes,logo,'AlphaData',alpha); axis(logoAxes,'image','off');
addText(fig,[0.12 0.27 0.76 0.13],options.teamName,28,colors.silver,'center','bold');
addText(fig,[0.12 0.14 0.76 0.11], ...
    "Bennett University  |  SIH26037  |  MathWorks / Robotics and Drones", ...
    18,colors.cyan,'center','normal');
saveSlide(fig,outputDirectory,'01-opening.png');

fig=newSlide(colors); heading(fig,"THE ROAD DOES NOT BEHAVE LIKE A LANE");
cards={"MISSING LANE MARKINGS","UNPREDICTABLE CROSSINGS", ...
    "MIXED VEHICLE BEHAVIOUR","POTHOLES + UNCLEAR EDGES"};
details={"Free space matters more than a painted centreline", ...
    "Pedestrians and animals can change intent", ...
    "Two-wheelers, cars and vulnerable road users interact", ...
    "Hard boundaries and local hazards constrain every path"};
for k=1:4
    column=mod(k-1,2); row=floor((k-1)/2);
    x=0.09+0.46*column; y=0.55-0.27*row;
    annotation(fig,'rectangle',[x y 0.37 0.19],'Color',colors.teal,'LineWidth',1.5, ...
        'FaceColor',colors.panel);
    addText(fig,[x+0.02 y+0.105 0.33 0.055],cards{k},17,colors.cyan,'left','bold');
    addText(fig,[x+0.02 y+0.025 0.33 0.075],details{k},12,colors.silver,'left','normal');
end
addText(fig,[0.11 0.10 0.78 0.08], ...
    "Fixed lanes and fixed obstacle footprints discard the uncertainty that matters.", ...
    18,colors.amber,'center','bold');
saveSlide(fig,outputDirectory,'02-problem.png');

fig=newSlide(colors); heading(fig,"PLAN AROUND WHERE THEY MIGHT BE");
steps={sprintf('CURRENT\nAGENT'),sprintf('PREDICTED\nMEAN'), ...
    sprintf('WIDENING\nUNCERTAINTY'),sprintf('CANDIDATE\nPATHS'), ...
    sprintf('RISK-AWARE\nCHOICE')};
for k=1:numel(steps)
    x=0.055+(k-1)*0.19;
    annotation(fig,'ellipse',[x 0.45 0.13 0.20],'Color',colors.cyan, ...
        'FaceColor',colors.panel,'LineWidth',2);
    addText(fig,[x+0.005 0.49 0.12 0.10],steps{k},14,colors.silver,'center','bold');
    if k<numel(steps)
        annotation(fig,'arrow',[x+0.135 x+0.18],[0.55 0.55],'Color',colors.teal,'LineWidth',2);
    end
end
addText(fig,[0.10 0.24 0.80 0.11], ...
    "Our baseline predicts mean motion. PathWeaver also scores expanding, class-conditioned uncertainty.", ...
    19,colors.cyan,'center','normal');
saveSlide(fig,outputDirectory,'03-core-idea.png');

fig=newSlide(colors); heading(fig,"ARCHITECTURE: WORKING CORE + NEXT INTEGRATION");
annotation(fig,'rectangle',[0.05 0.14 0.43 0.69],'Color',colors.teal,'FaceColor',colors.panel,'LineWidth',2);
annotation(fig,'rectangle',[0.52 0.14 0.43 0.69],'Color',colors.amber,'FaceColor',colors.panel,'LineWidth',2);
addText(fig,[0.08 0.74 0.37 0.06],"IMPLEMENTED + VERIFIED",18,colors.cyan,'center','bold');
implemented=sprintf(['Village scenario\n↓\nSimulated world state\n↓\n' ...
    'Class-conditioned prediction\n↓\nRisk-aware trajectory selection\n↓\n' ...
    'Behaviour + controller\n↓\nClosed-loop motion + metrics']);
addText(fig,[0.10 0.22 0.33 0.49],implemented,16,colors.silver,'center','normal');
addText(fig,[0.55 0.74 0.37 0.06],"PLANNED INTEGRATION",18,colors.amber,'center','bold');
planned=sprintf(['RoadRunner (Windows/Linux)\n↓\nSynthetic camera / LiDAR / radar\n↓\n' ...
    'Detection + fusion + tracking\n↓\nFull Simulink vehicle execution\n↓\n' ...
    'Expanded scenario validation']);
addText(fig,[0.57 0.27 0.33 0.40],planned,16,colors.silver,'center','normal');
saveSlide(fig,outputDirectory,'04-architecture.png');

fig=newSlide(colors); heading(fig,"VALIDATION PROGRAM");
scenarios={"UNMARKED VILLAGE ROAD","UNSIGNALIZED INTERSECTION", ...
    "HIGHWAY MERGE","DENSE MARKET","CATTLE CROSSING"};
statuses={"IMPLEMENTED","PLANNED","PLANNED","PLANNED","PLANNED"};
for k=1:numel(scenarios)
    y=0.73-(k-1)*0.125;
    addText(fig,[0.12 y 0.50 0.075],scenarios{k},16,colors.silver,'left','bold');
    color=colors.amber; if k==1,color=colors.cyan;end
    addText(fig,[0.66 y 0.21 0.075],statuses{k},15,color,'center','bold');
end
addText(fig,[0.10 0.08 0.80 0.08], ...
    "Metrics: collision | completion | minimum TTC | latency | path length | curvature | jerk", ...
    15,colors.teal,'center','normal');
saveSlide(fig,outputDirectory,'05-validation.png');

evaluationDirectory=options.evaluationDirectory;
if strlength(evaluationDirectory)==0
    evaluationDirectory=string(fullfile(root,'artifacts','release','evaluation'));
end
if options.refreshEvaluation
    runPathWeaverEvaluation(outputDirectory=evaluationDirectory);
end
evaluationFile=fullfile(evaluationDirectory,'evaluation_results.mat');
assert(isfile(evaluationFile),'PathWeaver:MissingEvidence','Run the release evaluation before generating measured-result slides.');
evidence=load(evaluationFile);
assert(isfield(evidence,'metadata') && ~evidence.metadata.dirty && ...
    all(ismember({'status','ttcHorizonS','preset'},evidence.results.Properties.VariableNames)), ...
    'PathWeaver:UnverifiedEvidence','Use the current-schema evaluation from a clean revision.');
results=evidence.results;
fig=newSlide(colors); heading(fig,sprintf('PAIRED SIMULATION EVIDENCE — %d RUNS',height(results)));
base=results(results.mode=="baseline",:); risk=results(results.mode=="risk-aware",:);
facts={sprintf('RISK-AWARE\n%d / %d COMPLETE',sum(risk.status=="goal_reached"),height(risk)), ...
    sprintf('OBSERVED\n%d COLLISIONS',sum(risk.status=="collision")), ...
    sprintf('MIN CLEARANCE\n%.3f m',min(risk.minimumClearanceM,[],'omitnan')), ...
    sprintf('MEAN LATENCY\n%.2f ms',mean(risk.averagePlannerLatencyS,'omitnan')*1000)};
for k=1:4
    x=0.055+(k-1)*0.235;
    annotation(fig,'rectangle',[x 0.46 0.20 0.23],'Color',colors.teal, ...
        'FaceColor',colors.panel,'LineWidth',2);
    addText(fig,[x+0.01 0.50 0.18 0.14],facts{k},17,colors.cyan,'center','bold');
end
revision=char(evidence.metadata.revision);
comparison=sprintf(['Baseline: %d/%d complete, %d collisions | Risk-aware: %d/%d complete, %d collisions\n' ...
    '%d invalid / %d timeout runs retained | Source: %s\n' ...
    'Preliminary simulation evidence — not proof of road safety or collision reduction'], ...
    sum(base.status=="goal_reached"),height(base),sum(base.status=="collision"), ...
    sum(risk.status=="goal_reached"),height(risk),sum(risk.status=="collision"), ...
    sum(results.status=="invalid"),sum(results.status=="timeout"),revision(1:min(8,numel(revision))));
addText(fig,[0.09 0.20 0.82 0.14],comparison,16,colors.silver,'center','normal');
saveSlide(fig,outputDirectory,'06-results.png');

fig=newSlide(colors); heading(fig,"SEVEN BEHAVIOUR CODES — SHARED STATEFUL LOGIC");
stateNames={"CRUISE","FOLLOW","YIELD","AVOID","EMERGENCY BRAKE", ...
    "GOAL REACHED","COLLISION"};
stateColors={colors.cyan,colors.teal,colors.amber,colors.teal, ...
    [0.88 0.28 0.24],[0.25 0.82 0.48],[0.88 0.28 0.24]};
for k=1:numel(stateNames)
    if k<=4
        x=0.055+(k-1)*0.235; y=0.56;
    else
        x=0.17+(k-5)*0.25; y=0.31;
    end
    annotation(fig,'rectangle',[x y 0.19 0.14],'Color',stateColors{k}, ...
        'FaceColor',colors.panel,'LineWidth',2);
    addText(fig,[x+0.01 y+0.035 0.17 0.07],stateNames{k},15,stateColors{k},'center','bold');
end
addText(fig,[0.10 0.14 0.80 0.09], ...
    "Stateflow executes the shared kernel on tested sequences. The live vehicle loop remains MATLAB.", ...
    17,colors.silver,'center','normal');
saveSlide(fig,outputDirectory,'stateflow-summary.png');

fig=newSlide(colors); logoAxes=axes(fig,'Position',[0.29 0.62 0.42 0.22]);
image(logoAxes,logo,'AlphaData',alpha); axis(logoAxes,'image','off');
addText(fig,[0.10 0.36 0.80 0.16], ...
    "PathWeaver does not merely plan around where road users are.", ...
    23,colors.silver,'center','normal');
addText(fig,[0.10 0.22 0.80 0.13],"It plans around where they might be.", ...
    27,colors.cyan,'center','bold');
addText(fig,[0.10 0.09 0.80 0.08],"SIH26037  |  BENNETT UNIVERSITY", ...
    15,colors.teal,'center','bold');
saveSlide(fig,outputDirectory,'07-closing.png');
provenance=struct('evaluation',evidence.metadata,'generator',pathweaver.evaluation.environmentInfo(), ...
    'runCount',height(results),'seeds',unique(results.seed),'presets',unique(results.preset));
save(fullfile(outputDirectory,'pitch_provenance.mat'),'provenance');
fprintf('Pitch assets written to %s\n',outputDirectory);
end

function colors=palette()
colors=struct('background',[0.018 0.027 0.035],'panel',[0.045 0.075 0.09], ...
    'cyan',[0.00 0.90 0.88],'teal',[0.20 0.62 0.65], ...
    'silver',[0.82 0.87 0.91],'amber',[0.95 0.63 0.18]);
end
function fig=newSlide(colors)
fig=figure('Visible','off','Color',colors.background,'Position',[100 100 1600 900]);
end
function heading(fig,value)
addText(fig,[0.06 0.86 0.88 0.09],value,25,[0.00 0.90 0.88],'center','bold');
end
function addText(fig,position,value,fontSize,color,alignment,weight)
annotation(fig,'textbox',position,'String',value,'Interpreter','none', ...
    'Color',color,'EdgeColor','none','HorizontalAlignment',alignment, ...
    'VerticalAlignment','middle','FontName','Avenir Next', ...
    'FontSize',fontSize,'FontWeight',weight,'FitBoxToText','off');
end
function saveSlide(fig,outputDirectory,name)
exportgraphics(fig,fullfile(outputDirectory,name),'Resolution',150); close(fig);
end
