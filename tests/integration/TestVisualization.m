function tests=TestVisualization
tests=functiontests(localfunctions);
end
function testPlaybackDoesNotChangeOutcome(t)
cfg=defaultConfig(); cfg.maxSimulationTime=.4;
cfg.visualization.playbackSpeed=.5; view=pathweaver.visualization.TechnicalView(cfg);
cleanup=onCleanup(@()view.dispose());
a=pathweaver.simulation.runSimulation(cfg,struct('onStep',@(f)view.update(f)));
clear cleanup
cfg.visualization.playbackSpeed=4; view=pathweaver.visualization.TechnicalView(cfg);
cleanup=onCleanup(@()view.dispose());
b=pathweaver.simulation.runSimulation(cfg,struct('onStep',@(f)view.update(f)));
fields={'timeS','xWorldM','yWorldM','speedMps','riskScore','minimumTtcS','behavior'};
verifyEqual(t,a.log(:,fields),b.log(:,fields));
clear cleanup
end
function testPauseToggleAndResetSignal(t)
cfg=defaultConfig(); cfg.maxSimulationTime=.1;
r=pathweaver.simulation.runSimulation(cfg); view=pathweaver.visualization.TechnicalView(cfg);
cleanup=onCleanup(@()view.dispose());
view.key('space'); verifyTrue(t,view.Paused); view.key('space'); verifyFalse(t,view.Paused);
view.key('u'); verifyFalse(t,view.ShowUncertainty); view.key('c'); verifyFalse(t,view.ShowCandidates);
view.key('r'); verifyError(t,@()view.update(r.lastFrame),'PathWeaver:ResetRequested');
clear cleanup
end
