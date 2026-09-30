function replay = prepareSimulinkReplay(durationS)
%PREPARESIMULINKREPLAY Return recorded signals; no algorithm runs in replay blocks.
if nargin<1, durationS=2; end
cfg=defaultConfig(); cfg.maxSimulationTime=durationS;
run=pathweaver.simulation.runSimulation(cfg);
codes=arrayfun(@pathweaver.behavior.stateCode,run.log.behavior);
risk=run.log.riskScore;
values=[run.log.xWorldM run.log.yWorldM run.log.speedMps ...
    run.log.selectedEndXWorldM run.log.selectedEndYWorldM codes risk ...
    isfinite(risk) isfinite(run.log.selectedEndXWorldM)];
replay=[run.log.timeS values];
end
