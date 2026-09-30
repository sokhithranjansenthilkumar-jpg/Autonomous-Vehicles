function limits=decisionLimits(cfg)
%DECISIONLIMITS Fixed numeric order used by the simulation-time chart.
b=cfg.behavior;
limits=[b.emergencyTtcS b.yieldTtcS b.followDistanceM b.riskYield b.riskClear b.clearDwellS b.clearTtcMarginS];
end
