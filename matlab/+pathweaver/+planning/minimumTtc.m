function [ttcS, details] = minimumTtc(ego, agents, horizonS, cfg, obstacles)
%MINIMUMTTC First predicted footprint contact under fixed-heading CV motion.
if nargin<4, cfg=defaultConfig(); end
if nargin<5, obstacles=[]; end
ttcS=NaN;
details=struct('horizonS',horizonS,'available',false,'timeToClosestApproachS',NaN, ...
    'predictedMinimumClearanceM',NaN);
try
    pathweaver.core.validateEgoState(ego);
    assert(isfinite(horizonS) && horizonS>0);
    [centres,radius]=pathweaver.core.egoFootprint(ego.positionWorldM,ego.headingRad,cfg);
    velocity=ego.speedMps*[cos(ego.headingRad) sin(ego.headingRad)];
    for k=1:numel(obstacles)
        obstacle=obstacles(k);
        actor=struct('positionWorldM',obstacle.positionWorldM,'velocityWorldMps',[0 0], ...
            'collisionRadiusM',obstacle.geometry.radiusM,'timestampS',ego.timestampS);
        if isempty(agents), agents=actor; else
            names=fieldnames(actor);
            added=agents(1);
            for n=1:numel(names), added.(names{n})=actor.(names{n}); end
            agents(end+1)=added; %#ok<AGROW>
        end
    end
    best=Inf; clearance=Inf; closest=NaN;
    for k=1:numel(agents)
        assert(abs(agents(k).timestampS-ego.timestampS)<1e-8);
        for j=1:3
            [t,d,tc]=pathweaver.core.circleEncounter( ...
                agents(k).positionWorldM-centres(:,:,j), ...
                agents(k).velocityWorldMps-velocity, ...
                radius+agents(k).collisionRadiusM,horizonS);
            if isnan(t), return; end
            best=min(best,t);
            if d<clearance, clearance=d; closest=tc; end
        end
    end
    ttcS=best; details.available=true;
    details.timeToClosestApproachS=closest;
    details.predictedMinimumClearanceM=clearance;
catch
    % Invalid world data is unavailable; the behaviour layer fails closed.
end
end
