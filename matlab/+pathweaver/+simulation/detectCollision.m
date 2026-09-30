function [collision, reason, clearance] = detectCollision(ego, scenario, cfg, previousEgo, previousScenario)
%DETECTCOLLISION Swept conservative physical footprints, no risk inflation.
if nargin<4
    positions=ego.positionWorldM; headings=ego.headingRad;
    previousScenario=scenario;
else
    positions=[previousEgo.positionWorldM;ego.positionWorldM];
    headings=[previousEgo.headingRad;ego.headingRad];
end
clearance=pathweaver.core.boundaryClearance(positions,headings,cfg);
reason="road boundary";
for k=1:numel(scenario.staticObstacles)
    obs=scenario.staticObstacles(k);
    d=pathweaver.core.sweptClearance(positions,headings,obs.positionWorldM,obs.geometry.radiusM,cfg);
    if d<clearance, clearance=d; reason="static obstacle"; end
end
for k=1:numel(scenario.agents)
    actor=scenario.agents(k).positionWorldM;
    if size(positions,1)>1, actor=[previousScenario.agents(k).positionWorldM;actor]; end
    d=pathweaver.core.sweptClearance(positions,headings,actor,scenario.agents(k).collisionRadiusM,cfg);
    if d<clearance, clearance=d; reason="agent "+string(scenario.agents(k).id); end
end
collision=clearance<=0;
if ~collision, reason=""; end
end
