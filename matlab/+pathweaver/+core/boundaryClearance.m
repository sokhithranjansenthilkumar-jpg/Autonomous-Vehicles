function [clearance,sampleClearance] = boundaryClearance(positions, headings, cfg)
%BOUNDARYCLEARANCE Conservative disc-to-road clearance, including sample gaps.
[centres,radius]=pathweaver.core.egoFootprint(positions,headings,cfg);
slopeBound=0.18/15+0.08/5.5+0.18/21;
clearance=Inf;
sampleClearance=inf(size(positions,1),1);
for j=1:3
    p=centres(:,:,j); [left,right]=pathweaver.core.roadBounds(p(:,1),cfg.scenario);
    d=min([(left-p(:,2))/sqrt(1+slopeBound^2), ...
        (p(:,2)-right)/sqrt(1+slopeBound^2),p(:,1), ...
        cfg.scenario.roadLengthM-p(:,1)],[],2)-radius;
    sampleClearance=min(sampleClearance,d);
    if numel(d)>1
        rotationBound=cfg.ego.lengthM/3*(1-cos(abs(pathweaver.core.wrapAngle(diff(headings)))/2));
        d=min(d(1:end-1),d(2:end))-slopeBound*abs(diff(p(:,1)))/2-rotationBound;
    end
    clearance=min(clearance,min(d));
end
end
