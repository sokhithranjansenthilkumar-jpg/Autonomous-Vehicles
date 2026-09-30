function clearance = sweptClearance(positions, headings, actorPositions, actorRadius, cfg)
%SWEPTCLEARANCE Continuous contact test between piecewise-linear pose samples.
assert(all(isfinite([positions(:);headings(:);actorPositions(:);actorRadius])) && ...
    isscalar(actorRadius) && actorRadius>=0,'PathWeaver:InvalidGeometry','Footprints must be finite with nonnegative radius.');
[centres,radius]=pathweaver.core.egoFootprint(positions,headings,cfg);
if size(actorPositions,1)==1, actorPositions=repmat(actorPositions,size(positions,1),1); end
assert(isequal(size(actorPositions),size(positions)),'PathWeaver:TimeAlignment','Pose grids must match.');
clearance=Inf;
for j=1:3
    relative=centres(:,:,j)-actorPositions;
    if size(relative,1)==1
        d=norm(relative)-radius-actorRadius;
    else
        start=relative(1:end-1,:); delta=diff(relative);
        fraction=max(0,min(1,-sum(start.*delta,2)./max(sum(delta.^2,2),eps)));
        closest=start+fraction.*delta;
        angle=abs(pathweaver.core.wrapAngle(diff(headings)));
        rotationBound=cfg.ego.lengthM/3*(1-cos(angle/2));
        d=min(vecnorm(closest,2,2)-radius-actorRadius-rotationBound);
    end
    clearance=min(clearance,d);
end
end
