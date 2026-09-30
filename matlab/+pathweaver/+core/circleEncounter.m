function [ttc, clearance, tClosest] = circleEncounter(relativePosition, relativeVelocity, radius, horizon)
%CIRCLEENCOUNTER Exact finite-horizon contact time for two translating discs.
% Inf = no contact in horizon; NaN = invalid; zero = existing overlap.
ttc=NaN; clearance=NaN; tClosest=NaN;
if numel(relativePosition)~=2 || numel(relativeVelocity)~=2 || ...
        any(~isfinite([relativePosition(:);relativeVelocity(:);radius;horizon])) || ...
        radius<0 || horizon<0
    return
end
p=relativePosition(:); v=relativeVelocity(:);
a=dot(v,v); b=dot(p,v); c=dot(p,p)-radius^2;
tClosest=0;
if a>eps, tClosest=min(horizon,max(0,-b/a)); end
clearance=norm(p+tClosest*v)-radius;
if c<=0, ttc=0; return; end
ttc=Inf;
if a<=eps || b>=0, return; end
discriminant=b*b-a*c;
if discriminant<0, return; end
entry=c/(-b+sqrt(discriminant)); % stable first root
if entry<=horizon, ttc=entry; end
end
