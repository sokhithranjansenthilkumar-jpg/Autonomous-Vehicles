function [code,clearSince,reason]=stepDecision(previous,clearSince,timeS,risk,ttc,emergency,feasible,goal,collision,lateral,nearest,limits)
%STEPDECISION Numeric stateful reference shared by MATLAB and compiled Stateflow.
% Codes: cruise=0 follow=1 yield=2 avoid=3 emergency=4 goal=5 collision=6.
% Limits: emergencyTTC, yieldTTC, followDistance, riskYield, riskClear, dwell, TTC margin.
code=0; reason=1;
if previous==5 || previous==6
    code=previous; reason=2;
elseif collision
    code=6; reason=3;
elseif goal
    code=5; reason=4;
elseif emergency || isnan(ttc) || ttc<limits(1)
    code=4;
    if ~feasible, reason=5;
    elseif emergency, reason=6;
    elseif isnan(ttc), reason=7;
    else, reason=8; end
elseif ttc<limits(2) || risk>limits(4)
    code=2; reason=9;
elseif lateral
    code=3; reason=10;
elseif nearest<limits(3)
    code=1; reason=11;
end
if previous==4 && code~=5 && code~=6
    clearNow=ttc>limits(2)+limits(7) && risk<limits(5) && ~emergency;
    if clearNow
        if isnan(clearSince), clearSince=timeS; end
        if timeS-clearSince<limits(6)
            code=previous; reason=12;
        end
    else
        clearSince=NaN; code=previous; reason=13;
    end
else
    clearSince=NaN;
end
end
