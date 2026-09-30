function code = stateCode(name)
%STATECODE Stable numeric encoding shared with the Stateflow chart.
names=["CRUISE","FOLLOW","YIELD","AVOID","EMERGENCY_BRAKE","GOAL_REACHED","COLLISION"];
code=find(names==string(name),1)-1;
assert(~isempty(code),'PathWeaver:UnknownBehavior','Unknown behaviour state.');
end
