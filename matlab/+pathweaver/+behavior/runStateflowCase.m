function code=runStateflowCase(modelPath,conditions)
%RUNSTATEFLOWCASE One reset direct-decision case through the sequence harness.
row=[conditions.riskScore conditions.minimumTtcS conditions.emergencyFlag ...
    conditions.goalReached conditions.collisionFlag conditions.lateralAvoid conditions.leadDistanceM 1];
output=pathweaver.behavior.runStateflowSequence(modelPath,(0:.05:.1)',repmat(row,3,1));
code=output.stateCode(end);
end
