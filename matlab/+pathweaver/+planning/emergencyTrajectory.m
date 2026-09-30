function trajectory = emergencyTrajectory(ego,cfg)
%EMERGENCYTRAJECTORY Roll out the same bounded emergency actuator response.
trajectory=pathweaver.planning.generateCandidate(ego,ego.positionWorldM(2),0,cfg);
state=pathweaver.control.initialControllerState(); state.previousSteeringRad=ego.steeringRad;
current=ego;
for k=1:numel(trajectory.timestampsS)
    while current.timestampS<trajectory.timestampsS(k)-1e-9
        [command,state]=pathweaver.control.trackTrajectory(current,trajectory,true,state,cfg);
        current=pathweaver.control.updateBicycle(current,command,cfg.dt,cfg);
    end
    trajectory.positionsWorldM(k,:)=current.positionWorldM;
    trajectory.headingsRad(k)=current.headingRad;
    trajectory.speedsMps(k)=current.speedMps;
    trajectory.accelerationsMps2(k)=current.accelerationMps2;
    trajectory.curvaturesPerM(k)=tan(current.steeringRad)/cfg.ego.wheelbaseM;
end
trajectory.isFeasible=false;
trajectory.isEmergencyBraking=true;
trajectory.totalCost=Inf;
trajectory.rejectionReason="no feasible candidate; bounded braking, safety not guaranteed";
end
