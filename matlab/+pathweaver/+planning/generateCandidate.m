function candidate = generateCandidate(ego, lateralTargetM, targetSpeedMps, cfg, speedTransitionS)
%GENERATECANDIDATE Spatial quintic with coherent derivatives and initial pose.
if nargin<5, speedTransitionS=cfg.horizon; end
t=(0:cfg.planDt:cfg.horizon)'; u=min(1,t/speedTransitionS);
% Cubic x-speed profile; true path speed includes lateral motion.
vx0=ego.speedMps*cos(ego.headingRad);
initialAx=ego.accelerationMps2*cos(ego.headingRad);
delta=targetSpeedMps-vx0; aT=initialAx*speedTransitionS;
vx=max(0,vx0+aT*u+(3*delta-2*aT)*u.^2+(-2*delta+aT)*u.^3);
s=cumtrapz(t,vx); distance=s(end);
x=ego.positionWorldM(1)+s;
if distance<1e-6
    y=repmat(ego.positionWorldM(2),size(t));
    headings=repmat(ego.headingRad,size(t));
    curvature=zeros(size(t)); speeds=zeros(size(t));
else
    q=s/distance;
    slope0=tan(ego.headingRad);
    second0=tan(ego.steeringRad)/cfg.ego.wheelbaseM*(1+slope0^2)^1.5;
    a0=ego.positionWorldM(2); a1=slope0*distance; a2=second0*distance^2/2;
    tail=[1 1 1;3 4 5;6 12 20]\[lateralTargetM-a0-a1-a2;-a1-2*a2;-2*a2];
    a=[a0 a1 a2 tail'];
    y=polyval(fliplr(a),q);
    slope=polyval(fliplr((1:5).*a(2:6)),q)/distance;
    second=polyval(fliplr((1:4).*(2:5).*a(3:6)),q)/distance^2;
    headings=atan(slope); curvature=second./(1+slope.^2).^1.5;
    speeds=vx.*sqrt(1+slope.^2);
end
acceleration=gradient(speeds,t);
names={'dynamicRisk','staticObstacle','boundary','smoothness','curvature', ...
    'jerk','progress','routeDeviation','time'};
terms=cell2struct(num2cell(zeros(size(names))),names,2);
candidate=struct('timestampsS',ego.timestampS+t,'positionsWorldM',[x y], ...
    'headingsRad',headings,'speedsMps',speeds,'accelerationsMps2',acceleration, ...
    'curvaturesPerM',curvature,'costTerms',terms,'weightedCostTerms',terms, ...
    'totalCost',Inf,'isFeasible',true,'isEmergencyBraking',false,'rejectionReason',"");
end
