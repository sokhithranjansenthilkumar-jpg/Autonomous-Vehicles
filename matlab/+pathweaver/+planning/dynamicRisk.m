function [integratedRisk, hardCollision, minimumSeparationM] = dynamicRisk(positionsWorldM, predictions, cfg, timestampsS, headingsRad)
%DYNAMICRISK Uncalibrated occupancy score with separate swept physical checks.
n=size(positionsWorldM,1);
if nargin<4
    if isempty(predictions), timestampsS=(0:n-1)'*cfg.planDt;
    else, timestampsS=predictions(1).futureTimestampsS; end
end
if nargin<5, headingsRad=zeros(n,1); end
riskByTime=zeros(n,1); hardCollision=false; minimumSeparationM=Inf;
for a=1:numel(predictions)
    prediction=predictions(a);
    assert(numel(prediction.futureTimestampsS)==n && ...
        max(abs(timestampsS-prediction.futureTimestampsS))<1e-8, ...
        'PathWeaver:TimeAlignment','Candidate and prediction timestamps must agree.');
    separation=pathweaver.core.sweptClearance(positionsWorldM,headingsRad, ...
        prediction.expectedPositionsWorldM,prediction.collisionRadiusM,cfg);
    minimumSeparationM=min(minimumSeparationM,separation);
    hardCollision=hardCollision || separation<=cfg.planner.dynamicMarginM;
    for k=1:n
        delta=positionsWorldM(k,:)-prediction.expectedPositionsWorldM(k,:);
        cov=prediction.positionCovariancesWorldM2(:,:,k);
        covariance=cov+diag([(cfg.ego.lengthM/2+prediction.collisionRadiusM)^2, ...
            (cfg.ego.widthM/2+prediction.collisionRadiusM)^2]);
        factor=chol(covariance,'lower'); whitened=factor\delta';
        uncertainty=1+cfg.prediction.uncertaintyBeta*sqrt(trace(cov));
        riskByTime(k)=riskByTime(k)+exp(-0.5*sum(whitened.^2))*uncertainty;
    end
end
integratedRisk=trapz(timestampsS,riskByTime);
end
