function predictions = predictAgents(agents, currentTimeS, cfg)
%PREDICTAGENTS Class-conditioned constant-velocity future distributions.
tau = (0:cfg.planDt:cfg.horizon)';
template = struct('agentId', 0, 'class', "", 'futureTimestampsS', [], ...
    'expectedPositionsWorldM', [], 'positionCovariancesWorldM2', [], ...
    'collisionRadiusM', 0);
predictions = repmat(template, 1, numel(agents));
for a = 1:numel(agents)
    agent = agents(a);
    assert(all(isfinite([agent.positionWorldM agent.velocityWorldMps agent.timestampS])) && ...
        abs(agent.timestampS-currentTimeS)<1e-8,'PathWeaver:InvalidAgent','Invalid or stale actor state.');
    means = agent.positionWorldM + tau.*agent.velocityWorldMps;
    covariances = zeros(2, 2, numel(tau));
    growth = cfg.prediction.(char(agent.class));
    speed = norm(agent.velocityWorldMps);
    if speed > 1e-9
        forward = agent.velocityWorldMps/speed;
        rotation = [forward(1) -forward(2); forward(2) forward(1)];
    else
        rotation = eye(2);
    end
    for k = 1:numel(tau)
        if cfg.mode == "baseline"
            covariance = agent.positionCovarianceWorldM2;
        else
            localGrowth = diag(growth.*tau(k) + 0.5*growth.*tau(k).^2);
            covariance = agent.positionCovarianceWorldM2 + ...
                rotation*localGrowth*rotation';
        end
        covariances(:,:,k) = pathweaver.prediction.projectCovariance( ...
            covariance, cfg.prediction.varianceFloorM2);
    end
    predictions(a) = struct('agentId', agent.id, 'class', agent.class, ...
        'futureTimestampsS', currentTimeS + tau, ...
        'expectedPositionsWorldM', means, ...
        'positionCovariancesWorldM2', covariances, ...
        'collisionRadiusM', agent.collisionRadiusM);
end
end
