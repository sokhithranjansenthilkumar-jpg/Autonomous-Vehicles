# Algorithms

Defaults live in `config/defaultConfig.m`; physical variants live in
`config/scenarioConfig.m`. [Metrics](METRICS.md) specifies collision geometry,
TTC semantics and the paired evaluation protocol.

## Class-conditioned prediction

For current world position p and velocity v:

```text
mu(tau) = p + v*tau
Sigma(tau) = projectPSD(Sigma0 + R*diag(g)*R' * (u + 0.5*u^2))
u = tau / (1 second)
```

R aligns growth with current velocity, or the world axes at standstill. The
numerical coefficients g set variance growth on a one-second design scale:

| Class | Forward coefficient | Lateral coefficient |
| --- | ---: | ---: |
| pedestrian | 0.25 | 0.45 |
| two_wheeler | 0.18 | 0.28 |
| car | 0.10 | 0.16 |
| animal | 0.30 | 0.55 |

Covariance is symmetrized and eigenvalues floored at 0.04 m². Invalid covariance
is rejected. Baseline uses the same CV mean and initial covariance but no growth.
Physical actor trajectories never depend on these coefficients.

This is a configurable uncertainty heuristic, not learned intent, a calibrated
forecast, or a prediction of crossing onset before velocity changes. Animals
and cars are supported prediction classes, not additional implemented scenes.

## Time-indexed risk

For an aligned ego sample q and agent prediction mu, Sigma:

```text
S = Sigma + diag([(egoLength/2 + agentRadius)^2,
                  (egoWidth/2  + agentRadius)^2])
d2 = (q - mu)' * inv(S) * (q - mu)
r = exp(-0.5*d2) * (1 + beta*sqrt(trace(Sigma)))
C_dynamicRisk = integral(sum_over_agents(r), dt)
```

Implementation uses a Cholesky solve, not matrix inversion. beta is 0.15
numerically in inverse metres. The footprint-shaped soft-risk ellipse is
world-axis-aligned; the hard physical checks use the heading-dependent covering
discs. Pointwise risk is an uncalibrated score, potentially greater than one;
its time integral has score-seconds units. It is **not collision probability**.
Expanding covariance broadens the cost around uncertain agents. Both modes use
this same formula; only covariance propagation differs.

Definite predicted contact and clearance-margin violations are hard constraints,
separate from risk. No Mahalanobis contour is treated as a physical collision.

## Candidate generation and selection

The sampler is world-road-forward, not lane-centred or a general Frenet planner.
It combines lateral targets −2.1, −1.05, 0, 1.05, 2.1 m and current y with target
speeds 8, 5.2, 2, 0 m/s. Additional stopping profiles transition over 2.2 and
2.6 s. All share the 3 s prediction horizon. A separately rolled-out bounded
emergency stop is also scored: normally 31–37 candidates per replan.

Longitudinal x-speed is a cubic with current speed/projected acceleration and
target speed/zero terminal acceleration. Nonnegative x-speed is integrated
numerically. A spatial quintic y(s) matches current y, heading and curvature,
then the target y with zero terminal slope/curvature. Analytic spatial
derivatives give heading, path speed and curvature; time differences give
acceleration and jerk. A stationary zero-speed candidate cannot slide sideways.
Normal proposals are geometric sampled trajectories, not exact dynamic rollouts.

Reject nonfinite signals, swept drivable-boundary violations, swept static or
dynamic footprint/margin contact, acceleration outside −5.5…+2.8 m/s², excessive
curvature/steering, steering rate above 50°/s, or ordinary jerk above 8 m/s³.
The emergency candidate retains all physical constraints but treats jerk as a
soft comfort cost. It is generated using the actual bounded emergency controller
and bicycle step, not a safe animation.

Select lowest total cost, then greatest terminal x, then generation order.
If every candidate fails, return an explicitly infeasible bounded-braking
response. It is **not labelled safe**; unavoidable contact can still occur.

## Inspectable cost

J is the sum of the following weighted raw terms. These are engineering
trade-offs with mixed raw units, not calibrated or uniformly normalized costs.

| Term | Calculation | Default weight |
| --- | --- | ---: |
| dynamicRisk | Integrated time-aligned uncertainty risk | 28 |
| staticObstacle | Integral of inverse squared pothole clearance | 12 |
| boundary | Integral of inverse squared road clearance | 8 |
| smoothness | Integral of acceleration squared | 0.25 |
| curvature | Integral of curvature squared | 2 |
| jerk | Integral of jerk squared | 0.15 |
| progress | Remaining forward goal distance / road length | 20 |
| routeDeviation | Mean squared world y over horizon | 0.35 |
| time | Reciprocal terminal speed, floored at 0.25 m/s | 5 |

Inverse-clearance denominators are floored at 0.1 m only for numerical scoring;
hard constraints run first and are not clipped away. The time term is a speed
proxy, not an optimized travel-time estimate. A soft y=0 preference does not
impose a lane. Both raw and weighted terms remain available on every scored
candidate; rejected candidates have a rejection reason and infinite total cost.

## Behaviour

Priority is terminal-state retention, observed collision, goal, emergency,
yield, lateral avoidance, nearby-user following label, then cruise.

| Condition | Result |
| --- | --- |
| Infeasible fallback or explicitly selected emergency profile | EMERGENCY_BRAKE |
| TTC unavailable or below 0.75 s | EMERGENCY_BRAKE |
| TTC below 3.5 s or integrated risk above 0.9 | YIELD |
| Selected terminal lateral displacement exceeds 0.6 m | AVOID |
| Any actor within 14 m | FOLLOW |
| Otherwise | CRUISE |

Because TTC has a 3 s horizon, the 3.5 s yield threshold includes any finite
predicted contact in that horizon. Emergency recovery requires TTC above 4.3 s
(normally Inf), risk below 0.45, and no emergency flag continuously for 0.5 s.
A failed clearance condition resets dwell memory. Goal/collision are terminal.

CRUISE/FOLLOW/YIELD/AVOID describe the selected situation; they do not invoke
separate scripted ego manoeuvres. FOLLOW is a proximity label, not a verified
car-following policy. Only emergency overrides tracking. Normal labels can
change rapidly; hysteresis applies specifically to emergency recovery.

The MATLAB adapter and separate Stateflow execution harness share the numeric
stateful kernel. Tests cover declared sequences, not formal equivalence over
every input or a Stateflow-controlled vehicle loop.

## Tracking and dynamics

Pure pursuit chooses a forward, non-stale trajectory point with lookahead
2.5 + 0.35*v metres. Steering is bounded to ±30° and its change to 50°/s.
Longitudinal control uses trajectory-time speed interpolation, acceleration
feedforward, proportional gain 1.3 and integral gain 0.15. Integral error is
bounded to ±5. This is bounded PI, not MPC.

Emergency commands −5.5 m/s², resets the speed integral, and rate-limits steering
back toward zero. Explicit Euler updates the virtual bicycle:

```text
positionNext = position + v*[cos(heading), sin(heading)]*dt
headingNext = wrap(heading + v*tan(steering)/wheelbase*dt)
speedNext = max(0, v + accelerationCommand*dt)
```

Wheelbase is 2.7 m; applied acceleration is measured from the actual speed change,
so braking at rest does not produce fictitious acceleration or jerk. This
low-speed planar approximation omits slip, actuator lag, suspension and tire
forces. It is not production vehicle dynamics.
