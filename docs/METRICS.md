# Geometry and metric definitions

All positions are world-frame metres: x forward along the road, y lateral,
heading counterclockwise from +x in radians. Speed is m/s, acceleration m/s²,
steering radians, time seconds, covariance m². The virtual kinematic bicycle
reference point is also the body centre; this is a simplified vehicle model,
not a rear-axle calibrated dynamics model.

## Contact and clearance

The 4.2 × 1.8 m ego rectangle is conservatively covered by three equal discs.
Their longitudinal offsets are −1.4, 0, +1.4 m; radius is
sqrt(0.7² + 0.9²) = 1.1402 m. Pedestrian and two-wheeler physical disc radii
are 0.42 m and 0.75 m. The pothole is a forbidden 0.85 m disc.
These same physical footprints are used for candidate rejection, observed
contact, clearance and TTC. Planning additionally requires 1.25 m dynamic
and 0.35 m pothole clearance, identically in both modes.

Observed contact means overlap during the executed simulation, including the
segment between consecutive 0.05 s integration endpoints. Relative linear
segment minima detect fast encounters between samples. A heading-interpolation
sagitta bound covers rotation of the offset discs. Candidate checks use the
same sweep between 0.2 s knots. Curved polynomial paths are executed through
sampled tracking; constraints describe the sampled, swept representation,
not an exact collision proof for every point on an analytic quintic.

Road clearance uses the analytical irregular boundaries and a Lipschitz slope
bound, includes road end caps, and bounds between-sample variation. Reported
minimum clearance is the minimum conservative signed gap over the executed
run, including actors, pothole and boundaries. Zero is contact; negative means
overlap under this conservative model. It is not just actor centre distance.

## TTC is not closest approach

TTC predicts first footprint contact assuming ego maintains its **current
world velocity and heading**, each actor maintains current world velocity,
and the pothole remains static. It does not extrapolate future steering,
braking or uncertain motion. It excludes road-edge TTC; boundaries have their
own swept contact check. This is a **3 s finite-horizon** prediction.

For relative centre p, velocity v and summed disc radius R, solve
||p + t v||² = R² for the earliest nonnegative entry time, over all ego discs
and obstacles. This analytic solution cannot miss contact between samples.
Time to closest approach is separately clamped −(p·v)/(v·v), and may be finite
when TTC is infinite. Distance divided by relative-speed magnitude is not used.

- Existing overlap: TTC = 0.
- No contact predicted within 3 s: Inf, displayed “none within 3.0 s”.
- Invalid or stale input: NaN, displayed “unavailable”; behaviour fails closed.
- Finite TTC: time to first predicted contact, not observed collision.

Run minimum TTC considers available samples; unavailable sample count is
exported separately. If all samples are unavailable the minimum is NaN.
Inf does not mean no collision is possible beyond the horizon.
Uncertainty risk is an uncalibrated Gaussian-distance **risk score**, not TTC
and not a calibrated collision probability.

## Timing and comfort

Scenario, ego and world snapshots share one clock. Predictions and candidates
start at the same snapshot and share 0.2 s knots over 3 s; mismatches raise an
error. Integration uses 0.05 s, replanning 0.15 s. Display may show the last
plan; its timestamp/age is labelled. Rendering and playback never advance time.

Planner wall time includes prediction, candidate generation, constraints,
scoring and selection. It excludes control, metrics, rendering and export.
One sample is recorded per planning call; the first two calls of **each run**
are excluded from mean, maximum and nearest-rank p95. Counts and environment
are exported. Whole-run wall time is a separate quantity.

Applied acceleration is Δspeed/Δt (zero when braking at rest). Jerk is
Δacceleration/Δt; integrated absolute jerk is sum(|Δacceleration|), in m/s².
This includes startup and braking discontinuities: no smoothing hides them.
Curvature is tan(steering)/wheelbase, reported only while moving; standstill
contributes zero. Path length sums executed position increments. Completion
requires the ego reference point within the 3 m goal region. Status separates
goal_reached, timeout, collision and invalid. Time to goal is NaN otherwise.

When all candidates fail, the selected response is a rollout of bounded
emergency braking and rate-limited steering return. It remains labelled
**infeasible; safety not guaranteed**. Emergency entries are counted, not
every frame spent braking. A collision still terminates and remains in results.

The same bounded-braking rollout also participates in candidate selection before
infeasibility. It must pass physical contact, road, acceleration and steering
constraints. Only its comfort jerk threshold is soft, not a hard rejection:
emergency stopping may be uncomfortable. Ordinary candidates retain the hard
8 m/s³ jerk threshold. Both modes receive the identical braking option.

## Reproducible village variants

| Preset | Ego initial speed | Pedestrian start (x, y) | Crossing onset | Crossing speed |
| --- | --- | --- | --- | --- |
| nominal | 8 m/s | (49, −4.2) m | 3.65 s | 1.25 m/s |
| challenging | 9 m/s | (47, −4.2) m | 3.00 s | 1.65 m/s |
| emergency | 8 m/s | (25, −3.4) m | 1.00 s | 2.40 m/s |

Each seed adds independent normal x offsets (σ = 0.6 m) to the pedestrian
and two-wheeler. The latter starts at (103, 2.3) m, travelling at −5.2 m/s
along x. Actual parameters are exported; table positions are before jitter.
Onset means the actor starts walking continuously, not that it teleports or
is hidden from the world model. The emergency variant tests a rapid intrusion
with finite stopping room, not an unavoidable collision claim. All are variants
of one MATLAB scenario family, not three RoadRunner scenes.

The local mt19937ar scenario stream never resets global randomness. The planner
is deterministic and uses no random stream. Physical motion depends only on
scenario time and configuration, not ego position, planner mode or covariance.

## Paired evaluation protocol

Default evaluation uses 27001:27010, disjoint from demo/tuning seed 26037,
across all three presets and both modes (60 runs). Modes receive identical
physical states, exogenous actor timing, footprints, clearance margins,
candidate families, horizon, costs, controller and termination rules. Only
covariance propagation differs: baseline keeps the supplied covariance fixed;
risk-aware grows it by class. Both use the same uncalibrated risk formula.
There is no random planner, privileged actor schedule, or baseline handicap.

Mode execution order alternates with seed parity. Every run is saved, and the
per-run CSV/MAT is checkpointed after each result. Exceptions become explicit
invalid rows with unavailable numerical outcomes, an error identifier and raw
configuration. Timeouts and collisions are not discarded. Exports include
actual scenario parameters, Git revision/dirty flag and environment inventory.
Summary latency columns are explicitly **means of per-run means/p95s**, not
pooled percentiles. Completion-time plots exclude non-completions but label
their counts; outcome tables retain all runs. Zero observed collisions in this
small scenario family does not establish safety or a general improvement.
