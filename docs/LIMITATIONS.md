# Limitations and next phases

This is a simulation research prototype, not a road-ready autonomy system.
It must not control a real vehicle.

## Current boundaries

- Input is simulated actor ground truth. Camera, LiDAR, radar, detection,
  tracking and sensor fusion are not implemented.
- Three presets are variants of one programmatic MATLAB village-road family,
  not three independent RoadRunner scenes. Only pedestrian and two-wheeler
  actors appear; car/animal prediction classes alone are not scene coverage.
- Actors follow scripted exogenous motion. CV prediction does not infer intent,
  interactions or crossing onset. Uncertainty coefficients and risk are
  heuristic, not learned or calibrated probabilities.
- A local forward sampler and low-speed virtual bicycle omit reverse motion,
  slip, actuator lag and production dynamics. Covering discs are conservative
  approximations, including for the two-wheeler.
- Physical checks sweep sampled poses; this is not a formal continuous-time
  safety proof for arbitrary trajectories. Tracking can differ from a proposal.
  A bounded brake may still collide when a situation is unavoidable.
- TTC assumes fixed heading/current velocity over 3 s; it does not predict the
  actual controlled future. Inf means no predicted contact **within the horizon**.
- Normal behaviour labels may flicker. Emergency recovery has tested hysteresis.
- A small paired sample can suggest efficiency/comfort differences, not establish
  collision reduction, statistical generality or road safety.
- Full vehicle execution remains in MATLAB. The Simulink vehicle model replays
  recorded signals. A separate Stateflow harness executes the shared behaviour
  kernel on test inputs; the demo does not use Stateflow as its controller.
- No native RoadRunner project, scene, sensor simulation or bridge is present.
- No trained ML, Hybrid A*, MPC, SIL/HIL or real-road validation is claimed.
- Display pacing is best-effort under graphics load, not a real-time guarantee.
  Simulation time and encoded video cadence remain independent of wall time.

## Next implementation phases

1. **Robustness and model validation.** Add delayed/noisy tracks, missed detections,
   wider encounter timing/speed variation, unavoidable intrusions and controlled
   failure cases. Calibrate uncertainty, inspect tracking error and extend held-out
   evaluation with explicit regression thresholds.
2. **Native integration.** On a supported licensed RoadRunner host, reconstruct
   the existing scene, validate coordinate/time mapping and close an actor-truth
   bridge to MATLAB. Incrementally execute planning/control in Simulink with
   numerical equivalence tests; retain the working MATLAB reference.
3. **Perception and scenario breadth.** Implement detection, fusion and track
   lifecycle management behind the world-state adapter. Add intersection, merge,
   market and cattle-crossing scenarios, then evaluate higher-fidelity vehicle
   dynamics and interaction-aware prediction.

Each phase needs executed evidence before its capabilities appear in the pitch.
