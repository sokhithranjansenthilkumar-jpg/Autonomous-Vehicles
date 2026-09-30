# BU SIH-26 Phase 2 demonstration

Team PathWeaver · Bennett University · SIH26037

## Before presenting

Open this repository as MATLAB's Current Folder. Connect power, check the
projector, and run these commands before judges arrive:

```matlab
setupPath
runPathWeaverTests;
runPathWeaverDemo(preset="nominal", visualization=false);
runPathWeaverDemo(preset="challenging", visualization=false);
runPathWeaverDemo(preset="emergency", visualization=false);
```

The verified environment is [R2026a Update 5 on Apple silicon](ENVIRONMENT.md).
The core requires MATLAB; optional integration tests additionally require
Simulink/Stateflow. Do not install or reconfigure toolboxes during the pitch.

## Launch and controls

```matlab
runPathWeaverDemo(preset="nominal", seed=26037, plannerMode="risk-aware", playbackSpeed=4);
runPathWeaverDemo(preset="challenging", seed=26037, playbackSpeed=4);
runPathWeaverDemo(preset="emergency", seed=26037, playbackSpeed=4);
```

Use the same commands with `plannerMode="baseline"` for comparison.
Options also include `visualization=false`, `video=true`, `startPaused=true`,
`showCandidates=false`, `showUncertainty=false`, `maximumSimulationTime`
and `outputDirectory`.

- Space / Pause: pause or resume; R / Reset: rerun from the same seed/configuration.
- U / C: uncertainty and candidate display toggles.
- Playback menu: 0.5×, 1×, 2× or 4×; graphics load can limit the achieved speed.
- Candidate menu: inspect the actual selected proposal or any alternative,
  including its rejection reason.
- Esc or closing the demo window: stop. An interrupted run is not exported as
  a completed run.

Reset during a run discards that partial attempt. Re-running a preset into the
same output directory replaces its previous generated result/video; use separate
directories when retaining recordings. None of these controls changes physics.

## Three-minute walkthrough

Prepare the commands above in the Command Window. Approximate wall-clock timing
below assumes a responsive projector; pause briefly at the relevant **simulation**
times rather than rushing the narration.

| Time | Action and narration |
| --- | --- |
| 0:00–0:15 | “We are PathWeaver from Bennett University. Our thesis is: plan around where road users might be, not only where they are. This is a MATLAB simulation using actor ground truth—not working cameras or LiDAR.” |
| 0:15–1:05 | Launch nominal. Around simulated 3.8–4.8 s, briefly pause: “The pedestrian's predicted mean is amber. These expanding ellipses show configured uncertainty at one, two and three seconds. The planner evaluates multiple paths and a bounded braking option.” Resume and show progress toward the goal. |
| 1:05–1:35 | Launch challenging. “The pedestrian starts earlier and walks faster, with a faster initial ego. The same planner and controller run unchanged.” Show the interaction; use Esc if needed for time. Do not claim completion for an interrupted preview. |
| 1:35–2:05 | Launch emergency. Around simulated 1–3 s: “This rapid intrusion exercises bounded braking. The displayed acceleration and state come from control execution. This case has stopping room; emergency braking is not a promise that every intrusion is avoidable.” |
| 2:05–2:35 | Show the saved comparison plot and [release results](RELEASE.md): “Both modes use the same physical encounters. Our small sample shows the measured outcomes here; it does not establish road safety or a collision-reduction advantage.” |
| 2:35–2:55 | “The complete vehicle loop runs in MATLAB. Simulink's vehicle model is replay. A separate Stateflow harness executes our shared behaviour logic and matches the tested state-memory sequences. RoadRunner and sensor fusion are the next integration phase.” |
| 2:55–3:00 | “PathWeaver plans around where road users might be.” |

If all three full live runs fit, let them finish. Otherwise show the key
interactions and use the verified full-run evidence for completion claims.
The backup nominal clip is the best approximately 25-second recording for a pitch.

## Reading the display

Cyan is the selected feasible proposal; gray is a readable subset of feasible
alternatives; muted red is rejected proposals. An infeasible selected braking
response is red and explicitly not guaranteed safe. All candidates remain in
the inspector even when only a subset is drawn.

Ego has a rectangular body with the three covering-disc outlines used by the
conservative checks. Amber actor discs are physical footprints; the ellipses
are 2-sigma uncertainty contours, not calibrated “95% safe” regions.

The cost chart shows actual weighted components. “Emergency override” identifies
when behaviour overrides normal tracking, including during clearance dwell.
Clearance includes road edges and pothole, not just moving actors.

CV TTC predicts footprint contact over three seconds under constant current
velocity/heading. “None within 3.0 s” is not infinite safety. “Unavailable” means
invalid/missing input. Risk is a separate uncalibrated planning score.

## Evidence and recovery

```matlab
% Reproduce the paired 60-run comparison:
runPathWeaverEvaluation(outputDirectory="artifacts/release/evaluation");

% Regenerate the real backup video and provenance:
recordPathWeaverEvidence(preset="nominal");

% Optional pitch slides, using the clean-revision benchmark above:
generatePathWeaverPitchAssets(outputDirectory="artifacts/release/pitch");

% Optional integration evidence:
buildPathWeaverModel;
buildPathWeaverStateflowModel;
```

Backup: `artifacts/release/backup/nominal/pathweaver_demo.mp4`.
Its `recording_provenance.mat` records preset, seed, mode, source revision,
dirty flag, simulated/encoded duration and frame rate. A real snapshot is saved
alongside it. Generated evidence is ignored by Git, so regenerate it on a clone.

If the GUI stalls, press Esc and run `runPathWeaverDemo(visualization=false)`.
Use the backup MP4 from a normal video player; do not attempt toolbox repairs
during judging. Keep the comparison PNG open in advance. Tests can be launched
non-interactively with `matlab -batch "runPathWeaverTests;"` when MATLAB is on PATH.

## Judge questions

**What makes this lane-agnostic?** It checks free-space boundaries and obstacle
footprints, not lane IDs or painted centres. A soft road-forward preference
encourages progress but does not constrain the vehicle to a lane.

**What does uncertainty represent?** Configurable short-horizon model uncertainty,
growing faster for pedestrians/animals. It is not calibrated sensor covariance
or learned intent.

**Why constant velocity?** It is transparent, reproducible and computationally
light for the first vertical slice. Sudden intent changes remain a limitation.

**What changes in baseline?** The CV mean stays the same; covariance stays fixed.
Risk-aware mode grows covariance by class. Physical encounters, safety constraints,
candidate families and controller are identical.

**How are contact and TTC calculated?** Conservative covering discs, swept relative
segments for actual/candidate contact, and analytic first-contact roots for
finite-horizon fixed-heading CV TTC. Closest approach alone is not TTC.

**What if no candidate is feasible?** The system reports infeasibility and applies
bounded braking/steering recovery. It does not label an unavoidable collision safe.

**What really runs in Simulink/Stateflow?** Vehicle signals are replayed in the
Simulink vehicle model. The separate Stateflow harness executes only the shared
behaviour kernel on declared sequences. The live vehicle loop is MATLAB.

**What evidence supports improvement?** Read the actual paired results. Preliminary
comfort/progress differences are not proof of reduced collisions or general safety;
clearance need not improve.

**What changes with sensor tracks?** The adapter must handle timestamps, uncertainty,
missing/stale tracks, association and calibration before planning can trust them.

**What is the next RoadRunner step?** On a supported licensed host, reconstruct this
same physical scene, verify coordinates/timing and connect actor truth plus ego
commands. Sensor simulation/fusion comes after that verified bridge.
