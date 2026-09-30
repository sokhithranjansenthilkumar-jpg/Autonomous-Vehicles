# Phase 2 release verification

Baseline audited at `bb913c0`; implementation milestones are retained as local,
unsquashed conventional commits. The repository began clean. No native RoadRunner
or sensor integration was attempted.

## Verified release evidence

Corrected footprint/TTC, three physical presets, paired evaluation, live
diagnostics and the separate stateful Stateflow harness are implemented.
The complete suite passed **39/39 tests**, with **zero MATLAB Code Analyzer
findings**, at clean revision **84aff77057ae0ba9175bdc675162da6883761161**.
The run included both model
builders, ordered Stateflow sequences, rendered playback invariance, paired
conditions and all default presets. The MIT-licence commit e70b0de is preserved.
Verification used MATLAB R2026a Update 5 on Apple-silicon macOS; see
[Environment](ENVIRONMENT.md) for the product inventory and execution limits.

The clean-revision batch, all six preset/mode demo checks, backup recording and
pitch generation completed successfully. The transcript contains no MATLAB
warnings or errors. A later presentation-only fix, 08aeba8, makes benchmark
plot contrast independent of the desktop theme; its evaluation regression and
Code Analyzer check passed. No simulation algorithms changed after the recorded
evidence revision. The comparison PNG was re-rendered from the saved results.

Verified default risk-aware runs at seed 26037:

| Preset | Completion time | Minimum conservative clearance | Observed contact |
| --- | ---: | ---: | --- |
| nominal | 24.25 s | 0.6272 m | none |
| challenging | 25.25 s | 0.6618 m | none |
| emergency | 25.20 s | 0.6274 m | none |

Baseline also completed all three demo presets without contact: nominal 24.65 s,
challenging 25.25 s and emergency 24.90 s. Matching configurations reproduced
identical simulated logs; rendered playback at 0.5× and 4× was tested independently
of the simulation timestep. Wall-clock latency is not deterministic.

## Paired benchmark

Seeds **27001–27010**, three presets and two modes produced **60 actual runs**
at clean revision 84aff77. These seeds are separate from demo tuning seed 26037.
Each mode completed **30/30**, with **zero collisions, timeouts or invalid runs**.
Every record was retained. A separate audit checked all 60 MAT/CSV rows against
their raw run metrics and parameters, all 30 physical pairs, and ten distinct
seed-generated physical encounters per preset.

| Preset | Mode | Runs | Mean completion (s) | Min. clearance (m) | Mean integrated absolute jerk (m/s²) | Mean / p95 planner latency (ms)¹ |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| nominal | baseline | 10 | 27.745 | 0.410 | 32.505 | 8.616 / 11.366 |
| nominal | risk-aware | 10 | 27.720 | 0.410 | 29.808 | 8.558 / 11.198 |
| challenging | baseline | 10 | 24.800 | 0.626 | 40.185 | 8.475 / 11.161 |
| challenging | risk-aware | 10 | 23.565 | 0.623 | 29.967 | 8.555 / 11.324 |
| emergency | baseline | 10 | 25.025 | 0.632 | 45.300 | 8.538 / 11.193 |
| emergency | risk-aware | 10 | 24.880 | 0.600 | 44.092 | 8.931 / 11.331 |

¹ Mean of each run's mean and mean of each run's p95, not a pooled percentile.
The first two planning calls per run are excluded. Rendering/export is excluded
from the measured prediction/planning section. Geometry, TTC horizon and missing
values are defined in [Metrics](METRICS.md).

This sample suggests a preliminary progress/comfort benefit, most visibly in
the challenging preset. It shows **no collision-reduction advantage and no
consistent clearance improvement**. Ten nearby variations of one scenario
family cannot establish generalisation or road safety. Earlier exploratory
dirty-revision batches are retained as diagnostics, not release evidence.

## Audit findings addressed

The old proximity/closest-approach proxy was not valid generic 2D TTC; old TTC
numbers must not be compared with the corrected metric. Geometry is now shared
and swept, world clocks align, actor timing is independent of ego motion, and
randomness is isolated. Intermediate failures exposed insufficient stopping
choices; bounded braking now participates in planning before infeasibility.
Collision checks were not weakened to obtain passing runs.

The Stateflow harness is tested on 186 ordered samples of state/reason/dwell
memory and seven reset cases (21 additional samples). These are defined-input
equivalence checks, not a Stateflow-controlled vehicle validation.

## Evidence locations

- `artifacts/phase2_release_verification.txt`: final full-suite, batch, demo,
  recording, pitch-generation and analyzer transcript.
- `artifacts/phase2_evidence_audit.txt`: export/pairing/video-repeatability audit
  and post-contrast-fix regression.
- `artifacts/phase2_test_results.mat`: machine-readable test results.
- `artifacts/release/evaluation/`: final per-run records, summary, plot and metadata.
- `artifacts/release/backup/nominal/`: backup video, snapshot and provenance.
- `artifacts/release/pitch/`: rendered pitch slides and source provenance.
- Earlier `artifacts/phase2_*.txt`: retained audit/regression diagnostics.

Generated evidence is not committed. Reproduction commands are in
[the demonstration guide](DEMO.md); metric definitions are in [Metrics](METRICS.md).

The backup `pathweaver_demo.mp4` is **24.3 s, 1920 × 1080, 10 fps**, encoding a
24.25 s nominal/risk-aware run at seed 26037 and clean revision 84aff77.
Its simulated logs match the headless run exactly when wall-clock latency is
excluded. The snapshot, representative decoded frames, comparison plot and
architecture/results pitch slides were visually inspected. The live recording
measured mean/p95 planner latency of 11.24/13.95 ms; this is a different workload
from the headless benchmark. Render speed is best-effort, not a real-time claim.

## Remaining limitations and next work

The vehicle loop is authoritative MATLAB. The Simulink vehicle model is replay;
only the separate Stateflow behaviour harness executes the shared decision
kernel inside its model. RoadRunner, sensors and fusion remain unimplemented.
Other MATLAB releases/platforms and arbitrary intrusion conditions are unverified.
Next priorities are broader robustness/track-input validation, native
RoadRunner/Simulink integration, and sensor processing—not road deployment.
