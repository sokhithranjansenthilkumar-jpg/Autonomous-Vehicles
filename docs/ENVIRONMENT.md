# Verified environment

Verification date: 12 September 2026 (Asia/Kolkata).

| Item | Observed |
| --- | --- |
| MATLAB | 26.1.0.3346908, R2026a Update 5 |
| Host | Apple-silicon macOS 26.5.1, build 25F80; MATLAB architecture `maca64` |
| Core execution | MATLAB closed-loop simulation, tests, graphics and MP4 encoding executed |
| Simulink | Replay model builds, updates and runs |
| Stateflow | Shared behaviour kernel compiles/runs; reset and ordered-sequence tests executed |
| RoadRunner | MATLAB API resolves; no native application or integration verified |

## Installed inventory

All nine products report version 26.1:

MATLAB; Simulink; Stateflow; Automated Driving Toolbox; Navigation Toolbox;
Vehicle Dynamics Blockset; Computer Vision Toolbox; Sensor Fusion and Tracking
Toolbox; Image Processing Toolbox.

Installed does not mean used or licensed for every possible API. MATLAB,
Simulink and Stateflow were exercised by this release. The numerical core does
not require the other toolboxes. No sensor or vehicle-blockset capability is
claimed merely because its product is installed.

## API and licence evidence

`RandStream`, `VideoWriter`, `getframe`, `drivingScenario`, `roadrunner`,
`sim`, `sfroot` and `Simulink.SimulationInput` resolve locally. Stateflow's
dynamic chart classes are verified by the actual builder; a standalone
`which('Stateflow.Chart')` lookup does not resolve them.

Installed MathWorks help was used to verify `SimulationInput.setVariable`
and `setModelParameter` signatures. The runtime inventory records version,
products, API lookups and active licence feature names, not licence identifiers.

From the repository in MATLAB:

```matlab
setupPath
info = pathweaver.evaluation.environmentInfo;
disp(info)
disp(struct2table(ver))
disp(license('inuse'))
```

Use the normal licensed MATLAB application or its `bin/matlab -batch`
executable. The development sandbox could not launch Qt/CPU detection correctly;
the same batch commands succeeded under approved normal execution. This is not
a MATLAB numerical test failure.

RoadRunner installation and sensor integration were intentionally not attempted
for this release. See [RoadRunner port](ROADRUNNER_PORT.md).
