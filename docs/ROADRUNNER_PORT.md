# RoadRunner port plan

The RoadRunner MATLAB API is present, but its application is absent. No
`.rrscene`, project, screenshot, or recording has been fabricated. The MATLAB
scenario is runnable; this checklist is for a later complete installation.

This release does not install or connect RoadRunner. Perform the following
steps on a supported workstation with the required entitlement, after checking
the installed release's local platform/compatibility documentation.

## Automated checks before GUI work

1. Record MATLAB and RoadRunner versions and licence results.
2. Use `which roadrunner` and local release documentation to identify the exact
   connection/scenario APIs; signatures differ between releases.
3. Confirm MATLAB and RoadRunner release compatibility.
4. Create the project with the supported API or RoadRunner GUI, never by inventing
   undocumented binary/source formats.

## Scene reconstruction checklist

1. Create a project under `roadrunner/projects/pathweaver_v0` only after the tool
   selects/creates its valid source structure.
2. Build 135 m of unmarked asphalt following the MATLAB centre reference,
   with the same sampled left/right drivable boundaries and no lane markings.
3. Add modest shoulder/terrain context without changing collision geometry.
4. Place the pothole/obstruction using the same world coordinates and footprint.
5. Add ego, pedestrian, and two-wheeler assets; record asset substitutions and
   preserve stable actor names/IDs.
6. Recreate initial poses, speeds, clock-driven crossing onset, deterministic
   sequence, and goal marker from scenario configuration. Use the actual seeded
   parameters in the saved initial scenario, not just unperturbed nominal values.
7. Establish the supported MATLAB/Simulink connection and map actor truth into
   the documented world-state adapter. Keep coordinates and timestamps explicit.
8. Drive ego from PathWeaver control output; do not keyframe a safe path.
9. Compare actor trajectories and ego outcomes against MATLAB-only replay within
   declared tolerances.
10. Save/open/run from a clean checkout, then generate a screenshot or recording
    as proof and document the exact commands plus unavoidable GUI steps.

Native integration remains unavailable until these steps are executed and
verified on an installed, licensed release.
