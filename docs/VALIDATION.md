# Validation Results

This report summarizes the manual test runs used to validate EcoSort's core
claim: **the robot can find and sort objects placed at arbitrary, non-fixed
positions on the table, using only live camera detection** — plus the
reliability issues found along the way, the fixes applied, and what remains
a known, accepted limitation.

## Methodology

Validation was done through repeated manual runs of the full three-terminal
system (`ecosort_sim` → `ecosort_moveit_config` → `ecosort_pick_place`),
varying two things between runs:

1. **Object positions** — cubes were placed by hand at different (x, y)
   locations on the table each run, instead of relying on the coordinates
   they happen to spawn at in the world file.
2. **Object presence** — some runs had all three categories on the table,
   others had only one or two, to check that the robot doesn't waste time
   or make mistakes chasing a category that was never actually there.

Terminal output (planner results, retry counts, and the real gripper-finger
position read back from `/joint_states`) was captured and compared across
runs to tell apart three different kinds of failure: a bad grasp, a dropped
object in transit, and a motion-planning failure.

## Results by category

| Category (color) | Outcome across tested runs |
|---|---|
| Paper (blue) | Reliable in every tested run — detected correctly at random positions, grasped, and placed in the correct bin. |
| Glass (green) | Reliable in every tested run — same as above. |
| Plastic (yellow) | Succeeded in the large majority of runs; occasionally either grasped weakly (dropped the cube mid-transit) or failed to find a motion plan at all, from one specific, reproducible table region very close to the robot base (see below). |

An earlier issue — the arm traveling to a stale/fixed position for a
category that was never actually detected on the table — was found and
fixed (see [Issues found and fixed](#issues-found-and-fixed)); after the
fix, absent categories are skipped entirely rather than visited.

## Grasp verification: real observed data

Because the simulated Robotiq gripper has no dedicated force/pressure
sensor, grasp success is inferred indirectly: after commanding the gripper
closed, the code reads back the **real** (not commanded) position of
`robotiq_85_left_knuckle_joint` from `/joint_states`. If the finger actually
closed almost all the way, nothing is between the fingers (failed grasp); if
it stopped short, it's pressing against an object (successful grasp).

Observed values from real test runs (`kGripperClosed` commanded = `0.79`):

| Case | Real finger position after closing |
|---|---|
| Successful grasps (typical, multiple runs) | ≈ 0.45 |
| One weak/failed grasp that later caused a drop | 0.634 |

`kGripFailThreshold` was calibrated to **0.60** from this data — low enough
to catch the observed marginal grasp (0.634) as a failure, while staying
safely above every observed good grasp (≈0.45). When a grasp is judged
failed, the node reopens the gripper, retries closing once in the same
spot, and re-checks; if it still fails, that object is skipped (the arm does
not carry an unheld/empty gripper to a bin).

## The known hard case: plastic near the robot base

A specific, reproducible position — approximately **(x=0.284, y=-0.196)** in
world coordinates, close to the robot base on the negative-Y side — was
found to reliably cause motion-planning failure for the "move above object"
step. The planner (OMPL RRTConnect) was given up to 10 seconds and 10
internal planning attempts per call
(`setPlanningTime(10.0)`, `setNumPlanningAttempts(10)`), retried up to 4
times at the call site (worst case 40 planning attempts total), and still
failed to find a valid path in this exact case across separate test runs.

This pattern — consistently timing out rather than failing near-instantly —
points to a genuine narrow/hard-to-sample region of the arm's configuration
space near that point, not a missing IK solution (which fails almost
immediately instead of consuming the full time budget). As a partial
mitigation, `setGoalOrientationTolerance` was widened from `0.02` to `0.05`
radians, giving the planner a larger valid solution space to sample from
without visibly changing the gripper's downward approach angle. This
reduced but did not fully eliminate the failure in this region.

**Current status**: after this mitigation, remaining occasional plastic
grasp/planning failures were judged acceptable, and further tuning of the
grasp/planning logic was intentionally paused to avoid destabilizing the
categories that already work reliably. This is documented as a known
limitation (see the main [README](../README.md#known-limitations)) rather
than a bug being actively chased.

## Issues found and fixed

| Issue observed | Root cause | Fix |
|---|---|---|
| Gripper silently drops an object; arm continues as if still holding it | No grasp verification existed — the code assumed "commanded closed" meant "holding something." | Added real-time `/joint_states` feedback (see above): check the actual finger position right after closing, retry once in place, and skip the object entirely (returning to home empty) if it still isn't grasped. |
| Arm visits a fixed/stale position for a waste category that isn't actually on the table | Old fallback used a hardcoded SDF spawn coordinate whenever no camera detection had arrived yet for that category. | Removed the fixed-position fallback. If a category has no detection by the time its turn comes, it is skipped outright; the initial wait before concluding "not present" was also raised from 1500 ms to 3000 ms to reduce false negatives. |
| Occasional cube ends up misplaced | Combination of the two issues above plus the grasp-threshold miscalibration below. | Covered by the two fixes above and the threshold recalibration. |
| Weak grasp (0.634) passed as "successful" under the original threshold | Original threshold (`kGripperClosed - 0.05` = 0.74) was too lenient. | Tightened to a fixed `0.60`, based on the real observed data above. |

## What was verified

- Detection and pick-and-place work correctly for objects placed at
  positions not present anywhere in the world file's default spawn
  coordinates — confirmed across multiple runs with hand-placed cubes.
- The system correctly handles a subset of categories being absent (1 or 2
  of the 3), skipping absent ones without visiting stale positions.
- Grasp-failure detection and single retry-in-place work as intended for the
  cases exercised (including the deliberately marginal 0.634 case).

## What would be improved with more time

- A real contact/force sensor (or a Gazebo plugin providing one) instead of
  the indirect `/joint_states`-based inference, to catch a wider range of
  failure modes and remove the need for a hand-tuned threshold.
- Further investigation into the near-base planning failure — e.g. trying
  an alternative approach orientation or a different OMPL planner for that
  region specifically, rather than only widening tolerances globally.
- Automated (rather than manual) regression runs, sweeping many random
  positions per category and logging pass/fail automatically.
