# First-Pass Pedestrian Data Collection Stability

## Goal

Make the toilet benchmark repeatable enough for an initial navigation dataset
without replacing its semantic state machine or adding HuNav.

## This Iteration

1. Plan each travel leg against the static voxel map. Robot motion must not
   change the nominal A* route.
2. Treat a nearby robot as a yield condition at execution time. Pause the
   pedestrian and resume the existing route after the robot clears.
3. Do not count robot-yield time as a pedestrian stall or trigger recovery.
4. Disable the kinematic pedestrian physics proxy in the
   `physx_diff_contact` phase so it cannot push the dynamic robot.
5. Keep portal serialization, resource ownership, animation, lifecycle, and
   final alignment unchanged.

## Verification

- Fixed inputs produce the same nominal path with or without fresh robot odom.
- A robot-blocked pedestrian does not accumulate recovery attempts.
- The contact profile exports the stop policy and disables pedestrian physics
  proxies; the legacy `physx_wheels` profile keeps its existing behavior.
- Existing planner, portal, profile, and dynamic-guard tests continue to pass.

## Later

- Add pedestrian-pedestrian yielding and overlap metrics.
- Record route hashes and interaction events per episode.
- Replace the local yield layer with HuNav or ORCA when higher-density social
  interaction is required.
