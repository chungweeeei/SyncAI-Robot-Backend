"""The controller settings the console may change, and what it may change them to.

``controller_server`` declares its parameters with no descriptor -- no
description, no range -- so ``DescribeParameters`` answers a name and a type
and nothing an operator could act on. This table is where the console's form
comes from instead: a curated list of fields, each with its operator-facing
label, the limits it may be set within, and how a change takes effect.

A field is not one ROS parameter. ``targets`` lists every parameter it writes,
because several are one calibration that must move together (the walking
speed is MPPI's ``vx_max`` *and* RPP 3D's ``desired_linear_vel``; the arrival
tolerance is the goal checker's *and* MPPI's ``GoalAngleCritic`` threshold).
The value read back is the first target's.

``apply`` is what the controller does with a live set, read off its C++
callbacks (2026-11-03), not assumed:

- ``live``: a plugin parameter its callback stores into the running
  controller; it applies mid-goal, at the next control cycle.
- ``idle``: controller_server's own (undotted) parameters. Its callback refuses
  every set while a goal holds its lock (``successful: false``, "...while the
  controller is currently running"), so these are sent in a set of their own.
- ``restart``: read once at startup (StairMode's geometry, for one); a live set
  would be stored by the node and used by nothing. Written to the overlay only,
  and in force from the next start of the nav session.

Moving a parameter between those three in the workspace (a new callback
branch, a cached value) is a change here too; so is renaming one -- the
workspace's CLAUDE.md lists these names as part of the contract.

Ranges are what the console may write, not what the controller accepts: the
controller range-checks almost nothing, and a value it accepts can still leave
the robot unable to walk (below the gait's walking floor) or overrun it.
"""


from typing import Optional

from syncai_backend.services.settings_schema import (
    APPLY_IDLE,
    APPLY_LIVE,
    APPLY_RESTART,
    Group,
    Rule,
    SettingField,
    SettingsSchema,
    ValueOf,
    below,
)

GROUPS: tuple[Group, ...] = (
    Group(
        "speed",
        "Speed",
        "How fast the robot walks and how hard it speeds up or slows down.",
    ),
    Group(
        "arrival",
        "Arrival",
        "When the robot counts itself as arrived at a destination.",
    ),
    Group(
        "stuck",
        "Stuck detection",
        "How long the robot keeps trying before it reports that it cannot get through.",
    ),
    Group(
        "path",
        "Path following",
        "How the robot trades staying on its route against keeping clear of obstacles.",
    ),
    Group("stairs", "Stairs", "Walking up and down stairs."),
)


FIELDS: tuple[SettingField, ...] = (
    # --- Speed -------------------------------------------------------------
    SettingField(
        id="max_speed",
        group="speed",
        label="Maximum walking speed",
        description=(
            "The fastest the robot walks on a flat route. Both route-following "
            "methods use the same value. Above about 0.8 m/s the gait's speed "
            "calibration has not been measured."
        ),
        # One calibration: MPPI's cap and RPP 3D's cruise speed (README:
        # "MPPI's vx_max 0.80 matches RPP 3D's desired_linear_vel"). The
        # optimizer's copy of vx_max is dynamic; the controller's own static
        # copy is unused.
        targets=("FollowPath.vx_max", "FollowPathRPP3D.desired_linear_vel"),
        apply=APPLY_LIVE,
        unit="m/s",
        min=0.4,
        max=1.0,
        step=0.05,
    ),
    SettingField(
        id="max_linear_accel",
        group="speed",
        label="Acceleration",
        description=(
            "How quickly the robot speeds up and slows down while walking. Lower "
            "is gentler; higher reacts faster to obstacles."
        ),
        targets=("FollowPath.max_linear_accel",),
        apply=APPLY_LIVE,
        unit="m/s²",
        min=0.3,
        max=2.0,
        step=0.1,
    ),
    SettingField(
        id="max_angular_accel",
        group="speed",
        label="Turning acceleration",
        description="How quickly the robot starts and stops turning.",
        targets=("FollowPath.max_angular_accel",),
        apply=APPLY_LIVE,
        unit="rad/s²",
        min=1.0,
        max=6.0,
        step=0.1,
    ),
    # --- Arrival -----------------------------------------------------------
    SettingField(
        id="xy_goal_tolerance",
        group="arrival",
        label="Position tolerance",
        description=(
            "How close to a destination the robot must stand to count as "
            "arrived. Smaller is more precise but takes longer to settle."
        ),
        # The controller README: the goal checker's xy tolerance equals
        # GoalAngleCritic.threshold_to_consider; change them together.
        targets=(
            "goal_checker.xy_goal_tolerance",
            "FollowPath.GoalAngleCritic.threshold_to_consider",
        ),
        apply=APPLY_LIVE,
        unit="m",
        min=0.05,
        max=0.5,
        step=0.01,
    ),
    SettingField(
        id="yaw_goal_tolerance",
        group="arrival",
        label="Heading tolerance",
        description=(
            "How closely the robot must face a destination's heading to count as "
            "arrived (0.1 rad is about 6°)."
        ),
        targets=("goal_checker.yaw_goal_tolerance",),
        apply=APPLY_LIVE,
        unit="rad",
        min=0.05,
        max=0.8,
        step=0.01,
    ),
    SettingField(
        id="approach_dist",
        group="arrival",
        label="Final approach distance",
        description=(
            "Within this distance of a destination the robot stops planning "
            "around and walks straight onto it, slowly."
        ),
        targets=("FollowPath.approach_dist",),
        apply=APPLY_LIVE,
        unit="m",
        min=0.2,
        max=1.5,
        step=0.05,
    ),
    # --- Stuck detection ---------------------------------------------------
    SettingField(
        id="required_movement_radius",
        group="stuck",
        label="Minimum progress",
        description=(
            "How far the robot must move within the time below to count as "
            "making progress."
        ),
        targets=("progress_checker.required_movement_radius",),
        apply=APPLY_LIVE,
        unit="m",
        min=0.05,
        max=1.0,
        step=0.05,
    ),
    SettingField(
        id="movement_time_allowance",
        group="stuck",
        label="Progress time limit",
        description=(
            "How long the robot may go without making progress before the "
            "navigation fails."
        ),
        targets=("progress_checker.movement_time_allowance",),
        apply=APPLY_LIVE,
        unit="s",
        min=5.0,
        max=120.0,
        step=1.0,
    ),
    SettingField(
        id="failure_tolerance",
        group="stuck",
        label="Blocked time per attempt",
        description=(
            "How long the robot waits for a way through before an attempt "
            "counts as failed. It retries a few times, and is meant to wait "
            "long enough for a re-route to come first; can only be changed "
            "while the robot is not navigating."
        ),
        # Undotted: controller_server's own, refused mid-goal. The workspace
        # sizes (retries + 1) x failure_tolerance against the BT's blocked_time.
        targets=("failure_tolerance",),
        apply=APPLY_IDLE,
        unit="s",
        min=1.0,
        max=30.0,
        step=0.5,
    ),
    # --- Path following ----------------------------------------------------
    SettingField(
        id="path_follow_weight",
        group="path",
        label="Stay on route",
        description="How strongly the robot keeps to its planned route.",
        targets=("FollowPath.PathFollowCritic.cost_weight",),
        apply=APPLY_LIVE,
        min=0.0,
        max=20.0,
        step=0.5,
    ),
    SettingField(
        id="obstacle_weight",
        group="path",
        label="Keep clear of obstacles",
        description="How strongly the robot keeps its distance from obstacles.",
        # CostCritic divides by 254 at startup and in its dynamic callback
        # alike, so the number here is on the same scale either way.
        targets=("FollowPath.CostCritic.cost_weight",),
        apply=APPLY_LIVE,
        min=0.0,
        max=20.0,
        step=0.1,
    ),
    SettingField(
        id="twirling_weight",
        group="path",
        label="Smooth steering",
        description="How strongly the robot avoids weaving left and right.",
        targets=("FollowPath.TwirlingCritic.cost_weight",),
        apply=APPLY_LIVE,
        min=0.0,
        max=20.0,
        step=0.5,
    ),
    # --- Stairs ------------------------------------------------------------
    SettingField(
        id="stair_speed",
        group="stairs",
        label="Climbing speed",
        description="How fast the robot walks on a flight of stairs.",
        # Both controllers climb; each declares its own copy (README: stair
        # mode is "declared under each plugin's name with the same values").
        targets=("FollowPath.stair_linear_vel", "FollowPathRPP3D.stair_linear_vel"),
        apply=APPLY_LIVE,
        unit="m/s",
        min=0.1,
        max=0.8,
        step=0.05,
    ),
    SettingField(
        id="stair_standoff",
        group="stairs",
        label="Stop before stairs",
        description=(
            "How far before the first step the robot stops to switch to its "
            "stair gait. Takes effect after a restart."
        ),
        # StairMode reads its geometry once in initialize(): static in both.
        targets=("FollowPath.stair_standoff", "FollowPathRPP3D.stair_standoff"),
        apply=APPLY_RESTART,
        unit="m",
        min=0.5,
        max=2.0,
        step=0.1,
    ),
    # Docking (Docking / Undocking, dock_goal_checker) is left off the page
    # for now, by request (2026-11-03). DockingController's callback refuses a
    # bad batch properly, so adding its fields back needs only rows here and
    # the v_linear_min / v_linear_max rule.
)

RULES: tuple[Rule, ...] = (
    # MPPI raises any walking command to walk_vx_min (the gait stands still
    # under it); a cap at or under the floor leaves nothing to regulate.
    Rule(
        fields=("max_speed",),
        check=below(
            "FollowPath.vx_max",
            "FollowPath.walk_vx_min",
            "The maximum walking speed ({hi} m/s) must stay above the slowest "
            "speed the robot can walk at ({lo} m/s).",
        ),
    ),
    # The MPPI README: approach_decel and goal_stop_decel stay below
    # max_linear_accel, or the slowdown is clipped by the accel clamp.
    Rule(
        fields=("max_linear_accel",),
        check=below(
            "FollowPath.max_linear_accel",
            "FollowPath.approach_decel",
            "Acceleration ({hi} m/s²) must stay above the slowdown used on the "
            "final approach ({lo} m/s²).",
        ),
    ),
    Rule(
        fields=("max_linear_accel",),
        check=below(
            "FollowPath.max_linear_accel",
            "FollowPath.goal_stop_decel",
            "Acceleration ({hi} m/s²) must stay above the slowdown used to stop "
            "at a destination ({lo} m/s²).",
        ),
    ),
    # The MPPI README: approach_floor_dist stays above xy_goal_tolerance, or the
    # approach's speed floor ends outside the tolerance and the robot creeps.
    Rule(
        fields=("xy_goal_tolerance",),
        check=below(
            "FollowPath.approach_floor_dist",
            "goal_checker.xy_goal_tolerance",
            "The position tolerance ({lo} m) must stay under the distance at "
            "which the final approach reaches its slowest speed ({hi} m).",
        ),
    ),
    Rule(
        fields=("xy_goal_tolerance", "approach_dist"),
        check=below(
            "FollowPath.approach_dist",
            "goal_checker.xy_goal_tolerance",
            "The position tolerance ({lo} m) must stay under the final approach "
            "distance ({hi} m).",
        ),
    ),
    # The workspace's CLAUDE.md: keep stair_standoff + stair_enter_dist under
    # stair_exit_ahead_dist, or the stair gait is switched back off before the
    # robot reaches the stair it stopped for.
    *(
        Rule(
            fields=("stair_standoff",),
            check=(
                lambda value_of, plugin=plugin: _standoff_sentence(value_of, plugin)
            ),
        )
        for plugin in ("FollowPath", "FollowPathRPP3D")
    ),
)


def _standoff_sentence(value_of: ValueOf, plugin: str) -> Optional[str]:
    standoff = value_of(f"{plugin}.stair_standoff")
    enter = value_of(f"{plugin}.stair_enter_dist")
    exit_ahead = value_of(f"{plugin}.stair_exit_ahead_dist")
    if standoff is None or enter is None or exit_ahead is None:
        return None
    if standoff + enter < exit_ahead:
        return None
    return (
        f"Stopping {standoff} m before stairs is too far: with the {enter} m "
        f"lead-in it must stay under {exit_ahead} m, or the robot leaves its "
        "stair gait before it reaches the stairs."
    )


CONTROLLER = SettingsSchema(groups=GROUPS, fields=FIELDS, rules=RULES)
