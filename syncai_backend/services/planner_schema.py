"""The planner settings the console may change, and what it may change them to.

All of them are ``Octo3D``'s (the 3D planner, ``syncai_global_planner``), and
all are ones its dynamic callback stores into the running planner
(``OctomapPlanner3D::dynamicParametersCallback``, read 2026-11-03), so every
field is ``live``: the next plan uses it. That callback refuses a build-time
key with a reason instead of storing it, which is why nothing here is sent
that it would refuse.

Left off on purpose:

- The robot's geometry (``robot_radius``, ``body_clearance``,
  ``max_step_height``, ``max_slope_deg``) and everything else the surface map
  is built from: build-time here, *and* one calibration with the controller's
  local costmap terrain layer, a second node this page does not reach.
- ``keepout.block_radius``: the planner's README pins it to ``robot_radius``
  ("change the two together"), which is not on the page.
- ``goal_z_mode`` (a string: the schema has no choice kind yet) and the
  search's internals (``heuristic_weight``, ``max_iterations``, the step edge
  curve), which are an engineer's, not an operator's.

The workspace's planner README has each parameter's reasoning; the
descriptions here are what an operator needs to choose a value.
"""

from syncai_backend.services.settings_schema import (
    APPLY_LIVE,
    KIND_BOOLEAN,
    Group,
    SettingField,
    SettingsSchema,
)

GROUPS: tuple[Group, ...] = (
    Group(
        "route",
        "Route choice",
        "What the robot prefers when it picks a route to a destination.",
    ),
    Group(
        "obstacles",
        "Obstacles",
        "How routes avoid people and things the robot sees on its way.",
    ),
    Group(
        "destinations",
        "Destinations and limits",
        "Where a route may end, and how long the robot may spend finding one.",
    ),
)

FIELDS: tuple[SettingField, ...] = (
    # --- Route choice ------------------------------------------------------
    SettingField(
        id="wall_clearance_weight",
        group="route",
        label="Keep away from walls",
        description=(
            "How strongly a route keeps to the middle of corridors rather than "
            "hugging walls. 0 takes the shortest line; higher costs a longer "
            "route to stay clear."
        ),
        targets=("Octo3D.clearance_cost_weight",),
        apply=APPLY_LIVE,
        min=0.0,
        max=5.0,
        step=0.1,
    ),
    SettingField(
        id="stair_cost",
        group="route",
        label="Stair avoidance",
        description=(
            "How much longer a flat route may be before the robot takes the "
            "stairs instead. 1 treats a metre of stairs like a metre of floor."
        ),
        # A factor on a stair link's 3D length; the planner wants >= 1.
        targets=("Octo3D.stair_cost_multiplier",),
        apply=APPLY_LIVE,
        unit="×",
        min=1.0,
        max=10.0,
        step=0.5,
    ),
    SettingField(
        id="kerb_cost",
        group="route",
        label="Kerb avoidance",
        description=(
            "How much the robot avoids stepping over kerbs and single steps "
            "when there is a level way round."
        ),
        targets=("Octo3D.kerb_cost_multiplier",),
        apply=APPLY_LIVE,
        unit="×",
        min=1.0,
        max=5.0,
        step=0.1,
    ),
    SettingField(
        id="shortcut",
        group="route",
        label="Straighten routes",
        description=(
            "Smooth a route into straight lines where the way is clear, "
            "instead of following the map's grid."
        ),
        targets=("Octo3D.shortcut",),
        apply=APPLY_LIVE,
        kind=KIND_BOOLEAN,
    ),
    # --- Obstacles ---------------------------------------------------------
    SettingField(
        id="obstacle_clearance",
        group="obstacles",
        label="Distance kept from obstacles",
        description=(
            "How far a route keeps from a person or object the robot has seen. "
            "Too small and the robot brushes past; too large and a busy "
            "corridor reads as blocked."
        ),
        # The README: between the inscribed 0.25 and the circumscribed 0.455,
        # what the removed 2D perimeter test swept.
        targets=("Octo3D.dynamic.block_radius",),
        apply=APPLY_LIVE,
        unit="m",
        min=0.25,
        max=0.6,
        step=0.05,
    ),
    SettingField(
        id="obstacle_memory",
        group="obstacles",
        label="Remember obstacles for",
        description=(
            "How long an obstacle the robot can no longer see still blocks a "
            "route. It must outlast the wait before the robot looks for a way "
            "round, or the detour forgets what it was going round."
        ),
        # The README: outlasts the BT's blocked_time (10 s) plus a plan plus
        # the detour's blind stretch.
        targets=("Octo3D.dynamic.obstacle_memory",),
        apply=APPLY_LIVE,
        unit="s",
        min=15.0,
        max=120.0,
        step=5.0,
    ),
    # --- Destinations and limits -------------------------------------------
    SettingField(
        id="goal_snap_distance",
        group="destinations",
        label="Destination snap distance",
        description=(
            "How far a destination off walkable floor (on a wall, in a gap of "
            "the map) may be moved to the nearest floor the robot can stand on."
        ),
        targets=("Octo3D.snap_xy_radius",),
        apply=APPLY_LIVE,
        unit="m",
        min=0.1,
        max=2.0,
        step=0.1,
    ),
    SettingField(
        id="planning_time",
        group="destinations",
        label="Planning time limit",
        description=(
            "How long the robot may search for a route before it gives up. A "
            "large building's longest routes need more."
        ),
        targets=("Octo3D.max_planning_time",),
        apply=APPLY_LIVE,
        unit="s",
        min=0.5,
        max=10.0,
        step=0.5,
    ),
)

# No cross-parameter rules yet: none of these is pinned to another value the
# way the controller's speed is to the gait's walking floor.
PLANNER = SettingsSchema(groups=GROUPS, fields=FIELDS)
