# Obstacle

Planning obstacles are created from Perception obstacles protobuf

Each obstacle has path properties like `l` and `s` values. Obstacle decision is also associated with the current path

Also, obstacles have their priorities:

- Lateral decision safety priority order: nudge > ignore
- Longitudinal decision safety priority order: stop > yield >= follow > overtake > ignore

Obstacle class contains several useful static methods:

- `CreateObstacles()` - creates planning obstacles from prediction obstacle data. If a prediction obstacle has multiple trajectories, will generate multiple obstacles 
- `CreateStaticVirtualObstacles()` - creates static virtual obstacle with given id from Box2d
- `IsValidPerceptionObstacle()` - checks if given obstacle is valid perception obstacle

Contains:

- `id` - if of the obstacle
- `perception_id` - id of the obstacle from perception
- `is_static` - flag if this obstacle is static
- `is_virtual` - flag if this obstacle is virtual. Virtual obstacles are mainly stopping fences
- `speed` - speed of the obstacle
- `path_st_boundary_initialized` - flag if st boundary from path decision is initialized
- `trajectory` - obstacle trajectory from prediction
- `perception_obstacle` - obstacle from perception
- `perception_bounding_box` - obstacle bounding box from perception
- `perception_polygon` - obstacle polygon from perception. For finer resolution
- `decisions` - vector of all decisions for the obstacle. Decisions are of type `ObjectDecisionType`. Description can be found [here](https://github.com/Sarrasor/BehaviourPlanning/tree/main/docs/apollo_docs/planning_module/decision_protobuf.md)
- `decider_tags` - human-readable names for decisions 
- `sl_boundary` - sl boundary of the obstacle
- `reference_line_st_boundary` - st boundary of the obstacle based on the adc's s start coordinate and reference line
- `path_st_boundary` - st boundary of the obstacle from the path decision class 
- `lateral_decision` - lateral decision on the obstacle. One of: `Nudge` or `Ignore`
- `longitudinal_decision` - longitudinal decision on the obstacle. On of: `Stop`, `Yield`, `Follow`, `Overtake`, `Ignore`
- `is_blocking_obstacle` - is this obstacle blocking. Used for keep clear
- `is_lane_blocking` - does this obstacle occupy the whole lane width. Has meaning when the obstacle is static
- `is_change_lane_blocking` - does this obstacle prevents lane change
- `is_caution_level_obstacle` - level of the importance. If set, obstacle will not be ignored right away
- `min_radius_stop_distance` - stop distance to the obstacle based on the ADC's minimum turning radius
- `s_lateral_decision_safety_sorter` -
- `s_longitudinal_decision_safety_sorter` -