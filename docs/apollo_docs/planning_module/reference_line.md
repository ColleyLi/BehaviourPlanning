# Reference Line

Provides the interface of interaction with data the from PNC map

Use-cases:

- Cartesian <-> Frenet transformations
- Check whether given point (XY or Frenet), bounding box, SL-boundary are within the reference line
- Extract information about the reference line: road width, lane width, driving width, offset to map, road type, length, priority, speed limits

Contains:

- `speed_limit` - vector of speed limits in format (`start_s`, `end_s`, `speed_limit`). Overwrite speed limits from the HD map
- `reference_points` - reference points. Described below
- `map_path` - defined in the PNC map sub-module. More about this module [here](https://github.com/Sarrasor/BehaviourPlanning/tree/main/docs/apollo_docs/map_module)
- `priority` - some kind of priority 

Interesting methods:

- `Stitch(ReferenceLine)` - tries to stitch current reference line with the argument reference line. The stitching strategy is to use current reference points as much as possible
- `Segment(point, forward, backward)` - updates map_path to be [forward; backward] around the point
- `HasOverlap(Box2d)` - 
- `IsOnLane()` - checks if given argument(point, box, sl-boundary) is along the reference line
- `IsOnRoad()` - checks if given argument(point, box, sl-boundary) is along the reference line, excluding sideways and medians
- `IsBlockRoad(Box2d, gap)` - check if a box is blocking the road such that the remaining space gap is less than the gap
- `GetReferencePoint(s)` - extracts interpolated point from the `map_path` points based on the `s` argument
- `GetSpeedLimitFromS(s)` - first searches `speed_limit`, then sets speed_limit to min of `upper_planning_speed_limit` and HD map lane speed limit. If there was no HD map lane speed limit, will set `default_city_road_speed_limit` or `default_highway_speed_limit`

### Reference point

Helper class, extension of the MapPathPoint (from PNC map `path.h` file) class with curvature and derivative of curvature

## Reference Line Provider

The main purpose of the reference line provider is to produce reference lines based on the PNC map, routing response, and the current position 

The public interface interaction pipeline is as follows:

- Create the reference line provider instance
- Use `Start()` and `Stop()` methods if you want to use threaded mode
- Use `UpdateRoutingResponse(routing)` and `UpdateVehicleState(vehicle_state)` to update the state of the provider
- Use `GetReferenceLines(reference_lines, segments)` to request current reference lines and segments based on the updated state
- Also, can use `FutureRouteWaypoints()` to request future waypoints from the routing request

Reference line sub-module has three smoothers. The default smoother in Apollo 6 is QP smoother

Contains:

- `smoother` - smoother to use
- `smoother_config` - configuration of the smoother
- `pnc_map` - pnc map. Described [here](https://github.com/Sarrasor/BehaviourPlanning/tree/main/docs/apollo_docs/map_module)
- `relative_map` - relative map. Described [here](https://github.com/Sarrasor/BehaviourPlanning/tree/main/docs/apollo_docs/map_module)
- `vehicle_state` - current vehicle state 
- `vehicle_state_provider` - used only when creating reference lines from the relative map
- `routing` - current routing request
- `reference_lines` - list of reference lines
- `route_segments` - list of route segments

## Reference Line Info

Wraps static reference line with dynamic info about obstacles and obstacle decisions, destination information, planning cost and priority cost etc

Initialization procedure:
- All obstacles from the frame are passed to the `Init()`
- ADC's bounding box is extracted and set to current position
- ADC's bounding box is converted to SLBoundary
- Nearest overlaps are initialized based on the SLBoundary
- If the ADC is out of this lane more than 10 meters, initialization is failed
- `is_on_reference_line_` flag is set is sl boundary is on the reference line
- Obstacles are added. Each obstacle is evaluated according to its relevance. For example, if an obstacle is far behind the vehicle, it gets Ignore decision right away 
- Speed bump speed limits are added
- Cruise speed is set to default cruise speed
- Vehicle signal is cleared

Usages:

- `IsChangeLanePath()` - check if ADC is not on the current reference line. This means the current reference line is a change lane path
- `IsNeighborPath()` - check if the current reference line is neighbor lane to ADC's current lane
- `GetNeighborLineInfo()` - extracts info about neighbor lane with given direction (`LeftForward`, `LeftReverse`, `RightForward`, `RightReverse`). If no neighbor with given direction, `nullptr` is returned
- `CombinePathAndSpeedProfile()` - generates `DiscretizedTrajectory` from path and speed profiles
- `SetTurnSignal()` - sets vehicle's turn signal
- `ExportVehicleSignal()` - executes internal turn decision based on lane turn type and returns it
- `SDistanceToDestination()` - returns distance to destination. If there is no destination point, will return `std::numeric_limits<double>::max()`
- `ExportDecision()` - generates internal decision (obstacles decision and main decision) based on the planning context. Calls `ExportVehicleSignal()`
- `GetRightOfWayStatus()` - returns current "right of way" (who has the priority in current situation) status (`PROTECTED` or `UNPROTECTED`)

Contains:

- `junction_right_of_way_map` - defines "right of way" rule application for junctions. Sets `RightOfWayStatus` in the `ADCTrajectory` 
- `vehicle_state` - state of the vehicle
- `adc_planning_point` - current planning starting point
- `reference_line` - reference line which is being wrapped
- `cost` - "goodness" of the reference line. The lower - the better
- `is_drivable` - flag to mark line as drivable
- `path_decision` - obstacle and stop decision related to this reference line. Description is [here](https://github.com/Sarrasor/BehaviourPlanning/tree/main/docs/apollo_docs/planning_module/trajectory_generation.md)
- `blocking_obstacle` - current blocking obstacle
- `candidate_path_boundaries` - keeps candidate path boundaries
- `candidate_path_data` - keeps candidate path data
- `path_data` - selected path data from candidates
- `fallback_path_data` - generated fallback path data
- `speed_data` - generated speed data
- `discretized_trajectory` - combined path and speed profiles
- `rss_info` - info related to RSS. More info [here](https://www.mobileye.com/responsibility-sensitive-safety/)
- `adc_sl_boundary` - sl boundary of the stitching point (starting point of the current trajectory) in the reference line frame
- `debug` - some debugging messages
- `latency_stats` - total execution time + frame initialization times + task execution durations
- `lanes` - route segments
- `is_on_reference_line` - flag if the ADC is on the reference line
- `is_path_lane_borrow` - flag if this lane is borrow lane
- `status` - right of way status
- `offset_to_other_reference_line` - offset to other reference line. Set manually
- `priority_cost` - priority cost of the lane
- `planning_target` - planning target for the lattice planner
- `trajectory_type` - type of the planned trajectory. One of: `UNKNOWN`, `NORMAL`, `PATH_FALLBACK`, `SPEED_FALLBACK`, `PATH_REUSED`
- `first_encounter_overlaps` - nearest overlaps along the reference line in front of the ADC
- `st_graph_data` - data generated by speed bounds decider for st graph optimizers
- `vehicle_signal` - signals like horn, emergency lights, turn lights etc
- `cruise_speed` - speed to keep while cruising
- `path_reusable` - is current path reusable flag 
