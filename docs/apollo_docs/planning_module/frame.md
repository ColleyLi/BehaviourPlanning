# Frame

The data container that is used throughout every planning cycle

Use cases:

- `FindDriveReferenceLineInfo()` - returns current drive reference line info
- `CreateStopObstacle()` - adds stop obstacle to obstacles on lane with given id
- `CreateStaticObstacle()` - adds static obstacle to obstacles on given reference lane info 
- `Rerouting()` - process rerouting request need
- `UpdateReferenceLinePriority()` - updates reference line priorities for given pairs `{id, priority}`
- `GetSignal()` - returns traffic light data for given traffic light id
- `CreateReferenceLineInfo()` - creates reference line infos from given reference lines and segments. Assumes obstacles were initialized before
- `FindCollisionObstacle()` - finds the obstacle that collides with ADC, if exists 
- `AddObstacle()` - adds given obstacle to obstacle list

Contains:

- `pad_msg_driving_action` - enum of `NONE`, `FOLLOW`, `CHANGE_LEFT`, `CHANGE_RIGHT`, `PULL_OVER`, `STOP`, `RESUME_CRUISE`
- `sequence_num` - sequence number of a frame. initialized on frame creation
- `local_view` - data from other apollo modules
- `hdmap` - hdmap pointer. Can be used to gather info from the HD map
- `planning_start_point` - initialized on frame creation
- `vehicle state` - protobuf message with the state of the ADC
- `reference_line_info` - list of reference lines. Described [here](https://github.com/Sarrasor/BehaviourPlanning/tree/main/docs/apollo_docs/planning_module/reference_line.md)
- `is_near_destination` - will be set true if any segment from CreateReferenceLineInfo has stop for destination
- `drive_reference_line_info` - resultant reference line to drive on
- `obstacles` - list of all obstacles 
- `traffic_lights` - map of traffic light detection protobuf messages
- `current_frame_planned_trajectory` - current frame trajectory to pass to the control module
- `current_frame_planned_path` - current frame path for possible future speed fallback
- `reference_line_provider` - generator of reference lines to plan on. NOT USED
- `open_space_info` - data for the open space planner
- `future_route_waypoints` - waypoints to pass through
- `monitor_logger_buffer` - logging buffer for the monitor module

Frame can also request rerouting by calling `Rerouting(PlanningContext)` and filling the rerouting field in the planning context. Frame does not contain the planning context within itself

## Local View

Contains messages from planning-related topics:

- `prediction_obstacles`
- `chassis`
- `localization_estimate`
- `traffic_light`
- `routing`
- `relative_map`
- `pad_msg`
- `stories`

## Planning context

Contains `planning_status` protobuf. Claimed to be persistent across multiple frames

### Planning status

Protobuf that contains:

- `BareIntersectionStatus` - current pnc junction overlap id, zero or more previous pnc junction overlap ids, and clear counter
- `ChangeLaneStatus` - current status (IN_CHANGE_LANE, CHANGE_LANE_FAILED, CHANGE_LANE_FINISHED) and id of the current route segment, timestamp of setting the current state, lane change starting position, timestamp of the last successful lane change planning, state of success of current path and speed profile generation, and flag if it is clear to change the lane
- `CreepDeciderStatus` - creep clear counter
- `CrosswalkStatus` - crosswalk id, zero or more timestamps of when to start stopping for the crosswalk + id of the corresponding obstacle, and zero or more finished crosswalks
- `DestinationStatus` - flag whether an ADC has passed the destination
- `EmergencyStopStatus` - coordinates of the stop fence location
- `OpenSpaceStatus` - flag position init and index history of partitioned trajectories
- `ParkAndGoStatus` - the initial position and heading of an ADC, status flag of in check stage, and ADC adjust end pose 
- `PathDeciderStatus` - number of frontal static obstacles and id of frontal static obstacle (probably current active), status flag of lane borrow scenario and decided side-pass direction (can be multiple) + counter of "able to use self lane" 
- `PullOverStatus` - pull-over type (usual or emergency), bool flag to ask to plan pull-over path, and other parameters like: position, theta, length and width
- `ReroutingStatus` - last rerouting time and routing request + bool flag to ask for rerouting
- `ScenarioStatus` - scenario type and stage type (probably current active)
- `SpeedDeciderStatus` - zero or more pedestrian stop times
- `StopSignStatus` - current and previous overlap ids with stop sign + ids of the obstacles to wait for
- `TrafficLightStatus` - current and previous overlap ids (zero or more of each type) with traffic light
- `YieldSignStatus` - current and previous overlap ids (zero of more of each type) with yield sign + ids of the obstacles to yield to