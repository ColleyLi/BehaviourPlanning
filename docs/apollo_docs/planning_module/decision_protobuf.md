# Decision protobuf

This protobuf contains data structure definitions related to decision making process

## DecisionResult

- `main_decision` - main decision. Described below
- `object_decision` - all obstacle decisions. Described below
- `vehicle_signal` - signal decision like right or left turn signal

## MainDecision

- `task` - task to perform. One of : `cruise`, `stop`, `estop`, `mission_complete`, `not_ready`, `parking`

### MainCruise

- `change_lane_type` - cruise current lane?

### MainStop

- `reason_code` - stop reason code. Described above
- `reason` - human-readable stop reason explanation
- `stop_point` - vehicle position when stopped
- `stop_heading` - vehicle heading when stopped
- `change_lane_type` - change lane type?

### MainEmergencyStop

- `reason_code` - code for emergency stop. One of: `internal_err`, `collision`, `st_find_path`, `st_make_decision`, `sensor_error`
- `reason` - human-readable stop reason explanation
- `task` - task to perform. One of: `hard_brake` or `cruise_to_stop`

### MainMissionComplete

- `stop_point` - vehicle position when stopped
- `stop_heading` - vehicle heading when stopped

### MainNotReady

- `reason` - human-readable explanation why the system is not ready

### MainParking

- `status` - parking status. Currently only `IN_PARKING`

## ObjectStatus

- `motion_type` - type of an obstacle. Can be `static` or `dynamic`
- `object_decision` - decision for an obstacle. Described below

## ObjectDecision

- `id` - id of a planning obstacle
- `perception_id` - id of a perception obstacle
- `object_decision` - actual decision. Can be: `ignore`, `stop`, `follow`, `yield`, `overtake`, `nudge`, `avoid`, `side_pass`. Each decision is a protobuf message with related information

### ObjectIgnore

Empty message

### ObjectStop

- `stop_reason_code` - reason why to stop. Can be: `head_vehicle`, `destination`, `pedestrian`, `obstacle`, `preparking`, `signal`, `stop_sign`, `yield_sign`, `clear_zone`, `crosswalk`, `creeper`, `reference_end`, `yellow_signal`, `pull_over`, `sidepass_safety`, `pre_open_space_stop`, `lane_change_urgency`, `emergency`
- `distance_s` - s coordinate of the stop position along the reference line 
- `stop_point` - vehicle position when stopped
- `stop_heading` - vehicle heading when stopped
- `wait_for_obstacle` - ids of obstacles to wait for

### ObjectFollow

- `distance_s` - minimum distance to keep
- `fence_point` - virtual fence location (displayed in Dreamview)
- `fence_heading` - virtual fence heading

### ObjectYield

- `distance_s` - minimum distance to keep
- `fence_point` - virtual fence location
- `fence_heading` - virtual fence heading
- `time_buffer` - time before the obstacle reaches the intersection point

### ObjectOvertake

- `distance_s` - minimum distance
- `fence_point` - virtual fence location
- `fence_heading` - virtual fence heading
- `time_buffer` - time before the obstacle reaches the intersection point

### ObjectNudge

- `type` - nudge type: `LEFT_NUDGE`, `RIGHT_NUDGE`, `NO_NUDGE`
- `distance_l` - lateral distance to keep. Should be positive if `type=LEFT_NUDGE`, and negative if `type=RIGHT_NUDGE`

### ObjectAvoid

Empty message. All objects will receive this type of decision in case of emergency stop

### ObjectSidePass

- `type` - type of the side-pass: `LEFT` or `RIGHT`

