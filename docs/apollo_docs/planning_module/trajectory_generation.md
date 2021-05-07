# Trajectory Generation

In this section we will review Apollo's approach to trajectory generation

High-level algorithm is as follows:

- Create reference line info lanes that capture obstacles, traffic rules and other related information
- Decide if change lane is needed, or if it can be reused, or if lane borrow maneuver is required for each reference line info 
- Generate path bounds for each reference line based on the previous decisions
- Generate paths based on path bounds
- Evaluate generated paths and select the best one

## Frenet coordinate system

Trajectory generation in Apollo is performed in the Frenet coordinate system

## Path Decision

The class represents all obstacle decisions for a given Reference Line

- Lateral and Longitudinal decisions have type `ObjectDecisionType`
- Main stop decision has type `MainStop`

All decision type descriptions can be found [here](https://github.com/Sarrasor/BehaviourPlanning/tree/main/docs/apollo_docs/planning_module/decision_protobuf.md)

Obstacle class description can be found [here](https://github.com/Sarrasor/BehaviourPlanning/tree/main/docs/apollo_docs/planning_module/obstacle.md)

Usages:

- `AddObstacle()` - adds obstacle to obstacle list
- `AddLateralDecision()` - adds given lateral decision to obstacle with given id
- `AddLongitudinalDecision()` - adds given longitudinal decision to obstacle with given id
- `Find()` - finds obstacle by given id
- `FindPerceptionObstacle()` - finds perception obstacle inside obstacles by given id
- `SetSTBoundary()` -  adds given ST boundary to obstacle with given id
- `EraseSTBoundaries()` - erases ST boundaries of all obstacles
- `MergeWithMainStop()` - creates main stop decision from given Object Stop decision and given obstacle id. Creation can fail if given obstacle if farther than the current stop decision obstacle or if the given obstacle is out of s-range 

Contains:

- `obstacles` - indexed list of it-> Obstacle with obstacles
- `main_stop` - main stop decision
- `stop_reference_line_s` - s coordinate of the main stop decision

## Path generation

Paths are represented as `PathData` class

High-level path generation is as follows:

- Decide lane changes (influences path boundaries)
- Decide to reuse path (if reusable, no need to perform lane borrow, bound and path generation decisions)
- Decide lane-borrows (influences path boundaries)
- Decide path bounds (use previous decisions to generate path boundaries)
- Generate paths

### Path boundaries

In order to constrain search space for path-optimization `PathBoundsDecider` class will generate path boundaries. Here are possible boundaries:

- `FallbackBoundary` - takes into account lane info and current position of the ADC. No static obstacles are taken into consideration. Static obstacle processing task is left to the speed profile generator
- `RegularPathBoundary` - takes into account lane info, current position of the ADC and static obstacles. Will also use Lane-borrow decisions and generate one boundary for each lane-borrow decision + no lane-borrow boundary. Regular boundaries will also receive blocking obstacle id if exists.
- `LaneChangePathBoundary` - takes into account lane info, current position of the ADC and static obstacles. Will also use lane change state to generate the boundary. Lane change maneuver consists of nudging to the border of the lane for `FLAGS_lane_change_prepare_length` and then performing the actual change. So, the boundary is set accordingly 
- `PullOverPathBoundary` - takes into account road size, lane boundaries and static obstacles to generate pull-over bounds. Will also use pullover state. If no pullover position is in the state, will generate one
While processing static obstacles, bounds decider will extend the boundary to all possible side-pass directions for every obstacle

**Important: no dynamic obstacles are considered in the path boundary generation process now**

Every generated boundary is inserted into candidate path boundaries list along with an appropriate label: `fallback`, `regular`, `regular/pullover`, `regular/lanechange`, `regular/{self | left | right}/{reverse | forward | "empty_string"}`

Worth noting, that since path bounds are generated for single lane there can be the following combinations of candidate boundaries:

- `Fallback` + `Pulllover`
- `Fallback` + `LaneChange`
- `Fallback` + `Regular`

### Piecewise-Jerk Path Optimizer

This class will generate actual paths based on the results of path boundary generation

For each candidate boundary it generates `PathData` based on actual bounds and previous path data

## Speed generation

Speed profiles are represented as `SpeedData` class

### STBoundary

Speed profile generation is performed via ST-graph optimization. This class prepares data for optimization

## References

- [Inspiration for Apollo EM planner](https://www.researchgate.net/profile/Moritz-Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af/Optimal-Trajectory-Generation-for-Dynamic-Street-Scenarios-in-a-Frenet-Frame.pdf)
- [Apollo EM planner paper](https://arxiv.org/pdf/1807.08048.pdf)
- [Apollo EM planning algorithm review](https://mp.weixin.qq.com/s/ao5hC_3A7fn8_L_PFw399A)
- [Apollo 6.0 planner review](https://www.zhihu.com/column/c_1311678411488632832)

