# Apollo Planning Module

Planning Module is responsible for the behaviour planning and trajectory generation

## Module overview

![Module overview](images/module_overview.png)

## Planning Component

The main purpose of the component is to serve as the adapter between the planning module and other modules

It connects to CyberRT to listen for incoming messages and write the resultant trajectory

Pipeline:

- Init
  - Create dependency injector
  - Instantiate and initialize current planning base
  - Load planning config
  - Configure message_process instance if offline learning is enabled
  - Create readers and writers
- Proc
  - Check rerouting (check if rerouting flag in the planning context is set) and send rerouting request if necessary
  - Fill local view with incoming data
  - Check input data in the local view
  - Handle learning data
  - Run planning base
  - Fill the result message and send it
  - Save the result to history

Contains:

- `traffic_light_reader` - traffic light detection messages reader
- `routing_reader` - routing response message reader
- `pad_msg_reader` - ?
- `relative_map_reader` - relative map message reader
- `story_telling_reader` - stories reader
- `planning_writer` - planning result trajectory writer
- `rerouting_writer` - rerouting request writer
- `planning_learning_data_writer` - dataset writer for the learning module 
- `traffic_light` - traffic light detections
- `routing` - routing response to plan on
- `pad_msg` - ?
- `relative_map` - relative map for navi planning
- `stories` - stories from the storytelling module
- `local_view` - local view data to store CyberRT messages
- `planning_base` - instance of planning base. On lane or navi planning. Will be described below
- `injector` - dependency injector. Will be described below
- `config` - planning config
- `message_process` - learning data manager

## Planning base (On-lane planning case)

Here we consider the planning base class, the On-lane planning base in particular

Pipeline of the On-lane planning:

- Init
  - CheckPlanningConfig
  - Init planning dispatcher
  - Load traffic rules
  - Reset local view
  - Initialize HD map
  - Initialize reference line provider
  - Dispatch and initialize current planner
  - If in learning mode, instantiate bird-view feature renderer
- RunOnce
  - Remember planning cycle start time
  - Update vehicle state and align timestamp (if failed, generate stop trajectory)
  - Update reference line provider with new routing (if failed, generate stop trajectory)
  - Update reference line provider with new vehicle state
  - Compute stitching trajectory
  - Update ego state with stitching trajectory and vehicle state (if failed, generate stop trajectory)
  - Initialize current frame and add obstacles to reference lines to create reference line info
  - Apply traffic rules to reference lines 
  - Execute Plan function
  - Fill the result trajectory message
  - Add current frame to frame history
- Plan
  - Run planner 
  - If was using open space planner, process related results
  - Else, extract the best reference line and related info, then fill the result trajectory message
  - Update last publishable trajectory with current trajectory

Contains:

- `local_view` - local view. Described [here](https://github.com/Sarrasor/BehaviourPlanning/tree/main/docs/apollo_docs/planning_module/frame.md) 
- `hdmap` - pointer to the HD map. Check out the documentation for the mapping module [here](https://github.com/Sarrasor/BehaviourPlanning/tree/main/docs/apollo_docs/map_module)
- `start_time` - time of the start of a planning cycle
- `seq_num` - current sequence number. Used as index in history
- `config` - planning config
- `traffic_rule_configs` - config for traffic rules sub-module
- `frame` - frame with planning data. Described [here](https://github.com/Sarrasor/BehaviourPlanning/tree/main/docs/apollo_docs/planning_module/frame.md)
- `planner` - planner instance. Will be discussed below
- `last_publishable_trajectory` 
- `planner_dispatcher` - creates the current planner instance
- `injector` - dependency injector, created from the injector from the planning component

On-lane planning has in addition:

- `last_routing` - last routing response. Used to check whether new routing response arrived and it is necessary to reset the planning state  
- `reference_line_provider` - reference line provider. Described [here](https://github.com/Sarrasor/BehaviourPlanning/tree/main/docs/apollo_docs/planning_module/reference_line.md)
- `planning_smoother` - smoothes planing trajectory. Will be described below

## Planner (Public road case)

Here we consider planner class, the public road planner in particular

Pipeline:

- Init
  - Initialize scenario manager
- Plan
  - Update the scenario manager with current point and frame
  - Get current scenario from the manager
  - Execute current scenario
  - If the scenario is done, update the manager

Contains: 

- `config` - planning config
- `scenario_manager` - scenario manager. Will be discussed below
- `scenario` - current scenario

## Scenario manager

Responsible for the current scenario selection

Pipeline:

- Init
  - Save planning config
  - Register scenarios (creates map of scenario configs)
  - Set default scenario type as `LANE_FOLLOW`
  - Create default scenario
- Update
  - Observe function call
  - ScenarioDispatch function call
- Observe
  - Clear `first_encountered_overlap_map`
  - Get first reference line info
  - Extract first encountered overlaps from the reference line
  - For each overlap, add current overlap if it is of type: `PNC_JUNCTION` or `SIGNAL` or `STOP_SIGN` or `YIELD_SIGN`
- ScenarioDispatch
  - If learning mode is active, update learning data and call ScenarioDispatchLearning function (it will return `LEARNING_MODEL_SAMPLE` scenario)
  - Else, call ScenarioDispatchNonLearning function
  - UpdatePlanningContext function call
  - If the current scenario is not the selected one, create current scenario
- ScenarioDispatchNonLearning
  - Will go through scenario select functions. Every select function will return its scenario type on success, and default scenario on failure. If current select function has returned its scenario, other select functions won't be invoked. The interesting thing is the order of invocation: "What dictates this order of invocation?" 
- UpdatePlanningContext
 - Will call several methods that will update Planning context data. Contents of the planning context are described below

Contains:

- `injector` - dependency injector from the planning base
- `planning_config` - planning config
- `config_map` - map: scenario type -> scenario config
- `current_scenario` - current scenario
- `default_scenario_type` - initial scenario
- `scenario_context` - current scenario data. Now is empty struct
- `first_encountered_overlap_map` - map: overlap type -> path overlap 

## Dependency Injector

Implementation of the Dependency Injection Design Pattern

Was introduced in the Apollo 6

The purpose is to pass dependencies to other modules like `planning_base` or `planner` while controlling the creation of dependency objects

Contains:

- `planning_context` - state of the planning
- `frame_history` - indexed queue(`seq_num` -> `Frame`) of max size `FLAGS_max_frame_history_num`
- `history` - history of planned trajectories, decisions on obstacles (decision is described [here](https://github.com/Sarrasor/BehaviourPlanning/tree/main/docs/apollo_docs/planning_module/decision_protobuf.md))
- `ego_info` - start point, vehicle state, front clear distance, ego vehicle config, and ego bounding box
- `vehicle_state` - vehicle state provider
- `learning_based_data` - future trajectory from the learning module

## Ego Info

## Trajectory stitcher

## Storytelling

## Pad messages

## Planning smoother