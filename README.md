# Behaviour Planning
Custom Behaviour Planning module based on Hierarchical Behaviour Trees for Apollo 5.0

## Repo structure

```
BehaviourPlanning
│ 
├── apollo - Folder for Apollo system code
│   └──  - 
├── docs - Documentation
│   ├── apollo_docs - Documentation related to the Apollo system
|   |   ├── apollo_with_inno_sim - How to run Apollo with the Innopolis Simulator
|   |   ├── map_module - Apollo Map module documentation
|   |   ├── planning_module - Apollo Planning module documentation
│   │   └── routing_module - Apollo Routing module documentation
│   └── behaviour_planning - Everything related to the Behaviour Planning approach
├── simulator - Folder for the simulator
```

## Things to check out

### Apollo modules documentation
Documentation of Apollo `map`, `routing` and `planning` modules in the `BehaviourPlanning/docs` folder

### Visualisations
Visualization tools for map, Routing Response, ADC Trajectory are in the `BehaviourPlanning/apollo/my_scripts/visualization` folder

## Installation

Clone the repo into some folder: 

```
git clone https://github.com/Sarrasor/BehaviourPlanning.git
```

I will refer to this folder as `BehaviourPlanning` from now on

To prepare the environment, check out the `BehaviourPlanning/docs/apollo_with_inno_sim/README.md`