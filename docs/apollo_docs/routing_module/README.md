# Apollo Routing Module

The routing module uses routing map and routing requests to create routing responses with path data

## Module code structure

```
│
├── common - flag configuration files
├── conf - protobuf configuration files
├── core - main routing logic implementation
├── dag - dag file for the CyberRT Routing Component
├── graph - routing graph classes definitions
├── launch - Routing Component launch file
├── proto - Routing protobuf data structures definitions
├── strategy - Strategies for the Navigator. (It uses the Strategy Design Pattern)
├── tools - Some helper utilities
  ├── routing_cast.cc - Send routing response from a file to CyberRT every second
  ├── routing_dump.cc - Save routing response to a file
  └── routing_tester.cc - Send routing request request from a file to CyberRT every second
└── topo_creator - `base_map` to `routing_map` converter implementation
```

## Module overview

![Module overview](images/module_overview.png)

## Input format

### Routing request

Routing request is usually generated in Dreamview and has the following format:

```
header
{
  timestamp_sec: 1617348030.1481693
  module_name: "dreamview"
  sequence_num: 0
}

waypoint
{
  id: "lane_410"
  s: 21.692566022640214
  pose
  {
    x: 358519.69714355469
    y: 6180750.5960693359
  }
}

waypoint
{
  id: "lane_416"
  s: 6.5860233596309747
  pose
  {
    x: 358560.62141369242
    y: 6180735.8507164549
  }
}
```

## Output format

### Routing response

The structure of the Routing response is defined in the `routing/proto/routing.proto`, you can check it out

Here is an example of a Routing response

```
header
{
  timestamp_sec: 1617348030.1530888
  module_name: "routing"
  sequence_num: 0
}

road
{
  id: "road_14"
  passage
  {
    segment
    {
      id: "lane_410"
      start_s: 21.692566022640214
      end_s: 41.997633175161575
    }
    segment
    {
      id: "lane_413"
      start_s: 0
      end_s: 16.464481778093997
    }
    segment
    {
      id: "lane_415"
      start_s: 0
      end_s: 6.582507217805504
    }
    can_exit: false
    change_lane_type: LEFT
  }

  passage
  {
    segment
    {
      id: "lane_411"
      start_s: 0
      end_s: 42.174547607606577
    }
    segment
    {
      id: "lane_414"
      start_s: 0
      end_s: 16.464422707083131
    }
    segment
    {
      id: "lane_416"
      start_s: 0
      end_s: 6.5860233596309747
    }
    can_exit: true
    change_lane_type: FORWARD
  }
}

measurement
{
  distance: 43.362050809627434
}

map_version: "gejson2proto_3.0"
status
{
  error_code: OK
  msg: "Success!"
}
```

## Routing map creation

In order to perform routing, we need to have the `routing_map`. Apollo creates it from the HD map called `base_map`. The code for the `routing_map` generation can be found in the `routing/topo_creator` folder

In Apollo routing map topology lanes represent nodes and relations between lanes are described by edges

Here is the picture that describes the situation:

![road_structure](images/road_structure.png)


## Links

- [Routing module review](https://github.com/daohu527/dig-into-apollo/tree/master/modules/routing)