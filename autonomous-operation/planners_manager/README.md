# PlannersManager

ROS package to manage the use of one of the following planning algorithms:

* [PRM](../../prm/tree/PlannersManager)
* [RRT](../../rrt/tree/Planners)
* [Planner](../../planner)

## Configuration

It is possible to configure the launch package by terminal using the [arguments](#arguments).
The remaining configuration require the intervention of the user in the [parameters](#parameters) in the [launch file](./launch/Planners_Manager.launch).

### Arguments: 

| Name              | Type          | Description                                                |
| ----------------- | ------------- | ---------------------------------------------------------- |
| *argPlannerType*  | `int` | Value of Planner Type (1 - RRT; 2 - PRM) |
| *argGoalTolerance*| `float` | Radius of acceptance circle in meters (Default: 1.0 m) |

### Parameters:

| Name              | Type          | Description                                                |
| ----------------- | ------------- | ---------------------------------------------------------- |
| *paramRobotSize*  | `float` | Size of Safety Robot Rectangular Block X, Y, Z in meters (Ex: [1.0, 1.0, 1.0]) |
| *paramWorkspaceSize*| `float` | Size of Planner Workspace X, Y, Z in meters (Ex: [20.0, 20.0, 10.0])  |
| *paramWorkspaceCenter* | `float` | Offset of Workspace Center from the Robot (Ex: [5.0, 0.0, 0.0])|

### Planner Choise

Based on the coise of argument [argPlannerType](#argPlannerType), the package will run the correspondent planner.
And to do so, it will need the yaml files 

* [PRM yaml file](../../prm/tree/PlannersManager/config/params.yaml)
* [RRT yaml file](../../rrt/tree/Planners/config/params.yaml)

## Usage

To run this package, it is possible to use the [launch file](../launch/Planners_Manager.launch).

```
roslaunch planners_manager Planners_Manager.launch
```