# Robot Planner

Robot Planner is an example ROS2 package demonstrating high level planning and execution using PDDL and Behavior Trees. It provides utilities to convert symbolic plans into BehaviorTree.CPP XML and a node that executes actions sequentially. A set of simple Plansys2 action nodes illustrate how concrete robot behaviors can be implemented.

## Features

- **PlanExecutor** – ROS2 node that reads plans encoded in JSON and executes each action
  sequentially.
- **BTConverter** – converts plans to BehaviorTree.CPP XML for BT-based execution.
- **Action nodes** – example implementations (`move`, `pickup`, `drop`, etc.) built with
  `plansys2_executor` to demonstrate integration with Nav2 and other systems.
- **Launch file** – starts the executor, converter and all action nodes with parameters
  from `config/config.yaml`.
- **PDDL Template Receiver** – subscribes to a topic with a domain PDDL template,
  fills placeholders and saves the generated domain file.
- **PDDL files** – example domain and problem description located in the `pddl` directory.

## Building

This package is intended for use in a ROS2 workspace. Clone the repository into the
`src` directory of your workspace and build with `colcon`:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# clone repository here
cd ..
colcon build --packages-select robot_planner
```

ROS2 Foxy or later is required along with dependencies listed in `package.xml`.

## Running

After building, source the workspace and launch the example system:

```bash
source install/setup.bash
ros2 launch robot_planner planner.launch.py
```

This will start the plan executor, BT converter and all demo action nodes.
Plans can be published as JSON strings to the `generated_plan` topic to trigger
execution.

### Domain Template

`PddlTemplateReceiver` listens for a PDDL domain template. The topic name can be
configured via the `domain_template_topic` parameter in `config/config.yaml`.

## Directory Overview

- `src/` – source code for nodes and action implementations
- `include/` – C++ headers
- `launch/` – launch file for starting the system
- `config/` – configuration and location mappings
- `pddl/` – example PDDL domain and problem files

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file
for details.
