# Robot Planner

Robot Planner is an example ROS2 package demonstrating high level planning and execution using PDDL and Behavior Trees. It provides utilities to convert symbolic plans into BehaviorTree.CPP XML and a node that executes actions sequentially. A set of simple Plansys2 action nodes illustrate how concrete robot behaviors can be implemented.

## Features

- **PlanExecutor** – ROS2 node that reads plans encoded in JSON and executes each action
  sequentially.
- **BTConverter** – converts plans to BehaviorTree.CPP XML for BT-based execution.
- **Action nodes** – example implementations (`move`, `pickup`, `drop`, etc.) built with
- `plansys2_upf_plugin` support – Plansys2 can run with the Unified Planning
  Framework using this plugin. The default solver is **OPTIC** and parameters
  are configured in `config/config.yaml`.
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
colcon build --packages-select robot_planner pddl_aggregator
```

ROS2 Foxy or later is required along with dependencies listed in `package.xml`.

## Running

After building, source the workspace and launch the example system:

```bash
source install/setup.bash
ros2 launch robot_planner planner.launch.py
```

This will start the plan executor, BT converter, the PDDL aggregator, Plansys2 bringup, and all demo action nodes.
Plans can be published as JSON strings to the `generated_plan` topic to trigger
execution.

To launch the PDDL aggregator by itself and override topic names:

```bash
ros2 launch pddl_aggregator pddl_aggregator.launch.py \
  map_pddl_topic:=/pddl/map \
  people_topic:=/people \
  objects_topic:=/objects \
  robots_topic:=/robots \
  problem_pddl_topic:=/pddl/problem
```

The aggregator exposes services for fetching the current domain/problem:

```bash
ros2 service call /get_problem_pddl std_srvs/srv/Trigger {}
ros2 service call /get_domain_pddl std_srvs/srv/Trigger {}
ros2 service call /pddl/get_problem std_srvs/srv/Trigger {}
ros2 service call /pddl/get_domain std_srvs/srv/Trigger {}
```

### Plansys2 Bringup

The launch file includes `plansys2_bringup` by default. If your install uses a different
launch file or argument names, override these from the command line:

```bash
ros2 launch robot_planner planner.launch.py \
  plansys2_launch_package:=plansys2_bringup \
  plansys2_launch_file:=plansys2_bringup_launch.py \
  plansys2_domain_arg:=domain_file \
  plansys2_problem_arg:=problem_file \
  plansys2_domain_file:=pddl/domain.pddl \
  plansys2_problem_file:=/tmp/problem.pddl
```

Planning is triggered on demand via Plansys2 services (for example, `/planner/get_plan`
and `/executor/execute_plan`).

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
