> A city simulation with prius for ROS2, Gazebo and Autoware!

Unfortunately, due to the support for multiple rendering enginer,
the script tag in the material tag does not work for custom input.
Only solid color is supported starting from Gazebo Harmonic.
Read more about it [here](https://discourse.openrobotics.org/t/migration-support-for-models-with-material-scripts/48782)
, if you are interested.


## Build

```bash
mkdir sim_ws/src -p
cd sim_ws/src
git clone https://github.com/BruceChanJianLe/prius_simulator.git --recurse-submodules
cd ..
colcon build
```

## Usage

```bash
# Prius in Empty World
ros2 launch prius_gazebo prius_empty_world.launch.py
```

```bash
# Prius in Simple City
ros2 launch prius_gazebo prius_simple_city.launch.py
```
