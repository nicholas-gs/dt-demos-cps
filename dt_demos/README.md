# dt-demos-cps

## Demonstrations

### 1. __simple_joystick_driving__

Using a gamepad joystick controller, move the Duckiebot around. Only the minimal
amount of packages required are used. 

On the duckiebot,

```bash
ros2 launch simple_joystick_driving.launch.xml
```

On your computer/duckiebot where your game controller is attached to, run

```bash
ros2 launch dt-joystick joystick_node.launch.xml veh:=<robot name>
```

Requires the `joy` package that can be found in the [joystick_drivers](https://github.com/ros-drivers/joystick_drivers) repository. It should be part of the default ROS2 installation.

Double check that the `joy` topic is remapped so that the `joy_mapper_node` receives the published messages. 

## Credits

* https://github.com/duckietown/dt-core/tree/daffy/packages/duckietown_demos
