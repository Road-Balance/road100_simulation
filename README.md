# road100_simulation

* Description
```
ros2 launch road100_description description.launch.py 
```

* Ignition Gazebo
```
ros2 launch road100_ignition lab_world.launch.py

# joy control
ros2 launch road100_ignition joy_control.launch.py 
```

* SLAM
```
ros2 launch road100_slam slam_toolbox.launch.py
```

* Navigation
```
ros2 launch road100_ignition lab_world.launch.py use_rviz:=false

```