# Prerequirements
```
sudo apt install python3-rosdep2
```

# Build
Zbuduj sklonowany projekt
```bash
mkdir -p src/
git clone https://github.com/PartyKusZ/HighFiveBot.git src/
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
```

# Bez moveita
Odpal symulację bez moveita
```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:="ur3"
```

# Z moveitem
```bash
LC_NUMERIC=en_US.UTF-8 ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:="ur3"
```

# Test
W osobnym terminalu odpal test czy działa trajectory controller
```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [shoulder_pan_joint, elbow_joint , shoulder_lift_joint, wrist_1_joint,  wrist_2_joint, wrist_3_joint ],
    points: [
        { positions: [0.0, -1.57, 1.57, -3.14, -1.57, 0.0], time_from_start: { sec: 3, nanosec: 0 } },
    ]
  }
}"
```

ros2 run high_five_bot_driver high_five_bot_driver  --ros-args -p use_sim_time:=True
ros2 service call /go_to_hand std_srvs/srv/Trigger