# Build
Zbuduj sklonowany projekt
```bash
mkdir -p src/
git clone https://github.com/PartyKusZ/HighFiveBot.git
colcon build
source install/setup.bash
```

# Run
Odpal symulację
```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py
```
# Test
W osobnym terminalu odpal test czy działa trajectory controller
```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [shoulder_pan_joint, elbow_joint , shoulder_lift_joint, wrist_1_joint,  wrist_2_joint, wrist_3_joint ],
    points: [
        { positions: [1.0, 0.5, -1.0, 1.0, 0.0, 0.0], time_from_start: { sec: 3, nanosec: 0 } },
    ]
  }
}"
```