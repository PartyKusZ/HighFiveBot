
# Packages

```bash
sudo apt-get install -Y ros-humble-pcl-ros ros-humble-pcl-conversions
```

```bash
sudo apt-get install -Y ros-foxy-pcl-ros ros-foxy-pcl-conversions
```

# Build

```bash
colcon build
source ./install/setup.sh
ros2 run point_cloud_processing recognise --ros-args -p use_sim_time:=True
```