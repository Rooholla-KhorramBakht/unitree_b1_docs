
# Librealsense installation procedure

Update Ubuntu distribution, including getting the latest stable kernel

```bash
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
```

Clone the velodyne repository:

```bash
mkdir velodyne 
cd velodyne
mkdir src
cd src
git clone https://github.com/ros-drivers/velodyne
```

The install the dependencies using the rosdep tool:

```bash
cd ..
rosdep install --from-paths src -y --ignore-src
colcon build
```

# Running the Node
Run the following commands to start the node:
```bash
source ./install/setup.bash
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
```
If you want to visualize the the pointclouds, open rivz, select the pointcloud topic and set the world frame to `velodyne`.
