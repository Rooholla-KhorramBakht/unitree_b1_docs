
# Goal

By the end of this tutorial, you should be able to sream the camera data on the dog as ROS messages or access the camera within Python directly. 

# Installation

First, let's remove any previously installed packages.

```bash
sudo apt-get remove ros-<distro>-librealsense* librealsense2-*
```
The tutorial assumes that jetpack is installed. 
## Install OpenCV with CUDA

Use the script provided [here](https://github.com/mdegans/nano_build_opencv) to compile and install opencv with CUDA support. 

## Install Librealsense

- Compile and install the librealsense with python support
```bash
sudo apt-get install git libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev -y
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build 
cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=release -DFORCE_RSUSB_BACKEND=false -DBUILD_WITH_CUDA=true -DBUILD_PYTHON_BINDINGS:bool=true -DCMAKE_CUDA_ARCHITECTURES=70 -DPYTHON_EXECUTABLE:FILEPATH=/usr/bin/python3
make -j$(($(nproc)-1))
sudo make install

```
**Note:** Choose the value of `DCMAKE_CUDA_ARCHITECTURES` according to (here)[https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/].
- update your `PYTHONPATH` environment variable to add the path to the pyrealsense library export `PYTHONPATH=$PYTHONPATH:/usr/local/lib`
    Alternatively, copy the build output (librealsense2.so and pyrealsense2.so) next to your script.

## Compile and Install Realsense-ROS
Create the catkin workspace:
```bash
mkdir catkin_ws && mkdir catkin_ws/src
cd catkin_ws/src
git clone https://github.com/ros-perception/vision_opencv.git 
cd vision_opencv && git checkout noetic
cd .. 
git clone git@github.com:Rooholla-KhorramBakht/realsense-ros.git
cd realsense-ros && git checkout ros1-legacy
cd ..
```

Add opencv to the Cmakefile according to the following ([source1](https://github.com/IntelRealSense/realsense-ros/issues/2467), [source2](https://github.com/IntelRealSense/realsense-ros/issues/2326#issuecomment-1107658481)):

```cmake
find_package( OpenCV REQUIRED )
include_directories(
include
${realsense2_INCLUDE_DIR}
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
${realsense2_LIBRARY}
${catkin_LIBRARIES}
${CMAKE_THREAD_LIBS_INIT}
${OpenCV_LIBRARIES}
)
```
Finally, install dependencies and compile the workspace:
```bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```
# Running
Finally, in order to capture the images from the camera, run the following command:

```bash
roslaunch realsense2_camera rs_camera.launch enable_color:=false enable_infra1:=true enable_infra2:=true enable_gyro:=true enable_accel:=true initial_reset:=true depth_fps:=60 infra_fps:=60 infra_width:=640 infra_height:=480 depth_width:=640 depth_height:=480
```

If you have more than one cameras, you can run them seperately based on their serial number:

```bash
roslaunch realsense2_camera rs_camera.launch enable_color:=false enable_infra1:=true enable_infra2:=true enable_gyro:=true enable_accel:=true initial_reset:=true depth_fps:=60 infra_fps:=60 infra_width:=640 infra_height:=480 depth_width:=640 depth_height:=480 camera:=cam_1 serial_no:=<serial-num>
```

For the Unitree robot, the following launch files are also avialable that simplify the launch process at the begining of each experiment:
- *Orin D455 Forward Looking Camera:* `orin_camera_blk_launch.launch`

# Cameras On CRRL B1
## Jetson with IP 192.168.123.23
- **Camera1 (Front Looking):** 141122079634
- **Camera2 (Front Downward Looking):** 141222071164

