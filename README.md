# Jetson nano - VIO path - OpenVins

Upgrade swap memory

```jsx
1. df -h This will show your file system and how much space you have left.
2. sudo fallocate -l 8G /swapfile Create a 8 GB of swap
3. sudo chmod 600 /swapfileChange file permissions
4. sudo mkswap /swapfile
5. sudo swapon /swapfile
6. free -mThis will show you the swap file is on. You can also pull up the System MonitorHowever this is only temporary. If you reboot, swap file is gone.
7. sudo nano /etc/fstab
8. Within this file add the line “/swapfile none swap 0 0”. Do not include the quotes.Exit and save file
9. Now you can reboot and your Swap will be activated.
```

## Ceres-2.0.0 and Eigen 3.3.7

```jsx
$ wget -O eigen.zip [https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip](https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip) #check version
$ unzip eigen.zip
$ cd eigen-3.3.7
& mkdir build && cd build
$ cmake .. && sudo make install
```

```jsx
# Install Dependencies
sudo apt update -y && sudo apt upgrade -y

sudo apt-get install libgoogle-glog-dev libgflags-dev -y
sudo apt-get install libatlas-base-dev -y
sudo apt-get install libeigen3-dev -y
sudo apt-get install libsuitesparse-dev -y

# Download
cd /tmp
CERES_VERSION="ceres-solver-2.0.0"
CERES_ARCHIVE="$CERES_VERSION.tar.gz"
wget http://ceres-solver.org/$CERES_ARCHIVE
tar xfv $CERES_ARCHIVE

# Install
cd $CERES_VERSION
mkdir build
cd build
NUM_CPU_CORES=$(grep -c ^processor /proc/cpuinfo)
cmake ..
cmake --build . -j $NUM_CPU_CORES

sudo apt install checkinstall libssl-dev -y
sudo checkinstall --pkgname ceres-solver
```

## CUDA

```bash
$ sudo apt install gcc make
$ sudo reboot

$ gedit ~/.bashrc
# type and save
export PATH=<CUDA_PATH>/bin:$PATH #ex: /usr/local/cuda-11.1
export LD_LIBRARY_PATH=<CUDA_PATH>/lib64:$LD_LIBRARY_PATH #ex : /usr/local/cuda-11.1
$ . ~/.bashrc

# check if installed well
$ dpkg-query -W | grep cuda
```

```jsx
# check installed cuda version
$ nvcc --version
# if nvcc --version does not print out CUDA,
$ gedit ~/.profile
# type below and save
export PATH=<CUDA_PATH>/bin:$PATH #ex: /usr/local/cuda-11.1
export LD_LIBRARY_PATH=<CUDA_PATH>/lib64:$LD_LIBRARY_PATH #ex : /usr/local/cuda-11.1
$ source ~/.profile
```

if error occured during the dpkg -i 

```jsx
sudo apt-get -o Dpkg::Options::="--force-overwrite" install --fix-broken
```

## OPENCV

```jsx
$ sudo apt-get purge libopencv* python-opencv
$ sudo apt-get update
$ sudo apt-get install -y build-essential pkg-config
$ sudo apt-get install -y cmake libavcodec-dev libavformat-dev libavutil-dev \
    libglew-dev libgtk2.0-dev libgtk-3-dev libjpeg-dev libpng-dev libpostproc-dev \
    libswscale-dev libtbb-dev libtiff5-dev libv4l-dev libxvidcore-dev \
    libx264-dev qt5-default zlib1g-dev libgl1 libglvnd-dev pkg-config \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev mesa-utils #libeigen3-dev # recommend to build from source : http://eigen.tuxfamily.org/index.php?title=Main_Page
$ sudo apt-get install python2.7-dev python3-dev python-numpy python3-numpy
$ mkdir <opencv_source_directory> && cd <opencv_source_directory>
$ wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.1.zip # check version
$ unzip opencv.zip
$ cd <opencv_source_directory>/opencv && mkdir build && cd build
# check your BIN version : http://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/
# 8.6 for RTX3080 7.2 for Xavier, 5.2 for GTX TITAN X, 6.1 for GTX TITAN X(pascal), 6.2 for TX2
# -D BUILD_opencv_cudacodec=OFF #for cuda10-opencv3.4
$ cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_C_COMPILER=gcc-6 \
      -D CMAKE_CXX_COMPILER=g++-6 \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_GENERATE_PKGCONFIG=YES \
      -D WITH_CUDA=ON \
      -D CUDA_ARCH_BIN=8.6 \
      -D CUDA_ARCH_PTX="" \
      -D ENABLE_FAST_MATH=ON \
      -D CUDA_FAST_MATH=ON \
      -D WITH_CUBLAS=ON \
      -D WITH_LIBV4L=ON \
      -D WITH_GSTREAMER=ON \
      -D WITH_GSTREAMER_0_10=OFF \
      -D WITH_QT=ON \
      -D WITH_OPENGL=ON \
      -D BUILD_opencv_cudacodec=OFF \
      -D CUDA_NVCC_FLAGS="--expt-relaxed-constexpr" \
      -D WITH_TBB=ON \
      ../
$ time make -j8 # 8 : numbers of core
$ sudo make install
$ sudo rm -r <opencv_source_directory> #optional
```

## OPENVins Setup

```jsx
cd ~/catkin_ws/src
git clone https://github.com/rpng/open_vins/
cd ..
catkin build
```

## Camera Setup (Csi-camera and rotation settings)

```jsx
https://github.com/JetsonHacksNano/CSI-Camera
```

To start calibration you need to get CSI-camera package for taking raw image data. INside the workspace.

```jsx
roslaunch jetson_csi_cam jetson_csi_cam.launch width:=1280 height:=720 fps:=30

roslaunch jetson_csi_cam jetson_csi_cam.launch width:=960 height:=540
-------------------------------------
roslaunch jetson_camera jetson_camera.launch 

To change the camera settings
cd catkin_ws/src/CSI_camera/launch
sudo nano jetson_camera.launch

1080px720p =30 - 60 fps, 
640x480p =90 fps,
3280 x 2464 21 fps
```

![Screenshot from 2022-03-16 00-47-11.png](Jetson%20nano%20-%20VIO%20path%20-%20OpenVins%202f180045f04242aca1011ad64d77372e/Screenshot_from_2022-03-16_00-47-11.png)

 

## IMU (Inertial Measurement Unit Setup on ROS Melodic)

TO install all dependencies without error please update your python version to ≤3.7.

```jsx
pip3 install adafruit-blinka
pip3 install adafruit-circuitpython-mpu6050
sudo apt-get install python3-smbus2 # smbus caused error so I used smbus2 and I imported related files like "import smbus2 as smbus" change this in imu_node.py file inside the scripts folder
cd catkin_ws/src/mpu_6050_driver/scripts
chmod +x imu_node.py
chmod +x tf_broadcaster_imu.py
catkin build

rospack find mpu_6050_driver
rosdep update
rosdep check mpu_6050_driver

```

If you get all system dependencies have been satisfied you are good to keep going. If you get error messages related to imu_tools go back to the catkin_ws package src folder. Then you will install the imu_tools package from the source.

```jsx
sudo apt-get install git-core
git clone -b melodic https://github.com/ccny-ros-pkg/imu_tools.git
rosdep install imu_tools
cd ~/catkin-ws
catkin_make
```

```jsx
samet@samet:~/workspace/catkin_ws_ov$ rostopic list
/csi_cam_0/camera_info
/csi_cam_0/image_raw
/csi_cam_0/image_raw/compressed
/csi_cam_0/image_raw/compressed/parameter_descriptions
/csi_cam_0/image_raw/compressed/parameter_updates
/csi_cam_0/image_raw/compressedDepth
/csi_cam_0/image_raw/compressedDepth/parameter_descriptions
/csi_cam_0/image_raw/compressedDepth/parameter_updates
/csi_cam_0/image_raw/theora
/csi_cam_0/image_raw/theora/parameter_descriptions
/csi_cam_0/image_raw/theora/parameter_updates
/imu/data
/rosout
/rosout_agg
/temperature
/tf
```

![Untitled](Jetson%20nano%20-%20VIO%20path%20-%20OpenVins%202f180045f04242aca1011ad64d77372e/Untitled.png)

```jsx
////####/IMU/DATA OUTPUT FROM rostopic echo /imu/data
header: 
  seq: 16280
  stamp: 
    secs: 1648567865
    nsecs: 762681007
  frame_id: "imu_link"
orientation: 
  x: -0.742927750142
  y: -0.0241602520371
  z: 0.0
  w: 0.668935453008
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity: 
  x: -2.48854961832
  y: -1.50381679389
  z: -1.49618320611
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration: 
  x: 0.033203125
  y: -1.02099609375
  z: -0.10791015625
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

## Kalibr tool for Ubuntu18.04 Melodic

This process completed for taking calibration file

```jsx
https://github.com/ori-drs/kalibr
```

```jsx
rosbag record -O camera_calibration.bag /csi_cam_0/image_raw

rosbag record -O camera_imu_calibration.bag /csi_cam_0/image_raw /imu/data
```

After getting record from csi camera topic and imu_tools topics. We faced with camera problem which is providing rotated image raw on rviz.

First attempt ros record was too fast to determine the correct things so we will start record slowly to all axes.

![Screenshot from 2022-03-10 04-12-06.png](Jetson%20nano%20-%20VIO%20path%20-%20OpenVins%202f180045f04242aca1011ad64d77372e/Screenshot_from_2022-03-10_04-12-06.png)

## Kalibr Tool Usage

[https://blog.actorsfit.com/a?ID=00850-a0378441-d4aa-4d34-a884-fe8f91cc3485](https://blog.actorsfit.com/a?ID=00850-a0378441-d4aa-4d34-a884-fe8f91cc3485)

```jsx
sudo apt-get install python-setuptools python-rosinstall ipython libeigen3-devlibboost-all-dev doxygen libopencv-dev ros-indigo-vision-opencvros-indigo-image-transport-plugins ros-indigo-cmake-modulespython-software-properties software-properties-common libpoco-devpython-matplotlib python-scipy python-git python-pip ipython libtbb-devlibblas-dev liblapack-dev python-catkin-tools libv4l-dev
```

```jsx
sudo -i
pip install python_igraph
```

```jsx
mkdir -p~/kalibr_workspace/src
cd ~/kalibr_workspace
source/opt/ros/indigo/setup.bash
catkin init
catkin config --extend/opt/ros/indigo
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```jsx
cd ~/kalibr_workspace/src
git clone https://github.com/ethz-asl/Kalibr.git

cd ~/kalibr_workspace
catkin build -DCMAKE_BUILD_TYPE=Release -j4 #this can change your usage I used j3 mostly 
```

```jsx
source ~/kalibr_workspace/devel/setup.bash #basic ROS source
```

```jsx
#####CALIBRATION

cd ~/kalibr_workspace

kalibr_calibrate_imu_camera --target april_6x6_50x50cm.yaml --cam mynteye.yaml --imu imu.yaml --bag stereo_imu_calibra.bag

kalibr_calibrate_imu_camera --cam chain.yaml --target aprilgrid.yaml --imu imu.yaml --bag camera_imu_calibration.bag.active --show-extraction

kalibr_calibrate_imu_camera --cam chain.yaml --target aprilgrid.yaml --imu imu.yaml --bag camera_imu_calibration.bag --timeoffset-padding 0.2
```

## Rosbag Recording and Getting results from kalibr

```jsx
####ROSBAG RECORD FILE INFO : 
samet@samet:~/kalibr_ws$ rosbag info camera_imu_calibration.bag.active 
path:        camera_imu_calibration.bag.active
version:     2.0
duration:    1:06s (66s)
start:       Mar 29 2022 19:01:02.08 (1648569662.08)
end:         Mar 29 2022 19:02:08.12 (1648569728.12)
size:        1.5 GB
messages:    2726
compression: none [1015/1015 chunks]
types:       sensor_msgs/Image [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/Imu   [6a62c6daae103f4ff57a132d6f95cec2]
topics:      /csi_cam_0/image_raw   1015 msgs    : sensor_msgs/Image
             /imu/data              1711 msgs    : sensor_msgs/Imu
```

![Untitled](Jetson%20nano%20-%20VIO%20path%20-%20OpenVins%202f180045f04242aca1011ad64d77372e/Untitled%201.png)

[camchain-imucam-camera_imu_calibration.yaml](Jetson%20nano%20-%20VIO%20path%20-%20OpenVins%202f180045f04242aca1011ad64d77372e/camchain-imucam-camera_imu_calibration.yaml)

[report-imucam-camera_imu_calibration.pdf](Jetson%20nano%20-%20VIO%20path%20-%20OpenVins%202f180045f04242aca1011ad64d77372e/report-imucam-camera_imu_calibration.pdf)

[results-imucam-camera_imu_calibration.txt](Jetson%20nano%20-%20VIO%20path%20-%20OpenVins%202f180045f04242aca1011ad64d77372e/results-imucam-camera_imu_calibration.txt)

![Untitled](Jetson%20nano%20-%20VIO%20path%20-%20OpenVins%202f180045f04242aca1011ad64d77372e/Untitled%202.png)

The output of the kalibr tool gaves this results pdf and some parameter for use them on the openvins. -5 between 5 values more stable to estimate the movements.

```jsx
rostosamet@samet:~/workspace/catkin_ws_ov$ rostopic list
/clicked_point
/csi_cam_0/camera_info
/csi_cam_0/image_raw
/csi_cam_0/image_raw/compressed
/csi_cam_0/image_raw/compressed/parameter_descriptions
/csi_cam_0/image_raw/compressed/parameter_updates
/csi_cam_0/image_raw/compressedDepth
/csi_cam_0/image_raw/compressedDepth/parameter_descriptions
/csi_cam_0/image_raw/compressedDepth/parameter_updates
/csi_cam_0/image_raw/theora
/csi_cam_0/image_raw/theora/parameter_descriptions
/csi_cam_0/image_raw/theora/parameter_updates
/imu/data
/initialpose
/move_base_simple/goal
/ov
/ov_msckf/loop_feats
/ov_msckf/pathgt
/ov_msckf/pathimu
/ov_msckf/points_aruco
/ov_msckf/points_msckf
/ov_msckf/points_slam
/rosout
/rosout_agg
/temperature
/tf
/tf_static
```

## Progress

```jsx
samet@samet:~/workspace/datasets$ rostopic info /csi_cam_0/image_raw
Type: sensor_msgs/Image

Publishers: 
 * /csi_cam_0 (http://localhost:40519/)

Subscribers: None

samet@samet:~/workspace/datasets$ rostopic info /imu/data
Type: sensor_msgs/Imu

Publishers: 
 * /imu_node (http://localhost:46515/)

Subscribers: 
 * /tf_broadcaster_imu (http://localhost:33425/)
```

```jsx
<launch>

    <!-- mono or stereo and what ros bag to play -->
    <arg name="max_cameras" default="1" />
    <arg name="use_stereo"  default="false" />
    
    <!-- imu starting thresholds -->
    <arg name="init_window_time"  default="0.1" />
    <arg name="init_imu_thresh"   default="0.05"/>

    <!-- saving trajectory path and timing information -->
    <arg name="dosave"      default="false" />
    <arg name="dotime"      default="false" />
    <arg name="path_est"    default="/tmp/traj_estimate.txt" />
    <arg name="path_time"   default="/tmp/traj_timing.txt" />

    <!-- MASTER NODE! -->
    <remap from="/ov_msckf/odomimu" to="/mavros/odometry/out"/>
    <node name="run_subscribe_msckf" pkg="ov_msckf" type="run_subscribe_msckf" output="screen" clear_params="true" required="true">

        <!-- bag topics -->
        <param name="topic_imu"      type="string" value="/imu/data" />
        <param name="topic_camera0"  type="string" value="/csi_cam_0/image_raw" />

        <!-- world/filter parameters -->
        <param name="use_fej"                type="bool"   value="true" />
        <param name="use_imuavg"             type="bool"   value="true" />
        <param name="use_rk4int"             type="bool"   value="true" />
        <param name="use_stereo"             type="bool"   value="$(arg use_stereo)" />
        <param name="calib_cam_extrinsics"   type="bool"   value="true" />
        <param name="calib_cam_intrinsics"   type="bool"   value="true" />
        <param name="calib_cam_timeoffset"   type="bool"   value="true" />
        <param name="calib_camimu_dt"        type="double" value="0.0" />
        <param name="do_calib_camera_pose"   type="bool"   value="true" />
        <param name="max_clones"             type="int"    value="15" />
        <param name="max_slam"               type="int"    value="150" />
        <param name="max_slam_in_update"     type="int"    value="40" />
        <param name="max_msckf_in_update"    type="int"    value="40" />
        <param name="max_cameras"            type="int"    value="$(arg max_cameras)" />
        <param name="dt_slam_delay"          type="double" value="1" />
        <param name="init_window_time"       type="double" value="$(arg init_window_time)" />
        <param name="init_imu_thresh"        type="double" value="$(arg init_imu_thresh)" />
        <rosparam param="gravity">[1.57950687 -7.05490347 -6.63095732 ]</rosparam>
        <param name="feat_rep_msckf"         type="string" value="GLOBAL_3D" />
        <param name="feat_rep_slam"          type="string" value="ANCHORED_INVERSE_DEPTH_SINGLE" /> <!-- ANCHORED_MSCKF_INVERSE_DEPTH, ANCHORED_INVERSE_DEPTH_SINGLE -->
        <param name="feat_rep_aruco"         type="string" value="ANCHORED_INVERSE_DEPTH_SINGLE" /> <!-- ANCHORED_MSCKF_INVERSE_DEPTH, ANCHORED_INVERSE_DEPTH_SINGLE -->

        <!-- zero velocity update parameters -->
        <param name="try_zupt"               type="bool"   value="false" />
        <param name="zupt_chi2_multipler"    type="int"    value="0" />
        <param name="zupt_max_velocity"      type="double" value="0.5" />
        <param name="zupt_noise_multiplier"  type="double" value="50" />

        <!-- timing statistics recording -->
        <param name="record_timing_information"   type="bool"   value="$(arg dotime)" />
        <param name="record_timing_filepath"      type="string" value="$(arg path_time)" />

        <!-- tracker/extractor properties -->
        <param name="use_klt"            type="bool"   value="true" />
        <param name="num_pts"            type="int"    value="200" />
        <param name="fast_threshold"     type="int"    value="5" />
        <param name="grid_x"             type="int"    value="10" />
        <param name="grid_y"             type="int"    value="8" />
        <param name="min_px_dist"        type="int"    value="10" />
        <param name="knn_ratio"          type="double" value="0.65" />
        <param name="downsample_cameras" type="bool"   value="false" />
        <param name="multi_threading"    type="bool"   value="true" />
        <param name="histogram_method"   type="string" value="NONE" /> <!-- NONE, HISTOGRAM, CLAHE -->

        <!-- aruco tag/mapping properties -->
        <param name="use_aruco"        type="bool"   value="false" />
        <param name="num_aruco"        type="int"    value="1024" />
        <param name="downsize_aruco"   type="bool"   value="true" />

        <!-- sensor noise values / update -->
        <param name="up_msckf_sigma_px"            type="double"   value="1" />
        <param name="up_msckf_chi2_multipler"      type="double"   value="1" />
        <param name="up_slam_sigma_px"             type="double"   value="1" />
        <param name="up_slam_chi2_multipler"       type="double"   value="1" />
        <param name="up_aruco_sigma_px"            type="double"   value="1" />
        <param name="up_aruco_chi2_multipler"      type="double"   value="1" />
>
        <param name="gyroscope_noise_density"      type="double"   value="1.73" />
        <param name="gyroscope_random_walk"        type="double"   value="1.4142" />
        <param name="accelerometer_noise_density"  type="double"   value="1.73" />
        <param name="accelerometer_random_walk"    type="double"   value="1" />

        <!-- camera intrinsics -->
        <rosparam param="cam0_wh">[960, 540]</rosparam>
        <param name="cam0_is_fisheye" type="bool" value="false" />

        <rosparam param="cam0_k">[973.762621764148, 973.6022260612037, 462.42631396946325, 270.2344985963051]</rosparam>

        <rosparam param="cam0_d">[0.17910436988939454, -0.3204102665851758, 0.0036771944409561353, 0.0029979920150217335]</rosparam>

        <!-- camera extrinsics -->
        <rosparam param="T_C0toI">
            [
             -0.96456044 -0.05623926  0.25779897 -0.09936548,
              0.05941996 -0.99822267  0.00455719  0.01781487,
              0.25708449  0.01971409  0.96618783 -0.57984049,
              0.0, 0.0, 0.0, 1.0
            ]
        </rosparam>
  

    </node>

    <node name="rvizvisualisation" pkg="rviz" type="rviz" output="log" args="-d $(find ov_msckf)/launch/display.rviz" />

</launch>
```

[Jetson Nano taken errors/](https://www.notion.so/Jetson-Nano-taken-errors-74ed08a8336a4469949f180041ef156b)