# **Onion-LO: Why Does LiDAR Odometry Fail Across Different LiDAR Types and Scenarios?**

## 📌 What is Onion-LO

Onion-LO is a general-purpose LiDAR odometry framework that supports various types of LiDAR sensors and diverse complex scenarios, addressing the robustness issues faced by traditional methods under changes in hardware and environment.

### ✅ System overview

<div align="center">
  <img src="doc/1.png" width="600">
</div>

### ✅ Illustration of Multi-Scenario & Multi-LiDAR Support

<div align="center">
  <img src="doc/2.png" width="600">
</div>

### ✅ Demo


| Dataset 1 | Demo 1 | Dataset 2 | Demo 2 |
|-----------|--------|-----------|--------|
| KITTI     | ![](doc/kitti.gif) | NCLT      | ![](doc/nclt.gif) |
| NTU       | ![](doc/ntu.gif) | HILTI     | ![](doc/hilti.gif) |
| SEUA      | ![](doc/seua.gif) | SEUG      | ![](doc/seug.gif) |

---

## ⚙️ Dependency

Recommended System **Ubuntu 20.04 + ROS Noetic**

### 🔧 Build

```bash
# 1. Livox SDK
cd livox_sdk/build
cmake ..
make -j8
sudo make install

# 2. fmt
cd fmt/build
cmake ..
make -j8
sudo make install

# 3. Eigen
cd eigen/build
cmake ..
make -j8
sudo make install

# 4. Sophus
sudo apt-get install ros-noetic-sophus

# 5. Ceres Solver
sudo apt-get install libceres-dev

# 6. PCL
sudo apt install libpcl-dev
sudo apt install pcl-tools

# 7. OpenCV
sudo apt install libopencv-dev python3-opencv

# 8. Others
sudo apt-get install ros-noetic-tf2-sensor-msgs
sudo apt-get install ros-noetic-eigen-conversions
sudo apt-get install liboctomap-dev
sudo apt install ros-noetic-octomap ros-noetic-octomap-rviz

# 9. Install

## ⚙️ Run
After modifying the config file for your environment, you can run Onion-LO. Here is an example to test it with a Livox LiDAR.
<pre> ```
source devel/setup.bash
roslaunch onion-lv livox.launch
rosbag play your_data.bag
``` </pre>


