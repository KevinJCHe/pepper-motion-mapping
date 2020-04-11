# Pepper motion mapping
A ROS package to map human motion (captured using the **Nuitrack** skeleton tracking software) to humanoid robot **Pepper**.

Supports
- real-time motion mapping
- motion mapping from a recorded ros bag (topic of interest is /body_tracker/skeleton_v2). 

Tested on Ubuntu 16.04 with Microsft Kinect Sensor v1.

# Installation
1. Install Nuitrack SDK and linux drivers:
	- http://download.3divi.com/Nuitrack/doc/Installation_page.html

2. Install Python NAOqi SDK: 
	- http://doc.aldebaran.com/2-5/dev/community_software.html
	- http://doc.aldebaran.com/2-5/dev/python/install_guide.html#python-install-guide

2. Clone the following repo into your catkin workspace

		git clone https://gitlab.com/KevinJCHe22/motion-capture.git

	*Make sure to edit line 9 in the CMakeLists.txt file in nuitrack_body_tracker: set(NUITRACK_SDK_PATH /path/to/NuitrackSDK)*

3. Install these dependencies

        sudo apt-get install ros-kinetic-pepper-.*
        pip install torch==1.4.0+cpu torchvision==0.5.0+cpu -f https://download.pytorch.org/whl/torch_stable.html
        pip install future
        git clone https://github.com/sugarsweetrobotics/python_pepper_kinematics.git

# Using the ROS package

NOTE: make sure you have an RGBD sensor connected. 

        roslaunch move_pepper move_pepper.launch 

Optional command line arguments: 
- `only_sim:=<true or false, default is true. If false, it will connect with the phyiscal Pepper robot>`
- `use_bag:=<true of false, default is true. If false, real time video stream will be used instead of a recorded ros bag>`

Additional argument changes can be made in the joint_convert.launch file:
- `<arg name="bag_path" default="/path/to/rosbag" />`
- `<arg name="bag_playback_rate" default="1"/>`
- `<arg name="model_dir" default="$(find joint_converter)/models" />`

