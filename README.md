# Pepper motion mapping
A ROS package to map human motion (captured using the **Nuitrack** skeleton tracking software) to humanoid robot **Pepper**.

Supports
- real-time motion mapping
- motion mapping from a recorded ros bag (topic of interest is /body_tracker/skeleton_v2). 

Tested on Ubuntu 16.04 with Microsft Kinect Sensor v1. 
Currently does not work for Ubuntu 18.04 due to unsupported dependencies. 

# Installation
1. Install Nuitrack Linux Drivers
	- http://download.3divi.com/Nuitrack/doc/Installation_page.html
	
    **Important**: Make sure you remove OpenNI - it conflicts with the version supplied by Nuitrack

        sudo apt-get purge --auto-remove openni-utils

2. Install Nuitrack SDK:
	- http://download.3divi.com/Nuitrack/

3. Install Python NAOqi SDK: 
	- http://doc.aldebaran.com/2-5/dev/community_software.html
	- http://doc.aldebaran.com/2-5/dev/python/install_guide.html#python-install-guide

4. Clone the following repo into your catkin workspace

		git clone https://gitlab.com/asblab/motion-capture.git

	*Make sure to edit line 9 in the CMakeLists.txt file in nuitrack_body_tracker: set(NUITRACK_SDK_PATH /path/to/NuitrackSDK)*

5. Install these dependencies

        sudo apt-get install ros-kinetic-pepper-.*
        pip install torch==1.4.0+cpu torchvision==0.5.0+cpu -f https://download.pytorch.org/whl/torch_stable.html
        pip install pepper_kinematics
        pip install future
        pip install pandas

# Using the ROS package

        roslaunch move_pepper move_pepper.launch 

Optional command line arguments: 
- `only_sim:=<true or false, default is true. If false, it will connect with the phyiscal Pepper robot>`
- `use_bag:=<happy, sad, angry, surprise, fear, disgust, default is None. If None, real time video stream will be used instead of a recorded ros bag>`
**NOTE**: make sure you have an RGBD sensor connected to your device if use_bag:=None

Additional argument changes can be made in the joint_convert.launch file:
- `<arg name="<emotion>_bag_path" default="/path/to/rosbag" />`
- `<arg name="bag_playback_rate" default="1"/>`
- `<arg name="model_dir" default="$(find joint_converter)/models" />`