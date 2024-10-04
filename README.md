# CURI Bimanual Teleoperation
This repository implements the retargeting algorithms 
in terms of dual Franka arm teleoperation and Softhand2 teleoperation 



## Prerequisites
### Hardwares
+ CURI Robot with Softhand2 or Parallel Gripper
+ Manus Data Glove
+ Xsens Motion Capture Suit
+ Moma Microphone LARK MIX12

### Softwares
+ Ubuntu 20.04
+ ROS Noetic

### Installation
```
1. git clone https://github.com/Zixin-Tang/CURI-Bimanual-Teleoperation.git
2. cd path/to/CURI-Bimanual-Teleoperation
3. catkin_make
```

## Quick Start
### Step 1
+ Connect the receiver of the Microphone with the client host and ensure it can be detected successfully
+ Put on Xsens suit and Manus glove, and fix the Motion Trackers in corresponding positions
+ Finish calibration in the Xsens software application (MVN Animate)
+ Ensure the Network Stream communication in MVN Animate is well-configured so that the client host can receive the UDP package from the host running MVN Animate

### Step 2
+ roslaunch xsens run_xsens.launch
  + This command will receive and parse the UDP packages from the MVN Animate host. The extracted pose transformations will be publiished as ROS topics such as "/xsens/left_finger_poses" and "/xsens/left_tcp"
  + If the communication between the MVN Animate host and the client host is successful, terminal will keep refreshing the information "[Xsens] Data received on (Current time)"

+ roslaunch eeteleop run_eeteleop.launch
  + This command will run the softhand teleoperation script. If the arg `enable_safechecker` is False, then the teleoperation will be started automatically and immediately. Otherwise, the teleoperation should be triggered by the human voice like "start" to start teleoperation and "close" to stop teleoperation
  + If teleoperation is beginning successfully, terminal will keep refreshing the information "[Softhand] Publishing synergies command on (Current time)"

+ roslaunch armteleop run_armteleop.launch
  + This command will run the dual-arm teleoperation script. If the arg `enable_safechecker` is False, then the teleoperation will be started automatically and immediately. Otherwise, the teleoperation should be triggered by the human voice like "start" to start teleoperation and "close" to stop teleoperation
  + If teleoperation is beginning successfully, terminal will keep refreshing the information "[Franka] Publishing control command on (Current time)"

## To Do List of the Next Version

- [ ] Support different bimanual gripper, such as left hand using Softhand2 and right hand using parallel gripper
- [ ] Improve workspace retargeting algorithm
- [ ] Improve inverse kinematics algorithm