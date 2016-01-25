*** A ROS Gazebo simulator for Happy Mini by Kosei Demura ***  
Happy Mini is a "kawaii" life-support robot based on the Kobuki base

1. Environment  
  Ubuntu14.04 and ROS Iindigo

2. Ubuntu Package Install  
  $ sudo apt-get install ros-indigo-openni-camera ros-indigo-rosbridge-suite ros-indigo-moveit-full ros-indigo-turtlebot-* ros-indigo-kobuki-* ros-indigo-moveit-python ros-indigo-laser-pipeline ros-indigo-laser-filters ros-indigo-hokuyo-node ros-indigo-depthimage-to-laserscan ros-indigo-gazebo-ros ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-msgs ros-indigo-gazebo-plugins 
   ros-indigo-gazebo-ros-control ros-indigo-cmake-modules ros -indigo-kobuki-gazebo-plugins ros-indigo -kobuki-gazebo ros-indigo-robot-pose-publisher ros-indigo-tf2-web-republisher ros-indigo-move-base-msgs ros-indigo-fake-localization liburdfdom-tools ros-indigo-laptop-battery-monitor ros-indigo-ar-track-alvar* ros-indigo-map-server ros-indigo-move-base* ros-indigo-simple-grasping 

3. Install  
  $ cd catkin_ws  
  $ git clone https://github.com/demulab/mini_sim.git

4. Build  
  $ cd catkin_ws  
  $ catkin_make  

5. Run  
(1) Mini 1.0 (with a 2 DOF arm)  
  $ roslaunch mini_description mini1.0.launch  
  $ rosrun mini_control mini_control  
(2) Mini 1.5 (with a 5 DOF arm)  
  $ roslaunch mini_description mini1.5.launch  
  $ rosrun mini_control mini_control  

