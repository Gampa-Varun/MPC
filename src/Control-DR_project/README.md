
# Controls Directed Research / Project

## Install additional ROS dependencies for building packages as below:
    - sudo apt update
    - sudo apt install ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink
    - sudo apt install ros-noetic-octomap-mapping ros-noetic-control-toolbox
    - sudo apt install python3-vcstool python3-catkin-tools protobuf-compiler libgoogle-glog-dev
    - rosdep update
    - sudo apt-get install ros-noetic-ros libgoogle-glog-dev
## Create a new ROS workspace and download the ROS packages for the robot:
    - mkdir -p ~/rbe502_project/src
    - cd ~/rbe502_project/src
    - catkin_init_workspace # initialize your catkin workspace
    - cd ~/rbe502_project
    - catkin init
    - cd ~/rbe502_project/src
    - git clone -b dev/ros-noetic https://github.com/Apatil10/CrazyS_Iris.git
    - git clone -b med18_gazebo9 https://github.com/gsilano/mav_comm.git
    - git clone -b branch_1 https://github.com/Apatil10/Controls-DR-Project.git
## Build the project workspace using python_catkin_tools
    - cd ~/rbe502_project
    - rosdep install --from-paths src -i
    - rosdep update
    - catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING= False
    - catkin build

## Source to your .bashrc file:
    - echo "source ~/rbe502_project/devel/setup.bash" >> ~/.bashrc
    - source ~/.bashrc

## Build the ROS package
    - cd ~/rbe502_project
    - catkin build

## Spawn the quadrotor in Gazebo
    - roslaunch rotors_gazebo iris_without_controller.launch

### Gazebo environment starts in the paused mode, so make sure that you start the simulation by clicking on the play button before proceed.

## Run the controller
    - Navigate to: 
        ~/rbe502_project/src/Controls-DR-Project/scripts 
    - These folder has all the controllers 
    - Open the *smc_iris.py* file in your code editor and run it!

## Files Developed for DR
    - *test.py* - This file is exact copy of the matlab simulation with '8' trajectory. No Gazebo
    - *test2.py* - This file is exact copy of the matlab simulation with desired 'square' trajectory. No Gazebo
    - *robust_adaptive_controller.py* - This file has the robust adaptive controller implemented on the iris drone platform
    - *robust_controller.py* - # This file has the robust controller implemented on the iris drone platform, without the adaptive part
    - *cascaded_pd_crazyflie.py* - Needs filename edit - This file is exact copy of the matlab simulation with desired 'circle' trajectory. No Gazebo
    - *tuning_Performance.py* - To check the performance of the robust controller