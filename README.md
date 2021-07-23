# autonomous_drone_racing

## Instructions  
cd ~  
wget https://github.com/microsoft/AirSim/archive/refs/tags/v1.4.0-linux.tar.gz  
tar -xvf AirSim-1.4.0-linux.tar.gz  
cd AirSim-1.4.0-linux/ros/src  
rm -rf airsim_ros_pkgs  
git clone https://github.com/brunopinto900/autonomous_drone_racing.git  
git clone https://github.com/brunopinto900/mavros_controllers.git  
git clone https://github.com/brunopinto900/mav_trajectory_generation.git  
git clone https://github.com/ethz-asl/mav_comm  
git clone https://github.com/ethz-asl/eigen_catkin.git  
git clone https://github.com/ethz-asl/nlopt.git  
git clone https://github.com/catkin/catkin_simple.git  
git clone https://github.com/ethz-asl/mavros.git  
git clone https://github.com/ethz-asl/glog_catkin.git  

cd ..  
catkin build  
