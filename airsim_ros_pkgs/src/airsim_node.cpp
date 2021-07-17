#include "ros/ros.h"
#include "airsim_ros_wrapper.h"
#include <ros/spinner.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <fstream>

// MAVROS MSGS
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

std::ofstream file_gate;
ros::Publisher gate_poses_pub;
ros::Publisher gates_viz_pub, debug_gates_pub;
std::vector<msr::airlib::Pose> gateposes_arr;
double once = 0;
ros::Subscriber pwm_sub, ctrl_sub, mavros_attitude_control_sub, mavros_altitude_control_sub;
msr::airlib::MultirotorRpcLibClient client;
int counter = 0;
double altitude = 0;

void mavros_bodyrate_command_callback(mavros_msgs::AttitudeTarget attitude_cmd)
{

    double roll_rate,pitch_rate,yaw_rate,thrust;
    roll_rate = attitude_cmd.body_rate.x;
    pitch_rate = attitude_cmd.body_rate.y;
    yaw_rate = attitude_cmd.body_rate.z;
    thrust = attitude_cmd.thrust;
    client.moveByAngleRatesThrottleAsync(roll_rate, pitch_rate, yaw_rate, thrust, 1/100.0);
}

void mavros_altitude_callback(mavros_msgs::PositionTarget altitude_cmd)
{
    altitude = altitude_cmd.position.z;
}

void controller_callback(geometry_msgs::Pose control_cmd) {

    double x = control_cmd.orientation.x;
    double y = control_cmd.orientation.y;
    double z = control_cmd.orientation.z;
    double w = control_cmd.orientation.w;
    tf2::Quaternion q(x,y,z,w);
    
    double roll, pitch, yaw, altitude;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    altitude = control_cmd.position.z;
  
    client.moveByRollPitchYawZAsync(-roll, -pitch, yaw, altitude, 1/100.0);
    
}
void pwm_callback(geometry_msgs::Wrench pwms) 
{
    float pwm1 = pwms.force.x;
    float pwm2 = pwms.torque.x;
    float pwm3 = pwms.torque.y;
    float pwm4 = pwms.torque.z;
    
    client.moveByMotorPWMsAsync(pwm1,pwm2,pwm3,pwm4,1/100.0);
}

std::vector<msr::airlib::Pose> get_gate_poses(rpc::client &rpc_client, std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f>> &A)
{
    std::vector<msr::airlib::Pose> gateposes_array;
    msr::airlib::Pose gatepose;
    std::vector<std::string> gate_list = rpc_client.call("simListSceneObjects","Gate.*").as<std::vector<string>>();
    std::sort(gate_list.begin(),gate_list.end());
    for(auto s : gate_list){
        do{
            gatepose = rpc_client.call("simGetObjectPose",s,false).as<msr::airlib_rpclib::RpcLibAdapatorsBase::Pose>().to();
        }while(gatepose.position.hasNaN() || gatepose.orientation.vec().hasNaN());
        gateposes_array.push_back(gatepose);
        Eigen::Quaternionf tangent = gatepose.orientation*Eigen::Quaternionf{0,0,1,0}*gatepose.orientation.conjugate();
        A.push_back(std::pair<Eigen::Vector3f,Eigen::Vector3f>{gatepose.position,tangent.vec()});
    }
    A.push_back(std::pair<Eigen::Vector3f,Eigen::Vector3f>{A[A.size()-1].first+A[A.size()-1].second*5,A[A.size()-1].second});
    A.insert(A.begin(),std::pair<Eigen::Vector3f,Eigen::Vector3f>{A[0].first-A[0].second/A[0].second.norm()*5,A[0].second});
    for(auto g : A){
        std::cout << g.first << " : " << g.second << '\n';
    }

    return gateposes_array;
}

void timer_callback(const ros::TimerEvent&) {
    //counter++;

    if(counter <= 1)
    {
        geometry_msgs::PoseStamped gates_debug;
        geometry_msgs::PoseArray gate_poses_msg;
        gate_poses_msg.header.frame_id = "world_ned";
        gate_poses_msg.header.stamp = ros::Time::now();
    
        // Gate visualization
        visualization_msgs::MarkerArray gates_viz;
        

        for(int i = 0; i < gateposes_arr.size(); i++)
        {
            geometry_msgs::Pose tmp;
            gate_poses_msg.poses.push_back(tmp);
            gate_poses_msg.poses[i].position.x = gateposes_arr[i].position(0);
            gate_poses_msg.poses[i].position.y = gateposes_arr[i].position(1);
            gate_poses_msg.poses[i].position.z = gateposes_arr[i].position(2);
            file_gate << gateposes_arr[i].position(0) << "," << gateposes_arr[i].position(1) << "," << gateposes_arr[i].position(2) << "\n";


            gates_debug.header.frame_id = "world_ned";
            gates_debug.header.stamp = ros::Time::now();
            gates_debug.pose.position.x = gateposes_arr[i].position(0);
            gates_debug.pose.position.y = gateposes_arr[i].position(1);
            gates_debug.pose.position.z = gateposes_arr[i].position(2);
            


            double qx,qy,qz,qw;
            qx = gateposes_arr[i].orientation.x();
            qy = gateposes_arr[i].orientation.y();
            qz = gateposes_arr[i].orientation.z();
            qw = gateposes_arr[i].orientation.w();

            tf2::Quaternion quat_tf(qx,qy,qz,qw);

            // Conversion yaw (from NED)
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
            yaw = std::atan2(1-2*(qy*qy+qz*qz), -2*(qw*qz+qx*qy));

            

            tf2::Quaternion quat_newYaw;
            quat_newYaw.setRPY( roll, pitch, yaw ); // + 1.57
            quat_newYaw.normalize();
            geometry_msgs::Quaternion quat_msg;
            quat_msg = tf2::toMsg(quat_newYaw);

            gate_poses_msg.poses[i].orientation = quat_msg;
            gates_debug.pose.orientation = quat_msg;
            debug_gates_pub.publish(gates_debug); 
         
            // Gate visualization
            visualization_msgs::Marker gate_viz;
            gate_viz.header.frame_id = "world_ned";
            gate_viz.header.stamp = ros::Time::now();
            gate_viz.id = i;
            gate_viz.type = visualization_msgs::Marker::CUBE;

            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            gate_viz.pose.position.x = gate_poses_msg.poses[i].position.x;
            gate_viz.pose.position.y = gate_poses_msg.poses[i].position.y;
            gate_viz.pose.position.z = gate_poses_msg.poses[i].position.z;
            gate_viz.pose.orientation = quat_msg;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            gate_viz.scale.x = 0.1;
            gate_viz.scale.y = 1.0;
            gate_viz.scale.z = 1.0;
            // Set the color -- be sure to set alpha to something non-zero!
            gate_viz.color.r = 1.0f;
            gate_viz.color.a = 1.0;

            gates_viz.markers.push_back(gate_viz);
            
            
        }
   
        // Publish the marker
        gates_viz_pub.publish(gates_viz);   
        gate_poses_pub.publish(gate_poses_msg);

    }
    

}


int main(int argc, char ** argv)
{
   
    file_gate.open ("/home/bruno/gates.txt");

    std::ofstream myfile;
    myfile.open("/home/bruno/logging.txt");
    myfile << "Logging\n";
    std::cout << "AirSim\n";
    ros::init(argc, argv, "airsim_node");
    ros::NodeHandle nh; 
    ros::NodeHandle nh_private("~");
    gates_viz_pub = nh_private.advertise<visualization_msgs::MarkerArray>("gates_viz", 10);

    pwm_sub = nh.subscribe("/pwm",100, pwm_callback);
    ctrl_sub = nh.subscribe("/control_cmd",100,controller_callback);
    mavros_attitude_control_sub = nh.subscribe("/mavros/setpoint_raw/attitude",100, mavros_bodyrate_command_callback);
    mavros_altitude_control_sub = nh.subscribe("/mavros/setpoint_raw/local",100,mavros_altitude_callback);


    std::string host_ip = "localhost";
    nh_private.getParam("host_ip", host_ip);
    AirsimROSWrapper airsim_ros_wrapper(nh, nh_private, host_ip);

    if (airsim_ros_wrapper.is_used_img_timer_cb_queue_)
    {
        airsim_ros_wrapper.img_async_spinner_.start();
    }
    
    rpc::client rpc_client("127.0.0.1",41451);
    rpc_client.call("simLoadLevel","Qualifier_Tier_1");
    
    myfile << "API ENABLED\n";
    client.enableApiControl(true);
    myfile << "ARMED\n";
    client.armDisarm(true);
    //client.takeoffAsync(5)->waitOnLastTask();
    myfile << "Takeoff\n";
    
    myfile << "Positioned\n";
   
    //std::cout << "Commanded attitude\n";

    std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f>> A;
    gateposes_arr = get_gate_poses(rpc_client,  A);
    myfile << "getting gates\n";
    gate_poses_pub = nh_private.advertise<geometry_msgs::PoseArray>("/gates_ground_truth", 30);
    debug_gates_pub = nh.advertise<geometry_msgs::PoseStamped>("/debug_gates",100);
    rpc_client.call("simStartRace",1);     
    myfile.close();
    ros::Timer timer = nh_private.createTimer(ros::Duration(2), timer_callback);

    ros::spin();

    return 0;
} 
