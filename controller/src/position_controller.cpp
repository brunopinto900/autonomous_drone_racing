#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Wrench.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/spinner.h>
#include <core/State.h>
#include <core/Trajectory.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <airsim_ros_pkgs/GimbalAngleEulerCmd.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <cstdint>
#include <geographic_msgs/GeoPoint.h>

// MAVROS MSGS
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

#define GEOMETRIC_CONTROL 1
#define PID_CONTROL 2
// MAVROS MASK FLAGS
uint8_t IGNORE_RATE_THRUST = 1 | 2 | 4 | 64;
uint16_t NOT_IGNORE_Z = 1 | 2 | 8 | 16 | 32 | 64 | 128 | 256 | 512 | 1024 | 2048;

ros::Time current_time;
ros::Time previous_time;
ros::Timer cmdloop_timer_;
float It_x = 0;
float It_y = 0;
float It_z = 0;

double t = 0;
int ctrl_mode = PID_CONTROL;


typedef struct trajectory_st{
    Eigen::Vector3f pos_d;
    Eigen::Vector3f dpos_d;
    Eigen::Vector3f ddpos_d;
    double yaw;
}trajectory;

 nav_msgs::Odometry drone_state;
trajectory traj;
std::vector<trajectory> traj_array;
Eigen::Vector4f forces_to_pwm(const Eigen::Vector4f &f_u);
Eigen::Vector3f trajectory_tracking(const nav_msgs::Odometry &state, const trajectory& traj);

void pid_control();
void geometric_control();

ros::Subscriber quadrotor_pose_sub;
ros::Subscriber trajectory_sub;
ros::Publisher pwm_pub, attitude_cmd_pub,controller_cmd_pub, mavros_attitude_control_pub, mavros_altitude_control_pub; 
ros::Publisher setpoint_pub, reference_yaw_pub, quadrotor_pose_pub, quadrotor_velocity_pub;

bool start_tracking = true;

core::State one_traj;
core::Trajectory desired_trajectory;

void trajectory_callback(core::Trajectory trajectory_cb)
{
    desired_trajectory = trajectory_cb;
    if(start_tracking)
    {
        t = trajectory_cb.start_time;
        start_tracking = false;
    }
        
    for(int i = 0; i < trajectory_cb.state.size(); i++)
    {
        traj.pos_d[0] = trajectory_cb.state[i].position.x;
        traj.pos_d[1] = trajectory_cb.state[i].position.y;
        traj.pos_d[2] = trajectory_cb.state[i].position.z;

        traj.dpos_d[0] = trajectory_cb.state[i].velocity.x;
        traj.dpos_d[1] = trajectory_cb.state[i].velocity.y;
        traj.dpos_d[2] = trajectory_cb.state[i].velocity.z;

        traj.ddpos_d[0] = trajectory_cb.state[i].acceleration.x;
        traj.ddpos_d[1] = trajectory_cb.state[i].acceleration.y;
        traj.ddpos_d[2] = trajectory_cb.state[i].acceleration.z;

        traj.yaw = trajectory_cb.state[i].yaw;

        traj_array.push_back(traj);
    }

}

// void quadrotor_pose_callback(nav_msgs::Odometry state) {


//     current_time = ros::Time::now(); 
//     float time_passed = (current_time.toSec()*1000) - (previous_time.toSec()*1000);

//     geometry_msgs::PoseStamped quadrotor_pose;
//     geometry_msgs::TwistStamped quadrotor_velocity;

//     // Pose
//     quadrotor_pose.header.stamp = current_time;
//     quadrotor_pose.header.frame_id = "world_ned";
//     quadrotor_pose.pose = state.pose.pose;

//     // Velocity
//     quadrotor_velocity.header.stamp = current_time;
//     quadrotor_velocity.header.frame_id = "world_ned";
//     quadrotor_velocity.twist.linear = state.twist.twist.linear;

//     quadrotor_pose_pub.publish(quadrotor_pose);
//     quadrotor_velocity_pub.publish(quadrotor_velocity);


//     // if dt passed
//     if( ( time_passed > 1000*desired_trajectory.dt ) & (t < desired_trajectory.end_time) &  )
//     {
      
//         int idx = t*10;
//         traj = traj_array[idx];
       
//         t +=  desired_trajectory.dt;
        
//         Eigen::Vector3f attitude = trajectory_tracking(state, traj);
//         double roll = attitude(0);
//         double pitch = attitude(1);
//         double yaw = attitude(2);

//         tf2::Quaternion quat_tf;  
//         quat_tf.setRPY( roll, pitch, yaw );
//         quat_tf.normalize();

//         geometry_msgs::Quaternion quat_msg;
//         quat_msg = tf2::toMsg(quat_tf);

        
//         geometry_msgs::Pose cmd;
//         cmd.orientation = quat_msg;
//         cmd.position.x = traj.pos_d[0];
//         cmd.position.y = traj.pos_d[1];
//         cmd.position.z = traj.pos_d[2];


//         controller_cmd_pub.publish(cmd);

//         //// Attitude control
//         // mavros_msgs::AttitudeTarget attitude_cmd;
//         // attitude_cmd.header.stamp = ros::Time::now(); 
//         // attitude_cmd.header.frame_id = "world_ned";
//         // attitude_cmd.orientation = quat_msg;
//         // attitude_cmd.type_mask = IGNORE_RATE_THRUST;


//         //// Altitude control
//         // mavros_msgs::PositionTarget altitude_cmd;
//         // altitude_cmd.header.stamp = ros::Time::now(); 
//         // altitude_cmd.header.frame_id = "world_ned";
//         // altitude_cmd.coordinate_frame = 1; // FRAME_LOCAL_NED
        
//         // geometry_msgs::Point point;
//         // point.z = traj.pos_d[2];
//         // altitude_cmd.position = point;

//         // altitude_cmd.type_mask = NOT_IGNORE_Z;

//         //// Sending the commands
//         // mavros_attitude_control_pub.publish(attitude_cmd);
//         // mavros_altitude_control_pub.publish(altitude_cmd);

//         previous_time = current_time; 

//     }
    
// }


void control_loop(const ros::TimerEvent &event)
{
    current_time = ros::Time::now(); 
    double time_passed = current_time.toSec() - previous_time.toSec();
    previous_time = current_time; 

    if(  (t < desired_trajectory.end_time)  & !start_tracking )
    {
      
      if(time_passed >= desired_trajectory.dt) 
      {
        int idx = t*(1/desired_trajectory.dt);
        traj = traj_array[idx];
        t +=  desired_trajectory.dt;
      }
      
      if(ctrl_mode == GEOMETRIC_CONTROL)
      {
          geometric_control();
      }
      if(ctrl_mode == PID_CONTROL) 
      {
          pid_control();
      }

    }

}

void quadrotor_pose_callback(nav_msgs::Odometry state) {

    drone_state = state;

    current_time = ros::Time::now(); 
    float time_passed = (current_time.toSec()*1000) - (previous_time.toSec()*1000);

    geometry_msgs::PoseStamped quadrotor_pose;
    geometry_msgs::TwistStamped quadrotor_velocity;

    // Pose
    quadrotor_pose.header.stamp = current_time;
    quadrotor_pose.header.frame_id = "world_ned";
    quadrotor_pose.pose = state.pose.pose;

    // Velocity
    quadrotor_velocity.header.stamp = current_time;
    quadrotor_velocity.header.frame_id = "world_ned";
    quadrotor_velocity.twist.linear = state.twist.twist.linear;

    quadrotor_pose_pub.publish(quadrotor_pose);
    quadrotor_velocity_pub.publish(quadrotor_velocity);
    
}

int main(int argc, char **argv)
{
    ROS_INFO("controller started\n");
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    ros::Time current_time = ros::Time::now();
    ros::Time previous_time = current_time;
    cmdloop_timer_ = nh.createTimer(ros::Duration(1.0/100), &control_loop);  // Define timer for constant loop rate

    quadrotor_pose_sub = nh.subscribe("/airsim_node/drone_1/odom_local_ned",100, quadrotor_pose_callback);
    trajectory_sub = nh.subscribe("/trajectory",100,trajectory_callback);
    pwm_pub = nh.advertise<geometry_msgs::Wrench>("/pwm",100);
    attitude_cmd_pub = nh.advertise<airsim_ros_pkgs::GimbalAngleEulerCmd>("/airsim_node/gimbal_angle_euler_cmd",100);
    controller_cmd_pub = nh.advertise<geometry_msgs::Pose>("/control_cmd",100);
    mavros_attitude_control_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",100);
    mavros_altitude_control_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",100);
    quadrotor_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100);
    quadrotor_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/local_position/velocity",100);
    setpoint_pub = nh.advertise<geometry_msgs::TwistStamped>("reference/setpoint",100);;
    reference_yaw_pub = nh.advertise<std_msgs::Float64>("reference/yaw",100);

    ros::spin();
    
    return 0;
}

double normalize_angle(double angle) {
  while(angle > M_PI)
        angle -= 2.0 * M_PI;
  while(angle < -M_PI)
        angle += 2.0 * M_PI;

    return angle;
}

Eigen::Vector3f trajectory_tracking(const nav_msgs::Odometry& state, const trajectory& traj)
{
    Eigen::Vector3f pos;
    pos[0] = state.pose.pose.position.x;
    pos[1] = state.pose.pose.position.y;
    pos[2] = state.pose.pose.position.z;
    
    float qx = state.pose.pose.orientation.x;
    float qy = state.pose.pose.orientation.y;
    float qz = state.pose.pose.orientation.z;
    float qw = state.pose.pose.orientation.w;


    Eigen::Vector3f E;
    E[0] = std::atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy));
    E[1] = std::asin(2*(qw*qy-qz*qx));
    E[2] = std::atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz));
    Eigen::Vector3f v;
    v[0] = state.twist.twist.linear.x;
    v[1] = state.twist.twist.linear.y;
    v[2] = state.twist.twist.linear.z;

    Eigen::Vector3f w;
    w[0] =  state.twist.twist.angular.x;
    w[1] =  state.twist.twist.angular.y;
    w[2] =  state.twist.twist.angular.z;

    float m = 1, g = 9.8;
    float k_p = 1;
    float k_pz = 2;
    float k_v = 3;
    float k_vz = 3;
    float k_E = 2;
    float k_Ez = 0.5;
    float k_w = 0.2;
    float k_wz = 0.1;
    Eigen::Vector3f e3(0,0,1);
    Eigen::Vector4f f_u(0,0,0,0);

    //Position Control
    It_x += ( traj.pos_d[0]-pos[0] )*desired_trajectory.dt;
    It_y += ( traj.pos_d[1]-pos[1] )*desired_trajectory.dt;
    It_z += ( traj.pos_d[2]-pos[2] )*desired_trajectory.dt;

    float v_xc = k_p*(traj.pos_d[0]-pos[0])+traj.dpos_d[0];
    float v_yc = k_p*(traj.pos_d[1]-pos[1])+traj.dpos_d[1];
    float v_zc = k_pz*(traj.pos_d[2]-pos[2])+traj.dpos_d[2];

    float a_x = k_v*(v_xc-v[0])+traj.ddpos_d[0];
    float a_y = k_v*(v_yc-v[1])+traj.ddpos_d[1];
    float a_z = k_vz*(v_zc-v[2])+traj.ddpos_d[2];

    //Attitude Control (inertial to body frame)
    float phi_d = (-sin(E[2])*a_x+cos(E[2])*a_y)/g;
    float theta_d = (cos(E[2])*a_x+sin(E[2])*a_y)/g;
    float psi_d = traj.yaw; //atan2(traj.pos_d[1],traj.pos_d[0]); //normalize_angle(traj.yaw); // ;

    float max_tilt = (40.0/180.0)*M_PI;
    //float max_yaw = 179.0/180.0*M_PI;
    if(abs(phi_d) > max_tilt)
        phi_d = max_tilt*phi_d/abs(phi_d);

    if(abs(theta_d) > max_tilt)
        theta_d = max_tilt*theta_d/abs(theta_d);
    
    //if(abs(psi_d) > max_yaw)
    //    psi_d = max_yaw*psi_d/abs(psi_d);

    Eigen::Vector3f attitude;
    attitude(0) = phi_d;  attitude(1) = theta_d; attitude(2) = psi_d;

    return attitude;

    // float w_xc = k_E*(phi_d-E[0]);
    // float w_yc = k_E*(theta_d-E[1]);
    // float w_zc = k_Ez*(psi_d-E[2]);

    // float tau_xc = k_w*(w_xc-w[0]);
    // float tau_yc = k_w*(w_yc-w[1]);
    // float tau_zc = k_wz*(w_zc-w[2]);

    // f_u[0] = -m*(a_z-g);
    // f_u[1] = tau_xc;
    // f_u[2] = tau_yc;
    // f_u[3] = tau_zc;
    // return f_u;
}


Eigen::Vector4f forces_to_pwm(const Eigen::Vector4f &f_u)
{
    float propeller_diameter = 0.2286, standard_air_density = 1.225, d = 0.2275; //d is arm_length
    float c_T = 0.109919*pow(propeller_diameter, 4)*standard_air_density;
    float c_Q = 0.040164*pow(propeller_diameter, 5)*standard_air_density/(2*M_PI);

    Eigen::Matrix<float,4,4> M;
    M <<    c_T,    c_T,   c_T,    c_T,
         -d*c_T,  d*c_T, d*c_T, -d*c_T,
          d*c_T, -d*c_T, d*c_T, -d*c_T,
            c_Q,    c_Q,  -c_Q,   -c_Q;

    Eigen::Vector4f thrust =  c_T*M.inverse()*f_u;

    float max_thrust = 4.179446268;
    float air_density_ratio = 1.225/standard_air_density;
    Eigen::Vector4f pwm;
    float max_pwm = -1000;

    for(int i = 0;i<thrust.size();++i){
        pwm[i] = std::max<float>(0,thrust[i]/(air_density_ratio*max_thrust));
        max_pwm = std::max<float>(max_pwm,pwm[i]);
    }
    if(max_pwm > 1)
        for(int i = 0;i<pwm.size();++i)
            pwm[i] /= max_pwm;
    
    std::cout << "PWM: " << pwm << "\n";
    return pwm;
}

void pid_control()
{
    Eigen::Vector3f attitude = trajectory_tracking(drone_state, traj);
    double roll = attitude(0);
    double pitch = attitude(1);
    double yaw = attitude(2);

    tf2::Quaternion quat_tf;  
    quat_tf.setRPY( roll, pitch, yaw );
    quat_tf.normalize();

    geometry_msgs::Quaternion quat_msg;
    quat_msg = tf2::toMsg(quat_tf);

    geometry_msgs::Pose cmd;
    cmd.orientation = quat_msg;
    cmd.position.x = traj.pos_d[0];
    cmd.position.y = traj.pos_d[1];
    cmd.position.z = traj.pos_d[2];

    controller_cmd_pub.publish(cmd);


}

void geometric_control()
{
    geometry_msgs::TwistStamped setpoint;
    setpoint.twist.angular.x = traj.pos_d[0];
    setpoint.twist.angular.y = traj.pos_d[1];
    setpoint.twist.angular.z = traj.pos_d[2];
    setpoint.twist.linear.x = traj.dpos_d[0];
    setpoint.twist.linear.y = traj.dpos_d[1];
    setpoint.twist.linear.z = traj.dpos_d[2];

    std_msgs::Float64 yaw_ref;
    yaw_ref.data = traj.yaw;
    setpoint_pub.publish(setpoint);
    reference_yaw_pub.publish(yaw_ref);

}