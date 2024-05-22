#include "main.hpp"
#include "MW_value.hpp"
#include "MW_serial.hpp"
#include "stella.hpp"
#include <math.h>

#define convertor_d2r (M_PI / 180.0)

static bool RUN = false;

std::array<double, 36> pose_covariance = {
    0.1, 0.0, 0.0, 0.0, 0.0, 0.0,  // Covariance in x
    0.0, 0.1, 0.0, 0.0, 0.0, 0.0,  // Covariance in y
    0.0, 0.0, 0.1, 0.0, 0.0, 0.0,  // Covariance in z
    0.0, 0.0, 0.0, 0.01, 0.0, 0.0, // Covariance in roll
    0.0, 0.0, 0.0, 0.0, 0.01, 0.0, // Covariance in pitch
    0.0, 0.0, 0.0, 0.0, 0.0, 0.01  // Covariance in yaw
};

std::array<double, 36> twist_covariance = {
    0.05, 0.0, 0.0, 0.0, 0.0, 0.0,  // Covariance in vx
    0.0, 0.05, 0.0, 0.0, 0.0, 0.0,  // Covariance in vy
    0.0, 0.0, 0.05, 0.0, 0.0, 0.0,  // Covariance in vz
    0.0, 0.0, 0.0, 0.02, 0.0, 0.0, // Covariance in wx
    0.0, 0.0, 0.0, 0.0, 0.02, 0.0, // Covariance in wy
    0.0, 0.0, 0.0, 0.0, 0.0, 0.02  // Covariance in wz
};

inline int Limit_i (int v, int lo, int hi)
{

 
	if(abs(v) > lo && abs(v) < hi) return v;
	
}

inline double pulse2meter()
{
  double meter= ((2 * M_PI * Differential_MobileRobot.wheel_radius) / Differential_MobileRobot.gear_ratio / MyMotorConfiguration.encoder_ppr[0]);

  return meter *-1 ;
}

inline double rpm2mps()
{
  double mps = M_PI / 30 * Differential_MobileRobot.wheel_radius / Differential_MobileRobot.gear_ratio;

  return mps * -1;
}

stellaN1_node::stellaN1_node() 
{

//  ahrs_sub = n.subscribe("imu/yaw", 1, &stellaN1_node::ahrs_yaw_data_callback, this);
  sub = n.subscribe("cmd_vel", 10, &stellaN1_node::command_velocity_callback, this);

  chatter_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  timer = n.createTimer(ros::Duration(0.05),&stellaN1_node::serial_callback,this);

  current_time = ros::Time::now();
  last_time = ros::Time::now();
}

stellaN1_node::~stellaN1_node()
{
  MW_Serial_DisConnect();
}


//void stellaN1_node::ahrs_yaw_data_callback(const std_msgs::Float64::ConstPtr &ahrs_msg)
//{
//  ahrs_yaw = ahrs_msg->data;
//  if (ahrs_yaw_init == false)
//  {
//    ahrs_yaw_first = ahrs_yaw;
//    ahrs_yaw_init = true;
//  }
//}

void stellaN1_node::command_velocity_callback(const geometry_msgs::Twist::ConstPtr &cmd_msg)
{
  if(RUN)
  { 
    
    goal_linear_velocity_ = -1 * cmd_msg->linear.x ;
    goal_angular_velocity_ = cmd_msg->angular.z ;

    dual_m_command(dual_m_command_select::m_lav, goal_linear_velocity_, goal_angular_velocity_);
  
  }
}


void stellaN1_node::serial_callback(const ros::TimerEvent& event)
{
  if(RUN)
  {
    Motor_MonitoringCommand(channel_1, _velocity);
    Motor_MonitoringCommand(channel_2, _velocity);

    update_odometry();
  }
}

bool stellaN1_node::update_odometry()
{

  current_time = ros::Time::now();
  dt = (current_time - last_time).toSec();

  delta_left = MyMotorCommandReadValue.velocity[channel_1] * rpm2mps(); // m/s
  delta_right = MyMotorCommandReadValue.velocity[channel_2] * rpm2mps(); // m/s

  delta_s  = (delta_right + delta_left) / 2.0 ;
  delta_th = -1 * ((delta_right - delta_left) / Differential_MobileRobot.axle_length);
  delta_x = delta_s * cos(th);
  delta_y = delta_s * sin(th);

  x += delta_x * dt;
  y += delta_y * dt;
  th += delta_th * dt;

  // Clip the orientation from -pi to pi
  if (th > M_PI)
  {
    th -= 2 * M_PI;
  }
  else if (th < -M_PI)
  {
    th += 2 * M_PI;
  }

  nav_msgs::Odometry odom;

  tf2::Quaternion Quaternion;

  Quaternion.setRPY(0, 0, th);

  odom.pose.pose.orientation.x = Quaternion.x();
  odom.pose.pose.orientation.y = Quaternion.y();
  odom.pose.pose.orientation.z = Quaternion.z();
  odom.pose.pose.orientation.w = Quaternion.w();

  geometry_msgs::TransformStamped t;

  t.header.stamp = current_time;
  t.header.frame_id = "odom";
  t.child_frame_id = "base_footprint";

  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = 0.0;

  t.transform.rotation.x = Quaternion.x();
  t.transform.rotation.y = Quaternion.y();
  t.transform.rotation.z = Quaternion.z();
  t.transform.rotation.w = Quaternion.w();

  odom_broadcaster.sendTransform(t);

  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;

  odom.twist.twist.linear.x = delta_s;
  odom.twist.twist.angular.z = delta_th;

  for (int i = 0; i < 36; ++i) {
    odom.pose.covariance[i] = pose_covariance[i];
    odom.twist.covariance[i] = twist_covariance[i];
  }

  odom.header.stamp = current_time;
  chatter_pub.publish(odom);

  last_time = current_time;

  return true;
}


//  {
//    delta_left = Limit_i((MyMotorCommandReadValue.position[channel_1] - left_encoder_prev)*-1 , 0, 10000) * pulse2meter();
//    delta_right = Limit_i((MyMotorCommandReadValue.position[channel_2] - right_encoder_prev)*-1 , 0, 10000) * pulse2meter();
//
//    left_encoder_prev = MyMotorCommandReadValue.position[channel_1];
//    right_encoder_prev = MyMotorCommandReadValue.position[channel_2];
//
//    delta_s  = -1*(delta_right + delta_left) / 2.0 ;
//    delta_th = ((delta_right - delta_left) / Differential_MobileRobot.axle_length)* dt;
//    th += delta_th;
//
//    delta_x = delta_s * cos(th);
//    delta_y = delta_s * sin(th);
//
//    x += delta_x;
//    y += delta_y;
//  }
//  last_time = current_time;

//
//
//  //로봇 기구학 적용및 센서퓨전
////  delta_s  = (delta_right + delta_left) / 2.0 ;
//
//  delta_th = ((ahrs_yaw - ahrs_yaw_first) * convertor_d2r);
//
//  //th 값은 AHRS YAW 값을 참조하여 아래의 식은 주석으로 처리한다.
//
//
//  delta_y  = delta_s * sin(delta_th);
//
//  x += delta_x;
//  y += delta_y;
//
//  nav_msgs::Odometry odom;
//
//  tf2::Quaternion Quaternion;
//
//  Quaternion.setRPY(0, 0, delta_th);
//
//  odom.pose.pose.orientation.x = Quaternion.x();
//  odom.pose.pose.orientation.y = Quaternion.y();
//  odom.pose.pose.orientation.z = Quaternion.z();
//  odom.pose.pose.orientation.w = Quaternion.w();
//
//  geometry_msgs::TransformStamped t;
//
//  t.header.stamp = current_time;
//  t.header.frame_id = "odom";
//  t.child_frame_id = "base_footprint";
//
//  t.transform.translation.x = x;
//  t.transform.translation.y = y;
//  t.transform.translation.z = 0.0;
//
//  t.transform.rotation.x = Quaternion.x();
//  t.transform.rotation.y = Quaternion.y();
//  t.transform.rotation.z = Quaternion.z();
//  t.transform.rotation.w = Quaternion.w();
//
//  odom_broadcaster.sendTransform(t);
//
//  odom.header.frame_id = "odom";
//
//  odom.pose.pose.position.x = x;
//  odom.pose.pose.position.y = y;
//  odom.pose.pose.position.z = 0.0;
//
//  odom.child_frame_id = "base_footprint";
//  odom.twist.twist.linear.x = goal_linear_velocity_;
//  odom.twist.twist.angular.z = goal_angular_velocity_ *-1;
//
//  odom.header.stamp = current_time;
//  chatter_pub.publish(odom);
//
//  left_encoder_prev = MyMotorCommandReadValue.position[channel_1];
//  right_encoder_prev = MyMotorCommandReadValue.position[channel_2];

  //rate.sleep();

  // Get the velocity of the robot

  // Calculate the linear and angular velocity of the robot
//  double linear_velocity = (left_wheel_vel + right_wheel_vel) / 2.0;
//  double angular_velocity = -1 * (right_wheel_vel - left_wheel_vel) / Differential_MobileRobot.axle_length;
//
//  // Publish the odometry message over ROS
//  nav_msgs::Odometry odom;
//  odom.header.stamp = current_time;
//  odom.header.frame_id = "odom";
//  odom.child_frame_id = "base_footprint";
//
//  // Set the twist of the odometry message
//  odom.twist.twist.linear.x = linear_velocity;
//  odom.twist.twist.angular.z = angular_velocity;

  // Set the position of the odometry message
//  odom.pose.pose.position.x = x;
//  odom.pose.pose.position.y = y;
//  odom.pose.pose.position.z = 0.0;

  // Set the orientation of the odometry message
//  tf2::Quaternion q;
//  q.setRPY(0, 0, th);
//  odom.pose.pose.orientation.x = q.x();

  // Publish the message
//  chatter_pub.publish(odom);
//
//  return true;


int main(int argc, char *argv[])
//int main(int argc, char **argv)
{
  ros::init(argc, argv,"stella_md_node");

  char port[] = "/dev/MW";
  MW_Serial_Connect(port, 115200);

  if(Robot_Setting(Robot_choice::N1)) RUN = true;
  Robot_Fault_Checking_RESET();
  
  stellaN1_node node;

  ros::spin();

  return 0;
}