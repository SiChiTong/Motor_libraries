#include "ros/ros.h"
#include <nav_converter/speed_wheel.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"

long _PreviousLeftEncoderCounts = 0;
long _PreviousRightEncoderCounts = 0;
ros::Time current_time_encoder, last_time_encoder;

double x;
double y;
double th;

double vx;
double vy;
double vth;
double speedLeft;
double speedRight;

std::string driver_left, driver_right;
float L, R, K;
bool isUseEncoder = true;

ros::Time current_time, last_time;
ros::Publisher odom_pub;

bool loadParam(std::string node_name);
double convertRpmToMetPerSecond(int32_t encoder);
double getYaw(double x, double y, double z, double w);
void pubOdometry();

void agvCurrentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  if(isUseEncoder == true){
    x = static_cast<double>(msg.pose.position.x);
    y = static_cast<double>(msg.pose.position.y);
    th = getYaw(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    ROS_ERROR("odometry.cpp-x: %lf", x);
    ROS_ERROR("odometry.cpp-y: %lf", y);
    ROS_ERROR("odometry.cpp-th: %lf", th);
  }
}
void currentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  if(isUseEncoder == false){
    x = static_cast<double>(msg.pose.position.x);
    y = static_cast<double>(msg.pose.position.y);
    th = getYaw(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
  }
}

void EncoderCallBack(const nav_converter::speed_wheel& msg)
{
  speedLeft =  convertRpmToMetPerSecond(msg.wheel_letf);
  speedRight = convertRpmToMetPerSecond(msg.wheel_right);
}

// void wheelLeftCallback(const nav_converter::speed_wheel& msg)
// {
// 	// ROS_INFO("odometry.cpp-wheelLeftCallback() -> left_encoder: %d", msg.encoder);
//   speedLeft = convertRpmToMetPerSecond(msg.encoder);
//   // ROS_WARN("odometry.cpp-wheelLeftCallback() -> speedLeft: %lf", speedLeft);
// }
// void wheelRightCallback(const nav_converter::speed_wheel& msg)
// {
// 	// ROS_INFO("odometry.cpp-wheelRightCallback() -> right_encoder: %d", msg.encoder);
//   speedRight = convertRpmToMetPerSecond(msg.encoder);
//   // ROS_WARN("odometry.cpp-wheelRightCallback() -> speedRight: %lf", speedRight);
// }

double convertRpmToMetPerSecond(int32_t rpm){
  double ms = (2*M_PI*R*rpm)/(K*60); 
  return ms;
}
double getYaw(double x, double y, double z, double w) {
  // ROS_INFO("odometry.cpp-60-GetYaw");
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  return yaw;
}

void pubOdometry(){
  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  // ROS_WARN("odometry.cpp-dt: %lf", dt);
  last_time = current_time;

  tf::TransformBroadcaster odom_broadcaster;

  //compute odometry in a typical way given the velocities of the robot
  vx  = -(speedLeft - speedRight)/2;
  vth = (speedLeft + speedRight)/L;
  // ROS_ERROR("odometry.cpp-vx: %lf", vx);
  // ROS_ERROR("odometry.cpp-vth: %lf", vth);

  double delta_x = (vx*cos(th) - vy*sin(th))*dt;
  double delta_y = (vx*sin(th) + vy*cos(th))*dt;
  double delta_th = vth*dt;


  if(isUseEncoder == true){
    x  += delta_x;
    y  += delta_y;
    th += delta_th;
  }

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_footprint";
  // odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_footprint";
  // odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z= vth;

  //publish the message
  odom_pub.publish(odom);
}

bool loadParam(std::string node_name){
	ROS_INFO("odometry.cpp-loadParam() -> node_name: %s", node_name.c_str());
	
	if(!ros::param::get("/L", L)){
		return false;
	}ROS_INFO("odometry.cpp- L: %f (Khoang cach 2 banh - m)", L);

	if(!ros::param::get("/R", R)){
		return false;
	}ROS_INFO("odometry.cpp- R: %f (Ban kinh banh xe - m)", R);

	if(!ros::param::get("/K", K)){
		return false;
	}ROS_INFO("odometry.cpp- K: %f (ti so truyen dong co)", K);
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry");
  ros::NodeHandle n;

	std::string node_name = ros::this_node::getName();
	ROS_INFO("odometry.cpp-node_name: %s", node_name.c_str());
	if(loadParam(node_name)){
		ROS_INFO("odometry.cpp-Load parameter successfull");
	}else{
		ROS_ERROR("odometry.cpp-Error when load parameter");
	}
  ros::Subscriber encoder_sub  = n.subscribe("control_wheel", 10, EncoderCallBack);
  
  ros::Subscriber agv_current_pose_sub = n.subscribe("/agv_current_pose", 10, agvCurrentPoseCallback);
  ROS_INFO("odometry.cpp- Subscriber topic /agv_current_pose");
  ros::Subscriber current_pose_sub = n.subscribe("/current_pose", 10, currentPoseCallback);
  ROS_INFO("odometry.cpp- Subscriber topic /current_pose");

  odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50); 

  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10);
  while(ros::ok())
  {
    pubOdometry();
    ros::spinOnce();
    r.sleep();
  }
  ros::spin();
}