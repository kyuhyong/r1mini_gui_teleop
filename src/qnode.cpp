/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include "../include/r1mini_gui_teleop/qnode.hpp"
#include <sensor_msgs/image_encodings.h>//added head file
#include <geometry_msgs/Pose.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace r1mini_gui_teleop {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"r1mini_gui_teleop");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"r1mini_gui_teleop");
  this->img_file_num = 0;
  this->img_file_name = "Unknown";
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
  vel_v_max = MAX_LIN_VEL;
  vel_w_max = MAX_ANG_VEL;
  vel_v_stepSize = LIN_VEL_STEP_SIZE;
  vel_w_stepSize = ANG_VEL_STEP_SIZE;
  vel_v_m_s = 0.0;
  vel_w_rad_s = 0.0;
  pub_twist = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  // Subscriptions
  image_transport::ImageTransport it(n);
  image_transport::TransportHints hints("compressed");
  image_sub = it.subscribe("/main_camera/image_raw",100, &QNode::myCallback_img,this,hints);
  //image_sub = it.subscribe("/main_camera/image_raw",100,&QNode::myCallback_img,this);           //This is slow
  //image_sub = it.subscribe("/main_camera/image_raw/compressed",100,&QNode::myCallback_img,this);  //This will not work
  sub_odom = n.subscribe("odom",1000,&QNode::myCallback_odom, this);
  sub_pose = n.subscribe("pose",1000,&QNode::myCallback_pose, this);
  // Service call
  clientSetColor = n.serviceClient<r1mini_gui_teleop::Color>("/set_led_color");
  clientSetHeadlight = n.serviceClient<r1mini_gui_teleop::Onoff>("/set_headlight");
  clientCalg = n.serviceClient<r1mini_gui_teleop::Calg>("/calibrate_gyro");
  clientResetOdom = n.serviceClient<r1mini_gui_teleop::ResetOdom>("/reset_odom");
  clientBattery = n.serviceClient<r1mini_gui_teleop::Battery>("/battery_status");
  this->battery_checkPeriodic = false;
  this->battery_checkCnt = 0;
	start();
	return true;
}

void QNode::run() {
  ros::Rate loop_rate(20);
	while ( ros::ok() ) {
    pub_twist_vw(vel_v_m_s, vel_w_rad_s);
    if(battery_checkPeriodic) {
      if(battery_checkCnt++>19) {
        battery_checkCnt = 0;
        if(clientBattery.call(serviceBattery)) {
          myBatteryStatus.Voltage = serviceBattery.response.volt;
          myBatteryStatus.SOC = serviceBattery.response.SOC;
          myBatteryStatus.Current = serviceBattery.response.current;
          Q_EMIT newDataReceived(msgType_battery);
        }
      }
    }
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::myCallback_img(const sensor_msgs::ImageConstPtr &msg){
  try
  {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    img = cv_ptr->image;
    qimage = QImage(img.data, img.cols, img.rows, img.step[0], QImage::Format_RGB888);//change  to QImage format
    //ROS_INFO("I'm setting picture in mul_t callback function!");
    Q_EMIT loggingCamera();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void QNode::myCallback_odom(const nav_msgs::Odometry::ConstPtr &msg){
  //ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  odom_x = msg->pose.pose.position.x;
  odom_y = msg->pose.pose.position.y;
  odom_theta = msg->pose.pose.orientation.z * 180*M_1_PI;
  Q_EMIT newDataReceived(msgType_odom);

}
void QNode::myCallback_pose(const geometry_msgs::Pose::ConstPtr &msg){
  pose_x = msg->orientation.x;
  pose_y = msg->orientation.y;
  pose_z = msg->orientation.z;
  Q_EMIT newDataReceived(msgType_pose);
}

int QNode::save_current_image() {
  cv::Mat dst;
  int height = this->img.rows;
  cv::resize(this->img, dst, cv::Size(height, height));
  cv::cvtColor(dst, dst, CV_RGB2BGR);
  cv::imwrite(this->img_file_name + "_"+ to_string(img_file_num) + ".jpg", dst);
  return img_file_num++;
}
void QNode::set_image_title(string &title){
  this->img_file_name = title;
}
void QNode::set_image_count(int cnt) {
  this->img_file_num = cnt;
}
void QNode::service_call_setColor(int64 red, int64 green, int64 blue) {
  serviceSetColor.request.red = red;
  serviceSetColor.request.green = green;
  serviceSetColor.request.blue = blue;
  clientSetColor.call(serviceSetColor);
}
void QNode::service_call_headlight(bool set){
  serviceSetHeadlight.request.set = set;
  clientSetHeadlight.call(serviceSetHeadlight);
}

void QNode::service_call_Calg() {
  std::cout<<"Service Call Calg"<<std::endl;
  clientCalg.call(serviceCalg);
}
void QNode::service_call_Battery() {
  std::cout<<"Service call Battery"<<std::endl;
  if(clientBattery.call(serviceBattery)) {
    myBatteryStatus.Voltage = serviceBattery.response.volt;
    myBatteryStatus.SOC = serviceBattery.response.SOC;
    myBatteryStatus.Current = serviceBattery.response.current;
    //std::cout<<"Battery V:"<<serviceBattery.response.volt<<
    //           "SOC:"<<serviceBattery.response.SOC<<
    //           "Current:"<<serviceBattery.response.current<<std::endl;
    Q_EMIT newDataReceived(msgType_battery);
  }
}
void QNode::service_call_resetOdom() {
  serviceResetOdom.request.x = 0.0;
  serviceResetOdom.request.y = 0.0;
  serviceResetOdom.request.theta = 0.0;
  clientResetOdom.call(serviceResetOdom);
}

void QNode::pub_twist_vw(double v, double w) {
  geometry_msgs::Twist twist;
  twist.linear.x = v;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = w;
  pub_twist.publish(twist);
}

void QNode::vel_up(){
  vel_v_m_s = constrain(vel_v_m_s+vel_v_stepSize, vel_v_max, -vel_v_max);
}

void QNode::vel_down(){
  vel_v_m_s = constrain(vel_v_m_s-vel_v_stepSize, vel_v_max, -vel_v_max);
}

void QNode::ang_left(){
  vel_w_rad_s = constrain(vel_w_rad_s+vel_w_stepSize, vel_w_max, -vel_w_max);
}
void QNode::ang_right(){
  vel_w_rad_s = constrain(vel_w_rad_s-vel_w_stepSize, vel_w_max, -vel_w_max);
}
void QNode::stop(){
  vel_v_m_s = 0.0;
  vel_w_rad_s = 0.0;
}
void QNode::ang_zero(){
  vel_w_rad_s = 0.0;
}
double QNode::get_odo_x(){
  return odom_x;
}
double QNode::get_odo_y(){
  return odom_y;
}
double QNode::get_odo_theta(){
  return odom_theta;
}
double QNode::get_Roll(){
  return pose_x;
}
double QNode::get_Pitch(){
  return pose_y;
}
double QNode::get_Yaw(){
  return pose_z;
}
batteryStatusType QNode::get_BatteryStatus(){
  return myBatteryStatus;
}
void  QNode::set_BatteryCheckPeriodic(bool set) {
  this->battery_checkCnt = 0;
  this->battery_checkPeriodic = set;
}
double QNode::constrain(double vel, double max, double min){
  if(vel > max) return max;
  else if(vel < min) return min;
  else return vel;
}

void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace r1mini_gui_teleop
