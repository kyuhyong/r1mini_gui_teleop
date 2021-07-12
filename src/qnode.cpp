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
  pub_twist = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  image_transport::ImageTransport it(n);
  image_transport::TransportHints hints("compressed");
  image_sub = it.subscribe("/main_camera/image_raw",100, &QNode::myCallback_img,this,hints);
  //image_sub = it.subscribe("/main_camera/image_raw",100,&QNode::myCallback_img,this);           //This is slow
  //image_sub = it.subscribe("/main_camera/image_raw/compressed",100,&QNode::myCallback_img,this);  //This will not work
  clientSetColor = n.serviceClient<r1mini_gui_teleop::Color>("/set_led_color");
  clientSetHeadlight = n.serviceClient<r1mini_gui_teleop::Onoff>("/set_headlight");
	start();
	return true;
}

void QNode::run() {
  ros::Rate loop_rate(20);
	while ( ros::ok() ) {
    pub_twist_vw(vel_v_m_s, vel_w_rad_s);
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
    image = QImage(img.data, img.cols, img.rows, img.step[0], QImage::Format_RGB888);//change  to QImage format
    //ROS_INFO("I'm setting picture in mul_t callback function!");
    Q_EMIT loggingCamera();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
void QNode::setColor(int64 red, int64 green, int64 blue) {
  serviceSetColor.request.red = red;
  serviceSetColor.request.green = green;
  serviceSetColor.request.blue = blue;
  clientSetColor.call(serviceSetColor);
}
void QNode::setHeadlight(bool set){
  serviceSetHeadlight.request.set = set;
  clientSetHeadlight.call(serviceSetHeadlight);
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
