/**
 * @file /include/r1mini_gui_teleop/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef r1mini_gui_teleop_QNODE_HPP_
#define r1mini_gui_teleop_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <QImage>

#include "r1mini_gui_teleop/Color.h"    //For set_led_color service
#include "r1mini_gui_teleop/Onoff.h"    //For set_led_color service

#define MAX_LIN_VEL 1.20
#define MAX_ANG_VEL 1.80
#define LIN_VEL_STEP_SIZE 0.05
#define ANG_VEL_STEP_SIZE 0.1

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace r1mini_gui_teleop {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
  //Added to image_view
  void myCallback_img(const sensor_msgs::ImageConstPtr& msg);//camera callback function
  QImage image;
	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
  void vel_up(void);
  void vel_down(void);
  void ang_left(void);
  void ang_right(void);
  void stop(void);
  void ang_zero(void);
  void setColor(int64 red, int64 green, int64 blue);
  void setHeadlight(bool set);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void loggingCamera();//Send a signal to set the camera picture

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;
  /*********************************
   * Publish cmd_vel
   *******************************/
  ros::Publisher pub_twist;
  double vel_v_m_s;
  double vel_w_rad_s;
  double vel_v_max;
  double vel_w_max;
  double vel_v_stepSize;
  double vel_w_stepSize;
  void pub_twist_vw(double v, double w);
  double constrain(double vel, double max, double min);
  /*********************************
   * Subscribe main_camera
   *******************************/
  image_transport::Subscriber image_sub;
  cv::Mat img;
  /********************************
   * Services
   *******************************/
  ros::ServiceClient clientSetColor;   //Client node to call service
  r1mini_gui_teleop::Color serviceSetColor;  //Service set color
  ros::ServiceClient clientSetHeadlight;   //Client node to call service
  r1mini_gui_teleop::Onoff serviceSetHeadlight; //Service set headlight
};

}  // namespace r1mini_gui_teleop

#endif /* r1mini_gui_teleop_QNODE_HPP_ */
