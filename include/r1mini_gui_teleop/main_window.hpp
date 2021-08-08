/**
 * @file /include/r1mini_gui_teleop/main_window.hpp
 *
 * @brief Qt based gui for r1mini_gui_teleop.
 *
 * @date November 2010
 **/
#ifndef r1mini_gui_teleop_MAIN_WINDOW_H
#define r1mini_gui_teleop_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QImage>//added
#include <QMutex>//added
#include <QColor>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace r1mini_gui_teleop {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:

	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void onNewData(QNode::MsgType);

private Q_SLOTS:
  void on_button_move_stop_clicked();
  void on_button_move_right_clicked();
  void on_button_move_left_clicked();
  void on_button_move_rev_clicked();
  void on_button_move_fwd_clicked();
  void updateLogcamera();//added
  void displayCamera(const QImage& image);//added

  void on_buttonTurnZero_clicked();

  void on_checkBoxHeadlightOnOff_clicked(bool checked);

  void on_buttonSetColor_clicked();

  void on_checkGetIMU_clicked(bool checked);

  void on_buttonCaptureImage_clicked();

  void on_textboxCaptureTitle_textChanged(const QString &arg1);

  void on_buttonCalibrateGyro_clicked();

  void on_buttonResetOdom_clicked();

  void on_buttonSetImageCount_clicked();

  void on_buttonCheckBattery_clicked();

  void on_checkBox_BatteryPeriodic_stateChanged(int arg1);

  void on_checkBox_BatteryPeriodic_clicked(bool checked);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
  QImage qimage_;//added
  mutable QMutex qimage_mutex_;//added
  QColor dispColor;     //Robot display color
};

}  // namespace r1mini_gui_teleop

#endif // r1mini_gui_teleop_MAIN_WINDOW_H
