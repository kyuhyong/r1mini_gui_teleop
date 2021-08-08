/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/r1mini_gui_teleop/main_window.hpp"
#include <QColorDialog>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace r1mini_gui_teleop {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  ui.dock_status->show();     //Display dock_status as default
	/*********************
	** Logging
	**********************/
    qRegisterMetaType<QNode::MsgType>("QNode::MsgType");
    //ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    QObject::connect(&qnode, SIGNAL(loggingCamera()),this,SLOT(updateLogcamera()));  //Added
    QObject::connect(&qnode, SIGNAL(newDataReceived(QNode::MsgType)), this, SLOT(onNewData(QNode::MsgType)));
    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        //ui.view_logging->scrollToBottom();
}

void MainWindow::displayCamera(const QImage &image)
{
  //std::cout << "displayCamera." << std::endl;
  qimage_mutex_.lock();
  //std::cout << "copy." << std::endl;
  //qimage_ = image.copy();
  ui.logging_cam->setPixmap(QPixmap::fromImage(image));
  //ui.label_camera->setPixmap(QPixmap::fromImage(qimage_));
  ui.logging_cam->resize(ui.logging_cam->pixmap()->size());
  qimage_mutex_.unlock();
}
void MainWindow::updateLogcamera()
{
  //std::cout << "updateLogcamera." << std::endl;
  displayCamera(qnode.qimage);
}
void MainWindow::onNewData(QNode::MsgType type)
{
  //std::cout << "New Msg" << msgType<< std::endl;
  //ui.textbox_odo_x->setText(QString::number(qnode.get_odo_x()));
  switch(type){
    case QNode::MsgType::msgType_odom:
      ui.textbox_odo_x->setText(QString::number(qnode.get_odo_x()));
      ui.textbox_odo_y->setText(QString::number(qnode.get_odo_y()));
      ui.textbox_odo_theta->setText(QString::number(qnode.get_odo_theta()));
      break;
    case QNode::MsgType::msgType_pose:
      ui.textbox_pose_x->setText(QString::number(qnode.get_Roll()));
      ui.textbox_pose_y->setText(QString::number(qnode.get_Pitch()));
      ui.textbox_pose_z->setText(QString::number(qnode.get_Yaw()));
      break;
    case QNode::MsgType::msgType_battery:
      batteryStatusType bat = qnode.get_BatteryStatus();
      ui.textBox_Voltage->setText((QString::number(bat.Voltage)));
      ui.textBox_SOC->setText((QString::number(bat.SOC)));
      ui.textBox_Current->setText((QString::number(bat.Current)));
      break;
  }
}
/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME R1mini GUI Demo Program 0.10</h2><p>2021(c) OMOROBOT Inc</p><p><a href=""http://www.omorobot.com"">Visit omorobot.com</a>.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "r1mini_gui_teleop");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "r1mini_gui_teleop");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace r1mini_gui_teleop


void r1mini_gui_teleop::MainWindow::on_button_move_stop_clicked()
{
  qnode.stop();
}

void r1mini_gui_teleop::MainWindow::on_button_move_fwd_clicked()
{
  qnode.vel_up();
}

void r1mini_gui_teleop::MainWindow::on_button_move_rev_clicked()
{
  qnode.vel_down();
}

void r1mini_gui_teleop::MainWindow::on_button_move_left_clicked()
{
  qnode.ang_left();
}

void r1mini_gui_teleop::MainWindow::on_button_move_right_clicked()
{
  qnode.ang_right();
}

void r1mini_gui_teleop::MainWindow::on_buttonTurnZero_clicked()
{
  qnode.ang_zero();
}

void r1mini_gui_teleop::MainWindow::on_checkBoxHeadlightOnOff_clicked(bool checked)
{
  qnode.service_call_headlight(checked);
}

void r1mini_gui_teleop::MainWindow::on_buttonSetColor_clicked()
{
  QColor newColor = QColorDialog::getColor(dispColor,parentWidget());
  if(newColor != dispColor) {
    dispColor = newColor;
    std::cout << "Color Set: R=" << dispColor.red() << " G="<<dispColor.green()<<" B="<<dispColor.blue() << std::endl;
    qnode.service_call_setColor(dispColor.red(), dispColor.green(), dispColor.blue());
  }
}

void r1mini_gui_teleop::MainWindow::on_checkGetIMU_clicked(bool checked)
{

}


void r1mini_gui_teleop::MainWindow::on_buttonCaptureImage_clicked()
{
    int imgCnt = qnode.save_current_image();
    ui.spinBoxImageCountNum->setValue(imgCnt);
}

void r1mini_gui_teleop::MainWindow::on_textboxCaptureTitle_textChanged(const QString &arg1)
{
  std::string utf8_text = arg1.toUtf8().constData();
  qnode.set_image_title(utf8_text);
}


void r1mini_gui_teleop::MainWindow::on_buttonCalibrateGyro_clicked()
{
    qnode.service_call_Calg();
}

void r1mini_gui_teleop::MainWindow::on_buttonResetOdom_clicked()
{
    qnode.service_call_resetOdom();
}

void r1mini_gui_teleop::MainWindow::on_buttonSetImageCount_clicked()
{
   int cntVal = ui.spinBoxImageCountNum->value();
   std::cout<<"CNT = "<<cntVal<<std::endl;
   qnode.set_image_count(cntVal);
}

void r1mini_gui_teleop::MainWindow::on_buttonCheckBattery_clicked()
{
    qnode.service_call_Battery();
}

void r1mini_gui_teleop::MainWindow::on_checkBox_BatteryPeriodic_clicked(bool checked)
{
  if(checked) qnode.set_BatteryCheckPeriodic(true);
  else qnode.set_BatteryCheckPeriodic(false);
}
