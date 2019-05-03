/**
 * @file /include/bug_algorithms/main_window.hpp
 *
 * @brief Qt based gui for bug_algorithms.
 *
 * @date November 2010
 **/
#ifndef bug_algorithms_MAIN_WINDOW_H
#define bug_algorithms_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace bug_algorithms {

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
    void on_BTN_CHECK_clicked(bool check);
    void on_BTN_START_clicked(bool check);
    void updatebugstates(QString bug_pose,QString hit_pose,QString leave_pose,QString goal_pose);
    //void updatebugPose(QString bug_pose1);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace bug_algorithms

#endif // bug_algorithms_MAIN_WINDOW_H
