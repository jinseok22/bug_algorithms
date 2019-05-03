/**
 * @file /include/bug_algorithms/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef bug_algorithms_QNODE_HPP_
#define bug_algorithms_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/network.h>

//#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QString>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
//#include <sys/stat.h>
//#include <move_base_msgs/MoveBaseActionResult.h>
//#include "actionlib_msgs/GoalStatusArray.h"
//#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#endif

#define default_linear_x_speed 0.2
#define obstacle_check_distance 0.3
#define GAIN_P 1.5;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace bug_algorithms {

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

        void sendOption(int option);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void bug_states(QString ,QString ,QString ,QString );
    //void bugPose(QString);

private:
	int init_argc;
	char** init_argv;

        std_msgs::Int32 tts_n;
        geometry_msgs::Twist twist; // ROS 메시지에 사용할 변수 선언
        nav_msgs::Odometry odom;

        float goal_pose_x, goal_pose_y, init_pose_x, init_pose_y;
        float current_pose_x, current_pose_y, current_orien_z;
        float q_hit_x,q_hit_y, q_leave_x, q_leave_y;
        float odom2goal_distance, q_leave_distance, d_leave, d_min1,d_min2;

        bool q_hit_flag, q_leave_flag,q_obstacle_excape, q_first_flag, init_pose_flag, q_again_hit,first_move_flag;

	ros::Publisher chatter_publisher;

        void LDSMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
        float lds_range_min, lds_range_max, lds_range[360],lds_range_min_theta;
        void obstacle_follow_moving(int dir);
        void obstacle_following();
        void obstacle_following2();
        void obstacle_following3();
        int edgeA,edgeB,edgeA2,edgeB2, edge1, edge2, obstacle_following_theta, obstacle_tanget_theta,edge_ox,edge_oy;
        int theta_err,target_tangent_theta;
        int moving_direction;
        bool moving_direction_flag;
        void odomposeCallback(const nav_msgs::Odometry &odom_msg);


        ros::Subscriber lds;
        ros::Subscriber odom_sub;

        //publish

        // Command velocity of robot using joystic remote controller
        geometry_msgs::Twist cmd_vel_msg;
        ros::Publisher cmd_vel_pub;
        void update_command_velocity(float x_v, float y_v);
        void linear_moving(float target_x, float target_y);

    QStringListModel logging_model;
    QString bug_pose;
    int option_type;
};

}  // namespace bug_algorithms

#endif /* bug_algorithms_QNODE_HPP_ */
