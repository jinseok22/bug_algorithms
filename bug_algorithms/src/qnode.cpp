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
#include <sstream>
#include "../include/bug_algorithms/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace bug_algorithms {

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
	ros::init(init_argc,init_argv,"bug_algorithms");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    lds         = n.subscribe("/scan",10, &QNode::LDSMsgCallback, this);
    odom_sub    = n.subscribe("/odom",10,&QNode::odomposeCallback, this);
    goal_pose_x=-4.5; goal_pose_y=-6.5;
    init_pose_x=init_pose_y=0;
    q_hit_x=q_hit_y=q_leave_x=q_leave_y=0;
    q_hit_flag=q_leave_flag=q_obstacle_excape=q_first_flag=init_pose_flag=q_again_hit=moving_direction_flag=first_move_flag=false;
    option_type=moving_direction=0;
    start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"bug_algorithms");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    lds         = n.subscribe("/scan",10, &QNode::LDSMsgCallback, this);
    odom_sub    = n.subscribe("/odom",10,&QNode::odomposeCallback, this);
    goal_pose_x=-4.5; goal_pose_y=-6.5;
    init_pose_x=init_pose_y=0;
    q_hit_x=q_hit_y=q_leave_x=q_leave_y=0;
    q_hit_flag=q_leave_flag=q_obstacle_excape=q_first_flag=init_pose_flag=q_again_hit=moving_direction_flag=first_move_flag=false;
    option_type=moving_direction=0;
    start();
	return true;
}

void QNode::run() {
    ros::Rate loop_rate(10);
	int count = 0;
	while ( ros::ok() ) {

        Q_EMIT bug_states(QString("%1, %2, %3").arg(current_pose_x).arg(current_pose_y).arg(current_orien_z),QString("x:%1 y:%2").arg(q_hit_x).arg(q_hit_y)
                          ,QString("x:%1 y:%2").arg(q_leave_x).arg(q_leave_y),QString("x:%1 y:%2").arg(goal_pose_x).arg(goal_pose_y));
        //linear_moving(goal_pose_x,goal_pose_y);
        if(option_type==1){
            obstacle_following();
        }
        else if(option_type==2){
            obstacle_following2();
        }
        else if(option_type==3){
            obstacle_following3();
        }

        //Q_EMIT bugPose("please give a chance");
		ros::spinOnce();
		loop_rate.sleep();
        //++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
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
                //logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                logging_model_msg << "[INFO] [" << "]: " << msg;
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




void QNode::LDSMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
        lds_range_min=0.35;// at least more size
        lds_range_max=msg->ranges[0];

        for(int i=0;i<360;i++)
            lds_range[i]=0;

        for (int i=0;i<360;i++)//(int i=0;i<360;i++) 90 270 100 260
        {
                float a = msg->ranges[i];
                lds_range[i]=msg->ranges[i];
                if(lds_range_min>a&&a>=msg->range_min){
                        lds_range_min=a;
                        lds_range_min_theta = i;
                }
                if(lds_range_max<a&&a<=msg->range_max){
                        lds_range_max=a;
                }
        }

}

void QNode::update_command_velocity(float x_v, float y_v){
    cmd_vel_msg.linear.x = x_v;
    cmd_vel_msg.linear.y = y_v;

    cmd_vel_pub.publish(cmd_vel_msg);
}

void QNode::linear_moving(float target_x, float target_y){
    float m = (target_y - current_pose_y)/(target_x - current_pose_x);
    float sign_x = copysign(1,target_x - current_pose_x);
    float sign_y = copysign(1,target_y - current_pose_y);
    float x_speed = sign_x*default_linear_x_speed;
    float y_speed = sign_y*default_linear_x_speed*(float)fabs(m);

    if(fabs(target_x - current_pose_x) <0.1){
        x_speed=0; m=1;
    }
    if(fabs(target_y - current_pose_y) <0.1){
        y_speed=0;
    }
    update_command_velocity(x_speed,y_speed);
}

void QNode::obstacle_follow_moving(int dir){
    edgeA=edgeA2=edgeB=edgeB2=edge1=edge2=obstacle_following_theta=obstacle_tanget_theta=edge_ox=edge_oy=0;
    theta_err=target_tangent_theta=0;
    for(int i=0;i<180;i++){
        if(lds_range[i]>0.01 && lds_range[i]<obstacle_check_distance){
            edgeA=i;
            break;
        }
    }
    for(int i=180;i<360;i++){
        if(lds_range[i]>0.01 && lds_range[i]<obstacle_check_distance){
            edgeA2=i;
            break;
        }
    }
    for(int j=359;j>=180;j--){
        if(lds_range[j]>0.01 && lds_range[j]<obstacle_check_distance){
            edgeB=j;
            break;
        }
    }
    for(int j=179;j>=0;j--){
        if(lds_range[j]>0.01 && lds_range[j]<obstacle_check_distance){
            edgeB2=j;
            break;
        }
    }
    if(edgeA<=1 && edgeB>=358){//front
        edge1=edgeB2;
        edge2=edgeA2;
        obstacle_following_theta=(edge1+360-edge2)/2 +edge2;
        if(obstacle_following_theta>=360) obstacle_following_theta-=360;
    }
    else if(edgeA2<=181&&edgeB2>=178){//back
        edge1=edgeB;
        edge2=edgeA;
        obstacle_following_theta=(edge1+edge2)/2;
    }
    else if(edgeA<=1&&edgeB2<=1){//right
        edge1=edgeB;
        edge2=edgeA2;
        obstacle_following_theta=(edge1+edge2)/2;
    }
    else if(edgeA2<=1&&edgeB<=1){//left
        edge1=edgeB2;
        edge2=edgeA;
        obstacle_following_theta=(edge1+edge2)/2;
    }
    if(option_type==3 && moving_direction_flag==false){
        moving_direction_flag=true;
        if(lds_range[edge1]<=lds_range[edge2])
            moving_direction=1;
        else
            moving_direction=-1;
        dir=moving_direction;
    }

    obstacle_tanget_theta=obstacle_following_theta+90*dir;
    if(obstacle_tanget_theta>=360) obstacle_tanget_theta-=360;
    if(obstacle_tanget_theta<0) obstacle_tanget_theta+=360;

    theta_err=(0.19-lds_range_min)*100*GAIN_P;

    target_tangent_theta = theta_err*dir + obstacle_tanget_theta;
    if(target_tangent_theta>=360) target_tangent_theta-=360;
    if(target_tangent_theta<0) target_tangent_theta+=360;

    update_command_velocity((float)cos(target_tangent_theta*M_PI/180)*default_linear_x_speed,(float)sin(target_tangent_theta*M_PI/180)*default_linear_x_speed);
}

void QNode::obstacle_following(){
    if(lds_range_min < obstacle_check_distance&&q_obstacle_excape==false){
        if(q_first_flag==false){
            q_first_flag=true;
            q_hit_x = current_pose_x;
            q_hit_y = current_pose_y;
            q_leave_x = current_pose_x;
            q_leave_y = current_pose_y;
            log(Info,std::string("Contact obstacle and Check qhit point"));
        }
        odom2goal_distance=sqrt((float)pow(current_pose_x-goal_pose_x,2)+(float)pow(current_pose_y-goal_pose_y,2));
        q_leave_distance=sqrt((float)pow(q_leave_x-goal_pose_x,2)+(float)pow(q_leave_y-goal_pose_y,2));
        if(odom2goal_distance<q_leave_distance){
            q_leave_x = current_pose_x;
            q_leave_y = current_pose_y;
        }
        if(abs(q_leave_x - current_pose_x)<0.05 && abs(q_leave_y-current_pose_y)<0.05&&q_leave_flag){
            q_obstacle_excape=true;
            log(Info,std::string("Leave obstacle"));
        }


        if(abs(q_hit_x - current_pose_x)<0.05 && abs(q_hit_y-current_pose_y)<0.05){
            if(q_hit_flag){
                if(q_leave_flag==false) log(Info,std::string("Contact again qhit point"));
                q_leave_flag=true;
            }
        }
        else{
            q_hit_flag=true;
        }
        obstacle_follow_moving(1);
    }
    else{
        q_hit_flag=q_leave_flag=q_first_flag=false;
        odom2goal_distance=q_leave_distance=0;
        if(q_obstacle_excape && lds_range_min>0.33){
            q_obstacle_excape=false;
        }
        linear_moving(goal_pose_x,goal_pose_y);
    }
}

void QNode::obstacle_following2(){
    if(lds_range_min < obstacle_check_distance&&q_obstacle_excape==false){
        if(q_first_flag==false){
            q_first_flag=true;
            q_hit_x = current_pose_x;
            q_hit_y = current_pose_y;
            q_leave_x = current_pose_x;
            q_leave_y = current_pose_y;
            log(Info,std::string("Contact obstacle and Check qhit point"));
        }

        if(abs(((goal_pose_y - init_pose_y)/(goal_pose_x-init_pose_x))*(current_pose_x-init_pose_x)-(current_pose_y-init_pose_y))<0.08&&q_hit_flag){//check contact M-Line
            q_obstacle_excape=true;
            log(Info,std::string("Contact M-Line and Leave obstacle"));
            q_leave_x = current_pose_x;
            q_leave_y = current_pose_y;
        }

        if(abs(q_hit_x - current_pose_x)<0.05 && abs(q_hit_y-current_pose_y)<0.05){

        }
        else{
            q_hit_flag=true;
        }

        obstacle_follow_moving(1);
    }
    else{
        q_hit_flag=q_leave_flag=q_first_flag=false;
        if(q_obstacle_excape && lds_range_min>0.33){
            q_obstacle_excape=false;
        }
        linear_moving(goal_pose_x,goal_pose_y);
    }
}

void QNode::obstacle_following3(){
    if(lds_range_min < obstacle_check_distance&&q_obstacle_excape==false){
        if(q_first_flag==false){
            q_first_flag=true;
            q_hit_x = current_pose_x;
            q_hit_y = current_pose_y;
            q_leave_x = current_pose_x;
            q_leave_y = current_pose_y;
            log(Info,std::string("Contact obstacle"));
        }
        odom2goal_distance=sqrt((float)pow(current_pose_x-goal_pose_x,2)+(float)pow(current_pose_y-goal_pose_y,2));

        if(moving_direction!=0){
            edge_ox=current_pose_x+lds_range[edge1]*(float)cos(edge1*M_PI/180);
            edge_oy=current_pose_y+lds_range[edge1]*(float)sin(edge1*M_PI/180);
            d_min1=sqrt((float)pow(edge_ox-goal_pose_x,2)+(float)pow(edge_oy-goal_pose_y,2));
            edge_ox=current_pose_x+lds_range[edge2]*(float)cos(edge2*M_PI/180);
            edge_oy=current_pose_y+lds_range[edge2]*(float)sin(edge2*M_PI/180);
            d_min2=sqrt((float)pow(edge_ox-goal_pose_x,2)+(float)pow(edge_oy-goal_pose_y,2));
            if(d_min1>d_min2){
                d_min1=d_min2;
            }
            d_leave=odom2goal_distance-obstacle_check_distance;

            float mm = atan2(goal_pose_y-current_pose_y,goal_pose_x-current_pose_x);
            //if(mm<0) mm=2*M_PI + mm;

            if((cos(mm-obstacle_following_theta*M_PI/180))<=0) q_leave_flag=true;//cos(A-B)=cos(A)*cos(B)+sin(A)*sin(B)

            if(d_leave<d_min1&&q_hit_flag&&q_leave_flag){
                q_leave_x = current_pose_x;
                q_leave_y = current_pose_y;
                if(q_obstacle_excape==false) log(Info,std::string("Leave obstacle"));
                q_obstacle_excape=true;
            }
        }
        if(abs(q_hit_x - current_pose_x)<0.05 && abs(q_hit_y-current_pose_y)<0.05){

        }
        else{
            q_hit_flag=first_move_flag=true;
        }

        obstacle_follow_moving(1);
    }
    else{
        q_hit_flag=q_leave_flag=q_first_flag=false;
        odom2goal_distance=q_leave_distance=moving_direction=0;
        moving_direction_flag=false;
        if(q_obstacle_excape && lds_range_min>0.33){
            q_obstacle_excape=false;
        }
        if(first_move_flag){
            first_move_flag=false;
            ros::Duration(0.3).sleep(); //0.3 sec sleep
        }
        linear_moving(goal_pose_x,goal_pose_y);
    }
}

void QNode::sendOption(int option){
    option_type = option;
    std::stringstream ss;
    ss << option_type;
    log(Info,std::string("Choice option : ")+ss.str());
    init_pose_flag=true;
}

void QNode::odomposeCallback(const nav_msgs::Odometry &odom_msg){
    current_pose_x=odom_msg.pose.pose.position.x;
    current_pose_y=odom_msg.pose.pose.position.y;
    current_orien_z=odom_msg.pose.pose.orientation.z;
    if(init_pose_flag){
        init_pose_x = current_pose_x;
        init_pose_y = current_pose_y;
        init_pose_flag=false;
        std::stringstream ss;
        ss << "init pose x : "<<init_pose_x<<" y : "<<init_pose_y;
        log(Info,ss.str());
    }
}

}  // namespace bug_algorithms
