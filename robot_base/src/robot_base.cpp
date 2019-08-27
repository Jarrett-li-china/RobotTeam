#include "ros/ros.h"
#include "base_manager.h"

#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class Robot_Con{
private:
  RobotManager *manager;
	void callBack(const geometry_msgs::Twist &twist);
	ros::NodeHandle m_handle;
	ros::NodeHandle m_private_handle;
	ros::Subscriber m_vel_sub;
	ros::Publisher m_odom_pub;
	boost::thread m_pubThread;
	bool m_workFlag;
	//topic cmd_vel
	double m_vel_x, m_vel_y, m_vel_th;	

	ros::Time m_currentTime;
	ros::Time m_lastTime;

	double m_pos_x, m_pos_y, m_pos_th;
	tf::TransformBroadcaster m_broadcaster;
	geometry_msgs::TransformStamped m_odom_trans;
	nav_msgs::Odometry m_odom;
	std::string m_port;
	
	void pub_work();
public:
	double l = 0.7125; //car len  
	double w = 0.58; //car width   

	geometry_msgs::Twist m_twist;
	Robot_Con(std::string port="/dev/ttyUSB0"); 
	virtual ~Robot_Con();
  
  const double reduction=7.5;
  const double radius=0.0845;
  
};

Robot_Con::Robot_Con(std::string port){
    
    m_private_handle=ros::NodeHandle("~");
    if(!m_private_handle.getParam("serialport", m_port)){
    	m_port="/dev/ttyS6";
    	ROS_INFO("cannot find param in launch file,try to open defaulte port /dev/ttyS4 ");
    }
    manager = new RobotManager(m_port);
    if(!manager){
    throw RobotExecption("open error on port:"+m_port);
    }
    m_vel_sub = m_handle.subscribe("cmd_vel", 1, &Robot_Con::callBack, this);
    m_odom_pub = m_handle.advertise<nav_msgs::Odometry>("Robot/odom", 1);
    
    m_workFlag = true;
    m_pos_x = m_pos_y = m_pos_th = 0;
    m_vel_x = m_vel_y = m_vel_th = 0;
    m_pubThread = boost::thread(boost::bind(&Robot_Con::pub_work, this));
    m_lastTime = ros::Time::now();
}

void Robot_Con::callBack(const geometry_msgs::Twist &twist){
	
	double vel_x = twist.linear.x*300;
	double vel_y = twist.linear.y;//暂时没有不设置
	double ang_z = twist.angular.z*200;
	ROS_INFO("[%g] [%g] [%g]\n", vel_x, vel_y, ang_z);

	std::vector<int> rpm(2, 0);
	if(vel_y == 1){
    rpm[0] = 0;
	  rpm[1] = 0;
    exit(0);	
  }else{
    rpm[0] = ang_z;
	  rpm[1] = vel_x;
  }  
//	double m_r = vel_x + ang_z*(w*w + l*l)/(2*w);
//	double m_l = -vel_x + ang_z*(w*w + l*l)/(2*w);
	//----------------------------------------------------------
	//set vel
//	std::vector<int> rpm(2, 0);
//	rpm[0] =  m_l / radius / 2 / 3.1415926 * 60 * reduction / 3;//rpm
//	rpm[1] =  m_r / radius / 2 / 3.1415926 * 60 * reduction / 3;
// 	ROS_INFO("---------[%d %d]\n", rpm[0], rpm[1]); 
  	manager->set_goal_rpm(rpm);
  
}

Robot_Con::~Robot_Con(){
  manager->set_goal_rpm(std::vector<int>(2, 0));
	m_workFlag = false;		
	m_pubThread.join();
	if(manager){
		delete manager;	
	}
}

void Robot_Con::pub_work(){
    ros::Rate loop (10);
    usleep(100*1000);
    while(m_workFlag){
    //ROS_INFO("------------------");
    m_currentTime = ros::Time::now();
    std::vector<int> current_rpm = manager->curr_rpm;
    //if(manager->get_rpm(current_rpm)){
	
	ROS_INFO("========%d  %d",current_rpm[0],current_rpm[1]);
       
      double m_r = -current_rpm[0]/reduction*2*3.1415926*radius/60; //m/s
      double m_l = current_rpm[1]/reduction*2*3.1415926*radius/60;
      m_vel_x = (m_r + m_l) / 2.0;
      m_vel_th = -2*w*(m_l - m_r)/(l*l + w*w);
//	m_vel_th = -(m_l - m_r)/w;
      m_vel_y = 0;

	    ROS_INFO("vel %5g %5g %5g\n", m_vel_x, m_vel_y, m_vel_th);  
			
	      
      double dt = (m_currentTime - m_lastTime).toSec();
      double delta_x = (m_vel_x * cos(m_pos_th) - m_vel_y * sin(m_pos_th)) * dt;
      double delta_y = (m_vel_x * sin(m_pos_th) + m_vel_y * cos(m_pos_th)) * dt;
      double delta_th = m_vel_th * dt;
      m_pos_x += delta_x;
      m_pos_y += delta_y;
      m_pos_th += delta_th;

      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,m_pos_th);
      m_odom.header.stamp = m_currentTime;
      	m_odom.header.frame_id = "odom";
      	m_odom.child_frame_id = "base_footprint";
        // position
        m_odom.pose.pose.position.x = m_pos_x;
        m_odom.pose.pose.position.y = m_pos_y;
        m_odom.pose.pose.position.z = 0.0;
        m_odom.pose.pose.orientation = odom_quat;

        	//velocity
        m_odom.twist.twist.linear.x = m_vel_x;
        m_odom.twist.twist.linear.y = m_vel_y;
        m_odom.twist.twist.linear.z = 0.0;
        m_odom.twist.twist.angular.x = 0.0;
        m_odom.twist.twist.angular.y = 0.0;
        m_odom.twist.twist.angular.z = m_vel_th;
	
      m_odom_pub.publish(m_odom);

      m_odom_trans.header.stamp = m_currentTime;
      m_odom_trans.header.frame_id = "odom";
      m_odom_trans.child_frame_id = "base_footprint";
        m_odom_trans.transform.translation.x = m_pos_x;
        m_odom_trans.transform.translation.y = m_pos_y;
        m_odom_trans.transform.translation.z = 0.0;
        m_odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(m_pos_th);
      m_broadcaster.sendTransform(m_odom_trans);
    //}
    loop.sleep();
    m_lastTime = m_currentTime;
    ros::spinOnce();  
    
  }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Robot_Controller");
		
	Robot_Con Bobac;
	
	ros::spin();
	return 0;
}