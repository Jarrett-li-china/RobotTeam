
/* Copyright : 2019 Henan Institute of Science and Technology Robotics Research Institute
 * Author : Li Yuejun
 * Description :A handle data publishing node of main program file.
 * History :
 *     201908 : Initial this cpp file.
 */

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<iostream>
#include<boost/thread/thread.hpp>
#include<boost/thread/mutex.hpp>
#include<boost/bind.hpp>
#include "robot_joy/joy_data.h"

using namespace std;

#define MAX_CAR_LINEAR_SPEED (0.3) 
#define MAX_CAR_ANGULAR_SPEED (0.8)

class RobotJoy{
public:
  RobotJoy();
  ~RobotJoy();
  void callBack(const sensor_msgs::Joy::ConstPtr &joy);
  ros::NodeHandle n;
  ros::Publisher n_car_vel_pub;
  ros::Publisher n_yuntai_vel_pub;
  ros::Publisher n_button_sta_pub;
  ros::Subscriber sub;

private:
  boost::thread car_vel_pub;
  boost::thread yuntai_vel_pub;
  boost::thread button_sta_pub;

  void car_vel_pub_work();
  void yuntai_vel_pub_work();
  void button_sta_pub_work();

  bool car_vel_pub_flag, yuntai_vel_pub_flag, button_sta_pub_flag;

  double car_linear_x, car_angular_z;
  double yuntai_angular_y, yuntai_angular_z;
  double button_a, button_b, button_x, button_y;
  // double yuntai_up, yuntai_down;

  std::vector<int8_t> m_a_button;
  bool m_a_req;
  
  std::vector<int8_t> m_b_button;
  bool m_b_req;
  
  std::vector<int8_t> m_x_button;
  bool m_x_req;
  
  std::vector<int8_t> m_y_button;
  bool m_y_req;
};

RobotJoy::RobotJoy()
{
    n_car_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
    n_yuntai_vel_pub = n.advertise<geometry_msgs::Twist>("yuntai_vel",1);
    n_button_sta_pub = n.advertise<robot_joy::joy_data>("button_data",1);
    sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &RobotJoy::callBack, this);

    car_vel_pub_flag = true;
	yuntai_vel_pub_flag = true;
    button_sta_pub_flag = true;

    m_a_button.resize(2);
	m_b_button.resize(2);
	m_x_button.resize(2);
	m_y_button.resize(2);

    m_a_req= false;
    m_b_req= false;
    m_x_req= false;
    m_y_req= false;

    car_vel_pub = boost::thread(boost::bind(&RobotJoy::car_vel_pub_work, this));
	yuntai_vel_pub = boost::thread(boost::bind(&RobotJoy::yuntai_vel_pub_work, this));
    button_sta_pub = boost::thread(boost::bind(&RobotJoy::button_sta_pub_work, this));
}

void RobotJoy::callBack(const sensor_msgs::Joy::ConstPtr &joy)
{
	
    car_angular_z = joy->axes[0]*0.7;
    car_linear_x = joy->axes[1]*0.7;
    yuntai_angular_z = joy->axes[3];
    yuntai_angular_y = joy->axes[4];
    // yuntai_up = joy->axes[2]; 
    // yuntai_down = joy->axes[5];
    
    //Button A
    if(m_a_button.size()<2) m_a_button.push_back(joy->buttons[0]);
    else{
        m_a_button.erase(m_a_button.begin());
        m_a_button.push_back(joy->buttons[0]);
        if(m_a_button[0]==0 && m_a_button[1]==1) {
            m_a_req = true;	
        }else {
            m_a_req= false;	
        }	
    }
    //Button B
    if(m_b_button.size()<2) m_b_button.push_back(joy->buttons[1]);
    else{
        m_b_button.erase(m_b_button.begin());
        m_b_button.push_back(joy->buttons[1]);
        if(m_b_button[0]==0 && m_b_button[1]==1) {
            m_b_req = true;	
        }else {
            m_b_req= false;	
        }	
    }
    //Button X
    if(m_x_button.size()<2) m_x_button.push_back(joy->buttons[2]);
    else{
        m_x_button.erase(m_x_button.begin());
        m_x_button.push_back(joy->buttons[2]);
        if(m_x_button[0]==0 && m_x_button[1]==1) {
            m_x_req = true;	
        }else {
            m_x_req= false;	
        }	
    }
    //Button Y
    if(m_y_button.size()<2) m_y_button.push_back(joy->buttons[3]);
    else{
        m_y_button.erase(m_y_button.begin());
        m_y_button.push_back(joy->buttons[3]);
        if(m_y_button[0]==0 && m_y_button[1]==1) {
            m_y_req = true;	
        }else {
            m_y_req= false;	
        }	
    }
  
}

void RobotJoy::car_vel_pub_work()
{
	ros::Rate loop(10);
	while(car_vel_pub_flag){
        geometry_msgs::Twist vel;
            vel.linear.x = car_linear_x*MAX_CAR_LINEAR_SPEED;
            vel.angular.z = car_angular_z*MAX_CAR_ANGULAR_SPEED;
        n_car_vel_pub.publish(vel);
        loop.sleep();
        ros::spinOnce();
	}
}

void RobotJoy::yuntai_vel_pub_work()
{
	ros::Rate loop(10);
	while(yuntai_vel_pub_flag){
		//ROS_INFO("yuntai_vel publisher");
		geometry_msgs::Twist vel;
			vel.angular.z = yuntai_angular_z;
			vel.angular.y = yuntai_angular_y;
		// if(yuntai_up < 0.95 && fabs(yuntai_up)>= 0.1){
		// 	vel.linear.z = 1 - yuntai_up;
		// }else if(yuntai_down < 0.95 && fabs(yuntai_down) >=0.1){
		// 	vel.linear.z = -(1 - yuntai_down);
		// }
		n_yuntai_vel_pub.publish(vel);
        //ROS_INFO("----------------yuntai_up=%g----linear.z=%g", yuntai_up,vel.linear.z);
		loop.sleep();
		ros::spinOnce();
	}
}

void RobotJoy::button_sta_pub_work()
{
    ros::Rate loop(10);
	while(button_sta_pub_flag){
        robot_joy::joy_data sta;
            sta.a_joybutton = m_a_req;
            sta.b_joybutton = m_b_req;
            sta.x_joybutton = m_x_req;
            sta.y_joybutton = m_y_req;
        n_button_sta_pub.publish(sta);
        loop.sleep();
		ros::spinOnce();
    }
}

RobotJoy::~RobotJoy()
{
    if(car_vel_pub_flag) car_vel_pub_flag = false;
	if(yuntai_vel_pub_flag) yuntai_vel_pub_flag = false;
    if(button_sta_pub_flag) button_sta_pub_flag = false;
    car_vel_pub.join();
	yuntai_vel_pub.join();
    button_sta_pub.join();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RobotJoy");
    RobotJoy Robot_turtle;
    ros::spin();
    return 0;
}