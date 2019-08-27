
/* Copyright : 2019 Henan Institute of Science and Technology Robotics Research Institute
 * Author : Li Yuejun
 * Description :A GPS data publishing node of main program file.
 * History :
 *     201908 : Initial this cpp file.
 */

#include "ros/ros.h"
#include "gps.h" 
#include "tf/tf.h"
#include "robot_gps/RobotGPS.h"
#include "tf/transform_broadcaster.h"
#include <iostream>
#include <string>
#include <vector>
#include <fstream>

using namespace std;
int cost[50][50];

class robot_gps_sensor{ 
public:
	robot_gps_sensor(std::string port = "/dev/ttyUSB0");
	virtual ~robot_gps_sensor();  
private:
	gps* m_gps;
	ros::NodeHandle m_handle;
	ros::NodeHandle m_private_handle;
	ros::Publisher m_gps_pub;
    ros::Subscriber sub;
	tf::TransformBroadcaster m_broadcaster;
	boost::thread m_pub_thread;
    boost::thread button_work;
	bool m_work_flag;
	void work();
	std::string m_port;
    double button_a, button_x, button_y;
};

robot_gps_sensor::robot_gps_sensor(std::string port):m_work_flag(true), m_private_handle(ros::NodeHandle("~"))
{
	if(!m_private_handle.getParam("serialport", m_port))
	{
		m_port = "/dev/ttyUSB1";
		ROS_INFO("cannot find param in launch file, try to open default prot: /dev/ttyUSB0");
	}
	m_gps = new gps(m_port);
	if(m_gps)
	{
		m_gps_pub = m_handle.advertise<robot_gps::RobotGPS>("robot/gps", 10);
		m_pub_thread = boost::thread(boost::bind(&robot_gps_sensor::work, this));
	}
}

robot_gps_sensor::~robot_gps_sensor()
{
	m_work_flag = false;
	m_pub_thread.join();
	if(m_gps) delete m_gps;
}

void robot_gps_sensor::work()
{
	sleep(1);
	ros::Rate loop(10);
	while(m_work_flag)
	{
		ros::Time currtime = ros::Time::now();
		robot_gps::RobotGPS msg;
		msg.header.frame_id = "robot_gps";
		msg.header.stamp = currtime;
		if(m_gps->is_ntr_available())
		{
			msg.deltax=m_gps->get_deltax();
			msg.deltay=m_gps->get_deltay();
			m_gps_pub.publish(msg);
			tf::Transform transform;
			transform.setOrigin(tf::Vector3(0, 0, 1));
			transform.setRotation(tf::Quaternion(0,0,0,1));		
			m_broadcaster.sendTransform(tf::StampedTransform(transform, currtime, "base_footprint", "gps"));
		}//if
		loop.sleep();
	}//while
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_gps");
	robot_gps_sensor gps;
	ros::spin();
	return 0;
}
