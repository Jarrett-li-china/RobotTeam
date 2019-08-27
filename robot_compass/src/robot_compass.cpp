
/* Copyright : 2019 Henan Institute of Science and Technology Robotics Research Institute
 * Author : Li Yuejun
 * Description :A compass node of main program file.
 * History :
 *     201908 : Initial this cpp file.
 */

#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include "geometry_msgs/Twist.h"

#include "boost/bind.hpp"
#include "boost/thread/thread.hpp"
#include "boost/thread/mutex.hpp"

#include "serial/serial.h"

#include "string"
#include "vector"

#include "robot_compass/Compass.h"

boost::mutex g_mutex;

//LP3300
class Compass{
public:
	Compass(std::string port="/dev/ttyS5");
	~Compass();
	ros::NodeHandle m_nh;
private:
	boost::thread m_query_thread;
	void query_work();
	bool m_query_flag;

	boost::thread m_compass_pub_thread;
	void pub_work();
	bool m_compass_pub_flag;

	ros::Publisher m_compass_pub;
	ros::Publisher m_calibration_pub;

	ros::ServiceServer m_calibration_srv;
	bool m_servicing;

	bool calibration(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
	serial::Serial* m_serial;
	std::string m_port;
	
	std::vector<uint8_t> m_raw_data;
};

void Compass::query_work()
{
	ros::Rate loop(10);
	while(m_query_flag){
		//read header
		if(m_servicing) continue;
		uint8_t header;
		std::vector<uint8_t> raw_data;
		boost::mutex::scoped_lock lock(g_mutex);	
		if( 1 != m_serial->read(&header, 1)) continue;
		if( header != 0xaa) continue;
			//read 19 Bytes
		if( 19 != m_serial->read(raw_data, 19)) continue;
		m_raw_data = raw_data;
		lock.unlock();
	
		loop.sleep();
	}

}


double get_value(uint16_t value)
{
	double tem;
	if(value <= 0x7fff) tem = value/1000.0/3.14159*180.0;
	else tem = -(value xor 0xffff)/1000.0/3.14159*180.0;
	return tem;
}

bool Compass::calibration(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
	if(req.data == true){
		ROS_INFO("call calibration");
		m_servicing = true;
		//ros::Duration du(10);
		//du.sleep();
		std::vector<uint8_t> frame;
		const uint8_t start_cmd = 0xd0; //rotate over 360 degree
		const uint8_t stop_cmd = 0xd1; //save calibration result
	//--------------------------------------------------------	
	//start calibration
		if(1 != m_serial->write(&start_cmd, 1)){
			res.success = false;
			res.message = "calibration failed on start cmd: 0xd0";
		}
		while(true){
			uint8_t header;
			if( 1 != m_serial->read(&header, 1)) continue;
			if( 0xaa != header) continue;
			printf("[aa]");
			if( 19 != m_serial->read(frame, 19)) continue;
			if( 0xd0 != frame[0]) continue;
			for(auto tem: frame) printf("[%x]", tem);	
			break;
		}
	ROS_INFO("start cmd (0xd0) has been written to compass register");
	//----------------------------------------------
	//correcting 
	//rotation the compass over 360 degree on a horizontal plane, 
	//mean that publish a rotation veloctiy topic
	geometry_msgs::Twist cali_vel; 	
		cali_vel.angular.z = 0.5;
		ros::Rate loop(10);
		uint8_t times = (2*3.14 / 0.5+0.5) *10;
		while(times--){
			m_calibration_pub.publish(cali_vel);		
			loop.sleep();
		}
		cali_vel.angular.z = 0;
		m_calibration_pub.publish(cali_vel);		
	
	ROS_INFO("calibration velocity has been pubished, agular.z=0.5");
	m_serial->flush();
	//	
		if(1 != m_serial->write(&stop_cmd, 1)){
			res.success = false;
			res.message = "calibration failed on save cmd 0xd1";
		}
		while(true){
			uint8_t header;
			frame.clear();
			if( 1 != m_serial->read(&header, 1)) continue;
			if( 0xaa != header) continue;
			if( 19 != m_serial->read(frame, 19)) continue;
			if( 0xd1 != frame[0]) continue;
			printf("[aa]");
			for(auto tem: frame) printf("[%x]", tem);	
			break;
		}
	ROS_INFO("save cmd (0xd1) write to compass register, save register success");

	res.success = true;
	res.message = "calibration success";
	}else{
		res.success = false;
		res.message = "calibaration request is false";
	}

	m_servicing = false;
	return true;
}

void Compass::pub_work()
{
	ros::Rate loop(10);
	ros::Duration du(1);
	du.sleep();
	robot_compass::Compass msg;
	msg.header.frame_id = "compass";
	while(m_compass_pub_flag){
		if(m_servicing) continue;
//		for(auto value:m_raw_data)
//			printf("[%x]", value);
//		printf("\n");
		msg.header.stamp = ros::Time::now();
//		//m_raw_data [5,6]
		
		uint16_t tem1 =  m_raw_data[5]<<8;
				 tem1 += m_raw_data[6];
		msg.x_horizon = get_value(tem1);
//		ROS_INFO("x_horizon[%g]", msg.x_horizon);
		//m_raw_data [7,8]
		uint16_t tem2 =  m_raw_data[7]<<8;
				 tem2 += m_raw_data[8];
		msg.y_horizon = get_value(tem2);
//		ROS_INFO("y_horizon[%g]", msg.y_horizon);
		//m_raw_data [15,16]
		uint16_t tem3 =  m_raw_data[15]<<8;
				 tem3 += m_raw_data[16];
		msg.north_err = tem3/100.0f;
//		ROS_INFO("north_err[%g]", msg.north_err);
		m_compass_pub.publish(msg);
		loop.sleep();
	}
}


Compass::Compass(std::string port):m_port(port)
{
	m_serial = new serial::Serial(m_port, 9600, serial::Timeout::simpleTimeout(500));
	if(!m_serial) {
		throw serial::SerialException("serial port open failed");
	}

	m_compass_pub = m_nh.advertise<robot_compass::Compass>("/compass", 10);

	m_servicing = false;

	m_query_flag = true;
	m_query_thread = boost::thread(boost::bind(&Compass::query_work, this));

	m_compass_pub_flag= true;
	m_compass_pub_thread = boost::thread(boost::bind(&Compass::pub_work, this));

	m_calibration_srv = m_nh.advertiseService("/compass_calibration", &Compass::calibration, this);
	m_calibration_pub = m_nh.advertise<geometry_msgs::Twist>("/cali_vel", 1);
}

Compass::~Compass()
{
	if(m_compass_pub_flag) m_compass_pub_flag = false;
	m_compass_pub_thread.join();
	if(m_query_flag) m_query_flag = false;
	m_query_thread.join();
	delete m_serial;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "compass");
	Compass com;
	ros::spin();
	return 0;
}

