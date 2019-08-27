#ifndef ROBOT_MANAGER_H 
#define ROBOT_MANAGER_H

#include "string"
#include "unistd.h"
#include "serial/serial.h"
#include "boost/bind.hpp"
#include "boost/thread/thread.hpp"
#include "boost/thread/mutex.hpp" 
#define TEST 1

class RobotExecption: public std::exception{
public:
	explicit RobotExecption (std::string error):m_errString(error){}
	std::string m_errString;
	virtual const char* what(){
		return m_errString.c_str();
	}
	virtual ~RobotExecption () throw(){}

};

class RobotManager{
public:
	RobotManager(std::string port="/dev/ttyUSB0");
	virtual ~RobotManager() throw();
	bool get_rpm(std::vector<int>& rpm);
	
	std::vector<int> curr_rpm;
	void set_goal_rpm(std::vector<int> rpm);
private:
  serial::Serial* m_serial;
	std::string m_port;
	bool flag;
	boost::thread m_query_thread;
	void query();
	boost::mutex g_mutex;
	
	boost::thread m_set_thread;
	bool set_flag;
	void set();
	std::vector<int> goal_rpm;
	bool set_rpm(std::vector<int> rpm);
};

#endif //ROBOT_MANAGER_H
