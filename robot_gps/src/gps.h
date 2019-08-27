#ifndef _GPS_H
#define _GPS_H

/*
	hardware: LCX-BD05F, www.skytoprf.com
	communication: serial Baud Rate@ 115200
	work mode: rmc
		notice: this software work at RMC && GGA mode. 
		if it's not, please refer PTK_config.txt to set work mode.
			1. freset
			2. log gprmc ontime 1
			3. log gpgga ontime 1
			4. savecofig
			5. reset
	author: Chen
	date: 09/11/2017
*/

#include "serial/serial.h"
#include "vector"
#include "string"
#include "boost/bind.hpp"
#include "boost/thread/thread.hpp"
#include "boost/thread/mutex.hpp"
#include "boost/algorithm/string/classification.hpp"
#include "boost/algorithm/string/split.hpp"
class gps{
public:
	gps(std::string port="/dev/ttyUSB1");
	~gps();
	bool is_ntr_available();
	std::string& get_gtime();//greenwich time
	std::string& get_gdate();	
	double get_angle();
	double get_deltax();
	double get_deltay();

private:
	serial::Serial *m_serial;
	std::string m_port;	
	std::vector<std::string> m_raw_data_ntr;	
	boost::thread m_work_thread;
	bool m_work_flag;
	void query();
	void split_ntr_data();
	boost::mutex m_mutex;
	std::string m_time;
	std::string m_date;
	bool m_available_ntr;

};
#endif //_GPS_H
