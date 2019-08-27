#include "gps.h"

gps::gps(std::string port):m_port(port){
	m_serial = new serial::Serial(m_port, 115200, serial::Timeout::simpleTimeout(1000));
	if(m_serial){
		m_work_flag = true;
		m_work_thread = boost::thread(boost::bind(&gps::query, this));
	}
}

gps::~gps(){
	m_work_flag = false;
	m_work_thread.join();//等待查询线程结束
	std::cout << "exit" << std::endl;
	if(m_serial)
		delete m_serial;
}

void gps::query(){
	std::string ntr("$GPNTR");
	while(m_work_flag){	
		std::string raw_line;
		uint8_t num = m_serial->readline(raw_line, 65536, "\r\n");
		if(raw_line.find(ntr) >= 0 && raw_line.find(ntr) < raw_line.size()){		
			boost::split(m_raw_data_ntr, raw_line, boost::is_any_of(","), boost::token_compress_on);			
		}
	}
}

bool gps::is_ntr_available(){
	if(m_raw_data_ntr[2][0] == '4')
		return true;
	else
		return false;
}

double gps::get_deltax(){
	double Deltax = std::stof(m_raw_data_ntr[4]);
	//std::cout << "sat.no: " << no << std::endl;
	return Deltax;
}
double gps::get_deltay(){
	double Deltay = std::stof(m_raw_data_ntr[5]);
	//std::cout << "sat.no: " << no << std::endl;
	return Deltay;
}












