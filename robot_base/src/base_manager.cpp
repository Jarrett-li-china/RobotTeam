#include "base_manager.h"

RobotManager::RobotManager(std::string port):m_port(port) {
		m_serial = new serial::Serial(m_port, 115200, serial::Timeout::simpleTimeout(500));
		if(!m_serial) throw RobotExecption("open error on port:" + m_port);
		
		m_query_thread = boost::thread(boost::bind(&RobotManager::query, this));
		m_set_thread = boost::thread(boost::bind(&RobotManager::set, this));
}

RobotManager::~RobotManager() throw(){
		if(flag) flag = false;
		m_query_thread.join();
		if(set_flag) set_flag = false;
		m_set_thread.join();
		if(m_serial){
				delete m_serial;
				m_serial = NULL;
		}
}


void RobotManager::query() {
    flag = true;
		while(flag){
				std::vector<int> tem_rpm (2, 999999999);
				if(get_rpm(tem_rpm)) curr_rpm = tem_rpm;			
				usleep(500);
		}	
}

void RobotManager::set_goal_rpm(std::vector<int> rpm){
    if (goal_rpm != rpm)  goal_rpm = rpm;
}

void RobotManager::set(){
    set_flag = true;
    while(set_flag){
        set_rpm(goal_rpm);
        usleep(50*1000);
    }
}



bool RobotManager::get_rpm(std::vector<int>& rpm){
		
		std::string cmd = "?S\r";
		boost::mutex::scoped_lock lock(g_mutex);
		uint8_t num = m_serial->write(cmd);
		if(num != cmd.size())
				throw RobotExecption("error: send command to driver, cmd: "+ cmd);

		std::string read_line;
		num = m_serial->readline(read_line, 65536, "\r");
		if(read_line == cmd) {
				//std::cout << "recevice cmd: " << read_line << std::endl;
		}
		//read_line.clear();
		num = m_serial->readline(read_line, 65536, "\r");
		#if TEST 
//		std::cout << "read_line: " <<read_line << std::endl; 
		#endif
		//S=0:0
		lock.unlock();
		int pos1 = read_line.find('=');//find pos '='
		if(pos1 == std::string::npos) {
		  //std::cout << "query rpm error" << std::endl;
		  return false;
		  //throw RobotExecption ("read error");
		}
		int pos2 = read_line.find(':');//find pos ':'
		if(pos2 == std::string::npos) {
		  //std::cout << "query rpm error" << std::endl;
		  return false;
		  //throw RobotExecption ("read error");
		}
		int pos3 = read_line.find('\r');//find pos '\r'
		if(pos3 == std::string::npos) {
		  //std::cout << "query rpm error" << std::endl;
		  return false;
		  //throw RobotExecption ("read error");
		}
	
//		std::cout <<"pos1: " << pos1 << std::endl;
//		std::cout <<"pos2: " << pos2 << std::endl;
//		std::cout <<"pos3: " << pos3 << std::endl;
//		std::cout <<"size: " << read_line.size()<< std::endl;
		std::string rpm1(&read_line[pos1+1], 0, pos2-pos1-1);
		std::string rpm2(&read_line[pos2+1], 0, pos3-pos2-1);
		
		//std::cout << "rpm1:" << rpm1 << "--" <<std::endl;
		//std::cout << "rpm2:" << rpm2 << "--" <<std::endl;

		rpm[0] = atoi(rpm1.c_str());
		rpm[1] = atoi(rpm2.c_str());
		
		return true;
}


bool RobotManager::set_rpm(std::vector<int> rpm)
{
		boost::mutex::scoped_lock lock(g_mutex);
	
		if(rpm.size() != 2) return false;
		//std::string cmd = "!M 100 100\r";
		std::string cmd = "!M ";
		cmd += std::to_string(-rpm[0]);
		cmd += " ";
		cmd += std::to_string(-rpm[1]);
		cmd += "\r";
		#if TEST
		std::cout << cmd << std::endl;
		#endif
		int num = m_serial->write(cmd);
		if(num != cmd.size()){
					std::cout << "set rpm success" << std::endl;
		}
		std::string read_line;
		num = m_serial->readline(read_line, 65536, "\r");
		if(read_line != "+\r") return false;
		//m_serial->flushOutput();
		return true;
}


