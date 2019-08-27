
/* Copyright : 2019 Henan Institute of Science and Technology Robotics Research Institute
 * Author : Li Yuejun
 * Description :A gps_nav node of main program file.
 * History :
 *     201908 : Initial this cpp file.
 */

#include<iostream>
#include <cstdlib>
#include <string>
#include <vector>
#include <fstream> 
#include<boost/thread/thread.hpp>
#include<boost/thread/mutex.hpp>
#include<boost/bind.hpp>
#include "ros/ros.h"
#include "robot_joy/joy_data.h"
#include "robot_gps/RobotGPS.h"

using namespace std;

class gps_nav_con{
public:
	gps_nav_con();
	~gps_nav_con();
    void Callback_gps(const robot_gps::RobotGPS::ConstPtr &gps);
    void Callback_button(const robot_joy::joy_data::ConstPtr &button); 
    ros::NodeHandle n;
    ros::Subscriber sub_gpsdata;
    ros::Subscriber sub_button;
private:
    boost::thread joy_gps;
    double x_data, y_data;
    double a_button, b_button, x_button, y_button;
    void button_work(); 
    ofstream outfile;  
};

//GPS数据回调函数
void gps_nav_con::Callback_gps(const robot_gps::RobotGPS::ConstPtr &gps)
{
    x_data = (gps->deltax) + 0.3;
    y_data = -(gps->deltay) - 0.15;

}
//手柄按钮数据回调函数
void gps_nav_con::Callback_button(const robot_joy::joy_data::ConstPtr &button)
{
    a_button = button->a_joybutton;
    b_button = button->b_joybutton;
    x_button = button->x_joybutton;
    y_button = button->y_joybutton;
}


gps_nav_con::gps_nav_con(){
    sub_gpsdata = n.subscribe<robot_gps::RobotGPS>("robot/gps", 10, &gps_nav_con::Callback_gps, this);
    sub_button = n.subscribe<robot_joy::joy_data>("button_data", 10, &gps_nav_con::Callback_button, this);
    joy_gps = boost::thread(boost::bind(&gps_nav_con::button_work, this));
}
gps_nav_con::~gps_nav_con(){
    joy_gps.join();
}

void gps_nav_con::button_work()
{
    // 设置循环的频率
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if(a_button)
        {
            outfile.open("GPS_data.txt", ios::trunc);   
            /*每次写都定位的文件结尾，不会丢失原来的内容，用out则会丢失原来的内容;用trunc清空文件内容*/
            if(!outfile.is_open ())
                cout << "Open file failure" << endl;
            else{
                cout << "Initialization file success" << endl;
            }
            outfile.close();
        }
        if(b_button)
        {
            outfile.open("GPS_data.txt", ios::app);
            if(!outfile.is_open ())
                cout << "Open file failure" << endl;
            else{
                outfile << x_data << "\t" << y_data << "\t" << endl;
                cout << "Write file success" << endl;
            }
            outfile.close();
        }     
        if(y_button)
        {
            system("gnome-terminal -x bash -c 'source ~/catkin_ws/devel/setup.bash'");
            system("rosrun robot_gps_nav nav_satrt");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GpsNav");
    gps_nav_con gps_nav;
    ros::spin();
    return 0;
}