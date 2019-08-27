
/* Copyright : 2019 Henan Institute of Science and Technology Robotics Research Institute
 * Author : Li Yuejun
 * Description :A gps_nav node file.
 * History :
 *     201908 : Initial this cpp file.
 */

#include "ros/ros.h"
#include <stdio.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <algorithm>
#include "robot_gps/RobotGPS.h"
#include <bobac_compass/Compass.h>
#include <string>
#include <Eigen/Dense>
#include <vector>
#include <fstream>  //文件流库函数

using namespace std;
using namespace Eigen;

#define PI 3.1415
#define MAX 100000000
#define k 6
#define c 12
#define p 2


//定义全局变量
double q0;
double q1;
double q2;
double q3;
//double delta;
double x;
double y;
double z;
double a;
double b;
double aaa;
double bbb;
double attractive;
double xattractive;
double yattractive;
double conpulsive;
double xconpulsive;
double yconpulsive;
double attractangle;
double conangle;
double xsum;
double ysum;
double angle;
double alpha;
double newspeed;
double newturnrate;
double Fsum;
double max_speed=0.35;
double max_turning_speed_rad = 1.2;

geometry_msgs::Twist car_vel;
ros::Publisher m_vel_pub;
ifstream infile;   //输入流
using namespace std;

//定义回调函数
void callback(const robot_gps::RobotGPS::ConstPtr &msg)
{
	x = msg->deltax;
	y = msg->deltay;
	y=-y;
	x=x+0.3;
	y=y-0.15;
	/*MatrixXd ww1(2,2);
	ww1<<0,1,1,0;
	MatrixXd ww2(2,1);
	ww2<<x,y;
	MatrixXd ww3(2,1);
	ww3=ww1*wm2;
	x=ww3(0,0);
	y=ww3(1,0);*/
	//cout<<x<<";"<<y<<endl; 
}

void callback1(const bobac_compass::Compass::ConstPtr &msg1)
{
	alpha = msg1->north_err;
	alpha=(alpha/180)*PI;
	//cout<<"alpha"<<delta<<endl; 
}

//定义取值的上限和下限
double limit(double n, double min, double max)
{
	if(n < min)
		return min;
	else if(n > max)
		return max;
	else 
	    return n;
}

//计算雷达探测的最小距离并计算所对应激光的索引
double get_min_distance_in_range(sensor_msgs::LaserScan laser, float min_degree, float max_degree,int &num)
{
	float limit_degrees = abs(laser.angle_min * 180 / PI);
	
	if(min_degree > max_degree)
	{
		float aux = max_degree;
		max_degree = min_degree;
		min_degree = aux;
	}	
	min_degree += limit_degrees;
	max_degree += limit_degrees;	
	float min_rad = min_degree / 180 * PI;
	float max_rad = max_degree / 180 * PI;
	int min_offset = (int)(min_rad / laser.angle_increment);
	int max_offset = (int)(max_rad / laser.angle_increment);
	float min = MAX;
	for(int i=min_offset; i<max_offset; i++)
	{
			if(laser.ranges[i] < min)
		    {
				min = laser.ranges[i];
				num = i;
			}
				
	}
	//num =find(laser.ranges.begin(), laser.ranges.end(), min) - laser.ranges.begin();
	return min;	
}

//见车车前方式是否有障碍物
bool check_front_obstacle(sensor_msgs::LaserScan laser, float x_region, float y_region)
{
	int offset = 0;
	float min = MAX;
	float x_min,y_min = 0.0;
	float angle, angle_min = 0.0;
	for(int i=offset; i<laser.ranges.size()-offset; i++)
	{
		if (laser.ranges[i] > 0.05 && laser.ranges[i] < 1.0) 
		{
			angle = laser.angle_min + i * laser.angle_increment;
			double x = laser.ranges[i] * sin(angle);
			double y = laser.ranges[i] * cos(angle);
			if ((fabs(x) < x_region) && (y < y_region)) 
			{
				return true;
			}
		} 
	}
	return false;
}

//合力计算
void wm(sensor_msgs::LaserScan laser,double x1,double y1,double x2,double y2)
{
        alpha=alpha+PI;
	while(alpha>PI)
	{
		alpha=alpha-2*PI;
	}
	alpha=-alpha;
	cout<<"pianhangjiao:"<<alpha<<endl;
	cout<<x<<","<<y<<endl;
	double a1=atan2(y1,x1);
	double a2=atan2(y2,x2);
	double a3=cos(alpha);
	double a4=sin(alpha);
	MatrixXd temp1(3,3);
	temp1<<a3,-a4,0,a4,a3,0,0,0,1;
	MatrixXd temp2(3,3);
	temp2=temp1.inverse();
	MatrixXd temp3(3,1);
	temp3<<x1,y1,a1;
	MatrixXd temp4(3,3);
	temp4<<1,0,0,0,1,0,0,0,1;
	MatrixXd temp5(3,1);
	temp5<<x2,y2,a2;
	MatrixXd temp6(3,1);
	temp6=temp2*(temp3-temp4*temp5);
	aaa=temp6(0,0);
	bbb=temp6(1,0);
	double rEndx=fabs(aaa);
	double rEndy=fabs(bbb);
	double rEnd=sqrt((rEndx * rEndx)+(rEndy * rEndy));
	if(rEndx==0)
	{
		if(bbb>y)
		{
			attractangle=PI / 2;
		}
		else
		{
			attractangle=-(PI / 2);
		}
	}
	else
	{
		attractangle=atan2((bbb),(aaa));
	}
//计算斥力
	/*int num;
	double aa=get_min_distance_in_range(laser, -90, 90, num);
	//cout<<num<<endl;
	int num1= num;
	if (aa>p)
	{
		xconpulsive=0;
		yconpulsive=0;
		conpulsive=0;
	}
	else
	{
		if(aa>(p/2))
		{
			conpulsive=c * ((1/aa)-(1/p)) * (1/(aa * aa));
            conangle=laser.angle_min+num1 * laser.angle_increment;
			if (conangle>PI)
		    {	
			    conangle-=2*PI;
			}
		    else if (conangle<-PI)
			{
			    conangle+=2*PI;
			}
			xconpulsive=-conpulsive * cos(conangle);
			yconpulsive=-conpulsive * sin(conangle);
		}
		if(aa>0.6&&aa<(p/2))
		{
			conpulsive=c * c * ((1/aa)-(1/p)) * (1/(aa * aa));
            conangle=laser.angle_min+num1 * laser.angle_increment;
			if (conangle>PI)
		    {
			    conangle-=2*PI;
			}
		    else if (conangle<-PI)
			{
			    conangle+=2*PI;
			}
			xconpulsive=-conpulsive * cos(conangle);
			yconpulsive=-conpulsive * sin(conangle);
		}
	}*/
xconpulsive=0;
		yconpulsive=0;
		conpulsive=0;
//计算引力
	attractive=k * rEnd;
	xattractive=k * rEnd * cos(attractangle);
	yattractive=k * rEnd * sin(attractangle);

	//计算合力
	xsum=xattractive+xconpulsive;
	ysum=yattractive+yconpulsive;
	angle=atan2(ysum,xsum);
	Fsum=xsum/cos(angle);
	Fsum=limit(Fsum,0,max_speed);
	newspeed=Fsum;
	newspeed = limit(Fsum, -max_speed, max_speed);
	newturnrate=angle;
	newturnrate = limit(newturnrate, -max_turning_speed_rad, max_turning_speed_rad);
	if(sqrt((x1-x)*(x1-x)+(y1-y)*(y1-y))<1.3)
	{
		car_vel.linear.x = 0.0;
		car_vel.linear.y = 0.0;
		car_vel.linear.z = 0.0;
		car_vel.angular.x = 0.0;
		car_vel.angular.y = 0.0;
		car_vel.angular.z = 0.0;
	}
	else
	{
		car_vel.linear.x = newspeed;
		car_vel.linear.y = 0.0;
		car_vel.linear.z = 0.0;
		car_vel.angular.x = 0.0;
		car_vel.angular.y = 0.0;
		car_vel.angular.z = newturnrate;
	}
	m_vel_pub.publish(car_vel);
}

//删除数组
void shuzuqianyi(int num,double *data)
{
	int i = 0;
	while (data[i + num])
	{
		data[i]= data[i + num];
		data[i+1] = data[i + num+1];
		i++;
	}
}

//double d[2][2]={1,2,3,2};
int i=0;

bool wm1(double x1,double y1,double x2,double y2)
{
	if(sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))<1.3)
	{
		return true;
	}
}

//回调函数
void autonomous_behave(const sensor_msgs::LaserScan &laser)
{
/*	double d[60]={-11.414,-16.378,-8.614,-18.617,-6.141,-20.274,-3.708,-25.383,
                  -1.315,-25.768,1.032,-25.846,4.260,-25.842,6.528,-25.814,
                  8.983,-25.744,11.081,-25.714,13.403,-25.680,15.581,-25.660,
                  18.308,-25.779,20.942,-25.736,24.948,-24.940,21.651,-17.609,
                  21.236,-13.766,19.811,-12.247,14.672,-11.690,11.744,-11.290,
                  9.400,-11.296,7.283,-11.286,4.834,-11.318,2.846,-11.357,
                  0.641,-11.357,-1.610,-11.345,-4.045,-12.518,-5.996,-13.874,
                  -8.110,-15.073,-10.386,-16.400};
*/
    int datalen=0;
    double d[100];
	int i=0;
    infile.open("GPS_data.txt", ios::in); 
    while(!infile.eof()&& infile.good()) infile>>d[datalen++];
    for(int i=0;i<datalen-1;i++)
    {
        cout<<d[i]<<" ";
    }
    cout<<" "<<endl;
    infile.close();   

    if(wm1(d[i],d[i+1],x,y))
	{
		if(i<datalen-3)
		{
			i=i+2;
		}
		car_vel.linear.x = 0.0;
		car_vel.linear.y = 0.0;
		car_vel.linear.z = 0.0;
		car_vel.angular.x = 0.0;
		car_vel.angular.y = 0.0;
		car_vel.angular.z = 0.0;
		m_vel_pub.publish(car_vel);

	}
	else
	{
        wm(laser,d[i],d[i+1],x,y);
	    cout<<"jiaosudu:"<<car_vel.angular.z<<endl;	
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_start");
    ros::NodeHandle nh;  
    m_vel_pub = nh.advertise<geometry_msgs::Twist>("car_vel", 1);
	ros::Subscriber laser_sub = nh.subscribe("scan", 1, autonomous_behave);
	ros::Subscriber acml_sub = nh.subscribe<robot_gps::RobotGPS>("robot/gps", 1, callback );
	ros::Subscriber acml_sub1 = nh.subscribe<bobac_compass::Compass>("/compass", 1, callback1 );
    ros::spin();
}