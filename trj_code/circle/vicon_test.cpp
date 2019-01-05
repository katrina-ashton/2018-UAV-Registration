#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <exception>

#include <ros/ros.h>

#include <GPU_2_Quad_ROS/to_px4.h>
#include <GPU_2_Quad_ROS/from_px4.h>

#define PI 3.14159f

using namespace std;
using namespace ros;




float xic, yic, zic;
float xbc, ybc, zbc;
bool data_recieved;
float yaw_home;
float xvel_sp_speed;

ros::Publisher cmd_pub;

GPU_2_Quad_ROS::from_px4 	quad_data;
GPU_2_Quad_ROS::to_px4		cmd_data;	



void quadCallback(const GPU_2_Quad_ROS::from_px4 quad_data_msg)
{
	quad_data = quad_data_msg;
	if (!data_recieved)
	{
		yaw_home = quad_data.yaw;
		data_recieved = true;
	}

}



int move_fb(int forward)
{
	float xyp = 	1.0f;
	float zp =	1.0f;

	//xic =	-xyp*(quad_data.x_pos-(-1.2f));
	if (xvel_sp_speed > 1.5f) { xvel_sp_speed = 1.5f; }

	if (forward==1)
	{
		xic =	xvel_sp_speed;
		if (quad_data.x_pos>1.35){ forward = -1; }
	}
	else
	{
		xic =	-xvel_sp_speed;
		if (quad_data.x_pos<-1.35){  forward = 1; } //xvel_sp_speed += 0.25;
	}
	



	yic =	-xyp*(quad_data.y_pos-(0.0f));
	zic =	-zp*(quad_data.z_pos-(-1.5f));

	float yaw = 		quad_data.yaw;
	float ang_diff =	yaw-atan2f(yic,xic);
	float vxy_mag =		sqrtf(xic*xic+yic*yic);

	xbc =	vxy_mag*cosf(-ang_diff);
	ybc =	vxy_mag*sinf(-ang_diff);
	zbc =	zic;

	cmd_data.x_vel_des	=	xbc;
	cmd_data.y_vel_des	=	ybc;
	cmd_data.z_vel_des	=	zbc;
	cmd_data.mode =			7.5f;
	cmd_data.yaw_des 	=	0;

	return forward;
}







void hold_1p2m()
{
	float xyp = 	0.7f;
	float zp =	0.5f;

	xic =	-xyp*(quad_data.x_pos-(-1.0f));
	yic =	-xyp*(quad_data.y_pos-(0.0f));
	zic =	-zp*(quad_data.z_pos-(-1.5f));

	float yaw = 		quad_data.yaw;
	float ang_diff =	yaw-atan2f(yic,xic);
	float vxy_mag =		sqrtf(xic*xic+yic*yic);

	xbc =	vxy_mag*cosf(-ang_diff);
	ybc =	vxy_mag*sinf(-ang_diff);
	zbc =	zic;

	cmd_data.x_vel_des	=	xbc;
	cmd_data.y_vel_des	=	ybc;
	cmd_data.z_vel_des	=	zbc;
	cmd_data.mode =			7.5f;
	cmd_data.yaw_des 	=	0;
}

void circle(double t)
{
	float xyp = 	1.0f;
	float zp =	1.0f;



	float circle_ang = 	t/40.0f*2*PI;
	float x_des = 		1.0f*cosf(circle_ang+PI);
	float y_des = 		1.0f*sinf(circle_ang+PI);

	float yaw_set =		circle_ang;


	xic =	-xyp*(quad_data.x_pos-(x_des));
	yic =	-xyp*(quad_data.y_pos-(y_des));
	zic =	-zp*(quad_data.z_pos-(-1.5f));

	float yaw = 		quad_data.yaw;
	float ang_diff =	yaw-atan2f(yic,xic);
	float vxy_mag =		sqrtf(xic*xic+yic*yic);

	xbc =	vxy_mag*cosf(-ang_diff);
	ybc =	vxy_mag*sinf(-ang_diff);
	zbc =	zic;

	cmd_data.x_vel_des	=	xbc;
	cmd_data.y_vel_des	=	ybc;
	cmd_data.z_vel_des	=	zbc;
	cmd_data.mode 		=	7.5f;
	cmd_data.yaw_des 	=	yaw_set;		
}

void hold_origin()
{
	float xyp = 	1.0f;
	float zp =	1.0f;

	xic =	-xyp*(quad_data.x_pos-(-0.0f));
	yic =	-xyp*(quad_data.y_pos-(-0.0f));
	zic =	-zp*(quad_data.z_pos-(-1.5f));

	float yaw = 		quad_data.yaw;
	float ang_diff =	yaw-atan2f(yic,xic);
	float vxy_mag =		sqrtf(xic*xic+yic*yic);

	xbc =	vxy_mag*cosf(-ang_diff);
	ybc =	vxy_mag*sinf(-ang_diff);
	zbc =	zic;

	cmd_data.x_vel_des	=	xbc;
	cmd_data.y_vel_des	=	ybc;
	cmd_data.z_vel_des	=	zbc;
	cmd_data.mode =			7.5f;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "vicon_test");
	ros::NodeHandle nm;
	ros::Rate pub_rate(60);
	
	double start_time = (double)ros::Time::now().toSec();

	data_recieved = false;
	yaw_home = 0;

	cmd_pub = nm.advertise<GPU_2_Quad_ROS::to_px4>("/px4_to", 2);
	ros::Subscriber quad_data_sub = nm.subscribe("/px4_from", 2, quadCallback);

	xic = 0; yic = 0; zic = 0.2f;
	xbc = 0; ybc = 0; zbc = 0.2f;

	
	// sleep for 1s to init subscriber
	usleep(1000000);

	xvel_sp_speed = 0.5f;

	int cmd = 0;
	int forward = 1;
	double time_cmd = (double)ros::Time::now().toSec();

	ROS_INFO("Started - launch!");
	double t;






	while(ros::ok())
	{
		t = (double)ros::Time::now().toSec()-time_cmd;
        cmd_data.yaw_des = yaw_home;
        

		if (cmd==0)
		{
			hold_1p2m();
			if(t > 20.0)
			{
				ROS_INFO("Circle mode!");
                time_cmd = (double)ros::Time::now().toSec();
				cmd++;
			}

		}
		else if (cmd==1)
		{
			circle(t);
			if(t > 80.0)
			{
				ROS_INFO("Hold mode at x=1.2m!  Will then land after 10s!");
				time_cmd = (double)ros::Time::now().toSec();
				cmd++;
			}

		}
		else if (cmd==2)
		{
			hold_1p2m();
			if(t > 10.0)
			{
				//ROS_INFO("Land!");
				cmd_data.z_vel_des = 0.3;
			}

		}	
	    if (cmd_data.z_vel_des>0.3)     { cmd_data.z_vel_des=0.3; }
        if (cmd_data.z_vel_des<-0.3)    { cmd_data.z_vel_des=-0.3; }
	    if (cmd_data.x_vel_des>1.0)     { cmd_data.x_vel_des=1.0; }
        if (cmd_data.x_vel_des<-1.0)    { cmd_data.x_vel_des=-1.0; }
	    if (cmd_data.y_vel_des>1.0)     { cmd_data.y_vel_des=1.0; }
        if (cmd_data.y_vel_des<-1.0)    { cmd_data.y_vel_des=-1.0; }
		//cmd_data.yaw_des = yaw_home;
        cmd_data.mode =			50.0f;
        //ROS_INFO("vz desired: %0.3f", cmd_data.z_vel_des);
        cmd_data.header.stamp =  ros::Time::now();
		cmd_pub.publish(cmd_data);

		ros::spinOnce();
		pub_rate.sleep();
	}



}



