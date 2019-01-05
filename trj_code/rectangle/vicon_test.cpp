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




void turn(double turn_time, double t, float x_set, float y_set, float start_yaw)
{
    float yaw_set =	start_yaw + (PI/2)*(t/turn_time);
	float xyp = 1.0f;
	float zp =	1.0f;

    ROS_INFO("yaw: %f", yaw_set);

	xic =	-xyp*(quad_data.x_pos-(x_set));
	yic =	-xyp*(quad_data.y_pos-(y_set));
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

void fwd(double t, int side)
{
    float speed = 0.1f;
	float xyp = 1.0f;
	float zp =	1.0f;

    float yaw_set;

    if (side==1) {
        float y_set = -t*speed;

	    xic =	-xyp*(quad_data.x_pos-(-1.0f));
	    yic =	-xyp*(quad_data.y_pos-(y_set));

        yaw_set = 0.0f;
    }

    else if (side==2) {
        float x_set = -1.0f+t*speed;

	    xic =	-xyp*(quad_data.x_pos-(x_set));
	    yic =	-xyp*(quad_data.y_pos-(-0.5));

        yaw_set = PI/2;
    }


    else if (side==3) {
        float y_set = -0.5+t*speed;

	    xic =	-xyp*(quad_data.x_pos-(1.0f));
	    yic =	-xyp*(quad_data.y_pos-(y_set));

        yaw_set = PI;
    }

    else if (side==4) {
        float x_set = 1.0f-t*speed;

	    xic =	-xyp*(quad_data.x_pos-(x_set));
	    yic =	-xyp*(quad_data.y_pos-(0.5f));

        yaw_set = (PI*3)/2;
    }

    else if (side==5) {
        float y_set = 0.5f-t*speed;

	    xic =	-xyp*(quad_data.x_pos-(-1.0f));
	    yic =	-xyp*(quad_data.y_pos-(y_set));

        yaw_set = 0.0f;
    }

    else { //shouldn't happen
        xic = 0.0f;
        yic = 0.0f;
        yaw_set = quad_data.yaw;
    }

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
    cmd_data.yaw_des 	=	yaw_set;

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
    int cmd_sqr = 0;
	double time_cmd = (double)ros::Time::now().toSec();

    ROS_INFO("Change 2");
	ROS_INFO("Started - launch!");
	double t;
    int num_laps = 0;
    double turn_time = 5.0f;


	while(ros::ok())
	{
		t = ((double)ros::Time::now().toSec())-time_cmd;
        cmd_data.yaw_des = yaw_home;
        

		if (cmd==0)
		{
			hold_1p2m();
			if(t > 20.0)
			{
				ROS_INFO("Square mode!");
                time_cmd = (double)ros::Time::now().toSec();
				cmd++;
			}

		}
		else if (cmd==1)
		{
            if (cmd_sqr==0){
                fwd(t, 1);
                if(t > 5.0) {
				ROS_INFO("First half side done, initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==1){
                turn(turn_time, t, -1.0f, -0.5f, 0.0f);
                ROS_INFO("t: %f", t);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==2){
                fwd(t, 2);
                if(t > 20.0) {
				ROS_INFO("Second side done, initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==3){
                turn(turn_time, t, 1.0f, -0.5f, PI/2);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==4){
                fwd(t, 3);
                if(t > 10.0) {
				ROS_INFO("Third side done, initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==5){
                turn(turn_time, t, 1.0f, 0.5f, PI);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==6){
                fwd(t, 4);
                if(t > 20.0) {
				ROS_INFO("Fourth side done, initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==7){
                turn(turn_time, t, -1.0f, 0.5f, (PI*3)/2);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==8){
                fwd(t, 5);
                if(t > 5.0) {
                num_laps++;
                if (num_laps==1) {
				    ROS_INFO("Second half side done, begining new lap");
                    time_cmd = (double)ros::Time::now().toSec();
				    cmd_sqr=0;
                }
                }                   
            }
			if(num_laps==2)
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



