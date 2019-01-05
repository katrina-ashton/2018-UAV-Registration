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




void turnr(double turn_time, double t, float x_set, float y_set, float start_yaw)
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

void turnl(double turn_time, double t, float x_set, float y_set, float start_yaw)
{
    float yaw_set =	start_yaw - (PI/2)*(t/turn_time);
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

        yaw_set = (PI*3)/2;
    }

    else if (side==2) {
        float x_set = -1.0f+t*speed;

	    xic =	-xyp*(quad_data.x_pos-(x_set));
	    yic =	-xyp*(quad_data.y_pos-(-0.5));

        yaw_set = 0;
    }


    else if (side==3) {
        float y_set = -0.5+t*speed;

	    xic =	-xyp*(quad_data.x_pos-(-0.5f));
	    yic =	-xyp*(quad_data.y_pos-(y_set));

        yaw_set = PI/2;
    }

    else if (side==4) {
        float x_set = -0.5f+t*speed;

	    xic =	-xyp*(quad_data.x_pos-(x_set));
	    yic =	-xyp*(quad_data.y_pos-(0.5f));

        yaw_set = 0;
    }


    else if (side==5) {
        float y_set = 0.5-t*speed;

	    xic =	-xyp*(quad_data.x_pos-(0.0f));
	    yic =	-xyp*(quad_data.y_pos-(y_set));

        yaw_set = (3*PI)/2;
    }

    else if (side==6) {
        float x_set = 0.0f+t*speed;

	    xic =	-xyp*(quad_data.x_pos-(x_set));
	    yic =	-xyp*(quad_data.y_pos-(-0.5));

        yaw_set = 0;
    }

    else if (side==7) {
        float y_set = -0.5+t*speed;

	    xic =	-xyp*(quad_data.x_pos-(0.5f));
	    yic =	-xyp*(quad_data.y_pos-(y_set));

        yaw_set = PI/2;
    }

    else if (side==8) {
        float x_set = 0.5f+t*speed;

	    xic =	-xyp*(quad_data.x_pos-(x_set));
	    yic =	-xyp*(quad_data.y_pos-(0.5f));

        yaw_set = 0;
    }

    else if (side==9) {
        float y_set = 0.5-t*speed;

	    xic =	-xyp*(quad_data.x_pos-(1.0f));
	    yic =	-xyp*(quad_data.y_pos-(y_set));

        yaw_set = (3*PI/2);
    }

    else if (side==10) {
        float x_set = 1.0f-t*speed;

	    xic =	-xyp*(quad_data.x_pos-(x_set));
	    yic =	-xyp*(quad_data.y_pos-(-0.5f));

        yaw_set = PI;
    }

    else if (side==11) {
        float y_set = -0.5+t*speed;

	    xic =	-xyp*(quad_data.x_pos-(0.5f));
	    yic =	-xyp*(quad_data.y_pos-(y_set));

        yaw_set = PI/2;
    }

    else if (side==12) {
        float x_set = 0.5f-t*speed;

	    xic =	-xyp*(quad_data.x_pos-(x_set));
	    yic =	-xyp*(quad_data.y_pos-(0.5f));

        yaw_set = PI;
    }

    else if (side==13) {
        float y_set = 0.5-t*speed;

	    xic =	-xyp*(quad_data.x_pos-(0.0f));
	    yic =	-xyp*(quad_data.y_pos-(y_set));

        yaw_set = (3*PI)/2;
    }

    else if (side==14) {
        float x_set = 0.0f-t*speed;

	    xic =	-xyp*(quad_data.x_pos-(x_set));
	    yic =	-xyp*(quad_data.y_pos-(-0.5f));

        yaw_set = PI;
    }

    else if (side==15) {
        float y_set = -0.5+t*speed;

	    xic =	-xyp*(quad_data.x_pos-(-0.5f));
	    yic =	-xyp*(quad_data.y_pos-(y_set));

        yaw_set = PI/2;
    }

    else if (side==16) {
        float x_set = -0.5f-t*speed;

	    xic =	-xyp*(quad_data.x_pos-(x_set));
	    yic =	-xyp*(quad_data.y_pos-(0.5f));

        yaw_set = PI;
    }

    else if (side==17) {
        float y_set = 0.5-t*speed;

	    xic =	-xyp*(quad_data.x_pos-(-1.0f));
	    yic =	-xyp*(quad_data.y_pos-(y_set));

        yaw_set = (3*PI)/2;
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
				ROS_INFO("Lawn mower mode!");
                time_cmd = (double)ros::Time::now().toSec();
				cmd++;
			}

		}
		else if (cmd==1)
		{
            if (cmd_sqr==0){
                fwd(t, 1);
                if(t > 5.0) {
				ROS_INFO("Initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==1){
                turnr(turn_time, t, -1.0f, -0.5f, 0.0f);
                ROS_INFO("t: %f", t);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==2){
                fwd(t, 2);
                if(t > 5.0) {
				ROS_INFO("Initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==3){
                turnr(turn_time, t, 0.5f, -0.5f, PI/2);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==4){
                fwd(t, 3);
                if(t > 10.0) {
				ROS_INFO("Initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==5){
                turnl(turn_time, t, 0.5f, 0.5f, PI);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==6){
                fwd(t, 4);
                if(t > 5.0) {
				ROS_INFO("Initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==7){
                turnl(turn_time, t, 0.0f, 0.5f, (PI*3)/2);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==8){
                fwd(t, 5);
                if(t > 10.0) {
			    ROS_INFO("Initialing turn");
                time_cmd = (double)ros::Time::now().toSec();
                }
                }                   
            }
            else if (cmd_sqr==9){
                turnr(turn_time, t, 0.0f, -0.5f, 0.0f);
                ROS_INFO("t: %f", t);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==10){
                fwd(t, 6);
                if(t > 5.0) {
				ROS_INFO("Initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==11){
                turnr(turn_time, t, -0.5f, -0.5f, PI/2);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==12){
                fwd(t, 7);
                if(t > 10.0) {
				ROS_INFO("Initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==13){
                turnl(turn_time, t, -0.5f, 0.5f, PI);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==14){
                fwd(t, 8);
                if(t > 5.0) {
				ROS_INFO("Initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==15){
                turnl(turn_time, t, -1.0f, 0.5f, (PI*3)/2);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==16){
                fwd(t, 9);
                if(t > 10.0) {
			    ROS_INFO("Initialing turn");
                time_cmd = (double)ros::Time::now().toSec();
                }
                }                   
            }


            else if (cmd_sqr==17){
                turnl(turn_time, t, 1.0f, -0.5f, PI);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==18){
                fwd(t, 10);
                if(t > 5.0) {
				ROS_INFO("Initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==19){
                turnl(turn_time, t, 0.5f, -0.5f, (PI*3)/2);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==20){
                fwd(t, 11);
                if(t > 10.0) {
			    ROS_INFO("Initialing turn");
                time_cmd = (double)ros::Time::now().toSec();
                }
                }                   
            }
            else if (cmd_sqr==21){
                turnr(turn_time, t, 0.5f, 0.5f, PI);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==22){
                fwd(t, 12);
                if(t > 5.0) {
				ROS_INFO("Initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==23){
                turnr(turn_time, t, 0.0f, 0.5f, (PI*3)/2);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==24){
                fwd(t, 13);
                if(t > 10.0) {
			    ROS_INFO("Initialing turn");
                time_cmd = (double)ros::Time::now().toSec();
                }
                }                   
            }
            else if (cmd_sqr==25){
                turnl(turn_time, t, 0.0f, -0.5f, PI);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==26){
                fwd(t, 14);
                if(t > 5.0) {
				ROS_INFO("Initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==27){
                turnl(turn_time, t, -0.5f, -0.5f, (PI*3)/2);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==28){
                fwd(t, 15);
                if(t > 10.0) {
			    ROS_INFO("Initialing turn");
                time_cmd = (double)ros::Time::now().toSec();
                }
                }                   
            }
            else if (cmd_sqr==29){
                turnr(turn_time, t, -0.5f, 0.5f, PI);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==30){
                fwd(t, 16);
                if(t > 5.0) {
				ROS_INFO("Initiating turn");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==31){
                turnr(turn_time, t, -1.0f, 0.5f, (PI*3)/2);
                if(t >turn_time) {
				ROS_INFO("Turn finished");
                time_cmd = (double)ros::Time::now().toSec();
				cmd_sqr++;                    
                    }
            }
            else if (cmd_sqr==32){
                fwd(t, 17);
                if(t > 5.0) {
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



