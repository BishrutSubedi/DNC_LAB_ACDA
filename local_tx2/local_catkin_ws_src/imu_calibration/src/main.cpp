#include <string>
#include <iostream>
#include <time.h>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <sys/time.h>
#include <fstream>
#include <cstring>
#include "R_COM.h"
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"


using namespace std;
using namespace std::chrono;

readcompass R_compass;
float heading;
steady_clock::time_point t1 = steady_clock::now();
steady_clock::time_point t2 = steady_clock::now();
duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

//ros set up
ros::Publisher local_imucal;
std_msgs::Int32 imucal_msg;

ros::Publisher local_pwm;
std_msgs::Int16 pwm_msg;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"local_imucal");    //argc,argv,name of the node
	ros::NodeHandle n;                      //node handle will initialize the node

	local_imucal=n.advertise<std_msgs::Int32>("local_imucal_msg",1); //publish on topic "local_imucal_msg,message to buffer up = 1

    local_pwm=n.advertise<std_msgs::Int16>("local_pwm",1); //publisher "local_pwm", buffer 1 message before throwing away

	sleep(60);                  //this is sleep time in seconds


	t1 = steady_clock::now();
	t2 = steady_clock::now();
	time_span = duration_cast<duration<double>>(t2 - t1); //doesnot return in second, clock cycles may be
	R_compass.setting_compass();//setting compass
    for(int m=1; m<8;m++)
    {
        pwm_msg.data=50;
        local_pwm.publish(pwm_msg);
        sleep(0.2);

    }

	while(time_span.count()<20)
	{
		heading=R_compass.c_heading();
//		cout<<heading<<endl;
		t2 = steady_clock::now();
		time_span = duration_cast<duration<double>>(t2 - t1);
		usleep(100); //sleep microseconds
	}
	
//	R_compass.start_repmotion();
	t1 = steady_clock::now();
	t2 = steady_clock::now();
	time_span = duration_cast<duration<double>>(t2 - t1);
    for(int m=1; m<8;m++)
    {
        pwm_msg.data=-50;
        local_pwm.publish(pwm_msg);
        sleep(0.2);
    }
//	motor_rotate_direction(2);
	while(time_span.count()<20)
	{
		heading=R_compass.c_heading();
//		cout<<heading<<endl;
		t2 = steady_clock::now();
		time_span = duration_cast<duration<double>>(t2 - t1);
		usleep(100);
	}

//	motor_rotate_direction(3);
//	Motor_speed(0);
    for(int m=1; m<8;m++)
    {
        pwm_msg.data=0;
        local_pwm.publish(pwm_msg);
        sleep(0.2);
    }
	int stable=1;
	t1 = steady_clock::now();
	t2 = steady_clock::now();
	time_span = duration_cast<duration<double>>(t2 - t1);
	float old_heading = heading;
	int stop_condition=0;

	while (stable==1)
	{
		t2=steady_clock::now();
		float new_heading=R_compass.c_heading();
		time_span = duration_cast<duration<double>>(t2 - t1);
		if (time_span.count()>20)
		{
//			imucal_msg.data = 0;
//			local_imucal.publish(imucal_msg);
			float error= new_heading-old_heading;
//
//			for(int m=1; m<8;m++)
//			{
//				imucal_msg.data=1;
//				local_imucal.publish(imucal_msg);
//				sleep(1);
//
//			}
//
//			break;





			if (abs(error)<1.5)
			{
				if(stop_condition==2)
				{
					stable=0;
					imucal_msg.data=1;
					for(int m=1; m<8;m++)
					{
						imucal_msg.data=1;
						local_imucal.publish(imucal_msg);
						sleep(1);

					}

					break;
				}
				stop_condition++;


			} else
			{
				old_heading=new_heading;
				stop_condition=0;
			}
			t1=steady_clock::now();

		}
		usleep(500);
//		cout<<new_heading<<endl;
		cout<<"local_imu_stop_condition "<< stop_condition<<endl;

	}




	


//	sleep(2);
//	heading=R_compass.c_heading();
//	float offset = heading-90;
//	ofstream myfile;
//	myfile.open("/home/ubuntu/offset.txt");
//	myfile << offset;
//	myfile.close();
	R_compass.close_runcalibration();


}
