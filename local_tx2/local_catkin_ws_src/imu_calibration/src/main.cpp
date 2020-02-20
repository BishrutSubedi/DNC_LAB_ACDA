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
steady_clock::time_point t1 = steady_clock::now(); //gets the current time point from std::chrono 
steady_clock::time_point t2 = steady_clock::now();
duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

//ros set up
ros::Publisher local_imucal;
std_msgs::Int32 imucal_msg;

ros::Publisher local_pwm;
std_msgs::Int16 pwm_msg;

int main(int argc, char **argv)
{
	
	ros::init(argc,argv,"local_imucal");
	ros::NodeHandle n;
	local_imucal=n.advertise<std_msgs::Int32>("local_imucal_msg",1);
    local_pwm=n.advertise<std_msgs::Int16>("local_pwm",1);
	sleep(6);	//sleeps time in second // from <unistd.h> library									

	t1 = steady_clock::now();
	t2 = steady_clock::now();
	time_span = duration_cast<duration<double>>(t2 - t1);
	R_compass.setting_compass();//setting compass

	//start Antenna rotation with motor pwm value is 40
    for(int m=1; m<8;m++)
    {
        pwm_msg.data=40;
        local_pwm.publish(pwm_msg);
        sleep(0.2);
    }

	//WHILE LOOP 1 -------------------STARTS
	while(time_span.count()<10)
	{
		heading=R_compass.c_heading();
		cout<<"Local compass heading is : " << heading << endl;
		t2 = steady_clock::now();
		time_span = duration_cast<duration<double>>(t2 - t1);
		usleep(100);//usleep suspends execution of thread for int usleep(useconds_t usec) usec microsecond <unistd.h>
	}
	//WHILE LOOP 1 -------------------ENDS

//	R_compass.start_repmotion();
	t1 = steady_clock::now();
	t2 = steady_clock::now();
	time_span = duration_cast<duration<double>>(t2 - t1);
    for(int m=1; m<8;m++)
    {
        pwm_msg.data=-40;
        local_pwm.publish(pwm_msg);
        sleep(0.2);		//in seconds
    }

	//WHILE LOOP 2 -------------------STARTS
	while(time_span.count()<10)  //motor_rotate_direction(2);
	{
		heading=R_compass.c_heading();
		cout<<"Local compass heading is : " << heading << endl;
		t2 = steady_clock::now();
		time_span = duration_cast<duration<double>>(t2 - t1);
		usleep(100); //in microsecond
	}
	//WHILE LOOP 2 -------------------ENDS

//	STOPPING THE MOTOR
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

	//WHILE LOOP 3 -------------------STARTS
	while (stable==1)
	{
		t2=steady_clock::now();
		float new_heading=R_compass.c_heading();
		time_span = duration_cast<duration<double>>(t2 - t1);
		float error= new_heading-old_heading; 		// copied here
		if (time_span.count()>20)
		{
			float error= new_heading-old_heading;

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
		usleep(100);
		cout<<"remote_imu_stop_condition " << stop_condition<< endl;
		cout<<"using cout error is " << error << endl;
		cout<<"old_heading is " << old_heading << endl;
		cout<<"new_heading is " << new_heading << endl;
		cout<<"local_imu_stop_condition "<< stop_condition<<endl;

	}
	//WHILE LOOP 3 -------------------ENDS

	R_compass.close_runcalibration();
}
