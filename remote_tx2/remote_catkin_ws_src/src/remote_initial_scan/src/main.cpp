#include "R_COM.h"
#include <string>
#include <iostream>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <ctime>
#include <ratio>
#include <chrono>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "sensor_msgs/NavSatFix.h"
#include <ros/callback_queue.h>




using namespace std;
using namespace std::chrono;
double Setpoint=0, Input, Output;
//using namespace exploringBB;
// Set parameters for the LQG cntroller
double lastW = 0;
double lastE = 0;
//double Kcp = 4.4721;
//double Kcf = 3.1535;
double Kcp = 5.121;
double Kcf = 4.51;//7.5;
double Kep = 1.7321;
double Kef = 1;
double J = 2;
const int sampleTime = 20;
double SampleTimeInSec = ((double)sampleTime)/1000;

// initial heading*******************************

int Azmuth = 183;

//*********************************************




int flag_LQG_motor=1; // decide if system runs motor

		

int GPS_AZIMUTH =0;// syncing the Azimuth from GPS locations

int last_rssi=-96;

int last_mode=0;
void setting();
void Lqg_Motor() ;
double Get_headingDiff(double Input, double Setpoint);
double LQG_Controller(double error);


readcompass R_compass;


double offsetTol=0.5;

//**************
//******setup timer*******//
steady_clock::time_point rssi_endtime = steady_clock::now();
steady_clock::time_point r_GPS_RS_endtime = steady_clock::now();
steady_clock::time_point stop_endtim = steady_clock::now();
duration<double> time_span;
steady_clock::time_point time_end,LQG_lastTime,now;
steady_clock::time_point rtime_end,rnow,r_lastTime, com_sent_time;

steady_clock::time_point pre_Azmuth_time =steady_clock::now();


//duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

steady_clock::time_point roate1_endtime, roate2_endtime, roate3_endtime, roate_star;


//***************
void check_rssi_com();
void find_best_heading();
void get_Azmuth(int data_rssi,float data_com);//calculate the Azmuth which has the biggest rssi
double average_rssi[]={-96,-96,-96,-96,-96};//initial the rssi value to average the rssi in 5 times
double last_Azmuth=0; //judge if the antenna rotate one circle
double best_Azmuth[100]; //select the heading where the rssi is the biggest
int index_Azmuth=0;//the number of best_Azmuth[];
void get_Azmuth_GPS(); //get the Azmuth with GPS_RS;
int remote_rssi_value=-96;

void remote_rssi_callback(std_msgs::Int8 rssi_msg)
{
	remote_rssi_value=rssi_msg.data;

}


ros::Publisher ros_serial;
ros::Publisher pwm_pub;
ros::Subscriber new_Azimuth;
ros::Subscriber remote_rssi;
std_msgs::Int16   pwm_msg;



/*
 * define the vars for gps
 */

int main(int argc, char **argv)
{
	ros::init(argc,argv,"remote_initial_scan");
	ros::NodeHandle n;
	pwm_pub = n.advertise<std_msgs::Int16>("remote_pwm",1);
	remote_rssi=n.subscribe("remote_rssi",1,remote_rssi_callback);

	setting();

	roate2_endtime=steady_clock::now();
	while(duration_cast<duration<double>>(steady_clock::now()- roate2_endtime).count()<10)
	{

		 Lqg_Motor();
	}

}



void setting()
{

			R_compass.setting_compass();//setting compass
			usleep(200000);
			//set motor pwm and direction
			
		    com_sent_time=steady_clock::now();

		   	LQG_lastTime=steady_clock::now();
		   	steady_clock::time_point  initial_cl_TIME=steady_clock::now();
			find_best_heading();

}


void Lqg_Motor()   //motor control
  {

    if (duration_cast<duration<double>>(steady_clock::now()- LQG_lastTime).count()>0.02 ) //decide the loop time 50 hz
    {
      float headingRaw =R_compass.c_heading();   //Get current compass
//
		// subscribe com
      //***************
      //**publish compass heading
      //***************
	/*      
	if(duration_cast<duration<double>>(steady_clock::now()- com_sent_time).count()>0.1)
      {
    	  com_sent_time=steady_clock::now();
		  com_msg.data=headingRaw;
    	  com_pub.publish(com_msg);
      }
	*/
      //**************************

      float targetHeading;

      //judge if running LQG
      //***************
      if(flag_LQG_motor==1)
      {
    	  targetHeading = Azmuth;
      }
      else
      {
    	  targetHeading = headingRaw;
      }
      //***************

      Input=headingRaw;
      Setpoint = targetHeading;

      float headingDiff = Get_headingDiff(Input, Setpoint);// get the small difference between the current heading and desired heading

      //if heading difference is less than offsettol, the motor will keep quite.
      if (abs(headingDiff) < offsetTol){
    	  pwm_msg.data=0;
    	  pwm_pub.publish(pwm_msg);
        }
       else
       {
        Output = LQG_Controller(headingDiff);
		   pwm_msg.data=-Output;
		   pwm_pub.publish(pwm_msg);

        }
      LQG_lastTime=steady_clock::now();
    }
  }

double Get_headingDiff(double Input, double Setpoint){
    double error1 = Setpoint - Input;
    double error2 = Setpoint + 360 - Input;
    double error3 = Setpoint - Input - 360;
    double error;
    if (abs(error1)  <= abs(error2) && abs(error1) <= abs(error3))
    {
      error = error1;
      }
    else if (abs(error2) <= abs(error1) && abs(error2) <= abs(error3))
    {
      error = error2;
      }
    else
    {
      error = error3;
    }
    return error;
  }


  //LQG controller
double LQG_Controller(double error)
  {
    Output = -Kcp * error - Kcf * lastW;
    if (Output > 254){
      Output = 254;
      }
    else if (Output < -254){
        Output = -254;
      }
    lastE = lastW * SampleTimeInSec + Kep * SampleTimeInSec * (error - lastE);
    lastW = Output * SampleTimeInSec /J + Kef * SampleTimeInSec * (error - lastE);
    return Output/254*100;
  }


void check_rssi_com()
{
	int D;
	float H;


	if (duration_cast<duration<double>>(steady_clock::now()- rssi_endtime).count()>0.22)
	{
		//cout<<duration_cast<duration<double>>(steady_clock::now()- rssi_endtime).count()<<endl;
		D=remote_rssi_value;
		//cout<< D <<endl;
		if (D >(-97) && D<0)
		{
//			cout<< D <<endl;
		H=R_compass.c_heading();
		get_Azmuth(D,H);
		}

		rssi_endtime=steady_clock::now();
		//publish RSSI
	}
}








void find_best_heading()
{
			float initialAzimuth=R_compass.c_heading();
			float currentAzimuth;
//			float upAzimuth= initialAzimuth+20;
//			float downAzimuth = initialAzimuth-20;
			cout<< initialAzimuth<<endl;


			
			pwm_msg.data=-45;
		   	pwm_pub.publish(pwm_msg);
            usleep(20000);
            pwm_msg.data=-45;
            pwm_pub.publish(pwm_msg);
            usleep(20000);
            pwm_msg.data=-45;
            pwm_pub.publish(pwm_msg);
            usleep(50000);
            pwm_msg.data=-45;
            pwm_pub.publish(pwm_msg);
            usleep(50000);


			roate2_endtime=steady_clock::now();
			while(duration_cast<duration<double>>(steady_clock::now()- roate2_endtime).count()<60)
			{
				check_rssi_com();
                ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.001));
				currentAzimuth=R_compass.c_heading();
                ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.001));
//				cout<<currentAzimuth<<endl;
//				float error = Get_headingDiff(currentAzimuth,initialAzimuth);
//
//				if (error<-40 )
//				{
//					motor_rotate_direction(2);
//				}
//				if (error>40 )
//				{
//					motor_rotate_direction(1);
////					rotate_diretiona=1;
//				}
//				if(duration_cast<duration<double>>(steady_clock::now()- r_GPS_RS_endtime).count()>0.22)
//				{	  GPS_RS.GPSdata();
//					r_GPS_RS_endtime = steady_clock::now();
//					if(GPS_RS.fix==1)
//					{
//						cout<<"obtain GPS location"<<endl;
//						break;
//
//					}
//
//				}

			}


}



void get_Azmuth(int data_rssi,float data_com)
{
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.001));
	if (data_rssi>=last_rssi )//campare the rssi with the last rssi and get the Azmuth
		{
			Azmuth=data_com;
			last_rssi=data_rssi;
			cout<<"remote best Azmuth: "<< Azmuth<< " rssi: "<<data_rssi<<endl;
			/*
			cout<<"best Azmuth: "<< Azmuth<< " rssi: "<<data_rssi<<endl;
			if (last_mode==1)
			{
								best_Azmuth[index_Azmuth]=Azmuth;
								//cout<<Azmuth<<endl;
								index_Azmuth++;
			}

			*/
		}
	last_Azmuth=data_com;
}





