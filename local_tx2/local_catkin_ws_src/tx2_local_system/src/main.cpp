#include "R_COM.h"
#include "Adafruit_BMP280.h"
#include "Adafruit_BMP280cp.h"
#include "Adafruit_Sensor.h"
#include <string>
#include <iostream>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "sensor_msgs/NavSatFix.h"
#include <ctime>
#include <ratio>
#include <chrono>
#include <thread>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
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
double Kcp = 10.121;
double Kcf = 7.51;//7.5;
double Kep = 1.7321;
double Kef = 1;
double J = 2;
const int sampleTime = 20;
double SampleTimeInSec = ((double)sampleTime)/1000;

// initial heading*******************************

int Azmuth = 183;

//*********************************************

int flag_LQG_motor=0; // decide if system runs motor



int GPS_AZIMUTH =0;// syncing the Azimuth from GPS locations

int last_rssi=-96;

int last_mode=0;
void setting();
void Lqg_Motor() ;
double Get_headingDiff(double Input, double Setpoint);
double LQG_Controller(double error);
readcompass R_compass;
Adafruit_BMP280 baro;

double offsetTol=0.5;

//**************
//******setup timer*******//
steady_clock::time_point rssi_endtime = steady_clock::now();
steady_clock::time_point baro_endtime = steady_clock::now();
steady_clock::time_point stop_endtim = steady_clock::now();
duration<double> time_span;
steady_clock::time_point time_end,LQG_lastTime,now;
steady_clock::time_point rtime_end,rnow,r_lastTime, com_sent_time;
steady_clock::time_point pre_Azmuth_time =steady_clock::now();
//steady_clock::time_point roate1_endtime, roate2_endtime, roate3_endtime, roate_star;

//***************
double best_Azmuth[100]; //select the heading where the rssi is the biggest
int index_Azmuth=0;//the number of best_Azmuth[];
//void get_Azmuth_GPS(); //get the Azmuth with GPS_RS;
void publish_lbarometerinfo(); //publish barometer information;

/*
 * define the vars for gps
 */

float l_lat, l_lon, r_lat,r_lon;
int flag_dual_gps_fix =0;




void temp_remote_gps_callback(sensor_msgs::NavSatFix rgps_msg)
{
    cout<< "********************the heading of antenna "<< Azmuth<<endl;
	if (flag_dual_gps_fix==1)
	{	  r_lat=rgps_msg.latitude;
		  r_lon=rgps_msg.longitude;

		  float LON1 = l_lon* 0.0174532925199433;
		  float LAT1 = l_lat * 0.0174532925199433;
		  float LON2 = r_lon* 0.0174532925199433;
		  float LAT2 = r_lat * 0.0174532925199433;
		  float az = 0;
		  az = atan2(sin(LON2-LON1) * cos(LAT2),cos(LAT1) * sin(LAT2) - sin(LAT1) * cos(LAT2) * cos(LON2-LON1));//calculate the new desired Azimuth
							//// take care of the following sentence
		  az = fmod (az, 9.118906528);
							////
		  az = fmod (az, 6.283185307179586)*(57.2957795131);
		  if (az < 0){
						az = az + 360;
					 }
			Azmuth = az;
			cout<< "the heading of antenna "<< Azmuth<<endl;

		  flag_LQG_motor=1;
		  steady_clock::time_point temp_remote_Azmuth_time =steady_clock::now();
		  while(duration_cast<duration<double>>(steady_clock::now()- temp_remote_Azmuth_time).count()<10)

		  {
		    publish_lbarometerinfo();
//		    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.001));
		    Lqg_Motor();


		  }
		  flag_LQG_motor=0;



	}










}



//local_gps_callback is to receive locations of another GPS module
void remote_gps_callback(sensor_msgs::NavSatFix rgps_msg)
{
	if (flag_dual_gps_fix==1)
		{  r_lat=rgps_msg.latitude;
		  r_lon=rgps_msg.longitude;

		  float LON1 = l_lon* 0.0174532925199433;
		  float LAT1 = l_lat * 0.0174532925199433;
		  float LON2 = r_lon* 0.0174532925199433;
		  float LAT2 = r_lat * 0.0174532925199433;
		  float az = 0;
		  az = atan2(sin(LON2-LON1) * cos(LAT2),cos(LAT1) * sin(LAT2) - sin(LAT1) * cos(LAT2) * cos(LON2-LON1));//calculate the new desired Azimuth
							//// take care of the following sentence
		  az = fmod (az, 9.118906528);
							////
		  az = fmod (az, 6.283185307179586)*(57.2957795131);
		  if (az < 0){
						az = az + 360;
					 }
			Azmuth = az;

		//  if (GPS_AZIMUTH ==1)
		//    {
		//    	Azmuth = az;
		//    	flag_LQG_motor=1;
		//    	pre_Azmuth_time =steady_clock::now();
		//    }
		//  else if (GPS_AZIMUTH==0)
		//  {
		//  	flag_LQG_motor=0;
		//  }
		  flag_LQG_motor=1; //decide if run the LQG to control the heading of antenna


		  }
		//  pre_Azmuth_time =steady_clock::now();
		 // cout<< Azmuth<<endl;
}

void local_gps_callback(sensor_msgs::NavSatFix rgps_msg)
{
	l_lat=rgps_msg.latitude;
	l_lon=rgps_msg.longitude;
	flag_dual_gps_fix =1;

}





//publisher declare
ros::Publisher com_pub; //compass publisher
ros::Publisher baro_pub; // baro_publisher
ros::Publisher ros_serial;
ros::Publisher pwm_pub;
ros::Subscriber local_gps_sub; //
ros::Subscriber remoter_gps_sub;
ros::Subscriber temp_remoter_gps_sub;
ros::Subscriber new_Azimuth;
ros::Subscriber Enable_GPS_Azimuth;
std_msgs::Float64 com_msg; // declare the compass message
std_msgs::Int16   pwm_msg;
std_msgs::String  serial_msg;


int main(int argc, char **argv)
{
	// setting system;
	//sleep(90);

	ros::init(argc,argv,"local_system");
	ros::NodeHandle n;

    //declare the publishers and subscriber
	com_pub=n.advertise<std_msgs::Float64>("local_com",1);
	baro_pub=n.advertise<std_msgs::Float64MultiArray>("local_baro",1);
	pwm_pub = n.advertise<std_msgs::Int16>("local_pwm",1);
	remoter_gps_sub=n.subscribe("remote_gps", 1, remote_gps_callback);
	temp_remoter_gps_sub=n.subscribe("temp_gps_location", 1, temp_remote_gps_callback);

    local_gps_sub  = n.subscribe("local_gps", 1, local_gps_callback);
    ros_serial = n.advertise<std_msgs::String>("local_ros_serial",1);

//	new_Azimuth =n.subscribe("lnewazimuth",1,new_Azimuth_callback);
//	Enable_GPS_Azimuth = n.subscribe("lenablegpsazimuth",1,l_enable_gpsazimuth_callback);


	setting(); // set up the system
	if (argc==2)
	{
		//check if obtain the initial heading from the user
		Azmuth=atoi( argv[1]);
		std::stringstream convert(argv[1]);

		if(!(convert>>Azmuth)) {
			Azmuth = R_compass.c_heading();
		} else{
			flag_LQG_motor=1;
		}


		cout<<"NEW Azimuth: "<<Azmuth<< "   "<<argv[1]<<endl;

	}
//	else{
//		Azmuth=R_compass.c_heading();
//		cout<<"no_arg_Azimuth: "<<Azmuth<< "   "<<Azmuth<<endl;
//	}
	cout<<"First_Azimuth: "<<Azmuth<< "   "<<Azmuth<<endl;
	//find_best_heading();


	// main loop for the directional antenna system
	while(ros::ok())
	{
//		 get_Azmuth_GPS();
		 publish_lbarometerinfo();
		 ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.001));
		 Lqg_Motor();
	}

}



void setting()
{
//	R_compass.calibration_setting();
			R_compass.setting_compass();//setting compass
			usleep(200000);

			//set motor pwm and direction
			//set Barometer
			baro.begin();

			//set RSSI
			//set up GPS
		    com_sent_time=steady_clock::now();
		   	LQG_lastTime=steady_clock::now();
		   	steady_clock::time_point  initial_cl_TIME=steady_clock::now();
		   	serial_msg.data = "start";
		   	for(int i=0;i<5;i++) {
				ros_serial.publish(serial_msg);
				usleep(200000);
			}
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
      if(duration_cast<duration<double>>(steady_clock::now()- com_sent_time).count()>0.1)
      {
    	  com_sent_time=steady_clock::now();
		  com_msg.data=headingRaw;
    	  com_pub.publish(com_msg);
      }
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

void publish_lbarometerinfo()
{
	float l_pressure, l_temp, l_alt;
	if(duration_cast<duration<double>>(steady_clock::now()-baro_endtime).count()>0.5)
	{
		l_pressure = baro.readPressure();
		l_temp = baro.readTemperature();
		l_alt = baro.readAltitude();

		std_msgs::Float64MultiArray baro_msg;
		baro_msg.data.push_back(l_pressure);
		baro_msg.data.push_back(l_temp);
		baro_msg.data.push_back(l_alt);
		baro_pub.publish(baro_msg);

		baro_endtime = steady_clock::now();
	}
}
