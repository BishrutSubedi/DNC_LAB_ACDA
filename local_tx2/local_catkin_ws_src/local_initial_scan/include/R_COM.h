#include <iostream>
#include <errno.h>
#include "i2cdevice.h"
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <ctime>
#include <ratio>
#include <chrono>
#include <sys/time.h>
#include <fstream>
using namespace std;
using namespace exploringBB;
using namespace std::chrono;

#define OPCODE_PIPESTATUS 0x04
#define OPCODE_NOTIFICATIONPIPE 0x05
#define OPCODE_MEASUREMENTPIPE 0x06
void readedate(void);
void handl_data(unsigned char * data);

//I2CDevice *xsens;
I2CDevice *xsens =new I2CDevice (6,0x6b);
class readcompass{

public:
	void setting_compass();
	float c_heading(void);
	void calibration_setting();
	float offset;

private:
	uint8_t Address=0x6b;
	union
	{
	  uint8_t asByte[4];
	  float asFloat;
	} roll;

	union
	{
	  uint8_t asByte[4];
	  float asFloat;
	} pitch;

	union
	{
	  uint8_t asByte[4];
	  float asFloat;
	} yaw;
};

void readcompass::setting_compass(void)
{

			uint8_t go_to_config[]={0x30,0x00,0xD1};
			uint8_t set_output_modle[]={0xC0, 0x04, 0x20, 0x34, 0x00, 0x64,0x85};
			uint8_t go_to_measure[]={0x10, 0x00, 0xF1};
			uint8_t SetSyncSettings[] = {0xFA,0xFF,0x2C,0x0C,0x08,0x06,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x00,0x00,0xB0};
			uint8_t enable_run_calibration[] = {0x48, 0x08, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x31};
			uint8_t enable_run_ahs[] = {0x48, 0x08, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0xA1};
			uint8_t disable_run_ahs[] = {0x48, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0xA1};
			uint8_t disable_run_calibration[] = {0x48, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x31};
			uint8_t setup_location_Arlington[] = {0x6E, 0x18, 0x40, 0x40, 0x5D, 0xD6, 0x94, 0xCC, 0xAB, 0x3F, 0xC0, 0x58, 0x47, 0x3A, 0x79, 0x78, 0x91, 0xE2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81 };
			uint8_t no_rotation[] = {0x22, 0x00, 0xDF};
//			ifstream myfile("/home/ubuntu/offset.txt");
//									if (myfile.is_open())
//									{
//										myfile >> offset;
//										cout<<offset<<endl;
//										myfile.close();
//									}
//
			xsens->writeRegisterblock(0x03, go_to_config,sizeof(go_to_config));
			usleep(500000);
			xsens->writeRegisterblock(0x03, disable_run_calibration,sizeof(disable_run_calibration));
			sleep(1);
//			xsens->writeRegisterblock(0x03, enable_run_ahs,sizeof(enable_run_ahs));
//			usleep(500000);
//
//			xsens->writeRegisterblock(0x03, no_rotation,sizeof(no_rotation));
//			usleep(500000);
			xsens->writeRegisterblock(0x03, set_output_modle,sizeof(set_output_modle));
			usleep(500000);
			xsens->writeRegisterblock(0x03, go_to_measure,sizeof(go_to_measure));
			sleep(1);



}

void readcompass::calibration_setting()
{
			uint8_t go_to_config[]={0x30,0x00,0xD1};
			uint8_t set_output_modle[]={0xC0, 0x04, 0x20, 0x34, 0x00, 0x64,0x85};
			uint8_t go_to_measure[]={0x10, 0x00, 0xF1};
			uint8_t SetSyncSettings[] = {0xFA,0xFF,0x2C,0x0C,0x08,0x06,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x00,0x00,0xB0};
			uint8_t enable_run_calibration[] = {0x48, 0x08, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x31};
			uint8_t enable_run_ahs[] = {0x48, 0x08, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0xA1};
			uint8_t disable_run_ahs[] = {0x48, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0xA1};
			uint8_t disable_run_calibration[] = {0x48, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x31};
			uint8_t setup_location_Arlington[] = {0x6E, 0x18, 0x40, 0x40, 0x5D, 0xD6, 0x94, 0xCC, 0xAB, 0x3F, 0xC0, 0x58, 0x47, 0x3A, 0x79, 0x78, 0x91, 0xE2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81 };
			uint8_t no_rotation[] = {0x22, 0x00, 0xDF};
//
//
//			xsens->writeRegisterblock(0x03, go_to_config,sizeof(go_to_config));
//			usleep(200000);
//			xsens->writeRegisterblock(0x03, disable_run_ahs,sizeof(disable_run_ahs));
//			usleep(200000);
//			xsens->writeRegisterblock(0x03, enable_run_calibration,sizeof(enable_run_calibration));
//			usleep(200000);
//			xsens->writeRegisterblock(0x03, set_output_modle,sizeof(set_output_modle));
//			usleep(200000);
//			xsens->writeRegisterblock(0x03, go_to_measure,sizeof(go_to_measure));
//			usleep(200000);







}


float readcompass::c_heading(void)
{

			  uint16_t notificationsize, measurementsize;
			  unsigned char *char1=xsens->readRegisters(4,0x04);

			  notificationsize=*(char1) | (*(char1+1) << 8);
			 // cout<<"notificationsize  "<<notificationsize<<endl;
			  measurementsize=*(char1+2) | (*(char1+3) << 8);
			  //cout<<"measurementsize  "<<measurementsize<<endl;
			  /*
			  if(notificationsize)
			  {
				  unsigned char * notice_data=xsens->readRegisters(notificationsize, 0x05);

				  for(int i=0; i < notificationsize; i++)
				   			  	  	  {
				   			  		  	 // cout<<*(notice_data+i)<<endl;
					  	  	  	  	  	  printf("%x ",*(notice_data+i));
				   			  	  	  }
				  cout<<endl;


			  }
			  */



		  	   if(measurementsize)
		  	  {
		  		  unsigned char * data=xsens->readRegisters(measurementsize, 0x06);
//		  		  	  	  	  	roll.asByte[3]=*(data+5);
//		  			  			roll.asByte[2]=*(data+6);
//		  			  			roll.asByte[1]=*(data+7);
//		  			  			roll.asByte[0]=*(data+8);
//		  			  			pitch.asByte[3]=*(data+9);
//		  			  			pitch.asByte[2]=*(data+10);
//		  			  			pitch.asByte[1]=*(data+11);
//		  			  			pitch.asByte[0]=*(data+12);
		  			  			yaw.asByte[3]=*(data+13);
		  			  			yaw.asByte[2]=*(data+14);
		  			  			yaw.asByte[1]=*(data+15);
		  			  			yaw.asByte[0]=*(data+16);



	//	  		  for(int i=0; i < measurementsize; i++)
	//	  			  	  	  {
	//	  			  		  	    // cout<< int(*(measure_data+i)) << " ";
	//	  			  	  	  	  	  //printf("%x ",*(measure_data+i));
	//	  			  	  	  }
	//	  		  cout<<"end"<<endl;
	//	  		 cout<<endl;

		  	  }
		  	 float heading = yaw.asFloat;
		  	 		  	   heading= heading;

		  	 		  	   if (heading<0)
		  	 		  	   {

		  	 		  		   heading=heading+360;
		  	 		  	   }

		  	 //		  	   if (yaw.asFloat<0)
		  	 //		  	   {
		  	 //
		  	 //		  		   yaw.asFloat=360+yaw.asFloat;
		  	 //
		  	 //		  	   }

		  	 //		  	   return yaw.asFloat;
//		  	 		    cout<< heading<<endl;
//						heading = heading+offset;

		  	 			  if (heading < 0)
		  	 				  	 		  	  {
		  	 				  	 		  		  heading=heading+360;

		  	 				  	 		  	  }
		  	 			else if (heading >=360 )
		  	 				  	 		  	  {
		  	 				  	 		  		  heading = heading -360;

		  	 				  	 		  	  }
		  	 		  	   return heading;

}





