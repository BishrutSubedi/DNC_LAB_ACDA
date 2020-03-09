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
ofstream myfile;

I2CDevice *xsens =new I2CDevice (6,0x6b);
class readcompass{

public:
	void setting_compass();
	float c_heading(void);
	void close_runcalibration();
	void  start_repmotion();
	void  stop_repmotion();
	int i=1;


private:

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
{			myfile.open("/home/ubuntu/offset.txt", ios::in | ios::binary);
			uint8_t Address=0x6b;
			uint8_t go_to_config[] = {0x30,0x00,0xD1};
			uint8_t set_output_modle[] = {0xC0, 0x04, 0x20, 0x34, 0x00, 0x64,0x85};
			uint8_t go_to_measure[] = {0x10, 0x00, 0xF1};
			uint8_t SetSyncSettings[] = {0x2C,0x0C,0x08,0x06,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x00,0x00,0xB0};
			uint8_t enable_run_calibration[] = {0x48, 0x08, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x31};

			uint8_t disable_run_calibration[] = {0x48, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x31};
			uint8_t setup_location_Arlington[] = {0x6E, 0x18, 0x40, 0x40, 0x5D, 0xD6, 0x94, 0xCC, 0xAB, 0x3F, 0xC0, 0x58, 0x47, 0x3A, 0x79, 0x78, 0x91, 0xE2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81 };
			uint8_t restoreFactoryDef[] = {0x0E, 0x00, 0xF3};
			uint8_t no_rotation[] = {0x22,0x00,0xDF};
			uint8_t setup_enable_ASH[]={ 0x48, 0x08, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0xA1};
			uint8_t setup_disenable_ASH[]={ 0x48, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0xA1};
			uint8_t setup_gen_filter[]={  0x64, 0x02, 0x00, 0x32, 0x69};
			uint8_t start_rep_motion[]={  0x74, 0x01, 0x00, 0x8C};
			uint8_t set_output_resdata[]={0xC0, 0x14, 0x20, 0x10, 0x00, 0x32, 0x20, 0x34, 0x00, 0x32, 0x40, 0x20, 0x00, 0x32, 0x80, 0x20, 0x00, 0x32, 0xC0, 0x20, 0x00, 0x32, 0xCF};
			uint8_t reqoutputconfiguration[]= {0xC0,0x00,0x41};
//			uint8_t restoreFactoryDef[]= {0x0E, 0x00, 0xF3};


			xsens->writeRegisterblock(0x03, go_to_config,sizeof(go_to_config));

		    usleep(300000);

			xsens->writeRegisterblock(0x03, setup_gen_filter, sizeof(setup_gen_filter));
			usleep(300000);

			usleep(300000);


			xsens->writeRegisterblock(0x03, restoreFactoryDef,sizeof(restoreFactoryDef));
			usleep(300000);
			xsens->writeRegisterblock(0x03, set_output_resdata,sizeof(set_output_resdata));
			usleep(300000);
//			xsens->writeRegisterblock(0x03, reqoutputconfiguration,sizeof(reqoutputconfiguration));
//			usleep(300000);


//		    xsens->writeRegisterblock(0x03, set_output_modle, sizeof(set_output_modle));
//			usleep(300000);
//			xsens->writeRegisterblock(0x03, setup_location_Arlington, sizeof(setup_location_Arlington));
//			usleep(300000);
		    xsens->writeRegisterblock(0x03, setup_disenable_ASH,sizeof(setup_disenable_ASH));
			usleep(300000);
		    xsens->writeRegisterblock(0x03, enable_run_calibration,sizeof(enable_run_calibration));
			usleep(300000);



//			while (i<1000) {
//
//				c_heading();
//				usleep(100);
//				i++;
//			}
//			cout<<"start motion"<<endl;



//				   	xsens->writeRegisterblock(0x03, no_rotation,sizeof(no_rotation));
//				   			   	   sleep(1);

		  	xsens->writeRegisterblock(0x03, go_to_measure,sizeof(go_to_measure));
			usleep(300000);

//			xsens->writeRegisterblock(0x03, start_rep_motion,sizeof(start_rep_motion));
//			usleep(1000);
		  	cout << "enable calibration ......";
//			uint8_t start_rep_motion[]={  0x74, 0x01, 0x00, 0x8C};


}

void readcompass:: close_runcalibration(void)
{

				uint8_t Address=0x6b;
				uint8_t go_to_config[] = {0x30,0x00,0xD1};
				uint8_t set_output_modle[] = {0xC0, 0x04, 0x20, 0x34, 0x00, 0x64,0x85};
				uint8_t go_to_measure[] = {0x10, 0x00, 0xF1};
				uint8_t SetSyncSettings[] = {0x2C,0x0C,0x08,0x06,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x0A,0x00,0x00,0xB0};
				uint8_t enable_run_calibration[] = {0x48, 0x08, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x31};
				uint8_t disable_run_calibration[] = {0x48, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x31};
				uint8_t setup_location_Arlington[] = {0x6E, 0x18, 0x40, 0x40, 0x5D, 0xD6, 0x94, 0xCC, 0xAB, 0x3F, 0xC0, 0x58, 0x47, 0x3A, 0x79, 0x78, 0x91, 0xE2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81 };
				uint8_t no_rotation[] = {0x22,0x00,0xDF};
				uint8_t reqoutputconfiguration[] = {0xC0,0x00,0x41};
				uint8_t setup_enable_ASH[]={ 0x48, 0x08, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0xA1};
				uint8_t setup_vru_filter[]={  0x64, 0x02, 0x00, 0x36, 0x65};
				uint8_t start_rep_motion[]={  0x74, 0x01, 0x00, 0x8C};
				uint8_t set_output_resdata[]={0xC0, 0x14, 0x20, 0x10, 0x00, 0x32, 0x20, 0x34, 0x00, 0x32, 0x40, 0x20, 0x00, 0x32, 0x80, 0x20, 0x00, 0x32, 0xC0, 0x20, 0x00, 0x32, 0xCF};

				xsens->writeRegisterblock(0x03, go_to_config,sizeof(go_to_config));
			    	usleep(300000);

			    xsens->writeRegisterblock(0x03, disable_run_calibration,sizeof(disable_run_calibration));
					usleep(300000);



				//
//				xsens->writeRegisterblock(0x03, set_output_resdata,sizeof(set_output_resdata));
//				sleep(1);
//			    xsens->writeRegisterblock(0x03, setup_enable_ASH,sizeof(setup_enable_ASH));
//					usleep(500000);
//
//			    xsens->writeRegisterblock(0x03, no_rotation,sizeof(no_rotation));
//					usleep(500000);
//				xsens->writeRegisterblock(0x03, setup_vru_filter,sizeof(setup_vru_filter));
//				usleep(300000);
			    xsens->writeRegisterblock(0x03, go_to_measure,sizeof(go_to_measure));
					usleep(300000);
			  	cout <<"close calibration ......"<<endl;
				myfile.close();





}



float readcompass::c_heading(void)
{

			  uint16_t notificationsize, measurementsize;
			  unsigned char *char1=xsens->readRegisters(4,0x04);

			  notificationsize=*(char1) | (*(char1+1) << 8);
			 // cout<<"notificationsize  "<<notificationsize<<endl;
			  measurementsize=*(char1+2) | (*(char1+3) << 8);
			  //cout<<"measurementsize  "<<measurementsize<<endl;

//			  if(notificationsize)
//			  {
//				  unsigned char * notice_data=xsens->readRegisters(notificationsize, 0x05);
//
////				  for(int j=0; j < notificationsize; j++)
////				   			  	  	  {
////				   			  		  	 // cout<<*(notice_data+i)<<endl;
////					  	  	  	  	  	  printf("%x ",*(notice_data+j));
////										  myfile<<*(notice_data+j)<< " ";
////				   			  	  	  }
//////				  myfile.write(notice_data, notificationsize);
////				  myfile<<"output configuration "<<endl;
////				  cout<<"output configuration "<< i<<endl;
//
//
//
//			  }




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
//		  			  			yaw.asByte[3]=*(data+13);
//		  			  			yaw.asByte[2]=*(data+14);
//		  			  			yaw.asByte[1]=*(data+15);
//		  			  			yaw.asByte[0]=*(data+16);

				   yaw.asByte[3]=*(data+32);
				   yaw.asByte[2]=*(data+33);
				   yaw.asByte[1]=*(data+34);
				   yaw.asByte[0]=*(data+35);

//		  		  for(int k=0; k < measurementsize; k++)
//		  			  	  	  {
//		  			  		  	    // cout<< int(*(measure_data+i)) << " ";
////		  			  	  	  	  	  printf("%x ",*(data+k));
//					  *(data+k);
//
//		  			  	  	  }
////		  		  cout<<"message end"<<endl;//	  		 cout<<endl;
//
		  	  }

		  	   if (yaw.asFloat<0)
		  	   {

		  		   yaw.asFloat=360+yaw.asFloat;

		  	   }

		  	   return yaw.asFloat;
//	return 0;

}

void readcompass::start_repmotion() {

	uint8_t start_rep_motion[]={  0x74, 0x01, 0x01, 0x8B};
	uint8_t get_rep_motion_state[]= {0x74, 0x01, 0x03, 0x89};
	uint8_t save_parameter_to_xsens[]={0x74,0x01,0x02,0x8A};
//	xsens->writeRegisterblock(0x03, save_parameter_to_xsens,sizeof(save_parameter_to_xsens));
//	usleep(2000);
	xsens->writeRegisterblock(0x03, get_rep_motion_state,sizeof(get_rep_motion_state));
	usleep(1000);


//	i=0;
//	while (i<100) {
//
//		c_heading();
//		usleep(100);
//		i++;
//	}

	cout<<" stop motion"<<endl;
	xsens->writeRegisterblock(0x03, start_rep_motion,sizeof(start_rep_motion));
	usleep(2000);



}
