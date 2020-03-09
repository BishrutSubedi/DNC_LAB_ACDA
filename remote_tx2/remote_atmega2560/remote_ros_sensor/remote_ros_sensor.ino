
  /*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>

#include <sensor_msgs/NavSatFix.h>
// set up GPS
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
Adafruit_GPS GPS(&Serial3);

//set up ROS 
ros::NodeHandle  nh;
std_msgs::String str_msg;

std_msgs::Int8   rssi_val;
ros::Publisher remote_rssi("remote_rssi", &rssi_val);


sensor_msgs::NavSatFix gps_val;
ros::Publisher remote_gps("remote_gps", &gps_val);

// set up GPS if using interrupt
#define GPSECHO  true
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

/***
 * set up pwm
 */
int Motor_cw = 44;
int Motor_ccw = 46;
int Motor_PWM_temp, Motor_PWM=0;

void motor_pwm_Cb( const std_msgs::Int16& local_pwm_msg){
   Motor_PWM_temp=local_pwm_msg.data;
    Motor_PWM = Motor_PWM_temp;
       

        
}
ros::Subscriber<std_msgs::Int16> pwm_sub("/remote_pwm", motor_pwm_Cb);


void setup()
{
  pinMode(Motor_cw, OUTPUT);   
  pinMode(Motor_ccw, OUTPUT);
  nh.initNode();
  nh.subscribe(pwm_sub);
  delay(200);
// rssi
  Serial2.begin(9600); //Rssi
  
  delay(500);
  nh.advertise(remote_rssi);
  delay(500);
  nh.advertise(remote_gps);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
//  GPS.sendCommand(PMTK_SET_BAUD_57600);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 1 Hz update rate
  #ifdef __arm__
    usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
  #else
    useInterrupt(true);
  #endif
  
    delay(1000);

}

// set up for GPS
#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif //#ifdef__AVR__
///////////////////////////

// definte the Global parameters
uint32_t gps_timer = millis();
uint32_t rssi_timer = millis();


void loop()
{
  pub_gps();
  pub_rssi();
  nh.spinOnce();
  motor_run();
  delay(20);
}

void pub_gps()
{
  

  if (gps_timer > millis())  gps_timer = millis(); 
  if (millis() - gps_timer > 250) { 
         if (! usingInterrupt) {
            char c = GPS.read();
          }
        //  delay(200);
          
          // if a sentence is received, we can check the checksum, parse it...
          if (GPS.newNMEAreceived()) {
          
            if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
              return;  // 
          }

    gps_timer = millis(); // reset the timer

  
//      gps_val.latitude = 32.732464;
//      gps_val.longitude = -97.113751;
//      gps_val.status.status =1;
//      remote_gps.publish( &gps_val );

    
    if (GPS.fix) {
      gps_val.latitude = GPS.latitudeDegrees;
      gps_val.longitude = GPS.longitudeDegrees;
      gps_val.status.status =1;
      remote_gps.publish( &gps_val );
      
////      Serial.print(GPS.latitude, 6); //
//      Serial.print(GPS.latitudeDegrees,8);
//      Serial.print(", "); 
////      Serial.print(GPS.longitude, 6); //
//      Serial.println(GPS.longitudeDegrees,8);
    }
  }  
}

void pub_rssi()
{
  if (rssi_timer > millis())  rssi_timer = millis(); 
  if (millis() - rssi_timer > 300) { 
        rssi_timer = millis(); 
        String Rev_Rssi = "";
        if (Serial2.available() > 0)
        {
          while (Serial2.available())
          {
            char Inchar_Rssi = Serial2.read();
            Rev_Rssi += Inchar_Rssi;
            delay(1);
            } 
            rssi_val.data = Rev_Rssi.toInt();
            remote_rssi.publish( &rssi_val );
            
        

            
        }
  }    
}

void motor_run()
{
   if(Motor_PWM > 0)
         {
          analogWrite(Motor_cw, Motor_PWM);
          digitalWrite(Motor_ccw, LOW);
          }
         else if(Motor_PWM < 0 )
               {
                 digitalWrite(Motor_cw,LOW);
                 analogWrite(Motor_ccw, abs(Motor_PWM));
               }
         else if(Motor_PWM == 0 )
          {
                 digitalWrite(Motor_ccw,LOW);
                 digitalWrite(Motor_cw,LOW);              
          }
  }
