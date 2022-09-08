// ---------------------------------------------------------------------------
//propulsionWithRollCutoff_DISABLED.ino
//Arduino Nano old bootloader
// Use the Arduino serial monitor to get CH1_PWMtime and CH2_PWMtime center and endpoint value to normalize values.
// This a unique application that allows the propeller to get both forward & reverse, the reverse to help with deceleration 
// 
// *** MEASURED DATA ****
// channel 2 is used for speed reference
// CH2_PWMtime center = 1498 to 1506, usec pulsewidth
// CH2_PWMtime full fwd = 1919, usec pulsewidth
// CH2_PWMtime full back = 1133, usec pulsewidth
//
// MPU6050 is a MEM gyro used to detect pitch, roll, and yaw. if roll or pitch exceed 35 degrees, the PitchRoll_OK bit drops out to remove power to the prop
//   MPU 6050 is actually mounted in the later build with a roll value of 60 degrees. During running the roll is giving bad data. Not sure if bad sensor or library timing
//
//This program was first written for a generic L298N H bridge, but is slightly modified to control a DRI0002 DFrobot L298N H bridge
//
// ---------------------------------------------------------------------------

unsigned long CH2_PWMtime; //pulsewidth time of Channel 2 (into pin 4) RC in microseconds, 1000 = full REV (or -90 degress), 1500=zerospeed or 0 degrees, 2000=full FWD (or +90 degrees)
unsigned long timeout = 23000; //timeout of PWM , set timeout for reading a pulsewidth at 15% moree than max expected 20ms (tried 2300 and caused problems)

int CH2_PWMtime_int; // CH1_PWMtime is an unsigned long, convert it to integer

int CH2_PWM_REF_usec ; // magnitude of microseconds pulse width from 1500 usec centerpoint

int CH2_PWM_SpdRef ; // RHS speed reference scaled for 255 = 100%
int CH2_PWMtime_int_LastScan=1500;
int FWD=LOW; // direction of Channel 2 speed ref 
int REV=LOW; // direction of Channel 2 speed ref

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long timer = 0;
float roll=0.0;
float pitch=0.0;
float yaw=0.0;
//**********************Tip Over Protection Adjustments********
int roll_limit_degrees=30;
int pitch_limit_degrees=25;
int PitchRoll_OK = HIGH;
//**********************************************************

void setup() 
      {
      pinMode(2, OUTPUT);  // High when prop is spinning 
      pinMode(4, INPUT); // RC receiver Channel 2 motor speed control input
 
      pinMode(9, OUTPUT); // PWM motor FWD, bit low in reverse DFRobot DRI0002 E1 & E2 pins are PWM control
      pinMode(10, OUTPUT);  // PWM motor REV, bit low in FWD   DFRobot DRI0002 M1 & M2 pins, FWD=0  REV=1  
   //   pinMode(11, OUTPUT); // prop running 
   //   pinMode(12, OUTPUT); // pitch & roll OK relay output, PitchRoll_OK
      pinMode(13, OUTPUT); // MPU 6050 Calibrate process
      Serial.begin(115200); 
      
// MPU6050 setup  - ??  get a calibration cycle on a reboot, and also on entering and exiting the serial monitor
      Wire.begin();
      
      byte status = mpu.begin();
      digitalWrite(13,HIGH); // Arduino Nano D13 LED 
      Serial.print(F("MPU6050 status: "));
      Serial.println(status);
     while(status!=0){ } // stop everything if could not connect to MPU6050
      
      //Serial.println(F("Calculating offsets, do not move MPU6050"));
      delay(2000);
      // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
      mpu.calcOffsets(); // gyro and accelero
      Serial.println("Done!\n");
      digitalWrite(13,LOW); // Arduino Nano D13 LED 
      }

void loop() 

    {  
     // get pitch roll and yaw from the MPU 6050    
      mpu.update();
 
      if((millis()-timer)>100)
        { // print data every 100ms
          Serial.print("X : ");
          Serial.print(mpu.getAngleX());
          roll=(mpu.getAngleX());
        
          Serial.print("\tY : ");
          Serial.print(mpu.getAngleY());
          pitch=(mpu.getAngleY());
          
          //Serial.print("\tZ : ");
          //Serial.println(mpu.getAngleZ());
          //yaw = (mpu.getAngleZ());
        
          timer = millis();  
        }
        
  //      if ((abs(roll)<(roll_limit_degrees))&& ((abs(pitch))<(pitch_limit_degrees)))  
   //         { PitchRoll_OK = HIGH;
             // digitalWrite(12,HIGH);
   //           }
   //         else 
   //         {PitchRoll_OK = LOW;
          //   digitalWrite(12,LOW);
  //           }
          //     Serial.print("PitchRoll_OK = ");
           //    Serial.println(PitchRoll_OK); 
            //   delay(2000);
   
      //   Reads a pulse (either HIGH or LOW) on a pin. For example, if value is HIGH, pulseIn() waits for the pin to go HIGH, starts timing, then waits for the pin to go LOW and stops timing. Returns the length of the pulse in microseconds. Gives up and returns 0 if no pulse starts within a specified time out.
      // Works on pulses from 10 microseconds to 3 minutes in length.
  
   CH2_PWMtime = pulseIn(4, HIGH, timeout); // pulseIn(pin, value, timeout)  value is HIGH or LOW, timeout and value are long int 
         //   Serial.print("CH2 Time: ");
         //   Serial.println(CH2_PWMtime); 
        //    Serial.print("CH2 Spd Ref: ");
        //    Serial.println(CH2_PWM_SpdRef); 

     CH2_PWMtime_int = (int) CH2_PWMtime; //convert unsigned long to int
       if (CH2_PWMtime_int==0) {CH2_PWMtime_int=CH2_PWMtime_int_LastScan;} // 0 is a missed scan , zero directions   
       if (CH2_PWMtime_int>1999) {CH2_PWMtime_int=1999;} // max limit forward
       if (CH2_PWMtime_int<1000) {CH2_PWMtime_int=1000;} // max limit reverse
             
              Serial.print("CH2 Time: ");
              Serial.println(CH2_PWMtime_int); 
               
     /// create fwd and reverse direction bits    
     
  if ((CH2_PWMtime_int - 1500) >= 30 ){ FWD = HIGH; } else {(FWD=LOW);} // 10 allows some deadband switching bridges
  if ((CH2_PWMtime_int - 1500) <= -30 ){ REV = HIGH; } else {(REV=LOW);} // -10 allows some deadband switching bridges
 // if (CH2_PWMtime_int==0) {FWD = LOW; REV =LOW;} // 0 is a missed scan , zero directions
       
                  Serial.print("FWD = ");
                Serial.println(FWD); 
                 Serial.print("REV= ");
                Serial.println(REV);    
                 
 /// create a speed reference in microsends
 CH2_PWM_REF_usec = abs(CH2_PWMtime_int - 1500); // zero to 500usecs = 0 to 100 % speed 
 CH2_PWM_SpdRef = (CH2_PWM_REF_usec/2); // cheap and dirty way to make speed reference, top = 500 microseconds, which corresponds to an analogWrite value 255. divie by 2 makes it just a tad slower.
          //     Serial.print("255 is top speed reference ");
          //    Serial.println(CH2_PWM_SpdRef);    
 
 
 //if (CH2_PWM_SpdRef>200)
 //    { 
 //     CH2_PWM_SpdRef =(CH2_PWM_SpdRef+45); 
//     } // if near top speed kick in that leftover remainder spdref
    // }
            //   Serial.print("255 is top speed reference ");
           //   Serial.println(CH2_PWM_SpdRef);                   

if (CH2_PWM_SpdRef>50)     // relay for prop running LEDs
     { 
      digitalWrite(2,HIGH);
      } 
      else               
       { 
      digitalWrite(2,LOW);
      } 
             
 // output logic for the H Bridge  DFRobot DRI0002            
      analogWrite(9, (CH2_PWM_SpdRef));  // analogWrite values from 0 to 255   //comment out for generic L298N module
              
      if ((FWD==HIGH)&& (PitchRoll_OK==HIGH)) 
      {
//      analogWrite(9, (CH2_PWM_SpdRef));  // analogWrite values from 0 to 255 remove comment out for generic L298N module

      analogWrite(10, 0);
       } 
    else if ((REV==HIGH)&& (PitchRoll_OK==HIGH)) 
      {
 //     analogWrite(9, 0) ;  // analogWrite values from 0 to 255  remove comment out for generic L298N module
 //     analogWrite(10,(CH2_PWM_SpdRef));
       analogWrite(10, 255);
       } 
      else 
      {
      analogWrite(9, 0);  // analogWrite values from 0 to 255
      analogWrite(10, 0) ;
       } 
       delay (50);


//  if ((FWD==HIGH)||(REV==HIGH)) 
//    { digitalWrite (11,HIGH); 
//    }
//  else 
//    {
//      digitalWrite(11,LOW);
//    }

// PitchRoll_OK is defaulted to HIGH , PitchRoll_OK code is commented out
//if (PitchRoll_OK==LOW)
//  { delay(2000);} 

CH2_PWMtime_int_LastScan=CH2_PWMtime_int;
  }
