#include <ros.h>
#include <sensor_msgs/JointState.h>
#include "MeMegaPi.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeEncoderOnBoard.h>

ros::NodeHandle nh;
MeEncoderOnBoard Encoder_1 (SLOT1);
/*float encoder_vel[6];
float GR1 = 1/9;
float GR2 = 1/9;
float GR3 = 1/7;
MeEncoderNew Encoder_2 (0x09, SLOT2);
MeEncoderNew Encoder_3 (0x09, SLOT3);

void messageCb(const sensor_msgs::JointState &msg)
{
  for(int i=0;i<6;i++){
    encoder_vel[i] = msg.velocity[i];
  }

}
 
ros::Subscriber<sensor_msgs::JointState> sub("/joint_states", &messageCb);
*/
void _isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
   {
     Encoder_1.pulsePosMinus();
   }
     else
   {
     Encoder_1.pulsePosPlus();;
   }
 }
    
void setup()
{
  
  //Set Pwm 8KHz
  TCCR1A = _BV(WGM10);//PIN12
    TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);
    
    TCCR2A = _BV(WGM21) | _BV(WGM20);//PIN8
    TCCR2B = _BV(CS22);
    
    TCCR3A = _BV(WGM30);//PIN9
    TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);
    
    TCCR4A = _BV(WGM40);//PIN5
    TCCR4B = _BV(CS41) | _BV(CS40) | _BV(WGM42);

  attachInterrupt(Encoder_1.getIntNum(), _isr_process_encoder1, RISING);
  //Encoder_2.begin();
  //Encoder_3.begin();
  //nh.initNode();
  //nh.subscribe(sub);
}
 
float deg(float rad){
  return rad * 180 / M_PI;  
}
void loop()
{
  Encoder_1.setTarPWM(200);
  //Encoder_2.runSpeed(deg(encoder_vel[1] * GR2));
 // Encoder_3.runSpeed(deg(encoder_vel[2] * GR3));
  Encoder_1.loop();
  //nh.spinOnce();
  delay(200);
}

void _delay(float seconds){
    long endTime = millis() + seconds * 1000;
    while(millis() < endTime)Encoder_1.loop();
}
