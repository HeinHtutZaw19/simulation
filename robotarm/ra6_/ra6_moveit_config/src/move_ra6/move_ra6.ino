
#include <ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <stdlib.h>
#include <MeMegaPi.h>
#include <MeEncoderOnBoard.h>


ros::NodeHandle nh;

float angle[6] = {0,0,0,0,0,0};

Servo dof4,dof5,dof6;

std_msgs::Float64 mydata;

void cmd_cb(const sensor_msgs::JointState& cmd_arm)
{
  angle[0] = cmd_arm.position[0];
  angle[1] = cmd_arm.position[1];
  angle[2] = cmd_arm.position[2];
  angle[3] = cmd_arm.position[3];
  angle[4] = cmd_arm.position[4];
  angle[5] = cmd_arm.position[5];
  mydata.data = angle[0];
 }
MeEncoderOnBoard Encoder_1(SLOT1);
    MeEncoderOnBoard Encoder_2(SLOT2);
    MeEncoderOnBoard Encoder_3(SLOT3);
    
    void isr_process_encoder1(void)
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
    void isr_process_encoder2(void)
    {
      if(digitalRead(Encoder_2.getPortB()) == 0)
      {
        Encoder_2.pulsePosMinus();
      }
      else
      {
        Encoder_2.pulsePosPlus();
      }
    }
    void isr_process_encoder3(void)
    {
      if(digitalRead(Encoder_3.getPortB()) == 0)
      {
        Encoder_3.pulsePosMinus();
      }
      else
      {
        Encoder_3.pulsePosPlus();
      }
    }

ros::Subscriber<sensor_msgs::JointState> sub("/joint_states", cmd_cb);
ros::Publisher chatter("chatter", &mydata);

void setup()
{    
      //Set PWM 8KHz
      TCCR1A = _BV(WGM10);
      TCCR1B = _BV(CS11) | _BV(WGM12);
      TCCR2A = _BV(WGM21) | _BV(WGM20);
      TCCR2B = _BV(CS21);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  attachInterrupt(Encoder_3.getIntNum(), isr_process_encoder3, RISING);
  
  Encoder_1.setPulse(8);
  Encoder_1.setRatio(75*9);
  Encoder_1.setPosPid(17.5,0,1.8);
  Encoder_1.setSpeedPid(0.18,0,0);
  Encoder_2.setPulse(8);
  Encoder_2.setRatio(75*9);
  Encoder_2.setPosPid(21, 0, 1.8);
  Encoder_2.setSpeedPid(0.18,0,0);
  
  Encoder_3.setPulse(8);
  Encoder_3.setRatio(46*7);
  Encoder_3.setPosPid(12.5,0,1.7);
  Encoder_3.setSpeedPid(0.18,0,0);
  
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  dof4.attach(60);
  
  dof5.attach(62);
  dof6.attach(61);
}

void loop()
{
  Encoder_1.moveTo(-angle[0] * 180 / M_PI);//- ka bel bt lel tr
  Encoder_2.moveTo(angle[1] * 180 / M_PI);// + ka tat tr
  Encoder_3.moveTo(-angle[2] * 180 / M_PI);//- ka tat tr
  dof4.write(angle[3] * 180 / M_PI + 90);
  dof5.write(-angle[4] * 180 / M_PI + 90) ;
  dof6.write(angle[5] * 180 / M_PI + 90);
  chatter.publish(&mydata);
  nh.spinOnce();
  delay(1);
  Encoder_1.loop();
  Encoder_2.loop();
  Encoder_3.loop();
  
}
