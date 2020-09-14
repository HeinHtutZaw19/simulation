
#include <ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <stdlib.h>
#include <MeMegaPi.h>
#include <MeEncoderOnBoard.h>

ros::NodeHandle nh;
 MeEncoderOnBoard Encoder_1(SLOT1);
 std_msgs::Float64 mydata;

float angle[6] = {0,0,0,0,0,0};

void cmd_cb(const sensor_msgs::JointState& cmd_arm)
{
  angle[0] = cmd_arm.position[0];
  angle[1] = cmd_arm.position[1];
  angle[2] = cmd_arm.position[2];
  angle[3] = cmd_arm.position[3];
  angle[4] = cmd_arm.position[4];
  angle[5] = cmd_arm.position[5];
 }

ros::Subscriber<sensor_msgs::JointState> sub("/move_group/fake_controller_joint_states", cmd_cb);
ros::Publisher chatter("chatter", &mydata);

void _isr_process_encoder1(void) {
  if(digitalRead(Encoder_1.getPortB()) == 0){
    Encoder_1.pulsePosMinus();
  }else{
    Encoder_1.pulsePosPlus();
  }
}

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  attachInterrupt(Encoder_1.getIntNum(), _isr_process_encoder1, RISING);
  Encoder_1.setPulse(8);
  Encoder_1.setRatio(75);
  Encoder_1.setPosPid(17.5,0,1.8);
  Encoder_1.setSpeedPid(0.18,0,0);
}

void loop()
{
  
  mydata.data = angle[1];
  Encoder_1.moveTo(mydata.data * 180 / M_PI);
  chatter.publish( &mydata );
  nh.spinOnce();
  delay(1);
}

void _loop(){
  Encoder_1.loop();
}
