#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <RMCS2303drive.h>
#include <SwitchMonitor.h>


//For Arduino Uno Software serial needs to be used as there is only one hardware serial port and its connected to USB-Serial. 
//   Drive to Arduino UNO/Nano connections
//   GND         -      GND
//   RXD         -      D3 5
//   TXD         -      D2 4
//
//For arduino mega and other arduinos with multiple hardware serial port, any port other than 0 can be selected to connect the drive.
//
//   Drive to Arduino Mega2560 connections
//   GND         -      GND
//   RXD         -      Tx1/Tx2/Tx3
//   TXD         -      Rx1/Rx2/Rx3




ros::NodeHandle nh;

byte slave_id_right=2;
int INP_CONTROL_MODE=256;           
int PP_gain=16;
int PI_gain=16;
int VF_gain=16;
int LPR=2262;
int acceleration=30;
int speed=0;

int sp = 5;
std_msgs::Int64 rightposition;

long int Current_position_right;

ros::Publisher right_motor_pub("/rightmotor/feedback", &rightposition);

RMCS2303 rmcs;

void subscribe_right_command(const std_msgs::Float64& msg){
  if(msg.data > 0){
    rmcs.Speed(slave_id_right, msg.data); 
    rmcs.Enable_Digital_Mode(slave_id_right,0); 
  }
  else if(msg.data < 0){
    rmcs.Speed(slave_id_right, abs(msg.data)); 
    rmcs.Enable_Digital_Mode(slave_id_right,1);
  }
  // if(msg.data == 0){
  //   if(previousrightspeed > 0){
  //     rmcs.Disable_Digital_Mode(slave_id_right,0);
  //   }
  //   else if(previousrightspeed < 0){
  //     rmcs.Disable_Digital_Mode(slave_id_right,1);
  //   }
  // }
  // previousrightspeed = msg.data;  
  else{
    rmcs.Disable_Digital_Mode(slave_id_right,0);
    rmcs.Disable_Digital_Mode(slave_id_right,1);
  }
}

void subscribe_software_estop(const std_msgs::Float64& msg){
  if(msg.data == 1){
    // Software E-Stop is active
    rmcs.STOP(slave_id_right); 
  }
}

ros::Subscriber<std_msgs::Float64> right_motor_sub("/rightmotor/command", subscribe_right_command);

ros::Subscriber<std_msgs::Float64> software_estop_sub("/es_status/software", subscribe_software_estop);


void setup() {
  Serial.begin(115200);
  motor_driver_init();
  right_init();
  Serial.println("\nRight Done");
  
  nh.initNode();
  nh.advertise(right_motor_pub);
  nh.subscribe(right_motor_sub);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  publish_right_position();
  nh.spinOnce();

}

void motor_driver_init(){
  rmcs.Serial_selection(0);
  rmcs.begin(&Serial1,9600); 
}

void right_init(){
  rmcs.WRITE_PARAMETER(slave_id_right,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);
  rmcs.READ_PARAMETER(slave_id_right);
//  rmcs.Speed(slave_id_right, 20); 
//  rmcs.Enable_Digital_Mode(slave_id_right,0); 
  rmcs.Disable_Digital_Mode(slave_id_right,0);
}

void publish_right_position(){
  Current_position_right=rmcs.Position_Feedback(slave_id_right); 
//  Serial.print("r ");
//  Serial.println(Current_position_right);
  rightposition.data = Current_position_right;
  right_motor_pub.publish(&rightposition);
}