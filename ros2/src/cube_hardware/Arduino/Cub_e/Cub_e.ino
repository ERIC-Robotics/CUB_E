#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
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

byte slave_id_left=7;
byte slave_id_right=2;
int INP_CONTROL_MODE=256;           
int PP_gain=16;
int PI_gain=16;
int VF_gain=16;
int LPR=2262;
int acceleration=30;
int speed=0;

const byte es_button_pin = 2;
const byte cs_button_pin = 3;
const int es_sig_pin = 53;
const int cs_sig_pin = 52;

int es_status = 0;
int cs_status = 0;

int sp = 5;
std_msgs::Int32 leftposition;
std_msgs::Int32 rightposition;
std_msgs::Int32 es_status_msg;
std_msgs::Int32 cs_status_msg;

long int Current_position_left;
long int Current_position_right;

double previousleftspeed;
double previousrightspeed;

ros::Publisher left_motor_pub("/leftmotor/feedback", &leftposition);
ros::Publisher right_motor_pub("/rightmotor/feedback", &rightposition);
ros::Publisher es_status_pub("/es_status/hardware", &es_status_msg);
ros::Publisher cs_status_pub("/cs_status/hardware", &cs_status_msg);

RMCS2303 rmcs;

SwitchMonitor switchMonitor(es_button_pin, cs_button_pin, es_sig_pin, cs_sig_pin);


void subscribe_left_command(const std_msgs::Float32& msg){
  if(msg.data > 0){
    rmcs.Speed(slave_id_left, msg.data); 
    rmcs.Enable_Digital_Mode(slave_id_left,1); 
  }
  else if(msg.data < 0){
    rmcs.Speed(slave_id_left, abs(msg.data)); 
    rmcs.Enable_Digital_Mode(slave_id_left,0);
  }
  // if(msg.data == 0){
  //   if(previousleftspeed > 0){
  //     rmcs.Disable_Digital_Mode(slave_id_left,1);
  //   }
  //   else if(previousleftspeed < 0){
  //     rmcs.Disable_Digital_Mode(slave_id_left,0);
  //   }
  // }
  // previousleftspeed = msg.data;
  else{
    rmcs.Disable_Digital_Mode(slave_id_left,0);
    rmcs.Disable_Digital_Mode(slave_id_left,1);
  }
}

void subscribe_right_command(const std_msgs::Float32& msg){
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

void subscribe_software_estop(const std_msgs::Float32& msg){
  if(msg.data == 1){
    // Software E-Stop is active
    rmcs.STOP(slave_id_right);
    rmcs.STOP(slave_id_left);
  }
}

ros::Subscriber<std_msgs::Float32> left_motor_sub("/leftmotor/command", subscribe_left_command);
ros::Subscriber<std_msgs::Float32> right_motor_sub("/rightmotor/command", subscribe_right_command);
ros::Subscriber<std_msgs::Float32> software_estop_sub("/es_status/software", subscribe_software_estop);

void setup() {
  Serial.begin(230400);
  motor_driver_init();
  right_init();
  Serial.println("\nRight Done");
  left_init();
  Serial.println("\nLeft Done");
  switchMonitor.begin();
  
  nh.initNode();
  nh.advertise(left_motor_pub);
  nh.advertise(right_motor_pub);
  nh.advertise(es_status_pub);
  nh.advertise(cs_status_pub);
  
  nh.subscribe(left_motor_sub);
  nh.subscribe(right_motor_sub);
  nh.subscribe(software_estop_sub);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  publish_left_position();
  publish_right_position();
  cnf_led_handle();
  es_statusUpdate_ros();
  cs_statusUpdate_ros();
  nh.spinOnce();

}


void cnf_led_handle(){
  switchMonitor.update();
}

void es_statusUpdate_ros(){
  es_status_msg.data = switchMonitor.get_es();
  es_status_pub.publish(&es_status_msg);
}

void cs_statusUpdate_ros(){
  cs_status_msg.data = switchMonitor.get_cs();
  cs_status_pub.publish(&cs_status_msg);
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

void left_init(){   
  rmcs.WRITE_PARAMETER(slave_id_left,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);
  rmcs.READ_PARAMETER(slave_id_left);
//  rmcs.Speed(slave_id_left, 5); 
//  rmcs.Enable_Digital_Mode(slave_id_left,1);
  rmcs.Disable_Digital_Mode(slave_id_left,0);
}

void publish_left_position(){
  Current_position_left=rmcs.Position_Feedback(slave_id_left); 
//  Serial.print("l ");
//  Serial.println(Current_position_left);
  leftposition.data = Current_position_left;
  left_motor_pub.publish(&leftposition);
}

void publish_right_position(){
  Current_position_right=rmcs.Position_Feedback(slave_id_right); 
//  Serial.print("r ");
//  Serial.println(Current_position_right);
  rightposition.data = Current_position_right;
  right_motor_pub.publish(&rightposition);
}
