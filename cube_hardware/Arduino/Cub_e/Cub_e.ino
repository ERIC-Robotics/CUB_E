#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <RMCS2303drive.h>


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


RMCS2303 rmcs;

byte slave_id_left=7;
byte slave_id_right=2;
int INP_CONTROL_MODE=256;           
int PP_gain=32;
int PI_gain=16;
int VF_gain=32;
int LPR=2262;
int acceleration=200;
int speed=0;

int sp = 5;
std_msgs::Int64 leftposition;
std_msgs::Int64 rightposition;

long int Current_position_left;
long int Current_position_right;

double previousleftspeed;
double previousrightspeed;

ros::Publisher left_motor_pub("/leftmotor/feedback", &leftposition);
ros::Publisher right_motor_pub("/rightmotor/feedback", &rightposition);

void subscribe_left_command(const std_msgs::Float64& msg){
  if(msg.data > 0){
    rmcs.Speed(slave_id_left, msg.data); 
    rmcs.Enable_Digital_Mode(slave_id_left,1); 
  }
  if(msg.data < 0){
    rmcs.Speed(slave_id_left, abs(msg.data)); 
    rmcs.Enable_Digital_Mode(slave_id_left,0);
  }
  if(msg.data == 0){
    if(previousleftspeed > 0){
      rmcs.Disable_Digital_Mode(slave_id_left,1);
    }
    else if(previousleftspeed < 0){
      rmcs.Disable_Digital_Mode(slave_id_left,0);
    }
  }
  previousleftspeed = msg.data;
}

void subscribe_right_command(const std_msgs::Float64& msg){
  if(msg.data > 0){
    rmcs.Speed(slave_id_right, msg.data); 
    rmcs.Enable_Digital_Mode(slave_id_right,0); 
  }
  if(msg.data < 0){
    rmcs.Speed(slave_id_right, abs(msg.data)); 
    rmcs.Enable_Digital_Mode(slave_id_right,1);
  }
  if(msg.data == 0){
    if(previousrightspeed > 0){
      rmcs.Disable_Digital_Mode(slave_id_right,0);
    }
    else if(previousrightspeed < 0){
      rmcs.Disable_Digital_Mode(slave_id_right,1);
    }
  }
  previousrightspeed = msg.data;  
}

ros::Subscriber<std_msgs::Float64> left_motor_sub("/leftmotor/command", subscribe_left_command);
ros::Subscriber<std_msgs::Float64> right_motor_sub("/rightmotor/command", subscribe_right_command);

void setup() {
  Serial.begin(115200);
  right_init();
  Serial.println("\nRight Done");
  left_init();
  Serial.println("\nLeft Done");
//  right_init();
//  Serial.println("\nRight Done");

//  delay(8000);
  nh.initNode();
  nh.advertise(left_motor_pub);
  nh.advertise(right_motor_pub);

  
  nh.subscribe(left_motor_sub);
  nh.subscribe(right_motor_sub);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  publish_left_position();
  publish_right_position();
//  Current_position_left=rmcs.Position_Feedback(slave_id_left); 
//  Serial.print("l ");
//  Serial.println(Current_position_left);
//  Current_position_right=rmcs.Position_Feedback(slave_id_right); 
//  Serial.print("r ");
//  Serial.println(Current_position_right);
  nh.spinOnce();

}

void right_init(){
  rmcs.Serial_selection(0);
//  Serial.println("Writing Right Parameters");  
  rmcs.begin(&Serial1,9600);    
  rmcs.WRITE_PARAMETER(slave_id_right,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);
  rmcs.READ_PARAMETER(slave_id_right);
//  rmcs.Speed(slave_id_right, 20); 
//  rmcs.Enable_Digital_Mode(slave_id_right,0); 
  rmcs.Disable_Digital_Mode(slave_id_right,0);
}

void left_init(){
//  rmcs.Serial_selection(0);           
//  Serial.println("Writing Left Parameters");
//  rmcs.begin(&Serial2,9600);    
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
