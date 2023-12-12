#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <RMCS2303drive.h>
#include <SwitchMonitor.h>
#include <std_msgs/String.h>
#include <FastLED.h>

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

#define NUM_LEDS 104
#define DATA_PIN 4
#define factor 10.819

byte slave_id_right=2;
int INP_CONTROL_MODE=256;           
int PP_gain=12;
int PI_gain=3;
int VF_gain=16;
int LPR=2262;
int acceleration=100;
int speed=0;

int sp = 5;
std_msgs::Int64 rightposition;
std_msgs::Float64 battery_soc;

int cs_status = 0;
int es_status = 0;
int soft_es = 0;
int nav_status = 0;

long int Current_position_right;

ros::Publisher right_motor_pub("/rightmotor/feedback", &rightposition);
ros::Publisher battery_soc_pub("/battery_soc", &battery_soc);

RMCS2303 rmcs;
CRGB leds[NUM_LEDS];

void subscribe_right_command(const std_msgs::Float64& msg){
  if(cs_status){
    if(msg.data > 0){
      rmcs.Speed(slave_id_right, msg.data); 
      rmcs.Enable_Digital_Mode(slave_id_right,0); 
    }
    else if(msg.data < 0){
      rmcs.Speed(slave_id_right, abs(msg.data)); 
      rmcs.Enable_Digital_Mode(slave_id_right,1);
    }
    else{
      rmcs.Disable_Digital_Mode(slave_id_right,0);
      rmcs.Disable_Digital_Mode(slave_id_right,1);
    }
  }
}

void subscribe_software_estop(const std_msgs::Int64& msg){
  soft_es = msg.data;
  if(es_status == 0){
    if(soft_es == 0){
      if(nav_status == 2){
        for (int  i = 0; i < NUM_LEDS; i++)
        {
          leds[i] = CRGB::Yellow;
        }
        FastLED.show();
      }
      else{      
        for (int  i = 0; i < NUM_LEDS; i++)
        {
          leds[i] = CRGB::Red;
        }
        FastLED.show();
      }
    }
    else{
      for (int  i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CRGB::Green;
      }
      FastLED.show();
    }
  }
}

void subscribe_nav_feedback(const std_msgs::Int64& msg){
  nav_status = msg.data;
  if(!es_status && !soft_es){
    if(nav_status == 1){
      for (int  i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CRGB::Red;
      }
      FastLED.show();
    }
    else if(nav_status == 2){
      for (int  i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CRGB::Yellow;
      }
      FastLED.show();
    }
    else if(nav_status == 0){
      for (int  i = 0; i < NUM_LEDS; i++)
      {
        leds[i] = CRGB::Green;
      }
      FastLED.show();
    }
  }
}

void subscribe_cs(const std_msgs::Int64& msg){
  cs_status = msg.data;
  if(cs_status == 0){
    // Software E-Stop is active
    rmcs.Disable_Digital_Mode(slave_id_right,0);
    rmcs.Disable_Digital_Mode(slave_id_right,1);
  }
}

void subscribe_es(const std_msgs::Int64& msg){
  es_status = msg.data;
  if(es_status == 1){
    for (int  i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Green;
    }
    FastLED.show();
  }
}

ros::Subscriber<std_msgs::Float64> right_motor_sub("/rightmotor/command", subscribe_right_command);
ros::Subscriber<std_msgs::Int64> nav_feedback_sub("/nav_feedback", subscribe_nav_feedback);
ros::Subscriber<std_msgs::Int64> software_estop_sub("/es_status/software", subscribe_software_estop);

ros::Subscriber<std_msgs::Int64> cs_sub("/cs_status/hardware", subscribe_cs);
ros::Subscriber<std_msgs::Int64> es_sub("/es_status/hardware", subscribe_es);


void setup() {
  Serial.begin(115200);
  motor_driver_init();
  right_init();
  Serial.println("\nRight Done");

  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  
  for (int  i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Red;
  }
  FastLED.show();
  
  
  nh.initNode();
  nh.advertise(right_motor_pub);
  nh.advertise(battery_soc_pub);
  nh.subscribe(right_motor_sub);
  nh.subscribe(cs_sub);
  nh.subscribe(es_sub);
  nh.subscribe(nav_feedback_sub);
  nh.subscribe(software_estop_sub);

}

void loop() {
  // put your main code here, to run repeatedly:
  publish_right_position();
  publish_battery_soc();
  nh.spinOnce();

}

void motor_driver_init(){
  rmcs.Serial_selection(0);
  rmcs.begin(&Serial2,9600); 
}

void right_init(){
  rmcs.WRITE_PARAMETER(slave_id_right,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);
  rmcs.READ_PARAMETER(slave_id_right);
//  rmcs.Speed(slave_id_right, 20); 
//  rmcs.Enable_Digital_Mode(slave_id_right,0); 
  rmcs.Disable_Digital_Mode(slave_id_right,0);
}

void publish_battery_soc(){
  battery_soc.data = (analogRead(A0) * 4.88 * factor) / 1024.0;
  battery_soc_pub.publish(&battery_soc);
}

void publish_right_position(){
  Current_position_right=rmcs.Position_Feedback(slave_id_right); 
//  Serial.print("r ");
//  Serial.println(Current_position_right);
  rightposition.data = Current_position_right;
  right_motor_pub.publish(&rightposition);
}
