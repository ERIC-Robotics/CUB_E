#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
//#include <RMCS2303drive.h>
//#include <SwitchMonitor.h>
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
int soft_es = 0;

long int Current_position_right;

CRGB leds[NUM_LEDS];

void subscribe_software_estop(const std_msgs::Int64& msg){
  if(msg.data == 0){
    // Software E-Stop is active
    for (int  i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Red;
    }
    FastLED.show();
  }
  else{
    for (int  i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Green;
    }
    FastLED.show();
  }
}

void subscribe_nav_feedback(const std_msgs::Int64& msg){
  if(msg.data == 1){
    for (int  i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Red;
    }
    FastLED.show();
  }
  else if(msg.data == 2){
    for (int  i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::White;
    }
    FastLED.show();
  }
  else if(msg.data == 0){
    for (int  i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Green;
    }
    FastLED.show();
  }
}


ros::Subscriber<std_msgs::Int64> nav_feedback_sub("/nav_feedback", subscribe_nav_feedback);
ros::Subscriber<std_msgs::Int64> software_estop_sub("/es_status/software", subscribe_software_estop);



void setup() {
  Serial.begin(115200);
  Serial.println("\nRight Done");

  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  
  for (int  i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Red;
  }
  FastLED.show();
  
  
  nh.initNode();

  nh.subscribe(software_estop_sub);
  nh.subscribe(nav_feedback_sub);

}

void loop() {
  nh.spinOnce();

}
