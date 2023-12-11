#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <RMCS2303drive.h>
#include <SwitchMonitor.h>
#include <std_msgs/String.h>
#include <FastLED.h>

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

RMCS2303 rmcs;
CRGB leds[NUM_LEDS];

void subscribe_software_estop(const std_msgs::Int64& msg){
  soft_es = msg.data;
  if(es_status == 0){
    if(msg.data == 0){
      if(nav_status == 2){
        for (int  i = 0; i < NUM_LEDS; i++)
        {
          leds[i] = CRGB::White;
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
}


void subscribe_es(const std_msgs::Int64& msg){
  es_status = msg.data;
  if(msg.data == 1){
    for (int  i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Green;
    }
    FastLED.show();
  }
}

ros::Subscriber<std_msgs::Int64> nav_feedback_sub("/nav_feedback", subscribe_nav_feedback);
ros::Subscriber<std_msgs::Int64> software_estop_sub("/es_status/software", subscribe_software_estop);

ros::Subscriber<std_msgs::Int64> es_sub("/es_status/hardware", subscribe_es);


void setup() {
  Serial.begin(115200);

  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  
  for (int  i = 0; i < NUM_LEDS; i++){
    leds[i] = CRGB::Red;
  }
  FastLED.show();
  
  
  nh.initNode();
  nh.subscribe(es_sub);
  nh.subscribe(nav_feedback_sub);
//
  nh.subscribe(software_estop_sub);

}

void loop() {
  nh.spinOnce();

}
