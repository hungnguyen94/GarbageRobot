/*
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);
char frameid_lv[] = "/oscar/sonar_lv";
char frameid_rv[] = "/oscar/sonar_rv";
char frameid_lm[] = "/oscar/sonar_lm";
char frameid_rm[] = "/oscar/sonar_rm";
char frameid_lav[] = "/oscar/sonar_lav";
char frameid_laa[] = "/oscar/sonar_laa";
char frameid_rav[] = "/oscar/sonar_rav";
char frameid_raa[] = "/oscar/sonar_raa";
*/

//WHITE
#define lav_trig 3
#define lav_echo A2
//GREY
#define laa_trig 4
#define laa_echo A3
//GREEN
#define raa_trig 4
#define raa_echo A1
//BLUE
#define rav_trig 5
#define rav_echo A0
//YELLOW
#define lv_trig 6
#define lv_echo 7
//ORANGE
#define lm_trig 8
#define lm_echo 9
//RED
#define rm_trig 10
#define rm_echo 11
//BLACK
#define rv_trig 12
#define rv_echo 13

int trigger[] = {
  lv_trig, rv_trig, rm_trig, rav_trig, raa_trig, laa_trig, lav_trig, lm_trig};
int echos[] = {
  lv_echo, rv_echo, rm_echo, rav_echo, raa_echo, laa_echo, lav_echo, lm_echo};
/*
char* frames[] = {
  frameid_lv, frameid_rv, frameid_rm, frameid_rav, frameid_raa, frameid_laa, frameid_lav, frameid_lm
};
*/

void setup() {
  Serial.begin (9600);
  //  for(int i = 0; i < 8 ; i++) {
  //    pinMode(trigger[i], OUTPUT);
  //    pinMode(echos[i], INPUT);
  //  }
  //setupRos();

}

/*
void setupRos() {
  nh.initNode();
  nh.advertise(pub_range);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.field_of_view = 0.15;
  range_msg.min_range = 0.00;
  range_msg.max_range = 1000.4;
}
*/

float getRange(int trig, int echo) {
  long duration;
  float distance;
  digitalWrite(trig, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trig, HIGH);
  //  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trig, LOW);
  duration = float(pulseIn(echo, HIGH, 500));
  distance = duration / 5820.0;
  return duration;
}

/*
void publishToROS(char* framee, float range) {
  range_msg.header.frame_id = framee; 
  range_msg.range = range;
  range_msg.header.stamp = nh.now();
  pub_range.publish(&range_msg);
  nh.spinOnce();
}
*/

long range_time;
int i = 0;
void loop() {
//  
//  if(i<8) {
//    pinMode(trigger[i], OUTPUT);
//    pinMode(echos[i], INPUT);
//    if ( millis() >= range_time ){
//      float range = getRange(trigger[i],echos[i]);
//      Serial.println(range);
//      //publishToROS(frames[i],range);     
//      if (i < 4){
//        pinMode(trigger[i+4], OUTPUT);
//        pinMode(echos[i+4], INPUT);
//        float range = getRange(trigger[i+4],echos[i+4]);
//        Serial.println(range);
//        //publishToROS(frames[i+4],range); 
//      }
//      else{
//        pinMode(trigger[i-4], OUTPUT);
//        pinMode(echos[i-4], INPUT);
//        float range = getRange(trigger[i-4],echos[i-4]);
//        Serial.println(range);
//        //publishToROS(frames[i-4],range); 
//      } 
//      range_time =  millis() + 250;
//      i++;
//    }
//  } 
//  else {
//    i = 0;
//  }

  pinMode(10, OUTPUT);
  pinMode(11, INPUT);
  Serial.println(getRange(10,11));
  
  delay(100);
}


