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

//WHITE
#define lv_trig 0
#define lv_echo 1
//GREY
#define rv_trig 2
#define rv_echo 3
#define lm_trig 4
#define lm_echo 5
#define rm_trig 6
#define rm_echo 7
#define lav_trig 8
#define lav_echo 9
#define laa_trig 9
#define laa_echo 9
#define rav_trig 9
#define rav_echo 9
#define raa_trig 9
#define raa_echo 9



void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  setupRos();

}

void setupRos() {
  nh.initNode();
  nh.advertise(pub_range);
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.15;
  range_msg.min_range = 0.03;
  range_msg.max_range = 0.4;
}

float getRange(int trig, int echo) {
  long duration, distance;
  digitalWrite(trig, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trig, HIGH);
  //  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  distance = (duration / 58.2);
  return distance;
}


void publishToROS(char frameid[] frame, float range) {
   range_msg.header.frame_id = frame; 
   range_msg.range = range;
   range_msg.header.stamp = nh.now();
   pub_range.publish(&range_msg);
   nh.spinOnce();
}
long range_time;
void loop() {
  if ( millis() >= range_time ){
    float range = getRange(trigPin,echoPin);
    publishToROS(frameid,range);      
    range_time =  millis() + 500;
  }
}
