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
//GREEN
#define lm_trig 4
#define lm_echo A1
//BLUE
#define rm_trig 5
#define rm_echo A0
//YELLOW
#define lav_trig 6
#define lav_echo 7
//ORANGE
#define laa_trig 8
#define laa_echo 9
//RED
#define rav_trig 10
#define rav_echo 11
//BLACK
#define raa_trig 12
#define raa_echo 13

int trigger[] = {
  lv_trig, rv_trig, rm_trig, rav_trig, raa_trig, laa_trig, lav_trig, lm_trig};
int echos[] = {
  lv_echo, rv_echo, rm_echo, rav_echo, raa_echo, laa_echo, lav_echo, lm_echo};


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
  for(int i = 0; i < 8; i++){
    if ( millis() >= range_time ){
      float range = getRange(trigger[i],echos[i]);
      publishToROS(frameid,range);      
      range_time =  millis() + 500;
    }
  }
}

