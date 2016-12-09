#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;


sensor_msgs::Range range_msg;
ros::Publisher pub_range( "range_data", &range_msg);
char frameid[] = "/sonar_test";

#define trigPin 2
#define echoPin 3

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
