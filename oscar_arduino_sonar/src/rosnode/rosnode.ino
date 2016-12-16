#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <NewPing.h>

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
char* frames[] = {
  frameid_lv, frameid_rv, frameid_rm, frameid_rav, frameid_raa, frameid_laa, frameid_lav, frameid_lm
};

#define sonar_num  8
#define max_distance 200
#define ping_interval 33

unsigned long pingTimer[sonar_num]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[sonar_num];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.


NewPing sonar[sonar_num] = {
  NewPing(trigger[0], echos[0], max_distance),
  NewPing(trigger[1], echos[1], max_distance),
  NewPing(trigger[2], echos[2], max_distance),
  NewPing(trigger[3], echos[3], max_distance),
  NewPing(trigger[4], echos[4], max_distance),
  NewPing(trigger[5], echos[5], max_distance),
  NewPing(trigger[6], echos[6],max_distance),
  NewPing(trigger[7], echos[7],max_distance)
};

void setup() {
  Serial.begin (9600);
  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < sonar_num; i++){ // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + ping_interval;
  };
  setupRos();

}

void setupRos() {
  nh.initNode();
  nh.advertise(pub_range);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.field_of_view = 0.15;
  range_msg.min_range = 0.00;
  range_msg.max_range = 1000.4;
}

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


void publishToROS(char* framee, float range) {
  range_msg.header.frame_id = framee; 
  range_msg.range = range;
  range_msg.header.stamp = nh.now();
  pub_range.publish(&range_msg);
  nh.spinOnce();
}

void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

long range_time;
int i = 0;
void loop() {
 for (uint8_t i = 0; i < sonar_num; i++) { // Loop through all the sensors.
    if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += ping_interval * sonar_num;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == sonar_num - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
//    if ( millis() >= range_time ){
//      //float range = getRange(trigger[2],echos[2]);
//      NewPing sonar(trigger[i], echos[i], max_distance);
//      float range = sonar.ping_cm();
//      publishToROS(frames[2],range);     
//      range_time =  millis() + 250;
//      i++;
//    }
//  } 
//  else {
//    i = 0;
//  }

}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  // The following code would be replaced with your code that does something with the ping results.
  for (uint8_t i = 0; i < sonar_num; i++) {
    publishToROS(frames[i],cm[i]);
  }
}
