  #include <ros.h>
  #include <ros/time.h>
  #include <sensor_msgs/Range.h>
  #include <NewPing.h>

  ros::NodeHandle  nh;

  sensor_msgs::Range range_msg;
  ros::Publisher pub_range( "sonar_data_raw", &range_msg);
  char frameid_lv[] = "/sonar_lv";
  char frameid_rv[] = "/sonar_rv";
  char frameid_lm[] = "/sonar_lm";
  char frameid_rm[] = "/sonar_rm";
  char frameid_lav[] = "/sonar_lav";
  char frameid_laa[] = "/sonar_laa";
  char frameid_rav[] = "/sonar_rav";
  char frameid_raa[] = "/sonar_raa";

  //WHITE
  #define lav_trig 2
  #define lav_echo 3
  //GREY
  #define laa_trig A3
  #define laa_echo A2
  //GREEN
  #define raa_trig A1
  #define raa_echo A0
  //BLUE
  #define rav_trig 4
  #define rav_echo 5
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
    lv_trig,     raa_trig, rv_trig, laa_trig, rm_trig, lav_trig, rav_trig, lm_trig};
  int echos[] = {
    lv_echo,     raa_echo, rv_echo, laa_echo, rm_echo, lav_echo, rav_echo, lm_echo};
  char* frames[] = {
    frameid_lv,  frameid_raa, frameid_rv, frameid_laa, frameid_rm, frameid_lav, frameid_rav, frameid_lm
  };

  #define sonar_num  8
  #define max_distance 400
  #define ping_interval 50

  unsigned long pingTimer[sonar_num]; // Holds the times when the next ping should happen for each sensor.
  unsigned int cm[sonar_num];         // Where the ping distances are stored.
  uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

  NewPing sonar[] = {
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
    range_msg.min_range = 0.04;
    range_msg.max_range = max_distance;
  }


  void publishToROS(char* framee, float range) {
    range_msg.header.frame_id = framee;
    range_msg.range = range;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    nh.spinOnce();
  }

  long range_time;
  int i = 0;


  void loop() {
   if (i<sonar_num) {

      if (millis() >= pingTimer[i]) {         // Is it this sensor's time to ping?
        pingTimer[i] += ping_interval * sonar_num;  // Set next time this sensor will be pinged.
        sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
        currentSensor = i;            // Sensor being accessed.
        float result = float(sonar[currentSensor].ping_cm())/100;
        publishToROS(frames[i],result);
        i++;
      }
     } else {
      i = 0;
    }
    //delay(70);
  }
