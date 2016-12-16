#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

//#include <Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;


  
  tf::Transform transform;

  ros::Rate rate(10.0);

//De oorsprong is 
  int x0 = 0;
  int y0 = 0;
  int z0 = 0;


//tf::createQuaternionFromRPY(1.5,1.5,1.5)
//	tf::createQuaternionFromRPY (2,2,2);\


float theta =0; //rotatie van het frame
float theta2 =0;
  while (node.ok()){

	
    transform.setOrigin( tf::Vector3(0.0156471-x0, -0.1867174-y0, 0.2965-z0) );
	transform.setRotation( tf::Quaternion(tf::createQuaternionFromRPY(0,0,1.134+theta) ));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", 		"sonar_rv"));
   
    transform.setOrigin( tf::Vector3(0.0156471-x0, 0.1867174-y0, 0.2965-z0) );
    transform.setRotation( tf::Quaternion(tf::createQuaternionFromRPY(0,0,-1.134+theta2) ) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "sonar_lv"));
 
    transform.setOrigin( tf::Vector3(-0.2745557-x0, -0.2269027-y0, 0.2965-z0) );
     transform.setRotation( tf::Quaternion(tf::createQuaternionFromRPY(0,0, 3.5779+theta) ));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "sonar_rm"));
     
   transform.setOrigin( tf::Vector3(-0.2745557-x0, 0.2269027-y0, 0.2965-z0) );
    transform.setRotation( tf::Quaternion(tf::createQuaternionFromRPY(0,0, -3.5779+theta2) ));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "sonar_lm"));

    transform.setOrigin( tf::Vector3(-0.5233967-x0, -0.2355432-y0, 0.2965-z0) );
	transform.setRotation( tf::Quaternion(tf::createQuaternionFromRPY(0,0,-0.4799+theta) ));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "sonar_rav"));

    transform.setOrigin( tf::Vector3(-0.5233967-x0, 0.2355432-y0, 0.2965-z0) );
    transform.setRotation( tf::Quaternion(tf::createQuaternionFromRPY(0,0,0.4799+theta2) ));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "sonar_lav"));

     transform.setOrigin( tf::Vector3(-0.5689928-x0,  0.22827-y0, 0.2965-z0) );
    transform.setRotation( tf::Quaternion(tf::createQuaternionFromRPY(0,0,-1.91986+theta2) ));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "sonar_laa"));

     transform.setOrigin( tf::Vector3(-0.5689928-x0, -0.22827-y0, 0.2965-z0) );
  transform.setRotation( tf::Quaternion(tf::createQuaternionFromRPY(0,0,1.91986+theta) ));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "sonar_raa"));

    rate.sleep();
  }
  return 0;
};
