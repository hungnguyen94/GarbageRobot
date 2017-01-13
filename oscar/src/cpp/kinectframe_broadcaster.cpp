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



  while (node.ok()){

	
    transform.setOrigin( tf::Vector3(-0.015-x0, 0-y0, 0.73-z0) );
	transform.setRotation( tf::Quaternion(tf::createQuaternionFromRPY(0,0,0) ));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", 		"kinect"));
   
   

    rate.sleep();
  }
  return 0;
};
