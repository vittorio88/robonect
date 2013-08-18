#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster_fixed");
  ros::NodeHandle n;

  ros::Rate r(3);

  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
	tf::Transform(tf::Quaternion( 0,0.382,0,0.924), tf::Vector3(0.0, 0.0, 0.4)),
ros::Time::now(),"/base_link", "/camera_link"));

    r.sleep();
  }
}
