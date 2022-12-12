#include "ros/ros.h"
#include <tf/transform_listener.h>


int main( int argc, char** argv ) {

	ros::init(argc, argv, "snake_tf");
	ros::NodeHandle nh;
	//Wait ROS node starts
	sleep(1); 
	//Declare the listener to use c++ tf API
  tf::TransformListener listener;
	//Declare the tranfsorm object to store tf data
  tf::StampedTransform transform;

	for(int i=0; i<10; i++ ) {
		try {
			//We want the current transform
			ros::Time now = ros::Time::now();
			if( listener.waitForTransform("/uav_link", "/Probe", now, ros::Duration(1.0)) ) {
				listener.lookupTransform("/uav_link", "/Probe",  now, transform);
				std::cout << "Translation: " << transform.getOrigin().x() << " " << transform.getOrigin().y() << " " << transform.getOrigin().z() << std::endl; 
				
				std::cout << "Orientation with Quaternion: "  << std::endl; 
				std::cout << "Rotation: "  << " " << transform.getRotation().x() << " " << transform.getRotation().y() << " " << transform.getRotation().z() << std::endl; 
				std::cout << "Axis: " << " " << transform.getRotation().w()<<std::endl;

				//Convert quaternion to euler angles
				tf::Matrix3x3 m( transform.getRotation() );
				double roll, pitch, yaw;
				m.getRPY(roll, pitch, yaw);

//				std::cout << "Orientation with RPY (xyz): " << roll << " " << pitch << " " << yaw << std::endl;
				std::cout << "Orientation with RPY (zyx): " << yaw << " " << pitch << " " << roll << std::endl;
			} else { ROS_WARN("Transform not ready"); }
		}
		catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		ros::Duration(1.0).sleep();	
	}
}
