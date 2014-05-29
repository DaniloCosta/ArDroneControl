#include <ros/ros.h>
#include <cmath>
#include "ardrone_autonomy/Navdata.h"
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include "tum_ardrone/filter_state.h"

/** possivelmente desnecessário

#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>


*/



/** DECLARES GLOBAL VARIABLES */


geometry_msgs::Vector3 firmware_euler_angles;


/** the node uses the class SubscriveAndPublish to publish the control signal within a subscriber call back function
 * the subscriver call back function will receive the data from the /navdata topic, will calculate the position using the EKF
 * then will calculate the control signals and publish it in the /cmd_vel topic
 */ 

class SubscribeAndPublish
{
public:
	SubscribeAndPublish()
	{

	    /** the subscribe() call is how you tell ROS that you want to receive messages
	    * on a given topic.  This invokes a call to the ROS
	    * master node, which keeps a registry of who is publishing and who
	    * is subscribing.  Messages are passed to a callback function, here
	    * called chatterCallback.  subscribe() returns a Subscriber object that you
	    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	    * object go out of scope, this callback will automatically be unsubscribed from
	    * this topic.
	    *
	    * The second parameter to the subscribe() function is the size of the message
	    * queue.  If messages are arriving faster than they are being processed, this
	    * is the number of messages that will be buffered up before beginning to throw
	    * away the oldest ones.
	    */
	    
		/** subscribe to ardrone/navdata topic to get the attitude (rotX rotY rotZ) 
		* and the linear velocitys (vx vy vz)  and altitude (altd) 
		*/ 
		navdataSub_ = n_.subscribe("ardrone/navdata", 1, &SubscribeAndPublish::navdataCallBack, this);
		predictedPoseSub_ = n_.subscribe("ardrone/predictedPose", 1, &SubscribeAndPublish::Interface, this);
	
	}
	/** body frame considered by ardrone_autonomy: x = pointing front, y = pointing left, z = pointing up 
	 * this is consistent with rotation (rotX = rotation about the x axis = roll), linear and angular velocities, 
	 * accelerometer and magnetometer. this body frame will also be considered by ArDroneControl.
	 * the reference frame considered by PTAM and tum_ardrone is a bit diferent.
	 * this node makes the interface between tum_ardrone and other ArDroneControl nodes.
	 * */
	

		
		/** declares callback function for ardrone/navdata subscriber*/
	void navdataCallBack(const ardrone_autonomy::Navdata navdata)		
	{
		firmware_euler_angles.x = navdata.rotX*3.1415 /180.0; /** roll (go to right/left) */ 
		firmware_euler_angles.y = navdata.rotY*3.1415 /180.0; /** pitch (go ahead/back) */
		firmware_euler_angles.z = navdata.rotZ*3.1415 /180.0; /** yaw (turn camera left/right) */
	}
		
		
		
		
		/** declares callback function for ardrone/predictedPose subscriber*/
	void Interface(const tum_ardrone::filter_state msg)
	{	
		tf::Matrix3x3 R;
		R.setRPY( firmware_euler_angles.x , firmware_euler_angles.y , firmware_euler_angles.z );
		tf::Transform transform( R, tf::Vector3( msg.y, -msg.x, msg.z ) );/** the tranform between "map" and "ardrone_base_link" witch is the pose predicted by tum_ardrone*/
		static tf::TransformBroadcaster broadcaster; /** declares object that will publish the pose between /map and /ardrone_base_link */

		/** publishes the transform */
		try
		{
			broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "ardrone_base_link"));
		}
		catch (tf::TransformException ex)
		{
			ROS_INFO("exceção dos sendTransforms");
			ROS_ERROR("%s",ex.what());  /** handle errors */
		}

	} /** end of the callback function interface*/



/** private part for the class SubscribeAndPublish */

private:
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node. */
  
	ros::NodeHandle n_; 
	ros::Subscriber predictedPoseSub_;
	ros::Subscriber navdataSub_;
	
} ; /** end of the class declaration */




int main(int argc, char **argv)
{
	/** The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line. For programmatic
	* remappings you can use a different version of init() which takes remappings
	* directly, but for most command-line programs, passing argc and argv is the easiest
	* way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.*/
   
	ros::init(argc, argv, "tum_interface");
	
	ROS_INFO("ArDroneControl: tum_interface started");
	/** creates an object of the declared class. this object will perform the comunication and calculation*/
	SubscribeAndPublish SAPObject;
	
	/** ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master. */
	
	ros::spin();

	return 0;
}
