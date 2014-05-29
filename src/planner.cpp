#include <ros/ros.h>
#include "std_msgs/String.h"
#include <std_msgs/Char.h>
#include "Eigen/Dense"
#include "Eigen/LU"
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include "ardrone_autonomy/Navdata.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

/** possivelmente desnecessário 
#include <geometry_msgs/Vector3.h>
*/

#define KEYCODE_L 0x6C // lock slam mapping and save map
#define KEYCODE_U 0x75 // unlock mapping
#define KEYCODE_R 0x72 // reset map

#define KEYCODE_M 0x6D // disable control back to manual, disable setpoint.
#define KEYCODE_G 0x67 // get startframe and setpoint to hold place in startframe, enable control
#define KEYCODE_0 0x30 // set startframe to origin and setpoint to hold place where it is, enable control
#define KEYCODE_P 0x70 // follow path relative to startframe, enable control. if startframe is not set, startframe=origin



/** DECLARES GLOBAL VARIABLES */
double starttime;
bool followpath;

Eigen::VectorXd posetime(1); 
Eigen::MatrixXd poseSP(1,4);
int pathindex;

tf::StampedTransform predictedpose; /**  pose predicted by the slam node */
tf::Transform startTransform; /** the tranform between /map and /startFrame, wich is the pose at the first second of flight */
tf::Transform SPTransform; /** the tranform between /startFrame and /PoseSetPoint, witch is the set point related to the start frame */



/** the node uses the class SubscriveAndPublish to publish the control signal within a subscriber call back function
 * the subscriver call back function will receive the data from the /navdata topic, will calculate the position using the EKF
 * then will calculate the control signals and publish it in the /cmd_vel topic
 */ 

class SubscribeAndPublish
{
public:

	tf::TransformListener listener; /** declares object that will read the pose between /map and /cam_front frames */
		

	SubscribeAndPublish() /** contructor for the class */
	{
		/**Topic you want to publish*/
		poseSetPointPub_ = n_.advertise<geometry_msgs::Twist>("poseSetPoint", 1);
		poseRateSetPointPub_ = n_.advertise<geometry_msgs::Twist>("poseRateSetPoint", 1);

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
		keySub_ = n_.subscribe("/keyinput", 1, &SubscribeAndPublish::keyCallBack, this);
		navdataSub_ = n_.subscribe("ardrone/navdata", 1, &SubscribeAndPublish::Planning, this);
	}
		
	
	
	
	
	void setSP2origin(void)
	{
		SPTransform.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) ); /** set translation between /startFrame and /PoseSetPoint,*/
		SPTransform.setRotation( tf::Quaternion( 0.0, 0.0, 0.0 ) ); /** set rotation between /startFrame and /PoseSetPoint,*/
	}


	void setSP2current(void)
	{
		getCurrentPredicted();
		
		double roll;
		double pitch;
		double yaw;
		
		tf::Matrix3x3 R=tf::Matrix3x3(predictedpose.getRotation());  /** get rotation matrix from quaternion in transform */
		R.getRPY(roll, pitch, yaw);	 /** get euler angles */

		SPTransform.setOrigin( predictedpose.getOrigin() ); /** set translation between /startFrame and /PoseSetPoint,*/
		SPTransform.setRotation( tf::Quaternion( 0.0, 0.0, yaw ) ); /** set rotation between /startFrame and /PoseSetPoint,*/
	}


	void setStart2origin(void)
	{
		startTransform.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) ); /** set translation between /map and /startFrame */
		startTransform.setRotation( tf::Quaternion( 0.0, 0.0, 0.0 ) ); /**set rotation between /map and /startFrame */
	}
		
	
	void setStart2current(void)
	{
		getCurrentPredicted();
		
		double roll;
		double pitch;
		double yaw;
		
		tf::Matrix3x3 R=tf::Matrix3x3(predictedpose.getRotation());  /** get rotation matrix from quaternion in transform */
		R.getRPY(roll, pitch, yaw);	 /** get euler angles */

		startTransform.setOrigin( predictedpose.getOrigin() );  		/** set translation between /map and /startFrame */
		startTransform.setRotation( tf::Quaternion(0.0, 0.0, yaw ) ); 	/**set rotation between /map and /startFrame */
	}
	
	
	void getCurrentPredicted(void)
	{
		/** get pose predicted by the slam node */
		try
		{
			listener.waitForTransform("map", "ardrone_base_link", ros::Time(0), ros::Duration(10.0) );
			listener.lookupTransform("map", "ardrone_base_link", ros::Time(0), predictedpose); /** gets the last published transformation */
		}
			catch (tf::TransformException ex)
			{
				ROS_INFO("exceção do lookupTransform entre map e cam_front");
				ROS_ERROR("%s",ex.what());  /** handle errors */
			}
	}
	
	
	
	
	
	
	void keyCallBack(const std_msgs::Char key)
	{
		switch(key.data)
		{
			case KEYCODE_G:
				followpath =0;
				setStart2current();
				setSP2origin();
				ROS_INFO("command to set start frame at current pose and hold place.");
				break;   
				
				
			case KEYCODE_0:
				followpath =0;
				setStart2origin();
				setSP2current();
				ROS_INFO("command to set start frame at origin and hold place.");
				break;   
				
			case KEYCODE_P:
				followpath =1;
				ROS_INFO("command to follow path relative to start frame");
				break;   

			case KEYCODE_M:
				followpath =0;
				ROS_INFO("command to manual. pose setpoint unfixed");
				break;   
		}
	}

	
	
	
	
	
		/** declares callback function for ardrone/navdata subscriber*/
	void Planning(const ardrone_autonomy::Navdata navdata)	
	{			
						
		if  (!followpath) /** if follow path is not enabled, then gets start time */
		{
			starttime = navdata.tm;
			pathindex = 0;
		}
		else /** if follow path is enabled, then send path setpoint relative to start frame */
		{
			double elapsedtime;
			elapsedtime = (navdata.tm - starttime)/1000000;

			while ( ( elapsedtime > posetime(pathindex) ) && ( pathindex < ( posetime.size() - 1 ) ) )
			{
				pathindex++;
			}
			
			//geometry_msgs::Twist poseRateSP; /** velocities setpoints */
			
			//poseRateSP.linear.x = 0.0; 
			//poseRateSP.linear.y = 0.0;
			//poseRateSP.linear.z = 0.0;
			
			//poseRateSP.angular.x = 0.0;
			//poseRateSP.angular.y = 0.0;
			//poseRateSP.angular.z = 0.0;	
			
			/** publishes setpoint topics */
			//poseRateSetPointPub_.publish(poseRateSP);
			
			/** stores the transform for the pose set point frame*/
			SPTransform.setOrigin( tf::Vector3( poseSP(pathindex,0) , poseSP(pathindex,1) , poseSP(pathindex,2) ) ); /** set translation */
			SPTransform.setRotation( tf::Quaternion(0.0, 0.0, poseSP(pathindex,3) )  ); /** set rotation */
			
		} /** END OF THE followpath condition*/
				
		/** publishes setpoint transforms */
		static tf::TransformBroadcaster broadcaster; /** declares object that will publish the pose between /map and /startFrame */
		try
		{
			broadcaster.sendTransform(tf::StampedTransform(startTransform, ros::Time::now(), "/map", "/startFrame"));
			broadcaster.sendTransform( tf::StampedTransform(SPTransform, ros::Time::now(), "/startFrame", "/PoseSetPoint") );
		}
		catch (tf::TransformException ex)
		{
			ROS_INFO("exceção dos sendTransforms");
			ROS_ERROR("%s",ex.what());  /** handle errors */
		}

					
	} /** end of the callback function for the class SubscriveAndPublish*/







/** private part for the class SubscribeAndPublish */

private:
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node. */
  
	ros::NodeHandle n_; 
	ros::Publisher poseSetPointPub_;
	ros::Publisher poseRateSetPointPub_;
	ros::Subscriber keySub_;
	ros::Subscriber predictedPoseSub_;
	ros::Subscriber navdataSub_;

} ; /** end of the class declaration */







void loadpath(const std::string &path_file_path)
{
	
	if (path_file_path=="")
	{
		ROS_INFO("ArDroneControl - Planner: No path loaded! Will hold position if issued to follow path");
	}
	else
	{
		ROS_INFO("ArDroneControl - Planner: Loading Path");
		int i=0;
		std::ifstream infile(path_file_path.c_str());
		std::string line;
		while (std::getline(infile, line))
		{
		    std::istringstream iss(line);
		    double t, x, y, z, yaw;
		    if (!(iss >> t >> x >> y >> z >> yaw)) { break; } // error
		    
			posetime.conservativeResize(i+1);
			posetime(i)=t;
			
			poseSP.conservativeResize(i+1, 4);
			poseSP(i,0)=x;
			poseSP(i,1)=y;
			poseSP(i,2)=z;
			poseSP(i,3)=yaw;
			
			i++;
		}
		if (i>0)
		{
			ROS_INFO("ArDroneControl - Planner: Path loaded");
		}
		else
		{
			ROS_INFO("ArDroneControl - Planner: problem loading Path! Will hold position if issued to follow path");
		}
	}
} /** end of loadpath() function */






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
   
	ros::init(argc, argv, "planner");
	
	ROS_INFO("ArDroneControl: planner started");
	
	/** initialize variables */
	posetime(0)=0;			
	poseSP(0,0)=0;
	poseSP(0,1)=0;
	poseSP(0,2)=0;
	poseSP(0,3)=0;
	pathindex = 0;
	
	SPTransform.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) ); /** set translation between /startFrame and /PoseSetPoint,*/
	SPTransform.setRotation( tf::Quaternion( 0.0, 0.0, 0.0 ) ); /** set rotation between /startFrame and /PoseSetPoint,*/
	startTransform.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) ); /** set translation between /map and /startFrame */
	startTransform.setRotation( tf::Quaternion( 0.0, 0.0, 0.0 ) ); /**set rotation between /map and /startFrame */

	/** get parameters */
	ros::NodeHandle private_node_handle("~");
	
	/** load map */
	std::string path_file_path;
	private_node_handle.param<std::string>("path_file_path", path_file_path, "");
	ROS_INFO("ArDroneControl - Planner: path_file_path = %s", path_file_path.c_str() );
	
	loadpath(path_file_path);
	
	/** creates an object of the declared class. this object will perform the comunication and calculation*/
	SubscribeAndPublish SAPObject;
	
	/** ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master. */
	
	ros::spin();

	return 0;
}
