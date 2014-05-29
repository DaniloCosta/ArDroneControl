#include <ros/ros.h>
#include <cmath>
#include "ardrone_autonomy/Navdata.h"
#include "Eigen/Dense"
#include "Eigen/LU"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Char.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>

/** possivelmente desnecessário

#include "std_msgs/String.h"
#include <sstream>

*/

#define KEYCODE_L 0x6C // lock slam mapping and save map
#define KEYCODE_U 0x75 // unlock mapping
#define KEYCODE_R 0x72 // reset map

#define KEYCODE_M 0x6D // disable control back to manual, disable setpoint.
#define KEYCODE_G 0x67 // get startframe and setpoint to hold place in startframe, enable control
#define KEYCODE_0 0x30 // set startframe to origin and setpoint to hold place where it is, enable control
#define KEYCODE_P 0x70 // follow path relative to startframe, enable control

/** parameters */

double euler_angle_max;
double control_vz_max;
double control_yawrate_max;
bool usehover; /** if usehover =1 uses the ardrone hover stabilization when near=1*/
double near_upper_threshold;
double near_lower_threshold;
double kpyaw;
double kdyaw;
double kpaltd;
double K1;
double K2;

/** DECLARES GLOBAL VARIABLES */

uint64_t timestamp; /** drone's timestamp buffer MUST BE RETENTIVE*/
double takeofftime;
geometry_msgs::Vector3 angular_velocity;
geometry_msgs::Twist poseRateSP; /** velocitys setpoints */
tf::StampedTransform transformPoseError; /** transform between /ardrone_base_link /poseSetPoint*/
int near; /** near=1 if distance to setpoint is small */
bool enablecontrol; /** to enable the control focus the terminal and press the C key on keyboard */

/** the node uses the class SubscriveAndPublish to publish the control signal within a subscriber call back function
 * the subscriver call back function will receive the data from the /navdata topic, will calculate the position using the EKF
 * then will calculate the control signals and publish it in the /cmd_vel topic
 */ 

class SubscribeAndPublish
{
public:

	tf::TransformListener listener; /** declares object that will read the pose between /map and /cam_front frames */
					

	SubscribeAndPublish()
	{
		/**Topic you want to publish*/
		cmdpub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
		errpub_ = n_.advertise<geometry_msgs::Twist>("/controlerror", 1);
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
		* and the linear velocitys (vx vy vz)  and altitude (altd) to execute the 
		* EKF algorithm
		*/ 
		keySub_ = n_.subscribe("/keyinput", 1, &SubscribeAndPublish::keyCallBack, this);
		poseRateSetPointSub_ = n_.subscribe("poseRateSetPoint", 1, &SubscribeAndPublish::poseRateSPCallback, this);
		navdataSub_ = n_.subscribe("ardrone/navdata", 1, &SubscribeAndPublish::Control, this);
		imusub_ = n_.subscribe("ardrone/imu", 1, &SubscribeAndPublish::IMU, this);
	}
	
	
	
	void keyCallBack(const std_msgs::Char key)
	{
		switch(key.data)
		{
			case KEYCODE_G:
				enablecontrol =1;
				ROS_INFO("Control enabled. Drone holding place");
				break;   
				
			case KEYCODE_0:
				enablecontrol =1;
				ROS_INFO("Control enabled. Drone holding place");
				break;  

			case KEYCODE_P:
				enablecontrol =1;
				ROS_INFO("Control enabled. Drone following path");
				break;  

			case KEYCODE_M:
				enablecontrol =0;
				ROS_INFO("Control disabled. Drone back in manual");
				break;   
		}
	}
	
	
	void poseRateSPCallback(const geometry_msgs::Twist msg)
	{
		poseRateSP = msg;
	}
	
	
	
	void IMU(const sensor_msgs::Imu	imu)
	{
		angular_velocity.x = imu.angular_velocity.x *3.1415 / 180.0; /** angular velocitys in rad/sec */
		angular_velocity.y = imu.angular_velocity.y *3.1415 / 180.0; /** angular velocitys in rad/sec */
		angular_velocity.z = imu.angular_velocity.z *3.1415 / 180.0; /** angular velocitys in rad/sec */
	
	}



		/** declara callback */
	void Control(const ardrone_autonomy::Navdata navdata)
	{
		timestamp = navdata.tm;
		
		
		/** CHECK IF THE DRONE IS FLYING. ASSUMES THE DRONE WILL TAKE OFF AT [0 0] */ 
		if  ((navdata.state == 2) || (navdata.state == 7)) /** 2 = landed, ready to take off. 7 = taking off */
		{	
			takeofftime = timestamp;
		}
		else 
		{
			if ((navdata.state == 3 ) || (navdata.state == 4) || ( navdata.state == 8)) /** 3=flying 8=transition to hover, 4= hovering */
			{
				if (((timestamp - takeofftime)/1000000.0) > 2) /** past 2 seconds of stable flight do the control:  */ 
				{
					/** GET TRANSFORM */
					try
					{
						listener.waitForTransform("ardrone_base_link", "PoseSetPoint", ros::Time(0), ros::Duration(10.0) );
						listener.lookupTransform("ardrone_base_link", "PoseSetPoint", ros::Time(0), transformPoseError); /** gets the last published transformation */
					}
					
					catch (tf::TransformException ex)
					{
						ROS_ERROR("%s",ex.what());  /** handle errors */
					}
					
					
					/** INPUTS */
					double linvx; linvx = navdata.vx/1000.0; /** linear velocity in the x axis (in m/sec) */
					double linvy; linvy = navdata.vy/1000.0; /** linear velocity in the x axis (in m/sec) */
					
					/** navdata.vx and .vy are linear velocitys in coordinates of the body frame. 
					 */
					 
					 
					double roll; /** euler angles for rotation */
					double pitch;
					double yawErr;
					
					tf::Matrix3x3 R=tf::Matrix3x3(transformPoseError.getRotation());  /** get rotation matrix from quaternion in transform */
					R.getRPY(roll, pitch, yawErr);	 /** get euler angles */
					
					double altitudeErr;
					altitudeErr = transformPoseError.getOrigin().z();
					
					Eigen::Vector2d xyErr;
					xyErr << transformPoseError.getOrigin().x() , transformPoseError.getOrigin().y() ;
					
					
					geometry_msgs::Twist error;
					
					error.linear.x=xyErr(0);
					error.linear.y=xyErr(1);
					error.linear.z=altitudeErr;
					error.angular.x=0;
					error.angular.y=0;
					error.angular.z=yawErr;
					
					errpub_.publish(error); 
											
					
					
					/** CONTROLER */
					
					geometry_msgs::Twist controlsig;
					
					
					/** it will publish control signals on the topic /cmd_vel of message type geometry_msgs::Twist
					* geometry_msg::twist tem seguinte forma
					* 		geometry_msg::vector3	linear
					*	 	geometry_msg::vector3	angular
					* 
					* geometry_msg::vector3 tem seguinte forma
					* 		float64 x
					* 		float64 y
					* 		float64 z    (tudo minusculo)  */

								
								
					/** CALCULATES CONTROL SIGNALS */


					/** YAW control law */
								
					double spYawRate;
					
					
					if (((170 < yawErr) && (yawErr <190)) || ((-190 < yawErr) && (yawErr < -170)))
					{
						yawErr = 170.0;
					}
					else if (190 < yawErr) 
					{
						yawErr = yawErr -360.0;
					}
					else if (yawErr < -190)
					{
						yawErr = yawErr +360.0;
					}
					
					spYawRate = kpyaw *yawErr - kdyaw * angular_velocity.z ;
					
					controlsig.angular.z = spYawRate/control_yawrate_max;
					
					
					
					
					/** ALTITUDE control */

					double spAltdRate;
					
					spAltdRate= kpaltd*altitudeErr;
					
					controlsig.linear.z = spAltdRate/(control_vz_max/1000);
					
					
										
					/** HORIZONTAL POSITION control */
					
					/** if distace is less than 15cm than it is near and will use hovermode.
					 * if it is in hover mode (near=1), and distance increses to more than 30cm, then leave hovermode */
					 
					if (sqrt(xyErr(1)*xyErr(1)+xyErr(0)*xyErr(0))<near_lower_threshold)
					{
						near = 1;
					}
					else
					{
						if (sqrt(xyErr(1)*xyErr(1)+xyErr(0)*xyErr(0))>near_upper_threshold)
						{
							near=0;
						}
					}
					
					
					if (usehover && near) /** if it is in hover null all control signals */
					{
						controlsig.linear.x = 0;
						controlsig.linear.y = 0;
						controlsig.angular.x = 0;   
						controlsig.angular.y = 0;	
					}
					else /** if it is not in hover mode calculates all control signals using PD law */
					{						
						double spPitch; /** control signals in rad */
						double spRoll;
						
						spPitch = K1 * ( xyErr(0) - K2 * linvx); /** control law */
						spRoll = - K1 * ( xyErr(1) - K2 * linvy);
						
						controlsig.linear.x = spPitch / euler_angle_max; /** pitch é roty, mas não confunda com o comando.... */
						controlsig.linear.y = -spRoll / euler_angle_max ; /** roll é rotx, mas não confunda com o comando.... */
						
						/** control signals not used, but must be != 0 to stay out of hover mode */
						controlsig.angular.x = 1 ;   
						controlsig.angular.y = 1 ;		
					}

			
					
					/** limit the control signals (talvez não seja necessário) */
					
	
					if (controlsig.angular.z > 1)
					{
						controlsig.angular.z =1;
					}
					if (controlsig.angular.z < -1)
					{
						controlsig.angular.z =-1;
					}
										
					if (controlsig.linear.x > 1)
					{
						controlsig.linear.x =1;
					}
					if (controlsig.linear.x < -1)
					{
						controlsig.linear.x =-1;
					}
					
					if (controlsig.linear.y > 1)
					{
						controlsig.linear.y =1;
					}
					if (controlsig.linear.y < -1)
					{
						controlsig.linear.y =-1;
					}
					
					if (controlsig.linear.z > 1)
					{
						controlsig.linear.z =1;
					}
					if (controlsig.linear.z < -1)
					{
						controlsig.linear.z =-1;
					}
					
					
					/** PUBLISH CONTROL SIGNALS */
					
					if (enablecontrol)
					{
						 cmdpub_.publish(controlsig); 
					}
					
					
					/** end of the control step */

				} /** END OF THE FLIGHT TIME CONDITION */
			} /** END OF THE FLYING STATE CONDITION */	 
		} /** END OF THE FLYING STATE CONDITION */
	} /** end of the callback function for the class SubscriveAndPublish*/




/** private part for the class SubscribeAndPublish */

private:
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node. */
  
	ros::NodeHandle n_; 
	ros::Publisher cmdpub_;
	ros::Publisher errpub_;
	ros::Subscriber keySub_;
	ros::Subscriber poseSetPointSub_ ;
	ros::Subscriber poseRateSetPointSub_ ;
	ros::Subscriber navdataSub_ ;
	ros::Subscriber imusub_ ;


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
   
	ros::init(argc, argv, "position_control");	
	
	ROS_INFO("ArDroneControl: position_control started");
	
	/** get parameters */
	ros::NodeHandle private_node_handle("~");
	
	private_node_handle.param<double>("euler_angle_max", euler_angle_max, 0.2);
	ROS_INFO("ArDroneControl - Position_control: euler_angle_max = %f", euler_angle_max);
	private_node_handle.param<double>("control_vz_max", control_vz_max, 700);
	ROS_INFO("ArDroneControl - Position_control: control_vz_max = %f", control_vz_max);
	private_node_handle.param<double>("control_yawrate_max", control_yawrate_max, 1.75);
	ROS_INFO("ArDroneControl - Position_control: control_yawrate_max = %f", control_yawrate_max);
	private_node_handle.param<bool>("usehover", usehover, false);
	ROS_INFO("ArDroneControl - Position_control: usehover = %d", usehover);	
	private_node_handle.param<double>("near_upper_threshold", near_upper_threshold, 0.6);
	ROS_INFO("ArDroneControl - Position_control: near_upper_threshold = %f", near_upper_threshold);
	private_node_handle.param<double>("near_lower_threshold", near_lower_threshold, 0.3);
	ROS_INFO("ArDroneControl - Position_control: near_lower_threshold = %f", near_lower_threshold);
	
	private_node_handle.param<double>("kpyaw", kpyaw, 1.5);
	ROS_INFO("ArDroneControl - Position_control: kpyaw = %f", kpyaw);
	private_node_handle.param<double>("kdyaw", kdyaw, 0.5);
	ROS_INFO("ArDroneControl - Position_control: kdyaw = %f", kdyaw);
	private_node_handle.param<double>("kpaltd", kpaltd, 0.45);
	ROS_INFO("ArDroneControl - Position_control: kpaltd = %f", kpaltd);
	private_node_handle.param<double>("K1", K1, 0.07);
	ROS_INFO("ArDroneControl - Position_control: K1 = %f", K1);
	private_node_handle.param<double>("K2", K2, 1.1);
	ROS_INFO("ArDroneControl - Position_control: K2 = %f", K2);

	
	/** creates an object of the declared class. this object will perform the comunication and calculation*/
	SubscribeAndPublish SAPObject;

	/** ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master. */
	
	ros::spin();

	return 0;
}
