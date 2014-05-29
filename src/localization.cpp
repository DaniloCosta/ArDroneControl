#include <ros/ros.h>
#include "std_msgs/String.h"
#include <std_msgs/Char.h>
#include <sstream>
#include <fstream>
#include <string>
#include <cmath>
#include "Eigen/Dense"
#include "Eigen/LU"
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include "ardrone_autonomy/Navdata.h"
#include <tum_ardrone/filter_state.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


//#include <vector>

/** parameters  */
double covarobsscalar; /** relates to the observation covariance */
double covarodoscalar; /** relates to the odometry covariance */
bool publishmap; /** option to publish map markers or not */
bool publishvelocitymarker; /** option to publish arrows representing linear velocitys */
bool covarobsproportional; /** option to calculate the observation covariance proportional to the estimated observation error */
double covarinit; /** initial value for the covariance */

/** GLOBAL VARIABLES */
Eigen::VectorXd mean(3); /** MUST BE RETENTIVE filter gaussian function mean (also the estimated position) mean(1)=X mean(2)=Y */
Eigen::MatrixXd covar(3,3); /** MUST BE RETENTIVE filter covariance matrix */
uint64_t timestamp; /** drone's timestamp buffer MUST BE RETENTIVE*/
double altd;

double prevyaw;
double yawoffset; /** to fix the weird yaw jump done by the ardrone firmware when it is put to fly */

Eigen::VectorXi list;
Eigen::VectorXd map;
tum_ardrone::filter_state filterState; /** tum filter state. this node only uses it to interface with tum_ardrone node drone_gui */
tf::Transform transform; /** the tranform between "map" and "ardrone_base_link" witch is the pose predicted by Kalman filter*/
geometry_msgs::Twist Pose;
		
/** the node uses the class SubscriveAndPublish to publish the control signal within a subscriber call back function
 * the subscriver call back function will receive the data from the /navdata topic, will calculate the position using the EKF
 * then will calculate the control signals and publish it in the /cmd_vel topic
 */ 

class SubscribeAndPublish
{
public:

	tf::TransformListener listener;


	SubscribeAndPublish()  /** construtor */
	{
		/**Topic you want to publish*/
		
		PosePub_ = n_.advertise<geometry_msgs::Twist>("/Pose", 1);
		mapMarkerPub_ = n_.advertise<visualization_msgs::MarkerArray>("/map/mapMarkerArray",1); /** array with the position of mapped landmarks */
		predictedPosePub_ = n_.advertise<tum_ardrone::filter_state>("/ardrone/predictedPose", 1);
		velocityMarkerPub_ = n_.advertise<visualization_msgs::Marker>("/velocityMarker",1); /** array with arrows representing linear velocitys */
		
	    /** DESELEGANTE, PUBLICA NO TOPICO DE FILTER_STATE SOMENTE PARA CHAMAR A CALLBACK DO PLANNER 
	     * utiliza também para publicar estado da bateria no tum_gui
	     * 
	     * */
	    
	   
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
	    
		navdataSub_ = n_.subscribe("ardrone/navdata", 1, &SubscribeAndPublish::odometry, this);
		markerSub_ = n_.subscribe("visualization_marker_array", 1, &SubscribeAndPublish::observation, this);
	} 
	
		
	
	
	
	
	/** declara callback */
	void odometry(const ardrone_autonomy::Navdata navdata)
	{

		if (timestamp == 0) /** if it is the first time the callback is called, then sets global variables */
		{
			yawoffset = navdata.rotZ *3.1415/180.0;
			
			timestamp= navdata.tm;
			altd = navdata.altd/1000.0;			
		}
		else
		{
			// ROS_INFO("SLAM: odometry step");
			double dt; dt = (navdata.tm - timestamp)/1000000.0; /** time between iterations (in seconds) */
			timestamp= navdata.tm;
			
			double linvx; linvx = navdata.vx/1000.0; /** linear velocity in the x axis (in m/sec) */
			double linvy; linvy = navdata.vy/1000.0; /** linear velocity in the y axis (in m/sec) */
			
			if(publishvelocitymarker)
			{
				publishvelmarker(linvx, linvy); /** publish arrow markers representing the linear velocitys */
			}					
			
			/** navdata.vx and .vy are linear velocitys in coordinates of the body frame. 
			 * we need the linear velocitys in coordinates of the reference frame.
			 * vel(reference frame) = rRb * vel(body frame)
			 */
			double roll;
			double pitch;
			double yaw; 
			roll = navdata.rotX *3.1415/180.0;  /** rotation angles in rad. navdata.rotZ is in degrees */
			pitch = navdata.rotY *3.1415/180.0;
			
			/** WEIRD!! STILL HAVE TO COME UP WITH A BETTER WAY TO OVERCOME THIS. MAYBE INTEGRATING ANGULAR VELOCITY BY MYSELF */
			yaw = navdata.rotZ *3.1415/180.0 - yawoffset;
			 if (fabs((yaw - prevyaw)) > 15*3.1415/180) /** if yaw jump happens */
			{
				yawoffset = yawoffset + yaw - prevyaw;
				yaw = navdata.rotZ *3.1415/180.0 - yawoffset;
			}
			prevyaw = yaw;

			/** updates the mean using the modeled g(X,U) */
			Eigen::Vector3d deltamean;
			deltamean << (linvx * cos(yaw) - linvy * sin(yaw)) * dt ,
							(linvx * sin(yaw) + linvy * cos(yaw)) * dt ,
							(navdata.altd/1000.0 - altd);
			
			altd = navdata.altd/1000.0;
											
			mean(0) = mean(0) + deltamean(0);
			mean(1) = mean(1) + deltamean(1);
			mean(2) = mean(2) + deltamean(2);
			
			/** errors at 10% of the velocity in m/s (absolute value) */
			/** G=I, so covar=G*covar*G' + covarodo = covar + covarodo */
				
			covar(0,0) = covar(0,0) + fabs(deltamean(0))/covarodoscalar;
			covar(1,1) = covar(1,1) + fabs(deltamean(1))/covarodoscalar;
			covar(2,2) = covar(2,2) + fabs(deltamean(2))/covarodoscalar;
			
			
			/** publish filter_state topic /predictedPose */
			
			filterState.batteryPercent = navdata.batteryPercent;
			filterState.droneState = navdata.state;
						
			/** publish transform with drone pose */
			tf::Matrix3x3 R;
			R.setRPY( roll , pitch , yaw );
			tf::Transform temporarytransform( R, tf::Vector3( mean(0), mean(1), mean(2) ) );
			transform=temporarytransform;
						
			/** publish pose topic */
			Pose.linear.x = mean(0);
			Pose.linear.y = mean(1);
			Pose.linear.z = mean(2);
			
			Pose.angular.x = roll;
			Pose.angular.y = pitch;
			Pose.angular.z = yaw;
			
			publishAll();
			 
		 } /** end of timestamp condition */
	 } /** end of callback odometry (prediction step) */
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 /** declara callback */
	void observation(const visualization_msgs::MarkerArray markerarray)
	{
		
		/** get transform between /ardrone_base_bottomcam and /map */
		tf::StampedTransform invcamerapose;
	
		try
		{
			listener.waitForTransform("ardrone_base_bottomcam", "map", ros::Time(0), ros::Duration(10.0) ); 
			listener.lookupTransform("ardrone_base_bottomcam", "map", ros::Time(0), invcamerapose); /** gets the last published transformation */
		}
			
			catch (tf::TransformException ex)
			{
				ROS_INFO("exceção do lookupTransform entre bottomcam e map");
				ROS_ERROR("%s",ex.what());  /** handle errors */
			}
	
		tf::Matrix3x3 tfcRI = tf::Matrix3x3(invcamerapose.getRotation());  /** get rotation matrix from quaternion in transform */
		Eigen::Matrix3d cRI;
		
		for(int n=0; n<3; n++)
		{
			for(int m=0; m<3; m++)
			{
				cRI(n,m) = tfcRI[n][m];  /** convert tf::matrix3x3 into eigen::Matrix3d */
			}
		}
		
		Eigen::Vector3d IPb;
		IPb << mean(0), mean(1), mean(2); /** position of bodyframe in the inertial frame */
		
		
		int numbofmarkers; // number of observed markers
		numbofmarkers = markerarray.markers.size();
		
		for(int m=0; m < numbofmarkers; m++)    // for each observed marker
		{
				
			visualization_msgs::Marker marker;
			marker = markerarray.markers[m];
			
			Eigen::Vector3d Cpi; /** marker position in the camera frame */
			Cpi << marker.pose.position.x, marker.pose.position.y, marker.pose.position.z ; /** CONFERIR !!!!!!!!!!!!!!! */
					
			int j = -1;
				
			for (int i=0; i < list.size(); i++)    /** check if observed landmark is in the list of mapped landmarks */
			{
				if ( list(i) == marker.id )
				{
					j = i;
				}
			}
		
			
			if ( j >= 0 )/** if the observed marker is already in the list updates filter */
			{	
				Eigen::Vector3d Ipi;
				Ipi << map(2*j), map(1+2*j), 0.0; /** maped position of the observed landmark in the inertial frame */
				
				Eigen::Vector3d h;
				h = cRI * ( Ipi - IPb ); /** modeled expression of the observation (position of landmark in the camera frame) */
				
				Eigen::Vector3d obsErr;
				obsErr = (Cpi - h); /** observation model error */
				
				Eigen::Matrix3d covarobs; /** observation covariance */
				
				if (covarobsproportional)
				{
					covarobs << fabs(obsErr(0)), 0.0, 				0.0,
								0.0, 			fabs(obsErr(1)), 	0.0,
								0.0, 			0.0, 				fabs(obsErr(2));
					covarobs = covarobs * covarobsscalar;
				}
				else
				{
					covarobs << covarobsscalar, 	0.0, 		0.0,
								0.0, 		covarobsscalar, 	0.0,
								0.0, 				0.0, 	covarobsscalar;
				}

					
				Eigen::Matrix3d H = -cRI;
				
				Eigen::MatrixXd K;
				K = covar * (H.transpose()) * ( ( H * covar * (H.transpose()) + covarobs ).inverse() ); /** kalman gain */
				
				Eigen::MatrixXd temp;
				temp = K*H;
				
				/** update mean and covariance matrix */
				covar = ( Eigen::MatrixXd::Identity(temp.rows(), temp.cols()) - temp) * covar;
				mean = mean + K * (Cpi - h );
				// ROS_INFO("SLAM: observation update OK");
				
			} /** end of if condition: it is in the list */

		} /** end of for each marker for loop */
		
		
		/** publish transform with drone pose */
		transform.setOrigin( tf::Vector3( mean(0), mean(1), mean(2) ) ); /** set translation between /map and /ardrone_base_link */
		publishAll();
		
		
	} /** end of observation callback */









	void publishAll(void) /** function to publish tranformation pose and markers */
	{
		
		static tf::TransformBroadcaster broadcaster; /** declares object that will publish the pose between /map and /ardrone_base_link */

		try
		{
			broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "ardrone_base_link")); 			/** publishes the transform */
		}
		catch (tf::TransformException ex)
		{
			ROS_INFO("exceção do sendTransforms, linha 151");
			ROS_ERROR("%s",ex.what());  /** handle errors */
		}	
				 
		/** publish filter_state */
		predictedPosePub_.publish(filterState);
			
		/** publish Pose topic */
		PosePub_.publish(Pose);
		
				
		
		if(publishmap)
		{
			visualization_msgs::MarkerArray mapMarkers;
			visualization_msgs::Marker individualMarker;
			
			for (int i = 0 ; i < list.size() ; i++ ) /** for each marker in map*/
			{
				
				individualMarker.pose.position.x = map(2*i);
				individualMarker.pose.position.y = map(2*i +1);
				individualMarker.pose.position.z = 0;
				 
				individualMarker.pose.orientation.x = 0;
				individualMarker.pose.orientation.y = 0;
				individualMarker.pose.orientation.z = 0;
				individualMarker.pose.orientation.w = 1;
				
				individualMarker.header.frame_id = "map";
				individualMarker.header.stamp = ros::Time();
				individualMarker.id = list(i);
	
		        individualMarker.scale.x = 0.1;
		        individualMarker.scale.y = 0.1;
		        individualMarker.scale.z = 0.1;
		        individualMarker.ns = "basic_shapes";
		        individualMarker.type = visualization_msgs::Marker::SPHERE;
		        individualMarker.action = visualization_msgs::Marker::ADD;
				switch (list(i))
				{
					case 0:
		            individualMarker.color.r = 1.0f;
		            individualMarker.color.g = 0.0f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
		            
		            case 1:
		            individualMarker.color.r = 1.0f;
		            individualMarker.color.g = 0.33f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
		            
		            case 2:
		            individualMarker.color.r = 1.0f;
		            individualMarker.color.g = 0.66f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
				
		            case 3:
		            individualMarker.color.r = 1.0f;
		            individualMarker.color.g = 1.0f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
		          
		          
					case 4:
		            individualMarker.color.r = 0.66f;
		            individualMarker.color.g = 0.0f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
		            
		            case 5:
		            individualMarker.color.r = 0.66f;
		            individualMarker.color.g = 0.33f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
		            
		            case 6:
		            individualMarker.color.r = 0.66f;
		            individualMarker.color.g = 0.66f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
		
		            case 7:
		            individualMarker.color.r = 0.66f;
		            individualMarker.color.g = 1.0f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
		          
					case 8:
		            individualMarker.color.r = 0.33f;
		            individualMarker.color.g = 0.0f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
		            
		            case 9:
		            individualMarker.color.r = 0.33f;
		            individualMarker.color.g = 0.33f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
		            
		            case 10:
		            individualMarker.color.r = 0.33f;
		            individualMarker.color.g = 0.66f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
		
		            case 11:
		            individualMarker.color.r = 0.33f;
		            individualMarker.color.g = 1.0f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
		          
					case 12:
		            individualMarker.color.r = 0.0f;
		            individualMarker.color.g = 0.0f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
		            
		            case 13:
		            individualMarker.color.r = 0.0f;
		            individualMarker.color.g = 0.33f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
		            
		            case 14:
		            individualMarker.color.r = 0.0f;
		            individualMarker.color.g = 0.66f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
		
		            case 15:
		            individualMarker.color.r = 0.0f;
		            individualMarker.color.g = 1.0f;
		            individualMarker.color.b = 0.0f;
		            individualMarker.color.a = 1.0;
		            break;
				
					default:
		            individualMarker.color.r = 1.0f;
		            individualMarker.color.g = 1.0f;
		            individualMarker.color.b = 1.0f;
		            individualMarker.color.a = 1.0;
				} /** end of switch case */
				
		        individualMarker.lifetime = ros::Duration (1.0);
		
		        mapMarkers.markers.push_back(individualMarker);
			} /** end of for each marker for loop */					
			
			mapMarkerPub_.publish(mapMarkers);
		}/** end of if(publishmap) condition */ 
	 } /** end of function publishAll() */







	void publishvelmarker(const double linvx, const double linvy)
	{

		visualization_msgs::Marker VelocityMarker;

		VelocityMarker.pose.position.x = 0;
		VelocityMarker.pose.position.y = 0;
		VelocityMarker.pose.position.z = 0;
		
		VelocityMarker.header.frame_id = "ardrone_base_link";
		VelocityMarker.header.stamp = ros::Time();
        VelocityMarker.lifetime = ros::Duration (1.0);
		VelocityMarker.ns = "basic_shapes";
        VelocityMarker.type = visualization_msgs::Marker::ARROW;
        VelocityMarker.action = visualization_msgs::Marker::ADD;
		VelocityMarker.id = 0;
        
        double absolute;
		absolute = sqrtf(linvx*linvx + linvy*linvy);
        VelocityMarker.scale.x = absolute;
        VelocityMarker.scale.y = 0.01;
        VelocityMarker.scale.z = 0.01;  
            
		double theta;
		theta = asin(fabs(linvy)/absolute);
		if ((linvx < 0) && (linvy > 0))
		{
			theta = 3.1415 - theta;
		}
		else if ((linvx < 0) && (linvy < 0))
		{
			theta = theta - 3.1415;
		}
		else if ((linvx > 0) && (linvy < 0))
		{
			theta = - theta;
		}
		
		VelocityMarker.pose.orientation.x = 0.0;
		VelocityMarker.pose.orientation.y = 0.0;
		VelocityMarker.pose.orientation.z = sin(theta/2);  /** sin(theta/2) rotation of angle theta under the z axis */
		VelocityMarker.pose.orientation.w = cos(theta/2);  /** cos(theta/2) */
        
        velocityMarkerPub_.publish(VelocityMarker);
		
	} /** end of publishvelmarker() function) */




/** end of public part and beggining of private part for the class SubscribeAndPublish */

private:
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node. */
  
	ros::NodeHandle n_; 
	
	ros::Publisher predictedPosePub_;
	ros::Publisher PosePub_;
	ros::Publisher mapMarkerPub_;
	ros::Publisher covarMarkerPub_;
	ros::Publisher velocityMarkerPub_;
	
	ros::Subscriber navdataSub_;
	ros::Subscriber markerSub_;
	
} ; /** end of the class declaration */






/** loadmap function declaration */
void loadmap(const std::string &map_file_path)
{
	
	if (map_file_path=="")
	{
		ROS_INFO("ArDroneControl - Localization: No map loaded! only deadreckoning will be performed");
	}
	else
	{
		ROS_INFO("ArDroneControl - Localization: Loading Map");
		int i=0;
		std::ifstream infile(map_file_path.c_str());
		std::string line;
		while (std::getline(infile, line))
		{
		    std::istringstream iss(line);
		    double id, x, y;
		    if (!(iss >> id >> x >> y)) { break; } // error
			list.conservativeResize(i+1);
			list(i)=int(id);
			map.conservativeResize(2*(i+1));
			map(2*i)=x;
			map(2*i+1)=y;
			i++;
		}
		if (i>0)
		{
			ROS_INFO("ArDroneControl - Localization: map loaded");
		}
		else
		{
			ROS_INFO("ArDroneControl - Localization: problem loading map! only deadreckoning will be performed");
		}
	}
} /** end of loadmap() function */





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
   
	ros::init(argc, argv, "localization");
	
	ROS_INFO("ArDroneControl: localization started");
	
	/** get parameters */
	ros::NodeHandle private_node_handle("~");
	
	private_node_handle.param<double>("covarobsscalar", covarobsscalar, 0.2);
	ROS_INFO("ArDroneControl - Localization: covarobsscalar = %f", covarobsscalar);
	private_node_handle.param<double>("covarodoscalar", covarodoscalar, 100.0);
	ROS_INFO("ArDroneControl - Localization: covarodoscalar = %f", covarodoscalar);
	private_node_handle.param<bool>("publishmap", publishmap, true);
	ROS_INFO("ArDroneControl - Localization: publishmap = %d", publishmap);	
	private_node_handle.param<bool>("publishvelocitymarker", publishvelocitymarker, true);
	ROS_INFO("ArDroneControl - Localization: publishvelocitymarker = %d", publishvelocitymarker);	
	private_node_handle.param<bool>("covarobsproportional", covarobsproportional, false);
	ROS_INFO("ArDroneControl - Localization: covarobsproportional = %d", covarobsproportional);	
	private_node_handle.param<double>("covarinit", covarinit, 1000000.0);
	ROS_INFO("ArDroneControl - Localization: covarinit = %f", covarinit);
	
	
	/** initializes some variables */
	mean << 0.0 , 0.0 , 0.0;
	
	covar << covarinit,		0.0,	0.0,
			0.0, 		covarinit, 	0.0,
			0.0, 			0.0, covarinit;
			
	timestamp = 0;
	altd = 0.0;

	/** load map */
	std::string map_file_path;
	private_node_handle.param<std::string>("map_file_path", map_file_path, "");
	ROS_INFO("ArDroneControl - Localization: map_file_path = %s", map_file_path.c_str() );
	
	loadmap(map_file_path);
	
	/** creates an object of the declared class. this object will perform the comunication and calculation*/
	SubscribeAndPublish SAPObject;

	/** ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master. */
	
	ros::spin();

	return 0;
}




