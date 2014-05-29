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
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include "ardrone_autonomy/Navdata.h"
#include <tum_ardrone/filter_state.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


#define KEYCODE_L 0x6C // lock slam mapping and save map
#define KEYCODE_U 0x75 // unlock mapping
#define KEYCODE_R 0x72 // reset map

#define KEYCODE_M 0x6D // disable control back to manual, disable setpoint.
#define KEYCODE_G 0x67 // get startframe and setpoint to hold place in startframe, enable control
#define KEYCODE_0 0x30 // set startframe to origin and setpoint to hold place where it is, enable control
#define KEYCODE_P 0x70 // follow path relative to startframe, enable control

/** parameters  */
double covarobsscalar; /** relates to the observation covariance */
double newmarkcovarobsscalar; /** relates to the observation covariance of new markers */
double covarodoscalar; /** relates to the odometry covariance */
bool publishmap; /** option to publish map markers or not */
bool publishvelocitymarker; /** option to publish arrows representing linear velocitys */
bool covarobsproportional; /** option to calculate the observation covariance proportional to the estimated observation error */
double covarinit; /** initial value for the covariance */
std::string map_file_path;

/** GLOBAL VARIABLES */
Eigen::VectorXd mean(3); /** MUST BE RETENTIVE filter gaussian function mean (also the estimated position) mean(1)=X mean(2)=Y */
Eigen::MatrixXd covar(3,3); /** MUST BE RETENTIVE filter covariance matrix */
uint64_t timestamp; /** drone's timestamp buffer MUST BE RETENTIVE*/
double altd;

double prevyaw;
double yawoffset; /** to fix the weird yaw jump done by the ardrone firmware when it is put to fly */

Eigen::VectorXi list(1);
tum_ardrone::filter_state filterState; /** tum filter state. this node only uses it to interface with tum_ardrone node drone_gui */
tf::Transform transform; /** the tranform between "map" and "ardrone_base_link" witch is the pose predicted by Kalman filter*/
geometry_msgs::Twist Pose;
bool lockmap;
	
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
		
		
		mapMarkerPub_ = n_.advertise<visualization_msgs::MarkerArray>("/map/mapMarkerArray",1); /** array with the position of mapped landmarks */
		// covarMarkerPub_ = n_.advertise<visualization_msgs::MarkerArray>("/map/mapMarkerArray",1); /** array with the covariance of mapped landmarks */	
		uncertaintyPub_ = n_.advertise<geometry_msgs::Vector3>("/FilterUncertainty", 1);
		PosePub_ = n_.advertise<geometry_msgs::Twist>("/Pose", 1);
		predictedPosePub_ = n_.advertise<tum_ardrone::filter_state>("/ardrone/predictedPose", 1);
	    velocityMarkerPub_ = n_.advertise<visualization_msgs::Marker>("/velocityMarker",1); /** arrow representing linear velocitys */
	
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
		keySub_ = n_.subscribe("/keyinput", 1, &SubscribeAndPublish::keyCallBack, this);
		navdataSub_ = n_.subscribe("ardrone/navdata", 1, &SubscribeAndPublish::odometry, this);
		markerSub_ = n_.subscribe("visualization_marker_array", 1, &SubscribeAndPublish::observation, this);
	} 
	
	
	
		void keyCallBack(const std_msgs::Char key)
	{
		switch(key.data)
		{
			case KEYCODE_L:
				lockmap =1;
				ROS_INFO("ArDroneControl - Slam: map locked");
				savemap(map_file_path);
				break;   

			case KEYCODE_U:
				lockmap =0;
				ROS_INFO("map unlocked");
		        break;  
		        
		   	case KEYCODE_R:
				double tempaltd;
				tempaltd = mean(2);
				mean.conservativeResize(3);
				mean << 0.0 , 0.0 , tempaltd;
				double tempcovar;
				tempcovar = covar(2,2);
				covar.conservativeResize( 3 , 3);
				covar << 0.0,		0.0,	0.0,
						0.0, 		0.0, 	0.0,
						0.0, 		0.0,	tempcovar;
				list.conservativeResize(1);
				list << -1;   	
				ROS_INFO("map reseted");
		        break;  
		        
		}
	}
				
	
	 /** save map functions */
	void savemap(const std::string &map_file_path)
	{
		Eigen::MatrixXd m(list.size(),3);
		for (int i=0; i<list.size(); i++)
		{
			m(i,0) = list(i);
			m(i,1) = mean(2*i + 3);
			m(i,2) = mean(2*i + 4);
		}
		
		std::ofstream file(map_file_path.c_str());
		if (file.is_open())
		{
			file << m << '\n';
		}
		file.close();
		ROS_INFO("ArDroneControl - Slam: map saved to path = %s", map_file_path.c_str() ); 
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
			if (fabs((yaw - prevyaw)) > 30*3.1415/180) /** if yaw jump happens */
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
				Ipi << mean(3+2*j), mean(4+2*j), 0.0; /** maped position of the observed landmark in the inertial frame */
				
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
							
				Eigen::Matrix3d Hb = -cRI;
				Eigen::MatrixXd Hi(3,2);
				Hi << cRI(0,0), cRI(0,1),
					  cRI(1,0), cRI(1,1),
					  cRI(2,0), cRI(2,1);
					  
				Eigen::MatrixXd H;
				H = Eigen::MatrixXd::Zero(3, mean.size() );
				H.block(0,0,3,3) = Hb;
				
				if (!lockmap)
				{		
					H.block(0 , 3+2*j , 3 , 2) = Hi;   /** observation hessian */	
				}
				
				Eigen::MatrixXd K;
				K = covar * (H.transpose()) * ( ( H * covar * (H.transpose()) + covarobs ).inverse() ); /** kalman gain */
				
				Eigen::MatrixXd temp;
				temp = K*H;
				
				
				/** update mean and covariance matrix */
				covar = ( Eigen::MatrixXd::Identity(temp.rows(), temp.cols()) - temp) * covar;
				mean = mean + K * (Cpi - h );
				// ROS_INFO("SLAM: observation update OK");
				
			} /** end of if condition: it is in the list */
			
			
			
			
			
			
			
			
			else /** if observed marker has not been seen yet extends covar matrix and state vector to map new marker*/
			{
				if (!lockmap)
				{			
					ROS_INFO("SLAM: NEW LANDMARK observed = %d ", marker.id );
	
					if (list(0) < 0) /** if no marker has been seen yet resets xy position and covariance*/
					{
						mean(0) = 0.0;
						mean(1) = 0.0;
						covar(0,0) = 0.0;
						covar(1,1) = 0.0;
						
						list(0) = marker.id;
					}
					else  /** if at least one marker has been already seen */
					{
						list.conservativeResize( list.size() + 1) ;
						list(list.size()-1) = marker.id ;
					}
	
					/** get transform between /map and /ardrone_base_bottomcam */
					tf::StampedTransform camerapose;
		
					try
					{
						listener.waitForTransform("map", "ardrone_base_bottomcam", ros::Time(0), ros::Duration(10.0) );
						listener.lookupTransform("map", "ardrone_base_bottomcam", ros::Time(0), camerapose); /** gets the last published transformation */
					}
						
						catch (tf::TransformException ex)
						{
							ROS_INFO("exceção do lookupTransform entre map e bottomcam");
							ROS_ERROR("%s",ex.what());  /** handle errors */
						}
						
					
					tf::Matrix3x3 tfIRc = tf::Matrix3x3(camerapose.getRotation());  /** get rotation matrix from quaternion in transform */
					Eigen::Matrix3d IRc;
					for(int n=0; n<3; n++)
					{
						for(int m=0; m<3; m++)
						{
							IRc(n,m) = tfIRc[n][m]; /** convert tf::matrix3x3 into eigen::Matrix3d */
						}
					}
					
					Eigen::Vector3d t;
					t << camerapose.getOrigin().x(), camerapose.getOrigin().y(), camerapose.getOrigin().z();
					/** get marker position in /map frame (inertial frame)*/
					Eigen::Vector3d Ipi;
					Ipi = IRc * Cpi + t;		
					
					Eigen::Matrix3d covarobs; /** observation covariance */
					covarobs << newmarkcovarobsscalar , 0.0, 			0.0,
								0.0 , 		newmarkcovarobsscalar, 		0.0,
								0.0 , 					0.0, 	newmarkcovarobsscalar;
					
					/** EXTEND MEAN VECTOR AND COVAR MATRIX */
					mean.conservativeResize(mean.size()+2);
					mean(mean.size()-2) = Ipi(0);
					mean(mean.size()-1) = Ipi(1);
					Eigen::MatrixXd Fz(2,3);
					Fz << IRc(0,0), IRc(0,1), IRc(0,2),
							IRc(1,0), IRc(1,1), IRc(1,2);
							
					covar.conservativeResize( covar.rows() + 2 , covar.cols() + 2 );
					covar.block(0, covar.cols()-2, covar.rows()-2, 2 ) = covar.block(0, 0, covar.rows()-2, 2);
					covar.block(covar.rows()-2, 0, 2, covar.cols()-2 ) = covar.block(0, 0, 2, covar.cols()-2);
					covar.block(covar.rows()-2, covar.cols()-2, 2, 2) = Fz * covarobs * (Fz.transpose() );
					
				}/** end of not locked map condition*/			
			} /** end of else condition: it is not in the list */			
		} /** end of for each marker for loop*/
		
		
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
		
		/** publish uncertainties */
		double positionUncert;
		double mapUncert;
		double Uncertainty;
		
		positionUncert = (covar.block(0, 0, 3, 3)).determinant();
		mapUncert = (covar.block(3, 3, covar.rows()-3, covar.cols()-3)).determinant();
		Uncertainty = covar.determinant();
		geometry_msgs::Vector3 incertesa;
		incertesa.x = positionUncert;
		incertesa.y = mapUncert;
		incertesa.z = Uncertainty;
		
		uncertaintyPub_.publish(incertesa);
		
		 
		/** publish filter_state */
		predictedPosePub_.publish(filterState);
		
		/** publish Pose topic */
		PosePub_.publish(Pose);
		
		
		 if (( mean.size() > 3 ) && publishmap)  /** if a landmark has been mapped, publish visualization markes */
		 {
			visualization_msgs::MarkerArray mapMarkers;
			visualization_msgs::Marker individualMarker;
			
			for (int i = 0 ; i < list.size() ; i++ )
			{
				
				individualMarker.pose.position.x = mean(2*i + 3);
				individualMarker.pose.position.y = mean(2*i + 4);
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
				}
				
		        individualMarker.lifetime = ros::Duration (1.0);
		
		        mapMarkers.markers.push_back(individualMarker);
			}					
			
			mapMarkerPub_.publish(mapMarkers);
			
			//ROS_INFO("SLAM: markers published after odometry step");
		 } /** end of if condition to publish visualization markers */ 
		
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


	 
	 
	 
	 
	 
	 
	 
	 

/** private part for the class SubscribeAndPublish */

private:
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node. */
  
	ros::NodeHandle n_; 
	
	ros::Publisher uncertaintyPub_;
	ros::Publisher PosePub_;
	ros::Publisher predictedPosePub_;
	ros::Publisher mapMarkerPub_;
	ros::Publisher covarMarkerPub_;
	ros::Publisher velocityMarkerPub_;
	
	ros::Subscriber keySub_;
	ros::Subscriber navdataSub_;
	ros::Subscriber markerSub_;
	
	

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
   
	ros::init(argc, argv, "slam");
	
	ROS_INFO("ArDroneControl: slam started");
		/** get parameters */
	ros::NodeHandle private_node_handle("~");
	
	private_node_handle.param<double>("covarobsscalar", covarobsscalar, 0.2);
	ROS_INFO("ArDroneControl - Slam: covarobsscalar = %f", covarobsscalar);
	private_node_handle.param<double>("newmarkcovarobsscalar", newmarkcovarobsscalar, 0.9);
	ROS_INFO("ArDroneControl - Slam: newmarkcovarobsscalar = %f", newmarkcovarobsscalar);
	private_node_handle.param<double>("covarodoscalar", covarodoscalar, 100.0);
	ROS_INFO("ArDroneControl - Slam: covarodoscalar = %f", covarodoscalar);
	private_node_handle.param<bool>("publishmap", publishmap, true);
	ROS_INFO("ArDroneControl - Slam: publishmap = %d", publishmap);	
	private_node_handle.param<bool>("publishvelocitymarker", publishvelocitymarker, true);
	ROS_INFO("ArDroneControl - Slam: publishvelocitymarker = %d", publishvelocitymarker);	
	private_node_handle.param<bool>("covarobsproportional", covarobsproportional, false);
	ROS_INFO("ArDroneControl - Slam: covarobsproportional = %d", covarobsproportional);	
	private_node_handle.param<double>("covarinit", covarinit, 1000000.0);
	ROS_INFO("ArDroneControl - Slam: covarinit = %f", covarinit);
	private_node_handle.param<std::string>("map_file_path", map_file_path, "~/map");
	ROS_INFO("ArDroneControl - Slam: map_file_path = %s", map_file_path.c_str() );
	
	mean << 0.0 , 0.0 , 0.0;
	
	covar << covarinit,		0.0,	0.0,
			0.0, 		covarinit, 	0.0,
			0.0, 			0.0, covarinit;
	list << -1;
		
	timestamp = 0;
	altd = 0.0;

	
	prevyaw=0;
	yawoffset=0; /** to fix the weird yaw jump done by the ardrone firmware when it is put to fly */

	
	/** creates an object of the declared class. this object will perform the comunication and calculation*/
	SubscribeAndPublish SAPObject;

	/** ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master. */
	
	ros::spin();

	return 0;
}
