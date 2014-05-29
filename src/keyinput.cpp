 #include <ros/ros.h>
 #include <signal.h>
 #include <termios.h>
 #include <stdio.h>
 #include <std_msgs/Char.h>

#define KEYCODE_L 0x6C // lock slam mapping and save map
#define KEYCODE_U 0x75 // unlock mapping
#define KEYCODE_R 0x72 // reset map

#define KEYCODE_M 0x6D // disable control back to manual, disable setpoint.
#define KEYCODE_G 0x67 // get startframe and setpoint to hold place in startframe, enable control
#define KEYCODE_0 0x30 // set startframe to origin and setpoint to hold place where it is, enable control
#define KEYCODE_P 0x70 // follow path relative to startframe, enable control

//voa aperta 0 e P


 

char prev;

 class keyinput
 {
 public:
   keyinput();
   void keyLoop();
 
 private:
   ros::NodeHandle nh_;
   ros::Publisher key_pub_;
   
 };
 
 
 
 keyinput::keyinput()
 {
   key_pub_ = nh_.advertise<std_msgs::Char>("/keyinput", 1);
 }

 int kfd = 0;
 struct termios cooked, raw; 
 
 
 
 void quit(int sig)
 {
   tcsetattr(kfd, TCSANOW, &cooked);
   ros::shutdown();
   exit(0);
 }



int main(int argc, char** argv)
 {
   ros::init(argc, argv, "keyinput");
   keyinput keyinput;
   signal(SIGINT,quit);
   keyinput.keyLoop();
   return(0);
 }
 
 
 
 void keyinput::keyLoop()
 {
   char c;
   // get the console in raw mode                                                              
   tcgetattr(kfd, &cooked);
   memcpy(&raw, &cooked, sizeof(struct termios));
   raw.c_lflag &=~ (ICANON | ECHO);
   // Setting a new line, then end of file                         
   raw.c_cc[VEOL] = 1;
   raw.c_cc[VEOF] = 2;
   tcsetattr(kfd, TCSANOW, &raw);
 
   puts("Reading from keyboard");
   puts("---------------------------");
   puts("Press L to lock map on SLAM node, or press C to use the controler node to keep drone in place");
   puts("Press U to unlock map on SLAM node, or press M to disable the controler node and put drone in manual");
 
   for(;;)
   {
     // get the next event from the keyboard  
     if(read(kfd, &c, 1) < 0)
     {
       perror("read():");
       exit(-1);
     }
 
     //ROS_INFO("value: 0x%02X\n", c);
   
     if(c != prev)
     {
	     switch(c)
	     {
	       case KEYCODE_L:
		 ROS_INFO("key L pressed");
		 break;      
		 
	       case KEYCODE_U:
		 ROS_INFO("key U pressed");
		 break;       

	       case KEYCODE_R:
		 ROS_INFO("key R pressed");
		 break;
	       
	       case KEYCODE_M:
		 ROS_INFO("key M pressed");
		 break;

	       case KEYCODE_G:
		 ROS_INFO("key G pressed");
		 break;

	       case KEYCODE_0:
		 ROS_INFO("key 0 pressed");
		 break;
		 
		   case KEYCODE_P:
		 ROS_INFO("key P pressed");
		 break;
	     }
	     std_msgs::Char key;
	     key.data = c;
	     key_pub_.publish(key);  
	}
	prev=c;
   }
   return;
}
