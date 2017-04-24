#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include <sstream>

// Compila con : simo@simo-VB:~/catkin_ws$ 	catkin_make
// Fallo girare: simo@simo-VB:~/catkin_ws$ 	rosrun beginner_tutorials HW1_listenNtalk 
// Vedi output : simo@simo-VB:~/catkin_ws$ 	rostopic echo /hw1_topic

//After: cd catkin_ws/ open tabs and copy and paste the following in each tab:

//roscore
//rosrun stage_ros stageros $(rospack find stage_ros)/world/willow-erratic.world
//rosrun teleop_twist_keyboard teleop_twist_keyboard.py
//roscd stage  	,		  rosrun rviz rviz -d `rospack find stage_ros`/rz/stage.rviz
//catkin_make
//rosrun beginner_tutorials HW1_listenNtalk
//rostopic echo hw1_topic
//rosed beginner_tutorials HW1_listenNtalk.cpp


class subscribeAndPublish {
	public:
		subscribeAndPublish() {
			pub_minDist = nodeH.advertise<std_msgs::Float32>("/hw1_topic", 1); //buffer length = 1 (one)
			sub = nodeH.subscribe<sensor_msgs::LaserScan>("/base_scan", 100, &subscribeAndPublish::waitForMessage, this); //Anche il 100 e' len buf
		}
		
		void waitForMessage(const sensor_msgs::LaserScan::ConstPtr &msg) { //Questa funzione e' considerata public!
			
			const float rangeMax = msg->range_max; // = 30.0
			const float rangeMin = msg->range_min; // =  0.0
			
			float distanzaMin = rangeMax;
			float distanzaNow = rangeMax;
			const int lenArray = (msg->ranges).size();
			int i = 0;
			
			while(i<lenArray && distanzaNow >= rangeMin) { //Se distanzaNow < 0.0, esco dal while.
				
				distanzaNow = (msg->ranges)[i];
				
				if(distanzaNow < rangeMin) { //Check su valori < 0.0
					distanzaMin = rangeMin; //Non tocco distanzaNow, che rimane < 0.0
				}
				else if(distanzaNow < distanzaMin) {
					distanzaMin = distanzaNow;
				}
				i++;
			}
			
			ROS_INFO("Range min = %f, range max = %f, dist min = %f", rangeMin, rangeMax, distanzaMin);
			
			std_msgs::Float32 pubMsg;
			pubMsg.data = distanzaMin;
			
			pub_minDist.publish(pubMsg);  //Publish!
		}
		
		
	private:
		ros::NodeHandle nodeH;
		ros::Publisher pub_minDist;
		ros::Subscriber sub;
	
};

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "HW1_listenNtalk");
	
	subscribeAndPublish sAPobj;
	
	ros::spin(); //bloccante, ros::spinOnce invece no (fa 1 spin e poi continua)
	// spin e spinOnce creano "dei thread"* che effettivamente fanno subscribe e poi pubblicano i msg
	
	//* spin and spinOnce are really meant for single-threaded applications => in un programma di questo tipo (ovvero che richiede 
	//	1 solo thread) c'e' o spin o spinOnce, perche' sia spin che spinOnce creano 1 thread!
	
	// Ogni nodeHandle (qui nodeH) ha una coda di callback associata.
	
	return 0;
}
