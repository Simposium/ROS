#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <sstream>


tf::TransformListener* pListener = NULL; //globale: Dovendo essere usato sia nella funzione che nel main, viene implementato come puntatore
//tf::StampedTransform* pTransform = NULL;	//non serve globale, perche' viene usato solo nella funzione waitForMessage()

void waitForMessage(const sensor_msgs::LaserScan::ConstPtr &msg) { //Questa funzione si attiva quando arriva un msg LaserScan
	
	tf::StampedTransform transform;
	ros::Time timeZero = ros::Time::now(); //Meglio di ros::Time(0), viene preso una volta sola per avere tutti i dati presi allo stesso tempo.
	
	pListener->waitForTransform("/odom", "/base_laser_link", timeZero, ros::Duration(10.0) ); //odom e' il punto di partenza del robot
	pListener->lookupTransform("/odom", "/base_laser_link", timeZero, transform);
	
	//Gli unici 2 dati effettivi che prendo da msg sono:
	int seconds = (msg->header).stamp.sec;
	int nseconds = (msg->header).stamp.nsec;
	
	tf::Quaternion quo = transform.getRotation();
	
	float x, y; 
	double theta;
	//All'avvio di Stage: (0.050000,0.000000,0.000000); il robot (e quindi il laser) punta verso sinistra.
	
	x = transform.getOrigin().x(); //x denota lo spostamento avanti-dietro del robottino (e quindi sinistra-destra del robot w.r.t. mappa), dove (0,0) e' il punto di partenza.
	y = transform.getOrigin().y(); //y denota lo spostamento sinistra-destra del robottino (e quindi giu'-su del robot w.r.t mappa).
	//Robot va avanti (a sinistra w.r.t mappa) -> numeri positivi x; Robot va a sinistra (giu' w.r.t. mappa) -> numeri positivi y.
	/*
	
	x		  __
	<-------<|__|
			  |
			  |
			  |
			  |
			  V y
	
	*/
	//float z = transform.getOrigin().z(); //Si puo' ottenere anche l'altezza del laser z() che risulta essere 0.25
	
	
	theta = tf::getYaw(quo); //Quarda in alto w.r.t. mappa (rot 90° a dx) : -1.6,  Quarda a destra w.r.t. mappa (rot 180°) : 3.1
	
	//90° a dx = -1.6; 90° a sx = 1.6; Va da -3.14 a 3.14 circa. L'angolo e' in radianti! (pi) = 3.14; (pi)/2 = 1.57
	
	ROS_INFO("LASER \t%d.%9d \t(%f,%f,%lf)", seconds, nseconds, x, y, theta);
	
}
		
int main(int argc, char **argv) {
	
	ros::init(argc, argv, "HW2_laser2Dloc");
	
	pListener = new tf::TransformListener;
	//pTransform = new tf::StampedTransform;	//stessa idea usata per pListener, ma non e' necessaria in questo caso.
	
	ros::NodeHandle nodeH;
	ros::Subscriber sub = nodeH.subscribe<sensor_msgs::LaserScan>("/base_scan", 1, waitForMessage); //Buffer len dei msgs in arrivo va bene = 1
	// A callback is a function that you do NOT call yourself.
	// La funzione waitForMessage e' una funzione callback. Infatti, dopo aver definito la waitForMessage, viene passata come parametro
	// a subscribe. NON va mai chiamata direttamente, ma viene chiamata da ros quando arriva un messaggio msg.
	// Tutto cio' avviene all'esecuzione di ros::spin() (oppure spinOnce) che legge la coda di callbacks e le chiama una a una.
	
	ros::spin(); //ros::spin() contiene un loop infinito fino a che non viene ucciso (es. da ctrl+C).
	
	return 0;
}
