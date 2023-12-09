#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "mavros_msgs/Thrust.h"
#include <unistd.h>
#include "PWM.h"
#include "RCOutput_Navio.h"
#include "RCOutput_Navio2.h"
#include "Util.h"
#include <unistd.h>
#include <memory>

using namespace Navio;

int main(int argc, char **argv)
{
   ros::init(argc, argv, "pwm_node_pub");
   ros::NodeHandle n;
 
   ros::Publisher thrust_pub = n.advertise<mavros_msgs::Thrust>("/mavros/setpoint_attitude/thrust",100);
   ros::Rate loop_rate(100);
   ros::spinOnce();
 
   mavros_msgs::Thrust msg;
   int count = 1;
     
        //PositionReciever qp;:
        //Body some_object;
        //qp.connect_to_server();
 
     
   while(ros::ok()){
       //some_object = qp.getStatus();
        // some_object.print();
        //printf("%f\n",some_object.position_x);
       msg.header.stamp = ros::Time::now();
       msg.header.seq=count;
       msg.header.frame_id = 1;
       msg.thrust = 0.5;
 
       thrust_pub.publish(msg);
       ros::spinOnce();
       count++;
       loop_rate.sleep();
   }
    
       
   return 0;
}