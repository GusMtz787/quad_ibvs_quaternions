#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "mavros_msgs/Thrust.h"
#include <unistd.h>
#include <memory>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen3/Eigen/Dense>

static float thrust_coefficient = 0.0000132;
static float torque_coefficient = 0.000000217;
static float L = 0.23; // distance from QUAV center to propeller center in meters
static float sin_pi_4 = 0.7071;

float thrust = 0.0;
Eigen::Vector3f tau = {0.0, 0.0, 0.0};
Eigen::Vector4f omega_squared = {0.0, 0.0, 0.0, 0.0};
Eigen::Vector4f control_inputs = {0.0, 0.0, 0.0, 0.0};
Eigen::Vector4f pwm_signal = {0.0, 0.0, 0.0, 0.0};

// Considering u = Ax with 
// u as [thrust, vector of torques]
// A as the matrix relating u and x
// x as the Omega squared (velocities squared) vector
static Eigen::Matrix4f A = (Eigen::Matrix4f() << thrust_coefficient, thrust_coefficient, thrust_coefficient, thrust_coefficient,
    L * sin_pi_4 * thrust_coefficient, L * sin_pi_4 * thrust_coefficient, -L * sin_pi_4 * thrust_coefficient, -L * sin_pi_4 * thrust_coefficient,
    L * sin_pi_4 * thrust_coefficient, -L * sin_pi_4 * thrust_coefficient, -L * sin_pi_4 * thrust_coefficient, L * sin_pi_4 * thrust_coefficient,
    -torque_coefficient, torque_coefficient, -torque_coefficient, torque_coefficient).finished();

///////////////////////////////////////////////////////////////////
//////////////////// Callback Functions ///////////////////////////
///////////////////////////////////////////////////////////////////
void ThrustInputCallback(const std_msgs::Float64::ConstPtr& thr)
{
	thrust = thr->data;
}

void TorqueInputsCallback(const geometry_msgs::Vector3::ConstPtr& tor)
{
	tau(0) = tor->x;
	tau(1) = tor->y;
	tau(2) = tor->z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pwm_node_pub");
    ros::NodeHandle nh;

    ros::Publisher pwm_values_pub = nh.advertise<geometry_msgs::Quaternion>("pwm_values",100);

	geometry_msgs::Quaternion pwm_values;

    ros::Subscriber thrust_sub = nh.subscribe("quad_thrust",100,&ThrustInputCallback);
	ros::Subscriber quad_torques_sub = nh.subscribe("quad_torques",100,&TorqueInputsCallback);

    //auto pwm = get_rcout();

    ros::Rate loop_rate(200);
    ros::spinOnce();
     
    while(ros::ok()) {
        
        control_inputs(0) = thrust;
        control_inputs(1) = tau(0);
        control_inputs(2) = tau(1);
        control_inputs(3) = tau(2);

        omega_squared << A.inverse() * control_inputs;

        pwm_signal(0) = -0.00000000137 * powf(omega_squared(0), 2) + 0.00226 * omega_squared(0) + 1076.6;
        pwm_signal(1) = -0.00000000137 * powf(omega_squared(1), 2) + 0.00226 * omega_squared(1) + 1076.6;
        pwm_signal(2) = -0.00000000137 * powf(omega_squared(2), 2) + 0.00226 * omega_squared(2) + 1076.6;
        pwm_signal(3) = -0.00000000137 * powf(omega_squared(3), 2) + 0.00226 * omega_squared(3) + 1076.6;

        pwm_values.w = pwm_signal(0);
        pwm_values.x = pwm_signal(1);
        pwm_values.y = pwm_signal(2);
        pwm_values.z = pwm_signal(3);

        std::cout << pwm_values << std::endl;

        pwm_values_pub.publish(pwm_values);

        ros::spinOnce();
        loop_rate.sleep();
    }
        
    return 0;
}