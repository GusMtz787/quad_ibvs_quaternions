//Including ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

using namespace std;

//Declaring global variables
/////////////////Error and Error dot variables///////////////
Eigen::Vector3f error;
Eigen::Vector3f error_dot;
Eigen::Vector3f tgt_vel;
Eigen::Vector3f tgt_accel;
Eigen::Vector3f quad_att;
Eigen::Vector3f quad_pos;
Eigen::Vector3f quad_desired_pos;
Eigen::Vector3f quad_vel_BF;
Eigen::Vector3f quad_vel_VF;
Eigen::Vector3f quad_desired_vel;
Eigen::Vector3f quad_attVel;

////////////////////Sliding surface and ASMC///////////////////
Eigen::Vector3f ss;
Eigen::Vector3f xi_1;
Eigen::Vector3f lambda;
Eigen::Vector3f xi_2;
Eigen::Vector3f varpi;
Eigen::Vector3f vartheta;
Eigen::Vector3f asmc;
Eigen::Vector3f K1;
Eigen::Vector3f K1_dot;
Eigen::Vector3f K2;
Eigen::Vector3f k_reg;
Eigen::Vector3f kmin;
Eigen::Vector3f mu;
Eigen::Vector3f alpha;
Eigen::Vector3f beta;

Eigen::Vector3f accelerations_desired(0.0, 0.0, 0.0);
Eigen::Vector3f velocities_desired(0.0, 0.0, 0.0);
Eigen::Vector3f e3;

//////////////////////////Desired attitude for the quadrotor//////////
Eigen::Vector3f attitude_desired;
float yawRate_desired = 0;
float roll_des_arg;
float pitch_des_arg;

float step_size = 0.01;
float quad_mass = 1.3;
float gravity = 9.81;
float thrust = quad_mass * gravity;
/////////////////////////Functions///////////////////////////////
Eigen::Matrix3f skewMatrix(Eigen::Vector3f vector)
{
	Eigen::Matrix3f Skew;
	Skew << 0, -vector(2), vector(1),
			vector(2), 0, -vector(0),
			-vector(1), vector(0), 0;			
	return Skew;
}

Eigen::Matrix3f Ryaw(float yaw)
{   
    Eigen::Matrix3f yaw_mat;
    yaw_mat << cos(yaw), -sin(yaw),0,
                sin(yaw), cos(yaw), 0,
                0, 0, 1;
    return yaw_mat;
}

Eigen::Matrix3f Rtp(float roll, float pitch)
{
    Eigen::Matrix3f pitch_mat;
    pitch_mat << cos(pitch), 0.0, sin(pitch),
        0.0, 1.0, 0.0,
        -sin(pitch), 0.0, cos(pitch);


    Eigen::Matrix3f roll_mat;
    roll_mat << 1.0, 0.0, 0.0,
        0.0, cos(roll), -sin(roll),
        0.0, sin(roll), cos(roll);

    Eigen::Matrix3f R_tp;
    R_tp = pitch_mat * roll_mat;

    return R_tp;

}

float sign(float var)
{
    float result;
    if(var>0)
    {
        result = 1;
    }
    else if(var<0)
    {
        result = -1;
    }
    else if (var == 0)
    {
        result = 0;
    }
    return result;
}


/////////////ROS Subscribers//////////////////////////////////
void tgtVelCallback(const geometry_msgs::Vector3::ConstPtr& tgtVel)
{
	tgt_vel(0) = tgtVel->x;
    tgt_vel(1) = tgtVel->y;
    tgt_vel(2) = tgtVel->z;
}

void quadVelBFCallback(const geometry_msgs::Vector3::ConstPtr& quadVel)
{
	quad_vel_BF(0) = quadVel->x;
    quad_vel_BF(1) = quadVel->y;
    quad_vel_BF(2) = quadVel->z;
}

void quadAttVelCallback(const geometry_msgs::Vector3::ConstPtr& quadAttVel)
{
	quad_attVel(0) = quadAttVel->x;
    quad_attVel(1) = quadAttVel->y;
    quad_attVel(2) = quadAttVel->z;
}

void quadAttCallback(const geometry_msgs::Quaternion::ConstPtr& quadAttQuaternion)
{
	roll  = atan2(2.0 * (quadAttQuaternion->w * quadAttQuaternion->y + quadAttQuaternion->w * quadAttQuaternion->x) , 1.0 - 2.0 * (quadAttQuaternion->x * quadAttQuaternion->x + quadAttQuaternion->y * quadAttQuaternion->y));
	// if (isnan(roll)) {
	// 	roll = 0.0;
	// }
    pitch = asin(2.0 * (quadAttQuaternion->y * quadAttQuaternion->w - quadAttQuaternion->z * quadAttQuaternion->x));
	// if (isnan(pitch)) {
	// 	pitch = 0.0;
	// }
    yaw = atan2(2.0 * (quadAttQuaternion->z * quadAttQuaternion->w + quadAttQuaternion->x * quadAttQuaternion->y) , - 1.0 + 2.0 * (quadAttQuaternion->w * quadAttQuaternion->w + quadAttQuaternion->x * quadAttQuaternion->x));
    // if (isnan(yaw)) {
    //     yaw = 0.0;
    // }
}

void tgtAccelCallback(const geometry_msgs::Vector3::ConstPtr& tgtAccel)
{
	tgt_accel(0) = tgtAccel->x;
    tgt_accel(1) = tgtAccel->y;
    tgt_accel(2) = tgtAccel->z;
}

void quadPosCallback(const geometry_msgs::Vector3::ConstPtr& quadPos)
{
	quad_pos(0) = quadPos->x;
    quad_pos(1) = quadPos->y;
    quad_pos(2) = quadPos->z;
}

/////////////////////////////////Main Program//////////////////////////
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "ibvs_pos_ctrl_VICON");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	
    
    //ROS publishers and subscribers
    ros::Publisher error_pub = nh.advertise<geometry_msgs::Vector3>("error",100);
    ros::Publisher error_dot_pub = nh.advertise<geometry_msgs::Vector3>("error_dot",100);
    ros::Publisher adaptive_gain_pub = nh.advertise<geometry_msgs::Vector3>("adaptive_gain",100);
    ros::Publisher asmc_pub = nh.advertise<geometry_msgs::Vector3>("asmc_output",100);
    ros::Publisher ss_pub = nh.advertise<geometry_msgs::Vector3>("ibvs_ss",100);
    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float64>("quad_thrust",100);
    ros::Publisher desired_att_pub = nh.advertise<geometry_msgs::Vector3>("desired_attitude",100);

    ros::Publisher accelerations_control = nh.advertise<geometry_msgs::Vector3>("accelerations_control",100);
    geometry_msgs::Vector3 accelerations_des_var;

    geometry_msgs::Vector3 error_var;
    geometry_msgs::Vector3 error_dot_var;
    geometry_msgs::Vector3 adaptive_gain_var;
    geometry_msgs::Vector3 asmc_var;
    geometry_msgs::Vector3 ss_var;
    std_msgs::Float64 thrust_var;
    std_msgs::Float64 z_des_var;
    geometry_msgs::Vector3 desired_attitude_var;

    ros::Subscriber quad_vel_BF_sub = nh.subscribe("velocity_estimates", 100, &quadVelBFCallback); // Check if should be Body Frame or Inertial Frame
    ros::Subscriber quad_attVel_sub = nh.subscribe("attVel_estimates", 100, &quadAttVelCallback);
    ros::Subscriber quad_att_sub = nh.subscribe("attitude_QUAV", 100, &quadAttCallback);
    ros::Subscriber position_quad_sub = nh.subscribe("position_QUAV", 100, &quadPosCallback);

    xi_1 << 4, 4, 6;
    lambda << 2.5, 2.5, 2;
    xi_2 << 3, 3, 3;
    varpi << 4, 4, 4;
    vartheta << 3, 3, 3;
    K1 << 0, 0, 0;
    K2 << 0.1, 0.1, 0.4;
    k_reg << 0.05, 0.05, 0.5;
    kmin << 0.01, 0.01, 0.01;
    mu << 0.05, 0.05, 0.1;
    //alpha << 0.008, 0.007, 0.5;
    alpha << 0.008, 0.008, 0.5;
    beta << 10, 10, 10;

    Eigen::Vector3f Kp(0.7, 0.7, 3.5);
    Eigen::Vector3f Kd(0.9, 0.9, 3.0);
    
    e3 << 0,0,1;
    attitude_desired << 0.0, 0.0, 0.0;
    quad_desired_pos << 0.0, 0.0, -2.0;
    quad_desired_vel << 0.0, 0.0, 0.0;

    thrust_var.data = thrust;
    thrust_pub.publish(thrust_var);

    desired_attitude_var.x = 0;
    desired_attitude_var.y = 0;
    desired_attitude_var.z = 0;
	desired_att_pub.publish(desired_attitude_var);
    ros::Duration(0.01).sleep();

    while(ros::ok()) {   

        // Calculate errors
        // error = quad_desired_pos - quad_pos;
        // error_dot = quad_desired_vel - quad_vel_BF;
        
        // Sliding surfaces and adaptive sliding mode controller
        // for (int i = 0; i<=2; i++)
        // {
        //     ss(i) = error(i) + xi_1(i) * powf(std::abs(error(i)),lambda(i)) * sign(error(i)) + xi_2(i) * powf(std::abs(error_dot(i)),(varpi(i)/vartheta(i))) * sign(error_dot(i));
            
        //     // *************** Traditional adaptive law ***************
        //     // if (K1(i)>kmin(i))
        //     // {
        //     //     K1_dot(i) = k_reg(i)*sign(std::abs(ss(i))-mu(i));
        //     // }
        //     // else
        //     // {
        //     //     K1_dot(i) = kmin(i);
        //     // }

        //     // K1(i) = K1(i) + step_size*K1_dot(i);
        //     // asmc(i) = -K1(i) * powf(std::abs(ss(i)),0.5) * sign(ss(i)) - K2(i) * ss(i);

        //     // *************** Modified adaptive law ***************
        //     K1_dot(i) = sqrt(alpha(i)) * sqrt(std::abs(ss(i))) - sqrt(beta(i)) * powf(K1(i),2);

        //     K1(i) = K1(i) + step_size*K1_dot(i);
        //     asmc(i) = -2 * K1(i) * sqrt(std::abs(ss(i))) * sign(ss(i)) - (pow(K1(i),2) / 2) * ss(i);
        // }
        
        // // Thrust calculation
        // thrust = -(quad_mass / cos(quad_att(0)) * cos(quad_att(1))) * (accelerations_desired(2) - gravity 
        //             - asmc(2) + ((1 / xi_2(2)*(varpi(2)/vartheta(2))) * powf(abs(error_dot(2)), (2 - (varpi(2)/vartheta(2)))) * sign(error_dot(2))) * 
        //             (1 + xi_1(2) * lambda(2) * powf(abs(error_dot(2)), (lambda(2) - 1))));
        
        // if (thrust > 30)
        // {
        //     thrust = 30;
        // }
        // else if (thrust < 0)
        // {
        //     thrust = 0;
        // }

        error = quad_desired_pos - quad_pos;
        error_dot = quad_desired_vel - quad_vel_BF;

        // Thrust calculation 
        thrust = -(quad_mass / (cos(quad_att(0))*cos(quad_att(1)))) * (accelerations_desired(2) + gravity - Kp(2)*error(2) - Kd(2)*error_dot(2));
        
        if (thrust < -30.0) {
            thrust = -30.0;
        }
        else if (thrust > 0.0) {
            thrust = 0.0;
        }

        /////////////////Desired attitude//////////////////   
        attitude_desired(2) = 0.0; // For now, yaw is fixed to 0     
        
        roll_des_arg = (quad_mass / thrust) * (sin(attitude_desired(2))*(Kp(0)*error(0) + Kd(0)*error_dot(0)) - cos(attitude_desired(2))*(Kp(1)*error(1) + Kd(1)*error_dot(1)));
        
        if (roll_des_arg > 1) {
            roll_des_arg = 1;
        }
        else if (roll_des_arg < -1) {
            roll_des_arg = -1;
        }
        
        attitude_desired(0) = asin(roll_des_arg); //Roll desired

        pitch_des_arg = ((quad_mass / thrust) * (Kp(0)*error(0) + Kd(0)*error_dot(0)) - sin(attitude_desired(2))*sin(attitude_desired(0))) / (cos(attitude_desired(2))*cos(attitude_desired(0)));

        //////////////////Saturating the desired roll and pitch rotations up to pi/2 to avoid singularities
        if (pitch_des_arg > 1) {
            pitch_des_arg = 1;
        }
        else if (pitch_des_arg < -1) {
            pitch_des_arg = -1;
        }

        attitude_desired(1) = asin(pitch_des_arg); //Pitch desired

        //Publishing data
        //error
        error_var.x = error(0);
        error_var.y = error(1);
        error_var.z = error(2);
        //K1
        adaptive_gain_var.x = K1(0);
        adaptive_gain_var.y = K1(1);
        adaptive_gain_var.z = K1(2);
        //asmc
        asmc_var.x = asmc(0);
        asmc_var.y = asmc(1);
        asmc_var.z = asmc(2);
        //Thrust
        thrust_var.data = thrust;
        //Desired attitude and yaw rate
        desired_attitude_var.x = attitude_desired(0);
        desired_attitude_var.y = attitude_desired(1);
        desired_attitude_var.z = attitude_desired(2);

        ss_var.x = ss(0);
        ss_var.y = ss(1);
        ss_var.z = ss(2);

        error_dot_var.x = error_dot(0);
        error_dot_var.y = error_dot(1);
        error_dot_var.z = error_dot(2);

        accelerations_des_var.x = asmc(0);
        accelerations_des_var.y = asmc(1);
        accelerations_des_var.z = asmc(2);
        accelerations_control.publish(accelerations_des_var);

        error_pub.publish(error_var);
        adaptive_gain_pub.publish(adaptive_gain_var);
        asmc_pub.publish(asmc_var);
        thrust_pub.publish(thrust_var);
        desired_att_pub.publish(desired_attitude_var); 
        error_dot_pub.publish(error_dot_var);
        ss_pub.publish(ss_var);

        std::cout << "error: " << error << std::endl;
        //std::cout << "pitch_des " << attitude_desired(1) << std::endl;

        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}