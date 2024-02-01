// Courtesy of Armando Miranda Moya.

//Including ROS libraries
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

Eigen::Vector3f position(0,0,0);
Eigen::Quaternionf attitude_quat(1.0, 0.0, 0.0, 0.0);
Eigen::Quaternionf quaternion_roll(0.0, 1.0, 0.0, 0.0);
Eigen::Vector3f attitude(0,0,0);

Eigen::VectorXf G1(6);
Eigen::VectorXf G2(6);

Eigen::VectorXf lambda(6);  // 0 < lambda < 1
Eigen::VectorXf varphi(6);  // varphi > 1 

Eigen::Vector3f pos_est(0,0,0);
Eigen::Vector3f vel_est(0,0,0);

Eigen::Vector3f att_est(0,0,0);
Eigen::Vector3f attvel_est(0,0,0);

Eigen::VectorXf x1_dot(6);
Eigen::VectorXf x2_dot(6);

float step = 0.01;

Eigen::Vector3f estimation_error_linear(0,0,0);
Eigen::Vector3f estimation_error_angular(0,0,0);


//MATH FUNCTIONS
float sign(float var)
{   
    float x;
    if (var > 0)
    {
        x = 1;
    }
    else if (var<0)
    {
        x = -1;
    }
    else if (var == 0)
    {
        x = 0;
    }
    return x;
}

Eigen::Quaternionf multiplyQuaternionTimesQuaternion(Eigen::Quaternionf q, Eigen::Quaternionf r) 
{
	Eigen::Quaternionf quat_result;

	quat_result.w() = r.w() * q.w() - r.x() * q.x() - r.y() * q.y() - r.z() * q.z();
	quat_result.x() = r.w() * q.x() + r.x() * q.w() - r.y() * q.z() + r.z() * q.y();
	quat_result.y() = r.w() * q.y() + r.x() * q.z() + r.y() * q.w() - r.z() * q.x();
	quat_result.z() = r.w() * q.z() - r.x() * q.y() + r.y() * q.x() + r.z() * q.w();
	
	return quat_result;
}

//CALLBACKS
void pos_att_Callback(const geometry_msgs::TransformStamped::ConstPtr& poseQUAV)
{
    position(0) = poseQUAV->transform.translation.x;
    position(1) = poseQUAV->transform.translation.y;
    position(2) = poseQUAV->transform.translation.z;

    attitude_quat.x() = poseQUAV->transform.rotation.x;
    attitude_quat.y() = poseQUAV->transform.rotation.y;
    attitude_quat.z() = poseQUAV->transform.rotation.z;
    attitude_quat.w() = poseQUAV->transform.rotation.w;

    attitude_quat = multiplyQuaternionTimesQuaternion(quaternion_roll, attitude_quat);

	attitude(0) = atan2(2.0 * (attitude_quat.w() * attitude_quat.y() + attitude_quat.w() * attitude_quat.x()) , 1.0 - 2.0 * (attitude_quat.x() * attitude_quat.x() + attitude_quat.y() * attitude_quat.y()));
    attitude(1) = asin(2.0 * (attitude_quat.y() * attitude_quat.w() - attitude_quat.z() * attitude_quat.x()));
    attitude(2) = atan2(2.0 * (attitude_quat.z() * attitude_quat.w() + attitude_quat.x() * attitude_quat.y()) , - 1.0 + 2.0 * (attitude_quat.w() * attitude_quat.w() + attitude_quat.x() * attitude_quat.x()));

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "fx_estimator");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	

    //Subscribers and publishers
    ros::Subscriber quav_pos_sub = nh.subscribe("vicon/QuadGus/QuadGus", 100, &pos_att_Callback);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    ros::Publisher positionEstimates_pub = nh.advertise<geometry_msgs::Vector3>("position_estimates",100);    
    ros::Publisher velocityEstimates_pub = nh.advertise<geometry_msgs::Vector3>("velocity_estimates",100); 

    ros::Publisher attitudeEstimates_pub = nh.advertise<geometry_msgs::Vector3>("attitude_estimates",100); 
    ros::Publisher attVelEstimates_pub = nh.advertise<geometry_msgs::Vector3>("attVel_estimates",100); 

    ros::Publisher estimationError_linear_pub = nh.advertise<geometry_msgs::Vector3>("estimation_error_linear",100);
    ros::Publisher estimationError_angular_pub = nh.advertise<geometry_msgs::Vector3>("estimation_error_angular",100);
    
    geometry_msgs::Vector3 positionEstimates_var;
    geometry_msgs::Vector3 velocityEstimates_var;

    geometry_msgs::Vector3 attitudeEstimates_var;
    geometry_msgs::Vector3 attVelEstimates_var;
    
    geometry_msgs::Vector3 estimationError_linear_var;
    geometry_msgs::Vector3 estimationError_angular_var;

    
    //FXTESO GAINS
    G1 << 16, 16, 16, 16, 16, 16;
    G2 << 150, 150, 150, 150, 150, 150;

    lambda << 0.6, 0.6, 0.6, 0.6, 0.6, 0.6;
    varphi << 1.2, 1.2, 1.2, 1.2, 1.2, 1.2;


    //Initial conditions
    pos_est << 0,0,0;
    vel_est << 0,0,0;

    att_est << 0,0,0;
    attvel_est << 0,0,0;

    x1_dot << 0,0,0,0,0,0;
    x2_dot << 0,0,0,0,0,0;

    estimation_error_linear(0) = position(0) - pos_est(0);
    estimation_error_linear(1) = position(1) - pos_est(1);
    estimation_error_linear(2) = position(2) - pos_est(2);

    estimation_error_angular(0) = attitude(0) - att_est(0);
    estimation_error_angular(1) = attitude(1) - att_est(1);
    estimation_error_angular(2) = attitude(2) - att_est(2);



    positionEstimates_var.x = pos_est(0);
    positionEstimates_var.y = pos_est(1);
    positionEstimates_var.z = pos_est(2);

    velocityEstimates_var.x = vel_est(0);
    velocityEstimates_var.y = vel_est(1);
    velocityEstimates_var.z = vel_est(2);

    attitudeEstimates_var.x = att_est(0);
    attitudeEstimates_var.y = att_est(1);
    attitudeEstimates_var.z = att_est(2);

    attVelEstimates_var.x = attvel_est(0);
    attVelEstimates_var.y = attvel_est(1);
    attVelEstimates_var.z = attvel_est(2);

    estimationError_linear_var.x = estimation_error_linear(0);
    estimationError_linear_var.y = estimation_error_linear(1);
    estimationError_linear_var.z = estimation_error_linear(2);
    
    estimationError_angular_var.x = estimation_error_angular(0);
    estimationError_angular_var.y = estimation_error_angular(1);
    estimationError_angular_var.z = estimation_error_angular(2);


    positionEstimates_pub.publish(positionEstimates_var);
    velocityEstimates_pub.publish(velocityEstimates_var);
    
    attitudeEstimates_pub.publish(attitudeEstimates_var);
    attVelEstimates_pub.publish(attVelEstimates_var);
    
    estimationError_linear_pub.publish(estimationError_linear_var);
    estimationError_angular_pub.publish(estimationError_angular_var);
    

    ros::Duration(2.68).sleep();
    //ros::Duration(3.7).sleep();

    while(ros::ok())
    { 
        ///////////////////////////////POSITION SUBSYSTEM///////////////////////////////////////////////////
        for(int i = 0; i<=2; i++)
        {
            //Estimation error
            estimation_error_linear(i) = position(i) - pos_est(i);
            
            // Proceding with the estimator
            x2_dot(i) = G2(i) * sign(estimation_error_linear(i)) * ( powf(std::abs(estimation_error_linear(i)),((lambda(i) + 1)/2))  +  powf(std::abs(estimation_error_linear(i)),(varphi(i) + 1)/2) );
            vel_est(i) = vel_est(i) + x2_dot(i) * step;

            x1_dot(i) = vel_est(i) + G1(i) * sign(estimation_error_linear(i)) * ( powf(std::abs(estimation_error_linear(i)),(lambda(i) + 2)/3)  +  powf(std::abs(estimation_error_linear(i)),(varphi(i) + 2)/3) );
            pos_est(i) = pos_est(i) + x1_dot(i) * step;

        }
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////ATTITUDE SUBSYSTEM////////////////////////////////////////////////////
        for(int i = 0; i<=2; i++)
        {
            //Estimation error
            estimation_error_angular(i) = attitude(i) - att_est(i);
            
            // Proceding with the estimator
            x2_dot(3+i) = G2(3+i) * sign(estimation_error_angular(i)) * ( powf(std::abs(estimation_error_angular(i)),((lambda(3+i) + 1)/2))  +  powf(std::abs(estimation_error_angular(i)),(varphi(3+i) + 1)/2) );
            attvel_est(i) = attvel_est(i) + x2_dot(3+i) * step;

            x1_dot(3+i) = attvel_est(i) + G1(3+i) * sign(estimation_error_angular(i)) * ( powf(std::abs(estimation_error_angular(i)),(lambda(3+i) + 2)/3)  +  powf(std::abs(estimation_error_angular(i)),(varphi(3+i) + 2)/3) );
            att_est(i) = att_est(i) + x1_dot(3+i) * step;

        }

        positionEstimates_var.x = pos_est(0);
        positionEstimates_var.y = pos_est(1);
        positionEstimates_var.z = pos_est(2);

        velocityEstimates_var.x = vel_est(0);
        velocityEstimates_var.y = vel_est(1);
        velocityEstimates_var.z = vel_est(2);

        attitudeEstimates_var.x = att_est(0);
        attitudeEstimates_var.y = att_est(1);
        attitudeEstimates_var.z = att_est(2);

        attVelEstimates_var.x = attvel_est(0);
        attVelEstimates_var.y = attvel_est(1);
        attVelEstimates_var.z = attvel_est(2);

        estimationError_linear_var.x = estimation_error_linear(0);
        estimationError_linear_var.y = estimation_error_linear(1);
        estimationError_linear_var.z = estimation_error_linear(2);
    
        estimationError_angular_var.x = estimation_error_angular(0);
        estimationError_angular_var.y = estimation_error_angular(1);
        estimationError_angular_var.z = estimation_error_angular(2);

        positionEstimates_pub.publish(positionEstimates_var);
        velocityEstimates_pub.publish(velocityEstimates_var);

        attitudeEstimates_pub.publish(attitudeEstimates_var);
        attVelEstimates_pub.publish(attVelEstimates_var);

        estimationError_linear_pub.publish(estimationError_linear_var);
        estimationError_angular_pub.publish(estimationError_angular_var);


        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}