//Including ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

Eigen::Quaternionf attitude_quaternion_des(1.0, 0.0, 0.0, 0.0);
Eigen::Vector3f attitude_vel_des(0.0, 0.0, 0.0);
Eigen::Quaternionf attitude_quaternion(1.0, 0.0, 0.0, 0.0);
Eigen::Vector3f attitude_quaternion_vel(0.0, 0.0, 0.0);
Eigen::Quaternionf quaternion_roll(0.0, 1.0, 0.0, 0.0);

Eigen::Vector3f ATT_DES_EULER(0.0, 0.0, 0.0);
Eigen::Vector3f ATT_EULER(0.0, 0.0, 0.0);

Eigen::Vector3f error(0.0, 0.0, 0.0);
Eigen::Vector3f error_dot(0.0, 0.0, 0.0);
Eigen::Quaternionf q_error(1.0, 0.0, 0.0, 0.0);

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

////////////////Outputs (Torques)////////////////////
Eigen::Vector3f tau; //tau_phi, //tau_theta //tau_psi

float step_size = 0.01;

///////////////Quad's parameters/////////////////////
float Jxx = 0.0411;
float Jyy = 0.0478;
float Jzz = 0.0599;
Eigen::Matrix3f J;

float yaw_ddot_des;

//////////////// General Functions ///////////////////////////
float sign(float value)
{
	int result;
	
	if(value > 0)
	{
		result = 1;
	}
	
	else if(value < 0)
	{
		result = -1;
	}
	
	else if(value == 0)
	{
		result = 0;
	}
	
	return result;
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

///////////////// Callback functions ///////////////////////////////////////////
void attQuaternionDesCallback(const geometry_msgs::Quaternion::ConstPtr& attQuatD)
{
	// attitude_quaternion_des.x() = attQuatD->x;
	// attitude_quaternion_des.y() = attQuatD->y;
	// attitude_quaternion_des.z() = attQuatD->z;
	// attitude_quaternion_des.w() = attQuatD->w;

	// ESTO ES SOLO FIJANDO EL QUATERNION A 0. PON AQUI EL VICOOOOOON
	attitude_quaternion_des.x() = 0.0;
	attitude_quaternion_des.y() = 0.0;
	attitude_quaternion_des.z() = 0.0;
	attitude_quaternion_des.w() = 1.0;
}

void attQuaternionCallback(const geometry_msgs::TransformStamped::ConstPtr& attQuat)
{
	attitude_quaternion.x() = attQuat->transform.rotation.x;
	attitude_quaternion.y() = attQuat->transform.rotation.y;
	attitude_quaternion.z() = attQuat->transform.rotation.z;
	attitude_quaternion.w() = attQuat->transform.rotation.w;

	attitude_quaternion = multiplyQuaternionTimesQuaternion(quaternion_roll, attitude_quaternion);

	ATT_EULER(0) = atan2(2.0 * (attitude_quaternion.w() * attitude_quaternion.y() + attitude_quaternion.w() * attitude_quaternion.x()) , 1.0 - 2.0 * (attitude_quaternion.x() * attitude_quaternion.x() + attitude_quaternion.y() * attitude_quaternion.y()));
    ATT_EULER(1) = asin(2.0 * (attitude_quaternion.y() * attitude_quaternion.w() - attitude_quaternion.z() * attitude_quaternion.x()));
    ATT_EULER(2) = atan2(2.0 * (attitude_quaternion.z() * attitude_quaternion.w() + attitude_quaternion.x() * attitude_quaternion.y()) , - 1.0 + 2.0 * (attitude_quaternion.w() * attitude_quaternion.w() + attitude_quaternion.x() * attitude_quaternion.x()));

}

void attQuatVelCallback(const geometry_msgs::Vector3::ConstPtr& attQuatVel)
{
	attitude_quaternion_vel(0) = attQuatVel->x;
	attitude_quaternion_vel(1) = attQuatVel->y;
	attitude_quaternion_vel(2) = attQuatVel->z;
}

void yawddotVelCallback(const std_msgs::Float64::ConstPtr& ydd)
{
	yaw_ddot_des = ydd->data;
}

void yawRateDesired(const std_msgs::Float64::ConstPtr& yaw_rate_des)
{
	attitude_vel_des(2) = yaw_rate_des->data;
}

void ATTITUDE_DES_EULER(const geometry_msgs::Vector3::ConstPtr& ATT_DES_EULER_CALL)
{
	ATT_DES_EULER(0) = ATT_DES_EULER_CALL->x;
	ATT_DES_EULER(1) = ATT_DES_EULER_CALL->y;
	ATT_DES_EULER(2) = ATT_DES_EULER_CALL->z;
}

int main(int argc, char *argv[])
{	
	ros::init(argc, argv, "attitude_nftasmc");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	ros::Subscriber desired_att_quaternions_sub = nh.subscribe("desired_attitude_quaternion",100, &attQuaternionDesCallback);
	ros::Subscriber quad_attitude_quaternions_sub = nh.subscribe("attitude_QUAV",100, &attQuaternionCallback);
	ros::Subscriber quad_attitude_velocity_sub = nh.subscribe("attVel_estimates",100, &attQuatVelCallback);
	ros::Subscriber yaw_ddot_des_sub = nh.subscribe("yaw_ddot_desired",100, &yawddotVelCallback);
	ros::Subscriber yaw_rate_desired_sub = nh.subscribe("yaw_rate_desired",100, &yawRateDesired);
	
	geometry_msgs::Vector3 quadTorques;
	geometry_msgs::Vector3 ss_att;
	geometry_msgs::Vector3 K1_values;	
	geometry_msgs::Vector3 angular_error_var;		
	geometry_msgs::Vector3 asmc_var;		
	geometry_msgs::Vector3 ss_attitude_var;		
	geometry_msgs::Vector3 angular_error_dot_var;		

	ros::Publisher quad_torques_pub = nh.advertise<geometry_msgs::Vector3>("quad_torques",100);
	ros::Publisher sigma_att_pub = nh.advertise<geometry_msgs::Vector3>("sigma_att",100);
	ros::Publisher k1_gain = nh.advertise<geometry_msgs::Vector3>("ganancia_adaptativa",100);
	ros::Publisher angular_error_pub = nh.advertise<geometry_msgs::Vector3>("angular_errors",100);
	ros::Publisher asmc_attitude_pub = nh.advertise<geometry_msgs::Vector3>("asmc_attitude",100);
	ros::Publisher ss_attitude_pub = nh.advertise<geometry_msgs::Vector3>("ss_attitude",100);
	ros::Publisher angular_error_dot_pub = nh.advertise<geometry_msgs::Vector3>("angular_error_dot",100);

	ros::Subscriber ATTITUDE_DESIRED_EULER = nh.subscribe("ATTITUDE_DESIRED_EULER",100, &ATTITUDE_DES_EULER);

 	xi_1 << 10, 10, 6;
    lambda << 1.8, 1.8, 1.8;
    xi_2 << 2, 2, 2;
    varpi << 4, 4, 4;
    vartheta << 3, 3, 3;
    K1 << 0, 0, 0;
    // K2 << 0.01, 0.01, 0.01;
    // k_reg << 1, 1, 1;
    // kmin << 2, 2, 1;
    // mu << 0.2, 0.2, 0.2;
	alpha << 10, 10, 10;
	beta << 5, 5, 5;

	// THESE WORK FOR A STATIC ARUCO MARKER
	// xi_1 << 6, 6, 6;
    // lambda << 1.8, 1.8, 1.8;
    // xi_2 << 2, 2, 2;
    // varpi << 4, 4, 4;
    // vartheta << 3, 3, 3;
    // K1 << 0, 0, 0;
    // // K2 << 0.01, 0.01, 0.01;
    // // k_reg << 1, 1, 1;
    // // kmin << 2, 2, 1;
    // // mu << 0.2, 0.2, 0.2;
	// alpha << 10, 10, 10;
	// beta << 5, 5, 5;

	J << Jxx, 0, 0,
		0, Jyy, 0,
		0, 0, Jzz;

	attitude_vel_des(0) = 0;
	attitude_vel_des(1) = 0;
	
	// ros::Duration(1).sleep();
	ros::Duration(2.05).sleep();

	while(ros::ok()) {

		q_error = multiplyQuaternionTimesQuaternion(attitude_quaternion.conjugate(), attitude_quaternion_des);
		q_error = q_error.normalized();

		if (q_error.w() >= 1.0) {
			error(0) = 0.0;
			error(1) = 0.0;
			error(2) = 0.0;
		}
		else {
			error(0) = 2.0 * ((q_error.x() / sqrt(powf(q_error.x(), 2.0) + powf(q_error.y(), 2.0) + powf(q_error.z(), 2.0))) * acos(q_error.w()));
			error(1) = 2.0 * ((q_error.y() / sqrt(powf(q_error.x(), 2.0) + powf(q_error.y(), 2.0) + powf(q_error.z(), 2.0))) * acos(q_error.w()));
			error(2) = 2.0 * ((q_error.z() / sqrt(powf(q_error.x(), 2.0) + powf(q_error.y(), 2.0) + powf(q_error.z(), 2.0))) * acos(q_error.w()));
		}

		//error = ATT_DES_EULER - ATT_EULER;

		// tf::Quaternion test_attitude_q(attitude_quaternion.x(), attitude_quaternion.y(), attitude_quaternion.z(), attitude_quaternion.w());
        // tf::Matrix3x3 m(test_attitude_q);
        // Eigen::Vector3d euler(0.0, 0.0, 0.0);
        // m.getRPY(euler[0], euler[1], euler[2]);
        // std::cout << "Angles UAV" << std::endl;
        // std::cout << euler[0] << std::endl;
        // std::cout << euler[1] << std::endl;
        // std::cout << euler[2] << std::endl;
		
		// tf::Quaternion test_attitude_des_q(attitude_quaternion_des.x(), attitude_quaternion_des.y(), attitude_quaternion_des.z(), attitude_quaternion_des.w());
        // tf::Matrix3x3 m2(test_attitude_des_q);
        // Eigen::Vector3d euler2(0.0, 0.0, 0.0);
        // m2.getRPY(euler2[0], euler2[1], euler2[2]);
        // std::cout << "Angles desired" << std::endl;
        // std::cout << euler2[0] << std::endl;
        // std::cout << euler2[1] << std::endl;
        // std::cout << euler2[2] << std::endl;

		error_dot = attitude_vel_des - attitude_quaternion_vel;

		for(int i = 0; i <= 2; i++) {	

			ss(i) = error(i) + xi_1(i) * powf(std::abs(error(i)),lambda(i)) * sign(error(i)) + xi_2(i) * powf(std::abs(error_dot(i)),(varpi(i)/vartheta(i))) * sign(error_dot(i));
			
			// *************** Traditional adaptive law ***************
			// if(K1(i) > kmin(i))	
			// {
			// 	K1_dot(i) = k_reg(i) * sign(std::abs(ss(i))-mu(i));
			// }
			// else
			// {
			// 	K1_dot(i) = kmin(i);
			// }
			
			// K1(i) = K1(i) + step_size * K1_dot(i); //New value of K1
			// asmc(i) = -K1(i) * powf(std::abs(ss(i)),0.5) * sign(ss(i)) - K2(i) * ss(i);

			//*************** Modified adaptive law ***************
			K1_dot(i) = sqrt(alpha(i)) * sqrt(std::abs(ss(i))) - sqrt(beta(i)) * pow(K1(i),2);

            K1(i) = K1(i) + step_size*K1_dot(i);
            asmc(i) = -2 * K1(i) * sqrt(std::abs(ss(i))) * sign(ss(i)) - (pow(K1(i),2) / 2) * ss(i);
		}

		std::cout << "Quat of error" << std::endl;
        std::cout << q_error.w() << std::endl;
        std::cout << q_error.x() << std::endl;
        std::cout << q_error.y() << std::endl;
		std::cout << q_error.z() << std::endl;
		std::cout << "Error quaternions" << std::endl;
        std::cout << error(0) << std::endl;
        std::cout << error(1) << std::endl;
        std::cout << error(2) << std::endl;

		Eigen::Vector3f angular_acceleration_desired(0.0, 0.0, yaw_ddot_des);
		Eigen::Vector3f division_varpi_vartheta(0.0, 0.0, 0.0);
		division_varpi_vartheta << varpi(0)/vartheta(0), varpi(1)/vartheta(1), varpi(2)/vartheta(2);
		Eigen::Vector3f division_one_over_xi2(0.0, 0.0, 0.0);
		division_one_over_xi2 << 1 / (xi_2(0) * division_varpi_vartheta(0)), 1 / (xi_2(1) * division_varpi_vartheta(1)), 1 / (xi_2(2) * division_varpi_vartheta(2));
		Eigen::Vector3f fourth_term(0.0, 0.0, 0.0);
		fourth_term << division_one_over_xi2(0) * powf(abs(error_dot(0)), 2 - (division_varpi_vartheta(0))) * sign(error_dot(0)), division_one_over_xi2(1) * powf(abs(error_dot(1)), 2 - (division_varpi_vartheta(1))) * sign(error_dot(1)), division_one_over_xi2(2) * powf(abs(error_dot(2)), 2 - (division_varpi_vartheta(2))) * sign(error_dot(2));
		Eigen::Vector3f fifth_term(0.0, 0.0, 0.0);
		fifth_term << division_one_over_xi2(0) * powf(abs(error_dot(0)), 2 - (division_varpi_vartheta(0))) * sign(error_dot(0)) * xi_1(0) * lambda(0) * powf(abs(error(0)), lambda(0) - 1), division_one_over_xi2(1) * powf(abs(error_dot(1)), 2 - (division_varpi_vartheta(1))) * sign(error_dot(1)) * xi_1(1) * lambda(1) * powf(abs(error(1)), lambda(1) - 1), division_one_over_xi2(2) * powf(abs(error_dot(2)), 2 - (division_varpi_vartheta(2))) * sign(error_dot(2)) * xi_1(2) * lambda(2) * powf(abs(error(2)), lambda(2) - 1);

		tau = J * (angular_acceleration_desired + J.inverse() * (attitude_quaternion_vel.cross(J * attitude_quaternion_vel)) - asmc + fourth_term + fifth_term);

		// Saturate values (torque should not be bigger than 0.025 Nm)
		for (int i = 0; i < tau.size(); i++) {
			if (tau[i] > 0.025) {
				tau[i] = 0.025;
			}
			else if (tau[i] < -0.025) {
				tau[i] = -0.025;
			}
    	}

		// tau(0) = Jxx * (-asmc(0) + (((Jyy-Jzz)/Jxx) * attitude_quaternion_vel(1) * attitude_quaternion_vel(2)) + (vartheta(0)/(varpi(0)*xi_2(0))) * sign(error_dot(0)) * powf(std::abs(error_dot(0)),(2-(varpi(0)/vartheta(0)))) * (1 + xi_1(0) * lambda(0) * powf(std::abs(error(0)),lambda(0)-1)));
		// tau(1) = Jyy * (-asmc(1) + (((Jzz-Jxx)/Jyy) * attitude_quaternion_vel(0) * attitude_quaternion_vel(2)) + (vartheta(1)/(varpi(1)*xi_2(1))) * sign(error_dot(1)) * powf(std::abs(error_dot(1)),(2-(varpi(1)/vartheta(1)))) * (1 + xi_1(1) * lambda(1) * powf(std::abs(error(1)),lambda(1)-1)));
		// tau(2) = Jzz * (-asmc(2) + (((Jxx-Jyy)/Jzz) * attitude_quaternion_vel(0) * attitude_quaternion_vel(1)) + (vartheta(2)/(varpi(2)*xi_2(2))) * sign(error_dot(2)) * powf(std::abs(error_dot(2)),(2-(varpi(2)/vartheta(2)))) * (1 + xi_1(2) * lambda(2) * powf(std::abs(error(2)),lambda(2)-1)));

		// tau(0) = Jxx * (-asmc(0) + (((Jyy-Jzz)/Jxx) * attitude_vel(1) * attitude_vel(2)) + (vartheta(0)/(varpi(0)*xi_2(0))) * sign(error_dot(0)) * powf(std::abs(error_dot(0)),(2-(varpi(0)/vartheta(0)))) * (1 + xi_1(0) * lambda(0) * powf(std::abs(error(0)),lambda(0)-1)));
		// tau(1) = Jyy * (-asmc(1) + (((Jzz-Jxx)/Jyy) * attitude_vel(0) * attitude_vel(2)) + (vartheta(1)/(varpi(1)*xi_2(1))) * sign(error_dot(1)) * powf(std::abs(error_dot(1)),(2-(varpi(1)/vartheta(1)))) * (1 + xi_1(1) * lambda(1) * powf(std::abs(error(1)),lambda(1)-1)));
		// tau(2) = Jzz * (-asmc(2) + (((Jxx-Jyy)/Jzz) * attitude_vel(0) * attitude_vel(1)) + (vartheta(2)/(varpi(2)*xi_2(2))) * sign(error_dot(2)) * powf(std::abs(error_dot(2)),(2-(varpi(2)/vartheta(2)))) * (1 + xi_1(2) * lambda(2) * powf(std::abs(error(2)),lambda(2)-1)));

		quadTorques.x = tau(0);
		quadTorques.y = tau(1);
		quadTorques.z = tau(2);
			
		ss_att.x = ss(0);
		ss_att.y = ss(1);
		ss_att.z = ss(2);	
		
		K1_values.x = K1(0);
		K1_values.y = K1(1);
		K1_values.z = K1(2);

		angular_error_var.x = error(0);
		angular_error_var.y = error(1);
		angular_error_var.z = error(2);

		asmc_var.x = asmc(0);
		asmc_var.y = asmc(1);
		asmc_var.z = asmc(2);

		ss_attitude_var.x = ss(0);
		ss_attitude_var.y = ss(1);
		ss_attitude_var.z = ss(2);

		angular_error_dot_var.x = error_dot(0);
		angular_error_dot_var.y = error_dot(1);
		angular_error_dot_var.z = error_dot(2);

		quad_torques_pub.publish(quadTorques);
		sigma_att_pub.publish(ss_att);	
		k1_gain.publish(K1_values);		
		angular_error_pub.publish(angular_error_var);	
		asmc_attitude_pub.publish(asmc_var);	
		ss_attitude_pub.publish(ss_attitude_var);
		angular_error_dot_pub.publish(angular_error_dot_var);

		// std::cout << "Error_yaw " << error(2) << std::endl;
		// std::cout << "Torques " << std::endl << tau << std::endl;
		std::cout << "QUAV attitude " << std::endl << ATT_EULER << std::endl;

		ros::spinOnce();
		loop_rate.sleep();
	
	}

	return 0;
}
