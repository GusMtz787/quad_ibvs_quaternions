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
#include <fstream>
#include <sstream>
#include <string>
//Including Eigen library
#include <eigen3/Eigen/Dense>

Eigen::Vector3f attitude_des;
Eigen::Vector3f attitude_vel_des;
Eigen::Vector3f attitude_acc_des;
Eigen::Vector3f attitude;
Eigen::Vector3f attitude_vel;

Eigen::Vector3f error;
Eigen::Vector3f error_dot;
Eigen::Vector3f error_integrated(0.0, 0.0, 0.0);

Eigen::Quaternionf quaternion_roll(0.0, 1.0, 0.0, 0.0);
Eigen::Quaternionf attitude_quat(1.0, 0.0, 0.0, 0.0);

Eigen::Vector3f Kp(0.0, 0.0, 0.0); //0.742
Eigen::Vector3f Ki(0.0, 0.0, 0.0); //0.01
Eigen::Vector3f Kd(0.0, 0.0, 0.0); //0.001, 0.001, 0.005

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
bool saturation = false;

///////////////Quad's parameters/////////////////////
float Jxx = 0.0411;
float Jyy = 0.0478;
float Jzz = 0.0599;

float yaw_ddot_des;

// General functions
Eigen::Quaternionf multiplyQuaternionTimesQuaternion(Eigen::Quaternionf q, Eigen::Quaternionf r) 
{
	Eigen::Quaternionf quat_result;

	quat_result.w() = r.w() * q.w() - r.x() * q.x() - r.y() * q.y() - r.z() * q.z();
	quat_result.x() = r.w() * q.x() + r.x() * q.w() - r.y() * q.z() + r.z() * q.y();
	quat_result.y() = r.w() * q.y() + r.x() * q.z() + r.y() * q.w() - r.z() * q.x();
	quat_result.z() = r.w() * q.z() - r.x() * q.y() + r.y() * q.x() + r.z() * q.w();
	
	return quat_result;
}

float sign(float value) {
	int result;
	
	if(value > 0) {
		result = 1;
	}
	
	else if(value < 0) {
		result = -1;
	}
	
	else if(value == 0) {
		result = 0;
	}
	
	return result;
}

// Function to split a string based on a delimiter
std::vector<std::string> split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// Callbacks
void attDesCallback(const geometry_msgs::Vector3::ConstPtr& attD) {
	attitude_des(0) = attD->x;
	attitude_des(1) = attD->y;
	attitude_des(2) = attD->z;
}

void attCallback(const geometry_msgs::Quaternion::ConstPtr& att) {

	// attitude_quat.x() = att->x;
    // attitude_quat.y() = att->y;
    // attitude_quat.z() = att->z;
    // attitude_quat.w() = att->w;

	// // Rotate the quaternion 180 degrees alongside the x axis because the vicon world is ENU and we want NED.
    // // So, even though the drone in the vicon is in NED, because of the World vicon reference, the NED of the
    // // drone is rotated 180 with respect to the vicon world ENU. 
	// attitude_quat = multiplyQuaternionTimesQuaternion(quaternion_roll, attitude_quat);

    // // Roll
	// attitude(0)  = atan2(2.0 * (attitude_quat.w() * attitude_quat.y() + attitude_quat.w() * attitude_quat.x()) , 1.0 - 2.0 * (attitude_quat.x() * attitude_quat.x() + attitude_quat.y() * attitude_quat.y()));
	// // if (isnan(roll)) {
	// // 	roll = 0.0;
	// // }

    // // Pitch
    // attitude(1) = asin(2.0 * (attitude_quat.y() * attitude_quat.w() - attitude_quat.z() * attitude_quat.x()));
	// // if (isnan(pitch)) {
	// // 	pitch = 0.0;
	// // }

    // // Yaw
    // attitude(2) = atan2(2.0 * (attitude_quat.z() * attitude_quat.w() + attitude_quat.x() * attitude_quat.y()) , - 1.0 + 2.0 * (attitude_quat.w() * attitude_quat.w() + attitude_quat.x() * attitude_quat.x()));
    // // if (isnan(yaw)) {
    // //     yaw = 0.0;
    // // }

	attitude(0)  = atan2(2.0 * (att->w * att->y + att->w * att->x) , 1.0 - 2.0 * (att->x * att->x + att->y * att->y));
	
    attitude(1) = asin(2.0 * (att->y * att->w - att->z * att->x));
	
    attitude(2) = atan2(2.0 * (att->z * att->w + att->x * att->y) , - 1.0 + 2.0 * (att->w * att->w + att->x * att->x));
}

void attVelCallback(const geometry_msgs::Vector3::ConstPtr& attVel) {
	attitude_vel(0) = attVel->x;
	attitude_vel(1) = attVel->y;
	attitude_vel(2) = attVel->z;
}

void yawddotVelCallback(const std_msgs::Float64::ConstPtr& ydd) {
	yaw_ddot_des = ydd->data;
}

int main(int argc, char *argv[]) {

	ros::init(argc, argv, "attitude_nftasmc_VICON");
	ros::NodeHandle nh;
	ros::Rate loop_rate(300);
	
	ros::Subscriber desired_att_sub = nh.subscribe("desired_attitude",100, &attDesCallback);
	ros::Subscriber quad_attitude_sub = nh.subscribe("attitude_QUAV",100, &attCallback);
	ros::Subscriber quad_attitude_velocity_sub = nh.subscribe("attVel_estimates",100, &attVelCallback);
	ros::Subscriber yaw_ddot_des_sub = nh.subscribe("yaw_ddot_desired",100, &yawddotVelCallback);
	
	geometry_msgs::Vector3 quadTorques;
	geometry_msgs::Vector3 adaptive_gains_att;
	geometry_msgs::Vector3 ss_att;	
	geometry_msgs::Vector3 error_att;	
	
	ros::Publisher quad_torques_pub = nh.advertise<geometry_msgs::Vector3>("quad_torques",1);
	ros::Publisher adaptive_gain_att_pub = nh.advertise<geometry_msgs::Vector3>("adaptive_gain_attitude",100);
	ros::Publisher sigma_att_pub = nh.advertise<geometry_msgs::Vector3>("sigma_att",100);
	ros::Publisher attitude_error_pub = nh.advertise<geometry_msgs::Vector3>("attitude_error",100);
	
    // xi_1 << 1, 0.5, 0.5;
    // lambda << 1.8, 1.8, 1.8;
    // xi_2 << 1.2, 1.2, 1.2;
    // varpi << 4, 4, 4;
    // vartheta << 3, 3, 3;
    // K1 << 0, 0, 0;
    // K2 << 0.01, 0.01, 0.01;
    // k_reg << 1, 1, 1;
    // kmin << 2, 2, 1;
    // mu << 0.2, 0.2, 0.2;
	// alpha << 10, 10, 1;
	// beta << 0.3, 0.1, 1;
	// attitude_vel_des(0) = 0;
	// attitude_vel_des(1) = 0;

	// Define the file name
    std::string filename = "/home/pi/catkin_ws/src/ibvs/attitudeParams.csv";

    // Create an input file stream object
    std::ifstream file(filename);

    // Check if the file is opened successfully
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return 1; // Return error code
    }

    // Read and print each line from the file
    std::string line;

	// Counter variable to track the line number
    int lineNumber = 0;

    while (std::getline(file, line) && lineNumber < 3) {
        // Split the line into tokens based on comma delimiter
        std::vector<std::string> tokens = split(line, ',');
		
		// Erase the first element
		tokens.erase(tokens.begin());
        
		if (lineNumber == 0) {
			Kp(0) = std::stof(tokens[0]);
			Kp(1) = std::stof(tokens[1]);
			Kp(2) = std::stof(tokens[2]);
		}
		
		if (lineNumber == 1) {
			Ki(0) = std::stof(tokens[0]);
			Ki(1) = std::stof(tokens[1]);
			Ki(2) = std::stof(tokens[2]);
		}

		if (lineNumber == 2) {
			Kd(0) = std::stof(tokens[0]);
			Kd(1) = std::stof(tokens[1]);
			Kd(2) = std::stof(tokens[2]);
		}
		
		lineNumber++;
    }

	std::cout << Kp << std::endl;
	std::cout << Ki << std::endl;
	std::cout << Kd << std::endl;

    // Close the file
    file.close();


	attitude_vel_des << 0.0, 0.0, 0.0;
	attitude_acc_des << 0.0, 0.0, 0.0;

	ros::Duration(1).sleep();
	while(ros::ok()) {	

		// for(int i = 0; i <= 2; i++)
		// {	
		// 	error(i) = attitude_des(i) - attitude(i);
		// 	error_dot(i) = attitude_vel_des(i) - attitude_vel(i);

		// 	ss(i) = error(i) + xi_1(i) * powf(std::abs(error(i)),lambda(i)) * sign(error(i)) + xi_2(i) * powf(std::abs(error_dot(i)),(varpi(i)/vartheta(i))) * sign(error_dot(i));
			
		// 	// *************** Traditional adaptive law ***************
		// 	// if(K1(i) > kmin(i))	
		// 	// {
		// 	// 	K1_dot(i) = k_reg(i) * sign(std::abs(ss(i))-mu(i));
		// 	// }
		// 	// else
		// 	// {
		// 	// 	K1_dot(i) = kmin(i);
		// 	// }
			
		// 	// K1(i) = K1(i) + step_size * K1_dot(i); //New value of K1
		// 	// asmc(i) = -K1(i) * powf(std::abs(ss(i)),0.5) * sign(ss(i)) - K2(i) * ss(i);

		// 	//*************** Modified adaptive law ***************
		// 	K1_dot(i) = sqrt(alpha(i)) * sqrt(std::abs(ss(i))) - sqrt(beta(i)) * powf(K1(i),2);

        //     K1(i) = K1(i) + step_size*K1_dot(i);
        //     asmc(i) = -2 * K1(i) * sqrt(std::abs(ss(i))) * sign(ss(i)) - (pow(K1(i),2) / 2) * ss(i);
		// }
		
		// tau(0) = Jxx * (-asmc(0) + (((Jyy-Jzz)/Jxx) * attitude_vel(1) * attitude_vel(2)) + (vartheta(0)/(varpi(0)*xi_2(0))) * sign(error_dot(0)) * powf(std::abs(error_dot(0)),(2-(varpi(0)/vartheta(0)))) * (1 + xi_1(0) * lambda(0) * powf(std::abs(error(0)),lambda(0)-1)));
		// tau(1) = Jyy * (-asmc(1) + (((Jzz-Jxx)/Jyy) * attitude_vel(0) * attitude_vel(2)) + (vartheta(1)/(varpi(1)*xi_2(1))) * sign(error_dot(1)) * powf(std::abs(error_dot(1)),(2-(varpi(1)/vartheta(1)))) * (1 + xi_1(1) * lambda(1) * powf(std::abs(error(1)),lambda(1)-1)));
		// tau(2) = Jzz * (-asmc(2) + (((Jxx-Jyy)/Jzz) * attitude_vel(0) * attitude_vel(1)) + (vartheta(2)/(varpi(2)*xi_2(2))) * sign(error_dot(2)) * powf(std::abs(error_dot(2)),(2-(varpi(2)/vartheta(2)))) * (1 + xi_1(2) * lambda(2) * powf(std::abs(error(2)),lambda(2)-1)));

		error = attitude_des - attitude;
		error_dot = attitude_vel_des - attitude_vel;

		//If the error is not saturated, then error should be fed. Else, error is 0 to prevent I wind-up.
        if (!saturation) { 
            error_integrated(0) = error_integrated(0) + step_size * error(0);      
            error_integrated(1) = error_integrated(1) + step_size * error(1);      
            error_integrated(2) = error_integrated(2) + step_size * error(2);  
			std::cout << error_integrated << std::endl;    
        }
        else {
            for(int i = 0; i <= 2; i++) {
                error_integrated(i) = 0.0;
            }
			std::cout << "Saturated" << std::endl;  
			saturation = false;  
        }

		// tau(0) = Jxx * (attitude_acc_des(0) - (((Jyy-Jzz)/Jxx) * attitude_vel(1) * attitude_vel(2)) + Kp(0)*error(0) + Kd(0)*error_dot(0));
		// tau(1) = Jyy * (attitude_acc_des(1) - (((Jzz-Jxx)/Jyy) * attitude_vel(0) * attitude_vel(2)) + Kp(1)*error(1) + Kd(1)*error_dot(1));
		// tau(2) = Jzz * (attitude_acc_des(2) - (((Jxx-Jyy)/Jzz) * attitude_vel(0) * attitude_vel(1)) + Kp(2)*error(2) + Kd(2)*error_dot(2));		
		
		tau(0) = Kp(0)*error(0) + Ki(0)*error_integrated(0) + Kd(0)*error_dot(0);
		tau(1) = Kp(1)*error(1) + Ki(1)*error_integrated(1) + Kd(1)*error_dot(1);
		tau(2) = Kp(2)*error(2) + Ki(2)*error_integrated(2) + Kd(2)*error_dot(2);	

		// Saturate torques for tests
		if (tau(0) > 0.5) {
			tau(0) = 0.5;
			saturation = true;
		}
		// if (tau(1) > 0.5) {
		// 	tau(1) = 0.5;
		// 	saturation = true;
		// }
		// if (tau(2) > 0.5) {
		// 	tau(2) = 0.5;
		// 	saturation = true;
		// }
		if (tau(0) < -0.5) {
			tau(0) = -0.5;
			saturation = true;
		}
		// if (tau(1) < -0.5) {
		// 	tau(1) = -0.5;
		// 	saturation = true;
		// }
		// if (tau(2) < -0.5) {
		// 	tau(2) = -0.5;
		// 	saturation = true;
		// }

		quadTorques.x = tau(0); // tau(0)
		quadTorques.y = tau(1); // tau(1)
		quadTorques.z = tau(2);
		
		adaptive_gains_att.x = K1(0);
		adaptive_gains_att.y = K1(1);
		adaptive_gains_att.z = K1(2);
			
		ss_att.x = ss(0);
		ss_att.y = ss(1);
		ss_att.z = ss(2);	

		error_att.x = error(0);
		error_att.y = error(1);
		error_att.z = error(2);
			
		quad_torques_pub.publish(quadTorques);
		adaptive_gain_att_pub.publish(adaptive_gains_att);
		sigma_att_pub.publish(ss_att);	
		attitude_error_pub.publish(error_att);			
			
		// std::cout << "Error_yaw " << error(2) << std::endl;
		//std::cout << "Torque_pitch " << tau(1) << std::endl;

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
