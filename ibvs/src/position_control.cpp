//Including ROS libraries
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

using namespace std;

//Declaring global variables
uint32_t rcMode = 0; // 1 for Visual-servoing guidance, 0 for Vicon guidance. 

/////////////////Error and Error dot variables///////////////
Eigen::Vector4f imgFeat;
Eigen::Vector4f imgFeat_des;
Eigen::Vector3f imgFeatLinear;
Eigen::Vector3f imgFeatLinear_dot;
Eigen::Vector4f error;
Eigen::Vector4f error_dot;
Eigen::Vector3f tgt_vel;
Eigen::Vector3f tgt_accel;

Eigen::Vector3f quad_vel_VF;
Eigen::Vector3f yawVel_e3;
Eigen::Vector3f tgt_vel_VF;
Eigen::Matrix4f Omega;
Eigen::Vector4f v_imgFeat;
Eigen::Vector4f kappa;
Eigen::Vector4f kappa_dot;

////////////////////Sliding surface and ASMC///////////////////
// Visual-servoing controller parameters
Eigen::Vector4f ss;
Eigen::Vector4f xi_1_visualServoing;
Eigen::Vector4f lambda_visualServoing;
Eigen::Vector4f xi_2_visualServoing;
Eigen::Vector4f varpi_visualServoing;
Eigen::Vector4f vartheta_visualServoing;
Eigen::Vector4f asmc;
Eigen::Vector4f K1;
Eigen::Vector4f K1_dot;
Eigen::Vector4f K2;
Eigen::Vector4f k_reg;
Eigen::Vector4f kmin;
Eigen::Vector4f mu;
Eigen::Vector4f alpha_visualServoing;
Eigen::Vector4f beta_visualServoing;

// Vicon controller parameters
Eigen::Vector4f xi_1_vicon;
Eigen::Vector4f lambda_vicon;
Eigen::Vector4f xi_2_vicon;
Eigen::Vector4f varpi_vicon;
Eigen::Vector4f vartheta_vicon;
Eigen::Vector4f alpha_vicon;
Eigen::Vector4f beta_vicon;

///////////////////////////Control input///////////////////////////
Eigen::Vector4f ibvs_ctrl_input;
////////////////////////////Quad's VF Dynamics/////////////////////
Eigen::Vector3f quad_accel_VF;
Eigen::Vector3f quad_linear_forces_VF;
Eigen::Vector3f e3;

//////////////////////////Desired attitude for the quadrotor//////////
float yawRate_desired = 0.0;

float a = 0.0;
float zD = 1.5;
float tgt_YR = 0.0;
float tgt_YAccel = 0.0;
float step_size = 0.01;
float quad_mass = 1.3;
float gravity = 9.81;
float thrust = quad_mass * gravity;
float roll, pitch, yaw = 0.0;

float yaw_desired = 0.0;
Eigen::Quaternionf attitude_desired_quaternion_z(1.0, 0.0, 0.0, 0.0);
Eigen::Vector3f nu(0.0, 0.0, 0.0);
Eigen::Quaternionf attitude_desired_quaternion_x_y(1.0, 0.0, 0.0, 0.0);
Eigen::Vector3f nz(0.0, 0.0, 1.0);
Eigen::Vector3f cross_product(0.0, 0.0, 0.0);
Eigen::Vector3f imaginary_part(0.0, 0.0, 0.0);
Eigen::Quaternionf attitude_desired_quaternion(1.0, 0.0, 0.0, 0.0);
Eigen::Vector3f quad_attitude_velocity(0.0, 0.0, 0.0);
Eigen::Vector3f quad_quaternions_velocity(0.0, 0.0, 0.0);
Eigen::Quaternionf attitude_quaternion(1.0, 0.0, 0.0, 0.0);
Eigen::Quaternionf attitude_x_quaternion(1.0, 0.0, 0.0, 0.0);
Eigen::Quaternionf attitude_y_quaternion(1.0, 0.0, 0.0, 0.0);
Eigen::Quaternionf attitude_z_quaternion(1.0, 0.0, 0.0, 0.0);
Eigen::Quaternionf attitude_x_y_quaternion(1.0, 0.0, 0.0, 0.0);
Eigen::Quaternionf attitude_position_q(1.0, 0.0, 0.0, 0.0);

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

Eigen::Quaternionf multiplyQuaternionTimesQuaternion(Eigen::Quaternionf q, Eigen::Quaternionf r) 
{
	Eigen::Quaternionf quat_result;

	quat_result.w() = r.w() * q.w() - r.x() * q.x() - r.y() * q.y() - r.z() * q.z();
	quat_result.x() = r.w() * q.x() + r.x() * q.w() - r.y() * q.z() + r.z() * q.y();
	quat_result.y() = r.w() * q.y() + r.x() * q.z() + r.y() * q.w() - r.z() * q.x();
	quat_result.z() = r.w() * q.z() - r.x() * q.y() + r.y() * q.x() + r.z() * q.w();
	
	return quat_result;
}

Eigen::Vector3f rotate_vector_by_quaternion_passive(Eigen::Vector3f v, Eigen::Quaternionf q)
{

	Eigen::Vector3f vprime;
	Eigen::Matrix3f quaternion_matrix;
	
	quaternion_matrix << (1-2*powf(q.y(),2.0)-2*powf(q.z(),2.0)), 2*(q.x()*q.y() + q.w()*q.z()), 2*(q.x()*q.z() - q.w()*q.y()),
						2*(q.x()*q.y() - q.w()*q.z()), (1 - 2*powf(q.x(),2.0) - 2 * powf(q.z(),2.0)), 2*(q.y()*q.z() + q.w()*q.x()),
						2*(q.x()*q.z() + q.w()*q.y()), 2*(q.y()*q.z() - q.w()*q.x()), (1 - 2*powf(q.x(),2.0) - 2*powf(q.y(),2.0));

    vprime = quaternion_matrix * v;

	return vprime;
}

Eigen::Vector3f rotate_quaternion(Eigen::Vector3f v, Eigen::Quaternionf q) {

	Eigen::Vector3f vprime;
	Eigen::Quaternionf v_prime_quat(1.0, 0.0, 0.0, 0.0);
	Eigen::Quaternionf vector_quat(0.0, v(0), v(1), v(2));

	v_prime_quat = multiplyQuaternionTimesQuaternion(multiplyQuaternionTimesQuaternion(q, vector_quat), q.conjugate());
	
	vprime(0) = v_prime_quat.x();
	vprime(1) = v_prime_quat.y();
	vprime(2) = v_prime_quat.z();

	return vprime;
}

/////////////ROS Subscribers//////////////////////////////////
void imFeatCallback(const geometry_msgs::Quaternion::ConstPtr& img_features)
{
	imgFeat(0) = img_features->x;
	imgFeat(1) = img_features->y;
	imgFeat(2) = img_features->z;
    imgFeat(3) = img_features->w;
}

void aValueCallback(const std_msgs::Float64::ConstPtr& aVal)
{
	a = aVal->data;
}

void quadVelIFCallback(const geometry_msgs::Vector3::ConstPtr& quadVel)
{
	quad_quaternions_velocity(0) = quadVel->x;
    quad_quaternions_velocity(1) = quadVel->y;
    quad_quaternions_velocity(2) = quadVel->z;
}

void quadAttQuaternionVelCallback(const geometry_msgs::Vector3::ConstPtr& quadAttQuatVel)
{
	quad_attitude_velocity(0) = quadAttQuatVel->x;
    quad_attitude_velocity(1) = quadAttQuatVel->y;
    quad_attitude_velocity(2) = quadAttQuatVel->z;
}

void quadAttQuaternionCallback(const geometry_msgs::Quaternion::ConstPtr& quadAttQuaternion)
{
    attitude_position_q.w() = quadAttQuaternion->w;
    attitude_position_q.x() = quadAttQuaternion->x;
    attitude_position_q.y() = quadAttQuaternion->y;
    attitude_position_q.z() = quadAttQuaternion->z;

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

    attitude_z_quaternion.w() = cos(yaw * 0.5);
    attitude_z_quaternion.x() = 0.0;
    attitude_z_quaternion.y() = 0.0;
    attitude_z_quaternion.z() = sin(yaw * 0.5);

	attitude_x_y_quaternion = multiplyQuaternionTimesQuaternion(attitude_x_quaternion, attitude_y_quaternion);
    attitude_quaternion = multiplyQuaternionTimesQuaternion(attitude_x_y_quaternion, attitude_z_quaternion);

    // tf::Quaternion test_q(quadAttQuaternion->x, quadAttQuaternion->y, quadAttQuaternion->z, quadAttQuaternion->w);
    // tf::Matrix3x3 m(test_q);
    // Eigen::Vector3d euler(0.0, 0.0, 0.0);
    // m.getRPY(euler[0], euler[1], euler[2]);

	// quad_att(0) = euler[0];
    // quad_att(1) = euler[1];
    // quad_att(2) = euler[2];
}

void tgtYRCallback(const std_msgs::Float64::ConstPtr& tgtYR)
{
	tgt_YR = tgtYR->data;

    tgt_YR = 0;
}

void tgtVelCallback(const geometry_msgs::Vector3::ConstPtr& tgtVel)
{
	tgt_vel(0) = tgtVel->x;
    tgt_vel(1) = tgtVel->y;
    tgt_vel(2) = tgtVel->z;

    tgt_vel(0) = 0; 
    tgt_vel(1) = 0;
    tgt_vel(2) = 0;
}

void tgtAccelCallback(const geometry_msgs::Vector3::ConstPtr& tgtAccel)
{
	tgt_accel(0) = tgtAccel->x;
    tgt_accel(1) = tgtAccel->y;
    tgt_accel(2) = tgtAccel->z;

    tgt_accel(0) = 0;
    tgt_accel(1) = 0;
    tgt_accel(2) = 0;
}

void tgtYawAccelCallback(const std_msgs::Float64::ConstPtr& tgtYawAccel)
{
	tgt_YAccel = tgtYawAccel->data;

	tgt_YAccel = 0;
}

void rcModeCallback(const std_msgs::Int32::ConstPtr& message)
{
	rcMode = message->data;
}

/////////////////////////////////Main Program//////////////////////////
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "ibvs_pos_ctrl");
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);	
    
    //ROS publishers and subscribers
    ros::Publisher error_pub = nh.advertise<geometry_msgs::Quaternion>("error_visual_servoing",100);
    ros::Publisher error_dot_pub = nh.advertise<geometry_msgs::Quaternion>("error_dot_visual_servoing",100);
    ros::Publisher adaptive_gain_pub = nh.advertise<geometry_msgs::Quaternion>("adaptive_gain",100);
    ros::Publisher asmc_pub = nh.advertise<geometry_msgs::Quaternion>("asmc_output",100);
    ros::Publisher ss_pub = nh.advertise<geometry_msgs::Quaternion>("ibvs_ss",100);
    ros::Publisher thrust_pub = nh.advertise<std_msgs::Float64>("quad_thrust",100);
    ros::Publisher desired_att_quaternions_pub = nh.advertise<geometry_msgs::Quaternion>("desired_attitude_quaternion",100);
    ros::Publisher psiddot_des_pub = nh.advertise<std_msgs::Float64>("yaw_ddot_desired",100);
    ros::Publisher z_des_pub = nh.advertise<std_msgs::Float64>("z_des",100);
    ros::Publisher linear_gain_pub = nh.advertise<geometry_msgs::Quaternion>("linear_gains",100);
    ros::Publisher desired_yaw_rate_pub = nh.advertise<std_msgs::Float64>("yaw_rate_desired",100);
	ros::Publisher linear_forces_VF_pub = nh.advertise<geometry_msgs::Vector3>("linear_forces_VF",100);
	ros::Publisher quad_accelerations_VF_pub = nh.advertise<geometry_msgs::Vector3>("quad_accs_VF",100);

    geometry_msgs::Quaternion error_var;
    geometry_msgs::Quaternion error_dot_var;
    geometry_msgs::Quaternion adaptive_gain_var;
    geometry_msgs::Quaternion asmc_var;
    geometry_msgs::Quaternion ss_var;
    std_msgs::Float64 thrust_var;
    std_msgs::Float64 yaw_ddot_des_var;
    std_msgs::Float64 z_des_var;
    geometry_msgs::Quaternion desired_attitude_quaternion_var;
    geometry_msgs::Quaternion linear_gains_var;
    std_msgs::Float64 yaw_rate_desired_var;
    std_msgs::Float64 yaw_desired_var;
    geometry_msgs::Vector3 linear_forces_VF_var;
    geometry_msgs::Vector3 quad_accs_var;

	ros::Subscriber im_feat_sub = nh.subscribe("ImFeat_vector", 100, &imFeatCallback);
    ros::Subscriber a_value_sub = nh.subscribe("a_value", 100, &aValueCallback);
    ros::Subscriber tgt_vel_sub = nh.subscribe("tgt_velocity", 100, &tgtVelCallback);
    ros::Subscriber tgt_YR_sub = nh.subscribe("tgt_yaw_rate", 100, &tgtYRCallback);
    ros::Subscriber tgt_accel_sub = nh.subscribe("tgt_acceleration", 100, &tgtAccelCallback);
    ros::Subscriber tgt_Yaw_accel_sub = nh.subscribe("tgt_yaw_acceleration", 100, &tgtYawAccelCallback);
    ros::Subscriber quad_vel_IF_sub = nh.subscribe("velocity_estimates", 100, &quadVelIFCallback);
    ros::Subscriber quad_att_quaternion_sub = nh.subscribe("attitude_QUAV", 100, &quadAttQuaternionCallback);
    ros::Subscriber quad_attQuaternionVel_sub = nh.subscribe("attVel_estimates", 100, &quadAttQuaternionVelCallback);
    ros::Subscriber rcMode_sub = nh.subscribe("rcMode", 10, &rcModeCallback);

    ros::Publisher ATTITUDE_DESIRED_EULER = nh.advertise<geometry_msgs::Vector3>("ATTITUDE_DESIRED_EULER",100);
    geometry_msgs::Vector3 ATTITUDE_DES_EULER_VAR;

    ros::Publisher ATTITUDE_EULER = nh.advertise<geometry_msgs::Vector3>("ATTITUDE_EULER",100);
    geometry_msgs::Vector3 ATTITUDE_EULER_VAR;

    imgFeat_des << 0,0,1,0;

    K1 << 0, 0, 0, 0;
    K1_dot << 0, 0, 0, 0;

    // Visual-servoing controller variables
    xi_1_visualServoing << 20, 20, 6, 8;
    lambda_visualServoing << 2, 2, 2, 2;
    xi_2_visualServoing << 3, 3, 3, 10;
    varpi_visualServoing << 4, 4, 4, 4;
    vartheta_visualServoing << 3, 3, 3, 3;
    // K2 << 0.1, 0.1, 0.4, 0.4;
    // k_reg << 0.05, 0.05, 0.5, 0.1;
    // kmin << 0.01, 0.01, 0.01, 0.1;
    // mu << 0.05, 0.05, 0.1, 0.1;
    //alpha_visualServoing << 0.008, 0.007, 0.5, 0.05;
    alpha_visualServoing << 0.0001, 0.0001, 0.5, 0.5;
    beta_visualServoing << 5, 5, 10, 0.5;

    // Vicon controller variables
    xi_1_vicon << 20, 20, 6, 8;
    lambda_vicon << 2, 2, 2, 2;
    xi_2_vicon << 3, 3, 3, 10;
    varpi_vicon << 4, 4, 4, 4;
    vartheta_vicon << 3, 3, 3, 3;
    alpha_vicon << 0.0001, 0.0001, 0.5, 0.5;
    beta_vicon << 5, 5, 10, 0.5;

    kappa_dot << 0,0,0,0;
    e3 << 0,0,1;

    thrust_var.data = thrust;
    thrust_pub.publish(thrust_var);

    desired_attitude_quaternion_var.w = 1.0;
    desired_attitude_quaternion_var.x = 0.0;
    desired_attitude_quaternion_var.y = 0.0;
    desired_attitude_quaternion_var.z = 0.0;
	desired_att_quaternions_pub.publish(desired_attitude_quaternion_var);
    ros::Duration(0.01).sleep();
    //ros::Duration(2.05).sleep();
    
    while(ros::ok()) {   
        
        // Visual-servoing mode
        if (rcMode == 1) {

            imgFeatLinear << imgFeat(0),imgFeat(1),imgFeat(2); 
            tgt_vel_VF = rotate_quaternion(tgt_vel, attitude_z_quaternion.conjugate());
            quad_vel_VF = rotate_quaternion(quad_quaternions_velocity, attitude_z_quaternion.conjugate());
            // std::cout << "QUAD velocity in IF: " << quad_quaternions_velocity << '\n';
            // std::cout << "QUAD velocity in VF: " << quad_vel_VF << '\n';
            yawVel_e3 << 0,0,quad_attitude_velocity(2);
            imgFeatLinear_dot = -skewMatrix(yawVel_e3) * imgFeatLinear - (1/zD) * quad_vel_VF + (1/zD) * tgt_vel_VF;
            //std::cout << "imgFeatLinear_dot: " << imgFeatLinear_dot << '\n';

            Omega << (-1/zD),0,0,imgFeat(1), 
            0,(-1/zD),0,-imgFeat(0),
            0,0,(-1/zD),0,
            0,0,0,-1;
        
            v_imgFeat << quad_vel_VF(0),quad_vel_VF(1),quad_vel_VF(2),quad_attitude_velocity(2);
            //v_imgFeat << quad_vel_VF(0),quad_vel_VF(1),quad_vel_VF(2),quad_attVel(2);
            kappa << (tgt_vel_VF(0)/zD), (tgt_vel_VF(1)/zD), (tgt_vel_VF(2)/zD), tgt_YR; 
            
            // std::cout << "errors: " << imgFeat_des - imgFeat << '\n';

            // IMPORTANT: consider that when the drone does not see the target, the image features 
            // is a vector of 0, 0, 1, 0. Meaning, there should be no error and the drone should 
            // only hover. For this, all errors should be 0, but sometimes the yaw accelerations will
            // report noise, and since we have a double integrator to obtain the yaw desired, this error
            // is doubly propagated, generating an increasing yaw desired even if there is no target 
            // to detect. For this, check if the image features are indeed 0, 0, 1, 0. If they are, 
            // then set all the errors to 0 manually in order to eliminate the increase.
            if (imgFeat(0) == 0 && imgFeat(1) == 0 && imgFeat(2) == 1 && imgFeat(3) == 0) {

                error << 0.0, 0.0, 0.0, 0.0;
                error_dot << 0.0, 0.0, 0.0, 0.0;
                ibvs_ctrl_input << 0.0, 0.0, 0.0, 0.0;
                yawRate_desired = 0.0;
                yaw_desired = 0.0;
                thrust = quad_mass * gravity;
                attitude_desired_quaternion.w() = 1.0;
                attitude_desired_quaternion.x() = 0.0;
                attitude_desired_quaternion.y() = 0.0;
                attitude_desired_quaternion.z() = 0.0;

            }

            else {

                error = imgFeat_des - imgFeat;
                error_dot = -(Omega*v_imgFeat) - kappa;

                //Sliding surfaces and adaptive sliding mode controller
                for (int i = 0; i<=3; i++) {

                    ss(i) = error(i) + xi_1_visualServoing(i) * powf(std::abs(error(i)),lambda_visualServoing(i)) * sign(error(i)) + xi_2_visualServoing(i) * powf(std::abs(error_dot(i)),(varpi_visualServoing(i)/vartheta_visualServoing(i))) * sign(error_dot(i));
                    
                    // *************** Traditional adaptive law ***************
                    // if (K1(i)>kmin(i))
                    // {
                    //     K1_dot(i) = k_reg(i)*sign(std::abs(ss(i))-mu(i));
                    // }
                    // else
                    // {
                    //     K1_dot(i) = kmin(i);
                    // }

                    // K1(i) = K1(i) + step_size*K1_dot(i);
                    // asmc(i) = -K1(i) * powf(std::abs(ss(i)),0.5) * sign(ss(i)) - K2(i) * ss(i);

                    // *************** Modified adaptive law ***************
                    K1_dot(i) = sqrt(alpha_visualServoing(i)) * sqrt(std::abs(ss(i))) - sqrt(beta_visualServoing(i)) * powf(K1(i),2.0);

                    K1(i) = K1(i) + step_size*K1_dot(i);
                    asmc(i) = -2 * K1(i) * sqrt(std::abs(ss(i))) * sign(ss(i)) - (powf(K1(i),2) / 2.0) * ss(i);
                
                }
                
                //Control inputs
                /////////////yaw_rotation///////////////////////
                ibvs_ctrl_input(3) = -(-asmc(3) - tgt_YAccel + (vartheta_visualServoing(3)/(varpi_visualServoing(3)*xi_2_visualServoing(3))) * sign(error_dot(3)) * powf(std::abs(error_dot(3)),(2-(varpi_visualServoing(3)/vartheta_visualServoing(3)))) * (1 + xi_1_visualServoing(3) * lambda_visualServoing(3) * powf(std::abs(error(3)),lambda_visualServoing(3)-1))); //yaw_ddot
                /////////////x-axis///////////////////////
                ibvs_ctrl_input(0) = -zD * (-asmc(0)  - ibvs_ctrl_input(3) * imgFeatLinear(1) - quad_attitude_velocity(2) * imgFeatLinear_dot(1) - (1/zD) * (tgt_accel(0)) + (vartheta_visualServoing(0)/(varpi_visualServoing(0)*xi_2_visualServoing(0))) * sign(error_dot(0)) * powf(std::abs(error_dot(0)),(2-(varpi_visualServoing(0)/vartheta_visualServoing(0)))) * (1 + xi_1_visualServoing(0) * lambda_visualServoing(0) * powf(std::abs(error(0)),lambda_visualServoing(0)-1)));
                // ibvs_ctrl_input(0) = -zD * (-asmc(0)  - ibvs_ctrl_input(3) * imgFeatLinear(1) - quad_attVel(2) * imgFeatLinear_dot(1) + (vartheta_visualServoing(0)/(varpi_visualServoing(0)*xi_2_visualServoing(0))) * sign(error_dot(0)) * powf(std::abs(error_dot(0)),(2-(varpi_visualServoing(0)/vartheta_visualServoing(0)))) * (1 + xi_1_visualServoing(0) * lambda_visualServoing(0) * powf(std::abs(error(0)),lambda_visualServoing(0)-1)));
                /////////////y-axis///////////////////////
                ibvs_ctrl_input(1) = -zD * (-asmc(1)  + ibvs_ctrl_input(3) * imgFeatLinear(0) + quad_attitude_velocity(2) * imgFeatLinear_dot(0) - (1/zD) * (tgt_accel(1)) + (vartheta_visualServoing(1)/(varpi_visualServoing(1)*xi_2_visualServoing(1))) * sign(error_dot(1)) * powf(std::abs(error_dot(1)),(2-(varpi_visualServoing(1)/vartheta_visualServoing(1)))) * (1 + xi_1_visualServoing(1) * lambda_visualServoing(1) * powf(std::abs(error(1)),lambda_visualServoing(1)-1)));
                // std::cout << "ASMC: " << -asmc(1) << '\n' << '\n';
                // std::cout << "ibvs_ctrl_input(3) * imgFeatLinear(0): " << ibvs_ctrl_input(3) * imgFeatLinear(0) << '\n' << '\n';
                // std::cout << "quad_attitude_velocity(2) * imgFeatLinear_dot(0): " << quad_attitude_velocity(2) * imgFeatLinear_dot(0) << '\n' << '\n';
                // std::cout << "(1/zD) * tgt_accel(1): " << - (1/zD) * tgt_accel(1) << '\n';
                // std::cout << "(vartheta_visualServoing(1)/(varpi_visualServoing(1)*xi_2_visualServoing(1))) * sign(error_dot(1)) * powf(std::abs(error_dot(1)),(2-(varpi_visualServoing(1)/vartheta_visualServoing(1)))) * (1 + xi_1_visualServoing(1) * lambda_visualServoing(1) * powf(std::abs(error(1)),lambda_visualServoing(1)-1)): " << (vartheta_visualServoing(1)/(varpi_visualServoing(1)*xi_2_visualServoing(1))) * sign(error_dot(1)) * powf(std::abs(error_dot(1)),(2-(varpi_visualServoing(1)/vartheta_visualServoing(1)))) * (1 + xi_1_visualServoing(1) * lambda_visualServoing(1) * powf(std::abs(error(1)),lambda_visualServoing(1)-1)) << '\n' << '\n';
                // ibvs_ctrl_input(1) = -zD * (-asmc(1)  + ibvs_ctrl_input(3) * imgFeatLinear(0) + quad_attVel(2) * imgFeatLinear_dot(0) + (vartheta_visualServoing(1)/(varpi_visualServoing(1)*xi_2_visualServoing(1))) * sign(error_dot(1)) * powf(std::abs(error_dot(1)),(2-(varpi_visualServoing(1)/vartheta_visualServoing(1)))) * (1 + xi_1_visualServoing(1) * lambda_visualServoing(1) * powf(std::abs(error(1)),lambda_visualServoing(1)-1)));
                /////////////z-axis///////////////////////
                ibvs_ctrl_input(2) = -zD * (-asmc(2) - (1/zD) * (tgt_accel(2)) + (vartheta_visualServoing(2)/(varpi_visualServoing(2)*xi_2_visualServoing(2))) * sign(error_dot(2)) * powf(std::abs(error_dot(2)),(2-(varpi_visualServoing(2)/vartheta_visualServoing(2)))) * (1 + xi_1_visualServoing(2) * lambda_visualServoing(2) * powf(std::abs(error(2)),lambda_visualServoing(2)-1)));       

                //Quad's virtual frame dynamics 
                quad_accel_VF << ibvs_ctrl_input(0), ibvs_ctrl_input(1), ibvs_ctrl_input(2); 
                // std::cout << "Quad accelerations VF: " << quad_accel_VF << '\n';
                quad_linear_forces_VF = (quad_mass * quad_accel_VF) + (quad_mass * skewMatrix(yawVel_e3)) * quad_vel_VF;
                //quad_linear_forces_VF = {1.0, 0.0, 0.0};
                //std::cout << "Quad linear forces VF: " << quad_linear_forces_VF << '\n';
                //////////////////Thrust///////////////////////////////
                thrust = ((e3 * (gravity * quad_mass)) - quad_linear_forces_VF).norm();
                
                if (thrust > 30) {
                    thrust = 30;
                }

                else if (thrust < 0) {
                    thrust = 0;
                }

                /////////////////Desired attitude//////////////////
                //Calculate yaw desired
                yawRate_desired = yawRate_desired + step_size * ibvs_ctrl_input(3);
                yaw_desired = yaw_desired + step_size * yawRate_desired; //Yaw desired in radians
                
                std::cout << "Yaw acceleration: " << std::endl;
                std::cout << ibvs_ctrl_input(3) << std::endl;

                std::cout << "Yaw desired: " << std::endl;
                std::cout << yaw_desired << std::endl;

                attitude_desired_quaternion_z.w() = cos(yaw_desired * 0.5);
                attitude_desired_quaternion_z.x() = 0.0;
                attitude_desired_quaternion_z.y() = 0.0;
                attitude_desired_quaternion_z.z() = sin(yaw_desired * 0.5);

                // Calculate roll and pitch desired
                // float roll_des_arg = asin(quad_linear_forces_VF(1)/thrust);
                // float pitch_des_arg = asin(-quad_linear_forces_VF(0)/(thrust*cos(roll_des_arg)));
                // std::cout << "Roll and pitch desired: " << std::endl;
                // std::cout << roll_des_arg << std::endl;
                // std::cout << pitch_des_arg << std::endl;

                // Eigen::Quaternionf attitude_desired_quaternion_x(1.0, 0.0, 0.0, 0.0);
                // Eigen::Quaternionf attitude_desired_quaternion_y(1.0, 0.0, 0.0, 0.0);

                // attitude_desired_quaternion_x.w() = cos(roll_des_arg * 0.5);
                // attitude_desired_quaternion_x.x() = sin(roll_des_arg * 0.5);
                // attitude_desired_quaternion_x.y() = 0.0;
                // attitude_desired_quaternion_x.z() = 0.0;

                // attitude_desired_quaternion_y.w() = cos(pitch_des_arg * 0.5);
                // attitude_desired_quaternion_y.x() = 0.0;
                // attitude_desired_quaternion_y.y() = sin(pitch_des_arg * 0.5);
                // attitude_desired_quaternion_y.z() = 0.0;

                // First rotate the forces to the inertial frame 
                Eigen::Vector3f quad_linear_forces_IF = rotate_quaternion(quad_linear_forces_VF, attitude_quaternion);
                quad_linear_forces_IF = (e3 * (gravity * quad_mass)) + quad_linear_forces_IF;
                // std::cout << "Virtual linear forces vector in IF: " << std::endl;
                // std::cout << quad_linear_forces_IF << std::endl;
                nu = quad_linear_forces_IF.normalized();
                // quad_linear_forces_VF = (e3 * (gravity * quad_mass)) + quad_linear_forces_VF;
                // std::cout << "Virtual linear forces vector in VF: " << std::endl;
                // std::cout << quad_linear_forces_VF << std::endl;
                // nu = quad_linear_forces_VF.normalized();
                // std::cout << "Virtual linear forces: " << std::endl;
                // std::cout << nu << std::endl;
                if (nu.dot(nz) == 1.0 || nu.dot(nz) == -1.0) {
                    attitude_desired_quaternion_x_y.w() = 1.0;
                }
                else {
                    attitude_desired_quaternion_x_y.w() = sqrt((1.0 + nu.dot(nz)) / 2.0);
                }   

                cross_product = nu.cross(nz);
                if (cross_product.norm() == 0.0) {
                    attitude_desired_quaternion_x_y.x() = 0.0;
                    attitude_desired_quaternion_x_y.y() = 0.0;
                    attitude_desired_quaternion_x_y.z() = 0.0;
                } 
                else {
                    imaginary_part = (cross_product / cross_product.norm()) * sqrt((1.0 - nu.dot(nz)) / 2.0);
                    //std::cout << "Imaginary part: " << imaginary_part << std::endl;
                    attitude_desired_quaternion_x_y.x() = imaginary_part(0);
                    attitude_desired_quaternion_x_y.y() = imaginary_part(1);
                    attitude_desired_quaternion_x_y.z() = imaginary_part(2);
                }

                // std::cout << "Dot product of nu and nz: " << nu.dot(nz) << std::endl;
                // std::cout << "Cross profuct of nu and nz: " << cross_product << std::endl;

                attitude_desired_quaternion = multiplyQuaternionTimesQuaternion(attitude_desired_quaternion_x_y, attitude_desired_quaternion_z);
                // attitude_desired_quaternion_x_y = multiplyQuaternionTimesQuaternion(attitude_desired_quaternion_x, attitude_desired_quaternion_y);
                // attitude_desired_quaternion = multiplyQuaternionTimesQuaternion(attitude_desired_quaternion_x_y, attitude_desired_quaternion_z);
                attitude_desired_quaternion = attitude_desired_quaternion.normalized();
                
                /////////////////////////////////////////////////////////////////////
                //// CAREFUL: THIS IS ONLY FOR TUNING THE ATTITUDE CONTROL //////////
                //// WARNING: REMOVE THIS PART WHEN FINISHED TUNING ATTITUDE ////////
                /////////////////////////////////////////////////////////////////////
                // attitude_desired_quaternion.w() = 1.0;
                // attitude_desired_quaternion.x() = 0.0;
                // attitude_desired_quaternion.y() = 0.0;
                // attitude_desired_quaternion.z() = 0.0;
                
                // tf::Quaternion test_q(attitude_desired_quaternion.x(), attitude_desired_quaternion.y(), attitude_desired_quaternion.z(), attitude_desired_quaternion.w());
                // tf::Matrix3x3 m(test_q);
                // Eigen::Vector3d euler(0.0, 0.0, 0.0);
                // m.getRPY(euler[0], euler[1], euler[2]);
                // std::cout << "Quaternion desired" << std::endl;
                // std::cout << attitude_desired_quaternion.w() << std::endl;
                // std::cout << attitude_desired_quaternion.x() << std::endl;
                // std::cout << attitude_desired_quaternion.y() << std::endl;
                // std::cout << attitude_desired_quaternion.z() << std::endl;
                // std::cout << "Angles desired Quaternions" << std::endl;
                // std::cout << euler[0] << std::endl;
                // std::cout << euler[1] << std::endl;
                // std::cout << euler[2] << std::endl;
            }
        }

        // VICON-guided
        else {

            //Sliding surfaces and adaptive sliding mode controller
            for (int i = 0; i<=3; i++) {

                ss(i) = error(i) + xi_1_visualServoing(i) * powf(std::abs(error(i)),lambda_visualServoing(i)) * sign(error(i)) + xi_2_visualServoing(i) * powf(std::abs(error_dot(i)),(varpi_visualServoing(i)/vartheta_visualServoing(i))) * sign(error_dot(i));
                
                // *************** Traditional adaptive law ***************
                // if (K1(i)>kmin(i))
                // {
                //     K1_dot(i) = k_reg(i)*sign(std::abs(ss(i))-mu(i));
                // }
                // else
                // {
                //     K1_dot(i) = kmin(i);
                // }

                // K1(i) = K1(i) + step_size*K1_dot(i);
                // asmc(i) = -K1(i) * powf(std::abs(ss(i)),0.5) * sign(ss(i)) - K2(i) * ss(i);

                // *************** Modified adaptive law ***************
                K1_dot(i) = sqrt(alpha_visualServoing(i)) * sqrt(std::abs(ss(i))) - sqrt(beta_visualServoing(i)) * powf(K1(i),2.0);

                K1(i) = K1(i) + step_size*K1_dot(i);
                asmc(i) = -2 * K1(i) * sqrt(std::abs(ss(i))) * sign(ss(i)) - (powf(K1(i),2) / 2.0) * ss(i);
            
            }       

        }
        
 
        //Publishing data
        //error
        error_var.x = error(0);
        error_var.y = error(1);
        error_var.z = error(2);
        error_var.w = error(3);
        //K1
        adaptive_gain_var.x = K1(0);
        adaptive_gain_var.y = K1(1);
        adaptive_gain_var.z = K1(2);
        adaptive_gain_var.w = K1(3);
        //asmc
        asmc_var.x = asmc(0);
        asmc_var.y = asmc(1);
        asmc_var.z = asmc(2);
        asmc_var.w = asmc(3);
        
        //Thrust
        thrust_var.data = thrust;
        
        // Desired attitude quaternion
        desired_attitude_quaternion_var.w = attitude_desired_quaternion.w();
        desired_attitude_quaternion_var.x = attitude_desired_quaternion.x();
        desired_attitude_quaternion_var.y = attitude_desired_quaternion.y();
        desired_attitude_quaternion_var.z = attitude_desired_quaternion.z();

        tf::Quaternion test_q(attitude_desired_quaternion.x(), attitude_desired_quaternion.y(), attitude_desired_quaternion.z(), attitude_desired_quaternion.w());
        tf::Matrix3x3 m(test_q);
        Eigen::Vector3d euler(0.0, 0.0, 0.0);
        m.getRPY(euler[0], euler[1], euler[2]);
        std::cout << "Desired attitude (Euler): " << std::endl;
        std::cout << euler[0] << std::endl;
        std::cout << euler[1] << std::endl;
        std::cout << euler[2] << std::endl;
        
        // Yaw rate desired
        yaw_rate_desired_var.data = yawRate_desired;
        
        yaw_ddot_des_var.data = ibvs_ctrl_input(3);

        ss_var.x = ss(0);
        ss_var.y = ss(1);
        ss_var.z = ss(2);
        ss_var.w = ss(3);

        error_dot_var.x = error_dot(0);
        error_dot_var.y = error_dot(1);
        error_dot_var.z = error_dot(2);
        error_dot_var.w = error_dot(3);

        z_des_var.data = -zD;

        linear_gains_var.x = K1(0);
        linear_gains_var.y = K1(1);
        linear_gains_var.z = K1(2);
        linear_gains_var.w = K1(3);
        linear_forces_VF_var.x = quad_linear_forces_VF(0);
        linear_forces_VF_var.y = quad_linear_forces_VF(1);
        linear_forces_VF_var.z = quad_linear_forces_VF(2);

        quad_accs_var.x = quad_accel_VF(0);
        quad_accs_var.y = quad_accel_VF(1);
        quad_accs_var.z = quad_accel_VF(2);

        error_pub.publish(error_var);
        adaptive_gain_pub.publish(adaptive_gain_var);
        asmc_pub.publish(asmc_var);
        thrust_pub.publish(thrust_var);
        desired_att_quaternions_pub.publish(desired_attitude_quaternion_var);
        psiddot_des_pub.publish(yaw_ddot_des_var);
        error_dot_pub.publish(error_dot_var);
        ss_pub.publish(ss_var);
        z_des_pub.publish(z_des_var);
        linear_gain_pub.publish(linear_gains_var);
        desired_yaw_rate_pub.publish(yaw_rate_desired_var);
        linear_forces_VF_pub.publish(linear_forces_VF_var);
        quad_accelerations_VF_pub.publish(quad_accs_var);

        // ATTITUDE_DES_EULER_VAR.x = roll_des_arg;
        // ATTITUDE_DES_EULER_VAR.y = pitch_des_arg;
        // ATTITUDE_DES_EULER_VAR.z = yaw_desired;
        // ATTITUDE_DESIRED_EULER.publish(ATTITUDE_DES_EULER_VAR);

        ATTITUDE_EULER_VAR.x = roll;
        ATTITUDE_EULER_VAR.y = pitch;
        ATTITUDE_EULER_VAR.z = yaw;
        ATTITUDE_EULER.publish(ATTITUDE_EULER_VAR);

        // std::cout << "error: " << error << std::endl;
        //std::cout << "pitch_des " << attitude_desired(1) << std::endl;

        ros::spinOnce();
		loop_rate.sleep();
    }

    return 0;
}

