#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include <iostream>
#include <math.h>
#include <cmath>
#include <vector>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>

//Declaring global variables
cv::Mat frame;

int ug, ug1, ug2, ug3, ug4;
int ng, ng1, ng2, ng3, ng4;
int p1,p2,p3,p4;
int p1x,p2x,p3x,p4x;
int p1y,p2y,p3y,p4y;

float roll, pitch = 0.0;
int rows, cols = 0;

Eigen::Vector3f p1_vs_cf;
Eigen::Vector3f p2_vs_cf;
Eigen::Vector3f p3_vs_cf;
Eigen::Vector3f p4_vs_cf;

Eigen::Vector3f p1_vs_vf;
Eigen::Vector3f p2_vs_vf;
Eigen::Vector3f p3_vs_vf;
Eigen::Vector3f p4_vs_vf;

Eigen::Vector3f uav_att;

Eigen::Vector3f e3;

Eigen::Quaternionf quaternion_x(1.0, 0.0, 0.0, 0.0);
Eigen::Quaternionf quaternion_y(1.0, 0.0, 0.0, 0.0);
Eigen::Quaternionf quaternion_x_y(1.0, 0.0, 0.0, 0.0);

float beta_p1;
float beta_p2;
float beta_p3;
float beta_p4;

float ug_vs, ng_vs;
float mu20,mu02,mu11;
float den;

float qx,qy,qz,qpsi,qyaw;
float a;
//float aD = 0.000001256;
//float aD = 0.0000005556;
//float aD = 0.0000007970; // 2.5 meters
float aD = 0.00000019713; // 1.5 meters

//Camera intrinsic parameters
float focal_length = 0.00304;
//float pixel_size = 0.00000112; //frame size = raspy cam's default image size
float pixel_size = 0.00000876923; //For frame_size = 410x308
//float new_focal_length = focal_length*pixel_size;

//////////////////////////Quaternions setup for math operations//////////
Eigen::Quaternionf q(1.0, 0.0, 0.0, 0.0);
Eigen::Vector3f quaternion_angle_axis(0.0, 0.0, 0.0);

/////////////////////////Functions///////////////////////////////

//Matrix R_phi_theta
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

Eigen::Matrix3f Ryaw(float yaw)
{
    Eigen::Matrix3f yaw_mat;
    yaw_mat << cos(yaw), -sin(yaw),0,
                sin(yaw), cos(yaw), 0,
                0, 0, 1;
    return yaw_mat;
}

Eigen::Vector3f rotate_vector_by_quaternion_passive(Eigen::Vector3f v, Eigen::Quaternionf q)
{

	Eigen::Vector3f vprime;
	Eigen::Matrix3f quaternion_matrix;
	
	quaternion_matrix << (1-2*pow(q.y(),2.0)-2*pow(q.z(),2.0)), 2*(q.x()*q.y() + q.w()*q.z()), 2*(q.x()*q.z() - q.w()*q.y()),
						2*(q.x()*q.y() - q.w()*q.z()), (1 - 2*pow(q.x(),2.0) - 2 * pow(q.z(),2.0)), 2*(q.y()*q.z() + q.w()*q.x()),
						2*(q.x()*q.z() + q.w()*q.y()), 2*(q.y()*q.z() - q.w()*q.x()), (1 - 2*pow(q.x(),2.0) - 2*pow(q.y(),2.0));

    vprime = quaternion_matrix * v;

	return vprime;
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

/////////////////////ROS Subscribers//////////////////////////////////
void attitude_quaternion_callback(const geometry_msgs::Quaternion::ConstPtr& attQuat)
{
	roll  = atan2(2.0 * (attQuat->w * attQuat->y + attQuat->w * attQuat->x) , 1.0 - 2.0 * (attQuat->x * attQuat->x + attQuat->y * attQuat->y));
	pitch = asin(2.0 * (attQuat->y * attQuat->w - attQuat->z * attQuat->x));

	quaternion_x.w() = cos(roll * 0.5);
	quaternion_x.x() = sin(roll * 0.5);
	quaternion_x.y() = 0.0;
	quaternion_x.z() = 0.0;

	quaternion_y.w() = cos(pitch * 0.5);
	quaternion_y.x() = 0.0;
	quaternion_y.y() = sin(pitch * 0.5);
	quaternion_y.z() = 0.0;

	// std::cout << "Roll: " << std::endl;
	// std::cout << roll << std::endl;
	// std::cout << "Pitch: " << std::endl;
	// std::cout << pitch << std::endl;
	// std::cout << "Quaternion x: " << std::endl;
	// std::cout << quaternion_x.w() << std::endl;
	// std::cout << quaternion_x.x() << std::endl;
	// std::cout << quaternion_x.y() << std::endl;
	// std::cout << quaternion_x.z() << std::endl;

	quaternion_x_y = multiplyQuaternionTimesQuaternion(quaternion_x, quaternion_y);
}

////////////////////Main program//////////////////////////////////////
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "image_features");
	ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
	ros::Rate loop_rate(100);

    //ROS publishers and subscribers
	ros::Publisher im_feat_pub = nh.advertise<geometry_msgs::Quaternion>("ImFeat_vector",100);
    ros::Publisher a_value_pub = nh.advertise<std_msgs::Float64>("a_value",100);
    ros::Publisher punto1_pub = nh.advertise<geometry_msgs::Pose2D>("point_one",100);
	ros::Publisher punto2_pub = nh.advertise<geometry_msgs::Pose2D>("point_two",100);
	ros::Publisher punto3_pub = nh.advertise<geometry_msgs::Pose2D>("point_three",100);
	ros::Publisher punto4_pub = nh.advertise<geometry_msgs::Pose2D>("point_four",100);
	ros::Publisher centroid_pub = nh.advertise<geometry_msgs::Pose2D>("centroid",100);
	image_transport::Publisher image_pub = it.advertise("camera/image/image_processed", 1);

    //image_transport::Subscriber sub = it.subscribe("camera/image", 100, imageCallback); //Real camera
	ros::Subscriber attitude_sub = nh.subscribe("attitude_QUAV",100, &attitude_quaternion_callback);

    //Declaring local variables
    geometry_msgs::Quaternion im_feat_vec;
    std_msgs::Float64 a_val;
	geometry_msgs::Pose2D punto1_vis;
	geometry_msgs::Pose2D punto2_vis;
	geometry_msgs::Pose2D punto3_vis;
	geometry_msgs::Pose2D punto4_vis;
	geometry_msgs::Pose2D centroid;
	sensor_msgs::ImagePtr msg;

    e3 << 0,0,1;
	
	//Loading the dictionary where the aruco markers belong to
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);

    // Open the camera using OpenCV
    cv::VideoCapture cap(0);  // Use camera index 0, adjust as needed

    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera!");
        return -1;
    }

    cap.set(cv::CAP_PROP_CONVERT_RGB, false);

    // Set the desired image size
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

	// Set the desired frame rate (adjust as needed)
    cap.set(cv::CAP_PROP_FPS, 80);  // Set the frame rate to 30 FPS

    while (ros::ok())
	{

        //Initializing the detector parameters using default values
		cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
		//Declaring the 2D vectors that contain the aruco's corners and rejected candidates
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectCandidates;
		//Declaring a vector to save de ID numbers of the detected arucos
		std::vector<int> markerIds;

        // Get frame
        cv::Mat frame;
        cap >> frame;

        // Rotate image this will depend on how the camera is PHYSICALLY installed
        cv::rotate(frame, frame, cv::ROTATE_90_CLOCKWISE);

		if(!frame.empty())
		{
			//Detect the markers in the image
		cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectCandidates);
		}
		else
		{
			std::cout << "empty " << std::endl;
			qx = 0;
			qy = 0;
			qz = 1;
			qpsi  = 0;
			//Publishing data via Rostopics
            im_feat_vec.x = qx;
            im_feat_vec.y = qy;
            im_feat_vec.z = qz;
            im_feat_vec.w = qpsi;

			im_feat_pub.publish(im_feat_vec);
		}

        //If no arucos are detected, then print on screen
		if (markerIds.size() != 4)
		{
            std::cout << "I see " << markerIds.size() << " arucos only. I need 4 to work properly" << std::endl;
			qx = 0;
			qy = 0;
			qz = 1;
			qpsi  = 0;

			 //Publishing data via Rostopics
            im_feat_vec.x = qx;
            im_feat_vec.y = qy;
            im_feat_vec.z = qz;
            im_feat_vec.w = qpsi;

			im_feat_pub.publish(im_feat_vec);
		}

		else if (markerIds.size() == 4)
        {
            //Assignment of Point 1
			if (markerIds[0] == 6)
			{
				//Corner extraction
				cv::Point p11 = markerCorners[0].at(0);
				cv::Point p21 = markerCorners[0].at(1);
				cv::Point p31 = markerCorners[0].at(2);
				cv::Point p41 = markerCorners[0].at(3);

				//Centroid of the aruco
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4;
			}
			else if (markerIds[1] == 6)
			{
				//Corner extraction
				cv::Point p11 = markerCorners[1].at(0);
				cv::Point p21 = markerCorners[1].at(1);
				cv::Point p31 = markerCorners[1].at(2);
				cv::Point p41 = markerCorners[1].at(3);

				//Centroid of the aruco
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4;
			}
			else if (markerIds[2] == 6)
			{
				//Corner extraction
				cv::Point p11 = markerCorners[2].at(0);
				cv::Point p21 = markerCorners[2].at(1);
				cv::Point p31 = markerCorners[2].at(2);
				cv::Point p41 = markerCorners[2].at(3);

				//Centroid of the aruco
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4;
			}
			else if (markerIds[3] == 6)
			{
				//Corner extraction
				cv::Point p11 = markerCorners[3].at(0);
				cv::Point p21 = markerCorners[3].at(1);
				cv::Point p31 = markerCorners[3].at(2);
				cv::Point p41 = markerCorners[3].at(3);

				//Centroid of the aruco
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4;
			}
			///////////////////////////////////////////////////////////////////////////////////////////////////////////
			//Assignment of Point 2
			if (markerIds[0] == 4)
			{
				//Corner extraction
				cv::Point p12 = markerCorners[0].at(0);
				cv::Point p22 = markerCorners[0].at(1);
				cv::Point p32 = markerCorners[0].at(2);
				cv::Point p42 = markerCorners[0].at(3);

				//Centroid of the aruco
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4;
			}
			else if (markerIds[1] == 4)
			{
				//Corner extraction
				cv::Point p12 = markerCorners[1].at(0);
				cv::Point p22 = markerCorners[1].at(1);
				cv::Point p32 = markerCorners[1].at(2);
				cv::Point p42 = markerCorners[1].at(3);

				//Centroid of the aruco
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4;
			}
			else if (markerIds[2] == 4)
			{
				//Corner extraction
				cv::Point p12 = markerCorners[2].at(0);
				cv::Point p22 = markerCorners[2].at(1);
				cv::Point p32 = markerCorners[2].at(2);
				cv::Point p42 = markerCorners[2].at(3);

				//Centroid of the aruco
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4;
			}
			else if (markerIds[3] == 4)
			{
				//Corner extraction
                cv::Point p12 = markerCorners[3].at(0);
				cv::Point p22 = markerCorners[3].at(1);
				cv::Point p32 = markerCorners[3].at(2);
				cv::Point p42 = markerCorners[3].at(3);

                //Centroid of the aruco
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4;
			}
			///////////////////////////////////////////////////////////////////////////////77
			//Assignment of Point 3
			if (markerIds[0] == 8)
			{
				//Corner extraction
				cv::Point p13 = markerCorners[0].at(0);
				cv::Point p23 = markerCorners[0].at(1);
				cv::Point p33 = markerCorners[0].at(2);
				cv::Point p43 = markerCorners[0].at(3);

				//Centroid of the aruco
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4;
			}
			else if (markerIds[1] == 8)
			{
				//Corner extraction
				cv::Point p13 = markerCorners[1].at(0);
				cv::Point p23 = markerCorners[1].at(1);
				cv::Point p33 = markerCorners[1].at(2);
				cv::Point p43 = markerCorners[1].at(3);

				//Centroid of the aruco
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4;
			}
			else if (markerIds[2] == 8)
			{
				//Corner extraction
				cv::Point p13 = markerCorners[2].at(0);
				cv::Point p23 = markerCorners[2].at(1);
				cv::Point p33 = markerCorners[2].at(2);
				cv::Point p43 = markerCorners[2].at(3);

				//Centroid of the aruco
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4;
			}
			else if (markerIds[3]==8)
			{
				//Corner extraction
				cv::Point p13 = markerCorners[3].at(0);
				cv::Point p23 = markerCorners[3].at(1);
				cv::Point p33 = markerCorners[3].at(2);
				cv::Point p43 = markerCorners[3].at(3);

				//Centroid of the aruco
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4;
			}
			////////////////////////////////////////////////////////////////////////////////////
			//Assignment of Point 4
			if (markerIds[0] == 10)
			{
				//Corner extraction
        		cv::Point p14 = markerCorners[0].at(0);
				cv::Point p24 = markerCorners[0].at(1);
				cv::Point p34 = markerCorners[0].at(2);
				cv::Point p44 = markerCorners[0].at(3);

				//Centroid of the aruco
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4;
			}
			else if (markerIds[1] == 10)
			{
				//Corner extraction
				cv::Point p14 = markerCorners[1].at(0);
				cv::Point p24 = markerCorners[1].at(1);
				cv::Point p34 = markerCorners[1].at(2);
				cv::Point p44 = markerCorners[1].at(3);

				//Centroid of the aruco
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4;
			}
			else if (markerIds[2] == 10)
			{
				//Corner extraction
				cv::Point p14 = markerCorners[2].at(0);
				cv::Point p24 = markerCorners[2].at(1);
				cv::Point p34 = markerCorners[2].at(2);
				cv::Point p44 = markerCorners[2].at(3);

				//Centroid of the aruco
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4;
			}
			else if (markerIds[3] == 10)
			{
				//Corner extraction
				cv::Point p14 = markerCorners[3].at(0);
				cv::Point p24 = markerCorners[3].at(1);
				cv::Point p34 = markerCorners[3].at(2);
				cv::Point p44 = markerCorners[3].at(3);

				//Centroid of the aruco
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4;
			}

            //Centroid of the target
			ug = (ug1 + ug2 + ug3 + ug4) / 4;
			ng = (ng1 + ng2 + ng3 + ng4) / 4;

            //Declaring the points required for visual servoing
			cv::Point p1 = cv::Point(ug1,ng1);
			cv::Point p2 = cv::Point(ug2,ng2);
			cv::Point p3 = cv::Point(ug3,ng3);
			cv::Point p4 = cv::Point(ug4,ng4);

            //Changing Image coordinates system from the left-top to the center.
			/*
											+y	^
												|
												|
												|
			      					-x <--------|---------> +x
					      						|
			      								|
			      								|
			      								|
			      								-y

			*/
			p1x = p1.x-frame.size().width/2;
			p1y = -(p1.y-frame.size().height/2);

			p2x = p2.x-frame.size().width/2;
			p2y = -(p2.y-frame.size().height/2);

			p3x = p3.x-frame.size().width/2;
			p3y = -(p3.y-frame.size().height/2);

			p4x = p4.x-frame.size().width/2;
			p4y = -(p4.y-frame.size().height/2);

      		//Camera frame point data (u,n,focal_length)
      		p1_vs_cf << p1x*pixel_size, p1y*pixel_size, focal_length;
			p2_vs_cf << p2x*pixel_size, p2y*pixel_size, focal_length;
			p3_vs_cf << p3x*pixel_size, p3y*pixel_size, focal_length;
			p4_vs_cf << p4x*pixel_size, p4y*pixel_size, focal_length;

            //Camera frame to Virtual frame conversion		
			beta_p1 = focal_length/(e3.transpose()*rotate_vector_by_quaternion_passive(p1_vs_cf, quaternion_x_y.conjugate()));
			beta_p2 = focal_length/(e3.transpose()*rotate_vector_by_quaternion_passive(p2_vs_cf, quaternion_x_y.conjugate()));
			beta_p3 = focal_length/(e3.transpose()*rotate_vector_by_quaternion_passive(p3_vs_cf, quaternion_x_y.conjugate()));
			beta_p4 = focal_length/(e3.transpose()*rotate_vector_by_quaternion_passive(p4_vs_cf, quaternion_x_y.conjugate()));
			
			// beta_p1 = focal_length/(e3.transpose()*Rtp(roll,pitch)*p1_vs_cf);
			// beta_p2 = focal_length/(e3.transpose()*Rtp(roll,pitch)*p2_vs_cf);
			// beta_p3 = focal_length/(e3.transpose()*Rtp(roll,pitch)*p3_vs_cf);
			// beta_p4 = focal_length/(e3.transpose()*Rtp(roll,pitch)*p4_vs_cf);

			p1_vs_vf = beta_p1*rotate_vector_by_quaternion_passive(p1_vs_cf, quaternion_x_y.conjugate());
			p2_vs_vf = beta_p2*rotate_vector_by_quaternion_passive(p2_vs_cf, quaternion_x_y.conjugate());
			p3_vs_vf = beta_p3*rotate_vector_by_quaternion_passive(p3_vs_cf, quaternion_x_y.conjugate());
			p4_vs_vf = beta_p4*rotate_vector_by_quaternion_passive(p4_vs_cf, quaternion_x_y.conjugate());
      		// p1_vs_vf = beta_p1*Rtp(roll,pitch)*p1_vs_cf;
			// p2_vs_vf = beta_p2*Rtp(roll,pitch)*p2_vs_cf;
			// p3_vs_vf = beta_p3*Rtp(roll,pitch)*p3_vs_cf;
			// p4_vs_vf = beta_p4*Rtp(roll,pitch)*p4_vs_cf;

			/*
			p1_vs_vf = Ryaw(uav_att(2)).transpose()*Ryaw(uav_att(2)).transpose()*p1_vs_vf;
			p2_vs_vf = Ryaw(uav_att(2)).transpose()*Ryaw(uav_att(2)).transpose()*p2_vs_vf;
			p3_vs_vf = Ryaw(uav_att(2)).transpose()*Ryaw(uav_att(2)).transpose()*p3_vs_vf;
			p4_vs_vf = Ryaw(uav_att(2)).transpose()*Ryaw(uav_att(2)).transpose()*p4_vs_vf;
			*/

      		//Ordinary moments. Centroid
			ug_vs = (p1_vs_vf(0) + p2_vs_vf(0) + p3_vs_vf(0) + p4_vs_vf(0)) / 4;
			ng_vs = (p1_vs_vf(1) + p2_vs_vf(1) + p3_vs_vf(1) + p4_vs_vf(1)) / 4;
			std::cout << "Points in virtual frame: " << std::endl;
			std::cout << p1_vs_vf << std::endl;
			std::cout << p2_vs_vf << std::endl;
			std::cout << p3_vs_vf << std::endl;
			std::cout << p4_vs_vf << std::endl;

			//Momentos centrados
			mu20 = powf((p1_vs_vf(0) - ug_vs), 2) +  powf((p2_vs_vf(0) - ug_vs), 2) +  powf((p3_vs_vf(0) - ug_vs), 2) +  powf((p4_vs_vf(0) - ug_vs), 2);
			mu02 = powf((p1_vs_vf(1) - ng_vs), 2) +  powf((p2_vs_vf(1) - ng_vs), 2) +  powf((p3_vs_vf(1) - ng_vs), 2) +  powf((p4_vs_vf(1) - ng_vs), 2);
			mu11 = ((p1_vs_vf(0) - ug_vs) * (p1_vs_vf(1) - ng_vs)) + ((p2_vs_vf(0) - ug_vs) * (p2_vs_vf(1) - ng_vs)) + ((p3_vs_vf(0) - ug_vs) * (p3_vs_vf(1) - ng_vs)) + ((p4_vs_vf(0) - ug_vs) * (p4_vs_vf(1) - ng_vs));
			den = mu20-mu02;

      		//Image features vector
			a = mu20 + mu02;
			qz = sqrt(aD/a);
			qx = qz * ng_vs/focal_length;
			qy = qz * ug_vs/focal_length;
			qpsi = -0.5 * atan(2*mu11/den);
			std::cout << std::endl;
			std::cout << "Analyzing data: " << std::endl;
			std::cout << ug_vs << std::endl;
			std::cout << ng_vs << std::endl;
			q.w() = den / (sqrt(4 * pow(mu11, 2.0) + pow(den, 2.0)));
			q.x() = 0.0;
			q.y() = 0.0;
			q.z() = -(2 * mu11) / (sqrt(4.0 * powf(mu11, 2.0) + powf(den, 2.0))); // check that we change the sign to imitate what Armando did on the psi calculation with the atan.
			// Transform the quaternion to Euler-angles and divide the yaw angle by 4 
			// (because it needs to match the calculation of the feature with atan)
			Eigen::Vector3f euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
		
			q = Eigen::AngleAxisf(euler[0], Eigen::Vector3f::UnitX()) * 
				Eigen::AngleAxisf(euler[1], Eigen::Vector3f::UnitY()) *
				Eigen::AngleAxisf(euler[2] / 4.0, Eigen::Vector3f::UnitZ());

			std::cout << "I see I need a yaw angle of: " << std::endl;
			std::cout << euler[2] / 4.0 * (180/3.14) << std::endl;

			// Get the angle-axis representation of yaw from the quaternion.
			// Check first if there can be a singularity, meaning, 
			// the vector part of the quaternion are all 0s.
			if (q.w() >= 1.0) {
				quaternion_angle_axis(0) = 0.0;
				quaternion_angle_axis(1) = 0.0;
				quaternion_angle_axis(2) = 0.0;
			}
			else {
				quaternion_angle_axis(0) = 2.0 * ((q.x() / sqrt(powf(q.x(), 2.0) + powf(q.y(), 2.0) + powf(q.z(), 2.0))) * acos(q.w()));
				quaternion_angle_axis(1) = 2.0 * ((q.y() / sqrt(powf(q.x(), 2.0) + powf(q.y(), 2.0) + powf(q.z(), 2.0))) * acos(q.w()));
				quaternion_angle_axis(2) = 2.0 * ((q.z() / sqrt(powf(q.x(), 2.0) + powf(q.y(), 2.0) + powf(q.z(), 2.0))) * acos(q.w()));
			}

			// std::cout << quaternion_angle_axis(2) * (180/3.14) << std::endl;
			// std::cout << std::endl;

			/////////////////////////////////////////////////////////////////
			/////////////////// IMAGE PROCESSING SECTION ////////////////////
			/////////////////////////////////////////////////////////////////
			// Indicators
			cv::circle(frame, cv::Point(ug, ng), 8, cv::Scalar(255, 0, 255));
			cv::circle(frame, cv::Point(ug1, ng1), 8, cv::Scalar(255, 0, 0)); // Upper-right
			cv::circle(frame, cv::Point(ug2, ng2), 8, cv::Scalar(0, 255, 0)); // Upper-left
			cv::circle(frame, cv::Point(ug3, ng3), 8, cv::Scalar(0, 0, 255)); // Lower-left
			cv::circle(frame, cv::Point(ug4, ng4), 8, cv::Scalar(255, 255, 0)); // Lower-right
			// Define the starting and ending points of the line
			cv::Point start_point(ug, ng);
			
			// Define the angle of the line in degrees
			double angle_degrees = quaternion_angle_axis(2) * (180/3.14);

			// Calculate the endpoint of the line
			int line_length = 25;
			cv::Point end_point_1, end_point_2, end_point_3, end_point_4;
			end_point_1.x = start_point.x + line_length * cos(angle_degrees * CV_PI / 180);
			end_point_1.y = start_point.y + line_length * sin(angle_degrees * CV_PI / 180);
			end_point_2.x = start_point.x - line_length * cos(angle_degrees * CV_PI / 180);
			end_point_2.y = start_point.y - line_length * sin(angle_degrees * CV_PI / 180);
			end_point_3.x = start_point.x + line_length * sin(angle_degrees * CV_PI / 180);
			end_point_3.y = start_point.y - line_length * cos(angle_degrees * CV_PI / 180);
			end_point_4.x = start_point.x - line_length * sin(angle_degrees * CV_PI / 180);
			end_point_4.y = start_point.y + line_length * cos(angle_degrees * CV_PI / 180);

			// Draw the line on the image
			cv::line(frame, start_point, end_point_1, cv::Scalar(255, 0, 255), 1);
			cv::line(frame, start_point, end_point_2, cv::Scalar(255, 0, 255), 1);
			cv::line(frame, start_point, end_point_3, cv::Scalar(255, 0, 255), 1);
			cv::line(frame, start_point, end_point_4, cv::Scalar(255, 0, 255), 1);

			// Get the dimensions of the image
			int image_width = frame.cols;
			int image_height = frame.rows;

			// Draw a dashed line along the vertical middle
			int vertical_middle = image_width / 2;
			for (int y = 0; y < image_height; y += 10) {
				cv::line(frame, cv::Point(vertical_middle, y), cv::Point(vertical_middle, y + 5), cv::Scalar(128, 128, 128), 1);
			}

			// Draw a dashed line along the horizontal middle
			int horizontal_middle = image_height / 2;
			for (int x = 0; x < image_width; x += 10) {
				cv::line(frame, cv::Point(x, horizontal_middle), cv::Point(x + 5, horizontal_middle), cv::Scalar(128, 128, 128), 1);
			}
			std::cout << "OK" << std::endl;
			/////////////////////////////////////////////////////////////////
			//////////////// END OF IMAGE PROCESSING SECTION ////////////////
			/////////////////////////////////////////////////////////////////

			//Publishing data via Rostopics
			im_feat_vec.x = qx;
			im_feat_vec.y = qy;
			im_feat_vec.z = qz;
			im_feat_vec.w = quaternion_angle_axis(2);

			a_val.data = a;

			punto1_vis.x = p1_vs_vf(0);
			punto1_vis.y = p1_vs_vf(1);
			punto2_vis.x = p2_vs_vf(0);
			punto2_vis.y = p2_vs_vf(1);
			punto3_vis.x = p3_vs_vf(0);
			punto3_vis.y = p3_vs_vf(1);
			punto4_vis.x = p4_vs_vf(0);
			punto4_vis.y = p4_vs_vf(1);
			centroid.x = ug_vs;
			centroid.y = ng_vs;

			im_feat_pub.publish(im_feat_vec);
			a_value_pub.publish(a_val);
			punto1_pub.publish(punto1_vis);
			punto2_pub.publish(punto2_vis);
			punto3_pub.publish(punto3_vis);
			punto4_pub.publish(punto4_vis);
			centroid_pub.publish(centroid);

			// std::cout << "p1 " << p1_vs_vf(0) << ", " << p1_vs_vf(1) << '\n';
			// std::cout << "p2 " << p2_vs_vf(0) << ", " << p2_vs_vf(1) << '\n';
			// std::cout << "p3 " << p3_vs_vf(0) << ", " << p3_vs_vf(1) << '\n';
			// std::cout << "p4 " << p4_vs_vf(0) << ", " << p4_vs_vf(1) << '\n';
			// std::cout << "mu11 " << mu11 << '\n';

        }

		msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
		image_pub.publish(msg);
		
        ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}
