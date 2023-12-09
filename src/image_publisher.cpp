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
float aD = 0.0000007970;

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

	quaternion_x_y = multiplyQuaternionTimesQuaternion(quaternion_x, quaternion_y);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;

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

	// Create an ImageTransport instance
    image_transport::ImageTransport it(nh);

    //ROS publishers and subscribers
	ros::Publisher im_feat_pub = nh.advertise<geometry_msgs::Quaternion>("ImFeat_vector",100);
    ros::Publisher a_value_pub = nh.advertise<std_msgs::Float64>("a_value",100);
    ros::Publisher punto1_pub = nh.advertise<geometry_msgs::Pose2D>("point_one",100);
	ros::Publisher punto2_pub = nh.advertise<geometry_msgs::Pose2D>("point_two",100);
	ros::Publisher punto3_pub = nh.advertise<geometry_msgs::Pose2D>("point_three",100);
	ros::Publisher punto4_pub = nh.advertise<geometry_msgs::Pose2D>("point_four",100);
	ros::Publisher centroid_pub = nh.advertise<geometry_msgs::Pose2D>("centroid",100);
    image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);

    //Declaring local variables
    double fps;
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
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);

    // Specify the publishing rate (adjust as needed)
    ros::Rate loop_rate(80);  // 30 Hz

    // Variables for FPS calculation
    int frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();

    while (ros::ok()) {


        // Initializing the detector parameters using default values
		cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
		// Declaring the 2D vectors that contain the aruco's corners and rejected candidates
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectCandidates;
		// Declaring a vector to save de ID numbers of the detected arucos
		std::vector<int> markerIds;

        // Get frame
        cv::Mat frame;
        cap >> frame;
        
        // Check if the frame is empty
        if (frame.empty()) {
            ROS_WARN("Empty frame received from the camera!");
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

            continue;
        }

        // Rotate image this will depend on how the camera is PHYSICALLY installed
        cv::rotate(frame, frame, cv::ROTATE_90_CLOCKWISE);

        //Detect the markers in the image
		cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectCandidates);

        /////////////////////////////////////////////////////////////////////
        /////////////////////// FPS CALCULATION /////////////////////////////
        ////////////////////////////////////////////////////////////////////
        // Increment frame count
        frame_count++;

        // Calculate FPS every second
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();

        if (elapsed_time >= 1) {
            fps = frame_count / static_cast<double>(elapsed_time);

            // Print FPS to the console
            ROS_INFO("FPS: %.2f", fps);

            // Reset variables for the next second
            start_time = end_time;
            frame_count = 0;
        }

        ////////////////////////////////////////////////////////////////////
        //////////////////// END OF FPS CALCULATION ////////////////////////
        ////////////////////////////////////////////////////////////////////

		// Convert the OpenCV image to a ROS image message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

        // Publish the image
        pub.publish(msg);

        // Sleep to control the publishing rate
        loop_rate.sleep();
    }

    // Release the camera
    cap.release();

    return 0;
}