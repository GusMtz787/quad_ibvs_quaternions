/*
Provided to you by Emlid Ltd (c) 2015.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Control servos connected to PWM driver onboard of Navio2 shield for Raspberry Pi.

Connect servo to Navio2's rc output and watch it work.
PWM_OUTPUT = 0 complies to channel number 1, 1 to channel number 2 and so on.
To use full range of your servo correct SERVO_MIN and SERVO_MAX according to it's specification.

To run this example navigate to the directory containing it and run following commands:
make
sudo ./Servo
*/

#include <ros/ros.h>
#include <unistd.h>
#include "Navio2/PWM.h"
#include "Navio+/RCOutput_Navio.h"
#include "Navio2/RCOutput_Navio2.h"
#include "Common/Util.h"
#include <memory>

#define SERVO_MIN 1250 /*mS*/
#define SERVO_MAX 1750 /*mS*/

using namespace Navio;

class ESCControlNode {
public:
    ESCControlNode(int pwmOutput) : pwmOutput_(pwmOutput) {
        pwm = get_rcout();

        if (check_apm()) {
            ROS_ERROR("APM not initialized!");
            ros::shutdown();
        }

        if (getuid()) {
            ROS_ERROR("Not root. Please launch like this: sudo rosrun your_package_name your_node_name");
            ros::shutdown();
        }

        if (!(pwm->initialize(pwmOutput_))) {
            ROS_ERROR("Failed to initialize PWM for output %d!", pwmOutput_);
            ros::shutdown();
        }

        pwm->set_frequency(pwmOutput_, 50);

        if (!(pwm->enable(pwmOutput_))) {
            ROS_ERROR("Failed to enable PWM for output %d!", pwmOutput_);
            ros::shutdown();
        }

        controlESC();
    }

private:
    std::unique_ptr<RCOutput> pwm;
    int pwmOutput_;

    void controlESC() {
        while (ros::ok()) {
            pwm->set_duty_cycle(pwmOutput_, SERVO_MIN);
            sleep(1);
            pwm->set_duty_cycle(pwmOutput_, SERVO_MAX);
            sleep(1);
            ros::spinOnce();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "esc_control_node");

    int pwmOutput = 0;  // Default PWM output
    if (argc > 1) {
        pwmOutput = std::stoi(argv[1]);
    }

    ESCControlNode escControlNode(pwmOutput);
    ros::spin();
    return 0;
}
