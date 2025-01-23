#ifndef COPERNICUS_BASE_COPERNICUS_HARDWARE_H
#define COPERNICUS_BASE_COPERNICUS_HARDWARE_H

#include <ros/ros.h>

#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include <string>
#include "sensor_msgs/JointState.h"

#include <copernicus_base/constants.h>

#include "std_msgs/Int16.h"
#include <copernicus_msgs/RPM.h>

namespace copernicus_base {
class CopernicusHardware: public hardware_interface::RobotHW {
public:
    CopernicusHardware(ros::NodeHandle nh, ros::NodeHandle private_nh);
	
	void rpmCallback(const copernicus_msgs::RPM::ConstPtr& rpmTemp);
		
	
    void update_encoder_readings_to_joints();
    void send_velocity_to_motors_from_joints();
    void register_controllers();
private:
	int16_t	subMotorRPMRight,subMotorRPMLeft , pubMotorRPMRight,pubMotorRPMLeft;
    double subMotorRotationsRight,subMotorRotationsLeft;
	
	ros::Publisher rpm_pub ;
	ros::Subscriber rpm_sub ;
	
    void set_speeds(double left, double right);
    void limit_speeds(int16_t &left, int16_t &right);
    double convert_rpm_to_radians(double rpm);
    double convert_rotation_to_radians(double rotation);
    double convert_radians_to_rpm(double radians);
    
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    ros::NodeHandle nh_, private_nh_;


    struct Joint {
        double position;
        double position_offset;
        double velocity;
        double effort;
        double velocity_command;

        Joint() :
            position(0),velocity(0),effort(0),velocity_command(0)
            {
            }
    } joints_[4];
};
}

#endif

