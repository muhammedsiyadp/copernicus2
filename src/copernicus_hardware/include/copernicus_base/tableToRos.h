
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/UInt8.h"
#include "constants.h"
#include "copernicus_msgs/Diagnostics.h"
#include "copernicus_msgs/RPM.h"
#include "copernicus_msgs/BMS.h"
#include "copernicus_base/conversion.h"

extern ros::Publisher rpm_pub;
extern ros::Subscriber rpm_sub;
extern ros::Subscriber estop_sub;
extern ros::Subscriber diagEn_sub;

extern copernicus_msgs::Diagnostics table_msg;
extern copernicus_msgs::BMS   BMS_msg;

extern ros::Publisher table_pub;
extern ros::Subscriber  publishTable;

extern ros::Publisher   bms_pub;


void rpmCallback(const copernicus_msgs::RPM::ConstPtr& rpmTemp);
void estopCallback(const std_msgs::Bool& e_stop_msg);
void diagEnCallback(const std_msgs::Bool& diagEn_msg);
void rosTopicInit(void);
void copernicus_update_RPM(float left, float right);
void publish_table(void);
void publish_bms(void);
