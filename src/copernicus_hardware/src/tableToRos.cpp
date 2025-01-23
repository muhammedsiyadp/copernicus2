#include <ros/ros.h>
#include "copernicus_base/tableToRos.h"
#include "copernicus_base/conversion.h"
#include <iostream>
#include <fstream>
#include "copernicus_base/copernicusDataStruct.h"


uint8_t rpmData=false;
uint8_t estopData=false;
uint8_t estopStatus=false;
uint8_t diagEnData=false;
uint8_t diagEnStatus= false;

extern float subMotorRPMRight;
extern float subMotorRPMLeft;
extern uint8_t rpmAvailable;

ros::Publisher rpm_pub;
ros::Subscriber rpm_sub;
ros::Subscriber estop_sub;
ros::Subscriber diagEn_sub;
ros::Publisher table_pub;
ros::Subscriber   publishTable;
copernicus_msgs::Diagnostics table_msg;
copernicus_msgs::BMS   BMS_msg;
ros::Publisher   bms_pub;


void rpmCallback(const copernicus_msgs::RPM::ConstPtr& rpmTemp) {
	subMotorRPMLeft= rpmTemp->left;
	subMotorRPMRight= rpmTemp->right;
	rpmAvailable = true;

}
void estopCallback(const std_msgs::Bool &e_stop_msg)
{	
	ROS_INFO("Estop called");
	estopData=true;
	estopStatus = e_stop_msg.data;
	copernicus_update_table(PRIORITY_DIAG,DIAG_ROS_ESTOP_STATE,&estopStatus,1);
}
void diagEnCallback(const std_msgs::Bool &diagEn_msg)
{
	
	if(diagEn_msg.data == true)
	{
		diagEnStatus = 1;
ROS_INFO("Diagnostics enabled");
	}
	else
	{
		diagEnStatus = 0;
	}
	copernicus_update_table(PRIORITY_DIAG,DIAG_EN,&diagEnStatus,1);
	//ROS_INFO("diag_status.en =%x",diag_status.en);


}


void rosTopicInit(void)
{
	ros::NodeHandle nh;

	rpm_sub 		= nh.subscribe("RPM_PUB",100,&rpmCallback);

	rpm_pub 		= nh.advertise<copernicus_msgs::RPM>("RPM_SUB", 100);

	estop_sub 		= nh.subscribe("e_stop_sw_enable",100,&estopCallback);

	diagEn_sub		= nh.subscribe("diag_enable",100,&diagEnCallback);

	table_pub 		= nh.advertise<copernicus_msgs::Diagnostics>("copernicus_diagnostics", 100);

	bms_pub			= nh.advertise<copernicus_msgs :: BMS>("bms",100);
}

void copernicus_update_RPM(float left,float right)
{
	copernicus_msgs::RPM rpm;
	rpm.left = left;
	rpm.right = right;
	rpm_pub.publish(rpm);
}

void publish_table(void)
{

	table_msg.LOWBATTERY_FLAG	= diag_status.low_battery_flag;
	table_msg.CHARGING_FLAG 	= diag_status.charging_flag;;
	table_msg.MOTOR_ALARM_FLAG	= diag_status.motor_alarm_flag;
	table_msg.BMS_ALARM_FLAG	= diag_status.bms_alarm_flag;
	table_msg.HIGH_TEMP_FLAG	= diag_status.high_temp_flag;
	table_msg.COMM_ERROR_FLAG	= diag_status.comm_error_flag;

	table_msg.BMS_SOC		= diag_status.bms_soc;	
	table_msg.BMS_SOH		= diag_status.bms_soh;	
	table_msg.BMS_Voltage		= diag_status.bms_voltage;
	table_msg.BMS_Current		= diag_status.bms_current;
	table_msg.BMS_voltageDiff	= diag_status.bms_voltage_diff;
	table_msg.BMS_tempDiff		= diag_status.bms_tempDiff;
	table_msg.BMS_BCUMode		= diag_status.bms_bcumode;
	table_msg.BMS_Alarm		= diag_status.bms_alarm;

	table_msg.M_State[0]		= diag_status.M_State[0];
	table_msg.M_State[1]		= diag_status.M_State[1];

	table_msg.M_Alarm[0]		= diag_status.M_Alarm[0];
	table_msg.M_Alarm[1]		= diag_status.M_Alarm[1];

 	table_msg.M_Power[0]		= diag_status.M_Power[0];
	table_msg.M_Power[1]		= diag_status.M_Power[1];

	table_msg.M_Voltage[0]		= diag_status.M_Voltage[0];
	table_msg.M_Voltage[1]		= diag_status.M_Voltage[1];

	table_msg.M_Current[0]		= diag_status.M_Current[0];
	table_msg.M_Current[1]		= diag_status.M_Current[1];

	table_msg.M_Comm[0]		= diag_status.M_Comm[0];
	table_msg.M_Comm[1]		= diag_status.M_Comm[1];

	table_msg.Relay_Temp 		= diag_status.Relay_Temp;
	table_msg.PowerConn_Temp	= diag_status.PowerConn_Temp;
	table_msg.PreCharge_Temp 	= diag_status.PreCharge_Temp;
	table_msg.Power_Supply_Temp	= diag_status.Power_Supply_Temp;
	table_msg.Battery_In_Volt	= diag_status.Battery_In_Volt;
	table_msg.PRE_CHARGE_Volt 	= diag_status.PRE_CHARGE_Volt;
	table_msg.HW_ESTOP_Volt		= diag_status.HW_ESTOP_Volt;
	table_msg.SW_ESTOP_Volt 	= diag_status.SW_ESTOP_Volt;

	table_msg.Motor_Alarm_Code[0] 	= diag_status.Motor_Alarm_Code[0];
	table_msg.Motor_Alarm_Code[1] 	= diag_status.Motor_Alarm_Code[1];

	table_msg.Motor_Fun_Fail[0]	= diag_status.Motor_Fun_Fail[0];
	table_msg.Motor_Fun_Fail[1]	= diag_status.Motor_Fun_Fail[1];

	table_msg.M_INIT[0]		= diag_status.M_INIT[0];
	table_msg.M_INIT[1]		= diag_status.M_INIT[1];

	table_msg.M_CW[0] 		= diag_status.M_CW[0];
	table_msg.M_CW[1] 		= diag_status.M_CW[1];

	table_msg.M_CCW[0] 		= diag_status.M_CCW[0];
	table_msg.M_CCW[1] 		= diag_status.M_CCW[1];

	table_msg.M_STOP[0] 		= diag_status.M_STOP[0];
	table_msg.M_STOP[1] 		= diag_status.M_STOP[1];

	table_msg.M_BREAK[0] 		= diag_status.M_BREAK[0];
	table_msg.M_BREAK[1] 		= diag_status.M_BREAK[1];

	table_msg.M_SETRPM[0] 		= diag_status.M_SETRPM[0];
	table_msg.M_SETRPM[1] 		= diag_status.M_SETRPM[1];


	table_msg.M_GETRPM[0] 		= diag_status.M_GETRPM[0];
	table_msg.M_GETRPM[1] 		= diag_status.M_GETRPM[1];

	table_msg.M_PARAM[0]		= diag_status.M_PARAM[0];
	table_msg.M_ALARM[1]		= diag_status.M_ALARM[1];

	table_msg.PRECHARGE_FUSE =     diag_status.PRECHARGE_FUSE;
	table_msg.MOTOR_FUSE =     diag_status.MOTOR_FUSE;

	table_msg.LED_STATE =     diag_status.LED_STATE;


	table_pub.publish(table_msg);


}
void publish_bms(void)
{

	BMS_msg.SOC = bms_status.soc ;
	BMS_msg.Current= bms_status.current;
	BMS_msg.Voltage= bms_status.voltage;
	bms_pub.publish(BMS_msg);
}
