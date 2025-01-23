/*
 * dataStruct.c
 *
 *  Created on: 19-May-2020
 *      Author: james
 */

#include <ros/ros.h>
#include <string.h>
#include "copernicus_base/copernicusDataStruct.h"
#include "copernicus_base/conversion.h"
#include "stdint.h"
#include "copernicus_base/queue.h"
#include "copernicus_base/tableToRos.h"

rpm_status_typedef rpm_status = {};
rpm_status_typedef rpm_fb_status = {};
bms_status_typedef bms_status= {};
diag_status_typedef diag_status= {};


void copernicus_default(void)
{
	rpm_status.left = 0.0;
	rpm_status.right = 0.0;

}
void copernicus_update_table(uint8_t table_id,uint8_t msg_id,uint8_t aData[],uint8_t size)
{
	uint8_t msgTemp[50];

	msgTemp[0] = table_id;
	msgTemp[1] = 0x00;
	msgTemp[2] = size+1;
	msgTemp[3] = msg_id;
	for(int i = 0 ; i < size ; i++)
	{
		msgTemp[4+i] = aData[i];
	}
	copernicus_data_callBack(msgTemp);
	queue_insert(msgTemp,1);
}
void copernicus_data_callBack(uint8_t* data_in)
{

	switch(data_in[0])
	{
	case PRIORITY_RPM:
		rpmDataHandler(data_in);
		break;
	case PRIORITY_RPM_FB:
		rpmfbDataHandler(data_in);
		copernicus_update_RPM(rpm_fb_status.left,rpm_fb_status.right);
		break;

	case PRIORITY_BMS:
		bmsDataHandler(data_in);
		publish_bms();
		break;
	case PRIORITY_DIAG:
		diagDataHandler(data_in);
		publish_table();
		break;
	}
}

void rpmDataHandler(uint8_t *data)
{
	bytes2Float((data+4),&rpm_status.left);
	bytes2Float((data+8),&rpm_status.right);	

}

void rpmfbDataHandler(uint8_t *data)
{
	bytes2Float((data+4),&rpm_fb_status.left);
	bytes2Float((data+8),&rpm_fb_status.right);	

}


void bmsDataHandler(uint8_t *data)
{
	switch(data[3]){

	case BMS_SOC1:
		bytes2unsignedshort((data+4),&bms_status.soc);
		break;

	case BMS_CURRENT:
		bytes2short((data+4),&bms_status.current);
		break;

	case BMS_VOLTAGE:
		bytes2unsignedshort((data+4),&bms_status.voltage);
		break;

	}
}


void diagDataHandler(uint8_t *data)
{
	switch(data[3]){

	case DIAG_LOW_BATT_FLAG://1
		diag_status.low_battery_flag = (data[4]!=0);
		break;

	case DIAG_CHARGING_FLAG://2
		diag_status.charging_flag = (data[4]!=0);
		break;

	case DIAG_MOTOR_ALARM_FLAG://3
		diag_status.motor_alarm_flag = (data[4]!=0);
		break;

	case DIAG_BMS_ALARM_FLAG://4
		diag_status.bms_alarm_flag = (data[4]!=0);
		break;

	case DIAG_HIGH_TEMP_FLAG://5
		diag_status.high_temp_flag = (data[4]!=0);
		break;

	case DIAG_COMM_ERROR_FLAG://6
		diag_status.comm_error_flag = (data[4]!=0);
		break;

	case DIAG_BMS_SOC://7
		bytes2unsignedshort(data+4,&diag_status.bms_soc);
		break;

	case DIAG_BMS_SOH://8
		bytes2unsignedshort(data+4,&diag_status.bms_soh);
		break;

	case DIAG_BMS_VOLT://9
		bytes2unsignedshort(data+4,&diag_status.bms_voltage);
		break;

	case DIAG_BMS_CURRENT://10
		bytes2short(data+4,&diag_status.bms_current);
		break;

	case DIAG_BMS_VOLT_DIFF://11
		bytes2unsignedshort(data+4,&diag_status.bms_voltage_diff);
		break;

	case DIAG_BMS_TEMP_DIFF://12
		//diag_status.bms_tempDiff=data[4];
		//ROS_INFO("tempdiff %d",data[4]);
		bytes2unsignedshort(data+4,&diag_status.bms_tempDiff);
		break;

	case DIAG_BMS_BCU_MODE://13
		//diag_status.bms_bcumode=data[4];
		//ROS_INFO("bcu %d",data[4]);
		bytes2unsignedshort(data+4,&diag_status.bms_bcumode);
		break;

	case DIAG_BMS_ALARM://14
		bytes2unsignedshort(data+4,&diag_status.bms_alarm);
		break;

	case DIAG_M_STATE://15
		bytes2unsignedshort(data+4,&diag_status.M_State[0]);
		bytes2unsignedshort(data+6,&diag_status.M_State[1]);
		break;

	case DIAG_M_ALARM1://16
		bytes2unsignedshort(data+4,&diag_status.M_Alarm[0]);
		bytes2unsignedshort(data+6,&diag_status.M_Alarm[1]);
		break;

	case DIAG_M_POWER ://17
		bytes2unsignedshort(data+4,&diag_status.M_Power[0]);
		bytes2unsignedshort(data+6,&diag_status.M_Power[1]);
		break;

	case DIAG_M_VOLATGE ://18
		bytes2unsignedshort(data+4,&diag_status.M_Voltage[0]);
		bytes2unsignedshort(data+6,&diag_status.M_Voltage[1]);
		break;

	case DIAG_M_CURRENT ://19
		bytes2unsignedshort(data+4,&diag_status.M_Current[0]);
		bytes2unsignedshort(data+6,&diag_status.M_Current[1]);
		break;

	case DIAG_M_COMM ://20
		diag_status.M_Comm[0] = data[4];
		diag_status.M_Comm[1] = data[5];
		break;

	case DIAG_RELAY_TEMP ://21
		bytes2Float(data+4,&diag_status.Relay_Temp);
		break;

	case DIAG_POWER_CONN_TEMP ://22
		bytes2Float(data+4,&diag_status.PowerConn_Temp);
		break;

	case DIAG_PRE_CHARGE_TEMP ://23
		bytes2Float(data+4,&diag_status.PreCharge_Temp);
		break;

	case DIAG_POWER_SUPPLY_TEMP ://24
		bytes2Float(data+4,&diag_status.Power_Supply_Temp);
		break;

	case DIAG_BATTERY_IN_VOLT ://25
		bytes2Float(data+4,&diag_status.Battery_In_Volt);
		break;

	case DIAG_PRE_CHRGE_VOLT ://26
		bytes2Float(data+4,&diag_status.PRE_CHARGE_Volt);
		break;

	case DIAG_HW_ESTOP_VOLT ://27
		bytes2Float(data+4,&diag_status.HW_ESTOP_Volt);
		break;

	case DIAG_SW_ESTOP_VOLT ://28
		bytes2Float(data+4,&diag_status.SW_ESTOP_Volt);
		break;

	case DIAG_MOTOR_ALRAM_CODE ://29
		bytes2unsignedshort(data+4,&diag_status.Motor_Alarm_Code[0]);
		bytes2unsignedshort(data+6,&diag_status.Motor_Alarm_Code[1]);
		break;
	case DIAG_MOTOR_FUN_FAIL ://30
		diag_status.Motor_Fun_Fail[0] = data[4];
		diag_status.Motor_Fun_Fail[1] = data[5];
		break;

	case DIAG_M_INIT ://31
		bytes2unsignedshort(data+4,&diag_status.M_INIT[0]);
		bytes2unsignedshort(data+6,&diag_status.M_INIT[1]);
		break;

	case DIAG_M_CW ://32
		bytes2unsignedshort(data+4,&diag_status.M_CW[0]);
		bytes2unsignedshort(data+6,&diag_status.M_CW[1]);
		break;

	case DIAG_M_CCW ://33
		bytes2unsignedshort(data+4,&diag_status.M_CCW[0]);
		bytes2unsignedshort(data+6,&diag_status.M_CCW[1]);
		break;
	case DIAG_M_STOP ://34
		bytes2unsignedshort(data+4,&diag_status.M_STOP[0]);
		bytes2unsignedshort(data+6,&diag_status.M_STOP[1]);
		break;
	case DIAG_M_BREAK ://35
		bytes2unsignedshort(data+4,&diag_status.M_BREAK[0]);
		bytes2unsignedshort(data+6,&diag_status.M_BREAK[1]);
		break;
	case DIAG_M_SETRPM://36
		bytes2unsignedshort(data+4,&diag_status.M_SETRPM[0]);
		bytes2unsignedshort(data+6,&diag_status.M_SETRPM[1]);
		break;
	case DIAG_M_GETRPM ://37
		bytes2unsignedshort(data+4,&diag_status.M_GETRPM[0]);
		bytes2unsignedshort(data+6,&diag_status.M_GETRPM[1]);
		break;
	case DIAG_M_PARAM ://38
		bytes2unsignedshort(data+4,&diag_status.M_PARAM[0]);
		bytes2unsignedshort(data+6,&diag_status.M_PARAM[1]);
		break;
	case DIAG_M_ALARM2 ://39
		bytes2unsignedshort(data+4,&diag_status.M_ALARM[0]);
		bytes2unsignedshort(data+6,&diag_status.M_ALARM[1]);
		break;
	case DIAG_PRECHARGE_FUSE ://40
		diag_status.PRECHARGE_FUSE=data[4];
		break;
	case DIAG_MOTOR_FUSE ://41
		diag_status.MOTOR_FUSE=data[4];
		break;
	case DIAG_LED_STATE ://42
		diag_status.LED_STATE=data[4];
		break;
	}

}
