#ifndef INC_DATASTRUCT_H
#define INC_DATASTRUCT_H
#include "stdint.h"
#include <stdbool.h>

typedef struct
{
	float left;
	float right;
} rpm_status_typedef;



typedef struct
{
    bool  low_battery_flag;
    bool  charging_flag;
    bool  motor_alarm_flag;
    bool  bms_alarm_flag;
    bool  high_temp_flag;
    bool  comm_error_flag;
    uint16_t  bms_soc;
    uint16_t  bms_soh;
    uint16_t  bms_voltage;
    int16_t   bms_current;
    uint16_t  bms_voltage_diff;
    uint16_t  bms_tempDiff;
    uint16_t  bms_bcumode;
    uint16_t  bms_alarm;
    uint16_t M_State[2];
    uint16_t M_Alarm[2];
    uint16_t M_Power[2];
    uint16_t M_Voltage[2];
    uint16_t M_Current[2];
    uint8_t M_Comm[2];
    float  Relay_Temp;
    float  PowerConn_Temp;
    float  PreCharge_Temp;
    float  Power_Supply_Temp;
    float  Battery_In_Volt;
    float  PRE_CHARGE_Volt;
    float  HW_ESTOP_Volt;
    float  SW_ESTOP_Volt;
    uint16_t Motor_Alarm_Code[2];
    uint32_t Motor_Fun_Fail[2];
    uint16_t M_INIT[2];
    uint16_t M_CW[2];
    uint16_t M_CCW[2];
    uint16_t M_STOP[2];
    uint16_t M_BREAK[2];
    uint16_t M_SETRPM[2];
    uint16_t M_GETRPM[2];
    uint16_t M_PARAM[2];
    uint16_t M_ALARM[2];
    uint8_t  PRECHARGE_FUSE;
    uint8_t  MOTOR_FUSE;
    uint8_t  LED_STATE;
} diag_status_typedef;

typedef struct
{
	uint16_t soc;
	int16_t current;
	uint16_t voltage;
} bms_status_typedef;


extern rpm_status_typedef rpm_status;
extern bms_status_typedef bms_status;
extern diag_status_typedef diag_status;


#define PRIORITY_RPM 				0x01
#define PRIORITY_RPM_FB				0x02
#define PRIORITY_BMS	 			0x03
#define PRIORITY_DIAG	 			0x04

#define BMS_SOC1					0x01
#define BMS_CURRENT					0x02
#define BMS_VOLTAGE					0x03

#define RPM_LEFT					0x01
#define RPM_RIGHT					0x02

#define DIAG_LOW_BATT_FLAG			0x01
#define DIAG_CHARGING_FLAG			0x02
#define DIAG_MOTOR_ALARM_FLAG		0x03
#define DIAG_BMS_ALARM_FLAG			0x04
#define DIAG_HIGH_TEMP_FLAG			0x05
#define DIAG_COMM_ERROR_FLAG		0x06
#define DIAG_BMS_SOC				0x07
#define DIAG_BMS_SOH				0x08
#define DIAG_BMS_VOLT				0x09
#define DIAG_BMS_CURRENT			0x0A
#define DIAG_BMS_VOLT_DIFF			0x0B
#define DIAG_BMS_TEMP_DIFF			0x0C
#define DIAG_BMS_BCU_MODE			0x0D
#define DIAG_BMS_ALARM				0x0E
#define DIAG_M_STATE				0x0F
#define DIAG_M_ALARM1				0x10
#define DIAG_M_POWER				0x11
#define DIAG_M_VOLATGE				0x12
#define DIAG_M_CURRENT				0x13
#define DIAG_M_COMM					0x14
#define DIAG_RELAY_TEMP				0x15
#define DIAG_POWER_CONN_TEMP		0x16
#define DIAG_PRE_CHARGE_TEMP		0x17
#define DIAG_POWER_SUPPLY_TEMP		0x18
#define DIAG_BATTERY_IN_VOLT		0x19
#define DIAG_PRE_CHRGE_VOLT			0x1A
#define DIAG_HW_ESTOP_VOLT			0x1B
#define DIAG_SW_ESTOP_VOLT			0x1C
#define DIAG_MOTOR_ALRAM_CODE		0x1D
#define DIAG_MOTOR_FUN_FAIL			0x1E
#define DIAG_M_INIT					0x1F
#define DIAG_M_CW					0x20
#define DIAG_M_CCW					0x21
#define DIAG_M_STOP					0x22
#define DIAG_M_BREAK				0x23
#define DIAG_M_SETRPM				0x24
#define DIAG_M_GETRPM				0x25
#define DIAG_M_PARAM				0x26
#define DIAG_M_ALARM2				0x27
#define DIAG_PRECHARGE_FUSE			0x28
#define DIAG_MOTOR_FUSE				0x29
#define DIAG_LED_STATE				0x2A
#define DIAG_EN					0x2B
#define DIAG_ROS_ESTOP_STATE			0x2C

void copernicus_default(void);
void copernicus_update_table(uint8_t table_id,uint8_t msg_id,uint8_t aData[],uint8_t size);
void copernicus_data_callBack(uint8_t *data_in);
void rpmDataHandler(uint8_t * data);
void rpmfbDataHandler(uint8_t *data);
void bmsDataHandler(uint8_t * data);
void diagDataHandler(uint8_t * data);
void speedControlHandler(uint8_t *data);
#endif
