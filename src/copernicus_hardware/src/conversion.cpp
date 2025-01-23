/*
 * conversion.c
 *
 *  Created on: 19-May-2020
 *      Author: james
 */


#include "copernicus_base/conversion.h"
#include "stdint.h"
#include "string.h"

void float2Bytes(uint8_t *bytes_temp,float float_variable){
	union {
		float a;
		uint8_t bytes[4];
	} thing;
	thing.a = float_variable;
	memcpy(bytes_temp, thing.bytes, 4);
	//*((float *)bytes_temp) = float_variable;
//	bytes_temp[0] = (uint32_t) float_variable &  0x000000ff;
//	bytes_temp[1] =  ((uint32_t)float_variable >> 8)& 0x000000ff;
//	bytes_temp[2] =  ((uint32_t)float_variable >> 16)&  0x000000ff;
//	bytes_temp[3] =  ((uint32_t)float_variable >> 24)&  0x000000ff;
}
void int2Bytes(uint8_t *bytes_temp,int int_variable){
	union {
		int a;
		unsigned char bytes[4];
	} thing;
	thing.a = int_variable;
	memcpy(bytes_temp, thing.bytes, 4);
}
void unsignedint2Bytes(uint8_t *bytes_temp,uint32_t int_variable){
	union {
		uint32_t a;
		unsigned char bytes[4];
	} thing;
	thing.a = int_variable;
	memcpy(bytes_temp, thing.bytes, 4);
}
void short2Bytes(uint8_t *bytes_temp,int16_t variable){

	bytes_temp[0] = (variable&0xff00)>>8;
	bytes_temp[1] = (variable&0x00ff);
}
void unsignedshort2Bytes(uint8_t *bytes_temp,uint16_t variable){

	bytes_temp[0] = (variable&0xff00)>>8;
	bytes_temp[1] = (variable&0x00ff);
}
void bytes2Float(uint8_t *bytes_temp,float *float_variable)
{
	//		float * temp = (float *) bytes_temp;
	//		*float_variable = *temp;
	union {
		float a;
		unsigned char bytes[4];
	} thing;
	memcpy(thing.bytes,bytes_temp, 4);
	*float_variable = thing.a;
}
void bytes2int(uint8_t *bytes_temp,int *variable)
{

	*variable = 0;
	int temp = bytes_temp[0]<<8*3;
	*variable |= temp;
	temp = bytes_temp[1]<<8*2;
	*variable |= temp;
	temp = bytes_temp[2]<<8*1;
	*variable |= temp;
	temp = bytes_temp[3]<<8;
	*variable |= temp;
}
void bytes2unsignedint(uint8_t *bytes_temp,uint32_t *variable)
{

	*variable = 0;
	uint32_t temp = bytes_temp[0]<<8*3;
	*variable |= temp;
	temp = bytes_temp[1]<<8*2;
	*variable |= temp;
	temp = bytes_temp[2]<<8*1;
	*variable |= temp;
	temp = bytes_temp[3]<<8;
	*variable |= temp;
}
void bytes2short(uint8_t *bytes_temp,int16_t *variable)
{
		*variable=0;
		*variable = bytes_temp[0];
		*variable = *variable<<8;
		*variable |=  bytes_temp[1];
}
void bytes2unsignedshort(uint8_t *bytes_temp,uint16_t *variable)
{

	*variable = bytes_temp[0];
	*variable = *variable<<8;
	*variable |=  bytes_temp[1];
}
