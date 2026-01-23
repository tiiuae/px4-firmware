#pragma once
#include "health_types.hpp"
#include <array>

enum Okbits : HealthMask {
	OK_BIT_00	= 1ull << 0,
	OK_BIT_01	= 1ull << 1,
	OK_BIT_02	= 1ull << 2,
	OK_BIT_03	= 1ull << 3,
	OK_BIT_04	= 1ull << 4,
	OK_BIT_05	= 1ull << 5,
	OK_BIT_06	= 1ull << 6,
	OK_BIT_07	= 1ull << 7,
	OK_BIT_08	= 1ull << 8,
	OK_BIT_09	= 1ull << 9,
	OK_BIT_10	= 1ull << 10,
	OK_BIT_11	= 1ull << 11,
	OK_BIT_12	= 1ull << 12,
	OK_BIT_13	= 1ull << 13,
	OK_BIT_14	= 1ull << 14,
	OK_BIT_15	= 1ull << 15,
	OK_BIT_16	= 1ull << 16,
	OK_BIT_17	= 1ull << 17,
	OK_BIT_18	= 1ull << 18,
	OK_BIT_19	= 1ull << 19,
	OK_BIT_20	= 1ull << 20,
	OK_BIT_21	= 1ull << 21,
	OK_BIT_22	= 1ull << 22,
	OK_BIT_23	= 1ull << 23,
	OK_BIT_24	= 1ull << 24,
	OK_BIT_25	= 1ull << 25,
	OK_BIT_26	= 1ull << 26,
	OK_BIT_27	= 1ull << 27,
	OK_BIT_28	= 1ull << 28,
	OK_BIT_29	= 1ull << 29,
	OK_BIT_30	= 1ull << 30,
	OK_BIT_31	= 1ull << 31,
	OK_BIT_32	= 1ull << 32,
	OK_BIT_33	= 1ull << 33,
	OK_BIT_34	= 1ull << 34,
	OK_BIT_35	= 1ull << 35,
	OK_BIT_36	= 1ull << 36,
	OK_BIT_37	= 1ull << 37,
	OK_BIT_38	= 1ull << 38,
	OK_BIT_39	= 1ull << 39,
	OK_BIT_40	= 1ull << 40,
	OK_BIT_41	= 1ull << 41,
	OK_BIT_42	= 1ull << 42,
	OK_BIT_43	= 1ull << 43,
	OK_BIT_44	= 1ull << 44,
	OK_BIT_45	= 1ull << 45,
	OK_BIT_46	= 1ull << 46,
	OK_BIT_47	= 1ull << 47,
	OK_BIT_48	= 1ull << 48,
	OK_BIT_49	= 1ull << 49,
	OK_BIT_50	= 1ull << 50,
	OK_BIT_51	= 1ull << 51,
	OK_BIT_52	= 1ull << 52,
	OK_BIT_53	= 1ull << 53,
	OK_BIT_54	= 1ull << 54,
	OK_BIT_55	= 1ull << 55,
	OK_BIT_56	= 1ull << 56,
	OK_BIT_57	= 1ull << 57,
	OK_BIT_58	= 1ull << 58,
	OK_BIT_59	= 1ull << 59,
	OK_BIT_60	= 1ull << 60,
	OK_BIT_61	= 1ull << 61,
	OK_BIT_62	= 1ull << 62,
};

enum Faultbits : HealthMask {
	FAULT_BIT_00	= 1ull << 0,
	FAULT_BIT_01	= 1ull << 1,
	FAULT_BIT_02	= 1ull << 2,
	FAULT_BIT_03	= 1ull << 3,
	FAULT_BIT_04	= 1ull << 4,
	FAULT_BIT_05	= 1ull << 5,
	FAULT_BIT_06	= 1ull << 6,
	FAULT_BIT_07	= 1ull << 7,
	FAULT_BIT_08	= 1ull << 8,
	FAULT_BIT_09	= 1ull << 9,
	FAULT_BIT_10	= 1ull << 10,
	FAULT_BIT_11	= 1ull << 11,
	FAULT_BIT_12	= 1ull << 12,
	FAULT_BIT_13	= 1ull << 13,
	FAULT_BIT_14	= 1ull << 14,
	FAULT_BIT_15	= 1ull << 15,
	FAULT_BIT_16	= 1ull << 16,
	FAULT_BIT_17	= 1ull << 17,
	FAULT_BIT_18	= 1ull << 18,
	FAULT_BIT_19	= 1ull << 19,
	FAULT_BIT_20	= 1ull << 20,
	FAULT_BIT_21	= 1ull << 21,
	FAULT_BIT_22	= 1ull << 22,
	FAULT_BIT_23	= 1ull << 23,
	FAULT_BIT_24	= 1ull << 24,
	FAULT_BIT_25	= 1ull << 25,
	FAULT_BIT_26	= 1ull << 26,
	FAULT_BIT_27	= 1ull << 27,
	FAULT_BIT_28	= 1ull << 28,
	FAULT_BIT_29	= 1ull << 29,
	FAULT_BIT_30	= 1ull << 30,
	FAULT_BIT_31	= 1ull << 31,
	FAULT_BIT_32	= 1ull << 32,
	FAULT_BIT_33	= 1ull << 33,
	FAULT_BIT_34	= 1ull << 34,
	FAULT_BIT_35	= 1ull << 35,
	FAULT_BIT_36	= 1ull << 36,
	FAULT_BIT_37	= 1ull << 37,
	FAULT_BIT_38	= 1ull << 38,
	FAULT_BIT_39	= 1ull << 39,
	FAULT_BIT_40	= 1ull << 40,
	FAULT_BIT_41	= 1ull << 41,
	FAULT_BIT_42	= 1ull << 42,
	FAULT_BIT_43	= 1ull << 43,
	FAULT_BIT_44	= 1ull << 44,
	FAULT_BIT_45	= 1ull << 45,
	FAULT_BIT_46	= 1ull << 46,
	FAULT_BIT_47	= 1ull << 47,
	FAULT_BIT_48	= 1ull << 48,
	FAULT_BIT_49	= 1ull << 49,
	FAULT_BIT_50	= 1ull << 50,
	FAULT_BIT_51	= 1ull << 51,
	FAULT_BIT_52	= 1ull << 52,
	FAULT_BIT_53	= 1ull << 53,
	FAULT_BIT_54	= 1ull << 54,
	FAULT_BIT_55	= 1ull << 55,
	FAULT_BIT_56	= 1ull << 56,
	FAULT_BIT_57	= 1ull << 57,
	FAULT_BIT_58	= 1ull << 58,
	FAULT_BIT_59	= 1ull << 59,
	FAULT_BIT_60	= 1ull << 60,
	FAULT_BIT_61	= 1ull << 61,
	FAULT_BIT_62	= 1ull << 62,
};


struct HealthStatusFlags
{
	HealthMask S_OK;
	HealthMask S_WARN;
	HealthMask S_CRITICAL;
	HealthMask S_MARGIN_LOW;
	HealthMask S_NOT_SYNC;
};

//####################################################################
// DUMMY CASE
//####################################################################
static HealthStatusFlags HealthBitsDummyCase
{
	.S_OK 					= OK_BIT_00,
	.S_WARN					= FAULT_BIT_00,
	.S_CRITICAL				= FAULT_BIT_01,
	.S_MARGIN_LOW				= FAULT_BIT_02,
	.S_NOT_SYNC				= FAULT_BIT_03,
};

enum HealthBitsDummyCase : HealthMask{
	S_DUMMY_OK 					= OK_BIT_00,
	S_DUMMY_WARN					= FAULT_BIT_00,
	S_DUMMY_CRITICAL				= FAULT_BIT_01,
	S_DUMMY_MARGIN_LOW				= FAULT_BIT_02,
	S_DUMMY_NOT_SYNC				= FAULT_BIT_03,
};
//####################################################################
// NULL VALUES IN CONTROLLER CASE
//####################################################################
static HealthStatusFlags HealthBitsNullValuesControllerCase
{
	.S_OK 					= OK_BIT_01,
	.S_WARN					= FAULT_BIT_04,
	.S_CRITICAL				= FAULT_BIT_05,
	.S_MARGIN_LOW				= FAULT_BIT_06,
	.S_NOT_SYNC				= FAULT_BIT_07,
};

enum HealthBitsNullValuesController : HealthMask{
	S_NULL_CTL_OK 					= OK_BIT_01,
	S_NULL_CTL_WARN					= FAULT_BIT_04,
	S_NULL_CTL_CRITICAL				= FAULT_BIT_05,
	S_NULL_CTL_MARGIN_LOW				= FAULT_BIT_06,
	S_NULL_CTL_NOT_SYNC				= FAULT_BIT_07,
};
//####################################################################


std::array<HealthStatusFlags,2> HealthFlagsCases
{
	HealthBitsDummyCase,
	HealthBitsNullValuesControllerCase
};

// static HealthStatusFlags HealthBitsPx4Modules
// {

// };

enum HealthBitsHW : HealthMask {
	// Core sensors
	H_IMU_OK             	= OK_BIT_53,
	H_GPS_OK             	= OK_BIT_54,
	H_BARO_OK            	= OK_BIT_55,

	// Estimation
	H_ATT_EST_OK         	= OK_BIT_56,
	H_POS_EST_OK         	= OK_BIT_57,

	// Power
	H_BATTERY_OK         	= OK_BIT_58,

	// Actuation
	H_MOTORS_OK          	= OK_BIT_59,

	// Environment / links
	H_RC_OK              	= OK_BIT_60,
	H_DATA_LINK_OK       	= OK_BIT_61,

	// Hard safety
  	H_GEOFENCE_OK        	= OK_BIT_62,

	// Power
	H_BATTERY_CRITICAL   	= FAULT_BIT_60,

	// GPS
	H_GPS_CRITICAL		= FAULT_BIT_61,



};
