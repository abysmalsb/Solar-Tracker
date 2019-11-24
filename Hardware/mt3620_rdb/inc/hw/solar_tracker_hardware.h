#ifndef SOLAR_TRACKER_CONFIG_H
#define SOLAR_TRACKER_CONFIG_H

#include "mt3620_rdb.h"

/// <summary>Analog channel select A</summary>
#define SENSOR_SELECT_A_GPIO				MT3620_GPIO34
/// <summary>Analog channel select B. Channel C is hardwired to GND</summary>
#define SENSOR_SELECT_B_GPIO				MT3620_GPIO16
/// <summary>PWM Controller</summary>
#define PWM_CONTROLLER						MT3620_RDB_PWM_CONTROLLER0
/// <summary>Vertical servo pin</summary>
#define VERTICAL_SERVO_PWM_CHANNEL			MT3620_PWM_CHANNEL0
/// <summary>Horizontal servo pin</summary>
#define HORIZONTAL_SERVO_PWM_CHANNEL		MT3620_PWM_CHANNEL2
/// <summary>ADC Controller</summary>
#define ADC_CONTROLLER						MT3620_ADC_CONTROLLER0
/// <summary>Photo sensor ADC channel</summary>
#define PHOTO_SENSOR_CHANNEL				MT3620_ADC_CHANNEL1
/// <summary>Solar panel voltage ADC channel</summary>
#define SOLAR_PANEL_CHANNEL					MT3620_ADC_CHANNEL2

/// <summary>Number of photosensors</summary>
#define SENSOR_NUM							4
/// <summary>Photosensor top left select value</summary>
#define SENSOR_TL_SELECT					0
/// <summary>Photosensor top right select value</summary>
#define SENSOR_TR_SELECT					1
/// <summary>Photosensor bottom left select value</summary>
#define SENSOR_BL_SELECT					2
/// <summary>Photosensor bottom right select value</summary>
#define SENSOR_BR_SELECT					3

/// <summary>Vertical resting angle</summary>
#define VERTICAL_SERVO_STANDBY_ANGLE		0
/// <summary>Horizontal resting angle</summary>
#define HORIZONTAL_SERVO_STANDBY_ANGLE		90
/// <summary>Vertical min angle</summary>
#define VERTICAL_SERVO_MIN_ANGLE			0
/// <summary>Horizontal min angle</summary>
#define HORIZONTAL_SERVO_MIN_ANGLE			0
/// <summary>Vertical max angle</summary>
#define VERTICAL_SERVO_MAX_ANGLE			140
/// <summary>Horizontal max angle</summary>
#define HORIZONTAL_SERVO_MAX_ANGLE			180
/// <summary>Servo converging speed</summary>
#define SERVO_CONVERGING_SPEED				500.0f
/// <summary>Light level where the tracker will enter standby mode</summary>
#define STANDBY_CUT_OFF_LIGHT_LEVEL			200
/// <summary>Waiting seconds in standby mode</summary>
#define STANDBY_UPDATE_SPEED				2
/// <summary>Waiting nanoseconds in working mode</summary>
#define WORKING_UPDATE_SPEED				25000000
/// <summary>Waiting nanoseconds in working mode</summary>
#define POWER_UPDATE_SPEED					500000000
/// <summary>Solar panel demo power consumer resistance (ohm)</summary>
#define RESISTANCE							4.7

#endif