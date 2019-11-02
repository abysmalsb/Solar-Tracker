#ifndef SOLAR_TRACKER_CONFIG_H
#define SOLAR_TRACKER_CONFIG_H

#include "mt3620_rdb.h"

/// <summary>Analog channel select A</summary>
#define SENSOR_SELECT_A_GPIO				MT3620_GPIO34
/// <summary>Analog channel select B. Channel C is hardwired to GND</summary>
#define SENSOR_SELECT_B_GPIO				MT3620_GPIO16
/// <summary>Vertical servo pin</summary>
#define VERTICAL_SERVO_GPIO					MT3620_GPIO0
/// <summary>Horizontal servo pin</summary>
#define HORIZONTAL_SERVO_GPIO				MT3620_GPIO2
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
#define VERTICAL_SERVO_RESTING_ANGLE		0
/// <summary>Horizontal resting angle</summary>
#define HORIZONTAL_SERVO_RESTING_ANGLE		90
/// <summary>Vertical min angle</summary>
#define VERTICAL_SERVO_MIN_ANGLE			0
/// <summary>Horizontal min angle</summary>
#define HORIZONTAL_SERVO_MIN_ANGLE			0
/// <summary>Vertical max angle</summary>
#define VERTICAL_SERVO_MAX_ANGLE			140
/// <summary>Horizontal max angle</summary>
#define HORIZONTAL_SERVO_MAX_ANGLE			180

#endif