/**************************************************************
 *
 * Servo control library for Azure Sphere SDK based devices.
 *
 * author: Balázs Simon
 *
 **************************************************************/

#ifndef SERVO_H
#define SERVO_H

struct _SERVO_State;

struct SERVO_Config
{
	int minAngleDeg;
	int maxAngleDeg;
	int pwmFd;
	unsigned int pwmChannel;
	unsigned int periodNs;
	unsigned int minPulseNs;
	unsigned int maxPulseNs;
};

extern int SERVO_Init(struct SERVO_Config* config, struct _SERVO_State** state);

extern int SERVO_SetAngle(struct _SERVO_State* servoState, float angle);

extern int SERVO_Destroy(struct _SERVO_State* servoState);

#endif
