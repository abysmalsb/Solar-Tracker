#include <errno.h>
#include <stdbool.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <soc/mt3620_gpios.h>

#include <applibs/log.h>
#include <applibs/gpio.h>

#include "Servo.h"
#include "config.h"

void initServo(GPIO_Id gpio, struct _SERVO_State** servo, int minAngle, int maxAngle)
{
	struct SERVO_Config servoConfig;

	servoConfig.gpio = gpio;
	servoConfig.minAngle = minAngle;
	servoConfig.maxAngle = maxAngle;
	servoConfig.minPulse = 600000;
	servoConfig.maxPulse = 2400000;
	servoConfig.period = 20000000;

	if (SERVO_Init(&servoConfig, servo) < 0)
	{
		Log_Debug("Error initializing servo 0\n");
		return -1;
	}
}

int main(void)
{
	Log_Debug("Solar Tracker Application starting.\n");
    // This minimal Azure Sphere app repeatedly toggles GPIO 9, which is the green channel of RGB
    // LED 1 on the MT3620 RDB.
    // Use this app to test that device and SDK installation succeeded that you can build,
    // deploy, and debug an app with Visual Studio, and that you can deploy an app over the air,
    // per the instructions here: https://docs.microsoft.com/azure-sphere/quickstarts/qs-overview
    //
    // It is NOT recommended to use this as a starting point for developing apps; instead use
    // the extensible samples here: https://github.com/Azure/azure-sphere-samples
    Log_Debug(
        "\nVisit https://github.com/Azure/azure-sphere-samples for extensible samples to use as a "
        "starting point for full applications.\n");

	/*int fds[3];
	fds[0] = GPIO_OpenAsOutput(VERTICAL_SERVO_GPIO, GPIO_OutputMode_PushPull, GPIO_Value_High);
	fds[1] = GPIO_OpenAsOutput(HORIYONTAL_SERVO_GPIO, GPIO_OutputMode_PushPull, GPIO_Value_High);
	for (int i = 0; i < 2; i++) {
		if (fds[i] < 0) {
			Log_Debug(
				"Error opening GPIO: %s (%d). Check that app_manifest.json includes the GPIO used.\n",
				strerror(errno), errno);
			return -1;
		}
	}

    const struct timespec sleepTime = {1, 0};
	int counter = 0;
    while (true) {
		GPIO_SetValue(fds[0], (counter & 1) > 0);
		GPIO_SetValue(fds[1], (counter & 2) > 0);
        nanosleep(&sleepTime, NULL);
		counter = counter < 3 ? counter + 1 : 0;
		//GPIO_SetValue(fd, GPIO_Value_Low);
		//nanosleep(&sleepTime, NULL);
		//GPIO_SetValue(fd, GPIO_Value_High);
		//nanosleep(&sleepTime, NULL);
    }*/

	struct _SERVO_State* verticalServo;
	initServo(VERTICAL_SERVO_GPIO, &verticalServo, VERTICAL_SERVO_MIN_ANGLE, VERTICAL_SERVO_MAX_ANGLE);
	struct _SERVO_State* horizontalServo;
	initServo(HORIZONTAL_SERVO_GPIO, &horizontalServo, HORIZONTAL_SERVO_MIN_ANGLE, HORIZONTAL_SERVO_MAX_ANGLE);

	SERVO_SetAngle(verticalServo, VERTICAL_SERVO_RESTING_ANGLE);
	SERVO_SetAngle(horizontalServo, HORIZONTAL_SERVO_RESTING_ANGLE);

	const struct timespec sleepTime = { 1, 0 };
	nanosleep(&sleepTime, NULL);

	while (1) 
	{

	}
}