#include <errno.h>
#include <stdbool.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

//#include <soc/mt3620_gpios.h>

#include <applibs/log.h>
#include <applibs/gpio.h>
#include <applibs/adc.h>

//// ADC connection
#include <sys/time.h>
#include <sys/socket.h>
#include <applibs/application.h>

#include "hw/solar_tracker_hardware.h"
#include "servo.h"
#include "epoll_timerfd_utilities.h"

// Support functions.
static void TerminationHandler(int signalNumber);
static int InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);

// File descriptors - initialized to invalid value
int epollFd = -1;
int adcControllerFd = -1;
int sensorSelectAfd = -1;
int sensorSelectBfd = -1;

// The maximum voltage
static float maxVoltage = 2.5f;

//// ADC connection
static void UpdateLightLevels(void);
static void TimerEventHandler(EventData* eventData);
void recalculateServoAngles();
static int timerFd = -1;

int verticalServoAngle = VERTICAL_SERVO_RESTING_ANGLE;
int horizontalServoAngle = HORIZONTAL_SERVO_RESTING_ANGLE;
int lightLevels[4];	
struct _SERVO_State* verticalServo;
struct _SERVO_State* horizontalServo;

// event handler data structures. Only the event handler field needs to be populated.
static EventData timerEventData = { .eventHandler = &TimerEventHandler };
//// end ADC connection

// Termination state
volatile sig_atomic_t terminationRequired = false;

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
	// Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
	terminationRequired = true;
}

/// <summary>
///     
/// </summary>
static void TimerEventHandler(EventData* eventData)
{
	if (ConsumeTimerFdEvent(timerFd) != 0) {
		terminationRequired = true;
		return;
	}

	UpdateLightLevels();
}

/// <summary>
///     
/// </summary>
static void UpdateLightLevels()
{
	for (int channel = 0; channel < SENSOR_NUM; channel++)
	{
		GPIO_SetValue(sensorSelectAfd, (channel & 1) > 0);
		GPIO_SetValue(sensorSelectBfd, (channel & 2) > 0);

		uint32_t value;
		int result = ADC_Poll(adcControllerFd, PHOTO_SENSOR_CHANNEL, &value);
		if (result < -1) {
			Log_Debug("ADC_Poll failed with error: %s (%d)\n", strerror(errno), errno);
			terminationRequired = true;
			return;
		}

		lightLevels[channel] = value;
	}

	recalculateServoAngles();
}

/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals, and set up event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{
	struct sigaction action;
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_handler = TerminationHandler;
	sigaction(SIGTERM, &action, NULL);

	epollFd = CreateEpollFd();
	if (epollFd < 0) {
		Log_Debug("Error creating epollFd.\n");
		return -1;
	}

	adcControllerFd = ADC_Open(ADC_CONTROLLER);
	if (adcControllerFd < 0) {
		Log_Debug("ADC_Open failed with error: %s (%d)\n", strerror(errno), errno);
		return -1;
	}

	Log_Debug("fd: %d, open: %d, channel: %d\n", adcControllerFd, ADC_CONTROLLER, PHOTO_SENSOR_CHANNEL);

	int sampleBitCount = ADC_GetSampleBitCount(adcControllerFd, PHOTO_SENSOR_CHANNEL);
	if (sampleBitCount == -1) {
		Log_Debug("ADC_GetSampleBitCount failed with error : %s (%d)\n", strerror(errno), errno);
		return -1;
	}
	if (sampleBitCount == 0) {
		Log_Debug("ADC_GetSampleBitCount returned sample size of 0 bits.\n");
		return -1;
	}

	int result = ADC_SetReferenceVoltage(adcControllerFd, PHOTO_SENSOR_CHANNEL, maxVoltage);
	if (result < 0) {
		Log_Debug("ADC_SetReferenceVoltage failed with error : %s (%d)\n", strerror(errno), errno);
		return -1;
	}

	//// ADC connection

	static const struct timespec sendPeriod = { .tv_sec = 0,.tv_nsec = 25000000 };
	timerFd = CreateTimerFdAndAddToEpoll(epollFd, &sendPeriod, &timerEventData, EPOLLIN);
	if (timerFd < 0)
	{
		return -1;
	}

	//// end ADC Connection

	return 0;
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
	Log_Debug("Closing file descriptors.\n");
	CloseFdAndPrintError(epollFd, "Epoll");
	CloseFdAndPrintError(adcControllerFd, "ADC");
}

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

void recalculateServoAngles(void)
{
	int tl = lightLevels[SENSOR_TL_SELECT];
	int tr = lightLevels[SENSOR_TR_SELECT];
	int bl = lightLevels[SENSOR_BL_SELECT];
	int br = lightLevels[SENSOR_BR_SELECT];

	Log_Debug("%d, %d, %d, %d\n", tl, tr, bl, br);

	int tol = 50;

	int avt = (tl + tr) / 2; // average value top
	int avd = (bl + br) / 2; // average value down
	int avl = (tl + bl) / 2; // average value left
	int avr = (tr + br) / 2; // average value right

	int dvert = avt - avd; // check the difference of up and down
	int dhoriz = avl - avr;// check the difference of left and right

	if (-1 * tol > dvert || dvert > tol) // check if the difference is in the tolerance else change vertical angle
	{
		if (avt < avd)
		{
			verticalServoAngle++;
			if (verticalServoAngle > VERTICAL_SERVO_MAX_ANGLE)
			{
				verticalServoAngle = VERTICAL_SERVO_MAX_ANGLE;
			}
		}
		else if (avt > avd)
		{
			verticalServoAngle--;
			if (verticalServoAngle < VERTICAL_SERVO_MIN_ANGLE)
			{
				verticalServoAngle = VERTICAL_SERVO_MIN_ANGLE;
			}
		}
		SERVO_SetAngle(verticalServo, verticalServoAngle);
	}

	if (-1 * tol > dhoriz || dhoriz > tol) // check if the difference is in the tolerance else change horizontal angle
	{
		if (avl > avr)
		{
			horizontalServoAngle = --horizontalServoAngle;
			if (horizontalServoAngle < HORIZONTAL_SERVO_MIN_ANGLE)
			{
				horizontalServoAngle = HORIZONTAL_SERVO_MIN_ANGLE;
			}
		}
		else if (avl < avr)
		{
			horizontalServoAngle = ++horizontalServoAngle;
			if (horizontalServoAngle > HORIZONTAL_SERVO_MAX_ANGLE)
			{
				horizontalServoAngle = HORIZONTAL_SERVO_MAX_ANGLE;
			}
		}
		SERVO_SetAngle(horizontalServo, horizontalServoAngle);
	}
}

int main(int argc, char* argv[])
{
	Log_Debug("Solar Tracker Application starting.\n");

	sensorSelectAfd = GPIO_OpenAsOutput(SENSOR_SELECT_A_GPIO, GPIO_OutputMode_PushPull, GPIO_Value_High);
	sensorSelectBfd = GPIO_OpenAsOutput(SENSOR_SELECT_B_GPIO, GPIO_OutputMode_PushPull, GPIO_Value_High);
	// Select C pin of the 4051 multiplexer is hardwired to GND
	if (sensorSelectAfd < 0 || sensorSelectBfd < 0) {
		Log_Debug("Error opening GPIO: %s (%d). Check that app_manifest.json includes the GPIO used.\n", strerror(errno), errno);
		return -1;
	}

	initServo(VERTICAL_SERVO_GPIO, &verticalServo, VERTICAL_SERVO_MIN_ANGLE, VERTICAL_SERVO_MAX_ANGLE);
	initServo(HORIZONTAL_SERVO_GPIO, &horizontalServo, HORIZONTAL_SERVO_MIN_ANGLE, HORIZONTAL_SERVO_MAX_ANGLE);

	SERVO_SetAngle(verticalServo, verticalServoAngle);
	SERVO_SetAngle(horizontalServo, horizontalServoAngle);

	if (InitPeripheralsAndHandlers() != 0) {
		terminationRequired = true;
	}

	// Use epoll to wait for events and trigger handlers, until an error or SIGTERM happens
	while (!terminationRequired) {
		if (WaitForEventAndCallHandler(epollFd) != 0) {
			terminationRequired = true;
		}
	}

	ClosePeripheralsAndHandlers();
	Log_Debug("Application exiting.\n");
	return 0;
}