#include <errno.h>
#include <stdbool.h>
#include <signal.h>
#include <string.h>

#include <applibs/log.h>
#include <applibs/gpio.h>
#include <applibs/adc.h>
#include <applibs/i2c.h>
#include <applibs/pwm.h>
#include <soc/mt3620_i2cs.h>

#include "hw/solar_tracker_hardware.h"
#include "epoll_timerfd_utilities.h"
#include "rgb-lcd.h"
#include "servo.h"

// Support functions.
static void TerminationHandler(int signalNumber);
static int InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);

// File descriptors - initialized to invalid value
int epollFd = -1;
int adcControllerFd = -1;
int sensorSelectAfd = -1;
int sensorSelectBfd = -1;
int pwmControllerFd = -1;

// The maximum voltage
static float maxVoltage = 2.5f;

int displayColumns = 16;
int displayRows = 2;

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

// Your servos might have slightly different duty cycles so you might want to edit the 
// config values.
static const unsigned int periodNs = 20000000;
static const unsigned int maxDutyCycleNs = 2400000;
static const unsigned int minDutyCycleNs = 600000;
static const unsigned int minAngle = 0;
static const unsigned int maxAngle = 180;

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

void initServo(int pwmFd, unsigned int channel, struct _SERVO_State** servo, int minAngle, int maxAngle)
{
	struct SERVO_Config servoConfig;

	servoConfig.pwmFd = pwmFd;
	servoConfig.pwmChannel = channel;
	servoConfig.minAngleDeg = minAngle;
	servoConfig.maxAngleDeg = maxAngle;
	servoConfig.minPulseNs = minDutyCycleNs;
	servoConfig.maxPulseNs = maxDutyCycleNs;
	servoConfig.periodNs = periodNs;

	if (SERVO_Init(&servoConfig, servo) < 0)
	{
		Log_Debug("Error initializing servo 0\n");
		return -1;
	}

	return 0;
}

void printInitError(char* errorMessage)
{
	printLine(0, "E:Failed to init");
	printLine(1, errorMessage);
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

	//// LCD display

	int i2cFd = I2CMaster_Open(MT3620_RDB_HEADER4_ISU2_I2C);

	if (i2cFd < 0) {
		Log_Debug("ERROR: I2CMaster_Open: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	int result = I2CMaster_SetBusSpeed(i2cFd, I2C_BUS_SPEED_STANDARD);
	if (result != 0) {
		Log_Debug("ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	result = I2CMaster_SetTimeout(i2cFd, 100);
	if (result != 0) {
		Log_Debug("ERROR: I2CMaster_SetTimeout: errno=%d (%s)\n", errno, strerror(errno));
		return -1;
	}

	// set up the LCD's number of columns, rows and i2c file descriptor:
	begin(displayColumns, displayRows, i2cFd);
	// set background color for the display
	setRGB(0, 255, 255);
	// Print a message to the LCD.
	printLine(0, "Starting...");

	//// LCD display is initialized

	//// Init sensor channel select GPIO-s

	sensorSelectAfd = GPIO_OpenAsOutput(SENSOR_SELECT_A_GPIO, GPIO_OutputMode_PushPull, GPIO_Value_High);
	sensorSelectBfd = GPIO_OpenAsOutput(SENSOR_SELECT_B_GPIO, GPIO_OutputMode_PushPull, GPIO_Value_High);
	// Select C pin of the 4051 multiplexer is hardwired to GND
	if (sensorSelectAfd < 0 || sensorSelectBfd < 0) {
		printInitError("ADC ch select");
		Log_Debug("Error opening GPIO: %s (%d). Check that app_manifest.json includes the GPIO used.\n", strerror(errno), errno);
		return -1;
	}

	//// Sensor channel select GPIO-s are initialized

	//// ADC connection

	adcControllerFd = ADC_Open(ADC_CONTROLLER);
	if (adcControllerFd < 0) {
		printInitError("ADC controller");
		Log_Debug("ADC_Open failed with error: %s (%d)\n", strerror(errno), errno);
		return -1;
	}

	result = ADC_SetReferenceVoltage(adcControllerFd, PHOTO_SENSOR_CHANNEL, maxVoltage);
	if (result < 0) {
		printInitError("ADC ref voltage");
		Log_Debug("ADC_SetReferenceVoltage failed with error : %s (%d)\n", strerror(errno), errno);
		return -1;
	}

	static const struct timespec sendPeriod = { .tv_sec = 0,.tv_nsec = 25000000 };
	timerFd = CreateTimerFdAndAddToEpoll(epollFd, &sendPeriod, &timerEventData, EPOLLIN);
	if (timerFd < 0)
	{
		return -1;
	}

	//// ADC Connection is initialized

	//// Servos

	pwmControllerFd = PWM_Open(PWM_CONTROLLER);
	if (pwmControllerFd == -1) {
		Log_Debug(
			"Error opening PWM_CONTROLLER: %s (%d). Check that app_manifest.json "
			"includes the PWM used.\n",
			strerror(errno), errno);
		return -1;
	}

	initServo(pwmControllerFd, VERTICAL_SERVO_PWM_CHANNEL, &verticalServo, VERTICAL_SERVO_MIN_ANGLE, VERTICAL_SERVO_MAX_ANGLE);
	initServo(pwmControllerFd, HORIZONTAL_SERVO_PWM_CHANNEL, &horizontalServo, HORIZONTAL_SERVO_MIN_ANGLE, HORIZONTAL_SERVO_MAX_ANGLE);

	SERVO_SetAngle(verticalServo, verticalServoAngle);
	SERVO_SetAngle(horizontalServo, horizontalServoAngle);

	//// Servos are initialized

	return 0;
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
	SERVO_Destroy(verticalServo);
	SERVO_Destroy(horizontalServo);

	Log_Debug("Closing file descriptors.\n");
	CloseFdAndPrintError(epollFd, "Epoll");
	CloseFdAndPrintError(adcControllerFd, "ADC");
	CloseFdAndPrintError(pwmControllerFd, "PwmFd");
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