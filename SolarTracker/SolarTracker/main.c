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

//// ADC connection
#include <sys/time.h>
#include <sys/socket.h>
#include <applibs/application.h>

#include "servo.h"
#include "config.h"
#include "epoll_timerfd_utilities.h"

// Support functions.
static void TerminationHandler(int signalNumber);
static int InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);

// File descriptors - initialized to invalid value
int epollFd = -1;
int sensorSelectAfd;
int sensorSelectBfd;

//// ADC connection
static const char rtAppComponentId[] = "005180bc-402f-4cb3-a662-72937dbcde47";
static int sockFd = -1;
static void RequestDataFromRTCoreADC(void);
static void TimerEventHandler(EventData* eventData);
static void SocketEventHandler(EventData* eventData);
void recalculateServoAngles();
static int timerFd = -1;
uint8_t RTCore_status;
int channel = 0;

int verticalServoAngle = VERTICAL_SERVO_RESTING_ANGLE;
int horizontalServoAngle = HORIZONTAL_SERVO_RESTING_ANGLE;
int lightLevels[4];	
struct _SERVO_State* verticalServo;
struct _SERVO_State* horizontalServo;

// event handler data structures. Only the event handler field needs to be populated.
static EventData timerEventData = { .eventHandler = &TimerEventHandler };
static EventData socketEventData = { .eventHandler = &SocketEventHandler };
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
///     Handle socket event by reading incoming data from real-time capable application.
/// </summary>
static void SocketEventHandler(EventData* eventData)
{
	// Read response from real-time capable application.
	char rxBuf[32];
	union Analog_data
	{
		uint32_t u32;
		uint8_t u8[4];
	} analog_data;

	int bytesReceived = recv(sockFd, rxBuf, sizeof(rxBuf), 0);

	if (bytesReceived == -1) {
		//Log_Debug("ERROR: Unable to receive message: %d (%s)\n", errno, strerror(errno));
		terminationRequired = true;
	}

	// Copy data from Rx buffer to analog_data union
	for (int i = 0; i < sizeof(analog_data); i++)
	{
		analog_data.u8[i] = rxBuf[i];
	}

	lightLevels[channel] = analog_data.u32;

	if (channel == 3)
	{
		channel = 0;
		recalculateServoAngles();
	}
	else 
	{
		channel++;
	}
}

/// <summary>
///     Handle send timer event by writing data to the real-time capable application.
/// </summary>
static void TimerEventHandler(EventData* eventData)
{
	if (ConsumeTimerFdEvent(timerFd) != 0) {
		terminationRequired = true;
		return;
	}

	RequestDataFromRTCoreADC();
}

/// <summary>
///     Helper function for TimerEventHandler sends message to real-time capable application.
/// </summary>
static void RequestDataFromRTCoreADC()
{
	GPIO_SetValue(sensorSelectAfd, (channel & 1) > 0);
	GPIO_SetValue(sensorSelectBfd, (channel & 2) > 0);

	static int iter = 0;

	// Send a message to real-time capable application.
	static char txMessage[32];
	sprintf(txMessage, "Read-ADC-%d", iter++);
	//Log_Debug("Sending: %s, Channel: %d\n", txMessage, channel);

	int bytesSent = send(sockFd, txMessage, strlen(txMessage), 0);
	if (bytesSent == -1)
	{
		//Log_Debug("ERROR: Unable to send message: %d (%s)\n", errno, strerror(errno));
		terminationRequired = true;
		return;
	}
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

	//// ADC connection

	// Open connection to real-time capable application.
	sockFd = Application_Socket(rtAppComponentId);
	if (sockFd == -1)
	{
		//Log_Debug("ERROR: Unable to create socket: %d (%s)\n", errno, strerror(errno));
		Log_Debug("Real Time Core disabled or Component Id is not correct.\n");
		Log_Debug("The program will continue without showing light sensor data.\n");
		// Communication with RT core error
		RTCore_status = 1;
		//return -1;
	}
	else
	{
		// Communication with RT core success
		RTCore_status = 0;
		// Set timeout, to handle case where real-time capable application does not respond.
		static const struct timeval recvTimeout = { .tv_sec = 5,.tv_usec = 0 };
		int result = setsockopt(sockFd, SOL_SOCKET, SO_RCVTIMEO, &recvTimeout, sizeof(recvTimeout));
		if (result == -1)
		{
			//Log_Debug("ERROR: Unable to set socket timeout: %d (%s)\n", errno, strerror(errno));
			return -1;
		}

		// Register handler for incoming messages from real-time capable application.
		if (RegisterEventHandlerToEpoll(epollFd, sockFd, &socketEventData, EPOLLIN) != 0)
		{
			return -1;
		}

		// Register 0.05 second timer to send a message to the real-time core.
		static const struct timespec sendPeriod = { .tv_sec = 0,.tv_nsec = 5000000 };
		timerFd = CreateTimerFdAndAddToEpoll(epollFd, &sendPeriod, &timerEventData, EPOLLIN);
		if (timerFd < 0)
		{
			return -1;
		}
		RegisterEventHandlerToEpoll(epollFd, timerFd, &timerEventData, EPOLLIN);
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

	int dtime = 10;
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
			verticalServoAngle = ++verticalServoAngle;
			if (verticalServoAngle > VERTICAL_SERVO_MAX_ANGLE)
			{
				verticalServoAngle = VERTICAL_SERVO_MAX_ANGLE;
			}
		}
		else if (avt > avd)
		{
			verticalServoAngle = --verticalServoAngle;
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