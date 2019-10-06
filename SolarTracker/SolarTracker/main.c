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

//// ADC connection
static const char rtAppComponentId[] = "005180bc-402f-4cb3-a662-72937dbcde47";
static int sockFd = -1;
static void SendMessageToRTCore(void);
static void TimerEventHandler(EventData* eventData);
static void SocketEventHandler(EventData* eventData);
static int timerFd = -1;
uint8_t RTCore_status;

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

//// ADC connection

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

	Log_Debug("Light sensor: %d\n", analog_data.u32);
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

	SendMessageToRTCore();
}

/// <summary>
///     Helper function for TimerEventHandler sends message to real-time capable application.
/// </summary>
static void SendMessageToRTCore(void)
{
	static int iter = 0;

	// Send "Read-ADC-%d" message to real-time capable application.
	static char txMessage[32];
	sprintf(txMessage, "Read-ADC-%d", iter++);
	Log_Debug("Sending: %s\n", txMessage);

	int bytesSent = send(sockFd, txMessage, strlen(txMessage), 0);
	if (bytesSent == -1)
	{
		//Log_Debug("ERROR: Unable to send message: %d (%s)\n", errno, strerror(errno));
		terminationRequired = true;
		return;
	}
}

//// end ADC connection

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

		// Register one second timer to send a message to the real-time core.
		static const struct timespec sendPeriod = { .tv_sec = 1,.tv_nsec = 0 };
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

int main(int argc, char* argv[])
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

	// HAL_Delay(20);

	/*
	void HAL_Delay(int delayTime) {
	struct timespec ts;
	ts.tv_sec = 0;
	ts.tv_nsec = delayTime * 10000;
	nanosleep(&ts, NULL);
}
	*/
}