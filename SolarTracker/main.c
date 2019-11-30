#include <errno.h>
#include <stdbool.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <applibs/log.h>
#include <applibs/gpio.h>
#include <applibs/adc.h>
#include <applibs/i2c.h>
#include <applibs/pwm.h>
#include <soc/mt3620_i2cs.h>

#include "hw/solar_tracker_hardware.h"
#include "epoll_timerfd_utilities.h"
#include "cloud_options.h"
#include "rgb-lcd.h"
#include "servo.h"
#include "device_twin.h"
#include "device_twin.h"
#include "azure_iot_utilities.h"
#include "connection_strings.h"

// Support functions.
static void TerminationHandler(int signalNumber);
static int InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);
static void TrackerTimerEventHandler(EventData* eventData);
static void PowerTimerEventHandler(EventData* eventData);

// File descriptors - initialized to invalid value
int epollFd = -1;
int trackerUpdateTimerFd = -1;
int powerUpdateTimerFd = -1;
int adcControllerFd = -1;
int sensorSelectAfd = -1;
int sensorSelectBfd = -1;
int pwmControllerFd = -1;

// The maximum voltage
static float maxVoltage = 2.5f;

uint8_t displayColumns = 16;
uint8_t displayRows = 2;

float verticalServoAngle = VERTICAL_SERVO_STANDBY_ANGLE;
float horizontalServoAngle = HORIZONTAL_SERVO_STANDBY_ANGLE;
uint32_t lightLevels[4];
struct _SERVO_State* verticalServo;
struct _SERVO_State* horizontalServo;

// Your servos might have slightly different duty cycles so you might want to edit the 
// config values.
static const uint32_t periodNs = 20000000;
static const uint32_t maxDutyCycleNs = 2400000;
static const uint32_t minDutyCycleNs = 600000;
static const uint32_t minAngle = 0;
static const uint32_t maxAngle = 180;

// event handler data structures. Only the event handler field needs to be populated.
static EventData trackerTimerEventData = { .eventHandler = &TrackerTimerEventHandler };
static EventData powerTimerEventData = { .eventHandler = &PowerTimerEventHandler };
// Termination state
volatile sig_atomic_t terminationRequired = false;

enum State { Working, Standby};

enum State status = Working;

float voltage = 0;
float brightness = 0;
float powerMilliWatt = 0;

/// <summary>
///     Allocates and formats a string message on the heap.
/// </summary>
/// <param name="messageFormat">The format of the message</param>
/// <param name="maxLength">The maximum length of the formatted message string</param>
/// <returns>The pointer to the heap allocated memory.</returns>
static void* SetupHeapMessage(const char* messageFormat, size_t maxLength, ...)
{
	va_list args;
	va_start(args, maxLength);
	char* message =
		malloc(maxLength + 1); // Ensure there is space for the null terminator put by vsnprintf.
	if (message != NULL) {
		vsnprintf(message, maxLength, messageFormat, args);
	}
	va_end(args);
	return message;
}

/// <summary>
///     Direct Method callback function, called when a Direct Method call is received from the Azure
///     IoT Hub.
/// </summary>
/// <param name="methodName">The name of the method being called.</param>
/// <param name="payload">The payload of the method.</param>
/// <param name="responsePayload">The response payload content. This must be a heap-allocated
/// string, 'free' will be called on this buffer by the Azure IoT Hub SDK.</param>
/// <param name="responsePayloadSize">The size of the response payload content.</param>
/// <returns>200 HTTP status code if the method name is reconginized and the payload is correctly parsed;
/// 400 HTTP status code if the payload is invalid;</returns>
/// 404 HTTP status code if the method name is unknown.</returns>
static int DirectMethodCall(const char* methodName, const char* payload, size_t payloadSize, char** responsePayload, size_t* responsePayloadSize)
{
	Log_Debug("\nDirect Method called %s\n", methodName);

	int result = 404; // HTTP status code.

	if (payloadSize < 32) {

		// Declare a char buffer on the stack where we'll operate on a copy of the payload.  
		char directMethodCallContent[payloadSize + 1];

		// Prepare the payload for the response. This is a heap allocated null terminated string.
		// The Azure IoT Hub SDK is responsible of freeing it.
		*responsePayload = NULL;  // Reponse payload content.
		*responsePayloadSize = 0; // Response payload content size.


		// Look for the haltApplication method name.  This direct method does not require any payload, other than
		// a valid Json argument such as {}.

		if (strcmp(methodName, "haltApplication") == 0) {

			// Log that the direct method was called and set the result to reflect success!
			Log_Debug("haltApplication() Direct Method called\n");
			result = 200;

			// Construct the response message.  This response will be displayed in the cloud when calling the direct method
			static const char resetOkResponse[] =
				"{ \"success\" : true, \"message\" : \"Halting Application\" }";
			size_t responseMaxLength = sizeof(resetOkResponse);
			*responsePayload = SetupHeapMessage(resetOkResponse, responseMaxLength);
			if (*responsePayload == NULL) {
				Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");
				abort();
			}
			*responsePayloadSize = strlen(*responsePayload);

			// Set the terminitation flag to true.  When in Visual Studio this will simply halt the application.
			// If this application was running with the device in field-prep mode, the application would halt
			// and the OS services would resetart the application.
			terminationRequired = true;
			return result;
		}

		// Check to see if the setSensorPollTime direct method was called
		else if (strcmp(methodName, "setSensorPollTime") == 0) {

			// Log that the direct method was called and set the result to reflect success!
			Log_Debug("setSensorPollTime() Direct Method called\n");
			result = 200;

			// The payload should contain a JSON object such as: {"pollTime": 20}
			if (directMethodCallContent == NULL) {
				Log_Debug("ERROR: Could not allocate buffer for direct method request payload.\n");
				abort();
			}

			// Copy the payload into our local buffer then null terminate it.
			memcpy(directMethodCallContent, payload, payloadSize);
			directMethodCallContent[payloadSize] = 0; // Null terminated string.

			JSON_Value* payloadJson = json_parse_string(directMethodCallContent);

			// Verify we have a valid JSON string from the payload
			if (payloadJson == NULL) {
				goto payloadError;
			}

			// Verify that the payloadJson contains a valid JSON object
			JSON_Object* pollTimeJson = json_value_get_object(payloadJson);
			if (pollTimeJson == NULL) {
				goto payloadError;
			}

			// Pull the Key: value pair from the JSON object, we're looking for {"pollTime": <integer>}
			// Verify that the new timer is < 0
			int newPollTime = (int)json_object_get_number(pollTimeJson, "pollTime");
			if (newPollTime < 1) {
				goto payloadError;
			}
			else {

				Log_Debug("New PollTime %d\n", newPollTime);

				// Construct the response message.  This will be displayed in the cloud when calling the direct method
				static const char newPollTimeResponse[] =
					"{ \"success\" : true, \"message\" : \"New Sensor Poll Time %d seconds\" }";
				size_t responseMaxLength = sizeof(newPollTimeResponse) + strlen(payload);
				*responsePayload = SetupHeapMessage(newPollTimeResponse, responseMaxLength, newPollTime);
				if (*responsePayload == NULL) {
					Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");
					abort();
				}
				*responsePayloadSize = strlen(*responsePayload);

				// Define a new timespec variable for the timer and change the timer period
				struct timespec newAccelReadPeriod = { .tv_sec = newPollTime,.tv_nsec = 0 };
				SetTimerFdToPeriod(powerUpdateTimerFd, &newAccelReadPeriod);
				return result;
			}
		}
		else {
			result = 404;
			Log_Debug("INFO: Direct Method called \"%s\" not found.\n", methodName);

			static const char noMethodFound[] = "\"method not found '%s'\"";
			size_t responseMaxLength = sizeof(noMethodFound) + strlen(methodName);
			*responsePayload = SetupHeapMessage(noMethodFound, responseMaxLength, methodName);
			if (*responsePayload == NULL) {
				Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");
				abort();
			}
			*responsePayloadSize = strlen(*responsePayload);
			return result;
		}

	}
	else {
		Log_Debug("Payload size > 32 bytes, aborting Direct Method execution\n");
		goto payloadError;
	}

	// If there was a payload error, construct the 
	// response message and send it back to the IoT Hub for the user to see
payloadError:


	result = 400; // Bad request.
	Log_Debug("INFO: Unrecognised direct method payload format.\n");

	static const char noPayloadResponse[] =
		"{ \"success\" : false, \"message\" : \"request does not contain an identifiable "
		"payload\" }";

	size_t responseMaxLength = sizeof(noPayloadResponse) + strlen(payload);
	responseMaxLength = sizeof(noPayloadResponse);
	*responsePayload = SetupHeapMessage(noPayloadResponse, responseMaxLength);
	if (*responsePayload == NULL) {
		Log_Debug("ERROR: Could not allocate buffer for direct method response payload.\n");
		abort();
	}
	*responsePayloadSize = strlen(*responsePayload);

	return result;

}

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
	// Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
	terminationRequired = true;
}

void recalculateServoAngles(void)
{
	int tl = lightLevels[SENSOR_TL_SELECT];
	int tr = lightLevels[SENSOR_TR_SELECT];
	int bl = lightLevels[SENSOR_BL_SELECT];
	int br = lightLevels[SENSOR_BR_SELECT];

	int avt = (tl + tr) / 2; // average value top
	int avd = (bl + br) / 2; // average value down
	int avl = (tl + bl) / 2; // average value left
	int avr = (tr + br) / 2; // average value right

	brightness = (avt + avl) / 2;

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
	if (!solarTrackerEnabled) 
	{
		printLine(0, "Disabled from");
		printLine(1, "Azure");
		SERVO_SetAngle(verticalServo, VERTICAL_SERVO_DISABLED_ANGLE);
		SERVO_SetAngle(horizontalServo, HORIZONTAL_SERVO_DISABLED_ANGLE);
		const struct timespec sleepTime = { STANDBY_UPDATE_SPEED, 0 };
		nanosleep(&sleepTime, NULL);
		return;
	}
#endif

	if (brightness < STANDBY_CUT_OFF_LIGHT_LEVEL)
	{
		status = Standby;
		printLine(0, "Low light level");
		printLine(1, "Standing by");
		SERVO_SetAngle(verticalServo, VERTICAL_SERVO_STANDBY_ANGLE);
		SERVO_SetAngle(horizontalServo, HORIZONTAL_SERVO_STANDBY_ANGLE);
		const struct timespec sleepTime = { STANDBY_UPDATE_SPEED, 0 };
		nanosleep(&sleepTime, NULL);
		return;
	}
	
	status = Working;

	int dvert = avd - avt; // check the difference of up and down
	int dhoriz = avr - avl;// check the difference of left and right

	verticalServoAngle += dvert / SERVO_CONVERGING_SPEED;
	if (verticalServoAngle < VERTICAL_SERVO_MIN_ANGLE)
		verticalServoAngle = VERTICAL_SERVO_MIN_ANGLE;
	else if (verticalServoAngle > VERTICAL_SERVO_MAX_ANGLE)
		verticalServoAngle = VERTICAL_SERVO_MAX_ANGLE;
	SERVO_SetAngle(verticalServo, verticalServoAngle);

	horizontalServoAngle += dhoriz / SERVO_CONVERGING_SPEED;
	if (horizontalServoAngle < HORIZONTAL_SERVO_MIN_ANGLE)
		horizontalServoAngle = HORIZONTAL_SERVO_MIN_ANGLE;
	else if (horizontalServoAngle > HORIZONTAL_SERVO_MAX_ANGLE)
		horizontalServoAngle = HORIZONTAL_SERVO_MAX_ANGLE;
	SERVO_SetAngle(horizontalServo, horizontalServoAngle);
}

unsigned int AdcRead(int channel)
{
	uint32_t value;
	int result = ADC_Poll(adcControllerFd, channel, &value);
	if (result < -1) {
		Log_Debug("ADC_Poll failed with error: %s (%d)\n", strerror(errno), errno);
		terminationRequired = true;
		return -1;
	}

	return value;
}

/// <summary>
///	Updating the light levels of the photo sensors and calculating new positions for the servos
/// </summary>
void RefreshServoPositions()
{
	for (int channel = 0; channel < SENSOR_NUM; channel++)
	{
		GPIO_SetValue(sensorSelectAfd, (channel & 1) > 0);
		GPIO_SetValue(sensorSelectBfd, (channel & 2) > 0);

		lightLevels[channel] = AdcRead(PHOTO_SENSOR_CHANNEL);
	}

	recalculateServoAngles();
}

/// <summary>
///		Periodically updates the servos based on the sensors' data 
/// </summary>
static void TrackerTimerEventHandler(EventData* eventData)
{
	if (ConsumeTimerFdEvent(trackerUpdateTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	RefreshServoPositions();
}

/// <summary>
///		Periodically updates power production on the display and in Azure
/// </summary>
static void PowerTimerEventHandler(EventData* eventData)
{
	if (ConsumeTimerFdEvent(powerUpdateTimerFd) != 0) {
		terminationRequired = true;
		return;
	}

	if (status == Working)
	{
		voltage = 2.5 * AdcRead(SOLAR_PANEL_CHANNEL) / 4096;
		powerMilliWatt = 1000 * voltage * voltage / RESISTANCE;
		char buf[17]; // \0 will be the 17th character
		if (powerMilliWatt > 10) { snprintf(buf, 17, "Power: %.1f mW", powerMilliWatt); }
		else if(powerMilliWatt > 1) { snprintf(buf, 17, "Power: %.2f mW", powerMilliWatt); }
		else { snprintf(buf, 17, "Power: %.3f mW", powerMilliWatt); }
		printLine(0, buf);
		snprintf(buf, 17, "Sensor avg: %d", (int)brightness);
		printLine(1, buf);
	}

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))

	// Allocate memory for a telemetry message to Azure
	char* pjsonBuffer = (char*)malloc(JSON_BUFFER_SIZE);
	if (pjsonBuffer == NULL) {
		Log_Debug("ERROR: not enough memory to send telemetry");
	}

	// construct the telemetry message
	snprintf(pjsonBuffer, JSON_BUFFER_SIZE, "{\"Power\":\"%.4lf\", \"Voltage\":\"%.4lf\", \"Brightness\":\"%.4lf\"}",
		powerMilliWatt, voltage, brightness);

	Log_Debug("\n[Info] Sending telemetry: %s\n", pjsonBuffer);
	AzureIoT_SendMessage(pjsonBuffer);
	free(pjsonBuffer);

#endif 
}

/// <summary>
///		Initializes a servo
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
int InitServo(int pwmFd, unsigned int channel, struct _SERVO_State** servo, int minAngle, int maxAngle)
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

/// <summary>
///		Displays initialization related errors on the LCD display
/// </summary>
void DisplayInitError(char* errorMessage)
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
	printLine(0, "Initializing");
	printLine(1, "Solar Tracker");

	// Clearing the display from previous leftover messages
	const struct timespec sleepTime = { STANDBY_UPDATE_SPEED, 0 };
	nanosleep(&sleepTime, NULL);

	//// LCD display is initialized

	//// Init sensor channel select GPIO-s

	sensorSelectAfd = GPIO_OpenAsOutput(SENSOR_SELECT_A_GPIO, GPIO_OutputMode_PushPull, GPIO_Value_High);
	sensorSelectBfd = GPIO_OpenAsOutput(SENSOR_SELECT_B_GPIO, GPIO_OutputMode_PushPull, GPIO_Value_High);
	// Select C pin of the 4051 multiplexer is hardwired to GND
	if (sensorSelectAfd < 0 || sensorSelectBfd < 0) {
		DisplayInitError("ADC ch select");
		Log_Debug("Error opening GPIO: %s (%d). Check that app_manifest.json includes the GPIO used.\n", strerror(errno), errno);
		return -1;
	}

	//// Sensor channel select GPIO-s are initialized

	//// ADC connection

	adcControllerFd = ADC_Open(ADC_CONTROLLER);
	if (adcControllerFd < 0) {
		DisplayInitError("ADC controller");
		Log_Debug("ADC_Open failed with error: %s (%d)\n", strerror(errno), errno);
		return -1;
	}

	result = ADC_SetReferenceVoltage(adcControllerFd, PHOTO_SENSOR_CHANNEL, maxVoltage);
	if (result < 0) {
		DisplayInitError("ADC ref voltage");
		Log_Debug("ADC_SetReferenceVoltage failed with error : %s (%d)\n", strerror(errno), errno);
		return -1;
	}

	//// ADC Connection is initialized

	//// Timers init

	static const struct timespec trackerUpdatePeriod = { .tv_sec = 0,.tv_nsec = WORKING_UPDATE_SPEED };
	trackerUpdateTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &trackerUpdatePeriod, &trackerTimerEventData, EPOLLIN);
	if (trackerUpdateTimerFd < 0)
	{
		return -1;
	}

	static const struct timespec powerUpdatePeriod = { .tv_sec = 0,.tv_nsec = POWER_UPDATE_SPEED };
	powerUpdateTimerFd = CreateTimerFdAndAddToEpoll(epollFd, &powerUpdatePeriod, &powerTimerEventData, EPOLLIN);
	if (powerUpdateTimerFd < 0)
	{
		return -1;
	}

	//// Timers initialized

	//// Servos

	pwmControllerFd = PWM_Open(PWM_CONTROLLER);
	if (pwmControllerFd == -1) {
		Log_Debug(
			"Error opening PWM_CONTROLLER: %s (%d). Check that app_manifest.json "
			"includes the PWM used.\n",
			strerror(errno), errno);
		return -1;
	}

	InitServo(pwmControllerFd, VERTICAL_SERVO_PWM_CHANNEL, &verticalServo, minAngle, maxAngle);
	InitServo(pwmControllerFd, HORIZONTAL_SERVO_PWM_CHANNEL, &horizontalServo, minAngle, maxAngle);

	SERVO_SetAngle(verticalServo, verticalServoAngle);
	SERVO_SetAngle(horizontalServo, horizontalServoAngle);

	//// Servos are initialized

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
	// Tell the system about the callback function that gets called when we receive a device twin update message from Azure
	AzureIoT_SetDeviceTwinUpdateCallback(&deviceTwinChangedHandler);
	// Tell the system about the callback function to call when we receive a Direct Method message from Azure
	AzureIoT_SetDirectMethodCallback(&DirectMethodCall);
#endif

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
	CloseFdAndPrintError(trackerUpdateTimerFd, "TrackerUpdateTimer");
	CloseFdAndPrintError(powerUpdateTimerFd, "PowerUpdateTimer");
	CloseFdAndPrintError(sensorSelectAfd, "SensorChannelSelectA");
	CloseFdAndPrintError(sensorSelectBfd, "SensorChannelSelectB");
	CloseFdAndPrintError(adcControllerFd, "ADC");
	CloseFdAndPrintError(pwmControllerFd, "PwmFd");
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

#if (defined(IOT_CENTRAL_APPLICATION) || defined(IOT_HUB_APPLICATION))
		// Setup the IoT Hub client.
		// Notes:
		// - it is safe to call this function even if the client has already been set up, as in
		//   this case it would have no effect;
		// - a failure to setup the client is a fatal error.
		if (!AzureIoT_SetupClient()) {
			Log_Debug("ERROR: Failed to set up IoT Hub client\n");
			break;
		}

		// AzureIoT_DoPeriodicTasks() needs to be called frequently in order to keep active
		// the flow of data with the Azure IoT Hub
		AzureIoT_DoPeriodicTasks();
#endif 
	}

	ClosePeripheralsAndHandlers();
	Log_Debug("Application exiting.\n");
	return 0;
}