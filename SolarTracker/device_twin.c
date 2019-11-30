#include <errno.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>

#include "epoll_timerfd_utilities.h"

#include <applibs/log.h>
#include <applibs/i2c.h>

#include "device_twin.h"
#include "azure_iot_utilities.h"
#include "parson.h"
#include "cloud_options.h"

extern volatile sig_atomic_t terminationRequired;

static const char cstrDeviceTwinJsonInteger[] = "{\"%s\": %d}";
static const char cstrDeviceTwinJsonFloat[] = "{\"%s\": %.2f}";
static const char cstrDeviceTwinJsonBool[] = "{\"%s\": %s}";
static const char cstrDeviceTwinJsonString[] = "{\"%s\": \"%s\"}";
#ifdef IOT_CENTRAL_APPLICATION
static const char cstrDeviceTwinJsonFloatIOTC[] = "{\"%s\": {\"value\": %.2f, \"status\" : \"completed\" , \"desiredVersion\" : %d }}";
static const char cstrDeviceTwinJsonBoolIOTC[] = "{\"%s\": {\"value\": %s, \"status\" : \"completed\" , \"desiredVersion\" : %d }}";
static const char cstrDeviceTwinJsonIntegerIOTC[] = "{\"%s\": {\"value\": %d, \"status\" : \"completed\" , \"desiredVersion\" : %d }}";
static const char cstrDeviceTwinJsonStringIOTC[] = "{\"%s\": {\"value\": %s, \"status\" : \"completed\" , \"desiredVersion\" : %d }}";
#endif 

static int desiredVersion = 0;

// Define each device twin key that we plan to catch, process, and send reported property for.
// .twinKey - The JSON Key piece of the key: value pair
// .twinVar - The address of the application variable keep this key: value pair data
// .twinType - The data type for this item, TYPE_BOOL, TYPE_STRING, TYPE_INT, or TYPE_FLOAT
twin_t twinArray[] = {
	{.twinKey = "DeviceEnabled",.twinVar = &solarTrackerEnabled} };

// Calculate how many twin_t items are in the array.  We use this to iterate through the structure.
int twinArraySize = sizeof(twinArray) / sizeof(twin_t);

///<summary>
///		check to see if any of the device twin properties have been updated.  If so, send up the current data.
///</summary>
void checkAndUpdateDeviceTwin(char* property, void* value, bool ioTCentralFormat)
{
	int nJsonLength = -1;

	char* pjsonBuffer = (char*)malloc(JSON_BUFFER_SIZE);
	if (pjsonBuffer == NULL) {
		Log_Debug("ERROR: not enough memory to report device twin changes.");
	}

	if (property != NULL) {

		// report current device twin data as reported properties to IoTHub

#ifdef IOT_CENTRAL_APPLICATION
		if (ioTCentralFormat) {
			nJsonLength = snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrDeviceTwinJsonBoolIOTC, property, *(bool*)value ? "true" : "false", desiredVersion);
		}
		else
#endif 
			nJsonLength = snprintf(pjsonBuffer, JSON_BUFFER_SIZE, cstrDeviceTwinJsonBool, property, *(bool*)value ? "true" : "false", desiredVersion);


		if (nJsonLength > 0) {
			Log_Debug("[MCU] Updating device twin: %s\n", pjsonBuffer);
			AzureIoT_TwinReportStateJson(pjsonBuffer, (size_t)nJsonLength);
		}
		free(pjsonBuffer);
	}
}

///<summary>
///		Parses received desired property changes.
///</summary>
///<param name="desiredProperties">Address of desired properties JSON_Object</param>
void deviceTwinChangedHandler(JSON_Object* desiredProperties)
{
	int result = 0;

	// Pull the twin version out of the message.  We use this value when we echo the new setting back to IoT Connect.
	if (json_object_has_value(desiredProperties, "$version") != 0)
	{
		desiredVersion = (int)json_object_get_number(desiredProperties, "$version");
	}

#ifdef IOT_CENTRAL_APPLICATION		

	for (int i = 0; i < (sizeof(twinArray) / sizeof(twin_t)); i++) {

		if (json_object_has_value(desiredProperties, twinArray[i].twinKey) != 0)
		{

			JSON_Object* currentJSONProperties = json_object_dotget_object(desiredProperties, twinArray[i].twinKey);

			*(bool*)twinArray[i].twinVar = (bool)json_object_get_boolean(currentJSONProperties, "value");
			solarTrackerEnabled = *(bool*)twinArray[i].twinVar;

			Log_Debug("Received device update. New %s is %s\n", twinArray[i].twinKey, *(bool*)twinArray[i].twinVar ? "true" : "false");
			checkAndUpdateDeviceTwin(twinArray[i].twinKey, twinArray[i].twinVar, true);
		}
	}
#else // !IOT_CENTRAL_APPLICATION		

	for (int i = 0; i < (sizeof(twinArray) / sizeof(twin_t)); i++) {

		if (json_object_has_value(desiredProperties, twinArray[i].twinKey) != 0)
		{
			*(bool*)twinArray[i].twinVar = (bool)json_object_get_boolean(desiredProperties, twinArray[i].twinKey);
			solarTrackerEnabled = *(bool*)twinArray[i].twinVar;

			Log_Debug("Received device update. New %s is %s\n", twinArray[i].twinKey, *(bool*)twinArray[i].twinVar ? "true" : "false");
			checkAndUpdateDeviceTwin(twinArray[i].twinKey, twinArray[i].twinVar, true);
		}
	}
#endif 

}
