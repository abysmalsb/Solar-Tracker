#pragma once

#include <applibs/gpio.h>
#include "parson.h"

#define JSON_BUFFER_SIZE 128

bool solarTrackerEnabled;

typedef struct {
	char* twinKey;
	void* twinVar;
} twin_t;

///<summary>
///		Parses received desired property changes.
///</summary>
///<param name="desiredProperties">Address of desired properties JSON_Object</param>
void deviceTwinChangedHandler(JSON_Object* desiredProperties);

void checkAndUpdateDeviceTwin(char*, void*, bool);