#pragma once

#include <Arduino.h>

String getDisplayLatestRecord();
String getDisplayStoredDataHtml();
void streamDisplayStoredData(void (*emit)(const String&));
String getDisplayStorageLabel();

void displayGatewaySetup();
void displayGatewayLoop();
