#include <ArduinoJson.h>

#ifndef DATA_H
#define DATA_H

typedef struct Data {
  String ph;
  String temp;
  String rpm;
} Data;

String dataToJson(const Data& data) {
  DynamicJsonDocument doc(1024);
  doc["pH"] = data.ph.toFloat() / 1000.0;
  doc["heat"] = data.temp.toFloat() / 100.0;
  doc["rpm"] = data.rpm.toInt();

  String jsonString;
  serializeJson(doc, jsonString);
  return jsonString;
}

#endif