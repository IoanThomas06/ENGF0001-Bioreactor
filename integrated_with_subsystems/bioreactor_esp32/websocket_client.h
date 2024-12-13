#ifndef WEBSOCKET_CLIENT_H
#define WEBSOCKET_CLIENT_H

#include <WebSocketsClient.h>
#include "data.h"

WebSocketsClient webSocket;

void handleIncomingMessage(uint8_t *payload);

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch(type) 
  {
    case WStype_DISCONNECTED:
      Serial.println("Websocket not connected.");
      break;
    case WStype_CONNECTED:
      Serial.println("Websocket connected.");
      break;
    case WStype_TEXT:
      handleIncomingMessage(payload);
      break;
  }
}

void setupWebSocket(const char* host, uint16_t port)
{
  webSocket.begin(host, port, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  Serial.println("Websocket setup successful.");
}

void sendData(const Data& data) 
{
  String jsonString = dataToJson(data);
  webSocket.sendTXT(jsonString);
}

#endif