
#include <WiFiType.h>

void onIpAssign(WiFiEvent_t evt, WiFiEventInfo_t info);
void onStationDisconnected(WiFiEvent_t evt, WiFiEventInfo_t info);
void receiveFromSerial();
void receiveFromCam();
void serialPrintHex(uint8_t msg[], int numBytes);
void serialPrintMac(uint8_t *bssid);

void startAP();
void stopAP();

void sendToSingleCam(uint8_t* req, int numBytes, uint32_t camIp);

void drawMainLayout();
void drawTestLayout();
void checkTouch();
void sendToCam(uint8_t* req, int numBytes);
void heartbeat();
void refreshActionButton();