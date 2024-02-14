#include <M5StickC.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WiFiUdp.h>
#include "GoProCam.h"
#include "SPI.h"
#include "GoPro_M5StickC_RC.h"

//--------------------- GoPro MAC and IP declarations ------------------
//---change these to yours----------------------------------------------
uint8_t Cam1Mac[6] = {0xD4, 0x32, 0x60, 0x94, 0x3B, 0x62}; //HERO-8 //MAC of cam "M-040-01" = 0x04, 0x41, 0x69, 0x4F, 0x0F, 0x4B
uint8_t Cam2Mac[6] = {0x04, 0x41, 0x69, 0x5E, 0x4A, 0x33}; //HERO-5 "M-040-02"
uint8_t Cam3Mac[6] = {0x04, 0x41, 0x69, 0x5F, 0x11, 0x39}; //HERO-5 "M-040-03"
uint8_t Cam4Mac[6] = {0x04, 0x41, 0x69, 0x5F, 0x72, 0x39}; //HERO-5 "M-040-04"
//---don't change the rest----------------------------------------------

//Program variables ----------------------------------------------------
const int maxCams = 1;
uint8_t numConnected = 0;
uint8_t oldNumConnected = 0;
uint8_t newConnected = 0;
uint8_t currentMode = 10;
int camConTimeout = 5000;
// GoProCam cams[maxCams] = {GoProCam(Cam1Mac), GoProCam(Cam2Mac), GoProCam(Cam3Mac), GoProCam(Cam4Mac)};
// there is room for only one camera on lcd - sorry
GoProCam cams[maxCams] = {GoProCam(Cam1Mac)};
int lowCounter = 0;      // msg counter 1
int highCounter = 0;     // msg counter 2
int cmdIndicator = 0;    // last sent cmd indicator
bool lastCmd = false;    // switches between two different commands
int heartbeatRate = 800;
unsigned long lastHeartbeat;
bool rcOn = true;
bool isRecording = false;
struct station_info *stat_info;
uint8_t packetBuffer[1024]; // buffer to hold incoming and outgoing packets

//Cam commands ----------------------------------------------------------
uint8_t PW0[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x50, 0x57, 0x00}; // power off
uint8_t SH1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x53, 0x48, 0x01}; // shutter start (Also stops a recording in progress)
uint8_t SH0[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x53, 0x48, 0x00}; // shutter stop
uint8_t CMd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x43, 0x4D, 0x06}; // change mode (6: 'default mode')
uint8_t PA[] =  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x41, 0x02, 0x01}; //toggle cam mode
uint8_t OO1[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x4F, 0x01}; // One on One, used by rc, keeps connected
uint8_t st[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x74}; // status request
uint8_t lc[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6C, 0x63, 0x05}; // get status display (w=60px, h=75px, 1bpp)
uint8_t getBL[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 'Y', 'Y', 0x00, 0x08, 0x00, 0x00, 0x00}; // get battery level, 2nd byte in response is battery level in %

//Unused cam commands ------------------------------------------------------
//uint8_t CMv[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x43, 0x4D, 0x00}; // change mode (0: 'video')
//uint8_t CMp[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x43, 0x4D, 0x01}; // change mode (1: 'photo')
//uint8_t CMb[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x43, 0x4D, 0x02}; // change mode (2: 'burst')
//uint8_t CMl[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x43, 0x4D, 0x03}; // change mode (3: 'timelapse')
//uint8_t wt[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x77, 0x74}; // wifi

//Remote specific declarations -----------------------------------------
const unsigned int rcUdpPort = 8383;            // Local UDP-Port 8383
const unsigned int camUdpPort = 8484;           // Cams UDP-Port 8484
const unsigned int wifiChannel = 1;             // WiFi-Channel of Smart-Remote = 1
//uint8_t rcMac[] = {0xd8, 0x96, 0x85, 0x80, 0x00, 0x01}; // Universal MAC-Address of Smart-Remote
//const char *ssid = "HERO-RC-800001";          // Universal SSID of my Smart-Remote
uint8_t rcMac[] = {0x86, 0xF3, 0xEB, 0xE4, 0x23, 0xDD}; // MAC-Address of my Smart-Remote
const char *ssid = "HERO-RC-A1111425435131";    // SSID of my Smart-Remote
const char *g_hostname = "ESP_E423DD";          // Hostname of my Smart-Remote
IPAddress rcIp(10, 71, 79, 1);                    // IP of my Smart-Remote
IPAddress gateway(10, 71, 79, 1);               // GW of my Smart-Remote
IPAddress subnet(255, 255, 255, 0);             // SM of my Smart-Remote

// UI State ------------------------------------------------------------
uint8_t current_ui_action = 0;

//Instances ------------------------------------------------------------
WiFiUDP Udp;

//----------------------------------------------------------------------
void setup() {  

  M5.begin();
  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setCursor(0,0);
  M5.Lcd.setTextColor(TFT_WHITE); M5.Lcd.setTextSize(1);
  M5.Lcd.println("Start up remote...");
  
  WiFi.mode(WIFI_AP); // Set WiFi in AP mode

  esp_wifi_set_mac(WIFI_IF_AP, &rcMac[0]);  

  WiFi.onEvent(onIpAssign, WiFiEvent_t::ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED);
  WiFi.onEvent(onStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);



  WiFi.disconnect(true);
  WiFi.softAPdisconnect(true);

  Serial.begin(115200);

  //setup is done
  delay(250);
  Serial.println("");
  Serial.println("Ready!");


  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(TFT_WHITE);  M5.Lcd.setTextSize(1);
  M5.Lcd.println("Remote ready!");

  delay(500);

  startAP();

  drawMainLayout();  
}

//----------------------------------------------------------------------
void loop(void) {
  yield();

  checkTouch();

  if (oldNumConnected != numConnected) {
    oldNumConnected = numConnected;

    M5.Lcd.fillRect(70, 0, 8, 8, TFT_BLACK);
    M5.Lcd.setTextColor(TFT_WHITE);  M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(70, 0);
    M5.Lcd.print(numConnected);
  }

  if (millis() - lastHeartbeat > heartbeatRate) {
    lastHeartbeat = millis();
    heartbeat();
    // drawTestLayout();
  }
}

//----------------------------------------------------------------------
//Touch functions
void checkTouch() {
  M5.update();

  uint8_t btn_a_pressed = M5.BtnA.wasPressed();
  uint8_t btn_b_pressed = M5.BtnB.wasPressed();

  if (btn_b_pressed) {
    current_ui_action = ( current_ui_action+1 ) % 4;
    refreshActionButton();
  }
  
  if (btn_a_pressed) { 
    if (current_ui_action == 0) { //button 1 (rec)
      if (isRecording) { // stop record pressed
        sendToCam(SH0, 14);   

        isRecording = !isRecording;
        delay(250); //avoid bouncing
      } else { // start record pressed
        sendToCam(SH1, 14);
        if (currentMode == 0) { //if video mode draw stop rect
          
          isRecording = !isRecording;
        }
        delay(250); //avoid bouncing
      }
      refreshActionButton();
    } else if (current_ui_action == 1) { //button 2 (default mode)
      sendToCam(CMd, 14);
      delay(250); //avoid bouncing
    } else if (current_ui_action == 2) { //button 3 (toggle mode)
      sendToCam(PA, 15);
      delay(250); //avoid bouncing
    } else if (current_ui_action == 3) { //button 4 (power off)
      if (rcOn) {
        for (uint8_t i = 0; i < 6; i++) {
          sendToCam(PW0, 14);
          delay(200); //enough bounce prevention ;)
        }

        stopAP();        
      } else {
        startAP();
        
        delay(250); //avoid bouncing
      }
      refreshActionButton();
    }
  }
}

//----------------------------------------------------------------------
//TFT functions
// Test function for aligning items on screen
void drawTestLayout() {
    int i = 0;

    // number of connected gopros
    M5.Lcd.fillRect(70, 0, 8, 8, TFT_BLACK);
    M5.Lcd.setTextColor(TFT_WHITE);  M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(70, 0);
    M5.Lcd.print("4");

    // battery level
    M5.Lcd.fillRect(48, 13, 3*8, 8, TFT_WHITE);
    M5.Lcd.setTextColor(TFT_BLACK);  M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(48 + 0 * 80, 13);
    M5.Lcd.print("100%");

    // camera mode
    M5.Lcd.fillRect( 6*6 + i * 80, 37, 6*7, 8, TFT_BLACK);
    M5.Lcd.setCursor(6*6 + i * 80, 37);
    M5.Lcd.setTextColor(TFT_WHITE);  M5.Lcd.setTextSize(1);
    M5.Lcd.print("Video");

    // camera state
    M5.Lcd.fillRect(1 + i * 80, 53, 6*10, 8, TFT_BLACK);
    M5.Lcd.setCursor(5 + i * 80, 53);
    
    M5.Lcd.setTextColor(TFT_GREEN);  M5.Lcd.setTextSize(1);
    M5.Lcd.print("ready");
}


void drawMainLayout() {    
  M5.Lcd.setRotation(0);
  M5.Lcd.fillScreen(TFT_BLACK); //clear screen
  M5.Lcd.setTextColor(TFT_WHITE);  M5.Lcd.setTextSize(1);

  //header
  M5.Lcd.setCursor(4, 0);
  M5.Lcd.print("Cams conn: 0");

  for (uint8_t i = 0; i < maxCams; i++) {
    M5.Lcd.drawRect(0 + i * 80, 8, 80, 133, TFT_WHITE);   //Cam section outer frame

    M5.Lcd.fillRect(41 + i * 80,    17,  4,  7, TFT_WHITE);    //accu small rect
    M5.Lcd.fillRect(45 + i * 80,    13, 30, 15, TFT_WHITE);    //accu big rect

    M5.Lcd.fillRect(1 + i * 80,     65, 78, 75, TFT_YELLOW);   //LCD yellow background rect

    //Text
    M5.Lcd.setCursor(5 + i * 80, 17);
    M5.Lcd.print("Cam ");
    M5.Lcd.print(i + 1);


    M5.Lcd.setCursor(5 + i * 80, 37);
    M5.Lcd.print("Mode:");


    M5.Lcd.setCursor(5 + i * 80, 53);
    M5.Lcd.print("offline");    
  }

  //inner lines
  M5.Lcd.drawLine(1, 32, 80, 32, TFT_WHITE);    //first line
  M5.Lcd.drawLine(1, 48, 80, 48, TFT_WHITE);    //second line
  M5.Lcd.drawLine(1, 64, 80, 64, TFT_WHITE);    //third line

  //Buttons
  M5.Lcd.drawRect(1, 142, 78, 18, TFT_WHITE);    //Button 1 outer frame
  
  refreshActionButton();    
}

void refreshActionButton() {
  M5.Lcd.fillRect(2, 143, 76, 16, TFT_BLACK);    //Button 1 outer frame
  if (current_ui_action == 0) {

    if (isRecording) {
      M5.Lcd.fillRect(34, 145, 10, 10, TFT_RED); //stop rect  
    } else {      
      M5.Lcd.fillCircle(34 + 5, 145 + 5, 5, TFT_RED); //Button 1 red rec dot
    }
  } else if (current_ui_action == 1) {
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setCursor(5, 147); M5.Lcd.print("Def Mode");
  } else if (current_ui_action == 2) {
    M5.Lcd.setTextColor(TFT_WHITE);
    M5.Lcd.setCursor(5, 147); M5.Lcd.print("Mode+");
  } else if (current_ui_action == 3) {   
    if (rcOn) {
      M5.Lcd.setTextColor(TFT_RED);
      M5.Lcd.setCursor(5, 147); M5.Lcd.print("Pwr off");  
    } else {
      M5.Lcd.setTextColor(TFT_GREEN);
      M5.Lcd.setCursor(5, 147); M5.Lcd.print("Pwr on");  
    }
  }
}

void resetCamSection(uint8_t camIndex) {
  int camOffset = 80 * camIndex;

  M5.Lcd.fillRect(camOffset, 8, 80, 133, TFT_BLACK); //clear cam section

  M5.Lcd.setTextColor(TFT_WHITE);  M5.Lcd.setTextSize(1);

  //outer frame
  M5.Lcd.drawRect(camOffset, 8, 80, 133, TFT_WHITE);    //Cam 1 outer frame

  //inner lines
  M5.Lcd.drawLine(1, 32, 80, 32, TFT_WHITE);    //first line
  M5.Lcd.drawLine(1, 48, 80, 48, TFT_WHITE);    //second line
  M5.Lcd.drawLine(1, 64, 80, 64, TFT_WHITE);    //third line

  //accu symbol
  M5.Lcd.fillRect(41 + camOffset,    17,  4,  7, TFT_WHITE);    //accu small rect
  M5.Lcd.fillRect(45 + camOffset,    13, 30, 15, TFT_WHITE);    //accu big rect

  //LCD background
  M5.Lcd.fillRect(1 + camOffset,     65, 78, 75, TFT_YELLOW);   //LCD yellow background rect

  //Text
  M5.Lcd.setCursor(5 + camOffset, 17);
  M5.Lcd.print("Cam ");
  M5.Lcd.print(camIndex + 1);


  M5.Lcd.setCursor(5 + camOffset, 37);
  M5.Lcd.print("Mode:");

  M5.Lcd.setCursor(5 + camOffset, 53);
  M5.Lcd.print("offline"); 
}

void tftPrintLc(uint8_t* lcBuffer, int xpos, int ypos) {

  for (int y = 74; y > -1; y--) {
    for (uint8_t x = 0; x < 8; x++) {
      for (uint8_t b = 0; b < 8; b++) {
        if (bitRead(lcBuffer[(x + (y * 8)) + 15], 7 - b)) {
          M5.Lcd.drawPixel(x * 8 + b + xpos, 74 - y + ypos, TFT_BLACK);
        } else {
          M5.Lcd.drawPixel(x * 8 + b + xpos, 74 - y + ypos, TFT_YELLOW);
        }
      }
    }
  }
  
}

//----------------------------------------------------------------------
//Access point start/stop functions
void startAP() {
  WiFi.mode(WIFI_AP); // Set WiFi in AP mode

  tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_AP, g_hostname);

  lowCounter = 0; // msg counter counter 1 reset
  highCounter = 0; // msg counter counter 2 reset
  cmdIndicator = 0; // indicator reset
  WiFi.softAPConfig(rcIp, gateway, subnet);

  //Start AP
  WiFi.softAP(ssid, NULL, wifiChannel, 0, maxCams ); //.softAP(const char* ssid, const char* password, int channel, int ssid_hidden, int max_connection)

  //Start UDP
  Udp.begin(rcUdpPort);

  rcOn = true;
  Serial.println("RC turned on");
}

void stopAP() {
  numConnected = 0;
  Udp.stop();
  WiFi.softAPdisconnect(true);

  for (int i = 0; i < maxCams; i++) {
    if (cams[i].getIp() != 0) {
      cams[i].resetIp();
      resetCamSection(i); //reset screen for cam i
    }
  }

  rcOn = false;

  Serial.println("RC turned off");
}

//----------------------------------------------------------------------
//WiFi functions
void onStationDisconnected(WiFiEvent_t evt, WiFiEventInfo_t info) {
  for (int i = 0; i < maxCams; i++) {
    if (memcmp(info.wifi_sta_disconnected.bssid, cams[i].getMac(), 6) == 0) {
      if (cams[i].getIp() != 0) {
        cams[i].resetIp();
        resetCamSection(i);
        if (numConnected > 0) numConnected--;

        Serial.print("Cam ");
        Serial.print(i + 1);
        Serial.println(" raised onStationDisconnected");
      }
      break;
    }
  }
}

void onIpAssign(WiFiEvent_t evt, WiFiEventInfo_t info) {
  ip4_addr_t IPaddress;

  wifi_sta_list_t wifi_sta_list;
  tcpip_adapter_sta_list_t adapter_sta_list;

  memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
  memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));

  esp_wifi_ap_get_sta_list(&wifi_sta_list);
  tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);

  for (int i = 0; i < adapter_sta_list.num; i++) {
    tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];
    Serial.print("MAC ");
    serialPrintHex(station.mac, 6);
    Serial.println(" connecting...");

    for (int x = 0; x < maxCams; x++) {
      if (memcmp(station.mac, cams[x].getMac(), 6) == 0) {
        if (cams[x].getIp() != station.ip.addr) {
          cams[x].lastResponse = millis() + 5000;
          cams[x].setIp(station.ip.addr);
          numConnected++;
          Serial.print("Cam ");
          Serial.print(x + 1);
          Serial.println(" IP assigned");
          delay(50);
          sendToSingleCam(st, 13, station.ip.addr); //get status
        }
        break;
      }
    }
  }
}

//----------------------------------------------------------------------
//Cam communication
void sendToCam(uint8_t* req, int numBytes) {
  //send to each cam if online
  uint8_t sendings = 0;
  for (int i = 0; i < maxCams; i++) {
    if (cams[i].getIp() != 0) {
      Serial.print("Sending to cam ");
      Serial.println(i + 1);

      req[9] = (uint8_t)highCounter;
      req[10] = (uint8_t)lowCounter;

      Udp.beginPacket(cams[i].getIp(), camUdpPort);
      Udp.write(req, numBytes);
      Udp.endPacket();

      sendings++;
    }
  }

  //count up
  if (lowCounter >= 255) {
    highCounter++;
    lowCounter = 0;
  }
  if (highCounter >= 255) {
    highCounter = 0;
  }
  lowCounter++;

  while (sendings > 0) {
    receiveFromCam();
    sendings--;
  }
}

void sendToSingleCam(uint8_t* req, int numBytes, uint32_t camIp) {
  Serial.println("Sending welcome to new cam");

  req[9] = (uint8_t)highCounter;
  req[10] = (uint8_t)lowCounter;

  Udp.beginPacket(camIp, camUdpPort);
  Udp.write(req, numBytes);
  Udp.endPacket();

  //count up
  if (lowCounter >= 255) {
    highCounter++;
    lowCounter = 0;
  }
  if (highCounter >= 255) {
    highCounter = 0;
  }
  lowCounter++;

  receiveFromCam();
}

void receiveFromCam() {
  yield();
  unsigned long receiveStart = millis();

  int numBytes = Udp.parsePacket();

  while (!numBytes && 400 > millis() - receiveStart) { //400 is the receive timeout
    yield();
    numBytes = Udp.parsePacket();
  }

  if (numBytes) {
    char inCmd[3];

    Udp.read(packetBuffer, numBytes); // read the packet into the buffer

    inCmd[0] = packetBuffer[11];
    inCmd[1] = packetBuffer[12];
    inCmd[2] = 0; //terminate string

    for (int i = 0; i < maxCams; i++) {
      IPAddress iAdr(cams[i].getIp());
      if (Udp.remoteIP() == iAdr) {
        //cam foud
        Serial.print("Cam ");
        Serial.print(i + 1);
        Serial.println(" received");
        cams[i].lastResponse = millis();

        if (packetBuffer[13] == 0x1) { // illegal command for camera
          Serial.print("<illegal command \"");
          Serial.print(inCmd);
          Serial.print("\" in ");
          serialPrintHex(packetBuffer, numBytes);
          Serial.println(">");
        } else if (strstr_P(inCmd, PSTR("lc")) != NULL) { //got screen for RC
          tftPrintLc(packetBuffer, 10 + i * 80, 66);
        } else if (strstr_P(inCmd, PSTR("st")) != NULL) { //got status
          if (cams[i].camMode != packetBuffer[14]) {
            cams[i].camMode = packetBuffer[14];

            M5.Lcd.fillRect( 6*6 + i * 80, 37, 6*7, 8, TFT_BLACK);
            M5.Lcd.setCursor(6*6 + i * 80, 37);
            M5.Lcd.setTextColor(TFT_WHITE);  M5.Lcd.setTextSize(1);
            switch (packetBuffer[14]) { //mode
              case 0x0: //video mode
                M5.Lcd.print("Video");
                currentMode = 0;
                break;
              case 0x1: //photo mode
                M5.Lcd.print("Photo");
                currentMode = 1;
                break;
              case 0x2: //burst mode
                M5.Lcd.print("Burst");
                currentMode = 2;
                break;
              case 0x3: //timelapse mode
                M5.Lcd.print("Timel.");
                currentMode = 3;
                break;
            }
          }

          if (cams[i].camState != packetBuffer[15]) {
            cams[i].camState = packetBuffer[15];

            M5.Lcd.fillRect(1 + i * 80, 53, 6*10, 8, TFT_BLACK);
            M5.Lcd.setCursor(5 + i * 80, 53);
            switch (packetBuffer[15]) { //status
              case 0x0: //standby
                M5.Lcd.setTextColor(TFT_GREEN);  M5.Lcd.setTextSize(1);
                M5.Lcd.print("ready");
                break;
              case 0x1: //recording
                M5.Lcd.setTextColor(TFT_RED);  M5.Lcd.setTextSize(1);
                M5.Lcd.print("rec.");
                break;
            }
          }

        } else if (strstr_P(inCmd, PSTR("YY")) != NULL) { //special command
          if (packetBuffer[17] == 0xFC) { //command failure
            Serial.println(" cmd fail!");
            Serial.println();
          } else if (packetBuffer[15] == 0x07 && packetBuffer[16] == 0x1A) { //got DateTime from Cam - actually not used in this sketch
            cams[i].setCamTimeGotMillis(millis());
            uint8_t camTime[7] = {packetBuffer[20], packetBuffer[21], packetBuffer[22], packetBuffer[23], packetBuffer[24], packetBuffer[25], packetBuffer[26]};
            cams[i].setCamTime(camTime);
          } else if (packetBuffer[15] == 0x08 && packetBuffer[16] == 0x00) { //got battery level
            if (cams[i].getBattLevel() != packetBuffer[20]) {              
              
              M5.Lcd.fillRect(45 + i * 80,    13, 30, 15, TFT_WHITE); //accu 1 big rect              
              M5.Lcd.setTextColor(TFT_BLACK);  M5.Lcd.setTextSize(1);
              M5.Lcd.setCursor(48 + 0 * 80, 17);
              M5.Lcd.print(packetBuffer[20]);
              M5.Lcd.print("%");              
              cams[i].setBattLevel(packetBuffer[20]);
            }
          } else { //something else
            Serial.print("Cam ");
            Serial.print(i + 1);
            Serial.println(" has sent: ");
            Serial.print(inCmd);
            Serial.print(", Buffer: ");
            serialPrintHex(packetBuffer, numBytes);
            Serial.println();
            Serial.println();
          }
        }
        break;
      }
    }
  }
}

void heartbeat() {
  if (cmdIndicator == 0) {
    sendToCam(lc, 14); //get LCD
    cmdIndicator++;
  } else if (cmdIndicator == 1) {
    sendToCam(st, 13); //get status
    cmdIndicator++;
  } else if (cmdIndicator >= 2) {
    if (lastCmd) sendToCam(getBL, 18); //get battery level
    else sendToCam(OO1, 14); //keep connected
    lastCmd = !lastCmd;
    cmdIndicator = 0;
  }

  //check cams connection
  for (int i = 0; i < maxCams; i++) {
    if (cams[i].getIp() != 0 && millis() >= cams[i].lastResponse) {
      if (millis() - cams[i].lastResponse >= camConTimeout) {
        Serial.print("Cam ");
        Serial.print(i + 1);
        Serial.print(" kicked at T = ");
        Serial.print(millis());
        Serial.print(", Last response: ");
        Serial.println(cams[i].lastResponse);

        cams[i].resetIp(); //reset data for cam i
        resetCamSection(i); //reset screen for cam i

        if (numConnected > 0) numConnected--;
      }
    }
  }
}

//----------------------------------------------------------------------
//Helper functions
void serialPrintHex(uint8_t msg[], int numBytes) {
  for (int i = 0; i < numBytes; i++) {
    Serial.print(msg[i], HEX);
    if (i != numBytes - 1) Serial.print(" ");
  }
}
