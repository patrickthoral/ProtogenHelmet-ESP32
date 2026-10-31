//Make sure you have everything connected by the schematic in the repository and set these defines correctly!

#define MICpin ADC1_CHANNEL_0 //Microphone, pin 1
#define T_in 2 //Output from Touch Sensor (for KY-032, Capac)
#define T_en 42 //Enable pin to Touch Sensor (for KY-032)
#define DATA_PIN_EARS 5  //Ears(Blush) (from outer to inner, POV-right cheek, (if blush: from top to bottom, right cheek nearest to ear first))
#ifndef DATA_PIN_VISOR //can be overridden per board from platformio.ini (build_flags)
#define DATA_PIN_VISOR 7 //Face (right cheek, left segment of eye first)
#endif
#define I2C_SDA 8 //SDA for Gyro, OLED, INA219, ToF
#define I2C_SCL 9 //SCL for Gyro, OLED, INA219, ToF
#ifndef I2C_CLOCK //can be overridden per board from platformio.ini (build_flags)
#define I2C_CLOCK 400000 //I2C bus speed in Hz
#endif
#define MAX_CLK 12 //Clock for MAX72xx matrixes if used
#define MAX_MOSI 11 //Data for MAX72xx matrixes if used
#define MAX_CS 10 //ChipSelect for MAX72xx matrixes if used
#define animBtn 4 //Pulling this pin LOW cycles through animations
#define fanPWM 13 //PWM pin to control 4pin fan
#define ONBOARD_LED_PIN 21 //Onboard WS2812, used as a heartbeat so you can see the ESP32 hasn't crashed

#define visorType "WS2812" // What displays are you using? (WS2812 or MAX72XX so far)
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW //flip up-down: ::DR1CR0RR1_HW , flip left-right: ::PAROLA_HW , flip both: ::ICSTATION_HW
#define MATRIXESNUM 11 // How many matrices for visor? 11
#define FADESTEPS 4 //how many steps when fading between frames? (0=disabled; only for WS2812 displays)

bool earPresent = false; // Are you using ear leds?
#define earLedsNum 74 // How many? (74 or 32 rn)

bool blushPresent = false; // Are you using blush leds?
#define blushLedsNum 8 // How many? (might crash under 8)
bool useRGBblush = true; //Swaps red-green for an RGB-ordered blush strip; false for regular GRB WS2812 blush LEDs (check with the POST: the red pass must show red on the blush too)


#define boopMode "APDS9960" //"KY-032" for KY-032, "Capac" for capacitive sensor/boop when HIGH, "APDS9960" for ADPS9960, "VL53L1X" for VL53L1X, leave empty for none

#define revertTilt 8000 //The maximum time that animation caused by tilt gets shown (used as if tilt bugs out etc)

#define oldMatrixFix false //fix for Legacy WS2812B-2020 matrix

#define oledAddr 60 //define oled on address 0x3c

#define POST_ENABLED 1 //LED power-on self test: walks a pixel through every visor matrix in parallel and down the ears/blush chain, once per color (white, red, green, blue) at boot, then resumes normally. Takes ~9s, set to 0 to skip
#define POST_STEP_MS 5 //extra delay per POST pixel step (pushing the 704-LED visor already takes ~21ms on its own)
#define STARTUP_OK_MSG "PROTOGEN OK" //shown once every module initialized fine; otherwise each failed module is shown in red instead (e.g. "OLED ERR"). The lines after it are set in the web UI (startMsg1/2)
#define STARTUP_MSG_MS 3000 //how long each startup message line stays up
#define POST_BLINK_MS 250 //onboard LED blinks the walk color on/off at this interval during the POST
#define HEARTBEAT_BPM 12 //onboard LED breaths per minute (12 = one every 5s)
#define HEARTBEAT_COLOR CRGB::Blue //single subpixel only
#define HEARTBEAT_MAX 8 //peak brightness of the onboard LED (0-255; it is very bright, keep this low). Breathes from fully off up to this

#define DEBUG_LOGS 0 //set to 1 to enable verbose debug logPrint() calls (e.g. sensor value dumps) for troubleshooting; 0 compiles them out entirely (no rebuild-free toggle, flip this and reflash)

//--------------------------------//No touching after this!

#include <Arduino.h>

#include "driver/adc.h" //Arduino 2.x/IDF4.4 legacy ADC driver (official espressif32 platform is pinned to arduino-esp32 2.0.17 / IDF4.4, which doesn't have the esp_adc/adc_oneshot.h driver from IDF5.x)

#define earTypeSize 6
#define visTypeSize 2
String earTypes[earTypeSize] = {"custom","rainbow","white_noise","corner_sabers","custom_glow","none"}; //available ear type animations
String visorTypes[visTypeSize] = {"custom","all_rainbow"}; //available visor type animations
String vTAcro[visTypeSize] = {"cust","rnbw"}; //OLED acronyms for visor type animations

#include <ezButton.h>
ezButton hwBtn(animBtn);

#include <StreamUtils.h>
#include <sstream>
#define FASTLED_ESP8266_RAW_PIN_ORDER
//#define FASTLED_RMT5 = 0 //doesnt compile
//FASTLED_ESP32_FLASH_LOCK is set in platformio.ini's build_flags: it's used by FastLED's own idf4_rmt_impl.cpp, which never sees a #define in this file
#include <FastLED.h>
#define ARDUINOJSON_USE_DOUBLE 0
#include <ArduinoJson.h>

#define CONFIG_LITTLEFS_SPIFFS_COMPAT 1
#include <LittleFS.h>

#include "fileOp.h" //CRC + Config variables store/save/load/default
Config cfg;

#include "Misc.h" //Misc/helping functions

#include "oled.h"
SSDOLED oled;

#include "SparkFunLSM6DS3.h"
#include <Wire.h>
LSM6DS3 myIMU;

#include <Adafruit_INA219.h> //edited library in this sketch (replace 0.1R with 0.03R resistor on the board)
Adafruit_INA219 ina219;
#define INA_CAL_VALUE 5572 //must match ina219_calValue in setCalibration_16V_8A()
#define INA_CURRENT_DIVIDER_MA 4 //must match ina219_currentDivider_mA in setCalibration_16V_8A()
#define INA_CONFIG_VALUE (INA219_CONFIG_BVOLTAGERANGE_16V | INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT | INA219_CONFIG_SADCRES_12BIT_1S_532US | INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS) //must match setCalibration_16V_8A()
uint32_t inaResets = 0;

//INA219 register access that bypasses the library's reads: those ignore I2C errors (a failed read converts an uninitialized value) and
//getCurrent_mA() rewrites the calibration before every read. Here the pointer write and the read are separate transactions with a STOP in
//between, with retries, so a marginal connection (a loose INA219 GND caused Error -1 reads and constant resets) shows up as INA ERR instead.
bool inaRead16(uint8_t reg, uint16_t &value) {
  for(int attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(INA219_ADDRESS);
    Wire.write(reg);
    if(Wire.endTransmission() != 0) continue;
    if(Wire.requestFrom((uint8_t)INA219_ADDRESS, (uint8_t)2) == 2) {
      value = Wire.read() << 8;
      value |= Wire.read();
      return true;
    }
  }
  return false;
}

bool inaWrite16(uint8_t reg, uint16_t value) {
  for(int attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(INA219_ADDRESS);
    Wire.write(reg);
    Wire.write(value >> 8);
    Wire.write(value & 0xFF);
    if(Wire.endTransmission() == 0) return true;
  }
  return false;
}

#include "Adafruit_APDS9960.h"
Adafruit_APDS9960 apds;

//Diagnostic: read the APDS9960 ID register directly, bypassing the library's begin()/ID whitelist --
//useful to see what a clone chip actually reports if init keeps failing. The vendored library here
//already accepts both 0xAB (official) and 0xA8 (a known clone ID), so a failure despite that means
//either the chip isn't ACKing on the I2C bus at all (wiring/address/power), or it's a third clone ID.
uint8_t apdsRawID() {
  Wire.beginTransmission(0x39); //APDS9960_ADDRESS
  Wire.write(0x92); //APDS9960_ID register
  if(Wire.endTransmission(false) != 0) return 0xFF; //no ACK, not a real reading
  Wire.requestFrom((uint8_t)0x39, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

#include "Adafruit_VL53L1X.h"
Adafruit_VL53L1X vl53;

//--------------------------------//realtime logger
#define LOG_BUFFER_SIZE (50 * 1024)  // 50 KB

char *logBuffer = nullptr;
size_t logIndex = 0;

void logPrint(const char *str) {
  Serial.println(str);
  if (!logBuffer) return;
  size_t len = strlen(str);
  size_t needed = len + 1; //newline
  if (logIndex + needed >= LOG_BUFFER_SIZE) {
    logIndex = 0;
    logBuffer[0] = '\0';
  }
  memcpy(logBuffer + logIndex, str, len);
  logIndex += len;
  logBuffer[logIndex++] = '\n';
  logBuffer[logIndex] = '\0';
}

inline void logPrint(const __FlashStringHelper *str) {
  logPrint((const char*)str);
}

void logPrint(const String &str) {
    logPrint(str.c_str());
}

#if DEBUG_LOGS
  #define logDebug(x) logPrint(x)
#else
  #define logDebug(x) //compiled out entirely -- the argument (e.g. a String concatenation) isn't even evaluated
#endif

//--------------------------------//web / wifi
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include <ElegantOTA.h>

AsyncWebServer server(80);

//--------------------------------//Config vars
unsigned long restartAt = 0; //set by the web UI when a change needs a reboot (WiFi SSID/password), loop() restarts once the response has gone out
bool instantReload = false, oledInitDone = false, tiltInitDone = false, getfilesProper = true, ToFInitDone = false, inaInitDone = false;
uint8_t currentEarsFrame = 0, currentVisorFrame = 0, numOfSegm, numAnimBlush, totalAnims;
uint16_t visorLedsNum = MATRIXESNUM*64;
int8_t segGridRow[MATRIXESNUM], segGridCol[MATRIXESNUM]; //-1 = segment not placed by /visType.txt
int gridRows = 0, gridCols = 0;
bool visLayoutAvail = false;
String currentAnim = "", animToLoad = "", availAnims[50], getfilesCache;
float micDC = 800;

//--------------------------------//getting stored anims names and count
void getFilesFunc() {
  getfilesCache = "";
  totalAnims = 0;
  File root = LittleFS.open("/anims");
  File file = root.openNextFile();
  while(file){
    availAnims[totalAnims] = String(file.name());
    getfilesCache += availAnims[totalAnims] + ";";
    totalAnims++;
    file = root.openNextFile();
  }
  getfilesProper = false;
}

//--------------------------------//Structs for anims in psram
struct FramesEars {
  int timespan;
  long ledColor[earLedsNum];
};

struct AnimNowEars {
  int type;
  int numOfFrames;
  FramesEars* frames = nullptr;
};

struct FramesVisor {
  int timespan;
  uint64_t leds[MATRIXESNUM];
  long ledsBlush[blushLedsNum];
  long fColor[MATRIXESNUM];
  long ppColor[MATRIXESNUM][64];
};

struct AnimNowVisor {
  int type;
  int numOfFrames;
  FramesVisor* frames = nullptr;
  bool isMouth[MATRIXESNUM];
};

AnimNowEars* earsNow;
AnimNowVisor* visorNow;

//--------------------------------//MAX LEDs
#include <MD_MAX72xx.h>
#include <SPI.h>

MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, MAX_MOSI, MAX_CLK, MAX_CS, MATRIXESNUM);

//--------------------------------//WS2812 LEDs
CRGB earLeds[earLedsNum];
CRGB blushLeds[blushLedsNum];
CRGB visorLeds[MATRIXESNUM*64];
CRGB visorLedsNEW[MATRIXESNUM*64];
CRGB c2Leds[earLedsNum+blushLedsNum];

CLEDController *ledController[2];
CRGB onboardLed[1];
uint8_t onboardBrightness = 0;
bool pushLeds = false; //set when any LED buffer changed; loop() pushes everything once via showAllLeds()

CRGB pixelBuffer[18];
CRGB visorPixelBuffer[10];
uint8_t noiseData[earLedsNum];

DEFINE_GRADIENT_PALETTE( blackWhite_gp ) {
  0,   100,  0, 0,
  120,   0,  0, 0,
  255, 255,  255, 255
};
CRGBPalette16 blackWhite = blackWhite_gp;

const std::vector<std::vector<int>> lookupDiag1 = 
{{14,15,16},
 {13,27,28,1},
 {12,26,36,17,2},
 {25,35,29,18},
 {11,34,37,30,3},
 {24,33,31,19},
 {10,23,32,20,4},
 {9,22,21,5},
 {8,7,6}};

const std::vector<std::vector<int>> lookupDiag2 = 
{{2,3,4},
 {1,18,19,5},
 {16,17,30,20,6},
 {28,29,31,21},
 {15,36,37,32,7},
 {27,35,33,22},
 {14,26,34,23,8},
 {13,25,24,9},
 {12,11,10}};

//--------------------------------//Load visor segment grid layout (for OLED thumbnail), mirrors animator.html's createHTML() parsing of the same file
const char DEFAULT_VIS_LAYOUT[] = "t0;t1;-;-;-;t9;t10;_;blush0;blush1;-;t5;-;blush2;blush3;_;t2;t3;t4;-;t6;t7;t8"; //animator.html's "M16 mk2" preset (11 segments, 4 blush), used when /visType.txt is missing or empty
void loadVisLayout() {
  for(int i = 0; i < MATRIXESNUM; i++) { segGridRow[i] = -1; segGridCol[i] = -1; }
  gridRows = 0;
  gridCols = 0;

  String content;
  File file = LittleFS.open("/visType.txt", "r");
  if(file) {
    content = file.readString();
    file.close();
  }
  content.trim();
  if(content.length() == 0) {
    logPrint(F("[I] No /visType.txt found, using the built-in M16 mk2 layout for the OLED thumbnail"));
    content = DEFAULT_VIS_LAYOUT;
  }

  int row = 0, col = 0, maxCol = 0, start = 0;
  bool any = false;
  for(int i = 0; i <= (int)content.length(); i++) {
    if(i == (int)content.length() || content[i] == ';') {
      String tok = content.substring(start, i);
      start = i+1;
      if(tok == "_") {
        if(col > maxCol) maxCol = col;
        row++;
        col = 0;
      } else {
        if(tok.startsWith("t")) {
          int idx = tok.substring(1).toInt();
          if(idx >= 0 && idx < MATRIXESNUM) {
            segGridRow[idx] = row;
            segGridCol[idx] = col;
            any = true;
          }
        } //"-" (empty) and "bN" (blush) cells just consume a grid column, same as animator.html
        col++;
      }
    }
  }
  if(col > maxCol) maxCol = col; //account for the last row (no trailing "_")
  gridRows = row+1;
  gridCols = maxCol;

  visLayoutAvail = any && (gridCols*8 <= 128) && (gridRows*8 <= 32); //32, not 35: writeVisorThumb()'s box must not overlap writeINA()'s box, which starts at y=32
  if(!visLayoutAvail) {
    logPrint(F("[E] /visType.txt layout empty or too large for the OLED, showing the animation name as text"));
  }
}

//--------------------------------//Load functions
bool loadAnim(String anim, String temp) {
  JsonDocument doc;
  DeserializationError error;

  if (currentAnim != anim || anim == "POSTAnimLoad") {
    if(anim == "POSTAnimLoad") {
      logPrint(F("[I] POST load"));
      error = deserializeJson(doc, temp);
    } else {
      delay(25);
      File file = LittleFS.open("/anims/"+anim, "r");
      if (!file) {
        logPrint(F("[E] There was an error opening the animation file!"));
        file.close();
        return false;
      }
      logPrint(F("[I] Animation file opened!"));
      ReadBufferingStream bufferedFile{file, 64};
      error = deserializeJson(doc, bufferedFile);
      file.close();
    }
    
    if(error){
      logPrint("[E] Failed to deserialize animation file! : " + String(error.c_str()));
      return false;
    }

    // Free previously allocated frames
    if (earsNow->frames) { free(earsNow->frames); earsNow->frames = nullptr; }
    if (visorNow->frames) { free(visorNow->frames); visorNow->frames = nullptr; }

    currentAnim = anim;

    //Ears anim type
    for(int o=0;o<earTypeSize;o++) {
      if(doc["ears"]["type"].as<String>() == earTypes[o]) {
        earsNow->type = o;
        break;
      }
    }
    //Ears anim load
    earsNow->numOfFrames = doc["ears"]["frames"].size();
    earsNow->frames = (FramesEars*) ps_malloc(sizeof(FramesEars) * earsNow->numOfFrames);
    for(int x = 0; x < earsNow->numOfFrames; x++) {
      earsNow->frames[x].timespan = doc["ears"]["frames"][x]["timespan"].as<int>();
      for(int y = 0; y < doc["ears"]["frames"][x]["leds"].size(); y++) {
        earsNow->frames[x].ledColor[y] = strtol(doc["ears"]["frames"][x]["leds"][y].as<String>().c_str(), NULL, 16);
      }
    }
    //Visor anim type
    for(int o=0;o<visTypeSize;o++) {
      if(doc["visor"]["type"].as<String>() == visorTypes[o]) {
        visorNow->type = o;
        break;
      }
    }
    //isMouth
    for(int y = 0; y < doc["visor"]["isMouth"].size(); y++) {
      visorNow->isMouth[y] = doc["visor"]["isMouth"][y].as<bool>();
    }
    //Visor anim load
    visorNow->numOfFrames = doc["visor"]["frames"].size();
    visorNow->frames = (FramesVisor*) ps_malloc(sizeof(FramesVisor) * visorNow->numOfFrames);
    for(int x = 0; x < visorNow->numOfFrames; x++) {
      visorNow->frames[x].timespan = doc["visor"]["frames"][x]["timespan"].as<int>();
      numAnimBlush = (uint8_t)doc["visor"]["frames"][x]["ledsBlush"].size();
      for(int y = 0; y < numAnimBlush; y++) {
        if(y<blushLedsNum) {
          visorNow->frames[x].ledsBlush[y] = strtol(doc["visor"]["frames"][x]["ledsBlush"][y].as<String>().c_str(), NULL, 16);
        }
      }
      numOfSegm = doc["visor"]["frames"][x]["leds"].size();
      for(int y = 0; y < numOfSegm; y++) {
        visorNow->frames[x].fColor[y] = strtol(doc["visor"]["frames"][x]["fColor"][y].as<String>().c_str(), NULL, 16); //should return 0 if not present
        visorNow->frames[x].leds[y] = strtoull(doc["visor"]["frames"][x]["leds"][y].as<String>().c_str(), NULL, 16); //string to uint64
      }
      for(int i = 0; i < MATRIXESNUM; i++) { //wipe ppColor data
        for(int o = 0; o < 64; o++) {
          visorNow->frames[x].ppColor[i][o] = 0;
        }
      }
      int numOfpp = doc["visor"]["frames"][x]["ppColor"].size();
      for(int y = 0; y < numOfpp; y++) {
        int numOfppData = doc["visor"]["frames"][x]["ppColor"][y]["data"].size();
        for(int z = 0; z < numOfppData; z++) { //ppColor[matrix][pixel] = color
          visorNow->frames[x].ppColor[doc["visor"]["frames"][x]["ppColor"][y]["mIndex"].as<int>()][doc["visor"]["frames"][x]["ppColor"][y]["data"][z][0].as<int>()] = strtol(doc["visor"]["frames"][x]["ppColor"][y]["data"][z][1].as<String>().c_str(), NULL, 16);
        }
      }
    }

    instantReload = true;
    currentVisorFrame = 0;
    currentEarsFrame = 0;

    if(cfg.oledEna && oledInitDone) {
      if(!cfg.oledAnimMode || !visLayoutAvail) { //Animation mode is live-updated by setAllVisor() itself; instantReload forces an immediate render
        oled.writeAnim(anim.substring(0,anim.length()-5));
      }
      oled.writeRGB(vTAcro[visorNow->type]);
    }
    return true;
  }
  return false;
}

//--------------------------------//BLE
#define CONFIG_BT_NIMBLE_MAX_CONNECTIONS 2
#define CONFIG_BT_NIMBLE_ROLE_CENTRAL_DISABLED
#define CONFIG_BT_NIMBLE_ROLE_OBSERVER_DISABLED
#define CONFIG_BT_NIMBLE_MEM_ALLOC_MODE_EXTERNAL 1
#include "NimBLEDevice.h"

BLEServer *pServer = NULL;
BLECharacteristic * pCharacteristic;
BLEAdvertising* pAdvertising;

class MyCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
      String temp = String(pCharacteristic->getValue().c_str());
      if(temp.charAt(0) == 'g') { //legacy remote reasons
        pCharacteristic->setValue("i"+String(totalAnims));
        pCharacteristic->notify();
      } else if (temp.charAt(0) == '?') {
        String animtemp;
        for(int i = 0; i < totalAnims; i++) {
          animtemp += availAnims[i].substring(0, availAnims[i].length() - 5);
          animtemp += ";";
        }
        pCharacteristic->setValue(animtemp);
        pCharacteristic->notify(true);
      } else if (temp.charAt(0) == ';') { //command
        if (temp.indexOf("rgb") > 0 && visorType == "WS2812") {
          visorNow->type++;
          if(visorNow->type == visTypeSize)
            visorNow->type = 0;
          if(cfg.oledEna && oledInitDone)
            oled.writeRGB(vTAcro[visorNow->type]);
        } else if (temp.indexOf("set") > 0) {
          temp.remove(0,4);
          if(cfg.oledEna && oledInitDone)
            oled.writeSet(temp.toInt()+1);
        }
      } else if (temp.toInt() > 0 && temp.toInt() <= totalAnims){ //legacy remote reasons
        animToLoad = availAnims[temp.toInt()-1];
      } else {
        for(int i = 0; i < totalAnims; i++) {
          if(temp == availAnims[i].substring(0, availAnims[i].length() - 5)) {
            animToLoad = availAnims[i];
          }
        }
      }
      logPrint("[I] BT Recv.: "+temp);
    };
} chrCallbacks;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
      NimBLEDevice::startAdvertising();
  }
} serverCallbacks;

bool startBLE() {
  std::string stdStr(cfg.wifiName.c_str(), cfg.wifiName.length());
  BLEDevice::init(stdStr);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(&serverCallbacks);

  BLEService *pService = pServer->createService("ffe0");

  pCharacteristic = pService->createCharacteristic("ffe1",
      NIMBLE_PROPERTY::BROADCAST | NIMBLE_PROPERTY::READ  |
      NIMBLE_PROPERTY::NOTIFY    | NIMBLE_PROPERTY::WRITE |
      NIMBLE_PROPERTY::INDICATE
  );
  pCharacteristic->setValue(totalAnims);
  pCharacteristic->setCallbacks(&chrCallbacks);

  pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->setName(stdStr);
  pAdvertising->addServiceUUID(BLEUUID(pService->getUUID()));
  pAdvertising->enableScanResponse(true);
  if(!pAdvertising->start(0)) {
    return false;
  }
  return true;
}

//--------------------------------//WiFi server setup
void startWiFiWeb() {
  WiFi.softAP(cfg.wifiName, cfg.wifiPass);

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.on("/getfiles", HTTP_GET, [](AsyncWebServerRequest *request){ //returns current anim + all avaible anims
    if(getfilesProper) {
      getFilesFunc();
    }
    request->send(200, "text/plain", currentAnim+";"+getfilesCache);
  });

  server.on("/saveconfig", HTTP_GET, [](AsyncWebServerRequest *request){ //saves config
    //features enable
    cfg.getBool(request, "boopEna", cfg.boopEna);
    cfg.getBool(request, "speechEna", cfg.speechEna);
    cfg.getBool(request, "tiltEna", cfg.tiltEna);
    cfg.getBool(request, "bleEna", cfg.bleEna);
    cfg.getBool(request, "oledEna", cfg.oledEna);
    if(cfg.getBool(request, "oledFlip", cfg.oledFlip)) {
        if(cfg.oledEna && oledInitDone)
            oled.setFlip(cfg.oledFlip);
    }
    cfg.getInt(request, "oledAnimMode", cfg.oledAnimMode); //0=text, 1=animation, 2=animation+text
    String visTypeStr, visTypeMouthStr;
    if(cfg.getString(request, "visTypeStr", visTypeStr) && cfg.getString(request, "visTypeMouthStr", visTypeMouthStr) && visTypeStr.length() > 0 && visTypeMouthStr.length() > 0) {
      File vtFile = LittleFS.open("/visType.txt", "w");
      if(vtFile) { vtFile.print(visTypeStr); vtFile.close(); }
      File vmFile = LittleFS.open("/visTypeMouth.txt", "w");
      if(vmFile) { vmFile.print(visTypeMouthStr); vmFile.close(); }
      loadVisLayout(); //re-parse immediately, no reboot needed
    }
    cfg.getBool(request, "inaEna", cfg.inaEna); //actual (re-)init happens in loop() via inaInitDone retry, no crash risk from reading before begin()
    //brightness
    cfg.getInt(request, "bEar", cfg.bEar);
    cfg.getInt(request, "bVisor", cfg.bVisor);
    if(cfg.getInt(request, "bOled", cfg.bOled)) {
        if(cfg.oledEna && oledInitDone)
            oled.oledBright(cfg.bOled);
    }
    //animation settings
    cfg.getInt(request, "rbSpeed", cfg.rbSpeed);
    cfg.getInt(request, "rbWidth", cfg.rbWidth);
    cfg.getInt(request, "spMin", cfg.spMin);
    cfg.getInt(request, "spMax", cfg.spMax);
    cfg.getInt(request, "spTrig", cfg.spTrig);
    //tilt animations
    cfg.getString(request, "aTilt", cfg.aTilt);
    cfg.getString(request, "aUp", cfg.aUp);
    cfg.getString(request, "aBoop", cfg.aBoop);
    //tilt neutral
    cfg.getFloat(request, "neutralX", cfg.neutralX);
    cfg.getFloat(request, "neutralY", cfg.neutralY);
    cfg.getFloat(request, "neutralZ", cfg.neutralZ);
    //tilt triggers
    cfg.getFloat(request, "tiltX", cfg.tiltX);
    cfg.getFloat(request, "tiltY", cfg.tiltY);
    cfg.getFloat(request, "tiltZ", cfg.tiltZ);
    cfg.getFloat(request, "upX", cfg.upX);
    cfg.getFloat(request, "upY", cfg.upY);
    cfg.getFloat(request, "upZ", cfg.upZ);
    cfg.getFloat(request, "tiltTol", cfg.tiltTol);
    //RGB visor color
    if(cfg.getString(request, "visColor", cfg.visColorStr))
        cfg.visColor = strtol(cfg.visColorStr.c_str() + 1, NULL, 16);
    //wifi: only accept what WiFi.softAP() can start with (an empty password would also make it an open network), otherwise keep the old ones.
    //softAP() only runs at boot, so a change is applied by a restart after the response
    String newName = cfg.wifiName, newPass = cfg.wifiPass;
    cfg.getString(request, "wifiName", newName);
    cfg.getString(request, "wifiPass", newPass);
    bool wifiValid = newName.length() >= 1 && newName.length() <= 32 && newPass.length() >= 8 && newPass.length() <= 63;
    bool wifiChanged = wifiValid && (newName != cfg.wifiName || newPass != cfg.wifiPass);
    if(wifiValid) {
      cfg.wifiName = newName;
      cfg.wifiPass = newPass;
    } else {
      logPrint(F("[E] Invalid WiFi SSID (1-32 characters) or password (8-63 characters), keeping the old ones"));
    }
    //boop threshold
    cfg.getInt(request, "boopThresh", cfg.boopThresh);
    //startup message lines: one character per matrix, and only printable ASCII (the visor font has no UTF-8), shown at the next boot
    String *msgLines[] = {&cfg.startMsg1, &cfg.startMsg2};
    const char *msgParams[] = {"startMsg1", "startMsg2"};
    for(uint8_t i = 0; i < 2; i++) {
      String msg;
      if(cfg.getString(request, msgParams[i], msg)) {
        String clean;
        for(uint16_t c = 0; c < msg.length() && clean.length() < MATRIXESNUM; c++) {
          if(msg[c] >= ' ' && msg[c] <= '~') { clean += msg[c]; }
        }
        *msgLines[i] = clean;
      }
    }
    delay(25);
    if(cfg.save()) {
      instantReload = true;
      if(wifiChanged) {
        logPrint(F("[I] WiFi SSID/password changed, restarting to apply"));
        restartAt = millis() + 2000;
        request->redirect("/saved.html?main&restart");
      } else {
        request->redirect(wifiValid ? "/saved.html?main" : "/saved.html?main&wifiinvalid");
      }
    } else {
      request->send(200, "text/plain", F("Saving config failed!"));
    }
  });

  server.on("/savefile", HTTP_POST, [](AsyncWebServerRequest *request){ //saves data from POST to file
    if(request->hasParam("file", true) && request->hasParam("content", true)) {
      File file = LittleFS.open("/anims/"+request->getParam("file", true)->value()+".json", "w");
      if (!file) {
        logPrint(F("[E] There was an error opening the file for saving an animation!"));
        file.close();
        request->send(200, "text/plain", F("Error opening file for writing!"));
      } else {
        logPrint(F("[I] File saved!"));
        file.print(request->getParam("content", true)->value());
        file.close();
        getfilesProper = true;
        request->redirect("/saved.html?anim");
      }
    } else {
      request->send(200, "text/plain", F("No valid parameters detected!"));
    }
  });

  server.on("/deletefile", HTTP_GET, [](AsyncWebServerRequest *request){ //deletes asked file
    if(request->hasParam("file")) {
      LittleFS.remove("/anims/"+request->getParam("file")->value());
      getfilesProper = true;
      request->redirect("/saved.html?main");
    } else {
      request->send(200, "text/plain", F("Parameter 'file' not present!"));
    }
  });

  server.on("/change", HTTP_GET, [](AsyncWebServerRequest *request){ //loads anim from selected avaible anims
    if(request->hasParam("anim")) {
      if(request->getParam("anim")->value() == currentAnim) {
        request->redirect("/saved.html?main");
      } else if(loadAnim(request->getParam("anim")->value(),"")) {
        request->redirect("/saved.html?main");
      } else {
        request->send(200, "text/plain", F("Loading animation has failed!"));
      }
    } else {
      request->send(200, "text/plain", F("No valid parameters detected!"));
    }
  });

  server.on("/change", HTTP_POST, [](AsyncWebServerRequest *request){ //loads anim from POST request
    if(request->hasParam("anim", true)) {
      if(loadAnim("POSTAnimLoad",request->getParam("anim", true)->value())) {
        request->redirect("/saved.html?main");
      } else {
        request->send(200, "text/plain", F("Loading animation has failed!"));
      }
    } else {
      request->send(200, "text/plain", F("No valid parameters detected!"));
    }
  });

  server.on("/heap", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });

  server.on("/fanpwm", HTTP_GET, [](AsyncWebServerRequest *request){
    if(request->hasParam("duty")) {
      int duty = request->getParam("duty")->value().toInt();
      if(duty < 256 && duty >= 0) {
        ledcWrite(0, duty); //Arduino 2.x core (official espressif32 platform is pinned to arduino-esp32 2.0.17)
        //ledcWrite(fanPWM, duty); //Arduino 3.x core
        cfg.fanDuty = duty;
        cfg.save();
        request->send(200, "text/plain", "Set PWM to: " + String(duty));
      } else {
        request->send(200, "text/plain", F("Invalid duty cycle"));
      }
    } else {
      request->send(200, "text/plain", F("No valid parameters detected!"));
    }
  });
  
  server.on("/rgb", HTTP_GET, [](AsyncWebServerRequest *request){
    if(visorType == "WS2812") {
      visorNow->type++;
      if(visorNow->type == visTypeSize)
        visorNow->type = 0;
      if(cfg.oledEna && oledInitDone)
        oled.writeRGB(vTAcro[visorNow->type]);
        logPrint("[I] Changing visor type to: "+visorTypes[visorNow->type]);
    }
    request->redirect("/saved.html?main");
  });

  server.on("/gyro", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", String(floor(myIMU.readFloatAccelX()*100)/100)+";"+String(floor(myIMU.readFloatAccelY()*100)/100)+";"+String(floor(myIMU.readFloatAccelZ()*100)/100));
  });

  server.on("/tof", HTTP_GET, [](AsyncWebServerRequest *request){
    if(!ToFInitDone) {
      request->send(200, "text/plain", "ToF not initialized!");
    }
    if (boopMode == "APDS9960") {
        request->send(200, "text/plain", String(255 - apds.readProximity()));
    } else if (boopMode == "VL53L1X") {
      if (vl53.dataReady()) {
        request->send(200, "text/plain", String(vl53.distance()));
      }
      request->send(200, "text/plain", "Data not ready!");
    }
  });
  
  server.on("/log", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", logBuffer);
  });

  server.onNotFound([](AsyncWebServerRequest *request){request->send(404, "text/plain", "Not found");});
  ElegantOTA.begin(&server);
  server.begin();

  ElegantOTA.setAutoReboot(true);
}

//--------------------------------//LED output
//Onboard WS2812, bit-banged instead of being a 3rd FastLED controller: on the S3 FastLED's RMT driver only runs 2 controllers at once (FASTLED_RMT_MEM_BLOCKS 2
//of the 4 channels), so a 3rd one gets started from inside the RMT interrupt (doneOnChannel -> startNext), which calls flash-resident IDF code and panics
//("Cache disabled but cached memory region accessed") whenever a LittleFS read (e.g. the web UI) has the flash cache disabled at that moment.
//A single pixel is only 24 bits = 30us with interrupts off. Always called from the loop task, same as FastLED, so it never overlaps an RMT transmission.
//The pin is driven through an S3 "dedicated GPIO" channel (a CPU instruction, ee.wr_mask_gpio_out) instead of GPIO_OUT_W1TS/W1TC register writes: those go
//over the peripheral bus, which can stall while the other core is busy (WiFi, web server, PSRAM). A stretched 0-bit reads as a 1 and flashes the red/green subpixel.
//The bundle only works on the core that created it, so initOnboardLed() must run from setup() (loop task, same core as loop()).
#include "driver/dedic_gpio.h"
#include "hal/cpu_ll.h"
static dedic_gpio_bundle_handle_t onboardBundle = nullptr;
static uint32_t onboardMask = 0;
static portMUX_TYPE onboardMux = portMUX_INITIALIZER_UNLOCKED;
static inline uint32_t IRAM_ATTR cycleCount() { uint32_t c; __asm__ __volatile__("rsr %0, ccount" : "=a"(c)); return c; }

void initOnboardLed() {
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, LOW);
  const int pins[] = {ONBOARD_LED_PIN};
  dedic_gpio_bundle_config_t conf = {};
  conf.gpio_array = pins;
  conf.array_size = 1;
  conf.flags.out_en = 1;
  if(dedic_gpio_new_bundle(&conf, &onboardBundle) == ESP_OK) {
    dedic_gpio_get_out_mask(onboardBundle, &onboardMask);
  } else {
    onboardBundle = nullptr; //showOnboardLed() falls back to plain register writes
  }
}

void IRAM_ATTR showOnboardLed() {
  const uint8_t bytes[3] = {scale8(onboardLed[0].g, onboardBrightness), scale8(onboardLed[0].r, onboardBrightness), scale8(onboardLed[0].b, onboardBrightness)}; //GRB, uncorrected
  const uint32_t mhz = getCpuFrequencyMhz();
  //0.3us/0.75us high, 1.25us per bit: middle of the WS2812-2020 spec (T0H 0.22-0.38us, T1H 0.58-1us) that small onboard LEDs use. The usual 5050 values
  //(0.4/0.8us) are outside it, so a 0-bit with any extra delay read as a 1 and flashed the red/green subpixel. FastLED itself uses 0.25/0.875us.
  const uint32_t t0h = mhz*300/1000, t1h = mhz*750/1000, period = mhz*1250/1000;
  const bool dedic = onboardBundle != nullptr;
  const uint32_t pinMask = 1UL << ONBOARD_LED_PIN;
  portENTER_CRITICAL(&onboardMux);
  uint32_t t = cycleCount();
  for(uint8_t i = 0; i < 3; i++) {
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      const uint32_t high = (bytes[i] & bit) ? t1h : t0h;
      while(cycleCount() - t < period) {}
      t = cycleCount();
      if(dedic) { cpu_ll_write_dedic_gpio_mask(onboardMask, onboardMask); } else { REG_WRITE(GPIO_OUT_W1TS_REG, pinMask); }
      while(cycleCount() - t < high) {}
      if(dedic) { cpu_ll_write_dedic_gpio_mask(onboardMask, 0); } else { REG_WRITE(GPIO_OUT_W1TC_REG, pinMask); }
    }
  }
  portEXIT_CRITICAL(&onboardMux);
}

//Breathes the onboard LED from fully off to HEARTBEAT_MAX. Called from loop(), and from the startup message waits so it already breathes right after the POST.
void updateHeartbeat() {
  static unsigned long lastHeartbeat = 0;
  if(lastHeartbeat+20 < millis()) {
    lastHeartbeat = millis();
    uint8_t level = beatsin8(HEARTBEAT_BPM, 0, HEARTBEAT_MAX);
    if(level != onboardBrightness || onboardLed[0] != CRGB(HEARTBEAT_COLOR)) {
      onboardLed[0] = HEARTBEAT_COLOR;
      onboardBrightness = level;
      showOnboardLed(); //separate from FastLED, so no need to resend the visor/ears
    }
  }
}

//FastLED's ESP32 RMT driver (idf4_rmt_impl.cpp) only transmits once showLeds() has been called on *every* registered controller, then sends all of them together.
//Calling showLeds() on just one controller only latches its data until the others are called too, so frames get dropped/merged. Always push every controller at once.
//Unchanged outputs are sent as just their first LED (setLeds(..., 1), ~30us) instead of skipped, since the batch needs every controller: the rest of that chain keeps
//its data and nothing passes through it. Before this, e.g. rainbow ears (every rbSpeed) resent all 704 visor LEDs each time, blocking loop() ~21ms per push and
//giving the marginal last visor panels a chance to corrupt the data passing through them (tail flicker, see the BOOT button self test) dozens of times a second.
void showAllLeds() {
  static CRGB sentVisor[MATRIXESNUM*64], sentEars[earLedsNum+blushLedsNum];
  static int sentBVisor = -1, sentBEar = -1; //-1: nothing sent yet
  bool visorChanged = cfg.bVisor != sentBVisor || memcmp(visorLeds, sentVisor, sizeof(sentVisor)) != 0;
  bool earsChanged = cfg.bEar != sentBEar || memcmp(c2Leds, sentEars, sizeof(sentEars)) != 0;

  if(visorChanged || earsChanged) {
    if(ledController[0]) { //only registered for WS2812 visors
      if(!visorChanged) { ledController[0]->setLeds(visorLeds, 1); }
      ledController[0]->showLeds(cfg.bVisor);
      ledController[0]->setLeds(visorLeds, visorLedsNum);
    }
    if(ledController[1]) { //only registered with ears and/or blush
      if(!earsChanged) { ledController[1]->setLeds(c2Leds, 1); }
      ledController[1]->showLeds(cfg.bEar);
      ledController[1]->setLeds(c2Leds, earLedsNum+blushLedsNum);
    }
    memcpy(sentVisor, visorLeds, sizeof(sentVisor));
    memcpy(sentEars, c2Leds, sizeof(sentEars));
    sentBVisor = cfg.bVisor;
    sentBEar = cfg.bEar;
  }
  showOnboardLed();
}

//--------------------------------//POST
//c2Leds index -> is that pixel on the blush strip? (same ear-blush-ear chaining as the render code in loop())
bool isBlushPixel(uint16_t i) {
  if(blushPresent && earPresent) { return i >= (earLedsNum/2) && i < (earLedsNum/2)+blushLedsNum; }
  return blushPresent;
}

//Blush color as it has to be sent: red-green swapped for an RGB-ordered strip (useRGBblush). Applied to the copy going into c2Leds, never to blushLeds itself:
//loop() used to swap blushLeds in place on every display pass, so between blush frame changes it flipped back and forth (e.g. purple <-> teal with animated ears).
CRGB blushOut(CRGB c) {
  if(useRGBblush) { uint8_t t = c.r; c.r = c.g; c.g = t; }
  return c;
}

//Walks a lit pixel through every visor matrix in parallel (one per matrix, so 64 steps for the whole visor) and one down the ears/blush chain at the same
//time, once per color, so every pixel can be checked by eye. The ears/blush chain is a separate data line, so it's still obvious where a dead pixel is.
void runPOST() {
  const CRGB colors[] = {CRGB::White, CRGB::Red, CRGB::Lime, CRGB::Blue}; //Lime = full green (0x00FF00); FastLED's CRGB::Green is HTML green (0x008000), only half
  const char *names[] = {"white", "red", "green", "blue"};
  bool doVisor = visorType == "WS2812";
  bool doEars = earPresent || blushPresent;
  uint16_t c2Num = (earPresent ? earLedsNum : 0) + (blushPresent ? blushLedsNum : 0);
  uint16_t steps = max(doVisor ? (uint16_t)64 : (uint16_t)0, doEars ? c2Num : (uint16_t)0);

  logPrint(F("[I] POST: LED walk test start"));
  for(uint8_t c = 0; c < 4; c++) {
    logPrint("[I] POST: walking " + String(names[c]));
    onboardBrightness = HEARTBEAT_MAX;
    for(uint16_t pos = 0; pos < steps; pos++) {
      onboardLed[0] = ((millis() / POST_BLINK_MS) % 2) ? CRGB::Black : colors[c]; //onboard LED blinks the current walk color, so it's clearly distinct from the steady startup/heartbeat colors
      if(doVisor && pos < 64) { for(uint8_t m = 0; m < MATRIXESNUM; m++) { visorLeds[(m*64)+pos] = colors[c]; } }
      if(doEars && pos < c2Num) {
        c2Leds[pos] = colors[c];
        if(isBlushPixel(pos)) { c2Leds[pos] = blushOut(c2Leds[pos]); } //same red-green handling as loop()
      }
      showAllLeds();
      if(doVisor && pos < 64) { for(uint8_t m = 0; m < MATRIXESNUM; m++) { visorLeds[(m*64)+pos] = CRGB::Black; } }
      if(doEars && pos < c2Num) { c2Leds[pos] = CRGB::Black; }
      delay(POST_STEP_MS);
    }
  }
  onboardLed[0] = HEARTBEAT_COLOR; //steady heartbeat color while WiFi/BLE/sensors init (blocking, can't breathe), updateHeartbeat() takes over from the startup messages on
  onboardBrightness = HEARTBEAT_MAX;
  showAllLeds(); //last pixel off
  logPrint(F("[I] POST: LED walk test done"));
}

//--------------------------------//Startup message
extern const uint8_t PROGMEM _sysfont[]; //MD_MAX72XX's built-in 5x7 font (declared in its internal MD_MAX72xx_lib.h)

//Same LED index mapping as setAllVisor(): seg = matrix in the chain, row/col = 0..7 from the top-left
uint16_t visorIndex(uint8_t seg, uint8_t row, uint8_t col) {
  return (seg*64)+(row*8)+((oldMatrixFix && row%2==0)?7-col:col);
}

//Modules that failed to init during setup(), shown on the visor instead of STARTUP_OK_MSG
const char *initErrors[8];
uint8_t initErrorNum = 0;
void addInitError(const char *name) {
  if(initErrorNum < 8) { initErrors[initErrorNum++] = name; }
}

//Shows one line for STARTUP_MSG_MS, one character per matrix in chain order (matrix 0 = first character), each character centered on its matrix.
//Lines shorter than MATRIXESNUM are centered across the chain; pad with spaces to place them yourself.
void showMsgLine(const char *msg, CRGB color) {
  uint8_t len = min(strlen(msg), (size_t)MATRIXESNUM);
  if(len == 0) { return; }
  uint8_t first = (MATRIXESNUM - len) / 2;
  fill_solid(visorLeds, visorLedsNum, CRGB::Black);
  for(uint8_t k = 0; k < len; k++) {
    uint8_t cols[8];
    uint8_t width = mx.getChar((uint8_t)msg[k], sizeof(cols), cols); //one byte per column, bit 0 = top row
    uint8_t x0 = (8 - width) / 2;
    for(uint8_t x = 0; x < width; x++) {
      for(uint8_t y = 0; y < 8; y++) {
        if(bitRead(cols[x], y)) { visorLeds[visorIndex(first+k, y, x0+x)] = color; }
      }
    }
  }
  logPrint("[I] Startup message: " + String(msg));
  showAllLeds();
  for(unsigned long start = millis(); millis() - start < STARTUP_MSG_MS; ) { updateHeartbeat(); delay(5); }
}

//Runs at the end of setup(), once every module has been initialized: STARTUP_OK_MSG in green if all of them came up, otherwise one red line per failed module,
//then the startup message lines from the config (web UI) in the visor color (cfg.visColor), empty lines skipped. The last line stays up until the first animation frame replaces (fades over) it.
void showStartupMsg() {
  const String lines[] = {cfg.startMsg1, cfg.startMsg2};
  if(visorType != "WS2812") { return; }

  mx.setFont(_sysfont); //normally set by mx.begin(), which only runs for MAX72XX visors; getChar() itself needs no MAX72XX hardware
  if(initErrorNum == 0) {
    showMsgLine(STARTUP_OK_MSG, CRGB::Lime); //full green, see runPOST()
  } else {
    for(uint8_t i = 0; i < initErrorNum; i++) { showMsgLine(initErrors[i], CRGB::Red); }
  }
  CRGB color = cfg.visColor ? CRGB((uint32_t)cfg.visColor) : CRGB::White; //same color as the visor
  for(const String &msg : lines) { showMsgLine(msg.c_str(), color); }
}

//--------------------------------//Setup
void setup() {
  //onboard LED on first thing, so you can see the board is powered/booting before LittleFS/PSRAM/config (and the POST) even start
  initOnboardLed();
  onboardLed[0] = CRGB::Blue; //startup color, until the POST takes over
  onboardBrightness = HEARTBEAT_MAX;
  showOnboardLed();

  Serial.begin(115200);
  while(!Serial && millis() < 3000) { delay(10); } //native USB CDC re-enumerates after reset; give the serial monitor a moment to reconnect so early logs (POST) aren't lost. Boots on without a host after 3s

  pinMode(animBtn, INPUT_PULLUP);
  pinMode(0, INPUT_PULLUP);
  pinMode(fanPWM, OUTPUT);

  if(boopMode == "KY-032") {
    pinMode(T_in, INPUT_PULLUP);
    pinMode(T_en, OUTPUT);
  } else if ((boopMode == "Capac")) {
    pinMode(T_in, INPUT_PULLDOWN);
  }

  hwBtn.setDebounceTime(50);

  if(!LittleFS.begin(true)) {
    logPrint(F("[E] An Error has occurred while mounting LittleFS! Halting"));
    while(1){};
  }

  loadVisLayout();

  if(psramInit() && ESP.getFreePsram() != 0) {
    earsNow  = (AnimNowEars *)  ps_calloc(1, sizeof(AnimNowEars));
    visorNow = (AnimNowVisor *) ps_calloc(1, sizeof(AnimNowVisor));
    logBuffer = (char*) ps_malloc(LOG_BUFFER_SIZE);
    logIndex = 0;
    logBuffer[0] = '\0';
  } else {
    logPrint(F("[E] Could not init PSRAM, either this ESP doesn't have one or is malfunctioning, halting..."));
    while(1){};
  }

  if(!cfg.load()) {
    logPrint(F("[E] An Error has occurred while loading config file! Loading defaults"));
    addInitError("CFG ERR");
    cfg.setDefault();
  }

  micDC = (float)cfg.spMin;

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(MICpin, ADC_ATTEN_DB_12); //same attenuation as the old ADC_ATTEN_DB_11 name, just renamed

  //ledcAttach(fanPWM, 25000, 8); //support Arduino 3.x
  //ledcWrite(fanPWM, cfg.fanDuty); //Arduino 3.x core
  ledcSetup(0, 25000, 8); //For Arduino 2.x
  ledcAttachPin(fanPWM, 0); //For Arduino 2.x
  ledcWrite(0, cfg.fanDuty); //for Arduino 2.x

  if(visorType == "WS2812") {
    ledController[0] = &FastLED.addLeds<WS2812B, DATA_PIN_VISOR, GRB>(visorLeds, visorLedsNum);
  } else if (visorType == "MAX72XX") {
    mx.begin();
  }
  if(earPresent || blushPresent) { //RMT4 2 controller fix
    ledController[1] = &FastLED.addLeds<WS2812B, DATA_PIN_EARS, GRB>(c2Leds, earLedsNum+blushLedsNum);
  }
  /*if(earPresent) {
    ledController[1] = &FastLED.addLeds<WS2812B, DATA_PIN_EARS, GRB>(earLeds, earLedsNum);
  }
  if(blushPresent) {
    ledController[2] = &FastLED.addLeds<WS2812B, DATA_PIN_BLUSH, RGB>(blushLeds, blushLedsNum);
  }*/
  FastLED.setCorrection(TypicalPixelString); //not applied to the onboard LED (showOnboardLed() is uncorrected): it would scale blue to 140/255, leaving only ~4 visible heartbeat steps at HEARTBEAT_MAX 8
  FastLED.setDither(0);

#if POST_ENABLED
  runPOST();
#endif

  startWiFiWeb();

  getFilesFunc();
  if(cfg.bleEna) { //you can disable BLE in config
    if(!startBLE()) {
      logPrint(F("[E] An Error has occurred while starting BLE!"));
      addInitError("BLE ERR");
    }
  }

  //I2C things
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();
  Wire.setClock(I2C_CLOCK); //100k = 113ms; 400k = 33ms; (800k = 20ms; 1mhz / 2mhz = 17ms = breaks apds)
  String i2cFound; //boot-time bus scan, so the log shows which devices actually ACK (OLED 0x3C, APDS9960 0x39, VL53L1X 0x29, INA219 0x40, LSM6DS3 0x6A/0x6B)
  for(uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if(Wire.endTransmission() == 0) i2cFound += " 0x"+String(addr,HEX);
  }
  logPrint("[I] I2C devices found:"+(i2cFound.length() ? i2cFound : String(" none"))+" @ "+String(Wire.getClock()/1000)+"kHz");

  if(cfg.tiltEna) {
    if(myIMU.begin()) {
      logPrint(F("[E] An Error has occurred while initializing LSM!"));
      addInitError("TILT ERR");
      cfg.tiltEna = false;
    } else {
      tiltInitDone = true;
    }
  }

  if(cfg.oledEna) {
    if(!oled.init(oledAddr,cfg.bOled,cfg.inaEna,cfg.oledFlip)) {
      logPrint(F("[E] An Error has occurred while initializing SSD1306!"));
      addInitError("OLED ERR");
      cfg.oledEna = false;
    } else {
      oledInitDone = true;
      oled.speak(false);
      oled.writeSet(1);
    }
  }

  if(cfg.inaEna && oledInitDone) {
    if(!ina219.begin()) {
      logPrint(F("[E] An Error has occurred while initializing INA219 chip!"));
      addInitError("INA ERR");
      cfg.inaEna = false;
    } else {
      ina219.setCalibration_16V_8A();
      inaInitDone = true;
    }
  }

  if(boopMode == "APDS9960" && cfg.boopEna) {
    if(!apds.begin()){
      cfg.boopEna = false;
      logPrint("[E] An Error has occurred while initializing APDS9960 chip! (raw ID reg: 0x"+String(apdsRawID(),HEX)+")");
      addInitError("BOOP ERR");
    } else {
      ToFInitDone = true;
      apds.enableProximity(true);
      apds.setProxPulse(APDS9960_PPULSELEN_8US, 8);
    }
  } else if (boopMode == "VL53L1X" && cfg.boopEna) {
    Wire.beginTransmission(0x29);
    if(Wire.endTransmission() != 0) {
      cfg.boopEna = false;
      logPrint(F("[E] An Error has occurred while finding VL53L1X chip!"));
    } else {
      if(!vl53.begin()) {
        cfg.boopEna = false;
        logPrint(F("[E] An Error has occurred while initializing VL53L1X chip!"));
      } else {
        if (!vl53.startRanging()) {
          cfg.boopEna = false;
          logPrint(F("[E] An Error has occurred while starting ranging with VL53L1X chip!"));
        } else {
          ToFInitDone = true;
          vl53.setTimingBudget(50);
        }
      }
    }
    if(!ToFInitDone) { addInitError("BOOP ERR"); } //any of the three VL53L1X failures above
  }

  showStartupMsg(); //last, so "PROTOGEN OK" (or the failures) reflect every module above

  while(millis()<2000) {yield();} //2s delay for the anim to load properly (idk why but it doesnt without this or with 1s)
  loadAnim("default.json","");

  logPrint("[I] Free heap: "+String(ESP.getFreeHeap()));
  logPrint("[I] Free PSRAM: "+String(ESP.getFreePsram()));
}

//--------------------------------//Loop vars
String oldanim, boopoldanim;
bool FdisplayVisor = false, FdisplayBlush = false, FdisplayEar = false, booping = false, wasTilt = false, boopRea = false, remoteSign = false, speaking = true;
float zAx,yAx,finalMicAvg,avgMicArr[10], micAttack = 0.35f, micRelease = 0.2f, env = 0.0f;
int boopRead, startIndex = 1, micVolume, currentMicAvg = 0, btnNum = 0, currFade = 1, apdsprox = 255, apdsLastProx = -1;
unsigned long lastMillsEars = 0, lastMillsVisor = 0, lastMillsTilt = 0, laskSpeakCheck = 0, lastMillsBoop = 0, lastFLED = 0, lastFLEDvisor = 0, vaStatLast = 0, btnPressTime = 0, tiltChange = 0, check0button = 0, looptime = 0, fadeTime = 0, laskSpeakAnim = 0, lastBoopCheck = 0, lastOledThumbUpdate = 0, lastProxDebug = 0, apdsLastChange = 0;

void dynamicSpeak(uint64_t *leds, long colors[MATRIXESNUM][64], bool isMouth[MATRIXESNUM], int volume) {
  int mouthIndexes[MATRIXESNUM];
  int mouthCount = 0;
  for (int i = 0; i < MATRIXESNUM; i++) { // collect true (mouth) indexes
    if (isMouth[i]) {
      mouthIndexes[mouthCount++] = i;
    }
  }
  int half = mouthCount / 2;
  for (int i = 0; i < half; i++) {
    int x = map(volume, 0, 100, 0, half * 8) - (i * 8);
    int leftIndex  = mouthIndexes[half - 1 - i];
    int rightIndex = mouthIndexes[half + i];
    if (x > 0) {
      x = constrain(x, 0, 8);
      static long colorL[8][8], colorR[8][8]; //static: keep off the loop task's stack (only called from the single main loop task, never reentrant)
      for(int a = 0; a < 8; a++) {
        for(int b = 0; b < 8; b++) {
          colorL[a][b] = colors[leftIndex][(a*8)+b];
          colorR[a][b] = colors[rightIndex][(a*8)+b];
        }
      }
      leds[leftIndex]  = speakMatrix(leds[leftIndex],  x, true,  colorL);
      leds[rightIndex] = speakMatrix(leds[rightIndex], x, false, colorR);
      for(int a = 0; a < 8; a++) {
        for(int b = 0; b < 8; b++) {
          colors[leftIndex][(a*8)+b]  = colorL[a][b];
          colors[rightIndex][(a*8)+b] = colorR[a][b];
        }
      }
    }
  }
}

void setAllVisor(struct CRGB *ledArray, long ledColor, int visorFrame) {
  uint64_t tempLeds[MATRIXESNUM];
  memcpy(tempLeds, visorNow->frames[visorFrame].leds, sizeof(tempLeds));
  static long tempColors[MATRIXESNUM][64]; //static: ~2.8KB is too much to put on the loop task's 8KB stack on every call
  memcpy(tempColors, visorNow->frames[visorFrame].ppColor, sizeof(tempColors));
  if(speaking) {
    dynamicSpeak(tempLeds, tempColors, visorNow->isMouth, micVolume);
  }
  for(int y = 0; y < numOfSegm; y++) {
    for (int i = 0; i < 8; i++) {
      byte row = (tempLeds[y] >> i * 8) & 0xFF; //---------remove byte from upper global
      for (int j = 0; j < 8; j++) {
        if(visorType == "WS2812") {
          long tempColor = ledColor; //use given color
          if(ledColor == 0) { //if not given a color
            if(tempColors[y][(i*8)+j] != 0) { //use ppColor if available (speak-opened pixels inherit the nearest lit pixel's color, see dynamicSpeak)
              tempColor = tempColors[y][(i*8)+j];
            } else if(visorNow->frames[visorFrame].fColor[y] != 0) { //if not, use fColor if available
              tempColor = visorNow->frames[visorFrame].fColor[y];
            } else { // else config color
              tempColor = cfg.visColor;
            }
          }
          if(oldMatrixFix) {
            ledArray[(y*64)+(i*8)+((i%2!=0)?j:7-j)] = (bitRead(row,j))?tempColor:CRGB::Black; //includes fix for bad rgbmatrix
          } else {
            ledArray[(y*64)+(i*8)+j] = (bitRead(row,j))?tempColor:CRGB::Black;
          }
        } else if (visorType == "MAX72XX") {
          mx.setPoint(i, j+(y*8), bitRead(row, j)); //MAXstuff
        }
      }
    }
  }
  FdisplayVisor = true;

  //live-mirror the OLED thumbnail from the exact bitmask just rendered (post-dynamicSpeak overlay, so mouth flap is included).
  //Throttled: this function isn't uniformly rate-limited by its callers (the WS2812 rainbow branch calls it every loop() tick),
  //so without this the OLED would get hammered over I2C. 60ms matches the existing speaking-redraw cadence used for this same effect elsewhere.
  if(cfg.oledEna && oledInitDone && cfg.oledAnimMode && visLayoutAvail && lastOledThumbUpdate+60<=millis()) {
    oled.writeVisorThumb(tempLeds, segGridRow, segGridCol, numOfSegm, gridRows, gridCols, cfg.oledAnimMode == 2, currentAnim.substring(0, currentAnim.length()-5));
    lastOledThumbUpdate = millis();
  }
}

//--------------------------------//LED self test (BOOT button)
//A short press (<1s) of the board's BOOT button (GPIO0) cycles: off -> A (every visor/ears/blush LED lit once, then nothing is sent) -> B (same frame, resent
//continuously) -> off (back to the animation). Flicker in A can't be the data (nothing is transmitted), so it's power/hardware; flicker only in B is the transfer.
//Holding BOOT for 10s still resets the config (that check runs at the end of loop(), which is skipped while a test runs, so exit the test first).
#define SELFTEST_BRIGHTNESS 8
#define SELFTEST_COLOR CRGB::White //all subpixels: roughly 1.2A at 8/255 on top of the idle draw (CRGB::Blue would be ~0.3A, e.g. on USB power)
uint8_t selfTestMode = 0;

void showSelfTest() {
  fill_solid(visorLeds, visorLedsNum, SELFTEST_COLOR);
  fill_solid(c2Leds, earLedsNum+blushLedsNum, SELFTEST_COLOR);
  if(ledController[0]) { ledController[0]->showLeds(SELFTEST_BRIGHTNESS); }
  if(ledController[1]) { ledController[1]->showLeds(SELFTEST_BRIGHTNESS); }
}

//Returns true while a test is running, so loop() skips the normal rendering
bool handleSelfTest() {
  static unsigned long pressStart = 0;
  static bool wasPressed = false;
  bool pressed = digitalRead(0) == LOW;
  if(pressed && !wasPressed) { pressStart = millis(); }
  if(!pressed && wasPressed && millis() - pressStart >= 50 && millis() - pressStart < 1000) {
    selfTestMode = (selfTestMode + 1) % 3;
    if(selfTestMode == 1) {
      logPrint(F("[I] Self test A: all LEDs lit once, nothing sent afterwards"));
      showSelfTest();
    } else if(selfTestMode == 2) {
      logPrint(F("[I] Self test B: same frame resent continuously"));
    } else {
      logPrint(F("[I] Self test off"));
      instantReload = true; //re-render the current animation (fades over the test frame)
      FdisplayEar = true;
      FdisplayBlush = true;
    }
  }
  wasPressed = pressed;
  if(selfTestMode == 2) { showSelfTest(); }
  return selfTestMode != 0;
}

void loop() {
  ElegantOTA.loop();
  updateHeartbeat(); //onboard LED breathes as long as loop() keeps running
  if(restartAt && millis() > restartAt) { ESP.restart(); } //before the self test check, which skips the rest of loop()
  if(handleSelfTest()) { return; }
  //--------------------------------//EAR Leds render
  if(earPresent) {
    if(earsNow->type == 0) { //custom
      if(lastMillsEars+earsNow->frames[currentEarsFrame].timespan <= millis() || instantReload) {
        currentEarsFrame++;
        lastMillsEars = millis();
        if(currentEarsFrame == earsNow->numOfFrames) { currentEarsFrame = 0; } //loop back to first frame if last frame
        for(int y = 0; y < earLedsNum; y++) { earLeds[y] = earsNow->frames[currentEarsFrame].ledColor[y]; } //set ear leds
        FdisplayEar = true;
      }
    } else if (earsNow->type == 1) { //rainbow
      if(lastFLED+cfg.rbSpeed < millis()) { //throttle: was recomputing/pushing every loop() tick, hammering the shared RMT channel and glitching the tail LED (#82)
        lastFLED = millis();
        fill_rainbow(pixelBuffer, 4, millis()/cfg.rbSpeed, 255/cfg.rbWidth);
        if(earLedsNum == 74) {
          for(int x = 0;x<earLedsNum;x++) {
            if(x<16) {
              earLeds[x] = pixelBuffer[0];
              earLeds[x+37] = pixelBuffer[0];
            } else if(x<28) {
              earLeds[x] = pixelBuffer[1];
              earLeds[x+37] = pixelBuffer[1];
            } else if(x<36) {
              earLeds[x] = pixelBuffer[2];
              earLeds[x+37] = pixelBuffer[2];
            } else if(x==36) {
              earLeds[x] = pixelBuffer[3];
              earLeds[x+37] = pixelBuffer[3];
            }
          }
        } else {
          for(int x = 0;x<earLedsNum;x++) {
            earLeds[x] = pixelBuffer[0];
          }
        }
        FdisplayEar = true;
      }
    } else if (earsNow->type == 2) { //white_noise
      if(lastFLED+cfg.rbSpeed < millis()) { //throttle: same unbounded-refresh issue as rainbow above
        lastFLED = millis();
        memset(noiseData, 0, earLedsNum);
        fill_raw_noise8(noiseData, earLedsNum, 2, 0, 50, millis()/4);
        for(int x = 0;x<earLedsNum;x++) {
          earLeds[x] = ColorFromPalette(blackWhite, noiseData[x]);
        }
        FdisplayEar = true;
      }
    } else if (earsNow->type == 3 && earLedsNum == 74) { //corner_sabers - only 74 led mode
      if(lastFLED+cfg.rbSpeed < millis()) {
        lastFLED = millis();
        startIndex++;
        int tempIndex = startIndex;
        for(int x = 0;x<18;x++) {
          pixelBuffer[x] = ColorFromPalette(RainbowStripeColors_p, tempIndex, 255, NOBLEND);
          tempIndex+=3;
        }
        for(int x = 0;x<9;x++) {
          for(int y = 0;y<lookupDiag1[x].size();y++) {
            earLeds[lookupDiag2[x][y]-1] = pixelBuffer[x];
            earLeds[lookupDiag1[x][y]+36] = pixelBuffer[x];
          }
        }
        FdisplayEar = true;
      }
    } else if (earsNow->type == 4) { //custom_glow
      if(lastFLED+cfg.rbSpeed < millis()) { //throttle: same unbounded-refresh issue as rainbow above
        lastFLED = millis();
        fill_rainbow(pixelBuffer, 4, millis()/cfg.rbSpeed, 255/cfg.rbWidth);
        for(int y = 0; y < earLedsNum; y++) {
          if(earsNow->frames[0].ledColor[y] == 0) {
            earLeds[y] = 0x000000;
          } else {
            earLeds[y] = pixelBuffer[0];
          }
        }
        FdisplayEar = true;
      }
    } else if (earsNow->type == 5) {} //none
  }

  //--------------------------------//VISOR+BLUSH Leds render
  if(visorNow->type == 0 || (visorNow->type == 1 && visorType == "MAX72XX")) { //custom
    if(lastMillsVisor+visorNow->frames[currentVisorFrame].timespan <= millis() || instantReload) {
      currentVisorFrame++;
      lastMillsVisor = millis();
      if(currentVisorFrame == visorNow->numOfFrames) { currentVisorFrame = 0; }
      setAllVisor(visorLedsNEW,0,currentVisorFrame); //set visor leds
      if(blushPresent) {
        for(int x = 0; x<blushLedsNum; x++) { blushLeds[x] = visorNow->frames[currentVisorFrame].ledsBlush[x]; } //set blush leds
        FdisplayBlush = true;
      }
      instantReload = false;
    }
  } else if (visorNow->type == 1 && visorType == "WS2812") { //all_rainbow
    if(lastMillsVisor+visorNow->frames[currentVisorFrame].timespan <= millis() || instantReload) {
      currentVisorFrame++;
      lastMillsVisor = millis();
      if(currentVisorFrame == visorNow->numOfFrames) { currentVisorFrame = 0; }
      instantReload = false;
    }
    if(lastFLEDvisor+cfg.rbSpeed < millis()) { //throttle: was recomputing/pushing every loop() tick, same as the ears rainbow fix -- testing whether visor+ears RMT contention is behind the led82 glitch
      lastFLEDvisor = millis();
      fill_rainbow(visorPixelBuffer, 1, millis()/cfg.rbSpeed, 128/cfg.rbWidth);
      setAllVisor(visorLeds,((long)visorPixelBuffer[0].r << 16) | ((long)visorPixelBuffer[0].g << 8 ) | (long)visorPixelBuffer[0].b,currentVisorFrame);
      if(blushPresent) {
        for(int x = 0; x<blushLedsNum; x++) { blushLeds[x] = visorNow->frames[currentVisorFrame].ledsBlush[x]; } //set blush leds
        FdisplayBlush = true;
      }
    }
  }

  //--------------------------------//TILT
  if(lastMillsTilt+100<=millis() && cfg.tiltEna) {
    if(isApproxEqual(myIMU.readFloatAccelX(),myIMU.readFloatAccelY(),myIMU.readFloatAccelZ(),cfg.upX,cfg.upY,cfg.upZ,cfg.tiltTol) && !wasTilt) {
      logPrint(F("[I] Tilt: UP!"));
      wasTilt = true;
      oldanim = currentAnim;
      tiltChange = millis();
      loadAnim(cfg.aUp,"");
    } else if (isApproxEqual(myIMU.readFloatAccelX(),myIMU.readFloatAccelY(),myIMU.readFloatAccelZ(),cfg.tiltX,cfg.tiltY,cfg.tiltZ,cfg.tiltTol) && !wasTilt) {
      logPrint(F("[I] Tilt: Side!"));
      wasTilt = true;
      oldanim = currentAnim;
      tiltChange = millis();
      loadAnim(cfg.aTilt,"");
    } else if ((tiltChange+revertTilt<millis() || isApproxEqual(myIMU.readFloatAccelX(),myIMU.readFloatAccelY(),myIMU.readFloatAccelZ(),cfg.neutralX,cfg.neutralY,cfg.neutralZ,cfg.tiltTol)) && wasTilt) {
      logPrint(F("[I] Tilt: Neutral!"));
      wasTilt = false;
      loadAnim(oldanim,"");
    }
    lastMillsTilt = millis();
  } else if (!tiltInitDone && cfg.tiltEna) {
    if(myIMU.begin()) {
      logPrint(F("[E] An Error has occurred while connecting to LSM!"));
      cfg.tiltEna = false;
    } else {
      tiltInitDone = true;
    }
  }

  //looptime = micros();
  //--------------------------------//SPEECH Detection
  if(cfg.speechEna) { //1.045uS
    int nvol = 0, micline = 0, rawInput = 0;
    for (int i = 0; i<32; i++){
      rawInput = adc1_get_raw(MICpin);
      micline = abs(rawInput - 512);
      nvol = max(micline, nvol);
    }
    if(currentMicAvg == 9) {
      currentMicAvg = 0;
    } else {
      avgMicArr[currentMicAvg++] = nvol;
    }

    if(laskSpeakCheck+10<=millis()) {
      finalMicAvg = 0;
      for (int i = 0; i<10; i++){
        finalMicAvg+=avgMicArr[i];
      }
      finalMicAvg = finalMicAvg/10;
      //Serial.print(String(finalMicAvg));
      //Serial.print(",");
      float centered = finalMicAvg - micDC; //DC removal
      float mag = fabsf(centered);
      if (mag > env) { // Envelope follower
        env += (mag - env) * micAttack;
      } else {
        env += (mag - env) * micRelease;
      }
      if (env < 25.0f) { // Noise gate
        env = 0.0f;
      }
      if (env < 25.0f) { // Update DC only when quiet
          micDC = micDC * (1.0f - 0.0005f) + finalMicAvg * 0.0005f;
      }
      micVolume = (int)(env * 100.0f / (float)cfg.spMax + 0.5f); // Normalize 0-100
      micVolume = constrain(micVolume, 0, 100);
      //Serial.println(String(micVolume));

      if(micVolume > cfg.spTrig) {
        if(!speaking) {
          speaking = true;
          if(cfg.oledEna && oledInitDone) {
            oled.speak(true);
          }
          logPrint(F("[I] Speak"));
        }
      } else {
        if(speaking) {
          logPrint(F("[I] unSpeak"));
          speaking = false;
          if(cfg.oledEna && oledInitDone) {
            oled.speak(false);
          }
          if(visorNow->type == 0 || (visorNow->type == 1 && visorType == "MAX72XX")) {
            setAllVisor(visorLedsNEW,0,currentVisorFrame);
          }
        }
      }

      laskSpeakCheck = millis();
    }

    if(laskSpeakAnim+60<=millis() && speaking) {
      if(visorNow->type == 0 || (visorNow->type == 1 && visorType == "MAX72XX")) { //custom
        setAllVisor(visorLedsNEW,0,currentVisorFrame);
        laskSpeakAnim = millis();
      }
    }
  }
  //Serial.println(">SPK1:"+String(micros()-looptime));

  //--------------------------------//Single button anim change
  hwBtn.loop();
  if(hwBtn.isPressed()) { //detect press
    btnPressTime = millis();
  }
  if(hwBtn.isReleased()) {
    if(millis()-btnPressTime < 1500) { //short press
      btnNum++;
      if(btnNum >= totalAnims) {
        btnNum = 0;
      } else {
        animToLoad = availAnims[btnNum];
      }
      logPrint("Changing to "+availAnims[btnNum]+", amount of anims: "+String(totalAnims));
    }
  }

  //looptime = micros();
  //--------------------------------//BOOP Detection; 14-800uS
  if(lastBoopCheck+100<=millis() && cfg.boopEna) {
    if(boopMode == "KY-032") {
      digitalWrite(T_en, HIGH);
      delayMicroseconds(210);
      if(booping == false && !digitalRead(T_in)) {
        delayMicroseconds(395);
        if(!digitalRead(T_in)) {
          logPrint(F("[I] IR BOOP"));
          booping = true;
          boopoldanim = currentAnim;
          loadAnim(cfg.aBoop,"");
          lastMillsBoop = millis();
        }
        digitalWrite(T_en, LOW);
      } else if(booping == true && lastMillsBoop+1000<millis() && digitalRead(T_in)) {
        digitalWrite(T_en, LOW);
        logPrint(F("[I] IR unBOOP"));
        booping = false;
        if(!wasTilt) {
          loadAnim(boopoldanim,"");
        }
      }
    } else if (boopMode == "Capac") {
      if(booping == false && digitalRead(T_in)) {
        delayMicroseconds(395);
        if(digitalRead(T_in)) {
          logPrint(F("[I] Touch BOOP"));
          booping = true;
          boopoldanim = currentAnim;
          loadAnim(cfg.aBoop,"");
          lastMillsBoop = millis();
        }
      } else if(booping == true && lastMillsBoop+1000<millis() && !digitalRead(T_in)) {
        logPrint(F("[I] Touch unBOOP"));
        booping = false;
        if(!wasTilt) {
          loadAnim(boopoldanim,"");
        }
      }
    } else if (boopMode == "APDS9960") {
      if(ToFInitDone) {
        apdsprox = apds.readProximity();
        apds.clearInterrupt(); //clears the proximity/color saturation latch (PGSAT/CPSAT) every cycle -- left uncleared, a saturation event can wedge some clone chips' proximity engine until reset
        if(lastProxDebug+500<=millis()) { //throttled: readProximity() runs every loop() iteration, would flood the console otherwise
          logDebug("[D] APDS proximity: "+String(apdsprox)+", ToF: "+String(255-apdsprox)); //ToF = 255-proximity, matches the /tof webpage value
          lastProxDebug = millis();
        }
        if(apdsprox != apdsLastProx) {
          apdsLastProx = apdsprox;
          apdsLastChange = millis();
        } else if(millis()-apdsLastChange > 5000) { //reading hasn't moved at all in 5s -- real sensor noise never holds this still; treat it as wedged and recover
          logPrint(F("[E] APDS9960 proximity reading appears stuck, re-initializing..."));
          if(apds.begin()) {
            apds.enableProximity(true);
            apds.setProxPulse(APDS9960_PPULSELEN_8US, 8);
          } else {
            ToFInitDone = false; //hand off to the existing retry-until-successful path below
          }
          apdsLastChange = millis();
        }
        if(booping == false && (255 - apdsprox) < cfg.boopThresh) {
          logPrint(F("[I] ToF BOOP"));
          booping = true;
          boopoldanim = currentAnim;
          loadAnim(cfg.aBoop,"");
          lastMillsBoop = millis();
        } else if(booping == true && lastMillsBoop+1000<millis() && (255 - apdsprox) > cfg.boopThresh) {
          logPrint(F("[I] ToF unBOOP"));
          booping = false;
          if(!wasTilt) {
            loadAnim(boopoldanim,"");
          }
        }
      } else {
        if(!apds.begin()){
          cfg.boopEna = false;
          logPrint("[E] An Error has occurred while initializing APDS9960 chip! (raw ID reg: 0x"+String(apdsRawID(),HEX)+")");
        } else {
          ToFInitDone = true;
          apds.enableProximity(true);
        }
      }
    } else if (boopMode == "VL53L1X") {
      if(ToFInitDone) {
        int16_t distance = -1;
        if (vl53.dataReady()) {
          distance = vl53.distance();
        }
        if(booping == false && distance < cfg.boopThresh && distance != -1) {
          logPrint(F("[I] ToF BOOP"));
          booping = true;
          boopoldanim = currentAnim;
          loadAnim(cfg.aBoop,"");
          lastMillsBoop = millis();
        } else if(booping == true && lastMillsBoop+1000<millis() && distance > cfg.boopThresh && distance != -1) {
          logPrint(F("[I] ToF unBOOP"));
          booping = false;
          if(!wasTilt) {
            loadAnim(boopoldanim,"");
          }
        }
      } else {
        Wire.beginTransmission(0x29);
        if(Wire.endTransmission() != 0) {
          cfg.boopEna = false;
          logPrint(F("[E] An Error has occurred while finding VL53L1X chip!"));
        } else {
          if(!vl53.begin()){
            cfg.boopEna = false;
            logPrint(F("[E] An Error has occurred while initializing VL53L1X chip!"));
          } else {
            if (!vl53.startRanging()) {
              cfg.boopEna = false;
              logPrint(F("[E] An Error has occurred while starting ranging with VL53L1X chip!"));
            } else {
              ToFInitDone = true;
              vl53.setTimingBudget(50);
            }
          }
        }
      }
    }
    lastBoopCheck=millis();
  }
  //Serial.println(">BP:"+String(micros()-looptime));

  //--------------------------------//OLED routine, ~~10ms qwq~~, 1-5ms.. eh better
  if(cfg.oledEna && oledInitDone && vaStatLast+1000<millis()) {
    //looptime = micros();
    if(cfg.inaEna && inaInitDone) {
      uint16_t cal = 0, rawCurrent = 0, rawBus = 0;
      bool inaOk = inaRead16(INA219_REG_CALIBRATION, cal);
      if(inaOk && cal != INA_CAL_VALUE) { //the INA219 reset (power-on default: calibration 0, config 0x399F), so the current reads 0 until both are restored
        inaResets++;
        logPrint("[E] INA219 reset detected (calibration "+String(cal)+"), restoring config; "+String(inaResets)+" resets since boot, check its VCC/GND wiring");
        inaWrite16(INA219_REG_CONFIG, INA_CONFIG_VALUE);
        inaWrite16(INA219_REG_CALIBRATION, INA_CAL_VALUE);
        delay(2); //the current register is only recalculated at the end of the next conversion (532us shunt + 532us bus)
      }
      inaOk = inaOk && inaRead16(INA219_REG_CURRENT, rawCurrent) && inaRead16(INA219_REG_BUSVOLTAGE, rawBus);
      float currentMA = (int16_t)rawCurrent / (float)INA_CURRENT_DIVIDER_MA;
      float busV = (rawBus >> 3) * 4 * 0.001f; //drop CNVR and OVF, 4mV per bit
      oled.writeINA(busV,currentMA,inaOk);
    }
    if(cfg.bleEna) {
      if(pServer->getConnectedCount() == 0) {
        remoteSign = !remoteSign;
        oled.remote(remoteSign);
      } else if (pServer->getConnectedCount() > 0 && remoteSign == false) {
        oled.remote(true);
        remoteSign = true;
      }
    } else if (!cfg.bleEna && remoteSign) {
      oled.remote(false);
      remoteSign = false;
    }
    vaStatLast = millis();
    //Serial.println(">OLED:"+String(micros()-looptime));
  }
  if (!oledInitDone && cfg.oledEna) {
    if(!oled.init(oledAddr,cfg.bOled,cfg.inaEna,cfg.oledFlip)) {
      logPrint(F("[E] An Error has occurred while initializing SSD1306."));
      cfg.oledEna = false;
    } else {
      oledInitDone = true;
    }
  }
  if (!inaInitDone && cfg.inaEna && oledInitDone) { //retry so enabling INA219 live (web toggle) doesn't need a reboot
    if(!ina219.begin()) {
      logPrint(F("[E] An Error has occurred while initializing INA219 chip."));
      cfg.inaEna = false;
    } else {
      ina219.setCalibration_16V_8A();
      inaInitDone = true;
    }
  }

  if((FdisplayEar || FdisplayBlush || FdisplayVisor) ) {
    if(visorType == "WS2812") {
      if(FdisplayVisor) {
        if(visorNow->type == 0) {
          if(FADESTEPS == 0) { //just fading
            memcpy(visorLeds, visorLedsNEW, sizeof(CRGB) * visorLedsNum);
            pushLeds = true;
            FdisplayVisor = false;
          } else if(fadeTime + (visorLedsNum*0.03) < millis()) {
            for (uint16_t i = 0; i < visorLedsNum; i++) {
              visorLeds[i] = blend(visorLeds[i], visorLedsNEW[i], (currFade * 255) / FADESTEPS);
            }
            pushLeds = true;
            currFade++;
            if(currFade > FADESTEPS) {
              memcpy(visorLeds, visorLedsNEW, sizeof(CRGB) * visorLedsNum);
              FdisplayVisor = false;
              currFade = 1;
            }
          }
          /*for (uint8_t step = 1; step <= FADESTEPS; step++) {
            for (uint16_t i = 0; i < visorLedsNum; i++) {
              visorLeds[i] = blend(visorLeds[i], visorLedsNEW[i], (step * 255) / FADESTEPS);
            }
            ledController[0]->showLeds(cfg.bVisor); //visor
            delay(21); 
          }
          memcpy(visorLeds, visorLedsNEW, sizeof(CRGB) * visorLedsNum);
          FdisplayVisor = false;*/
        } else {
          pushLeds = true;
          FdisplayVisor = false;
        }
      }
    } else if (visorType == "MAX72XX") {
      if(FdisplayVisor) {
        if(cfg.bVisor > 15) { cfg.bVisor = 15;}
        mx.control(MD_MAX72XX::INTENSITY, cfg.bVisor);
        mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
        mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
        FdisplayVisor = false;
      }
    }
    if(earPresent && !blushPresent && FdisplayEar) { //RMT4 2 controllers fix
      for(int i = 0; i < earLedsNum; i++) {
        c2Leds[i] = earLeds[i];
      }
      pushLeds = true;
      FdisplayEar = false;
    } else if (blushPresent && !earPresent && FdisplayBlush) {
      for(int i = 0; i < blushLedsNum; i++) {
        c2Leds[i] = blushOut(blushLeds[i]);
      }
      pushLeds = true;
      FdisplayBlush = false;
    } else if (blushPresent && earPresent && (FdisplayBlush || FdisplayEar)) { //ear-blush-ear
      for(int i = 0; i < (earLedsNum/2); i++) {
        c2Leds[i] = earLeds[i];
      }
      for(int i = (earLedsNum/2); i < (earLedsNum/2)+blushLedsNum; i++) {
        c2Leds[i] = blushOut(blushLeds[i-(earLedsNum/2)]);
      }
      for(int i = (earLedsNum/2)+blushLedsNum; i < earLedsNum+blushLedsNum; i++) {
        c2Leds[i] = earLeds[i-blushLedsNum];
      }
      pushLeds = true;
      FdisplayEar = false;
      FdisplayBlush = false;
    }
    /*if(FdisplayEar && earPresent) {
      ledController[1]->showLeds(cfg.bEar); //ears
      FdisplayEar = false;
    }
    if(FdisplayBlush && blushPresent) {
      ledController[2]->showLeds(cfg.bBlush); //blush
      FdisplayBlush = false;
    }*/
  }
  if(pushLeds) { //one push per loop pass, see showAllLeds()
    showAllLeds();
    pushLeds = false;
  }

  //press boot button for 10sec to reset
  if(check0button+10000 < millis() && check0button+10500 > millis() && digitalRead(0) == LOW) {
    logPrint(F("[I] Resetting to defaults"));
    cfg.setDefault();
    delay(20);
    ESP.restart();
  } else if (digitalRead(0) == HIGH && check0button+10000 > millis() && check0button < millis()) {
    check0button = 0;
  } else if (digitalRead(0) == LOW && check0button+10000 < millis() && check0button < millis()) {
    check0button = millis();
  }

  if(animToLoad != "") {
    loadAnim(animToLoad,"");
    animToLoad = "";
    wasTilt = false;
    booping = false;
  }
}
