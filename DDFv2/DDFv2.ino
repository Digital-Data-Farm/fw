/* Integrated Farm Monitor Sketch — API INTA Compatible
 * - 4G + SDI-12 + RS485 + I2C (except MCP & INA)
 * - Contadores con MCP23017
 * - GPS externo
 * - Datos adaptados a estructura JSON Digital Data Farm (INTA)
 */

#include "utilities.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <SDI12.h>
#include <TinyGsmClient.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_INA219.h>
#include <time.h>
#include <map>
#define MODEM_BAUDRATE             115200
#define MODEM_DTR_PIN              25
#define MODEM_TX_PIN               26
#define MODEM_RX_PIN               27
#define BOARD_PWRKEY_PIN           4
#define BOARD_POWERON_PIN          12
#define MODEM_RING_PIN             33
#define MODEM_RESET_PIN            5
#define MODEM_RESET_LEVEL          HIGH
#define SerialAT                   Serial1

#define SerialGPS                  Serial2
#define BOARD_GPS_TX_PIN           21
#define BOARD_GPS_RX_PIN           22
#define BOARD_GPS_PPS_PIN          23
#define BOARD_GPS_WAKEUP_PIN       19

#define BOARD_SCK_PIN              14
#define BOARD_MISO_PIN             2
#define BOARD_MOSI_PIN             15
#define BOARD_SD_CS_PIN            13
#define SDI12_1_PIN                35
#define SDI12_2_PIN                34
#define SDI12_POWER_PIN            32
#define I2C_SDA_PIN                21
#define I2C_SCL_PIN                22

#define MCP1_ADDR                  0x20
#define MCP2_ADDR                  0x21
Adafruit_MCP23X17 mcp1, mcp2;
SDI12            sdiBus1(SDI12_1_PIN), sdiBus2(SDI12_2_PIN);
Adafruit_INA219  inaPanel(0x43), inaBattery(0x42);
TinyGPSPlus      gps;

String network_apn, apn_user, apn_pass, bearer_token;
const char* URL_SAVE  = "https://api.digitaldatafarm.com/api/v4/logger/save_sonda_info";
const char* URL_EVENT = "https://api.digitaldatafarm.com/api/v4/logger/event";

struct DeviceConfig {
  String plot_name;
  String dl_name;
  String version;
  int sensor_read_frec_sec;
  int send_data_frec_sec;
  int send_data_window_sec;
};


DeviceConfig config = {"TEST","1","0.2",5,30,5};

const char* CONFIG_FILE  = "/config.txt";
const char* PENDING_FILE = "/pending.txt";

float litersPerPulse = 10.0;
uint32_t pulseCount1 = 0, pulseCount2 = 0;
bool lastCnt1State = false, lastCnt2State = false;
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
unsigned long lastSendTime = 0;
double gpsLat = NAN, gpsLng = NAN;
char   gpsDateTime[32] = "";
std::map<String, JsonArray> sdi12Results;


void setup() {
  Serial.begin(115200);
  delay(2000);
  SPI.begin(BOARD_SCK_PIN, BOARD_MISO_PIN, BOARD_MOSI_PIN, BOARD_SD_CS_PIN);
  SD.begin(BOARD_SD_CS_PIN);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  sdiBus1.begin(); sdiBus2.begin();
  mcp1.begin_I2C(MCP1_ADDR); mcp2.begin_I2C(MCP2_ADDR);
  inaPanel.begin(); inaBattery.begin();
  SerialGPS.begin(9600, SERIAL_8N1, BOARD_GPS_RX_PIN, BOARD_GPS_TX_PIN);
  pinMode(SDI12_POWER_PIN, OUTPUT);
  digitalWrite(SDI12_POWER_PIN, LOW);
  readConfig();
  initModem();
}

void loop() {
  while (SerialGPS.available()) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {
        gpsLat = gps.location.lat(); gpsLng = gps.location.lng();
      }
      if (gps.date.isValid() && gps.time.isValid()) {
        snprintf(gpsDateTime, sizeof(gpsDateTime), "%04d-%02d-%02dT%02d:%02d:%02dZ",
                 gps.date.year(), gps.date.month(), gps.date.day(),
                 gps.time.hour(), gps.time.minute(), gps.time.second());
      }
    }
  }

  processCounters();
  readSensors();
  if (millis() - lastSendTime >= config.send_data_frec_sec * 1000UL) {
    lastSendTime = millis();
    String payload = createPayload();
    String resp = sendWithGSM(payload, URL_SAVE);
    if (resp.length()) handleResponse(resp);
    else bufferPending(payload);
  }
}



void readSensors() {
  sdi12Results.clear();
  digitalWrite(SDI12_POWER_PIN, HIGH);
  delay(5000);

  for (SDI12* bus : {&sdiBus1, &sdiBus2}) {
    for (int i = 0; i <= 61; i++) {
      char addr = (i < 10 ? '0' + i : (i < 36 ? 'A' + i - 10 : 'a' + i - 36));
      bus->clearBuffer();
      String cmd = String(addr) + "!";
      bus->sendCommand(cmd); delay(100);
      if (bus->available()) {
        bus->clearBuffer();
        String measureCmd = String(addr) + "C!";
        bus->sendCommand(measureCmd);
        delay(2000);
        String sondaID = "SENTEK__DD_" + String(addr) + String((int)millis() % 100000); // fake ID
        JsonArray temp = sdi12Results[sondaID].createNestedArray();
        JsonArray hum  = sdi12Results[sondaID].createNestedArray();
        JsonArray sal  = sdi12Results[sondaID].createNestedArray();
        for (int d = 0; d < 3; d++) {
          String cmdD = String(addr) + "D" + String(d) + "!";
          bus->sendCommand(cmdD); delay(300);
          while (bus->available()) {
            float val = bus->parseFloat(SKIP_NONE);
            if (!temp.size()) temp.add(val);
            else if (!hum.size()) hum.add(val);
            else sal.add(val);
          }
        }
      }
    }
  }
  digitalWrite(SDI12_POWER_PIN, LOW);
}
void saveConfigToSD() {
  File f = SD.open(CONFIG_FILE, FILE_WRITE);
  if (!f) {
    Serial.println("[ERROR] No se pudo abrir config.txt para escribir.");
    return;
  }
  f.println(config.plot_name);
  f.println(config.dl_name);
  f.println(config.sensor_read_frec_sec);
  f.println(config.send_data_frec_sec);
  f.println(config.send_data_window_sec);
 f.println(network_apn);
f.println(apn_user);
f.println(apn_pass);
f.println(bearer_token);

  f.close();
  Serial.println("[INFO] Configuración guardada en config.txt");
}

void initI2C() {
  mcp1.begin_I2C(MCP1_ADDR);
  for (uint8_t p: {0,1,2,3}) mcp1.pinMode(p, INPUT); // MAX3485
  mcp1.pinMode(5, INPUT); mcp1.pinMode(6, INPUT);   // counters
  for (uint8_t p=0; p<6; p++) mcp1.pinMode(p+8, OUTPUT); // sol

  mcp2.begin_I2C(MCP2_ADDR);
  for (uint8_t p: {3,5,6}) mcp2.pinMode(p, INPUT);
  for (uint8_t p=0; p<3; p++) mcp2.pinMode(p, OUTPUT);
  for (uint8_t p=4; p<7; p++) mcp2.pinMode(p+8-4, OUTPUT);

  inaPanel.begin(); inaBattery.begin();
}
void readConfig() {
  File f = SD.open(CONFIG_FILE);
  if (!f) return;
  config.plot_name            = f.readStringUntil('\n'); config.plot_name.trim();
  config.dl_name              = f.readStringUntil('\n'); config.dl_name.trim();
  config.sensor_read_frec_sec = f.readStringUntil('\n').toInt();
  config.send_data_frec_sec   = f.readStringUntil('\n').toInt();
  config.send_data_window_sec = f.readStringUntil('\n').toInt();
  network_apn = f.readStringUntil('\n'); network_apn.trim();
  apn_user    = f.readStringUntil('\n'); apn_user.trim();
  apn_pass    = f.readStringUntil('\n'); apn_pass.trim();
  bearer_token = f.readStringUntil('\n'); bearer_token.trim();
  f.close();
}

void processCounters() {
  bool st1 = (mcp1.digitalRead(5) == LOW);
  if (st1 && !lastCnt1State) pulseCount1++;
  lastCnt1State = st1;
  bool st2 = (mcp1.digitalRead(6) == LOW);
  if (st2 && !lastCnt2State) pulseCount2++;
  lastCnt2State = st2;
}
String createPayload() {
  StaticJsonDocument<2048> doc;
  doc["v"] = config.version;
  doc["sn"] = "203036583830500900490051";
  JsonArray entries = doc.createNestedArray("dl_entries");
  JsonObject entry = entries.createNestedObject();
  entry["ts"] = time(nullptr);
  JsonObject parcela = entry.createNestedObject(config.plot_name);
  JsonObject dl = parcela.createNestedObject(config.dl_name);

  JsonArray C = dl.createNestedArray("C");
  C.add(pulseCount1 + pulseCount2);

  for (auto& kv : sdi12Results) {
    dl[kv.first] = kv.second;
  }

  JsonArray D = dl.createNestedArray("D");
  D.add(inaBattery.getBusVoltage_V()*1000);
  D.add(inaBattery.getCurrent_mA());
  D.add(100); // estado fijo
  D.add(inaBattery.getPower_mW()/1000.0);

  return doc.as<String>();
}

String sendWithGSM(const String &json, const char* url) {
  modem.https_begin();
  modem.sendAT("AT+CSSLCFG=\"ignorecertCN\",0,1"); modem.waitResponse();
  if (!modem.https_set_url(url)) return "";
  modem.https_add_header("Content-Type","application/json");
  modem.https_add_header("Authorization", String("Bearer ")+bearer_token);
  int code = modem.https_post(json);
  if (code != 200) return "";
  return modem.https_body();
}

void handleResponse(const String &resp) {
  StaticJsonDocument<2048> doc, events;
  if (deserializeJson(doc, resp)) return;
  if (!doc["ok"].as<bool>()) return;
  if (!doc["data"]["commands"].is<JsonArray>()) return;
  for (JsonObject cmd: doc["data"]["commands"].as<JsonArray>()) {
    String t = cmd["command_type"].as<String>();
    int id      = cmd["id"].as<int>();
    String pkg  = cmd["package_sent_id"].as<String>();
    if (t=="SET_VALUE") cmdSetValue(cmd,pkg,id,events);
    else if (t=="GET_CONFIG") cmdGetConfig(pkg,id,events);
    else if (t=="REBOOT_REQ") cmdReboot(cmd,pkg,id,events);
    else if (t=="RELAY") cmdRelay(cmd);
    else if (t=="LATCH") cmdLatch(cmd);
  }
  if (events.size()) {
    String ev; serializeJson(events,ev);
    confirmEvents(ev);
  }
}
void confirmEvents(const String &evtJson) {
  sendWithGSM(evtJson, URL_EVENT);
}
void bufferPending(const String &json) {
  File f = SD.open(PENDING_FILE, FILE_APPEND);
  if (f) { f.println(json); f.close(); }
}
void flushPending() {
  if (!SD.exists(PENDING_FILE)) return;
  File f = SD.open(PENDING_FILE);
  while (f.available()) {
    String j = f.readStringUntil('\n');
    if (sendWithGSM(j, URL_SAVE).length()) continue;
    break;
  }
  f.close(); SD.remove(PENDING_FILE);
}

void cmdSetValue(JsonObject &cmd, const String &pkg, int id, JsonDocument &events) {
  bool ok = true;
  for (JsonObject it : cmd["payload"].as<JsonArray>()) {
    String n = it["var_name"].as<String>();
    String v = it["value"].as<String>();
    if (n == "sensor_read_frec_sec") config.sensor_read_frec_sec = v.toInt();
    else if (n == "send_data_frec_sec") config.send_data_frec_sec = v.toInt();
    else if (n == "send_data_window_sec") config.send_data_window_sec = v.toInt();
    else if (n == "plot_name") config.plot_name = v;
    else if (n == "liters_per_pulse") litersPerPulse = v.toFloat();
else if (n == "apn") network_apn = v;
else if (n == "apn_user") apn_user = v;
else if (n == "apn_pass") apn_pass = v;
else if (n == "bearer_token") bearer_token = v;

    else ok = false;
  }

  if (ok) {
    saveConfigToSD(); // 🟢 Guardar si se actualizaron valores válidos
  }

  JsonObject r = events.createNestedObject(pkg).createNestedObject(String(id));
  if (!ok) r["error_code"] = 1;
}


void cmdGetConfig(const String &pkg,int id,JsonDocument &events) {
  JsonObject r = events.createNestedObject(pkg).createNestedObject(String(id));
  r["plot_name"] = config.plot_name;
  r["dl_name"]   = config.dl_name;
  r["send_data_frec_sec"]    = config.send_data_frec_sec;
  r["send_data_window_sec"]  = config.send_data_window_sec;
  r["liters_per_pulse"]      = litersPerPulse;
}

void cmdReboot(JsonObject &cmd,const String &pkg,int id,JsonDocument &events) {
  int w = cmd["payload"][0]["wait_seconds"].as<int>();
  delay(w*1000); ESP.restart();
}

void cmdRelay(JsonObject &cmd) {
  cmdLatch(cmd); // same pattern on mcp2
}

void cmdLatch(JsonObject &cmd) {
  bool on = cmd["state"].as<String>()=="ON";
  // Relay on mcp2
  uint8_t a = on ? HIGH:LOW, b = on?LOW:HIGH;
  uint8_t pinA = cmd["id"].as<int>()==1?8:9;
  uint8_t pinB = cmd["id"].as<int>()==1?9:10;
  uint8_t en   = cmd["id"].as<int>()==1?10:11;
  mcp2.digitalWrite(pinA,a); mcp2.digitalWrite(pinB,b);
  mcp2.digitalWrite(en,HIGH); delay(config.send_data_window_sec*1000UL);
  mcp2.digitalWrite(en,LOW);
}
void initModem() {
  pinMode(BOARD_POWERON_PIN, OUTPUT); digitalWrite(BOARD_POWERON_PIN, HIGH);
  pinMode(MODEM_RESET_PIN, OUTPUT); digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
  delay(100); digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL); delay(2600);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
  pinMode(BOARD_PWRKEY_PIN, OUTPUT); digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(100); digitalWrite(BOARD_PWRKEY_PIN, HIGH); delay(1000); digitalWrite(BOARD_PWRKEY_PIN, LOW);
  SerialAT.begin(MODEM_BAUDRATE, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  while (!modem.testAT(1000)) delay(500);
  modem.sendAT(String("AT+CGDCONT=1,\"IP\",\"") + network_apn + "\""); modem.waitResponse();
  while (modem.getRegistrationStatus() != REG_OK_HOME && modem.getRegistrationStatus() != REG_OK_ROAMING) delay(1000);
}
