#include "utilities.h"
#include <SPI.h>
#include <SD.h>
#include <TinyGsmClient.h>
#include <Update.h>
#include <Arduino.h>

#define MODEM_RX_PIN 27
#define MODEM_TX_PIN 26
#define MODEM_PWRKEY 4
#define MODEM_RST    5
#define MODEM_POWER  12
#define MODEM_BAUD   115200
#define SerialAT     Serial1
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);

String apn, apn_user, apn_pass;
const char* CONFIG_FILE = "/config.txt";

const String version_actual = "1.0.0";
const char* version_url = "https://raw.githubusercontent.com/Digital-Data-Farm/fw/main/DDFv2/build/ddf/version.txt";
const char* firmware_url = "https://raw.githubusercontent.com/Digital-Data-Farm/fw/main/DDFv2/build/ddf/DDFv2.ino.bin";

void setup() {
  Serial.begin(115200);
  delay(2000);

  SD.begin(13);
  readConfig();

  pinMode(MODEM_POWER, OUTPUT); digitalWrite(MODEM_POWER, HIGH);
  pinMode(MODEM_RST, OUTPUT); digitalWrite(MODEM_RST, HIGH);
  pinMode(MODEM_PWRKEY, OUTPUT); digitalWrite(MODEM_PWRKEY, LOW);
  delay(100); digitalWrite(MODEM_PWRKEY, HIGH); delay(1000); digitalWrite(MODEM_PWRKEY, LOW);

  SerialAT.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  modem.restart();
  modem.gprsConnect(apn.c_str(), apn_user.c_str(), apn_pass.c_str());

  if (modem.isGprsConnected()) {
    Serial.println("Conectado a red GSM.");
    verificarActualizacion();
  } else {
    Serial.println("Fallo en conexión GSM.");
  }
}

void loop() {
  // Nada aquí
}

void readConfig() {
  File f = SD.open(CONFIG_FILE);
  if (!f) {
    Serial.println("No se pudo abrir config.txt");
    return;
  }
  f.readStringUntil('\n'); // plot_name (ignorar)
  f.readStringUntil('\n'); // dl_name (ignorar)
  f.readStringUntil('\n'); // frec1
  f.readStringUntil('\n'); // frec2
  f.readStringUntil('\n'); // frec3
  apn = f.readStringUntil('\n'); apn.trim();
  apn_user = f.readStringUntil('\n'); apn_user.trim();
  apn_pass = f.readStringUntil('\n'); apn_pass.trim();
  f.close();
}

void verificarActualizacion() {
  Serial.println("Verificando versión...");
  modem.https_begin();
  modem.sendAT("AT+CSSLCFG=\"ignorecertCN\",0,1"); modem.waitResponse();
  if (!modem.https_set_url(version_url)) return;
  int code = modem.https_get();
  if (code == 200) {
    String nueva = modem.https_body(); nueva.trim();
    Serial.println("Versión remota: " + nueva);
    if (nueva != version_actual) actualizarFirmware();
  } else {
    Serial.println("Error HTTP: " + String(code));
  }
  modem.https_end();
}

void actualizarFirmware() {
  modem.https_begin();
  modem.sendAT("AT+CSSLCFG=\"ignorecertCN\",0,1"); modem.waitResponse();
  if (!modem.https_set_url(firmware_url)) return;

  int code = modem.https_get();
  if (code != 200) {
    Serial.println("Error descargando firmware: " + String(code));
    modem.https_end();
    return;
  }

  String bin = modem.https_body();
  size_t len = bin.length();
  if (!Update.begin(len)) return;
  Update.write((uint8_t*)bin.c_str(), len);

  if (Update.end() && Update.isFinished()) {
    Serial.println("Firmware actualizado, reiniciando...");
    ESP.restart();
  } else {
    Serial.println("Fallo en actualización.");
  }
  modem.https_end();
}
