#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <Servo.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include "FirebaseESP8266.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Task.h>

#ifndef MACHINE_NAME
#define MACHINE_NAME "Assistive Device"
#define PASSWORD  "12341234"
#define FIREBASE_HOST "assistive-device-61085-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "ZDkEDVSydEGf8l40Lch923IP9DRWXJqFvf3nVqZe"
#endif

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
// Variable to save current epoch time
int timestamp;

FirebaseData update_mapped_data;
FirebaseData update_filtered_data;
FirebaseData update_data;

const char *softAP_ssid = MACHINE_NAME;
const char *softAP_password = PASSWORD;
/* hostname for mDNS. Should work at least on windows. Try http://esp8266.local */
const char *myHostname = "yasi_network.local";
/* Don't set this wifi credentials. They are configurated at runtime and stored on EEPROM */
char ssid[33] = "";
char password[65] = "";

// DNS server
const byte DNS_PORT = 53;
DNSServer dnsServer;
// Web server
ESP8266WebServer server(80);
/* Soft AP network parameters */
IPAddress apIP(172, 217, 28, 1);
IPAddress netMsk(255, 255, 255, 0);
/** Should I connect to WLAN asap? */
boolean connect;
/** Last time I tried to connect to WLAN */
unsigned long lastConnectTry = 0;
/** Current WLAN status */
unsigned int status = WL_IDLE_STATUS;

unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}

/*
  SimpleKalmanFilter(e_mea, e_est, q);
  e_mea: Measurement Uncertainty
  e_est: Estimation Uncertainty
  q: Process Noise
*/
SimpleKalmanFilter simpleKalmanFilter_x(22.5, 45, 0.1);
SimpleKalmanFilter simpleKalmanFilter_y(22.5, 45, 0.1);
SimpleKalmanFilter simpleKalmanFilter_z(22.5, 45, 0.1);

const long SERIAL_REFRESH_TIME = 50;
long refresh_time;

Servo servo_x_axis;
Servo servo_y_axis;

int init_x = 45, init_y = 45, init_z = 45;
int mapped_x = 0, mapped_y = 0, mapped_z = 0;
float previous_mapped_x = 0, previous_mapped_y = 0, previous_mapped_z = 0;
float previous_filtered_x = 0, previous_filtered_y = 0, previous_filtered_z = 0;

float x, y, z;
const int MPU_ADDR = 0x68;
int16_t accelerometer_x, accelerometer_y, accelerometer_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t temperature;

Task databaseTask(TASK_MILLISECOND * 100, TASK_FOREVER, NULL, NULL, NULL);

void databaseTaskCallback() {
  // Get the current date and time
  unsigned long currentTime = millis() / 1000;
  String time = String(currentTime);

  bool dataChanged = (mapped_x != previous_mapped_x ||
                      mapped_y != previous_mapped_y ||
                      mapped_z != previous_mapped_z ||
                      filtered_x != previous_filtered_x ||
                      filtered_y != previous_filtered_y ||
                      filtered_z != previous_filtered_z);

  if (dataChanged) {
    UpdateData(mapped_x, mapped_y, mapped_z, filtered_x, filtered_y, filtered_z);
    // Update previous values
    previous_mapped_x = mapped_x;
    previous_mapped_y = mapped_y;
    previous_mapped_z = mapped_z;
    previous_filtered_x = filtered_x;
    previous_filtered_y = filtered_y;
    previous_filtered_z = filtered_z;
  }
}

void setup() {
  Serial.begin(115200);
  servo_x_axis.attach(D6);
  servo_y_axis.attach(D7);
  servo_x_axis.write(0);
  servo_y_axis.write(0);
  delay(1000);
  servo_x_axis.write(90);
  servo_y_axis.write(90);
  delay(1000);
  servo_x_axis.write(init_x);
  servo_y_axis.write(init_y);
  delay(100);
  setup_gyro();

  delay(1000);
  /* You can remove the password parameter if you want the AP to be open. */
  WiFi.softAPConfig(apIP, apIP, netMsk);
  WiFi.softAP(softAP_ssid, softAP_password);
  delay(500); // Without delay I've seen the IP address blank

  /* Setup the DNS server redirecting all the domains to the apIP */
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, "*", apIP);

  /* Setup web pages: root, wifi config pages, SO captive portal detectors and not found. */
  server.on("/", handleRoot);
  server.on("/wifi", handleWifi);
  server.on("/wifisave", handleWifiSave);
  server.on("/generate_204", handleRoot);  //Android captive portal. Maybe not needed. Might be handled by notFound handler.
  server.on("/fwlink", handleRoot);  //Microsoft captive portal. Maybe not needed. Might be handled by notFound handler.
  server.onNotFound(handleNotFound);
  server.begin(); // Web server start
  loadCredentials(); // Load WLAN credentials from network
  connect = strlen(ssid) > 0; // Request WLAN connect if there is a SSID

  refresh_time = millis() + SERIAL_REFRESH_TIME;

  databaseTask.setCallback(databaseTaskCallback);
  databaseTask.enable();
}

void connectWifi() {
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  int connRes = WiFi.waitForConnectResult();
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  timeClient.begin();
}

void loop() {
  if (connect) {
    connect = false;
    connectWifi();
    lastConnectTry = millis();
  }
  {
    unsigned int s = WiFi.status();
    if (s == 0 && millis() > (lastConnectTry + 60000)) {
      /* If WLAN disconnected and idle try to connect */
      /* Don't set retry time too low as retry interfere the softAP operation */
      connect = true;
    }
    if (status != s) { // WLAN status change
      status = s;
      if (s == WL_CONNECTED) {
        /* Just connected to WLAN */
        // Setup MDNS responder
        if (!MDNS.begin(myHostname)) {
        } else {
          MDNS.addService("http", "tcp", 80);
        }
      } else if (s == WL_NO_SSID_AVAIL) {
        WiFi.disconnect();
      }
    }
    if (s == WL_CONNECTED) {
      MDNS.update();
    }
  }
  // Do work:
  fetch_gyro();
  if (x > 100) x = 100;
  if (x < -100) x = -100;
  if (y > 100) y = 100;
  if (y < -100) y = -100;
  if (z > 100) z = 100;
  if (z < -100) z = -100;

  mapped_x = map(x, -100, 100, 0, 90);
  mapped_y = map(y, -100, 100, 0, 90);
  mapped_z = map(z, -100, 100, 0, 90);

  float calculated_x = mapped_x + random(-100, 100) / 100.0;
  float calculated_y = mapped_y + random(-100, 100) / 100.0;
  float calculated_z = mapped_z + random(-100, 100) / 100.0;

  float filtered_x = simpleKalmanFilter_x.updateEstimate(calculated_x);
  float filtered_y = simpleKalmanFilter_y.updateEstimate(calculated_y);
  float filtered_z = simpleKalmanFilter_z.updateEstimate(calculated_z);

  if (millis() > refresh_time) {
    Serial.print("X:");
    Serial.print(mapped_x);
    Serial.print(",");
    Serial.print("EX:");
    Serial.print(filtered_x, 4);
    Serial.print(",");
    Serial.print("Y:");
    Serial.print(mapped_y);
    Serial.print(",");
    Serial.print("EY:");
    Serial.print(filtered_y, 4);
    Serial.print(",");
    Serial.print("Z:");
    Serial.print(mapped_z);
    Serial.print(",");
    Serial.print("EZ:");
    Serial.println(filtered_z, 4);
    refresh_time = millis() + SERIAL_REFRESH_TIME;
  }

  servo_x_axis.write(filtered_x);
  servo_y_axis.write(filtered_y);
  //DNS
  dnsServer.processNextRequest();
  //HTTP
  server.handleClient();
}

void UpdateData(float x_m, float y_m, float z_m, float x_f, float y_f, float z_f) {
  FirebaseJson json;
  timestamp = getTime();
  json.set("map_x", x_m);
  json.set("map_y", y_m);
  json.set("map_z", z_m);
  json.set("x", x_f);
  json.set("y", y_f);
  json.set("z", z_f);
  json.set("time", getTime());
  Firebase.pushJSON(update_data, "/D1/", json);
}

void setup_gyro() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void fetch_gyro() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 7 * 2, true);

  accelerometer_x = Wire.read() << 8 | Wire.read();
  accelerometer_y = Wire.read() << 8 | Wire.read();
  accelerometer_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();

  x = accelerometer_x / 100.0;
  y = accelerometer_y / 100.0;
  z = accelerometer_z / 100.0;
}