/*************************************************************************/
/*    Copyright (C) 2023 Wizardry and Steamworks - License: GNU GPLv3    */
/*************************************************************************/
// Current documentation @                                               //
//    https://grimore.org/arduino/esp32-cam-mqtt-stream                  //
//                                                                       //
// About:                                                                //
// This is an Arduino sketch that is meant to work with an ESP32 CAM     //
// development board. When uploaded, the sketch will stream images from  //
// the ESP32 camera to an MQTT broker on a configurable topic.           //
//                                                                       //
// Additionally, the sketch can be configured dynamically by sending     //
// messages to the control topic that the sketch subscribes to.          //
//                                                                       //
// Usage:                                                                //
// The sketch can be configured by setting the necessary parmeters in    //
// the configuration section. Once configured, the sketch subscribes to  //
// topics on the MQTT broker:                                            //
//   * MQTT_TOPIC (meant for controlling the sketch),                    //
//   * MQTT_TOPIC_STREAM, the topic where the sketch will publish the    //
//     binary buffer of the image captured from the camera (JPEG).       //
//                                                                       //
// In order to set various parameters for the sketch, a message can be   //
// published on the control MQTT topic MQTT_TOPIC having the following   //
// grammar:                                                              //
//                                                                       //
// action := set | get                                                   //
// action = set := flash                                                 //
// state := on | off                                                     //
//                                                                       //
// For example, a JSON payload with the following structure can be sent  //
// to the MQTT control topic MQTT_TOPIC in order to toggle the LED:      //
// {                                                                     //
//   "action": "set",                                                    //
//   "flash": {                                                          //
//     "state": "on"                                                     //
//   }                                                                   //
// }                                                                     //
//                                                                       //
// Similarly, the state of the flash LED can be retrieved by sending the //
// following JSON payload on the control topic MQTT_TOPIC:               //
// {                                                                     //
//   "action": "get",                                                    //
//   "flash": "state"                                                    //
// }                                                                     //
//                                                                       //
///////////////////////////////////////////////////////////////////////////
 
///////////////////////////////////////////////////////////////////////////
//                           configuration                               //
///////////////////////////////////////////////////////////////////////////
// The AP to connect to via Wifi.
#define STA_SSID "Kablonet Netmaster-84C6-G"
// The AP Wifi password.
#define STA_PSK "1136a352*"
// The MQTT broker to connect to.
#define MQTT_HOST "192.168.0.31"
// The MQTT broker username.
#define MQTT_USERNAME "mqtt_espressif"
// The MQTT broker password.
#define MQTT_PASSWORD "38qFCidDkIg"
// The MQTT broker port.
#define MQTT_PORT 1883
// The topic to subscribe to for control.
#define MQTT_TOPIC "esp/aaa"
// The topic to subscribe to for streaming images.
#define MQTT_TOPIC_STREAM "esp/aaa/stream"
// The estimated maximum picture buffer size.
#define PICTURE_BUFFER_SIZE 20000
// The default MQTT client ID is "esp-CHIPID" where CHIPID is the ESP8266
// or ESP32 chip identifier.
#define MQTT_CLIENT_ID() String("esp-" + String(GET_CHIP_ID(), HEX))
#include "esp_camera.h"
//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well
 
// ===================
// Select camera model
// ===================
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
//#define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
// ** Espressif Internal Boards **
//#define CAMERA_MODEL_ESP32_CAM_BOARD
//#define CAMERA_MODEL_ESP32S2_CAM_BOARD
//#define CAMERA_MODEL_ESP32S3_CAM_LCD
 
///////////////////////////////////////////////////////////////////////////
//                   general variable declarations                       //
///////////////////////////////////////////////////////////////////////////
// Platform specific defines.
#define GET_CHIP_ID() ((uint16_t)(ESP.getEfuseMac() >> 32))
 
// Miscellaneous defines.
//#define CHIP_ID_HEX (String(GET_CHIP_ID()).c_str())
#define HOSTNAME() String("esp-" + String(GET_CHIP_ID(), HEX))
#define EEPROM_SIZE 1
 
// Platform specific libraries.
#include <WiFi.h>
#include <EEPROM.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "camera_pins.h"
// General libraries.
#include <PubSubClient.h>
#include <ArduinoJson.h>
 
WiFiClient espClient;
#define MQTT_MAX_TRANSFER_SIZE PICTURE_BUFFER_SIZE
PubSubClient mqttClient(espClient);
 
TaskHandle_t cameraTaskHandle = NULL;
TaskHandle_t mqttTaskHandle = NULL;
SemaphoreHandle_t mutex_v;
uint8_t imageBuffer[PICTURE_BUFFER_SIZE];
size_t imageBufferLength;
int enableFlash;
 
String mqttSerialize(StaticJsonDocument<256> msg) {
  char output[256];
  serializeJson(msg, output, 256);
  return String(output);
}
 
///////////////////////////////////////////////////////////////////////////
//                            MQTT event handling                        //
///////////////////////////////////////////////////////////////////////////
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  String msgTopic = String(topic);
  // payload is not null terminated and casting will not work
  char msgPayload[length + 1];
  snprintf(msgPayload, length + 1, "%s", payload);
  Serial.println("Message received on topic: " + String(topic) + " with payload: " + String(msgPayload));
 
  // Parse the payload sent to the MQTT topic as a JSON document.
  StaticJsonDocument<256> doc;
  Serial.println("Deserializing message....");
  DeserializationError error = deserializeJson(doc, msgPayload);
  if (error) {
    Serial.println("Failed to parse MQTT payload as JSON: " + String(error.c_str()));
    return;
  }
 
  // Do not process messages without an action key.
  if (!doc.containsKey("action")) {
    return;
  }
 
  // Set various configuration parameters.
  String action = (const char *)doc["action"];
  if (action == "set") {
    String flashState = (const char *)doc["flash"]["state"];
    if (flashState == "on") {
      Serial.print("Flash is now: ");
      digitalWrite(LED_GPIO_NUM, HIGH);
      EEPROM.write(0, enableFlash = 1);
      Serial.println(flashState);
    } else if (flashState == "off") {
      Serial.print("Flash is now: ");
      digitalWrite(LED_GPIO_NUM, LOW);
      EEPROM.write(0, enableFlash = 0);
      Serial.println(flashState);
    } else {
      Serial.print("Unknown flash state received: ");
      Serial.println(flashState);
    }
 
    EEPROM.commit();
    return;
  }
 
  // Get the various configuration parameters.
  if (action == "get") {
    StaticJsonDocument<256> msg;
    String flashState = (const char *)doc["flash"];
    if (flashState == "state") {
      switch (enableFlash) {
        case 1:
          msg["flash"] = "on";
          break;
        case 0:
          msg["flash"] = "off";
          break;
        default:
          msg["flash"] = "unknown";
          break;
      }
    }
 
    mqttClient.publish(MQTT_TOPIC, mqttSerialize(msg).c_str());
    return;
  }
}
 
bool mqttConnect() {
  Serial.println("Attempting to connect to MQTT broker: " + String(MQTT_HOST));
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
 
  StaticJsonDocument<256> msg;
  if (mqttClient.connect(MQTT_CLIENT_ID().c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
    Serial.println("Established connection with MQTT broker using client ID: " + MQTT_CLIENT_ID());
    mqttClient.setCallback(mqttCallback);
    msg["action"] = "connected";
    mqttClient.publish(MQTT_TOPIC, mqttSerialize(msg).c_str());
    Serial.print("Attempting to subscribe to MQTT topic: ");
    if (!mqttClient.subscribe(MQTT_TOPIC)) {
      Serial.println("failure");
      return false;
    }
    Serial.println("success");
    msg.clear();
    msg["action"] = "subscribed";
    mqttClient.publish(MQTT_TOPIC, mqttSerialize(msg).c_str());
    return true;
  }
 
  Serial.println("Connection to MQTT broker failed with MQTT client state: " + String(mqttClient.state()));
 
  return false;
}
 
///////////////////////////////////////////////////////////////////////////
//                      Image and MQTT processing                        //
///////////////////////////////////////////////////////////////////////////
void SendImage(void *arg) {
START:
  if (!mqttClient.connected() || imageBufferLength == 0) {
    vTaskDelay(10 / portTICK_RATE_MS);
    goto START;
  }
 
  xSemaphoreTake(mutex_v, portMAX_DELAY);
  mqttClient.beginPublish(MQTT_TOPIC_STREAM, imageBufferLength, false);
  mqttClient.write(imageBuffer, imageBufferLength);
  xSemaphoreGive(mutex_v);
  mqttClient.endPublish();
  mqttClient.loop();
  goto START;
}
 
void TakeImage(void *arg) {
START:
  Serial.print("Taking picture: ");
  camera_fb_t *fb = esp_camera_fb_get();  // used to get a single picture.
  if (!fb) {
    Serial.println("failed");
    vTaskDelay(1000 / portTICK_RATE_MS);
    goto START;
  }
  Serial.print("done, size=");
  Serial.println(fb->len);
 
  if (fb->len > PICTURE_BUFFER_SIZE) {
    Serial.println("Picture too large, please increase the picture buffer size.");
    esp_camera_fb_return(fb);
    goto START;
  }
 
  xSemaphoreTake(mutex_v, portMAX_DELAY);
  memmove(imageBuffer, fb->buf, fb->len);
  imageBufferLength = fb->len;
  xSemaphoreGive(mutex_v);
 
  esp_camera_fb_return(fb);  // must be used to free the memory allocated by esp_camera_fb_get().
  goto START;
}
 
///////////////////////////////////////////////////////////////////////////
//                           Arduino functions                           //
///////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  Serial.println("Camera Streamer");
  Serial.println(MQTT_CLIENT_ID());
 
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 4;  // default 10
  config.fb_count = 1;
 
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.fb_count = 1;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }
 
#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif
 
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }
 
  sensor_t *s = esp_camera_sensor_get();
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }
 
  // Set the onboard flash LED as and output.
  pinMode(LED_GPIO_NUM, OUTPUT);
 
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME().c_str());
  WiFi.begin(STA_SSID, STA_PSK);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Failed to connect to Wifi, rebooting in 5s...");
    delay(5000);
    ESP.restart();
  }
 
  Serial.print("Connected to Wifi: ");
  Serial.println(WiFi.localIP());
 
  // Initialize configuration.
  EEPROM.begin(EEPROM_SIZE);
  enableFlash = EEPROM.read(0);
  switch (enableFlash) {
    case 1:
      digitalWrite(LED_GPIO_NUM, HIGH);
      break;
    case 0:
      digitalWrite(LED_GPIO_NUM, LOW);
      break;
  }
 
  // Set up MQTT client.
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setBufferSize(PICTURE_BUFFER_SIZE);
  mqttClient.setCallback(mqttCallback);
 
  // Spawn the task to capture an image from the camera.
  mutex_v = xSemaphoreCreateMutex();
  if (mutex_v == NULL) {
    Serial.println("Could not create a mutex");
    delay(1000);
    ESP.restart();
  }
  xTaskCreatePinnedToCore(TakeImage, "TakeImage", 4096, NULL, 1, &cameraTaskHandle, 0);
  xTaskCreatePinnedToCore(SendImage, "SendImage", 4096, NULL, 1, &mqttTaskHandle, 0);
 
  // Touchdown.
  Serial.println("Setup complete.");
}
 
void loop() {
  // Check the Wifi connection status.
  int wifiStatus = WiFi.status();
  switch (wifiStatus) {
    case WL_CONNECTED:
      // Process MQTT client loop.
      if (!mqttClient.connected()) {
        // If the connection to the MQTT broker has failed then sleep before carrying on.
        if (!mqttConnect()) {
          delay(1000);
          break;
        }
      }
      break;
    case WL_NO_SHIELD:
      Serial.println("No Wifi shield present.");
      goto DEFAULT_CASE;
      break;
    case WL_NO_SSID_AVAIL:
      Serial.println("Configured SSID not found.");
      goto DEFAULT_CASE;
      break;
    // Temporary statuses indicating transitional states.
    case WL_IDLE_STATUS:
    case WL_SCAN_COMPLETED:
      delay(1000);
      break;
    // Fatal Wifi statuses trigger a delayed ESP restart.
    case WL_CONNECT_FAILED:
    case WL_CONNECTION_LOST:
    case WL_DISCONNECTED:
    default:
      Serial.println("Wifi connection failed with status: " + String(wifiStatus));
DEFAULT_CASE:
      delay(10000);
      ESP.restart();
      break;
  }
}