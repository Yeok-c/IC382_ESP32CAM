#include "Arduino.h"
#include "esp_camera.h"
#include <WiFi.h>
#include <base64.h>
#include <HTTPClient.h>

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE  // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM

#include "camera_pins.h"

const char* ssid = "HMT-Freshmen";
const char* password = "stars15hmt";
const int baud = 115200;

void setup() {
  Serial.begin(baud);
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(psramFound()){
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  
  classifyImage();
}

void loop(){
    classifyImage();
}

void classifyImage() {
  
  unsigned long startCaptureTime = millis(); 
  // Capture picture
   camera_fb_t * fb = NULL;
   fb = esp_camera_fb_get();
   
   if(!fb) {
    Serial.println("Camera capture failed");
    return;
   }

  size_t size = fb->len;
  //bool jpeg_converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb);    
  String buffer = base64::encode((uint8_t *) fb->buf, fb->len);

  unsigned long endCaptureTime = millis();
  Serial.print("Time Taken for capture: ");
  Serial.println(endCaptureTime-startCaptureTime); 
  
  String payload = "{\"inputs\": [{ \"data\": {\"image\": {\"base64\": \"" + buffer + "\"}}}]}";

  buffer = "";
  unsigned long startTime = millis(); 
  Serial.println(payload);  // Uncomment this if you want to show the payload
  unsigned long endTime = millis();
  Serial.print("Time Taken for baud rate, ");
  Serial.print(baud);
  Serial.print(" : ");
  Serial.println(endTime-startTime); 
  esp_camera_fb_return(fb);

/*
  //SEND TO CLOUD
  // Generic model
  String model_id = "aaa03c23b3724a16a56b629203edc62c";

  HTTPClient http;
  http.begin("https://api.clarifai.com/v2/models/" + model_id + "/outputs");
  http.addHeader("Content-Type", "application/json");     
  http.addHeader("Authorization", "Key your_key"); 
  int response_code = http.POST(payload);

  //RECEIVED FROM CLOUD
  // Parse the json response: Arduino assistant
  const int jsonSize = JSON_ARRAY_SIZE(1) + JSON_ARRAY_SIZE(20) + 3*JSON_OBJECT_SIZE(1) + 6*JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + 20*JSON_OBJECT_SIZE(4) + 2*JSON_OBJECT_SIZE(6);
  DynamicJsonDocument doc(jsonSize);
  deserializeJson(doc, response);
  
  for (int i=0; i < 10; i++) {
    const name = doc["outputs"][0]["data"]["concepts"][i]["name"];
    const float p = doc["outputs"][0]["data"]["concepts"][i]["value"];
      
    Serial.println("=====================");
    Serial.print("Name:");
    Serial.println(name[i]);
    Serial.print("Prob:");
    Serial.println(p);
    Serial.println();
  }
  */
}
