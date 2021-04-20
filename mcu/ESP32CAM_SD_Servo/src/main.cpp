/*********
 * 
  Yeok 2021
  Modified version of Rui Santos' https://RandomNerdTutorials.com/esp32-cam-take-photo-save-microsd-card
  Notable changes:
  - Re-enabled brownout detector
  - Program captures 1 pic every second instead, no more deep sleeping
  - Some camera settings like brightness, awb are set so pictures are standard for CNN

*********/

#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory
#include "esp32-hal-ledc.h"

// define the number of bytes you want to access
#define EEPROM_SIZE 1

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5 //2 !
#define VSYNC_GPIO_NUM    25 //14!
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


int pictureNumber = 0;
void init_ESP32CAM(void);
void init_sd(void);
void take_and_save_picture(void);

#define TIMER_WIDTH 16
#define INCREMENT 36
unsigned long last_loop_time = 0;
int servo_x = 0;
int servo_y = 0;
int x_init = 4211;
int y_init = 4104;

int roll_upper_limit = 4811;
int roll_lower_limit = 3611;
int pitch_upper_limit = 4504;
int pitch_lower_limit = 3704;

int final_roll = 0;
int final_pitch = 0;
int roll_stage = 0;
int pitch_stage = 0;


void setup() {
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  Serial.begin(115200);
  //Serial.setDebugOutput(true);        
  //Serial.println();
  
  init_ESP32CAM();
  init_sd();
  //Serial.println("Starting SD Card");
  
  ledcSetup(1, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
  ledcSetup(2, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
  ledcAttachPin(2, 1);   // GPIO 22 assigned to channel 1
  ledcAttachPin(14, 2);       // GPIO 22 assigned to channel 1
  final_roll = x_init;
  final_pitch = y_init;
  ledcWrite(1, final_roll);       // sweep servo 1
  ledcWrite(2, final_pitch);
  delay(1000);
  //ledcDetachPin(2); 
  //ledcDetachPin(14);  
}


void loop() {
  int picture_flag = 0;
  /*
  if(millis() - last_loop_time > 50){

    ledcDetachPin(2); 
    ledcDetachPin(14);  

    //init_ESP32CAM();
    //delay(50);

    //take_and_save_picture();
    //delay(50);
    
    ledcSetup(1, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
    ledcSetup(2, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
    ledcAttachPin(2, 1);   // GPIO 22 assigned to channel 1
    ledcAttachPin(14, 2);       // GPIO 22 assigned to channel 1
    
    last_loop_time = millis();
  }
  */

  delay(3);
  Serial.print(final_roll);   Serial.print(" ");   Serial.print(roll_stage);  Serial.print(" ");  
  Serial.print(final_pitch);  Serial.print(" ");   Serial.println(pitch_stage);
  
  switch(roll_stage){
    case 0:
    final_roll +=2;
    if(final_roll > roll_upper_limit){
      roll_stage = 1;
    }
    break;
    case 1:
    final_roll -= 1;
    if(final_roll < roll_lower_limit){
      roll_stage = 0;
    }
    break;
  }

  switch(pitch_stage){
    case 0:
    final_pitch += 2;
    if(final_pitch > pitch_upper_limit){
      pitch_stage = 1;
    }
    break;

    case 1:
    final_pitch -= 3;
    if(final_pitch < pitch_lower_limit){
      pitch_stage = 0;
    }
    break;
  }

  if(picture_flag == 0){
        ledcWrite(1, final_roll);       
        ledcWrite(2, final_pitch);
  }
}

void init_ESP32CAM(){
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
    
    config.frame_size = FRAMESIZE_QVGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
    

  /*
    if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
    } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    }
    */

    // Init Camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      return;
    }

    sensor_t * s = esp_camera_sensor_get();
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 1);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
}

void init_sd(void){
  if(!SD_MMC.begin()){
    Serial.println("SD Card Mount Failed");
    return;
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return;
  }
}

void take_and_save_picture(){
  camera_fb_t * fb = NULL;
  
  // Take Picture with Camera
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0) + 1;

  // Path where new picture will be saved in SD Card
  String path = "/picture" + String(pictureNumber) +".jpg";

  fs::FS &fs = SD_MMC; 
  Serial.printf("Picture file name: %s\n", path.c_str());
  
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file in writing mode");
  } 
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
  }
  file.close();
  esp_camera_fb_return(fb); 
}