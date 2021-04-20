
#define TIMER_WIDTH 16
#define INCREMENT 36
#include "esp32-hal-ledc.h"
int servo_x = 0;
int servo_y = 0;
int x_init = 4400;
int y_init = 4400;
float bias_pitch = 0;
float bias_roll = 0;
float bias_g_pitch = 0;
float bias_g_roll = 0;
int MINTIME = 0;

unsigned long last_time = 0;
int SDAPin = 13;
int SCLPin = 15; 

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>

//TJK's Library integrates both Gyro + Accel data during the Kalman Filtering Process
//It doesn't work for me and without understanding it I cannot fix it
//So my approach is to use the simplest Kalman Filter and integrate Gyro readings my way.
//#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
//Kalman kalmanX; // Create the Kalman instances
//Kalman kalmanY;

float alpha = 0.75;
int KALMAN = 0;
bool MANUAL_OVERRIDE = 0;
uint32_t timer;

//float Estimate_Error = 0.35;
//float Measurement_Error = 0.6;
//float Process_Noise = 0.0148;


float Estimate_Error = 0.18;
float Measurement_Error = 0.01;
float Process_Noise = 0.0188;

//SimpleKalmanFilter xFilter(2, 2, 0.01);
//SimpleKalmanFilter yFilter(2, 2, 0.01);
SimpleKalmanFilter xFilter(Estimate_Error, Measurement_Error, Process_Noise);
SimpleKalmanFilter yFilter(Estimate_Error, Measurement_Error, Process_Noise);

Adafruit_MPU6050 mpu;
void IMU_setup(void);
void IMU_Test(void);


void setup() {
  Serial.begin(115200);

  //Setup IMU and bias
  IMU_setup();
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch;
  
  for(int i = 0; i< 50; i++){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //if chip is facing upwards
    ax = -a.acceleration.x;
    ay = a.acceleration.y;
    az = a.acceleration.z; 

    gx = -g.gyro.y * RAD_TO_DEG ;
    gy = g.gyro.x * RAD_TO_DEG;
    gz = g.gyro.z;

    roll  = atan2(ax, az) * RAD_TO_DEG;
    pitch = atan2(ay, az) * RAD_TO_DEG;

    bias_roll = xFilter.updateEstimate(roll);
    bias_pitch = yFilter.updateEstimate(pitch);
    bias_g_roll = gx;
    bias_g_pitch = gy;
    delay(5);
  }


  ledcSetup(1, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
  ledcSetup(2, 50, TIMER_WIDTH); // channel 1, 50 Hz, 16-bit width
  ledcAttachPin(2, 1);   // GPIO 22 assigned to channel 1
  ledcAttachPin(14, 2);       // GPIO 22 assigned to channel 1
  

  //8000 for 90 degree so
  servo_x = x_init;
  servo_y = y_init;
  ledcWrite(1, 4400);       // sweep servo 1
  ledcWrite(2, 4400);

  Serial.print("Setup:");
  Serial.print(bias_roll);
  Serial.print(" ");
  Serial.println(bias_pitch);
  Serial.print(" ");
  Serial.print(bias_g_roll);
  Serial.print(" ");
  Serial.println(bias_g_pitch);
  //-0.41 1.65
  delay(1000);
}

unsigned long last_loop_time = 0;

void loop() {
//  if(millis() - last_loop_time > 20){
    IMU_Test();
    last_loop_time = millis();
    //SERVO_Test();
// }
}


float comp_pitch = 0;
float comp_roll = 0;

void IMU_setup(void) {
  Wire.begin(SDAPin, SCLPin);
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin(0x68)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
//  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:    Serial.println("+-2G");    break;
    case MPU6050_RANGE_4_G:    Serial.println("+-4G");    break;
    case MPU6050_RANGE_8_G:    Serial.println("+-8G");    break;  
    case MPU6050_RANGE_16_G:    Serial.println("+-16G");    break;
  }
  
//  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:    Serial.println("+- 250 deg/s");    break;
    case MPU6050_RANGE_500_DEG:    Serial.println("+- 500 deg/s");    break;
    case MPU6050_RANGE_1000_DEG:    Serial.println("+- 1000 deg/s");    break;
    case MPU6050_RANGE_2000_DEG:    Serial.println("+- 2000 deg/s");    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:    Serial.println("260 Hz");    break;
    case MPU6050_BAND_184_HZ:    Serial.println("184 Hz");    break;
    case MPU6050_BAND_94_HZ:    Serial.println("94 Hz");    break;
    case MPU6050_BAND_44_HZ:    Serial.println("44 Hz");    break;
    case MPU6050_BAND_21_HZ:    Serial.println("21 Hz");    break;
    case MPU6050_BAND_10_HZ:    Serial.println("10 Hz");    break;
    case MPU6050_BAND_5_HZ:    Serial.println("5 Hz");    break;
  }

  Serial.println("");
  delay(100);
}
float pitch2 = 0;
float last_gyro_val = 0;
int gyro_count = 0;
unsigned long last_gyro_time = 0;
float last_roll;
float last_pitch;


void IMU_Test(void) {
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //if chip is facing downwards
  //ax = a.acceleration.x;
  //ay = a.acceleration.y;
  //az = -a.acceleration.z; 
  //gx = g.gyro.x;
  //gy = g.gyro.y;
  //gz = -g.gyro.z;
  
  //if chip is facing upwards
  ax = -a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z; 
  gx = -g.gyro.y * RAD_TO_DEG - bias_g_roll;
  gy = g.gyro.x * RAD_TO_DEG - bias_g_pitch;
  gz = g.gyro.z;

  roll  = atan2(ax, az) * RAD_TO_DEG;
  pitch = atan2(ay, az) * RAD_TO_DEG;

/*
  float gy_dt = micros() - last_gyro_time;
  pitch2 += (gx - last_gyro_val) / (gy_dt / 1000000);
  gyro_count++;
  if(gyro_count >= 2000){
    gyro_count = 0;
    pitch2 = 0;
  }
  last_gyro_val = gy;
  last_gyro_time = micros();
  */

  float dt = (micros() - last_gyro_time)/1000000;
  
  float kal_pitch = yFilter.updateEstimate(pitch) - bias_pitch;
  float kal_roll = xFilter.updateEstimate(roll) - bias_roll;
  comp_pitch = (1-alpha)*(comp_pitch + gy * dt) + (alpha)*(kal_pitch);
  comp_roll = (1-alpha)*(comp_roll + gx * dt) + (alpha)*(kal_roll);
  
  float final_pitch = 0;
  float final_roll = 0;
  if(KALMAN == 1) final_pitch = kal_pitch;
  else final_pitch = comp_pitch;
  if(KALMAN == 1) final_roll = kal_roll;
  else final_roll = comp_roll;
  last_gyro_time = micros();

  if(MANUAL_OVERRIDE == 0){

    //Limit roll and pitch so it doensn't hit anything
    if(final_roll > 20){
      final_roll = 20;
    }
    if(final_roll < -20){
      final_roll = -20;
    }
    
    if(final_pitch > 30){
      final_pitch = 30;
    }
    if(final_pitch < -30){
      final_pitch = -30;
    }
    
    //Travel in small step to not overcurrent
    int difference_roll= final_roll-last_roll; //positive is if current > last
    if(abs(difference_roll) > 5){
      if(difference_roll > 0) final_roll = last_roll + 5; //if difference is positive, add a bit to last and just move that increment
      if (difference_roll < 0) final_roll = last_roll - 5; //if difference i negative, deduct a bit to last and just move that increment
    }    
    servo_x = x_init - final_roll * INCREMENT;
    ledcWrite(1, servo_x);       

    //Travel in small step to not overcurrent
    int difference_pitch= final_pitch-last_pitch; //positive is if current > last
    if(abs(difference_pitch) > 5){
      if(difference_pitch > 0) final_pitch = last_pitch + 5; //if difference is positive, add a bit to last and just move that increment
      if (difference_pitch < 0) final_pitch = last_pitch - 5; //if difference i negative, deduct a bit to last and just move that increment
    }    
    servo_y = y_init - final_pitch * INCREMENT;
    ledcWrite(2, servo_y);       
  }
  last_roll = final_roll;
  last_pitch = final_pitch;

  Serial.print("Orientation: ");
  Serial.print(0);
  Serial.print(" ");
  Serial.print(final_pitch);
  Serial.print(" ");
  Serial.print(final_roll);
  Serial.print(" ");
  Serial.print(Measurement_Error, 4);
  Serial.print(" ");
  Serial.print(Estimate_Error, 4);
  Serial.print(" ");
  Serial.print(Process_Noise, 4);
  Serial.print(" ");
  Serial.print(KALMAN);
  Serial.print(" ");
  Serial.print(ax);
  Serial.print(" ");
  Serial.print(ay);
  Serial.print(" ");
  Serial.print(az);
  Serial.print(" ");
  Serial.print(gx);
  Serial.print(" ");
  Serial.print(gy);
  Serial.print(" ");
  Serial.print(0); //gz
  Serial.print(" ");
  Serial.print(comp_roll);
  Serial.print(" ");
  Serial.print(comp_pitch);
  Serial.print(" ");
  Serial.print(bias_roll);
  Serial.print(" ");
  Serial.print(bias_pitch);
  Serial.print(" ");
  Serial.println(alpha);

  Serial.print("Servos:"); 
  Serial.print(" ");
  Serial.print(servo_x); 
  Serial.print(" ");
  Serial.print(servo_y);
  Serial.print(" ");
  Serial.print(x_init); 
  Serial.print(" ");
  Serial.print(y_init);
  Serial.print(" ");
  Serial.println(MANUAL_OVERRIDE);

  char nRxData;
  if (Serial.available())    // Only begin if a byte is received from serial port.
  {
    nRxData = Serial.read();
    //Q_angle
    if (nRxData == '7' || nRxData == '4') {
      if(nRxData == '7') Measurement_Error += 0.005;
      if(nRxData == '4') Measurement_Error -= 0.005;
      xFilter.setMeasurementError(Measurement_Error);
      yFilter.setMeasurementError(Measurement_Error);
    }
    //Q_bias
    if (nRxData == '8' || nRxData == '5') {
      if(nRxData == '8') Estimate_Error += 0.005;
      if(nRxData == '5') Estimate_Error -= 0.005;
      xFilter.setEstimateError(Estimate_Error);
      yFilter.setEstimateError(Estimate_Error);
    }
    //R_measure
    if (nRxData == '9' || nRxData == '6') {
      if(nRxData == '9') Process_Noise += 0.001;
      if(nRxData == '6') Process_Noise -= 0.001;
      xFilter.setProcessNoise(Process_Noise);
      yFilter.setProcessNoise(Process_Noise);
    }
    if(nRxData == '0') {
      if(KALMAN == 1) KALMAN = 0;
      else KALMAN = 1;
    }
    
    if(MANUAL_OVERRIDE == 1){
      if(nRxData == 'a' || nRxData == 'A') servo_x-=INCREMENT;
      if(nRxData == 'd' || nRxData == 'D') servo_x+=INCREMENT;
      if(nRxData == 'w' || nRxData == 'W') servo_y+=INCREMENT;
      if(nRxData == 's' || nRxData == 'S') servo_y-=INCREMENT;
      if(nRxData == 'c' || nRxData == 'C') {
        y_init = servo_y;
        x_init = servo_x;
      }
      ledcWrite(1, servo_x);       
      ledcWrite(2, servo_y);       
    }

    if(nRxData == '.'){
      MANUAL_OVERRIDE ^= 1;
    }
    if(nRxData == '*') alpha += 0.05;
    if(nRxData == '/') alpha -= 0.05;
  }
}

//SimpleKalmanFilter::SimpleKalmanFilter(float mea_e, float est_e, float q)
