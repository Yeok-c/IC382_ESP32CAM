//DIRECTLY CHECK THE ANGLE AND FIX THAT!
//ACW = Tighter = Lower.
/*
  ctrl+alt+b / cmd-shift-b / ctrl-shift-b Build Project
  cmd-shift-d / ctrl-shift-d Debug project
  ctrl+alt+u Upload Firmware
  ctrl+alt+s Open Serial Port Monitor
*/


#include <Arduino.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "I2C.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

int SDAPin = 13;
int SCLPin = 15;
int tune_value = 0;
int tune_mode = 0; 

int KALMAN = 0;

/* IMU Data */
unsigned long accX, accY, accZ;
unsigned long gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

// TODO: Make calibration routine

void setup() {
  Serial.begin(115200);
  Wire.begin(SDAPin, SCLPin);
  //unsigned long TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  unsigned int clockFrequency = 400000;
  Wire.setClock(clockFrequency);

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3] ;
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}


double Q_angle = 0.001;
double Q_bias = 0.003;
double R_measure = 0.03;

//double Q_angle = 0.001;
//double Q_bias = 0.01;
//double R_measure = 0.001;

char nRxData;

void loop() {
    /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  float roll  = atan2(accY, accZ) * RAD_TO_DEG;
  float pitch = atan2(accX, accZ) * RAD_TO_DEG;
  //double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  Serial.print(" ");
  Serial.print(accX, 5);
  Serial.print(" ");
  Serial.print(accY, 5);
  Serial.print(" ");
  Serial.print(accZ, 5);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(gyroXrate);
  Serial.print(" ");
  Serial.println(gyroYrate);

/*
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
        
        //Q_angle = 0.001;
        //Q_bias = 0.003;
        //R_measure = 0.03;

#if 0 // Set to 1 to activate graph.pde format
  Serial.print(roll); Serial.print("\t");
  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");
#endif


#if 1 // Set to 1 to activate Visualizer.pde format
  Serial.print("Orientation: ");
  Serial.print(0);
  Serial.print(" ");
  if(KALMAN == 1) Serial.print(kalAngleX);
  else Serial.print(compAngleX); 
  Serial.print(" ");
  if(KALMAN == 1) Serial.print(kalAngleY);
  else Serial.print(compAngleY); 
  Serial.print(" ");
  Serial.print(Q_bias*1000);
  Serial.print(" ");
  Serial.print(Q_angle*1000);
  Serial.print(" ");
  Serial.print(R_measure*1000);
  Serial.print(" ");
  Serial.print(KALMAN);
  Serial.print(" ");
  Serial.print(accX);
  Serial.print(" ");
  Serial.print(accY);
  Serial.print(" ");
  Serial.print(accZ);
  Serial.print(" ");
  Serial.print(gyroX);
  Serial.print(" ");
  Serial.print(gyroY);
  Serial.print(" ");
  Serial.print(gyroZ);
  Serial.print(" ");
  Serial.print(roll);
  Serial.print(" ");
  Serial.println(pitch);
#endif

  Serial.print("\r\n");

*/

  if (Serial.available())    // Only begin if a byte is received from serial port.
  {
    nRxData = Serial.read();
    //Q_angle
    if (nRxData == 'q' || nRxData == 'a') {
      if(nRxData == 'q') Q_angle += 0.0001;
      if(nRxData == 'a') Q_angle -= 0.0001;
      kalmanX.setQangle(Q_angle);
      kalmanY.setQangle(Q_angle);
    }
    //Q_bias
    if (nRxData == 'w' || nRxData == 's') {
      if(nRxData == 'w') Q_bias += 0.0001;
      if(nRxData == 's') Q_bias -= 0.0001;
      kalmanX.setQbias(Q_bias);
      kalmanY.setQbias(Q_bias);
    }
    //R_measure
    if (nRxData == 'e' || nRxData == 'd') {
      if(nRxData == 'e') R_measure += 0.0001;
      if(nRxData == 'd') R_measure -= 0.0001;
      kalmanX.setRmeasure(R_measure);
      kalmanY.setRmeasure(R_measure);
    }
    if(nRxData == 'c') {
      if(KALMAN == 1) KALMAN = 0;
      else KALMAN = 1;
    }
  }
  delay(1);
}
