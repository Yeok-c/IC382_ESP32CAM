//convert all axis
final int minAngle = -60;
final int maxAngle = 60;
final int minGyro = -10;
final int maxGyro = 10;

void convert() {
  graph_gyroX[graph_gyroX.length - 1] = map(GyroX, minGyro, maxGyro, 0, height); // Convert to a float and map to the screen height, then save in buffer

  graph_gyroY[graph_gyroY.length - 1] = map(GyroY, minGyro, maxGyro, 0, height); // Convert to a float and map to the screen height, then save in buffer

  graph_accX[graph_accX.length - 1] = map(AccX, minAngle/6, maxAngle/6, 0, height); // Convert to a float and map to the screen height, then save in buffer

  graph_accY[graph_accY.length - 1] = map(AccY, minAngle/6, maxAngle/6, 0, height); // Convert to a float and map to the screen height, then save in buffer

  graph_servoX[graph_servoX.length - 1] = map((float)servo_x_angle, minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer

  graph_servoY[graph_servoY.length - 1] = map((float)servo_y_angle, minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer

  graph_roll[graph_roll.length - 1] = map(roll, minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer

  graph_pitch[graph_pitch.length - 1] = map(pitch, minAngle, maxAngle, 0, height); // Convert to a float and map to the screen height, then save in buffer
}
