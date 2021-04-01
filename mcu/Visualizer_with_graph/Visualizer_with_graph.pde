import processing.serial.*;
Serial myPort;

float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;
float Q_angle = 0.0;
float Q_bias = 0.0;
float R_measure = 0.0;
float GyroX = 0.0;
float GyroY = 0.0;
float GyroZ = 0.0;
float AccX = 0.0;
float AccY = 0.0;
float AccZ = 0.0;
float OG_roll = 0.0;
float OG_pitch = 0.0;
float initial_roll = 0.0;
float initial_pitch = 0.0;
int KALMAN = 0;
int servo_x = 0;
int servo_y = 0;
int x_init = 0;
int y_init = 0;
int MINTIME = 0;

float INCREMENT = 36;
float servo_x_angle = 0;
float servo_y_angle = 0;

int MANUAL_OVERRIDE = 0;

final int width = 800;
final int height = 600;

float[] graph_gyroX = new float[width];
float[] graph_gyroY = new float[width];

float[] graph_accX = new float[width];
float[] graph_accY = new float[width];

float[] graph_servoX = new float[width];
float[] graph_servoY = new float[width];

float[] graph_roll = new float[width];
float[] graph_pitch = new float[width];

char lastKey = ' ';

boolean drawValues  = false;

void setup() {
  size(1600, 600, P3D);
  //println(Serial.list()); // Use this to print connected serial devices
  //serial = new Serial(this, Serial.list()[0], 115200); // Set this to your serial port obtained using the line above
  myPort = new Serial(this, "COM19", 115200); // if you have only ONE serial port active
  //myPort.bufferUntil('\n'); // Buffer until line feed

  textMode(SHAPE); // set text mode to shape
  for (int i = 0; i < width; i++) { // center all variables
    graph_gyroX[i] = height/2;
    graph_gyroY[i] = height/2;
    graph_accX[i] = height/2;
    graph_accY[i] = height/2;
    graph_servoX[i] = height/2;
    graph_servoY[i] = height/2;
    graph_roll[i] = height/2;
    graph_pitch[i] = height/2;
  }
  drawGraph(); // Draw graph at startup
}

void draw() {
  serialEvent();  // read and parse incoming serial message
  background(255); // set background to white
  lights();
  /* Draw Graph */
  if (drawValues) {
    drawValues = false;
    strokeWeight(1);
    drawGraph();
  }
  
  if(keyPressed) {
    if(lastKey != key){          
      myPort.write(key);
    }
    lastKey = key;
  } 
  else {  
    lastKey = ' ';
  }
  fill(0, 102, 153); textSize(18);
  int i = 0;

  textInc(i++, "pitch", pitch, graph_colors[GRAPH_PITCH][1]);
  textInc(i++, "roll", roll, graph_colors[GRAPH_ROLL][1]);
  textInc(i++, "servo_y angle", getServoAngle(y_init, servo_y), graph_colors[GRAPH_SERVOY][1]);
  textInc(i++, "servo_x angle", getServoAngle(x_init, servo_x), graph_colors[GRAPH_SERVOX][1]);
  //textInc(i++, "servo_y", servo_y, graph_colors[GRAPH_SERVOY][1]);
  //textInc(i++, "servo_x", servo_x, graph_colors[GRAPH_SERVOX][1]);
  textInc(i++, "Measurement_Error", Q_bias);
  textInc(i++, "Estimate_Error", Q_angle);
  textInc(i++, "Process_Noise", R_measure);
  textInc(i++, "KALMAN?", KALMAN);
  textInc(i++, "AccX", AccX, graph_colors[GRAPH_ACCX][1]);
  textInc(i++, "AccY", AccY,graph_colors[GRAPH_ACCY][1]);
  textInc(i++, "AccZ", AccZ);
  textInc(i++, "GyroX", GyroX, graph_colors[GRAPH_GYROX][1]);
  textInc(i++, "GyroY", GyroY, graph_colors[GRAPH_GYROY][1]);
  textInc(i++, "GyroZ", GyroZ);
  textInc(i++, "OG_roll", OG_roll);
  textInc(i++, "OG_pitch", OG_pitch);
  textInc(i++, "x_init", x_init);
  textInc(i++, "y_init", y_init);
  textInc(i++, "Manual Mode", MANUAL_OVERRIDE);
  textInc(i++, "intial_roll", initial_roll);
  textInc(i++, "initial_pitch", initial_pitch);

  translate(width*3/2, height/2); // set position to centre

  pushMatrix(); // begin object

  yaw = 270;
  float c1 = cos(radians(roll));
  float s1 = sin(radians(roll));
  float c2 = cos(radians(pitch));
  float s2 = sin(radians(pitch));
  float c3 = cos(radians(yaw));
  float s3 = sin(radians(yaw));
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0, 
    -s2, c1*c2, c2*s1, 0, 
    c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0, 
    0, 0, 0, 1);

  drawArduino();

  popMatrix(); // end of object
}

void drawGraph() {
  background(255); // White
  stroke(200); // Grey
  strokeWeight(1);
  for (int i = 0; i < width/2; i++) {
    //line(i*10, 0, i*10, height);
    line(0, i*10, width, i*10);
  }

  stroke(0); // Black
  for (int i = 1; i <= 3; i++)
    line(0, height/4*i, width, height/4*i); // Draw line, indicating -90 deg, 0 deg and 90 deg

  convert();
  drawAxisX();
  drawAxisY();
}


float getServoAngle(int pos_init, int pos){
  float angle = (pos-pos_init)/INCREMENT;
  return angle;
}

void textInc(int pos, String valname, float value) {
  fill(0);
  text(valname + ": " + value, width+50, 100+pos*20);
}

void textInc(int pos, String valname, float value, int color_index) {
  fill(color_index);
  text(valname + ": " + value, width+50, 100+pos*20);
}

void serialEvent()
{
  int newLine = 13; // new line character in ASCII
  String message;
  do {
    message = myPort.readStringUntil(newLine); // read from port until new line
    if (message != null) {
      String[] list = split(trim(message), " ");
      if (list.length >= 4 && list[0].equals("Orientation:")) {
        yaw = float(list[1]); // convert to float yaw
        pitch = float(list[2]); // convert to float pitch
        roll = float(list[3]); // convert to float roll

        Q_angle = float(list[4]); // convert to float pitch
        Q_bias = float(list[5]); // convert to float roll
        R_measure = float(list[6]); // convert to float roll
        
        KALMAN = int(list[7]); // convert to float roll        
        AccX = float(list[8]); // convert to float roll
        AccY = float(list[9]); // convert to float roll
        AccZ = float(list[10]); // convert to float roll
        
        GyroX = float(list[11]); // convert to float roll
        GyroY = float(list[12]); // convert to float roll
        GyroZ = float(list[13]); // convert to float roll
        OG_roll = float(list[14]); // convert to float roll
        OG_pitch = float(list[15]); // convert to float roll
        initial_roll = int(list[16]);
        initial_pitch = int(list[17]);
        MINTIME = int(list[18]); // convert to float roll
      }
      if (list.length >= 2 && list[0].equals("Servos:")) {
        servo_x = int(list[1]);
        servo_y = int(list[2]);
        x_init = int(list[3]);
        y_init = int(list[4]);
        servo_x_angle = -getServoAngle(x_init, servo_x); //Although its like when tilted right, rotate left, values are normalized for better visualization
        servo_y_angle = -getServoAngle(y_init, servo_y);
        MANUAL_OVERRIDE = int(list[5]); // convert to float roll
      }
    }
  } while (message != null);
  
  //myPort.clear(); // Clear buffer
  drawValues = true; // Draw the graph

  //printAxis(); // Used for debugging
}

void drawArduino()
{ 
  /* function contains shape(s) that are rotated with the IMU */
  stroke(0, 90, 90); // set outline colour to darker teal
  fill(0, 130, 130); // set fill colour to lighter teal
  box(300, 10, 200); // draw Arduino board base shape

  stroke(0); // set outline colour to black
  fill(80); // set fill colour to dark grey

  translate(60, -10, 90); // set position to edge of Arduino box
  box(170, 20, 10); // draw pin header as box

  translate(-20, 0, -180); // set position to other edge of Arduino box
  box(210, 20, 10); // draw other pin header as box
}

/*
void serialEvent (Serial serial) {
  // Get the ASCII strings:
  stringAccX = serial.readStringUntil('\t');
  stringGyroX = serial.readStringUntil('\t');
  stringCompX = serial.readStringUntil('\t');
  stringKalmanX = serial.readStringUntil('\t');

  serial.readStringUntil('\t'); // Ignore extra tab

  stringAccY = serial.readStringUntil('\t');
  stringGyroY = serial.readStringUntil('\t');
  stringCompY = serial.readStringUntil('\t');
  stringKalmanY = serial.readStringUntil('\t');

  serial.clear(); // Clear buffer
  drawValues = true; // Draw the graph

  //printAxis(); // Used for debugging
}

void printAxis() {
  print(stringGyroX);
  print(stringAccX);
  print(stringCompX);
  print(stringKalmanX);

  print('\t');

  print(stringGyroY);
  print(stringAccY);
  print(stringCompY);
  print(stringKalmanY);

  println();
}
*/
