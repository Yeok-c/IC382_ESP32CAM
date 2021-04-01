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
int KALMAN = 0;
int servo_x = 0;
int servo_y = 0;
int x_init = 0;
int y_init = 0;

int INCREMENT = 36;
int servo_x_angle = 0;
int servo_y_angle = 0;

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
  size(800, 600);
  //println(Serial.list()); // Use this to print connected serial devices
  //serial = new Serial(this, Serial.list()[0], 115200); // Set this to your serial port obtained using the line above
  myPort = new Serial(this, "COM19", 115200); // if you have only ONE serial port active
  //myPort.bufferUntil('\n'); // Buffer until line feed

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
  /* Draw Graph */
  if (drawValues) {
    drawValues = false;
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

}

void drawGraph() {
  background(255); // White
  for (int i = 0; i < width; i++) {
    stroke(200); // Grey
    line(i*10, 0, i*10, height);
    line(0, i*10, width, i*10);
  }

  stroke(0); // Black
  for (int i = 1; i <= 3; i++)
    line(0, height/4*i, width, height/4*i); // Draw line, indicating -90 deg, 0 deg and 90 deg

  convert();
  drawAxisX();
  drawAxisY();
}


int getServoAngle(int pos_init, int pos){
  int angle = (pos-pos_init)/INCREMENT;
  return angle;
}

void textInc(int pos, String valname, float value) {
  fill(0);
  text(valname + ": " + value, 50, 100+pos*20);
}

void textInc(int pos, String valname, float value, int color_index) {
  fill(color_index);
  text(valname + ": " + value, 50, 100+pos*20);
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
