int GRAPH_GYROX = 1;
int GRAPH_ACCX = 2;
int GRAPH_SERVOX = 3;
int GRAPH_PITCH = 4;

int GRAPH_GYROY = 5;
int GRAPH_ACCY = 6;
int GRAPH_SERVOY = 7;
int GRAPH_ROLL = 8;

int graph_colors[][] = { // 10 Freqs, 2 Columns each: freq, color
  {0, #000000}, 
  {GRAPH_GYROX, #8931EF}, 
  {GRAPH_ACCX, #F2CA19}, 
  {GRAPH_SERVOX, #FF00BD}, 
  {GRAPH_PITCH, #0057E9}, 
  {GRAPH_GYROY, #87E911}, 
  {GRAPH_ACCY, #E11845}, 
  {GRAPH_SERVOY, #40A0D0}, 
  {GRAPH_ROLL, #6495ED}, 
  {9, #CCCCFF}, 
  {10, #ABB2B9}, 
};

void drawAxisX() {
  /* Draw gyro x-axis */
  noFill();
  stroke(graph_colors[GRAPH_GYROX][1]); // Yellow
  // Redraw everything
  beginShape();
  vertex(0, graph_gyroX[0]);
  for (int i = 1; i < graph_gyroX.length; i++) {
    if ((graph_gyroX[i] < height/4 && graph_gyroX[i - 1] > height/4*3) || (graph_gyroX[i] > height/4*3 && graph_gyroX[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, graph_gyroX[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < graph_gyroX.length;i++)
    graph_gyroX[i-1] = graph_gyroX[i];

  /* Draw acceleromter x-axis */
  noFill();
  stroke(graph_colors[GRAPH_ACCX][1]); // Yellow
  // Redraw everything
  beginShape();
  vertex(0, graph_accX[0]);
  for (int i = 1; i < graph_accX.length; i++) {
    if ((graph_accX[i] < height/4 && graph_accX[i - 1] > height/4*3) || (graph_accX[i] > height/4*3 && graph_accX[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, graph_accX[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < graph_accX.length;i++)
    graph_accX[i-1] = graph_accX[i];


  /* Draw complementary filter x-axis */
  noFill();
  stroke(graph_colors[GRAPH_SERVOX][1]); // Yellow
  // Redraw everything
  beginShape();
  vertex(0, graph_servoX[0]);
  for (int i = 1; i < graph_servoX.length; i++) {
    if ((graph_servoX[i] < height/4 && graph_servoX[i - 1] > height/4*3) || (graph_servoX[i] > height/4*3 && graph_servoX[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, graph_servoX[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < graph_servoX.length; i++)
    graph_servoX[i-1] = graph_servoX[i];
    

  /* Draw kalman filter x-axis */
  noFill();
  stroke(graph_colors[GRAPH_ROLL][1]); // Yellow
  // Redraw everything
  beginShape();
  vertex(0, graph_roll[0]);
  for (int i = 1; i < graph_roll.length; i++) {
    if ((graph_roll[i] < height/4 && graph_roll[i - 1] > height/4*3) || (graph_roll[i] > height/4*3 && graph_roll[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, graph_roll[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < graph_roll.length; i++)
    graph_roll[i-1] = graph_roll[i];
}


void drawAxisY() {
  /* Draw gyro y-axis */
  noFill();
  stroke(graph_colors[GRAPH_GYROY][1]); // Yellow
  // Redraw everything
  beginShape();
  vertex(0, graph_gyroY[0]);
  for (int i = 1; i < graph_gyroY.length; i++) {
    if ((graph_gyroY[i] < height/4 && graph_gyroY[i - 1] > height/4*3) || (graph_gyroY[i] > height/4*3 && graph_gyroY[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, graph_gyroY[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < graph_gyroY.length;i++)
   graph_gyroY[i-1] = graph_gyroY[i];

  /* Draw acceleromter y-axis */
  noFill();
  stroke(graph_colors[GRAPH_ACCY][1]); // Yellow
  // Redraw everything
  beginShape();
  vertex(0, graph_accY[0]);
  for (int i = 1; i < graph_accY.length; i++) {
    if ((graph_accY[i] < height/4 && graph_accY[i - 1] > height/4*3) || (graph_accY[i] > height/4*3 && graph_accY[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, graph_accY[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < graph_accY.length;i++)
    graph_accY[i-1] = graph_accY[i];


  /* Draw complementary filter y-axis */
  noFill();
  stroke(graph_colors[GRAPH_SERVOY][1]); // Yellow
  // Redraw everything
  beginShape();
  vertex(0, graph_servoY[0]);
  for (int i = 1; i < graph_servoY.length; i++) {
    if ((graph_servoY[i] < height/4 && graph_servoY[i - 1] > height/4*3) || (graph_servoY[i] > height/4*3 && graph_servoY[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, graph_servoY[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i < graph_servoY.length;i++)
    graph_servoY[i-1] = graph_servoY[i];


  /* Draw kalman filter y-axis */
  noFill();
  stroke(graph_colors[GRAPH_PITCH][1]); // Yellow
  // Redraw everything
  beginShape();
  vertex(0, graph_pitch[0]);
  for (int i = 1; i < graph_pitch.length; i++) {
    if ((graph_pitch[i] < height/4 && graph_pitch[i - 1] > height/4*3) || (graph_pitch[i] > height/4*3 && graph_pitch[i - 1] < height/4)) {
      endShape();
      beginShape();
    }
    vertex(i, graph_pitch[i]);
  }
  endShape();

  // Put all data one array back
  for (int i = 1; i<graph_pitch.length;i++)
    graph_pitch[i-1] = graph_pitch[i];
}
