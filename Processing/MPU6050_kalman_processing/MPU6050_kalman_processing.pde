/*
    MPU6050 Triple Axis Accelerometer & Gyroscope. Processing for MPU6050_kalman_processing.ino
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

import processing.serial.*;

Serial myPort;

// Data samples
int actualSample = 0;
int maxSamples = 300;
int sampleStep = 1;
boolean hasData = false;

// Charts
PGraphics pgChart;
int[] colors = { #ff4444, #33ff99, #5588ff };
String[] pyrSeries = { "Pitch", "Roll" };
String[] compareSeries = { "Normal", "Kalman" };
String[] xyzSeries = { "X", "Y", "Z" };

// Data for accelerometer Pitch, Roll
float[][] pyrValues = new float[2][maxSamples];
float[][] pyrValuesFiltered = new float[2][maxSamples];

// Data for compare
float[][] pitchValues = new float[2][maxSamples];
float[][] rollValues = new float[2][maxSamples];

// Data for acc / gyr
float[][] accValues = new float[3][maxSamples];
float[][] gyrValues = new float[3][maxSamples];

// Artificial Horizon
PGraphics pgArtificialHorizon;
PGraphics pgArtificialHorizonRing;
PImage imgArtificialHorizon;
PImage imgArtificialHorizonRing;
int ahWidth = 0;
int ahHeight = 0;
int ahDiameter = 0;
int ahRadius = 0;
float ahKappa = 0.5522847498;
float ahRadiusKappa = 0;

void setup ()
{
  size(1000, 550, P2D);
  background(0);

  // Init
  initArtificialHorizon();

  // Serial
  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.bufferUntil(10);
}

void drawChart(String title, String[] series, float[][] chart, int x, int y, int h, boolean symmetric, boolean fixed, int fixedMin, int fixedMax, int hlines) 
{
  int actualColor = 0;
  
  int maxA = 0;
  int maxB = 0;
  int maxAB = 0;
  
  int min = 0;
  int max = 0;
  int step = 0;
  int divide = 0;
 
  if (fixed)
  {
    min = fixedMin;
    max = fixedMax;
    step = hlines;
  } else
  {
    if (hlines > 2)
    {
      divide = (hlines - 2);
    } else
    {
      divide = 1;
    }
      
    if (symmetric)
    {
      maxA = (int)abs(getMin(chart));
      maxB = (int)abs(getMax(chart));
      maxAB = max(maxA, maxB);
      step = (maxAB * 2) / divide;
      min = -maxAB-step;
      max = maxAB+step;
    } else
    {
      min = (int)(getMin(chart));
      max = (int)(getMax(chart));
      
      if ((max >= 0) && (min <= 0)) step = (abs(min) + abs(max)) / divide; 
      if ((max < 0) && (min < 0)) step = abs(min - max) / divide; 
      if ((max > 0) && (min > 0)) step = (max - min) / divide; 
      
      if (divide > 1)
      {
        min -= step;
        max += step;
      }
    }
  }
  
  pgChart = createGraphics((maxSamples*sampleStep)+50, h+60);

  pgChart.beginDraw();

  // Draw chart area and title
  pgChart.background(0);
  pgChart.strokeWeight(1);
  pgChart.noFill();
  pgChart.stroke(50);
  pgChart.rect(0, 0, (maxSamples*sampleStep)+49, h+59);
  pgChart.text(title, ((maxSamples*sampleStep)/2)-(textWidth(title)/2)+40, 20);

  // Draw chart description
  String Description[] = new String[chart.length];
  int DescriptionWidth[] = new int[chart.length];
  int DesctiptionTotalWidth = 0;
  int DescriptionOffset = 0;

  for (int j = 0; j < chart.length; j++)
  {
     Description[j] = "  "+series[j]+" = ";
     DescriptionWidth[j] += textWidth(Description[j]+"+000.00");
     Description[j] += nf(chart[j][actualSample-1], 0, 2)+"  ";
     DesctiptionTotalWidth += DescriptionWidth[j];
  }

  actualColor = 0;

  for (int j = 0; j < chart.length; j++)
  {
    pgChart.fill(colors[actualColor]);
    pgChart.text(Description[j], ((maxSamples*sampleStep)/2)-(DesctiptionTotalWidth/2)+DescriptionOffset+40, h+50);
    DescriptionOffset += DescriptionWidth[j];
    actualColor++;
    if (actualColor >= colors.length) actualColor = 0;
  }

  // Draw H-Lines 
  pgChart.stroke(100);

  for (float t = min; t <= max; t=t+step)
  {
    float line = map(t, min, max, 0, h);
    pgChart.line(40, h-line+30, (maxSamples*sampleStep)+40, h-line+30);
    pgChart.fill(200, 200, 200);
    pgChart.textSize(12);
    pgChart.text(int(t), 5, h-line+34);
  }

  // Draw data series
  pgChart.strokeWeight(2);

  for (int i = 1; i < actualSample; i++)
  {
    actualColor = 0;

    for (int j = 0; j < chart.length; j++)
    {
      pgChart.stroke(colors[actualColor]);

      float d0 = chart[j][i-1];
      float d1 = chart[j][i];

      if (d0 < min) d0 = min;
      if (d0 > max) d0 = max;
      if (d1 < min) d1 = min;
      if (d1 > max) d1 = max;

      float v0 = map(d0, min, max, 0, h);
      float v1 = map(d1,   min, max, 0, h);

      pgChart.line(((i-1)*sampleStep)+40, h-v0+30, (i*sampleStep)+40, h-v1+30);

      actualColor++;

      if (actualColor >= colors.length) actualColor = 0;
    }
  }

  pgChart.endDraw();

  image(pgChart, x, y);
}

void initArtificialHorizon()
{
  imgArtificialHorizon = loadImage("artificialHorizon.png");
  imgArtificialHorizonRing = loadImage("artificialHorizonRing.png");
  ahWidth = imgArtificialHorizon.width - 20;
  ahHeight = imgArtificialHorizon.height - 20;
  ahDiameter = min(ahWidth, ahHeight);
  ahRadius =  ahDiameter / 2;
  ahRadiusKappa = ahRadius * ahKappa;
}

float getArtificialHorizon(float pitch)
{
  return -sin(pitch)*ahRadius;
}

void drawScale(float offset, float[][] pyr)
{
  float horizon;

  // Ground side
  horizon = getArtificialHorizon(radians(pyr[0][actualSample-1]) - offset * PI / 180);
  pgArtificialHorizon.noFill();
  pgArtificialHorizon.beginShape();
  pgArtificialHorizon.vertex(ahRadius, 0);
  pgArtificialHorizon.stroke(255);
  pgArtificialHorizon.strokeWeight(2);
  pgArtificialHorizon.bezierVertex(ahRadius, horizon * ahKappa, ahRadiusKappa, horizon, 0, horizon);
  pgArtificialHorizon.bezierVertex( -ahRadiusKappa, horizon, -ahRadius, horizon * ahKappa, -ahRadius, 0);
  pgArtificialHorizon.endShape();

  // Sky side
  horizon = getArtificialHorizon(radians(pyr[0][actualSample-1]) + offset * PI / 180);
  pgArtificialHorizon.noFill();
  pgArtificialHorizon.beginShape();
  pgArtificialHorizon.vertex(ahRadius, 0);
  pgArtificialHorizon.stroke(0);
  pgArtificialHorizon.strokeWeight(2);
  pgArtificialHorizon.bezierVertex(ahRadius, horizon * ahKappa, ahRadiusKappa, horizon, 0, horizon);
  pgArtificialHorizon.bezierVertex( -ahRadiusKappa, horizon, -ahRadius, horizon * ahKappa, -ahRadius, 0);
  pgArtificialHorizon.endShape();
}

void drawArtificialHorizon(int x, int y, float[][] pyr)
{
  pgArtificialHorizon = createGraphics(ahWidth, ahHeight);
  pgArtificialHorizonRing = createGraphics(ahWidth+20, ahHeight+20); 

  float horizon = getArtificialHorizon(radians(pyr[0][actualSample-1]));
  pgArtificialHorizon.clear();
  pgArtificialHorizon.beginDraw();

  // Ground
  pgArtificialHorizon.translate(ahRadius, ahRadius);
  pgArtificialHorizon.rotate(radians(-pyr[1][actualSample-1]));
  pgArtificialHorizon.strokeWeight(0);
  pgArtificialHorizon.fill(40, 40, 40);
  pgArtificialHorizon.arc(0.0, 0.0, ahDiameter, ahDiameter, 0, 2 * PI);

  // Sky
  pgArtificialHorizon.beginShape();
  pgArtificialHorizon.fill(200, 200, 250);
  pgArtificialHorizon.strokeWeight(2);
  pgArtificialHorizon.stroke(255);
  pgArtificialHorizon.vertex(-ahRadius, 0);
  pgArtificialHorizon.bezierVertex(-ahRadius, -ahRadius-20, ahRadius, -ahRadius-20, ahRadius, 0);
  pgArtificialHorizon.bezierVertex(ahRadius, horizon * ahKappa, ahRadiusKappa, horizon, 0, horizon);
  pgArtificialHorizon.bezierVertex(-ahRadiusKappa, horizon, -ahRadius, horizon * ahKappa, -ahRadius, 0);
  pgArtificialHorizon.endShape();

  // Scale
  drawScale(60, pyr);
  drawScale(50, pyr);
  drawScale(40, pyr);
  drawScale(30, pyr);
  drawScale(20, pyr);
  drawScale(10, pyr);

  pgArtificialHorizon.endDraw();

  image(pgArtificialHorizon, x+10, y+10);
  image(imgArtificialHorizon, x, y);

  // Draw ring
  pgArtificialHorizonRing.beginDraw();
  pgArtificialHorizonRing.clear();
  pgArtificialHorizonRing.translate(130,130);
  pgArtificialHorizonRing.rotate(radians(pyr[1][actualSample-1]));
  pgArtificialHorizonRing.image(imgArtificialHorizonRing, -130, -130);
  pgArtificialHorizonRing.endDraw();
  image(pgArtificialHorizonRing, x, y);
}

float getMin(float[][] chart)
{
  float minValue = 0;
  float[] testValues = new float[chart.length];
  float testMin = 0;

  for (int i = 0; i < actualSample; i++)
  {
    for (int j = 0; j < testValues.length; j++)
    {
      testValues[j] = chart[j][i];
    }
    
    testMin = min(testValues);
    
    if (i == 0)
    {
      minValue = testMin;
    } else
    {
      if (minValue > testMin) minValue = testMin;
    }
  }
 
  return ceil(minValue)-1; 
}

float getMax(float[][] chart)
{
  float maxValue = 0;
  float[] testValues = new float[chart.length];
  float testMax = 0;

  for (int i = 0; i < actualSample; i++)
  {
    for (int j = 0; j < testValues.length; j++)
    {
      testValues[j] = chart[j][i];
    }
    
    testMax = max(testValues);

    if (i == 0)
    {
      maxValue = testMax;
    } else
    {
      if (maxValue < testMax) maxValue = testMax;
    }
  }
 
  return ceil(maxValue); 
}

void draw() 
{
  if (!hasData) return;

  background(0);

  drawChart("Pitch [deg]", compareSeries, pitchValues, 10, 10, 200, true, true, -90, 90, 30);
  drawChart("Roll [deg]", compareSeries, rollValues, 10, 280, 200, true, true, -90, 90, 30);

  drawArtificialHorizon(370, 15, pyrValues);
  drawArtificialHorizon(370, 285, pyrValuesFiltered);
 
  drawChart("Accelerometer [m/s]", xyzSeries, accValues, 640, 10, 200, true, true, -25, 25, 5);
  drawChart("Gyroscope", xyzSeries, gyrValues, 640, 280, 200, true, true, -700, 700, 100);

}

void nextSample(float[][] chart)
{
    for (int j = 0; j < chart.length; j++)
    {
      float last = chart[j][maxSamples-1];

      for (int i = 1; i < maxSamples; i++)
      {
        chart[j][i-1] = chart[j][i];
      }

      chart[j][(maxSamples-1)] = last;
    }
}

void serialEvent (Serial myPort)
{
  String inString = myPort.readStringUntil(10);

  if (inString != null)
  {
    inString = trim(inString);
    String[] list = split(inString, ':');
    String testString = trim(list[0]);

    if (list.length != 10) return;

    // Fill Pitch & Roll
    pyrValues[0][actualSample] = (float(list[0]));
    pyrValues[1][actualSample] = (float(list[1]));

    // Fill Pitch & Roll (Filtered)
    pyrValuesFiltered[0][actualSample] = (float(list[2]));
    pyrValuesFiltered[1][actualSample] = (float(list[3]));

    // Fill Pitch compare
    pitchValues[0][actualSample] = (float(list[0]));
    pitchValues[1][actualSample] = (float(list[2]));

    // Fill Roll compare
    rollValues[0][actualSample] = (float(list[1]));
    rollValues[1][actualSample] = (float(list[3]));

    // Fill ACC compare
    accValues[0][actualSample] = (float(list[4]));
    accValues[1][actualSample] = (float(list[5]));
    accValues[2][actualSample] = (float(list[6]));

    // Fill GYR compare
    gyrValues[0][actualSample] = (float(list[7]));
    gyrValues[1][actualSample] = (float(list[8]));
    gyrValues[2][actualSample] = (float(list[9]));
    
    if (actualSample > 1)
    {
      hasData = true;
    }

    if (actualSample == (maxSamples-1))
    {
      nextSample(pyrValues);
      nextSample(pyrValuesFiltered);
      nextSample(pitchValues);
      nextSample(rollValues);
      nextSample(accValues);
      nextSample(gyrValues);
    } else
    {
      actualSample++;
    }
  }
}
