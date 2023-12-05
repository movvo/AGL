// Library includes
#include <math.h>
<<<<<<< HEAD
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#define MAX_SIZE_BUFFER 10

// Interruption times and speed calculation variables for right servo.
volatile float currentInterruptionTimeR = 0;
volatile float pastInterruptionTimeR = 0;
volatile float deltaInterruptionTimeR = 0;
volatile float currentTime = 0;
=======

// Interruption times and speed calculation variables for right servo.
volatile double currentInterruptionTimeR = 0;
volatile double pastInterruptionTimeR = 0;
volatile double deltaInterruptionTimeR = 0;
volatile double currentTime = 0;
>>>>>>> development

int IN3_MR = 5;
int IN4_MR = 4;

int encoderR = 19; // Right encoder pin.
int rWheel = 11;   // PWM pin for right servo, right wheel.
<<<<<<< HEAD

float rFrequency = 0; // Interruption frequency for right wheel.
float Vr = 0;         // Linear Velocity Right.
float Wr = 0;

float N = 24.0;                                  // Reads encoder.
float gear = 74.83;                              // Gear ratio 74.83:1.
float diameter = 10;                              // Wheel diamenter in cm.
float length = 35;                                // Lenght between to wheels, necessary for two wheels speed control.
int tickCounter = 3;                              // Times per tick in encoder for which we'll calculate speed in order to reduce signal noise.
int CR = 0;                                       // Counter < tickCounter. If equal calculate speed.
int vecSize = 10;                                 // Vector size for rVector.
float rVector[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Mean time frequency calculation vector for reducing signal noise.
int pwr = 0;                                      // Variable for predicted PWM value for servo.
int encoderCounterFilter = 0;                     // Variable for filtering in vector size vecSize.
bool firstEncoderRead = true;                     // Boolean for first encoder read control.
int x = 0;
String ros2Data = "";
char buffer[MAX_SIZE_BUFFER];
=======

double rFrequency = 0; // Interruption frequency for right wheel.
double Wr = 0;         // Angular Velocity Right.
double Vr = 0;         // Linear Velocity Right.

double N = 24.0;                                  // Reads encoder.
double gear = 74.83;                              // Gear ratio 74.83:1.
float diameter = 10;                              // Wheel diamenter in cm.
float length = 35;                                // Lenght between to wheels, necessary for two wheels speed control.
int tickCounter = 3;                              // Times per tick in encoder for which we'll calculate speed in order to reduce signal noise.
int CR = 0;                                       // Counter < tickCounter. If equal calculate speed.
int vecSize = 10;                                 // Vector size for rVector.
float rVector[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Mean time frequency calculation vector for reducing signal noise.
int pwr = 0;                                      // Variable for predicted PWM value for servo.
int encoderCounterFilter = 0;                     // Variable for filtering in vector size vecSize.
bool firstEncoderRead = true;                     // Boolean for first encoder read control.

>>>>>>> development
void setup()
{

  // Keep an eye on the wiring as it can produce unexpected behavior. Poorly adjusted wires and so on.
  pinMode(encoderR, INPUT);
  pinMode(rWheel, OUTPUT);

  digitalWrite(encoderR, HIGH);
  digitalWrite(IN3_MR, HIGH);
  digitalWrite(IN4_MR, LOW);

  attachInterrupt(digitalPinToInterrupt(encoderR), REncoder, FALLING); // PIN Interruption set to detect a falling flank in encoderR pin, it'll activate REncoder function.
<<<<<<< HEAD
  Serial.begin(115200);  // Start of Serial communication.
  Serial.setTimeout(1);
                                               
=======
  Serial.begin(9600);                                                  // Start of Serial communication.
>>>>>>> development
}

void REncoder() // Interruption function for right wheel encoder.
{
  CR++;

  if (CR == tickCounter && firstEncoderRead == false)
  {
    float mean = 0;
    currentInterruptionTimeR = millis();
    deltaInterruptionTimeR = currentInterruptionTimeR - pastInterruptionTimeR; // Time difference between encoder reads/ticks.

    for (int i = 0; i < vecSize - 1; i++)
    {
      rVector[i] = rVector[i + 1]; // Vector filling for later mean calculation.
    }
    rVector[vecSize - 1] = deltaInterruptionTimeR; // Vector last value.

    if (encoderCounterFilter < vecSize)
    {
      encoderCounterFilter++;
      rFrequency = (1000) / deltaInterruptionTimeR; // Right wheel frequency.
    }
    else
    {
      for (int i = 0; i < vecSize; i++)
      {
        mean = rVector[i] + mean; // Vector's mean calculation.
      }
      mean = mean / vecSize;
      deltaInterruptionTimeR = mean; // Delta becomes the calculated mean in order to reduce noise.

      rFrequency = (1000) / deltaInterruptionTimeR; // Right wheel frequency.
    }

    pastInterruptionTimeR = currentInterruptionTimeR; // Past interruption time actualization.
    CR = 0;                                           // Counter for tickCounter reset.
  }

  if (CR == tickCounter && firstEncoderRead == true)
  {
    firstEncoderRead = false;
    CR = 0; // Counter for tickCounter reset.
  }
}

void loop()
{

<<<<<<< HEAD
  while (Serial.available() > 0){
    // Transform recieved serial with division by 100.0 .
    ros2Data = Serial.readString();
  }
  currentTime = millis();

  float realDelta = (currentTime - pastInterruptionTimeR);
=======
  currentTime = millis();

  double realDelta = (currentTime - pastInterruptionTimeR);
>>>>>>> development

  if (realDelta >= 8 * tickCounter) // At 0 velocity our frequency should be 0.
  {
    rFrequency = 0; // The longest elapsed time between reads is no more than 20ms, we put 24ms.
  }

  Wr = (tickCounter * ((2 * 3.141516) / N) * rFrequency) / gear; // Angular speed Rad/s.
  Vr = Wr * (diameter / 2);                                      // Linear speed cm/s.
<<<<<<< HEAD
  float rounded_downWr = floorf(Wr * 100);                       // Rounding to two decimals our speeds
  float rounded_downVr = floorf(Vr * 100);
  int intWr = (int)rounded_downWr;                               // Getting the integers from our speeds, should divide by 100 in ros code.
  int intVr = (int)rounded_downVr;

  // Objective velocity in rad/s with 0-6.17 range where ~4.2 is the value in which the servo is functional.
  // If velocity is set between [0;4.2] hardcode 0 pwm as the polinomic regression applied to predict pwm values from angular velocities cannot deal with servo behavior for low PWMs (no speed until 100 PWM).
  const char * ros2DataChar = ros2Data.c_str();   // Must transform string to const char * in order to use atof function.
  float ros2DataFloat = atof(ros2DataChar);       // Usage of atof necessary for getting the float value of the data recieved via serial from ros.
  
  sprintf(buffer, "%d", intWr);                   // Transforming the integer speed to char buffer in order to print it via serial to ros2.
  Serial.println(buffer);
   
  delay(150);    // Necessary for ros program to be able to write. Otherwise we'll lock the buffer while reading.
  float wt = ros2DataFloat/100.0;                 // Using our floating value recieved form ros to set the desired angular speed.
  
  if (wt >= 4.2)
  {
    // Angular velocity to PWM transformation via polinomial regression.
    float kp = 0.05;
    float ki = 1;
    float e = wt - Wr;
    float eintegral = eintegral + e * deltaInterruptionTimeR;
    float u = kp * e + ki * eintegral;
=======

  // Objective velocity in rad/s with 0-6.17 range where ~4.2 is the value in which the servo is functional.
  // If velocity is set between [0;4.2] hardcode 0 pwm as the polinomic regression applied to predict pwm values from angular velocities cannot deal with servo behavior for low PWMs (no speed until 100 PWM).
  double wt = 4.3;

  if (wt >= 4.2)
  {
    // Angular velocity to PWM transformation via polinomial regression.
    double kp = 1;
    double ki = 1;
    double e = wt - Wr;
    double eintegral = eintegral + e * deltaInterruptionTimeR;
    double u = kp * e + ki * eintegral;
>>>>>>> development
    
    if (u > 0)
    {
      pwr = (int)(11.5142749480926 * pow(u, 2) - 29.7178828449269 * u + 2.95629260770791); // Set the motor speed
    }
    else
    {
      pwr = 0;
    }

    if (pwr > 255)
      pwr = 255;
  }
  else
    pwr = 0;

  ////////////// Print angular velocities for plotting. //////////////

  //    for(int i = 0; i <= 255; i += 5){
  //      analogWrite(ruedaR,0);
  //      Wr = (contadorTicks*((2*3.141516)/N)*frecuenciaR)/gear;
  //      Serial.println(Wr);
  //      Serial.print(",");
  //      Serial.println(i);
  //      delay(1000);
  //    }

<<<<<<< HEAD
//  ////////////////////////////////////////////////////////////////////
=======
  ////////////////////////////////////////////////////////////////////

  Serial.print(wt);

  Serial.print(" ");

  Serial.print(Wr);

  Serial.println();
>>>>>>> development

  analogWrite(rWheel, pwr); // PWM applied to right servo.
}