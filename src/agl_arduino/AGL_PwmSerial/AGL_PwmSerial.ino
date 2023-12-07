// Library includes
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#define MAX_SIZE_BUFFER 10

// Interruption times and speed calculation variables for right servo.
volatile float currentInterruptionTimeR = 0;
volatile float pastInterruptionTimeR = 0;
volatile float deltaInterruptionTimeR = 0;
volatile float currentTime = 0;
float wt = 0.0;   //With this declaration we should be able to keep previous target speed running in our arduino code.
bool ros2DataRead = false;    // Control of wt speed setting.

int IN3_MR = 5;
int IN4_MR = 4;

int encoderR = 19; // Right encoder pin.
int rWheel = 11;   // PWM pin for right servo, right wheel.

float rFrequency = 0; // Interruption frequency for right wheel.
float Wr = 0;         // Angular Velocity Right.
float Vr = 0;         // Linear Velocity Right.
int CRr = 0;                                       // Counter < tickCounter. If equal calculate speed.
int pwrR = 0;                                      // Variable for predicted PWM value for servo.
bool firstEncoderReadR = true;                     // Boolean for first encoder read control.
int encoderCounterFilterR = 0;                     // Variable for filtering in vector size vecSize.
float rVector[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Mean time frequency calculation vector for reducing signal noise.

// Interruption times and speed calculation variables for left servo.

volatile float currentInterruptionTimeL = 0;
volatile float pastInterruptionTimeL = 0;
volatile float deltaInterruptionTimeL = 0;

int IN1_ML = 7;
int IN2_ML = 6;

int encoderL = 3; 
int lWheel = 10;   

float lFrequency = 0; 
float Wl = 0;         
float Vl = 0;         
int CRl = 0;                                       
int pwrL = 0;                                      
bool firstEncoderReadL = true;                     
int encoderCounterFilterL = 0;                     
float lVector[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
float N = 24.0;                                  
float gear = 74.83;                              
float diameter = 10;                              
float length = 35;                                
int tickCounter = 3;                              
int vecSize = 10;                                 
int x = 0;
String ros2Data = "";
char buffer[MAX_SIZE_BUFFER];
void setup()
{

  // Keep an eye on the wiring as it can produce unexpected behavior. Poorly adjusted wires and so on.
  pinMode(encoderR, INPUT);
  pinMode(rWheel, OUTPUT);

  digitalWrite(encoderR, HIGH);
  digitalWrite(IN3_MR, HIGH);
  digitalWrite(IN4_MR, LOW);

  pinMode(encoderL, INPUT);
  pinMode(lWheel, OUTPUT);

  digitalWrite(encoderL, HIGH);
  digitalWrite(IN1_ML, LOW);  // Different order as the servo is rotated.
  digitalWrite(IN2_ML, HIGH);

  attachInterrupt(digitalPinToInterrupt(encoderR), REncoder, FALLING); // PIN Interruption set to detect a falling flank in encoderR pin, it'll activate REncoder function.
  attachInterrupt(digitalPinToInterrupt(encoderL), LEncoder, FALLING); 
  Serial.begin(115200);  // Start of Serial communication.
  Serial.setTimeout(1);
                                               
}

void REncoder() // Interruption function for right wheel encoder.
{
  CRr++;

  if (CRr == tickCounter && firstEncoderReadR == false)
  {
    float mean = 0;
    currentInterruptionTimeR = millis();
    deltaInterruptionTimeR = currentInterruptionTimeR - pastInterruptionTimeR; // Time difference between encoder reads/ticks.

    for (int i = 0; i < vecSize - 1; i++)
    {
      rVector[i] = rVector[i + 1]; // Vector filling for later mean calculation.
    }
    rVector[vecSize - 1] = deltaInterruptionTimeR; // Vector last value.

    if (encoderCounterFilterR < vecSize)
    {
      encoderCounterFilterR++;
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
    CRr = 0;                                           // Counter for tickCounter reset.
  }

  if (CRr == tickCounter && firstEncoderReadR == true)
  {
    firstEncoderReadR = false;
    CRr = 0; // Counter for tickCounter reset.
  }
}

void LEncoder() 
{
  CRl++;

  if (CRl == tickCounter && firstEncoderReadL == false)
  {
    float mean = 0;
    currentInterruptionTimeL = millis();
    deltaInterruptionTimeL = currentInterruptionTimeL - pastInterruptionTimeL; 

    for (int i = 0; i < vecSize - 1; i++)
    {
      lVector[i] = lVector[i + 1]; 
    }
    lVector[vecSize - 1] = deltaInterruptionTimeL; 

    if (encoderCounterFilterL < vecSize)
    {
      encoderCounterFilterL++;
      lFrequency = (1000) / deltaInterruptionTimeL;
    }
    else
    {
      for (int i = 0; i < vecSize; i++)
      {
        mean = lVector[i] + mean; 
      }
      mean = mean / vecSize;
      deltaInterruptionTimeL = mean; 

      lFrequency = (1000) / deltaInterruptionTimeL; 
    }

    pastInterruptionTimeL = currentInterruptionTimeL; 
    CRl = 0;                                           
  }

  if (CRl == tickCounter && firstEncoderReadL == true)
  {
    firstEncoderReadL = false;
    CRl = 0; 
  }
}

void loop()
{

  while (Serial.available() > 0){
    // Transform recieved serial with division by 100.0 .
    ros2Data = Serial.readString();
    ros2DataRead = true;
  }
  currentTime = millis();

  float realDeltaR = (currentTime - pastInterruptionTimeR);
  float realDeltaL = (currentTime - pastInterruptionTimeL);

  if (realDeltaR >= 8 * tickCounter) // At 0 velocity our frequency should be 0.
  {
    rFrequency = 0; // The longest elapsed time between reads is no more than 20ms, we put 24ms.
  }

  if (realDeltaL >= 8 * tickCounter) 
  {
    lFrequency = 0; 
  }

  Wr = (tickCounter * ((2 * 3.141516) / N) * rFrequency) / gear; // Angular speed Rad/s.
  Vr = Wr * (diameter / 2);                                      // Linear speed cm/s.

  Wl = (tickCounter * ((2 * 3.141516) / N) * lFrequency) / gear; 
  Vl = Wl * (diameter / 2);    

  float rounded_downWr = floorf(Wr * 100);                       // Rounding to two decimals our speeds
  float rounded_downVr = floorf(Vr * 100);
  int intWr = (int)rounded_downWr;                               // Getting the integers from our speeds, should divide by 100 in ros code.
  int intVr = (int)rounded_downVr;

  float rounded_downWl = floorf(Wl * 100);                       
  float rounded_downVl = floorf(Vl * 100);
  int intWl = (int)rounded_downWl;                               
  int intVl = (int)rounded_downVl;

  // Objective velocity in rad/s with 0-6.17 range where ~4.2 is the value in which the servo is functional.
  // If velocity is set between [0;4.2] hardcode 0 pwm as the polinomic regression applied to predict pwm values from angular velocities cannot deal with servo behavior for low PWMs (no speed until 100 PWM).
  const char * ros2DataChar = ros2Data.c_str();   // Must transform string to const char * in order to use atof function.
  float ros2DataFloat = atof(ros2DataChar);       // Usage of atof necessary for getting the float value of the data recieved via serial from ros.
  
  sprintf(buffer, "%d ", intWr);                   // Transforming the integer speed to char buffer in order to print it via serial to ros2.
  Serial.print(buffer);

  sprintf(buffer, "%d ", intVr);                   // Transforming the integer speed to char buffer in order to print it via serial to ros2.
  Serial.print(buffer);

  sprintf(buffer, "%d ", intWl);                   
  Serial.print(buffer);

  sprintf(buffer, "%d", intVl);                   
  Serial.println(buffer);
   
  delay(150);    // Necessary for ros program to be able to write. Otherwise we'll lock the buffer while reading.
  
  if(ros2DataRead){
    wt = ros2DataFloat/100.0;                 // Using our floating value recieved form ros to set the desired angular speed.
    ros2DataRead = false;
  }
  else{
    wt = 0;
  }
  
  
  if (wt >= 4.2)
  {
    // Angular velocity to PWM transformation via polinomial regression.
    float kpR = 0.05;
    float kiR = 1;
    float eR = wt - Wr;
    float eintegralR = eintegralR + eR * deltaInterruptionTimeR;
    float uR = kpR * eR + kiR * eintegralR;

    float kpL = 0.05;
    float kiL = 1;
    float eL = wt - Wl;
    float eintegralL = eintegralL + eL * deltaInterruptionTimeL;
    float uL = kpL * eL + kiL * eintegralL;

    if (uR > 0)
    {
      pwrR = (int)(11.5142749480926 * pow(uR, 2) - 29.7178828449269 * uR + 2.95629260770791); // Set the motor speed
    }
    else
    {
      pwrR = 0;
    }

    if (pwrR > 255)
      pwrR = 255;

    if (uL > 0)
    {
      pwrL = (int)(11.5142749480926 * pow(uL, 2) - 29.7178828449269 * uL + 2.95629260770791); // Inspect possibility of modeling servo behavior as it could be different than the other.
    }
    else
    {
      pwrL = 0;
    }

    if (pwrL > 255)
      pwrL = 255;
  }
  else
  {
    pwrR = 0;
    pwrL = 0;
  }
    
  ////////////// Print angular velocities for plotting. //////////////

  //    for(int i = 0; i <= 255; i += 5){
  //      analogWrite(ruedaR,0);
  //      Wr = (contadorTicks*((2*3.141516)/N)*frecuenciaR)/gear;
  //      Serial.println(Wr);   // Cambiar el Wr por el Wl cuando sea necesario.
  //      Serial.print(",");
  //      Serial.println(i);
  //      delay(1000);
  //    }

//  ////////////////////////////////////////////////////////////////////

  //analogWrite(rWheel, pwrR-L);

  analogWrite(rWheel, pwrR); // PWM applied to right servo.
  analogWrite(lWheel, pwrL); // PWM applied to left servo.
}