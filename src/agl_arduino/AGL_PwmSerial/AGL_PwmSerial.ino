// Library includes
#include <math.h>

volatile double currentTime = 0;

// Interruption times and speed calculation variables for right servo.

volatile double currentInterruptionTimeR = 0;
volatile double pastInterruptionTimeR = 0;
volatile double deltaInterruptionTimeR = 0;

int IN3_MR = 5;
int IN4_MR = 4;

int encoderR = 19; // Right encoder pin.
int rWheel = 11;   // PWM pin for right servo, right wheel.

double rFrequency = 0; // Interruption frequency for right wheel.
double Wr = 0;         // Angular Velocity Right.
double Vr = 0;         // Linear Velocity Right.
int CRr = 0;                                       // Counter < tickCounter. If equal calculate speed.
int pwrR = 0;                                      // Variable for predicted PWM value for servo.
bool firstEncoderReadR = true;                     // Boolean for first encoder read control.
int encoderCounterFilterR = 0;                     // Variable for filtering in vector size vecSize.
float rVector[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Mean time frequency calculation vector for reducing signal noise.

// Interruption times and speed calculation variables for left servo.

volatile double currentInterruptionTimeL = 0;
volatile double pastInterruptionTimeL = 0;
volatile double deltaInterruptionTimeL = 0;

int IN1_ML = 7;
int IN2_ML = 6;

int encoderL = 3; 
int lWheel = 10;   

double lFrequency = 0; 
double Wl = 0;         
double Vl = 0;         
int CRl = 0;                                       
int pwrL = 0;                                      
bool firstEncoderReadL = true;                     
int encoderCounterFilterL = 0;                     
float lVector[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
double N = 24.0;                                  
double gear = 74.83;                              
float diameter = 10;                              
float length = 35;                                
int tickCounter = 3;                              
int vecSize = 10;                                 

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

  digitalWrite(encoderR, HIGH);
  digitalWrite(IN1_ML, LOW);  // Different order as the servo is rotated.
  digitalWrite(IN2_ML, HIGH);

  attachInterrupt(digitalPinToInterrupt(encoderR), REncoder, FALLING); // PIN Interruption set to detect a falling flank in encoderR pin, it'll activate REncoder function.
  attachInterrupt(digitalPinToInterrupt(encoderL), LEncoder, FALLING); 
  Serial.begin(9600);                                                  // Start of Serial communication.
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

  if (CRl == tickCounter && firstEncoderReadR == false)
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

  currentTime = millis();

  double realDeltaR = (currentTime - pastInterruptionTimeR);
  double realDeltaL = (currentTime - pastInterruptionTimeL);

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

  // Objective velocity in rad/s with 0-6.17 range where ~4.2 is the value in which the servo is functional.
  // If velocity is set between [0;4.2] hardcode 0 pwm as the polinomic regression applied to predict pwm values from angular velocities cannot deal with servo behavior for low PWMs (no speed until 100 PWM).
  double wt = 5;

  if (wt >= 4.2)
  {
    // Angular velocity to PWM transformation via polinomial regression.
    double kpR = 0.05;
    double kiR = 1;
    double eR = wt - Wr;
    double eintegralR = eintegralR + eR * deltaInterruptionTimeR;
    double uR = kpR * eR + kiR * eintegralR;

    double kpL = 0.05;
    double kiL = 1;
    double eL = wt - Wl;
    double eintegralL = eintegralL + eL * deltaInterruptionTimeL;
    double uL = kpL * eL + kiL * eintegralL;

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
    pwrR = 0;
    pwrL = 0;

  ////////////// Print angular velocities for plotting. //////////////

  //    for(int i = 0; i <= 255; i += 5){
  //      analogWrite(ruedaR,0);
  //      Wr = (contadorTicks*((2*3.141516)/N)*frecuenciaR)/gear;
  //      Serial.println(Wr);   // Cambiar el Wr por el Wl cuando sea necesario.
  //      Serial.print(",");
  //      Serial.println(i);
  //      delay(1000);
  //    }

  ////////////////////////////////////////////////////////////////////

  Serial.print(wt);

  Serial.print(" ");

  Serial.print(Wr);
  
  Serial.print(" ");

  Serial.print(Wl);

  Serial.println();

  /////////////////// Getting the information sent by ros2 node. ///////

  // send data only when you receive data:
  //  if (Serial.available() > 0) {
  //    // read the incoming byte:
  //    float incomingByte = Serial.parseFloat();
  //
  //    // say what you got:
  //    //Serial.print("I received: ");
  //    Serial.println(incomingByte, 2);
  //  }

  ////////////////////////////////////////////////////////////////////

  analogWrite(rWheel, pwrR); // PWM applied to right servo.
  analogWrite(lWheel, pwrL); // PWM applied to left servo.
}