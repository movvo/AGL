// Library includes
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#define MAX_SIZE_BUFFER 10

// Variables for recieving Serial data.
float lastSerialWrite = 0.0;
bool firstSerialWrite = true;
int noDataSerialReadCounter = 0;      // Counter for consecutive no data readings from serial, when equals 5 set servos to 0 pwm.
const char * ros2DataRightWheelChar;  // Variables for transforming recieved data from serial to float.
const char * ros2DataLeftWheelChar;
float ros2DataRightWheelFloat;        // Variables that store floating value for recieved angular speeds from ros via serial port.
float ros2DataLeftWheelFloat;

float eIntegralLimit = 50.0;

// Interruption times and speed calculation variables for right servo.
volatile float currentInterruptionTimeR = 0;
volatile float pastInterruptionTimeR = 0;
volatile float deltaInterruptionTimeR = 0;
float eintegralR = 0.0;
volatile float currentTime = 0;
float wtRightWheel = 0.0;   //With this declaration we should be able to keep previous target speed running in our arduino code.
float wtLeftWheel = 0.0;
bool ros2DataRightWheelRead = false;    // Control of wt speed setting.
bool ros2DataLeftWheelRead = false;
bool rightWheelGoingForward = true;    // Control of wt speed setting.
bool leftWheelGoingForward = true;

int IN3_MR = 5;
int IN4_MR = 4;

int encoderR = 19; // Right encoder pin.
int rWheel = 11;   // PWM pin for right servo, right wheel.

float rFrequency = 0; // Interruption frequency for right wheel.
float Wr = 0;         // Angular Velocity Right.
int CRr = 0;                                       // Counter < tickCounter. If equal calculate speed.
int pwrR = 0;                                      // Variable for predicted PWM value for servo.
bool firstEncoderReadR = true;                     // Boolean for first encoder read control.
int encoderCounterFilterR = 0;                     // Variable for filtering in vector size vecSize.
float rVector[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Mean time frequency calculation vector for reducing signal noise.

// Interruption times and speed calculation variables for left servo.

volatile float currentInterruptionTimeL = 0;
volatile float pastInterruptionTimeL = 0;
volatile float deltaInterruptionTimeL = 0;
float eintegralL = 0.0;
int IN1_ML = 7;
int IN2_ML = 6;

int encoderL = 3; 
int lWheel = 10;   

float lFrequency = 0; 
float Wl = 0;                 
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
String ros2DataRightWheel = "";
String ros2DataLeftWheel = "";
char buffer[MAX_SIZE_BUFFER];


int count = 0;

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
  Serial.begin(230400);  // Start of Serial communication.
  Serial.setTimeout(1);
                                               
}

void REncoder() // Interruption function for right wheel encoder.
{
  CRr++;

  if (CRr == tickCounter && firstEncoderReadR == false){
    float mean = 0;
    currentInterruptionTimeR = millis();
    deltaInterruptionTimeR = currentInterruptionTimeR - pastInterruptionTimeR; // Time difference between encoder reads/ticks.

    for (int i = 0; i < vecSize - 1; i++){
      rVector[i] = rVector[i + 1]; // Vector filling for later mean calculation.
    }
    rVector[vecSize - 1] = deltaInterruptionTimeR; // Vector last value.

    if (encoderCounterFilterR < vecSize){
      encoderCounterFilterR++;
      rFrequency = (1000) / deltaInterruptionTimeR; // Right wheel frequency.
    }
    else{
      for (int i = 0; i < vecSize; i++){
        mean = rVector[i] + mean; // Vector's mean calculation.
      }
      mean = mean / vecSize;
      deltaInterruptionTimeR = mean; // Delta becomes the calculated mean in order to reduce noise.

      rFrequency = (1000) / deltaInterruptionTimeR; // Right wheel frequency.
    }

    pastInterruptionTimeR = currentInterruptionTimeR; // Past interruption time actualization.
    CRr = 0;                                           // Counter for tickCounter reset.
  }

  if (CRr == tickCounter && firstEncoderReadR == true){
    firstEncoderReadR = false;
    CRr = 0; // Counter for tickCounter reset.
  }
}

void LEncoder() 
{
  CRl++;

  if (CRl == tickCounter && firstEncoderReadL == false){
    float mean = 0;
    currentInterruptionTimeL = millis();
    deltaInterruptionTimeL = currentInterruptionTimeL - pastInterruptionTimeL; 

    for (int i = 0; i < vecSize - 1; i++){
      lVector[i] = lVector[i + 1]; 
    }
    lVector[vecSize - 1] = deltaInterruptionTimeL; 

    if (encoderCounterFilterL < vecSize){
      encoderCounterFilterL++;
      lFrequency = (1000) / deltaInterruptionTimeL;
    }
    else{
      for (int i = 0; i < vecSize; i++){
        mean = lVector[i] + mean; 
      }
      mean = mean / vecSize;
      deltaInterruptionTimeL = mean; 

      lFrequency = (1000) / deltaInterruptionTimeL; 
    }

    pastInterruptionTimeL = currentInterruptionTimeL; 
    CRl = 0;                                           
  }

  if (CRl == tickCounter && firstEncoderReadL == true){
    firstEncoderReadL = false;
    CRl = 0; 
  }
}

void SerialReading()
{
  if(Serial.available() <= 0){
    noDataSerialReadCounter++;
  }
  else if(Serial.available() == Serial.availableForWrite()){
    while(Serial.available() > 0){
      char dumpAllSerialInfo = Serial.read();
    }
    Serial.flush();
  }
  else{
    noDataSerialReadCounter = 0;

      ros2DataRightWheel = Serial.readStringUntil('\n');
      ros2DataRightWheelRead = true;

    if (Serial.available() > 0){
      ros2DataLeftWheel = Serial.readStringUntil('\n');
      ros2DataLeftWheelRead = true;
    }
  }
}

void SerialReadingTimeout(){
  if(noDataSerialReadCounter < 600){

    // When data from servos have been read update the values of desired angular speeds, otherwhise keep previous value.  

    if(ros2DataRightWheelRead){
      ros2DataRightWheelChar = ros2DataRightWheel.c_str();   // Must transform string to const char * in order to use atof function.
      ros2DataRightWheelFloat = atof(ros2DataRightWheelChar);       // Usage of atof necessary for getting the float value of the data recieved via serial from ros.
      wtRightWheel = ros2DataRightWheelFloat/100.0;                 // Using our floating value recieved form ros to set the desired angular speed.
      ros2DataRightWheelRead = false;
    }

    if(ros2DataLeftWheelRead){
      ros2DataLeftWheelChar = ros2DataLeftWheel.c_str();   
      ros2DataLeftWheelFloat = atof(ros2DataLeftWheelChar);      
      wtLeftWheel = ros2DataLeftWheelFloat/100.0;    
      ros2DataLeftWheelRead = false;
    }
  }
  else{
    wtRightWheel = 0.0;
    wtLeftWheel = 0.0;
  }
}

void checkIfShouldWriteSerial(int intWr, int intWl)
{
  if(firstSerialWrite || ((currentTime - lastSerialWrite) >= 150)){
    firstSerialWrite = false;

    lastSerialWrite = currentTime;
    
    sprintf(buffer, "%d ", intWr);                   // Transforming the integer speed to char buffer in order to print it via serial to ros2.
    Serial.print(buffer);
  
    sprintf(buffer, "%d ", intWl);                   
    Serial.println(buffer);
  }
}

float checkIntegralLimitsWindup(float eIntegral)
{
  if (eIntegral > eIntegralLimit){
    eIntegral = eIntegralLimit;
  }
  else if (eIntegral < -eIntegralLimit){
    eIntegral = -eIntegralLimit;
  }
  return eIntegral;
}

void RightServoControllerPI()
{
  if (abs(wtRightWheel) >= 4.2 && Wr != 0){
    // Angular velocity to PWM transformation via polinomial regression.
    float kpR = 0.5;
    float kiR = 0.1;
    float eR = abs(wtRightWheel) - Wr;
    eintegralR = eintegralR + eR * deltaInterruptionTimeR;

    eintegralR = checkIntegralLimitsWindup(eintegralR);

    float uR = kpR * eR + kiR * eintegralR;

    if (uR > 0){
      pwrR = (int)(11.5142749480926 * pow(uR, 2) - 29.7178828449269 * uR + 2.95629260770791); // Set the motor speed
    }
    else{
      pwrR = 0;
    }

    if (pwrR > 255){
      pwrR = 255;
    }

    if (pwrR < 0){
      pwrR = 0;
    }
  }
  else if(abs(wtRightWheel) >= 4.2 && Wr == 0){
    // This condition serves as start motor instructions (we could have the situation where the previous speed is 0 if robot halted or no angular speed recieved --> controller PI not necessary)
    pwrR = (int)(11.5142749480926 * pow(abs(wtRightWheel), 2) - 29.7178828449269 * abs(wtRightWheel) + 2.95629260770791);
    if (pwrR > 255){
      pwrR = 255;
    }
    if (pwrR < 0){
      pwrR = 0;
    }
  }
  else if(abs(wtRightWheel) < 4.2 && abs(wtRightWheel) >= 0){
    pwrR = 0;
  }
  
  if(wtRightWheel < 0){
    // For values lower than zero we must change HIGH-LOW distribution of current servo, this implies that we'll be setting said distribution two times (for positive and negative desired speeds)
    // For the right wheel the distribution necessary to go backwards is as it follows:
    digitalWrite(IN3_MR, LOW);
    digitalWrite(IN4_MR, HIGH);
    rightWheelGoingForward = false;
  }
  else{
    // Default servo direction
    digitalWrite(IN3_MR, HIGH);
    digitalWrite(IN4_MR, LOW);
    rightWheelGoingForward = true;
  }
}

void LeftServoControllerPI(){  
  if (abs(wtLeftWheel) >= 4.2 && Wl != 0){
    // Angular velocity to PWM transformation via polinomial regression.
    float kpL = 0.5;
    float kiL = 0.1;
    float eL = abs(wtLeftWheel) - Wl;
    eintegralL = eintegralL + eL * deltaInterruptionTimeL;

    eintegralL = checkIntegralLimitsWindup(eintegralL);

    float uL = kpL * eL + kiL * eintegralL;

    if (uL > 0){
      pwrL = (int)(11.5142749480926 * pow(uL, 2) - 29.7178828449269 * uL + 2.95629260770791); // Inspect possibility of modeling servo behavior as it could be different than the other.
    }
    else{
      pwrL = 0;
    }

    if (pwrL > 255){
      pwrL = 255;
    }

    if (pwrL < 0){
      pwrL = 0;
    }
  }
  else if(abs(wtLeftWheel) >= 4.2 && Wl == 0){
    // This condition serves as start motor instructions (we could have the situation where the previous speed is 0 if robot halted or no angular speed recieved --> controller PI not necessary)
    pwrL = (int)(11.5142749480926 * pow(abs(wtLeftWheel), 2) - 29.7178828449269 * abs(wtLeftWheel) + 2.95629260770791);
    if (pwrL > 255){
      pwrL = 255;
    }
      
    if (pwrL < 0){
      pwrL = 0;
    }
  }
  else if (abs(wtLeftWheel) < 4.2 && abs(wtLeftWheel) >= 0){
    pwrL = 0;
  }
    
  
  if(wtLeftWheel < 0){
    digitalWrite(IN1_ML, HIGH);  
    digitalWrite(IN2_ML, LOW);
    leftWheelGoingForward = false;
  }
  else{
    digitalWrite(IN1_ML, LOW);  // Different order as the servo is rotated.
    digitalWrite(IN2_ML, HIGH);
    leftWheelGoingForward = true;
  }
}
void loop()
{
  // SERIAL COMS READ
  SerialReading();
  SerialReadingTimeout();
  
  // COMPUTE ENCODER CALC
  currentTime = millis();

  float realDeltaR = (currentTime - pastInterruptionTimeR);
  float realDeltaL = (currentTime - pastInterruptionTimeL);

  if (realDeltaR >= 8 * tickCounter){ // At 0 velocity our frequency should be 0.
    rFrequency = 0; // The longest elapsed time between reads is no more than 20ms, we put 24ms.
  }

  if (realDeltaL >= 8 * tickCounter){
    lFrequency = 0; 
  }
    

  Wr = (tickCounter * ((2 * 3.141516) / N) * rFrequency) / gear; // Angular speed Rad/s.
  Wl = (tickCounter * ((2 * 3.141516) / N) * lFrequency) / gear; 

  float rounded_downWr = floorf(Wr * 100);                       // Rounding to two decimals our speeds
  int intWr = (int)rounded_downWr;                               // Getting the integers from our speeds, should divide by 100 in ros code.

  float rounded_downWl = floorf(Wl * 100);                       
  int intWl = (int)rounded_downWl;   
                
  // LINEAR REGRESSION FOR PWM CALCULATION FOR DESIRED ANGULAR SPEEDS.
  RightServoControllerPI();
  LeftServoControllerPI();

  if(!rightWheelGoingForward){
    intWr = -intWr;
  }

  if(!leftWheelGoingForward){
    intWl = -intWl;
  }

  // SEND RPM TO SERIAL
  checkIfShouldWriteSerial(intWr, intWl);

  analogWrite(rWheel, pwrR); // PWM applied to right servo.
  analogWrite(lWheel, pwrL); // PWM applied to left servo.

  delay(10);
}