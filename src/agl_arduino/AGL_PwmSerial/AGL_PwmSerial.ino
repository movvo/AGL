// // Library includes
// #include <math.h>
// #include <stdlib.h>
// #include <stdio.h>
// #include <string.h>
// #define MAX_SIZE_BUFFER 10

// // Variables for recieving Serial data.
// float lastSerialWrite = 0.0;
// bool firstSerialWrite = true;
// int noDataSerialReadCounter = 0;      // Counter for consecutive no data readings from serial, when equals 5 set servos to 0 pwm.
// const char * ros2DataRightWheelChar;  // Variables for transforming recieved data from serial to float.
// const char * ros2DataLeftWheelChar;
// float ros2DataRightWheelFloat;        // Variables that store floating value for recieved angular speeds from ros via serial port.
// float ros2DataLeftWheelFloat;

// float eIntegralLimit = 50.0;

// // Interruption times and speed calculation variables for right servo.
// volatile float currentInterruptionTimeR = 0;
// volatile float pastInterruptionTimeR = 0;
// volatile float deltaInterruptionTimeR = 0;
// float eintegralR = 0.0;
// volatile float currentTime = 0;
// float wtRightWheel = 0.0;   //With this declaration we should be able to keep previous target speed running in our arduino code.
// float wtLeftWheel = 0.0;
// bool ros2DataRightWheelRead = false;    // Control of wt speed setting.
// bool ros2DataLeftWheelRead = false;

// int IN3_MR = 5;
// int IN4_MR = 4;

// int encoderR = 19; // Right encoder pin.
// int rWheel = 11;   // PWM pin for right servo, right wheel.

// float rFrequency = 0; // Interruption frequency for right wheel.
// float Wr = 0;         // Angular Velocity Right.
// int CRr = 0;                                       // Counter < tickCounter. If equal calculate speed.
// int pwrR = 0;                                      // Variable for predicted PWM value for servo.
// bool firstEncoderReadR = true;                     // Boolean for first encoder read control.
// int encoderCounterFilterR = 0;                     // Variable for filtering in vector size vecSize.
// float rVector[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Mean time frequency calculation vector for reducing signal noise.

// // Interruption times and speed calculation variables for left servo.

// volatile float currentInterruptionTimeL = 0;
// volatile float pastInterruptionTimeL = 0;
// volatile float deltaInterruptionTimeL = 0;
// float eintegralL = 0.0;
// int IN1_ML = 7;
// int IN2_ML = 6;

// int encoderL = 3; 
// int lWheel = 10;   

// float lFrequency = 0; 
// float Wl = 0;                 
// int CRl = 0;                                       
// int pwrL = 0;                                      
// bool firstEncoderReadL = true;                     
// int encoderCounterFilterL = 0;                     
// float lVector[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
// float N = 24.0;                                  
// float gear = 74.83;                              
// float diameter = 10;                              
// float length = 35;                                
// int tickCounter = 3;                              
// int vecSize = 10;                                 
// int x = 0;
// String ros2DataRightWheel = "";
// String ros2DataLeftWheel = "";
// char buffer[MAX_SIZE_BUFFER];
// void setup()
// {

//   // Keep an eye on the wiring as it can produce unexpected behavior. Poorly adjusted wires and so on.
//   pinMode(encoderR, INPUT);
//   pinMode(rWheel, OUTPUT);

//   digitalWrite(encoderR, HIGH);
//   digitalWrite(IN3_MR, HIGH);
//   digitalWrite(IN4_MR, LOW);

//   pinMode(encoderL, INPUT);
//   pinMode(lWheel, OUTPUT);

//   digitalWrite(encoderL, HIGH);
//   digitalWrite(IN1_ML, LOW);  // Different order as the servo is rotated.
//   digitalWrite(IN2_ML, HIGH);

//   attachInterrupt(digitalPinToInterrupt(encoderR), REncoder, FALLING); // PIN Interruption set to detect a falling flank in encoderR pin, it'll activate REncoder function.
//   attachInterrupt(digitalPinToInterrupt(encoderL), LEncoder, FALLING); 
//   Serial.begin(115200);  // Start of Serial communication.
//   Serial.setTimeout(1);
                                               
// }

// void REncoder() // Interruption function for right wheel encoder.
// {
//   CRr++;

//   if (CRr == tickCounter && firstEncoderReadR == false)
//   {
//     float mean = 0;
//     currentInterruptionTimeR = millis();
//     deltaInterruptionTimeR = currentInterruptionTimeR - pastInterruptionTimeR; // Time difference between encoder reads/ticks.

//     for (int i = 0; i < vecSize - 1; i++)
//     {
//       rVector[i] = rVector[i + 1]; // Vector filling for later mean calculation.
//     }
//     rVector[vecSize - 1] = deltaInterruptionTimeR; // Vector last value.

//     if (encoderCounterFilterR < vecSize)
//     {
//       encoderCounterFilterR++;
//       rFrequency = (1000) / deltaInterruptionTimeR; // Right wheel frequency.
//     }
//     else
//     {
//       for (int i = 0; i < vecSize; i++)
//       {
//         mean = rVector[i] + mean; // Vector's mean calculation.
//       }
//       mean = mean / vecSize;
//       deltaInterruptionTimeR = mean; // Delta becomes the calculated mean in order to reduce noise.

//       rFrequency = (1000) / deltaInterruptionTimeR; // Right wheel frequency.
//     }

//     pastInterruptionTimeR = currentInterruptionTimeR; // Past interruption time actualization.
//     CRr = 0;                                           // Counter for tickCounter reset.
//   }

//   if (CRr == tickCounter && firstEncoderReadR == true)
//   {
//     firstEncoderReadR = false;
//     CRr = 0; // Counter for tickCounter reset.
//   }
// }

// void LEncoder() 
// {
//   CRl++;

//   if (CRl == tickCounter && firstEncoderReadL == false)
//   {
//     float mean = 0;
//     currentInterruptionTimeL = millis();
//     deltaInterruptionTimeL = currentInterruptionTimeL - pastInterruptionTimeL; 

//     for (int i = 0; i < vecSize - 1; i++)
//     {
//       lVector[i] = lVector[i + 1]; 
//     }
//     lVector[vecSize - 1] = deltaInterruptionTimeL; 

//     if (encoderCounterFilterL < vecSize)
//     {
//       encoderCounterFilterL++;
//       lFrequency = (1000) / deltaInterruptionTimeL;
//     }
//     else
//     {
//       for (int i = 0; i < vecSize; i++)
//       {
//         mean = lVector[i] + mean; 
//       }
//       mean = mean / vecSize;
//       deltaInterruptionTimeL = mean; 

//       lFrequency = (1000) / deltaInterruptionTimeL; 
//     }

//     pastInterruptionTimeL = currentInterruptionTimeL; 
//     CRl = 0;                                           
//   }

//   if (CRl == tickCounter && firstEncoderReadL == true)
//   {
//     firstEncoderReadL = false;
//     CRl = 0; 
//   }
// }

// void SerialReading()
// {
//   if(Serial.available() <= 0)
//   {
//     noDataSerialReadCounter++;
//   }
//   else{
//     noDataSerialReadCounter = 0;

//       ros2DataRightWheel = Serial.readStringUntil('\n');
//       Serial.println("AQUI: "+ros2DataRightWheel);
//       ros2DataRightWheelRead = true;

//     if (Serial.available() > 0){
//       ros2DataLeftWheel = Serial.readStringUntil('\n');
//       ros2DataLeftWheelRead = true;
//     }
//   }
// }

// void SerialReadingTimeout(){
//   if(noDataSerialReadCounter < 10){

//     // When data from servos have been read update the values of desired angular speeds, otherwhise keep previous value.  

//     if(ros2DataRightWheelRead){
//       Serial.println("ENTRAMOS Right");
//       ros2DataRightWheelChar = ros2DataRightWheel.c_str();   // Must transform string to const char * in order to use atof function.
//       ros2DataRightWheelFloat = atof(ros2DataRightWheelChar);       // Usage of atof necessary for getting the float value of the data recieved via serial from ros.
//       wtRightWheel = ros2DataRightWheelFloat/100.0;                 // Using our floating value recieved form ros to set the desired angular speed.
//       ros2DataRightWheelRead = false;
//       Serial.println(wtRightWheel);
//     }

//     if(ros2DataLeftWheelRead){
//       ros2DataLeftWheelChar = ros2DataLeftWheel.c_str();   
//       ros2DataLeftWheelFloat = atof(ros2DataLeftWheelChar);      
//       wtLeftWheel = ros2DataLeftWheelFloat/100.0;    
//       ros2DataLeftWheelRead = false;
//     }
//   }
//   else{
//     wtRightWheel = 0.0;
//     wtLeftWheel = 0.0;
//   }
// }

// void checkIfShouldWriteSerial(int intWr, int intWl){
//   if(firstSerialWrite || (currentTime - lastSerialWrite) >= 150){
//     firstSerialWrite = false;

//     lastSerialWrite = currentTime;
    
//     sprintf(buffer, "%d ", intWr);                   // Transforming the integer speed to char buffer in order to print it via serial to ros2.
//     Serial.print(buffer);
  
//     sprintf(buffer, "%d ", intWl);                   
//     Serial.println(buffer);
//   }
// }

// void checkIntegralLimitsWindup(float eIntegral){
//   if (eIntegral > eIntegralLimit)
//   {
//     eIntegral = eIntegralLimit;
//   }
//   else if (eIntegral < -eIntegralLimit)
//   {
//     eIntegral = -eIntegralLimit;
//   }
// }

// void RightServoControllerPI(){
//   if (wtRightWheel >= 4.2)
//   {
//     Serial.println("ENTRAMOS PI");
//     // Angular velocity to PWM transformation via polinomial regression.
//     float kpR = 0.05;
//     float kiR = 1;
//     float eR = wtRightWheel - Wr;
//     eintegralR = eintegralR + eR * deltaInterruptionTimeR;

//     // El error que pareceríamos tener es que no conseguimos escribir la velocidad deseada en el servo, esto hace que la delta de interrupción sea enorme y como las ruedas no se mueven 
//     // la velocidad angular de la rueda será cero y el error máximo. Esto hace que la eintegral nunca decrezca. 

//     checkIntegralLimitsWindup(eintegralR);

//     float uR = kpR * eR + kiR * eintegralR;
//     Serial.println(uR);

//     if (uR > 0)
//     {
//       pwrR = (int)(11.5142749480926 * pow(uR, 2) - 29.7178828449269 * uR + 2.95629260770791); // Set the motor speed
//     }
//     else
//     {
//       Serial.println("en else valor 0");
//       pwrR = 0;
//     }

//     if (pwrR > 255)
//       pwrR = 255;

//     Serial.println("PREVIO A PWM");
//     Serial.println(pwrR);
//   }
//   else
//   {
//     pwrR = 0;
//   }
// }

// void LeftServoControllerPI(){
//   if (wtLeftWheel >= 4.2)
//   {
//     // Angular velocity to PWM transformation via polinomial regression.
//     float kpL = 0.05;
//     float kiL = 1;
//     float eL = wtLeftWheel - Wl;
//     eintegralL = eintegralL + eL * deltaInterruptionTimeL;

//     checkIntegralLimitsWindup(eintegralL);

//     float uL = kpL * eL + kiL * eintegralL;

//     if (uL > 0)
//     {
//       pwrL = (int)(11.5142749480926 * pow(uL, 2) - 29.7178828449269 * uL + 2.95629260770791); // Inspect possibility of modeling servo behavior as it could be different than the other.
//     }
//     else
//     {
//       pwrL = 0;
//     }

//     if (pwrL > 255)
//       pwrL = 255;
//   }
//   else
//   {
//     pwrL = 0;
//   }
// }

// void loop()
// {
//   SerialReading();

//   SerialReadingTimeout();
  
//   currentTime = millis();

//   float realDeltaR = (currentTime - pastInterruptionTimeR);
//   float realDeltaL = (currentTime - pastInterruptionTimeL);

//   if (realDeltaR >= 8 * tickCounter) // At 0 velocity our frequency should be 0.
//   {
//     rFrequency = 0; // The longest elapsed time between reads is no more than 20ms, we put 24ms.
//   }

//   if (realDeltaL >= 8 * tickCounter) 
//   {
//     lFrequency = 0; 
//   }

//   Wr = (tickCounter * ((2 * 3.141516) / N) * rFrequency) / gear; // Angular speed Rad/s.
//   Wl = (tickCounter * ((2 * 3.141516) / N) * lFrequency) / gear; 

//   float rounded_downWr = floorf(Wr * 100);                       // Rounding to two decimals our speeds
//   int intWr = (int)rounded_downWr;                               // Getting the integers from our speeds, should divide by 100 in ros code.

//   float rounded_downWl = floorf(Wl * 100);                       
//   int intWl = (int)rounded_downWl;                  

//   checkIfShouldWriteSerial(intWr, intWl);
  
//   //delay(150);    // Necessary for ros program to be able to write. Otherwise we'll lock the buffer while reading.

//   wtRightWheel = 5.0;
//   wtLeftWheel = 5.0;
  
//   // Objective velocity in rad/s with 0-6.17 range where ~4.2 is the value in which the servo is functional.
//   // If velocity is set between [0;4.2] hardcode 0 pwm as the polinomic regression applied to predict pwm values from angular velocities cannot deal with servo behavior for low PWMs (no speed until 100 PWM).
//   RightServoControllerPI();
//   LeftServoControllerPI();
    
//   ////////////// Print angular velocities for plotting. //////////////

//   //    for(int i = 0; i <= 255; i += 5){
//   //      analogWrite(ruedaR,0);
//   //      Wr = (contadorTicks*((2*3.141516)/N)*frecuenciaR)/gear;
//   //      Serial.println(Wr);   // Cambiar el Wr por el Wl cuando sea necesario.
//   //      Serial.print(",");
//   //      Serial.println(i);
//   //      delay(1000);
//   //    }

// //////////////////////////////////////////////////////////////////////

//   //analogWrite(rWheel, pwrR-L);
//   analogWrite(rWheel, 0); // PWM applied to right servo.
//   analogWrite(lWheel, 0); // PWM applied to left servo.
// }

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

void SerialReading()
{
  if(Serial.available() <= 0)
  {
    noDataSerialReadCounter++;
  }
  else{
    noDataSerialReadCounter = 0;

      ros2DataRightWheel = Serial.readStringUntil('\n');
//      Serial.println("AQUI: "+ros2DataRightWheel);
      ros2DataRightWheelRead = true;

    if (Serial.available() > 0){
      ros2DataLeftWheel = Serial.readStringUntil('\n');
      ros2DataLeftWheelRead = true;
    }
    Serial.print("1.- RIGHT DAta: ");
    Serial.println(ros2DataRightWheel);
    Serial.print("2.- LEFT DAta: ");
    Serial.println(ros2DataLeftWheel);
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
      Serial.print("3.- RIGHT DAta in float: ");
      Serial.println(wtRightWheel);
    }

    if(ros2DataLeftWheelRead){
      ros2DataLeftWheelChar = ros2DataLeftWheel.c_str();   
      ros2DataLeftWheelFloat = atof(ros2DataLeftWheelChar);      
      wtLeftWheel = ros2DataLeftWheelFloat/100.0;    
      ros2DataLeftWheelRead = false;
      Serial.print("4.- LEFT DAta in float: ");
      Serial.println(wtLeftWheel);
    }
  }
  else{
    wtRightWheel = 0.0;
    wtLeftWheel = 0.0;
    Serial.println("TIMEOUT.- Hemos llegado al timeout");
  }
}

void checkIfShouldWriteSerial(int intWr, int intWl){
  if(firstSerialWrite || ((currentTime - lastSerialWrite) >= 150)){
    firstSerialWrite = false;

    lastSerialWrite = currentTime;
    
    sprintf(buffer, "%d ", intWr);                   // Transforming the integer speed to char buffer in order to print it via serial to ros2.
    Serial.print(buffer);
  
    sprintf(buffer, "%d ", intWl);                   
    Serial.println(buffer);
  }
}

void checkIntegralLimitsWindup(float eIntegral){
  if (eIntegral > eIntegralLimit)
  {
    eIntegral = eIntegralLimit;
  }
  else if (eIntegral < -eIntegralLimit)
  {
    eIntegral = -eIntegralLimit;
  }
}

void RightServoControllerPI(){
  if (wtRightWheel >= 4.2 && Wr != 0)
  {
//    Serial.println("ENTRAMOS PI");
    // Angular velocity to PWM transformation via polinomial regression.
    float kpR = 0.5;
    float kiR = 0.1;
    float eR = wtRightWheel - Wr;
    Serial.print("5.- Error es: ");
    Serial.println(eR);
    eintegralR = eintegralR + eR * deltaInterruptionTimeR;
    Serial.print("6.- Eintegral es: ");
    Serial.println(eintegralR);
    Serial.print("INTERLUDIO.- deltaInterruptionTimeR: ");
    Serial.println(deltaInterruptionTimeR);

    // El error que pareceríamos tener es que no conseguimos escribir la velocidad deseada en el servo, esto hace que la delta de interrupción sea enorme y como las ruedas no se mueven 
    // la velocidad angular de la rueda será cero y el error máximo. Esto hace que la eintegral nunca decrezca. 

    checkIntegralLimitsWindup(eintegralR);

    float uR = kpR * eR + kiR * eintegralR;
    Serial.print("7.- Velocidad angular corregida (margen de 4.2 a 6.0 indispensable): ");
    Serial.println(uR);

    if (uR > 0)
    {
      pwrR = (int)(11.5142749480926 * pow(uR, 2) - 29.7178828449269 * uR + 2.95629260770791); // Set the motor speed
      float aux = 11.5142749480926 * pow(uR, 2) - 29.7178828449269 * uR + 2.95629260770791;
      Serial.print("8.- Valor en float de pwrR: ");
      Serial.println(aux);
    }
    else
    {
      Serial.println("en else valor 0");
      pwrR = 0;
    }

    if (pwrR > 255)
      pwrR = 255;

    Serial.println("9.- Valor en int de pwrR despues de tratamientos de overflow de PWM: ");
    Serial.println(pwrR);
  }
  else if(wtRightWheel >= 4.2 && Wr == 0){
    // Como la velocidad a la que iba anteriormente es nula (no ha recibido comanda o comanda igual a halt)
    // no tiene sentido usar controller PI que se basa en un modelo en constante movimiento --> deltaInterruptionTime sería 0 siempre)
    pwrR = (int)(11.5142749480926 * pow(wtRightWheel, 2) - 29.7178828449269 * wtRightWheel + 2.95629260770791);
    if (pwrR > 255)
      pwrR = 255;
  }
  else if(wtRightWheel < 4.2)
  {
    Serial.println("NODATAREAD_PWR, velocidad objetivo rueda derecha: ");
    Serial.println(wtRightWheel);
    pwrR = 0;
  }
}

void LeftServoControllerPI(){
  if (wtLeftWheel >= 4.2)
  {
    // Angular velocity to PWM transformation via polinomial regression.
    float kpL = 0.5;
    float kiL = 0.1;
    float eL = wtLeftWheel - Wl;
    eintegralL = eintegralL + eL * deltaInterruptionTimeL;

    checkIntegralLimitsWindup(eintegralL);

    float uL = kpL * eL + kiL * eintegralL;

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
    pwrL = 0;
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

  if (realDeltaR >= 8 * tickCounter) // At 0 velocity our frequency should be 0.
  {
    rFrequency = 0; // The longest elapsed time between reads is no more than 20ms, we put 24ms.
  }

  if (realDeltaL >= 8 * tickCounter) 
  {
    lFrequency = 0; 
  }

  Wr = (tickCounter * ((2 * 3.141516) / N) * rFrequency) / gear; // Angular speed Rad/s.
  Wl = (tickCounter * ((2 * 3.141516) / N) * lFrequency) / gear; 

  float rounded_downWr = floorf(Wr * 100);                       // Rounding to two decimals our speeds
  int intWr = (int)rounded_downWr;                               // Getting the integers from our speeds, should divide by 100 in ros code.

  float rounded_downWl = floorf(Wl * 100);                       
  int intWl = (int)rounded_downWl;    
  
                
  // SEND RPM TO SERIAL
  checkIfShouldWriteSerial(intWr, intWl);
  
  //delay(150);    // Necessary for ros program to be able to write. Otherwise we'll lock the buffer while reading.

  
//  if (count < 500){
//    wtRightWheel = 5.0;
//    wtLeftWheel = 5.0;
//  }
//  else if (500 <= count && count < 1000){
//    wtRightWheel = 4.5;
//    wtLeftWheel = 4.5;
//  }
//  else if (count >= 1000) {
//    count = 0;
//  }
//  count ++;

  // Objective velocity in rad/s with 0-6.17 range where ~4.2 is the value in which the servo is functional.
  // If velocity is set between [0;4.2] hardcode 0 pwm as the polinomic regression applied to predict pwm values from angular velocities cannot deal with servo behavior for low PWMs (no speed until 100 PWM).
  Serial.print("Valor de wtRightWheel");
  Serial.println(wtRightWheel);
//  wtRightWheel=5.0;
  RightServoControllerPI();
//  LeftServoControllerPI();
//    
  ////////////// Print angular velocities for plotting. //////////////

  //    for(int i = 0; i <= 255; i += 5){
  //      analogWrite(ruedaR,0);
  //      Wr = (contadorTicks*((2*3.141516)/N)*frecuenciaR)/gear;
  //      Serial.println(Wr);   // Cambiar el Wr por el Wl cuando sea necesario.
  //      Serial.print(",");
  //      Serial.println(i);
  //      delay(1000);
  //    }

//////////////////////////////////////////////////////////////////////

  //analogWrite(rWheel, pwrR-L);
  analogWrite(rWheel, pwrR); // PWM applied to right servo.
  analogWrite(lWheel, 0); // PWM applied to left servo.
//  Serial.println(wtRightWheel);   // Cambiar el Wr por el Wl cuando sea necesario.
//  Serial.print(",");
//  Serial.println(Wr);

  delay(10);

  
}