// Interruption times and speed calculation variables for right servo.
volatile double currentInterruptionTimeR = 0; 
volatile double pastInterruptionTimeR = 0;
volatile double deltaInterruptionTimeR = 0;
volatile double currentTime = 0;

int IN3_MR = 5;
int IN4_MR = 4;

int encoderR = 19; // Right encoder pin.
int rWheel = 11;   // PWM pin for right servo, right wheel.

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

void setup()
{

  // Keep an eye on the wiring as it can produce unexpected behavior. Poorly adjusted wires and so on.
  pinMode(encoderR, INPUT);
  pinMode(rWheel, OUTPUT);

  digitalWrite(encoderR, HIGH);
  digitalWrite(IN3_MR, HIGH);
  digitalWrite(IN4_MR, LOW);

  attachInterrupt(digitalPinToInterrupt(encoderR), REncoder, FALLING); // PIN Interruption set to detect a falling flank in encoderR pin, it'll activate REncoder function.
  Serial.begin(9600);                                                  // Start of Serial communication.
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
  }

  if (CR == tickCounter && firstEncoderRead == true)
  {
    firstEncoderRead = false;
  }

  if (CR == tickCounter)
  {
    CR = 0; // Counter for tickCounter reset.
  }
}

void loop()
{

  currentTime = millis();

  double realDelta = (currentTime - pastInterruptionTimeR);

  if (realDelta >= 8 * tickCounter) // At 0 velocity our frequency should be 0.
  {
    rFrequency = 0; // The longest elapsed time between reads is no more than 20ms, we put 24ms.
  }

  Wr = (tickCounter * ((2 * 3.141516) / N) * rFrequency) / gear; // Angular speed Rad/s.
  Vr = Wr * (diameter / 2);                                      // Linear speed cm/s.

  Serial.println(Vr);     // Linear speed print.
  analogWrite(rWheel, 0); // PWM applied to right servo.
}
