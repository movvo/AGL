// ===================
// Define names to pins
// ===================
int m1_enA = 2;       // MOTOR RIGHT ENA
int m1_in1 = 6;       // MOTOR RIGHT IN1
int m1_in2 = 7;       // MOTOR RIGHT IN2

int m2_enB = 3;       // MOTOR LEFT ENB
int m2_in3 = 4;       // MOTOR LEFT IN3
int m2_in4 = 5;       // MOTOR LEFT IN4

int m1_enc_1 = A0;    // MOTOR LEFT ENCODER
int m1_enc_2 = A1;    // MOTOR LEFT ENCODER
int m2_enc_1 = A2;    // MOTOR LEFT ENCODER
int m2_enc_2 = A3;    // MOTOR LEFT ENCODER

//int rotDirection = 0;
//int count = 0;

// ===================
void setup()
// ===================
{
  // Pins
  pinMode(m1_enA, OUTPUT);
  pinMode(m1_in1, OUTPUT);
  pinMode(m1_in2, OUTPUT);
  pinMode(m2_enB, OUTPUT);
  pinMode(m2_in3, OUTPUT);
  pinMode(m2_in4, OUTPUT);

  // Led
  pinMode(LED_BUILTIN, OUTPUT);

  //Init Serial Readings
  Serial.begin(9600);
  Serial.setTimeout(50);
}

// ===================
void loop()
// ===================
{
  // TEST FULL SPEED
  digitalWrite(m1_in1, HIGH);
  digitalWrite(m1_in2, LOW);
  digitalWrite(m2_in3, HIGH);
  digitalWrite(m2_in4, LOW);
  analogWrite(m1_enA, 255);
  analogWrite(m2_enB, 255);
  digitalWrite(LED_BUILTIN, HIGH); // Led ON
  delay(2000);
  digitalWrite(m1_in1, LOW);
  digitalWrite(m1_in2, HIGH);
  digitalWrite(m2_in3, LOW);
  digitalWrite(m2_in4, HIGH);
  analogWrite(m1_enA, 255);
  analogWrite(m2_enB, 255);
  digitalWrite(LED_BUILTIN, LOW); // Led ON
  
  if (Serial.available())
  {
    int potValue = Serial.parseInt();
    //TODO: Definir el 0 255 a escala para que funcione el motor a minima o maxima potencia
    int pwmOutput = map(potValue, 0, 100, 0 , 255); // Map the potentiometer value from 0 to 255
    Serial.print(potValue);
    analogWrite(m1_enA, potValue); // Send PWM signal to L298N Enable pin
    analogWrite(m2_enB, potValue); // Send PWM signal to L298N Enable pin
    digitalWrite(LED_BUILTIN, HIGH); // Led ON
  }
  // else {
  //   int potValue = 0; // Read serial potenciometer value
  //   analogWrite(m1_enA, 0); // Send PWM signal to L298N Enable pin
  //   analogWrite(m2_enB, 0); // Send PWM signal to L298N Enable pin
  //  }
    
  delay(2000);
}
