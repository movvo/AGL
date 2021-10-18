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
  if (Serial.available())
  {
    int potValue = Serial.parseInt();
    /* Map an analog value to 8 bits (0 to 255) */
    int pwmOutput = map(potValue, 0, 1023, 0 , 255);
    Serial.print(potValue);
    analogWrite(m1_enA, pwmOutput);   // Send PWM signal to L298N Enable pin
    analogWrite(m2_enB, pwmOutput);    // Send PWM signal to L298N Enable pin
    if (potValue > 0){
      // Forward
      digitalWrite(m1_in1, LOW);
      digitalWrite(m1_in2, HIGH);
      digitalWrite(m2_in3, HIGH);
      digitalWrite(m2_in4, LOW);
      digitalWrite(LED_BUILTIN, HIGH);  // Led ON
    }else{
      // Backwards
      digitalWrite(m1_in1, HIGH);
      digitalWrite(m1_in2, LOW);
      digitalWrite(m2_in3, LOW);
      digitalWrite(m2_in4, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);  // Led ON
    } 
  }
  else {
    int potValue = 0; // Read serial potenciometer value
    analogWrite(m1_enA, 0); // Send PWM signal to L298N Enable pin
    analogWrite(m2_enB, 0); // Send PWM signal to L298N Enable pin
  }
    
  delay(10);
}
