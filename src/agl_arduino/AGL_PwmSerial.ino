
/*  Arduino DC Motor Control - PWM | H-Bridge | L298N  -  Example 01
    by Dejan Nedelkovski, www.HowToMechatronics.com
*/
// Define names to pins
#define enA 3
#define in1 52
#define in2 53

int rotDirection = 0;
int count = 0;
void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  //pinMode(button, INPUT);
  // Set initial rotation direction
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  pinMode(LED_BUILTIN, OUTPUT);

  //Init Serila Readings
  Serial.begin(9600);
  Serial.setTimeout(50);
}
void loop() {
  if (Serial.available())
  {
    int potValue = Serial.read(); // Read serial potenciometer value
    //TODO: Definir el 0 255 a escala para que funcione el motor a minima o maxima potencia
    int pwmOutput = map(potValue, 0, 100, 0 , 255); // Map the potentiometer value from 0 to 255

    analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
  }
  else {
    int potValue = 0; // Read serial potenciometer value
    analogWrite(enA, 0); // Send PWM signal to L298N Enable pin

  }

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)

  //delay(100);
  //digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(10);
}
