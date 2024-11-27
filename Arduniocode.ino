#include <Servo.h>

// Pin definitions for LEDs
#define LED1 D1
#define LED2 D3
#define LED3 D4
#define PIR_LED D10

// Pin definition for DC motor
#define MOTOR_CONTROL_PIN D2

// Pin definition for Ultrasonic Sensor
#define TRIG_PIN D6
#define ECHO_PIN D7

// Pin definition for Servo Motor
#define SERVO_PIN D8

// Pin definition for PIR Sensor
#define PIR_PIN D0

// Servo object
Servo myServo;

// Variables for ultrasonic sensor
volatile long duration = 0;
volatile int distance = 0;
volatile bool echoReceived = false;

// Variables for PIR
int lastPirState = LOW;

// Timer for ultrasonic trigger
unsigned long lastTriggerTime = 0;
const unsigned long triggerInterval = 100; // 100 ms interval

void setup() {
  Serial.begin(9600);

  // Set LED pins as output
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(PIR_LED, OUTPUT);

  // Set motor control pin as output
  pinMode(MOTOR_CONTROL_PIN, OUTPUT);

  // Set ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Set PIR sensor pin as input
  pinMode(PIR_PIN, INPUT);

  // Attach the servo object
  myServo.attach(SERVO_PIN);
  myServo.write(0); // Initialize servo

  // Initialize motor and LEDs
  digitalWrite(MOTOR_CONTROL_PIN, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(PIR_LED, LOW);
}

void loop() {
  unsigned long currentMillis = millis();

  // Handle ultrasonic sensor and servo motor logic
  if (currentMillis - lastTriggerTime >= triggerInterval) {
    lastTriggerTime = currentMillis;
    triggerUltrasonic();
  }
  if (echoReceived) {
    echoReceived = false; // Reset flag
    handleServo();
  }

  // Handle PIR sensor for motion detection
  handlePIRSensor();

  // Check serial commands for LEDs and motor
  checkSerialCommands();
}

// Non-blocking ultrasonic trigger and echo measurement
void triggerUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure echo pulse duration (non-blocking)
  duration = pulseIn(ECHO_PIN, HIGH, 20000); // Timeout after 20ms
  if (duration > 0) {
    distance = duration * 0.034 / 2;
    echoReceived = true;
  }
}

void handleServo() {
  if (distance < 20) {
    myServo.write(180); // Rotate servo
  } else {
    myServo.write(0); // Reset servo
  }

  // Debugging
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

void handlePIRSensor() {
  int pirState = digitalRead(PIR_PIN);
  if (pirState != lastPirState) { // React only to changes
    if (pirState == HIGH) {
      digitalWrite(PIR_LED, HIGH);
      Serial.println("Motion detected!");
    } else {
      digitalWrite(PIR_LED, LOW);
      Serial.println("No motion detected.");
    }
    lastPirState = pirState;
  }
}

void checkSerialCommands() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    switch (command) {
      case '0':
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, LOW);
        break;
      case '1':
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, LOW);
        break;
      case '2':
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, HIGH);
        digitalWrite(LED3, LOW);
        break;
      case '3':
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, HIGH);
        digitalWrite(LED3, HIGH);
        break;
      case '4':
        analogWrite(MOTOR_CONTROL_PIN, 255); // Full speed
        break;
      case '5':
        digitalWrite(MOTOR_CONTROL_PIN, LOW);
        break;
    }
  }
}
