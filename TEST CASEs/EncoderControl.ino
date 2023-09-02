#include <Arduino.h>

// Define the pins
const int interruptPinA = 2;
const int interruptPinB = 3;
const int PWMPin = 6;
const int DirPinA1 = 7;
const int DirPinA2 = 8;

// Encoder variables
volatile long encoderCount = 0;
int lastEncoded = 0;
long lastEncoderCount = 0;

// Motor speed control variable (0-255)
int motorSpeed = 128;

// Motor direction control variable
int LeftDirection = HIGH;  // You can change this based on sensor data

// Function to handle the encoder interrupts
void handleEncoder() {
  int MSB = digitalRead(interruptPinA);
  int LSB = digitalRead(interruptPinB);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderCount++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderCount--;
  }

  lastEncoded = encoded;
}

void setup() {
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  pinMode(PWMPin, OUTPUT);
  pinMode(DirPinA1, OUTPUT);
  pinMode(DirPinA2, OUTPUT);

  // Attach an interrupt to the A channel of the encoder
  attachInterrupt(digitalPinToInterrupt(interruptPinA), handleEncoder, CHANGE);
  
  // Set the initial motor direction based on LeftDirection variable
  if (LeftDirection == HIGH) {
    digitalWrite(DirPinA1, LOW);
    digitalWrite(DirPinA2, HIGH);
  } else {
    digitalWrite(DirPinA1, HIGH);
    digitalWrite(DirPinA2, LOW);
  }

  // Set initial motor speed
  analogWrite(PWMPin, motorSpeed);
}

void loop() {
  // Set motor speed
  analogWrite(PWMPin, motorSpeed);

  // Change motor direction if necessary based on LeftDirection variable
  if (LeftDirection != digitalRead(DirPinA1)) {
    if (LeftDirection == HIGH) {
      digitalWrite(DirPinA1, LOW);
      digitalWrite(DirPinA2, HIGH);
    } else {
      digitalWrite(DirPinA1, HIGH);
      digitalWrite(DirPinA2, LOW);
    }
  }

}
