#include <Arduino.h>
#include <Wire.h>

//////////////////////////////////////////////Motor//////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

// Define the pins for Motor A
const int interruptPinA = 2;
const int interruptPinB = 3;
const int PWMPin = 6;
const int DirPinA1 = 7;
const int DirPinA2 = 8;

// Define the pins for Motor B
const int interruptPinA2 = 18;
const int interruptPinB2 = 19;
const int PWMPin2 = 13;
const int DirPinB1 = 10;
const int DirPinB2 = 9;

// Encoder variables for Motor A
volatile long encoderCount = 0;
int lastEncoded = 0;
long lastEncoderCount = 0;

// Encoder variables for Motor B
volatile long encoderCount2 = 0;
int lastEncoded2 = 0;
long lastEncoderCount2 = 0;

// Motor speed control variable (0-255)
int motorSpeed = 128;

// Motor direction control variable
int LeftDirection = HIGH;  // You can change this based on sensor data

void handleEncoderA() {
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

void handleEncoderB() {
  int MSB = digitalRead(interruptPinA2);
  int LSB = digitalRead(interruptPinB2);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded2 << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderCount2++;
  } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderCount2--;
  }

  lastEncoded2 = encoded;
}


//////////////////////////////////////  Kalman MPU  //////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// Variables for storing raw gyroscope data
float RateRoll, RatePitch, RateYaw;

// Variables for gyroscope calibration
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;
int RateCalibrationNumber = 2000; // Number of calibration samples

// Variables for storing raw accelerometer data
float AccX, AccY, AccZ;

// Variables for storing roll and pitch angles
float AngleRoll, AnglePitch;

// Loop timer for controlling the update rate
uint32_t LoopTimer;

// Kalman filter variables for roll and pitch angles
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2; // Initial state and uncertainty
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;

// Array to store the output of the Kalman filter [Estimated Angle, Estimated Uncertainty]
float Kalman1DOutput[] = {0, 0};

// Kalman filter function for 1D state estimation
void kalman_1d(float &KalmanState, float &KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  // 1. Predict the current state of the system
  KalmanState = KalmanState + 0.004 * KalmanInput;

  // 2. Calculate the uncertainty of the prediction
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;

  // 3. Calculate the Kalman gain from the uncertainties on the prediction and measurements
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);

  // 4. Update the predicted state through the Kalman gain
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);

  // 5. Update the uncertainty of the predicted state
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  // Update the Kalman filter outputs
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

// Function to read gyroscope and accelerometer data from MPU6050 sensor
void read_gyro_accel() {
  // Set low pass filter for the gyroscope
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Set accelerometer output range
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Read accelerometer measurements from the sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Set gyro output range and read gyro measurements
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // Convert gyro measurements to rotation rates
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  // Convert accelerometer measurements to physical values
  AccX = (float)AccXLSB / 4096 - 0.02;
  AccY = (float)AccYLSB / 4096 + 0.02;
  AccZ = (float)AccZLSB / 4096;

  // Calculate the absolute roll and pitch angles using accelerometer data
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / PI);
  AnglePitch = atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / PI);
}


/////////////////////////////////////  SETUP  /////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////


void setup() {

  //////////////////////////////Motor//////////////////////////////////
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  pinMode(PWMPin, OUTPUT);
  pinMode(DirPinA1, OUTPUT);
  pinMode(DirPinA2, OUTPUT);

  pinMode(interruptPinA2, INPUT_PULLUP);
  pinMode(interruptPinB2, INPUT_PULLUP);
  pinMode(PWMPin2, OUTPUT);
  pinMode(DirPinB1, OUTPUT);
  pinMode(DirPinB2, OUTPUT);

  // Attach interrupts for both encoders
  attachInterrupt(digitalPinToInterrupt(interruptPinA), handleEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinA2), handleEncoderB, CHANGE);

  // Set the initial motor direction based on LeftDirection variable for Motor A
  if (LeftDirection == HIGH) {
    digitalWrite(DirPinA1, LOW);
    digitalWrite(DirPinA2, HIGH);
  } else {
    digitalWrite(DirPinA1, HIGH);
    digitalWrite(DirPinA2, LOW);
  }

  // Set the initial motor direction based on LeftDirection variable for Motor B
  if (LeftDirection == HIGH) {
    digitalWrite(DirPinB1, LOW);
    digitalWrite(DirPinB2, HIGH);
  } else {
    digitalWrite(DirPinB1, HIGH);
    digitalWrite(DirPinB2, LOW);
  }

  // Set initial motor speeds
  analogWrite(PWMPin, motorSpeed);
  analogWrite(PWMPin2, motorSpeed);

  ///////////////////////////////////END MOTOR SETUP/////////////////////////////////


  ////////////////////////////////// KALMAN SETUP  ////////////////////////////

  // Set LED pin as output and turn it on for indication
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Set I2C clock speed to 400kHz
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // Wake up MPU6050 sensor
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Calibrate gyroscopes by collecting samples and calculating the average
  for (int i = 0; i < RateCalibrationNumber; i++) {
    read_gyro_accel();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }

  // Calculate the gyroscopes' calibration averages
  RateCalibrationRoll /= RateCalibrationNumber;
  RateCalibrationPitch /= RateCalibrationNumber;
  RateCalibrationYaw /= RateCalibrationNumber;

  // Initialize loop timer
  LoopTimer = micros();

 /////////////////////////////////// END KALMAN SETUP //////////////////////////////////
  
}


///////////////////////////////////  LOOP START  ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////



void loop() {
  // Read gyroscope and accelerometer data
  read_gyro_accel();

  // Remove gyroscopes' calibration offsets
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Run the Kalman filter for roll and pitch angles
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);

  // Print the predicted angle values to the serial monitor
  Serial.print("Roll Angle [°]: ");
  Serial.print(KalmanAngleRoll);
  Serial.print("\tPitch Angle [°]: ");
  Serial.println(KalmanAnglePitch);

  // Control the motor direction based on KalmanAngleRoll
  if (KalmanAngleRoll > 50) {
    // If KalmanAngleRoll is greater than 50, go forward
    LeftDirection = HIGH;
  } else if (KalmanAngleRoll < 50) {
    // If KalmanAngleRoll is less than 50, go reverse
    LeftDirection = LOW;
  }

  // Set motor speeds
  analogWrite(PWMPin, motorSpeed);
  analogWrite(PWMPin2, motorSpeed);

  // Change motor directions if necessary based on LeftDirection variable for both motors
  if (LeftDirection != digitalRead(DirPinA1)) {
    if (LeftDirection == HIGH) {
      digitalWrite(DirPinA1, LOW);
      digitalWrite(DirPinA2, HIGH);
    } else {
      digitalWrite(DirPinA1, HIGH);
      digitalWrite(DirPinA2, LOW);
    }
  }

  if (LeftDirection != digitalRead(DirPinB1)) {
    if (LeftDirection == HIGH) {
      digitalWrite(DirPinB1, LOW);
      digitalWrite(DirPinB2, HIGH);
    } else {
      digitalWrite(DirPinB1, HIGH);
      digitalWrite(DirPinB2, LOW);
    }
  }

  // Wait for a fixed interval before updating again
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
