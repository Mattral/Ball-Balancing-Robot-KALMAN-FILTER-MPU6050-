
#include <Wire.h>   // IIC communication library

///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Motor//////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

bool LeftDirection = HIGH;
bool RightDirection =HIGH;
// Motor speed control variable (0-255)
int motorSpeed = 128;

// ************ DEFINITIONS************
float kp = 0.02;
float ki = 0.00015 ;
float kd = 0;

const byte interruptPinA = 2;
const byte interruptPinB = 3;
const byte PWMPin = 6;
const byte DirPinA1 = 7;
const byte DirPinA2 = 8;

volatile unsigned long count = 0;
unsigned long count_prev = 0;
volatile long EncoderCount = 0;
unsigned long t_prev = 0;
float Theta_prev = 0;
float e_prev = 0, inte_prev = 0;
float RPM = 0;



const byte interruptPinA2 = 18; // Interrupt pin for the additional motor's encoder
const byte interruptPinB2 = 19; // Interrupt pin for the additional motor's encoder
const byte PWMPin2 = 13; // Additional motor PWM pin
const byte DirPinB1 = 10; // Additional motor direction pin
const byte DirPinB2 = 9; // Additional motor direction pin

volatile unsigned long count2 = 0;
unsigned long count_prev2 = 0;
volatile long EncoderCount2 = 0;
unsigned long t_prev2 = 0;
float Theta_prev2 = 0;
float e_prev2 = 0, inte_prev2 = 0;
float RPM2 = 0;
//??????????????????????????????????????????????????????????????????????????????????????????????
//???????????????????????????????????????????????????????????????????????????????????????????????
//////////////////////////////////// Change Pin Numbers ??????????????????????????????????
////////////////////////////////////////////????????????????????????????????????????????????????

const byte interruptPinA3 = 20; // Interrupt pin for the additional motor's encoder
const byte interruptPinB3 = 21; // Interrupt pin for the additional motor's encoder
const byte PWMPin3 = 11; // Additional motor PWM pin
const byte DirPinA3 = 22; // Additional motor direction pin
const byte DirPinB3 = 23; // Additional motor direction pin

volatile long unsigned int count3 = 0;
unsigned long count_prev3 = 0;
volatile long EncoderCount3 = 0;
unsigned long t_prev3 = 0;
float Theta_prev3 = 0;
float e_prev3 = 0, inte_prev3 = 0;
float RPM3 = 0;



#define pi 3.1416
float Vmax = 6;
float Vmin = -6;


// motor
void ISR1_EncoderA() {
  bool PinB = digitalRead(interruptPinB);
  bool PinA = digitalRead(interruptPinA);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCount++;
    }
    else {
      EncoderCount--;
    }
  }

  else {
    if (PinA == HIGH) {
      EncoderCount--;
    }
    else {
      EncoderCount++;
    }
  }
}

void ISR1_EncoderB() {
  bool PinB = digitalRead(interruptPinA);
  bool PinA = digitalRead(interruptPinB);

  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCount--;
    }
    else {
      EncoderCount++;
    }
  }

  else {
    if (PinB == HIGH) {
      EncoderCount++;
    }
    else {
      EncoderCount--;
    }
  }
}

void ISR2_EncoderA() {
  bool PinB = digitalRead(interruptPinB2);
  bool PinA = digitalRead(interruptPinA2);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCount2++;
    }
    else {
      EncoderCount2--;
    }
  }
  else {
    if (PinA == HIGH) {
      EncoderCount2--;
    }
    else {
      EncoderCount2++;
    }
  }
}
void ISR2_EncoderB() {
  bool PinB = digitalRead(interruptPinA2);
  bool PinA = digitalRead(interruptPinB2);

  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCount2--;
    }
    else {
      EncoderCount2++;
    }
  }

  else {
    if (PinB == HIGH) {
      EncoderCount2++;
    }
    else {
      EncoderCount2--;
    }
  }
}

void ISR3_EncoderA() {
  bool PinB = digitalRead(interruptPinB3);
  bool PinA = digitalRead(interruptPinA3);

  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCount3++;
    } else {
      EncoderCount3--;
    }
  } else {
    if (PinA == HIGH) {
      EncoderCount3--;
    } else {
      EncoderCount3++;
    }
  }
}

void ISR3_EncoderB() {
  bool PinB = digitalRead(interruptPinA3);
  bool PinA = digitalRead(interruptPinB3);

  if (PinA == LOW) {
    if (PinB == HIGH) {
      EncoderCount3--;
    } else {
      EncoderCount3++;
    }
  } else {
    if (PinB == HIGH) {
      EncoderCount3++;
    } else {
      EncoderCount3--;
    }
  }
}



float sign(float x) {
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  } else {
    return 0;
  }
}


void WriteDriverVoltage(float V, float Vmax, byte DirPin1, byte DirPin2, byte PWMPin) {
  int PWMval2 = int(255 * abs(V) / Vmax);
  if (PWMval2 > 255) {
    PWMval2 = 255;
  }
  if (V > 0) {
    digitalWrite(DirPin1, HIGH);
    digitalWrite(DirPin2, LOW);
    

  }
  else if (V < 0) {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, HIGH);
   
  }
  else {
    digitalWrite(DirPin1, LOW);
    digitalWrite(DirPin2, LOW);
    
  }
  analogWrite(PWMPin, PWMval2);

}

void convertV(int rpm_m, byte DirPinA, byte DirPinB, byte PWMPin, const volatile unsigned long count,
              unsigned long count_prev, volatile long &EncoderCount, unsigned long &t_prev,
              float &Theta_prev, float &e_prev, float &inte_prev, float Vmax, float Vmin, float &rpm
             ) {

  if (count > count_prev) {
    unsigned long t = millis();
    float Theta = EncoderCount / 900.0;
    int dt = (t - t_prev);
    float RPM_d = rpm_m;// * (sin(2 * pi * 0.005 * t / 1000.0))
    //  * sign(sin(2 * pi * 0.05 * t / 1000.0));
    if (t / 1000.0 > 100) {
      RPM_d = 0;
    }
    rpm = (Theta - Theta_prev) / (dt / 1000.0) * 60;
    float e = RPM_d - rpm;
    float inte = inte_prev + (dt * (e + e_prev) / 2);
    float V = kp * e + ki * inte + (kd * (e - e_prev) / dt) ;
    if (V > Vmax) {
      V = Vmax;
      inte = inte_prev;
    }
    if (V < Vmin) {
      V = Vmin;
      inte = inte_prev;
    }


    WriteDriverVoltage(V, Vmax, DirPinA, DirPinB, PWMPin);

    // Serial.print(RPM_d); Serial.print(" \t");
    // Serial.print(RPM); Serial.print(" \t ");
    // Serial.print(V); Serial.print("\t  ");
    // Serial.print(e); Serial.println("  ");

    Theta_prev = Theta;
    count_prev = count;
    t_prev = t;
    inte_prev = inte;
    e_prev = e;
  }
}
ISR(TIMER1_COMPA_vect) {
  count++;
  count2++;
  count3++;
  // Serial.print(count * 0.05); Serial.print(" \t");
}

// END MOTOR

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////// KALMAN  /////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

//END OF KALMAN


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// SETUP  ?/////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void setup()
{
  
  // Join the I2C bus
  Wire.begin();         // Join the I2C bus sequence
  Serial.begin(115200); // Open serial monitor and set the baud rate to 115200

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////// KALMAN SETUP  /////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////MOTOR SETUP  ????////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  // motor start
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR1_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR1_EncoderB, CHANGE);

  pinMode(interruptPinA2, INPUT_PULLUP);
  pinMode(interruptPinB2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA2), ISR2_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB2), ISR2_EncoderB, CHANGE);

  pinMode(interruptPinA3, INPUT_PULLUP);
  pinMode(interruptPinB3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA3), ISR3_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB3), ISR3_EncoderB, CHANGE);


  pinMode(DirPinA1, OUTPUT);
  pinMode(DirPinA2, OUTPUT);
  pinMode(PWMPin, OUTPUT);

  pinMode(DirPinB1, OUTPUT); // Additional motor direction pin
  pinMode(DirPinB2, OUTPUT); // Additional motor direction pin
  pinMode(PWMPin2, OUTPUT); // Additional motor PWM pin

  pinMode(DirPinA3, OUTPUT); // Additional motor direction pin
  pinMode(DirPinB3, OUTPUT); // Additional motor direction pin
  pinMode(PWMPin3, OUTPUT); // Additional motor PWM pin

  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 12499; //Prescaler = 64
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11 | 1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  sei();
  // motor end
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////// LOOP  //////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop()
{


  ///////////////////////////////////////////////////////////////// KALMAN LOOP //////////////////////////////////////////////////

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

  // Wait for a fixed interval before updating again
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////// MOTOR LOOP //////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  // Motor motion here
  int motor_speed = 255; // Set the speed for drive motors

  // Adjust V values for each motor
  float V1 = motor_speed;   // Positive value for forward motion
  float V2 = -motor_speed;  // Negative value for backward motion
  float V3 = 100;           // Adjust this value for steering, e.g., positive for right, negative for left

  convertV(V1, DirPinA1, DirPinA2, PWMPin, count, count_prev, EncoderCount,
           t_prev, Theta_prev, e_prev, inte_prev, Vmax, Vmin, RPM);

  convertV(V2, DirPinB1, DirPinB2, PWMPin2, count2, count_prev2, EncoderCount2,
           t_prev2, Theta_prev2, e_prev2, inte_prev2, Vmax, Vmin, RPM2);

  convertV(V3, DirPinA3, DirPinB3, PWMPin3, count3, count_prev3, EncoderCount3,
           t_prev3, Theta_prev3, e_prev3, inte_prev3, Vmax, Vmin, RPM3);


//////////////////////////////////////////////////////////// DIRECTIONAL CONTROL LOOP _?////////////////////////////////////////////////////

  // Control the motor direction based on KalmanAngleRoll
  if (KalmanAngleRoll > 50) {
    // If KalmanAngleRoll is greater than 50, go forward
    LeftDirection = HIGH;
  } else if (KalmanAngleRoll < 50) {
    // If KalmanAngleRoll is less than 50, go reverse
    LeftDirection = LOW;
  }

  // Control the motor direction based on KalmanAnglePitch
  if (KalmanAnglePitch > 50) {
    // If KalmanAngleRoll is greater than 50, go forward
    RightDirection = HIGH;
  } else if (KalmanAnglePitch < 50) {
    // If KalmanAngleRoll is less than 50, go reverse
    RightDirection = LOW;
  }

  if (RightDirection==HIGH) {
    V3= abs(1*V3);
  }
  else if (RightDirection==LOW) {
    V3= -1*V3;
  }

  if (LeftDirection==HIGH){
    V1= abs(1*V1);
    V2= 1*V2;
  }
  else if (LeftDirection==LOW){
    V1= -1*V1;
    V2= abs(-1*V2);
  }

  // Set motor speeds
  analogWrite(PWMPin, motorSpeed);
  analogWrite(PWMPin2, motorSpeed);
  analogWrite(PWMPin3, motorSpeed);
  

//  // Change motor directions if necessary based on LeftDirection variable for both motors
//  if (LeftDirection != digitalRead(DirPinA1)) {
//    if (LeftDirection == HIGH) {
//      digitalWrite(DirPinA1, LOW);
//      digitalWrite(DirPinA2, HIGH);
//    } else {
//      digitalWrite(DirPinA1, HIGH);
//      digitalWrite(DirPinA2, LOW);
//    }
//  }
//
//  if (LeftDirection != digitalRead(DirPinB1)) {
//    if (LeftDirection == HIGH) {
//      digitalWrite(DirPinB1, LOW);
//      digitalWrite(DirPinB2, HIGH);
//    } else {
//      digitalWrite(DirPinB1, HIGH);
//      digitalWrite(DirPinB2, LOW);
//    }
//  }


}

///////////////////////////////////////// END OF LOOP HERE_________________________________________________________________





