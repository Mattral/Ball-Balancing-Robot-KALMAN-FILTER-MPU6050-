#include <MPU6050.h> // MPU6050 library
#include <Wire.h>   // IIC communication library

// MOTOR CODE

//GeeKee CeeBee
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


#define pi 3.1416
float Vmax = 6;
float Vmin = -6;

// END MOTOR 

MPU6050 mpu6050; // Instantiate an MPU6050 object; name mpu6050
int16_t ax, ay, az, gx, gy, gz; // Define three-axis acceleration, three-axis gyroscope variables
float AngleX, AngleY;
float Gyro_x, Gyro_y, Gyro_z; // Calculate angular velocity of each axis by gyroscope

///////////////////////Kalman_Filter for(X)////////////////////////////
float Q_angleX = 0.001; // Covariance of gyroscope noise
float Q_gyroX = 0.003;  // Covariance of gyroscope drift noise
float R_angleX = 0.5;   // Covariance of accelerometer
char C_0_X = 1;
float dt = 0.005;       // The value of dt is the filter sampling time.
float K1X = 0.05;       // A function containing the Kalman gain is used to calculate the deviation of the optimal estimate.
float K_0X, K_1X, t_0X, t_1X;
float angle_errX;
float q_biasX;     // Gyroscope drift
float accelz = 0;
float angleX, angleY;
float angle_speedX, angle_speedY;

float Pdot_X[4] = {0, 0, 0, 0};
float P_X[2][2] = {{1, 0}, {0, 1}};
float PCt_0_X, PCt_1_X, E_X;
///////////////////////Kalman_Filter for(X)////////////////////////////
float Q_angleX = 0.001; // Covariance of gyroscope noise
float Q_gyroX = 0.003;  // Covariance of gyroscope drift noise
float R_angleX = 0.5;   // Covariance of accelerometer
char C_0_X = 1;
float dt = 0.005;       // The value of dt is the filter sampling time.
float K1X = 0.05;       // A function containing the Kalman gain is used to calculate the deviation of the optimal estimate.
float K_0X, K_1X, t_0X, t_1X;
float angle_errX;
float q_biasX;     // Gyroscope drift
float accelz = 0;
float angleX, angleY;
float angle_speedX, angle_speedY;

float Pdot_X[4] = {0, 0, 0, 0};
float P_X[2][2] = {{1, 0}, {0, 1}};
float PCt_0_X, PCt_1_X, E_X;
//////////////////////Kalman_Filter/////////////////////////

// Function prototypes
void Kalman_FilterX(double angle_m, double gyro_m);
void Kalman_FilterY(double angle_m, double gyro_m);

void setup()
{
  // Join the I2C bus
  Wire.begin();         // Join the I2C bus sequence
  Serial.begin(115200); // Open serial monitor and set the baud rate to 115200
  delay(1500);
  mpu6050.initialize(); // Initialize MPU6050
  delay(100);

  // motor start
  pinMode(interruptPinA, INPUT_PULLUP);
  pinMode(interruptPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR1_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR1_EncoderB, CHANGE);

  pinMode(interruptPinA2, INPUT_PULLUP);
  pinMode(interruptPinB2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinA2), ISR2_EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinB2), ISR2_EncoderB, CHANGE);

  pinMode(DirPinA1, OUTPUT);
  pinMode(DirPinA2, OUTPUT);
  pinMode(PWMPin, OUTPUT);

  pinMode(DirPinB1, OUTPUT); // Additional motor direction pin
  pinMode(DirPinB2, OUTPUT); // Additional motor direction pin
  pinMode(PWMPin2, OUTPUT); // Additional motor PWM pin
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

void loop()
{
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  angle_calculateX(ax, ay, az, gx, gy, gz, dt, Q_angleX, Q_gyroX, R_angleX, C_0_X, K1X);
  angle_calculateY(ax, ay, az, gx, gy, gz, dt, Q_angleY, Q_gyroY, R_angleY, C_0_Y, K1Y);

  Serial.print("AngleX = ");
  Serial.print(AngleX);
  //Serial.print("  angleX = ");
  //Serial.print(angleX);
  Serial.print("  AngleY = ");
  Serial.println(AngleY);
  //Serial.print("  angleY = ");
  //Serial.println(angleY);

  // motor motion here
  int motor_speed = abs(sqrt(sq(AngleX)+sq(AngleY)));
  motor_speed = map(motor_speed, 0,180, 0,255);

  convertV(motor_speed,DirPinA1, DirPinA2, PWMPin, count, count_prev, EncoderCount,
    t_prev, Theta_prev, e_prev, inte_prev, Vmax, Vmin, RPM
   );
   convertV(motor_speed,DirPinB1, DirPinB2, PWMPin2, count2, count_prev2, EncoderCount2,
    t_prev2, Theta_prev2, e_prev2, inte_prev2, Vmax, Vmin, RPM2
  );
}

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

void convertV(int rpm_m, byte DirPinA, byte DirPinB, byte PWMPin, volatile unsigned long &count,
  unsigned long &count_prev, volatile long &EncoderCount, unsigned long &t_prev,
  float &Theta_prev, float &e_prev, float &inte_prev, float Vmax, float Vmin, float &rpm
) {

  if (count > count_prev) {
    unsigned long t = millis()
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


    WriteDriverVoltage(V, Vmax,DirPinA, DirPinB, PWMPin);

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
  // Serial.print(count * 0.05); Serial.print(" \t");
}

/////////////////////////////angle calculate FOR X AND Y///////////////////////
void angle_calculateX(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt, float Q_angleX, float Q_gyroX, float R_angleX, char C_0X, float K1X)
{
  AngleX = -atan2(ay, az) * (180 / PI); // Radial rotation angle calculation formula; negative sign is direction processing
  Gyro_x = -gx / 131;                   // The X-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
  Kalman_FilterX(AngleX, Gyro_x);       // KalmanFilter
}

void angle_calculateY(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt, float Q_angleY, float Q_gyroY, float R_angleY, char C_0Y, float K1Y)
{
  AngleY = -atan2(ax, az) * (180 / PI); // Radial rotation angle calculation formula; negative sign is direction processing
  Gyro_y = gy / 131;                   // The Y-axis angular velocity calculated by the gyroscope; the negative sign is the direction processing
  Kalman_FilterY(AngleY, Gyro_y);       // KalmanFilter
}
////////////////////////////////////////////////////////////////

///////////////////////////////KalmanFilter for (X)/////////////////////
void Kalman_FilterX(double angle_m, double gyro_m)
{
angleX += (gyro_m - q_biasX) * dt;
  angle_errX = angle_m - angleX;

  Pdot_X[0] = Q_angleX - P_X[0][1] - P_X[1][0];
  Pdot_X[1] = -P_X[1][1];
  Pdot_X[2] = -P_X[1][1];
  Pdot_X[3] = Q_gyroX;

  P_X[0][0] += Pdot_X[0] * dt;
  P_X[0][1] += Pdot_X[1] * dt;
  P_X[1][0] += Pdot_X[2] * dt;
  P_X[1][1] += Pdot_X[3] * dt;

  //Intermediate variable of matrix multiplication
  PCt_0_X = C_0_X * P_X[0][0];
  PCt_1_X = C_0_X * P_X[1][0];
  //Denominator
  E_X = R_angleX + C_0_X * PCt_0_X;
  //Gain value
  K_0X = PCt_0_X / E_X;
  K_1X = PCt_1_X / E_X;

  t_0X = PCt_0_X;
  t_1X = C_0_X * P_X[0][1];

  P_X[0][0] -= K_0X * t_0X;
  P_X[0][1] -= K_0X * t_1X;
  P_X[1][0] -= K_1X * t_0X;
  P_X[1][1] -= K_1X * t_1X;

  q_biasX += K_1X * angle_errX;
  angle_speedX = gyro_m - q_biasX;
  angleX += K_0X * angle_errX;
}
///////////////////////////////KalmanFilter for (Y)/////////////////////
void Kalman_FilterY(double angle_m, double gyro_m)
{
  angleY += (gyro_m - q_biasY) * dt;
  angle_errY = angle_m - angleY;

  Pdot_Y[0] = Q_angleY - P_Y[0][1] - P_Y[1][0];
  Pdot_Y[1] = -P_Y[1][1];
  Pdot_Y[2] = -P_Y[1][1];
  Pdot_Y[3] = Q_gyroY;

  P_Y[0][0] += Pdot_Y[0] * dt;
  P_Y[0][1] += Pdot_Y[1] * dt;
  P_Y[1][0] += Pdot_Y[2] * dt;
  P_Y[1][1] += Pdot_Y[3] * dt;

  // Intermediate variable of matrix multiplication
  PCt_0_Y = C_0_Y * P_Y[0][0];
  PCt_1_Y = C_0_Y * P_Y[1][0];
  // Denominator
  E_Y = R_angleY + C_0_Y * PCt_0_Y;
  // Gain value
  K_0Y = PCt_0_Y / E_Y;
  K_1Y = PCt_1_Y / E_Y;

  t_0Y = PCt_0_Y;
  t_1Y = C_0_Y * P_Y[0][1];

  P_Y[0][0] -= K_0Y * t_0Y;
  P_Y[0][1] -= K_0Y * t_1Y;
  P_Y[1][0] -= K_1Y * t_0Y;
  P_Y[1][1] -= K_1Y * t_1Y;

  q_biasY += K_1Y * angle_errY;
  angle_speedY = gyro_m - q_biasY;
  angleY += K_0Y * angle_errY;
}
