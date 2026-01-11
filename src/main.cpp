#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Motor Pins
const uint8_t motorPWM[] = {9, 10};
const uint8_t motorIN1[] = {4, 6};
const uint8_t motorIN2[] = {5, 7};

// Encoder Pins
const int encoderA[] = {2, 3};
const int encoderB[] = {13, 12};

// Button
const uint8_t buttonPin = 8;

// Movement Parameters
const int straightSpeed = 140;
const int brakePower = 100;
const int brakeDuration = 120;
const int turnPower = 100;
const int turnBrakePower = 120;
const float turnTolerance = 1; // Degrees

// ?Global Variables
volatile int encoderPos[] = {0, 0};
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

long prevTime;
long deltaTime;
double gyroHeadings[3];
float targetAngle = 0;

bool waitingForButton = false;

// ?Forward Declarations
double headingDiff(double h1, double h2);
template <int j> void readEncoder();
void fwd(int ticks);
void back(int ticks);
void right(int angle);
void left(int angle);
void moveStraight(int targetTicks, char direction);
void turn(float angle, int direction);
void setMotor(int dir, int pwmVal, int motorIndex);
void stopMotors();
void applyBrake(char direction);
void initGyro();
void updateGyro();
void computeDeltaT();

// ?PID Controller
class SimplePID {
public:
  float kp, kd, ki, umax, umin;
  float eprev, eintegral;

  SimplePID() : kp(1), kd(0), ki(0), umax(255), umin(0), eprev(0.0), eintegral(0.0) {}

  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn, float uminIn) {
    kp = kpIn;
    kd = kdIn;
    ki = kiIn;
    umax = umaxIn;
    umin = uminIn;
  }

  void evaluate(double value, double target, float deltaT, int &pwr, int &dir) {
    double error = headingDiff(target, value);
    float derivative = (error - eprev) / deltaT;
    eintegral += error * deltaT;
    float output = kp * error + kd * derivative + ki * eintegral;

    pwr = constrain((int)fabs(output), umin, umax);
    dir = (output < 0) ? -1 : 1;
    eprev = error;
  }
};

SimplePID pidStraight;
SimplePID pidTurn;

// ?Setup
void setup() {
  Serial.begin(9600);

  // Configure PID controllers
  pidStraight.setParams(4, 0, 0, 255, 0);
  pidTurn.setParams(1.0, 0.25, 0.0, 200, 80);

  // Initialize Button
  pinMode(buttonPin, INPUT_PULLUP);

  // Initialize Encoders
  attachInterrupt(digitalPinToInterrupt(encoderA[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderA[1]), readEncoder<1>, RISING);

  if (!bno.begin()) {
    Serial.println("BNO055 Not Detected");
  } else {
    Serial.println("BNO055 Initialized");
  }

  initGyro();
  delay(500);
  
  /*
  setMotor(-1, 255, 0);
  setMotor(1, 255, 1);
  delay(10000);
  stopMotors();
  Serial.println("Motor Test Complete");
  */

  Serial.println("Initialization Successful");
  waitingForButton = true;
}

// ?Main Loop
void loop() {
  // Full Tile: 780 Ticks, fwd(fullTile);
  // Half Tile: 400 Ticks, fwd(halfTile);
  // Robot Length: 100 Ticks

  const int fullTile = 780;
  const int halfTile = 400;
  const int robotLength = 180;

  while (waitingForButton) {
    if (digitalRead(buttonPin) == 0) {
      waitingForButton = false;
      Serial.println("Starting Sequences");
      delay(100); // Debounce
    } else {
      delay(10);
    }
  }

  fwd(robotLength);
  fwd(halfTile);
  fwd(fullTile);
  left(90);
  fwd(fullTile);
  right(90);
  fwd(fullTile);
  back(fullTile);
  right(90);
  fwd(fullTile);
  fwd(fullTile);
  back(fullTile);
  right(90);
  fwd(fullTile);
  left(90);
  fwd(fullTile);
  back(robotLength);

  //fwd(fullTile*4);

  // Stop robot
  stopMotors();
  while(1) {delay(100000000);}
}

// ?Movement Commands
void fwd(int ticks) {
  moveStraight(ticks, 'f');
  delay(50);
}

void back(int ticks) {
  moveStraight(ticks, 'b');
  delay(50);
}

void right(int angle) {
  turn(angle, 1);
  delay(50);
}

void left(int angle) {
  turn(-angle, -1);
  delay(50);
}

// ?Movement Implementations
void moveStraight(int targetTicks, char direction) {
  encoderPos[1] = 0;

  while (abs(encoderPos[1]) < targetTicks) {
    updateGyro();
    computeDeltaT();

    int correctionPwr, correctionDir;
    pidStraight.evaluate(gyroHeadings[0], targetAngle, deltaTime, correctionPwr, correctionDir);

    int leftPwr = straightSpeed - (correctionPwr * correctionDir);
    int rightPwr = straightSpeed + (correctionPwr * correctionDir);

    leftPwr = constrain(leftPwr, 0, 255);
    rightPwr = constrain(rightPwr, 0, 255);

    if (direction == 'f') {
      setMotor(-1, leftPwr, 0);
      setMotor(1, rightPwr, 1);
      Serial.println("Left Power: " + String(leftPwr) + " Right Power: " + String(rightPwr));
      Serial.println("Heading: " + String(gyroHeadings[0]) + " Target: " + String(targetAngle) + "\n");
      //Serial.println(correctionPwr);
    } else {
      setMotor(1, rightPwr, 0);
      setMotor(-1, leftPwr, 1);
    }
  }

  applyBrake(direction);
  stopMotors();
  delay(100);
}

void turn(float angle, int direction) {
  targetAngle += angle;

  // Normalize angle
  if (targetAngle <= -360) targetAngle += 360;
  else if (targetAngle >= 360) targetAngle -= 360;

  while (abs(headingDiff(gyroHeadings[0], targetAngle)) > turnTolerance) {
    updateGyro();
    computeDeltaT();

    int pwr, motorDir;
    pidTurn.evaluate(gyroHeadings[0], targetAngle, deltaTime, pwr, motorDir);

    setMotor(direction, turnPower, 0);
    setMotor(direction, turnPower, 1);
  }

  // Counter-rotation to stop
  setMotor(-direction, turnBrakePower, 0);
  setMotor(-direction, turnBrakePower, 1);
  delay(brakeDuration);

  stopMotors();
  delay(100);
}

// ?Motor Control
void setMotor(int dir, int pwmVal, int motorIndex) {
  analogWrite(motorPWM[motorIndex], pwmVal);

  if (dir == -1) {
    digitalWrite(motorIN1[motorIndex], HIGH);
    digitalWrite(motorIN2[motorIndex], LOW);
  } else if (dir == 1) {
    digitalWrite(motorIN1[motorIndex], LOW);
    digitalWrite(motorIN2[motorIndex], HIGH);
  } else {
    digitalWrite(motorIN1[motorIndex], LOW);
    digitalWrite(motorIN2[motorIndex], LOW);
  }
}

void stopMotors() {
  setMotor(0, 0, 0);
  setMotor(0, 0, 1);
}

void applyBrake(char direction) {
  if (direction == 'f') {
    setMotor(1, brakePower, 0);
    setMotor(-1, brakePower, 1);
  } else {
    setMotor(-1, brakePower, 0);
    setMotor(1, brakePower, 1);
  }
  
  delay(brakeDuration);
}

// ?Gyro Functions
void initGyro() {
  if (!bno.begin()) {
    while (1) delay(10); // Halt if gyro fails
  }
  bno.setExtCrystalUse(true);
  prevTime = millis();
}

void updateGyro() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyroHeadings[0] = euler.x();
  gyroHeadings[1] = euler.y();
  gyroHeadings[2] = euler.z();
}

// ?Utility Functions
void computeDeltaT() {
  long currTime = millis();
  deltaTime = currTime - prevTime;
  prevTime = currTime;
}

double headingDiff(double h1, double h2) {
  double left = h1 - h2;
  double right = h2 - h1;
  if (left < 0) left += 360;
  if (right < 0) right += 360;
  return left < right ? -left : right;
}

template <int j>
void readEncoder() {
  int b = digitalRead(encoderB[j]);
  encoderPos[j] += (b > 0) ? 1 : -1;
}