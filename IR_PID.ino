#include <boarddefs.h>
#include <IRremote.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>

#include <CPE123_EncoderLib_Fall_17.h>
#include <CPE123_Fall17.h>
#include <PID_v1.h>

//Pins to control right motor
#define rightForward 5
#define rightBackward 4

//Pins to control left motor
#define leftForward 7
#define leftBackward 6

// Pins to control ultrasonic sensor
#define echo 30
#define trig 32

#define rightEncoderPin1 2
#define rightEncoderPin2 3
#define leftEncoderPin1 20
#define leftEncoderPin2 21

#define DEFAULT_MOVEMENT_SPEED 100

#define STOPPED 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT_TURN 3
#define RIGHT_TURN 4
int globalMovementState = STOPPED;

int RECV_PIN = 12;
IRrecv irrecv(RECV_PIN);
decode_results results;

double left_speed_val = 0, right_speed_val = 0, left_encoder_val = 0, right_encoder_val = 0;
//double kp = 10.0, ki = 0, kd = 0;   //Values for the PID

double kp = 0.27, ki = 0.85, kd = 0;   //Values for the PID

/*
   input - left encoder val
   output - left speed val, what we are trying to modify
   set point - right encoder val, what we want the input to match
   to tune change kp ki kd
*/
PID leftPID(&left_encoder_val, &left_speed_val, &right_encoder_val, kp, ki, kd, DIRECT);

bool moveForward(double right_speed_val, double left_speed_val, int distance)
{
  globalMovementState = FORWARD;

  analogWrite(rightForward, right_speed_val);
  analogWrite(rightBackward, 0);

  analogWrite(leftForward, left_speed_val);
  analogWrite(leftBackward, 0);

  return true;
}

bool moveBackward(double right_speed_val, double left_speed_val, int distance)
{
  globalMovementState = BACKWARD;

  analogWrite(rightForward, 0);
  analogWrite(rightBackward, right_speed_val);

  analogWrite(leftForward, 0);
  analogWrite(leftBackward, left_speed_val);

  return true;
}

bool pivotLeftNinety()
{
  resetEncoders();
  globalMovementState = LEFT_TURN;
  left_speed_val = DEFAULT_MOVEMENT_SPEED;
  right_speed_val = DEFAULT_MOVEMENT_SPEED;

  analogWrite(rightForward, right_speed_val);
  analogWrite(rightBackward, 0);

  analogWrite(leftForward, 0);
  analogWrite(leftBackward, left_speed_val);

  while (rightEncoderCount() < 1420);

  stopMovement();
  return true;
}

bool pivotRightNinety()
{
  resetEncoders();
  globalMovementState = RIGHT_TURN;
  left_speed_val = DEFAULT_MOVEMENT_SPEED;
  right_speed_val = DEFAULT_MOVEMENT_SPEED;

  analogWrite(rightForward, 0);
  analogWrite(rightBackward, right_speed_val);

  analogWrite(leftForward, left_speed_val);
  analogWrite(leftBackward, 0);

  while (rightEncoderCount() < 1420);

  stopMovement();
  return true;
}

bool pivotRightFifteen()
{
  resetEncoders();
  globalMovementState = RIGHT_TURN;
  left_speed_val = DEFAULT_MOVEMENT_SPEED;
  right_speed_val = DEFAULT_MOVEMENT_SPEED;

  analogWrite(rightForward, 0);
  analogWrite(rightBackward, right_speed_val);

  analogWrite(leftForward, left_speed_val);
  analogWrite(leftBackward, 0);

  while (rightEncoderCount() < 125);

  stopMovement();
  return true;
}

bool pivotLeftFifteen()
{
  resetEncoders();
  globalMovementState = LEFT_TURN;
  left_speed_val = DEFAULT_MOVEMENT_SPEED;
  right_speed_val = DEFAULT_MOVEMENT_SPEED;

  analogWrite(rightForward, right_speed_val);
  analogWrite(rightBackward, 0);

  analogWrite(leftForward, 0);
  analogWrite(leftBackward, left_speed_val);

  while (rightEncoderCount() < 125);

  stopMovement();
  return true;
}

bool stopMovement()
{
  resetEncoders();
  globalMovementState = STOPPED;
  left_speed_val = DEFAULT_MOVEMENT_SPEED;
  right_speed_val = DEFAULT_MOVEMENT_SPEED;

  leftPID.SetMode(AUTOMATIC);

  analogWrite(rightForward, 0);
  analogWrite(rightBackward, 0);

  analogWrite(leftForward, 0);
  analogWrite(leftBackward, 0);
}

/* Function is used to calculate how far wall is from US sensor
   As of right now, values are printed to Serial terminal. Will need
   to use this data to figure out when to make robot turn.
*/
long distance()
{
  long duration, distance; // CREATE VARIABLES
  digitalWrite(trig, LOW); //  TURN OFF TRIG PIN
  delayMicroseconds(2);       // WAIT
  digitalWrite(trig, HIGH); // TURN ON TRIG PIN
  delayMicroseconds(10);       // WAIT
  digitalWrite(trig, LOW);  // TURN OFF TRIG PIN
  duration = pulseIn(echo, HIGH);   // WAITS FOR ECHO PIN TO BE HIGH
  distance = (duration / 2) / 29.1;
  return distance;
}

unsigned long calcLeftAngle(int angle)
{
  //2249.675
  unsigned long encTrans = (angle / 360 * 2249.675);
  return encTrans;
}


unsigned long calcRightAngle(int angle)
{
  //3197.84
  unsigned long encTrans = (angle / 360 * 3197.84);
  return encTrans;
}

//Dist is in cm
unsigned long calcLeftDistance(int dist)
{
  unsigned long wheelRatio = dist / 21.363;
  unsigned long numDegrees = wheelRatio * 360;
  return calcLeftAngle(numDegrees);
}

unsigned long calcRightDistance(int dist)
{
  unsigned long wheelRatio = dist / 21.363;
  unsigned long numDegrees = wheelRatio * 360;
  return calcRightAngle(numDegrees);
}

void printEncoderSpeed() {
  Serial.print("Difference: ");
  Serial.println(left_encoder_val - right_encoder_val);
  Serial.print("Left: ");
  Serial.print(leftEncoderCount());
  Serial.print("\t ");
  Serial.println(left_speed_val);
  Serial.print("Right: ");
  Serial.print(rightEncoderCount());
  Serial.print("\t ");
  Serial.println(right_speed_val);
  Serial.println();
}

void printKValues() {
  Serial.print("kp = ");
  Serial.print(kp);
  Serial.print("\t ki = ");
  Serial.print(ki);
  Serial.print("\t kd = ");
  Serial.println(kd);
}

void pollIrSensor()
{
  if (irrecv.decode(&results))
  {
    if (results.value == 0xFF38C7)
    {
      //Stopped
      stopMovement();
    }
    else if (results.value == 0xFF18E7)
    {
      //Forward
      moveForward(right_speed_val, left_speed_val, 1000);
    }
    else if (results.value == 0xFF4AB5)
    {
      //Backward
      moveBackward(right_speed_val, left_speed_val, 1000);
    }
    else if (results.value == 0xFF10EF)
    {
      //Left
      //pivotLeftNinety();
      pivotLeftFifteen();
    }
    else if (results.value == 0xFF5AA5)
    {
      //Right
      //pivotRightNinety();
      pivotRightFifteen();
    }
    irrecv.resume(); // Receive the next value
  }
  else
  {
    if (globalMovementState == STOPPED)
    {
      stopMovement();
    }
    else if (globalMovementState == FORWARD)
    {
      moveForward(right_speed_val, left_speed_val, 1000);
    }
    else if (globalMovementState == BACKWARD)
    {
      moveBackward(right_speed_val, left_speed_val, 1000);
    }
  }
}


/* Sets all pins as inputs or outputs
   Encoder pins correspond to RPMs for each motor, we aren’t sure
   which pins go with which encoders yet
*/

void setup() {
  Serial.begin(9600);
  delay(500);

  irrecv.enableIRIn(); // Start the receiver

  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);

  left_speed_val = 100;
  right_speed_val = 100;

  encoderSetup(rightEncoderPin1, rightEncoderPin2, leftEncoderPin1, leftEncoderPin2);
  leftPID.SetMode(AUTOMATIC);  //Turns PID on
  leftPID.SetSampleTime(50);   //How often in milliseconds the PID will be evaluated
}

//Main loop
void loop() 
{
  left_encoder_val = leftEncoderCount();
  right_encoder_val = rightEncoderCount();
  leftPID.Compute();
  pollIrSensor();
}
