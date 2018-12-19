#include <CPE123_EncoderLib_Fall_17.h>
#include <CPE123_Fall17.h>
#include <PID_AutoTune_v0.h>
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

byte ATuneModeRemember=2;
double left_encoder_val=00, left_speed_val=100, right_encoder_val=0, right_speed_val = 100;
double kp=2,ki=0.5,kd=2;

double kpmodel=1.5, taup=100, theta[50];
double left_speed_valStart=5;
double aTuneStep=30, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=.1;

boolean tuning = false;
unsigned long  modelTime, serialTime;

PID leftPID(&left_encoder_val, &left_speed_val, &right_encoder_val,kp,ki,kd, DIRECT);
PID_ATune aTune(&left_encoder_val, &left_speed_val);

bool moveForward(double right_speed_val, double left_speed_val, int distance)
{
    analogWrite(rightForward, right_speed_val);
    analogWrite(rightBackward, 0);

    analogWrite(leftForward, left_speed_val);
    analogWrite(leftBackward, 0);
  return true;
}

void printEncoderSpeed(){
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

void printKValues(){
  Serial.print("kp = ");
  Serial.print(kp);
  Serial.print("\t ki = ");
  Serial.print(ki);
  Serial.print("\t kd = ");
  Serial.println(kd);
}

void setup()
{

  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
 
  left_speed_val = 100;
  right_speed_val = 100;

  //Setup the pid 
  encoderSetup(rightEncoderPin1, rightEncoderPin2, leftEncoderPin1, leftEncoderPin2);
  leftPID.SetSampleTime(1000);   //How often in milliseconds the PID will be evaluated
  leftPID.SetMode(AUTOMATIC); //done

  if(tuning)
  {
    tuning=false;
    changeAutoTune();
    tuning=true;
  }
  
  serialTime = 0;
  Serial.begin(9600);
}


void loop()
{

  unsigned long now = millis();

  moveForward(right_speed_val, left_speed_val, 1000); // right left distance
  left_encoder_val = leftEncoderCount();
  right_encoder_val = rightEncoderCount();
  
  if(tuning)
  {
    
    Serial.println("A TUNE RUNNING");
    
    byte val = (aTune.Runtime());
    Serial.println(val);
    if (val!=0)
    {
      tuning = false;
    }
    if(!tuning)
    { //we're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      leftPID.SetTunings(kp,ki,kd);
      AutoTuneHelper(false);
    }
  }
  else leftPID.Compute();
  

  analogWrite(0,left_speed_val); 
  

  
  //send-receive with processing if it's time
  if(millis()>serialTime)
  {
    Serial.println("in serial time");
    SerialReceive();
    SerialSend();
    serialTime+=500;
  }
}

void changeAutoTune()
{
  Serial.println("IN change autotune");
 if(!tuning)
  {
    //Set the left_speed_val to the desired starting frequency.
    left_speed_val = aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    Serial.println("IN HERE");
    tuning = false;
    AutoTuneHelper(false);
  }
}

void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = leftPID.GetMode();
  else
    leftPID.SetMode(ATuneModeRemember);
}


void SerialSend()
{
  Serial.print("right_encoder_val: ");Serial.print(right_encoder_val);
  Serial.print("\t left_encoder_val: ");Serial.print(left_encoder_val);
  Serial.print("\t left_speed_val: ");Serial.print(left_speed_val);
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("\t kp: ");Serial.print(leftPID.GetKp());Serial.print(" ");
    Serial.print("\t ki: ");Serial.print(leftPID.GetKi());Serial.print(" ");
    Serial.print("\t kd: ");Serial.print(leftPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning)){
      Serial.println("WHY R U IN HERE");
      changeAutoTune();
   }
  }
}
