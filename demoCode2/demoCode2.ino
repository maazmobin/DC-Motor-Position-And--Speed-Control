#define buffer_size 1024
#define ratebuffer_size 5
#include <CircularBuffer.h>

float vSum = 0 , vAverage = 0;
float iSum = 0 , iAverage = 0;
float iError = 0 , vError = 0;
float rateSum = 0 , rateAverage = 0;
int loopCall = 0;
CircularBuffer<float, buffer_size> vBuffer;
CircularBuffer<float, buffer_size> iBuffer;
CircularBuffer<float, ratebuffer_size> encRateBuffer;

#define vSamplePin A0
#define vSamplePin A3

#define DEBUG 0
#define ticksPerRev 1540
#include <Encoder.h>
Encoder enc(0, 1);
long encCurr = 0;
float encRate = 0;  //ticks per second
long encBuff = 0;
float motorPosition = 0 ; //radians
int enc_sign = 1;

#include "MusafirMotor.h"
MusafirMotor motor(7, 6, 5); //5 is pwm

unsigned long currentMillis = 0;

unsigned long previousMillis = 0;
int Interval = 10; // in ms         // PID sample interval

int echoStatusMode = 1, duration = 40;  // Parameters for Broadcast mode and duration for repeated broadcast.
unsigned long echoPreviousMillis = 0;

unsigned long timeOutPreviousMillis = 0;  //Velocity Time Out.
int velocityTimeOut = 10000; //ms

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;

float max_pwm = 250;
float min_pwm = -250;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Serial.begin(9600);
  motor.setDir(FORWARD);
  pinMode(13,OUTPUT);
  pinMode(vSamplePin, INPUT);
  analogReadResolution(12);
  for(int i=0 ; i<=buffer_size+1000 ; i++){
  MeasureMotorVoltage();
  MeasureMotorCurrent();
  }
  iError = iAverage;
  vError = vAverage;    
}

void loop() {

  digitalWrite(13,HIGH);
  currentMillis = millis();
 if (stringComplete) {
    interpretSerialData();
    stringComplete = false;
    inputString = "";
  }
  
    if (currentMillis - previousMillis >= Interval ) {  // 
      previousMillis = currentMillis;
      MeasureEncPara();
            
      echoData();      
  } 
  MeasureMotorVoltage();
  MeasureMotorCurrent();
   /* if (currentMillis - timeOutPreviousMillis >= velocityTimeOut ) {  // Break Velocity after Certain Time.
    brakeMotor(250);
  }*/
}

void MeasureEncPara(void)
{
      encCurr = enc_sign * enc.read(); enc.write(0);
      encBuff += encCurr;
      motorPosition = (encBuff/(float)ticksPerRev) * 2.0 * 3.142; //3.142 is the convertion towards radians
      encRate=((float)encCurr/(float)Interval) * 1000.0;
      rateSum -= encRateBuffer[0];
      encRateBuffer.push(encRate);
      rateSum += encRateBuffer[ratebuffer_size-1];
      rateAverage = rateSum/(ratebuffer_size);
  }
  
void MeasureMotorVoltage(void)
{
  float reading = (11*3.3*(analogRead(A0))/4096)+0.7; // 1K/(1K+10K)=0.09
  vSum -= vBuffer[0];
  vBuffer.push(reading);
  vSum += vBuffer[buffer_size-1];
  vAverage = vSum/(buffer_size-1);
  if(DEBUG){
  Serial.print(vBuffer[0]);
  Serial.print(" , ");
  Serial.print(vBuffer[buffer_size]);
    Serial.print(" vAverage is ");
    Serial.println(vAverage);
  }
}

void MeasureMotorCurrent(void)
{
  float Ireading = (2*3.3*(analogRead(A3))/4096.0); //
  Ireading = (5.5556*Ireading)-13.9; 
  iSum -= iBuffer[0];
  iBuffer.push(Ireading);
  iSum += iBuffer[buffer_size-1];
  iAverage = iSum/(buffer_size-1);
  if(DEBUG){
  Serial.print(iBuffer[0]);
  Serial.print(" , ");
  Serial.print(iBuffer[buffer_size]);
  Serial.print(" Current average is ");
  Serial.println(iAverage);
  }
}
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  if(DEBUG == true){
  Serial.print("Serial Receive: ");
  Serial.print(inputString);
  }
}

void interpretSerialData(void) {
  int c1 = 1, c2 = 1;
  float val1 = 0, val2 = 0;
  switch (inputString[0]) {
    case 'L':
      c1 = inputString.indexOf(',') + 1;
      val1 = inputString.substring(c1).toInt();
      val1 = constrain( val1, -250 , 250 );
      timeOutPreviousMillis=currentMillis;
      driveMotor(val1);
     if(DEBUG == true){
      Serial.print("PWM Receive: ");
      Serial.println(val1);
      }
      break;
    default:
      Serial.print("UNKNOWN COMMAND: ");
      Serial.println(inputString);
      break;
  }
}

void brakeMotor(int intensity)
{
  intensity = constrain(intensity, 0, max_pwm); //Double Checks
  motor.setPWM(intensity);
  motor.setDir(BRAKE); 
}

void driveMotor(int intensity)
{
  if(intensity > 0)
  {motor.setDir(FORWARD);}
  else if(intensity < 0)
  {motor.setDir(BACKWARD);}
  else if(intensity == 0)
  {brakeMotor(250);}
  intensity = abs(intensity);
  intensity = constrain(intensity, 0, max_pwm);
  motor.setPWM(intensity);
  if(DEBUG == true){
    Serial.print("Applying PWM: ");
    Serial.println(intensity);
  }  
}
void echoData(void)
{
  Serial.print("ZP,");    //START SIGNATURE
  Serial.print(encCurr);
  Serial.print(",");
  Serial.print(encBuff);
  Serial.print(",");
  Serial.print(motorPosition);
  Serial.print(",");
  Serial.print(encRate);
  Serial.print(",");  
  Serial.print(vAverage-vError);
  Serial.print(",");
  Serial.println(iAverage-iError);
  }



