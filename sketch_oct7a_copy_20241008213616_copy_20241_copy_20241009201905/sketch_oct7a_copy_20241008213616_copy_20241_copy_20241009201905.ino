#include <QTRSensors.h>

QTRSensors qtr;                        //qtr sensor array
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


//var for motor driver
int ael=9;                        //el-elable is a value bet 0 to 255 for speed
int aph=6;                        //ph-phase is direction of the motor
int bel=5;                        
int bph=3;
int mode=8;                       //mode is operating mode with the values lke speed which in per(%)


int button_Cal=2;                 //calibrate
int button_NF=7;                  //on and off


int lastError
double Kp=0.09;
double Ki=0;
double Kd=0.9;


void setup() {

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){10,11,12,A0,A1,A2,A3,A4,A5}, SensorCount);
  qtr.setEmitterPin(7);

  pinMode(ael, OUTPUT);
  pinMode(aph, OUTPUT);
  pinMode(bel, OUTPUT);
  pinMode(bph, OUTPUT);
  pinMode(mode, OUTPUT);
  pinMode(button_Cal,INPUT);
  pinMode(button_NF,INPUT);
  digitalWrite(mode, HIGH);


  while(digialRead(button_Cal)==LOW){}
  calibratesensor()
  while(digialRead(button_NF)==LOW){}


}

void loop() {
   PID()
}

void PID(){
  uint16_t positionLine=qtr.readLineBlack(sensorValues);
  int error = 3500 - positionLine;

  int P = error;
  int I = error+I;
  int D = error-lastError;

  lastError = error;
  
  int motorspeedchange=(P*Kp)+(I*Ki)+(D*kp);

  int motorSpeedA = 100 + motorspeedchange;
  int motorSpeedB = 100 - motorspeedchange;

  if (motorSpeedA > 255){
    motorSpeedA=200
  }
  if (motorSpeedB > 255){
    motorSpeedB=200
  }
  if (motorSpeedA < -75){
    motorSpeedA=-75
  }
  if (motorSpeedB < -75){
    motorSpeedB =-75
  }
  
  motor_movement(motorSpeeA,motorSpeeA);
}


//calibrate

void calibratesensor(){

  for (uint16_t i = 0; i < 400; i++)
  {
    digialWrite(aph,50);
    digialWrite(bph,-50);
    qtr.calibrate();

  }  

}


// speeda - speed of motor a 
// speedb - speed of motor b
void motor_movement(int speedA,int speedB){      // speeda - speed of motor a and speedb - speed of motor b
  
  //motor A backward or forward
  if (speedA < 0){

    speedA=0-speedA                              //making the value +ve for the input
    digitalWrite(aph,HIGH);

  }
  else{
    digitalWrite(aph,LOW);
  }


  //motor B backward or forward
  if (speedB < 0){

    speedB=0-speedB
    digitalWrite(bph,HIGH);

  }
  else{
    digitalWrite(bph,LOW);
  }  
 
  analogWrite(ael,speedA);        // motor A
  analogWrite(bel,speedA);        // motor B
  

}