//Completed by Jon Karakus (ECE Queens University)

//Inspired by popular maze car challenges, the goal of this project was to create an autonomous car 
//prototype that would be capable of navigating around household objects in an intelligent manner 
//without any preset navigation commands. A combination of 6 sensors and 2 servo motors were used to complete 
//this engineering task. Further iterations would implement simultaneous localization and mapping algos in order 
//to map the environment while keeping track of the current location. 

//Enjoy the code!!

#include <AFMotor.h>
#include <PID_v2.h>

// PIN DECLARATIONS   //
#define pushButtonPin A0
#define leftIRPin     A1
#define rightIRPin    A2
#define trigPin       A3
#define echoPin       A4
#define bottomIRPin   A5
#define INITLEFTPWM 100
#define INITRIGHTPWM 150
#define sensorPeriod 100 //ms
#define K 5
#define DELAYTIME 200

volatile unsigned long LeftEncoderCount = 0, RightEncoderCount = 0;
unsigned long IRTimeStamp = 0, reflectiveIRTimeStamp = 0, ultrasonicTimeStamp = 0;
float LIRState, RIRState;
long reflectiveIRState;
int LeftPWM, RightPWM;
long distance;
bool circling;
AF_DCMotor left_motor(1, MOTOR34_1KHZ);  // left motor to M1 on motor control board
AF_DCMotor right_motor(3, MOTOR12_1KHZ); // right motor to M3 on motor control board

void setup() {
  // SETTING PINMODES
  pinMode(pushButtonPin, INPUT);    // PushButton
  pinMode(leftIRPin, INPUT);        // Left IR
  pinMode(rightIRPin, INPUT);       // Right IR
  pinMode(trigPin, OUTPUT);         // TRIG
  pinMode(echoPin, INPUT);          // ECHO
  pinMode(bottomIRPin, INPUT);      // Bottom IR
  
  attachInterrupt(1, countLEncoder, RISING); //calls on any rising level on pin 3 (interrupt #1, soldered to Pin3) 
  attachInterrupt(0, countREncoder, RISING); //calls on any rising level on pin 2 (interrupt #0, connected to header on Pin2) 
  interrupts();
  
  Serial.begin (115200);    // set up Serial library at 115200 bps
 
  left_motor.run(RELEASE);  // turn motors off initially
  right_motor.run(RELEASE); 

  checkSensors();
  
  delay(1500);
  
  Forward();
  
}

void loop() {
  //Sense
    // Read Forward facing IR sensors
    if(millis() - IRTimeStamp > sensorPeriod){
      LIRState = getVoltage(leftIRPin);
      RIRState = getVoltage(rightIRPin);
      IRTimeStamp = millis();
      }

    // Read ultrasonic sensor 
    if(millis() - ultrasonicTimeStamp > sensorPeriod){
      distance = getDistance();
      ultrasonicTimeStamp = millis();
      }

    // Read reflective IR sensor
    if(millis() - reflectiveIRTimeStamp > sensorPeriod){
      reflectiveIRState = analogRead(bottomIRPin);
      reflectiveIRTimeStamp = millis();
      }
      
  //Think
    //React if obstacle detetcted
    if(LIRState < 0.3){
      circling = true;
    }
    
    if(abs(LeftEncoderCount - RightEncoderCount)>3){
      Serial.println("Updated");
      LeftPWM -= K*(LeftEncoderCount - RightEncoderCount);
  
      if(LeftPWM>INITLEFTPWM*1.1){
        LeftPWM = INITLEFTPWM*1;
      }
      else if(LeftPWM<(int)INITLEFTPWM/2.5){
        LeftPWM = (int)INITLEFTPWM;
      }
      
      left_motor.setSpeed(LeftPWM);
    }
  if(circling){
    Stop();
    TurnL90();
    Forward();
    while(getVoltage(rightIRPin)<0.3){
      delay(200);
      }
    Stop();
    circling = false;
  }
}

void countLEncoder(){ // interrupt function for left encoder
      LeftEncoderCount++;
}

void countREncoder(){ // interrupt function for right encoder
      RightEncoderCount++;
}

float getVoltage(int pin){
  float voltage;
  voltage = 5.0 * analogRead(pin)/1024;
  return voltage;
}

void Forward(){
  LeftEncoderCount = 0;
  RightEncoderCount = 0;
  
  LeftPWM = INITLEFTPWM;
  RightPWM = INITRIGHTPWM;

  left_motor.run(BACKWARD);
  right_motor.run(BACKWARD);

  left_motor.setSpeed(LeftPWM);
  right_motor.setSpeed(RightPWM);
}

//MOTOR STOP
void Stop(){
  left_motor.setSpeed(0);
  right_motor.setSpeed(0);
  left_motor.run(RELEASE);
  right_motor.run(RELEASE);
  }

//ULTRA SONIC SENSOR
float getDistance(){
  float distance, duration;
  // Send Pulse //
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure high time of echo pin //
  duration = pulseIn(echoPin, HIGH);
  distance = (duration*0.034)/2;
  return distance;
  
  }

//INITIAL SENSOR CHECK
void checkSensors(){
  bool checkIRLeft = false;
  bool checkIRRight = false;
  bool checkUltrasonic = false;
  bool checkBottomIR = false;
  bool checkEncLeft = false;
  bool checkEncRight = false;
  
  while(true){
    if (getVoltage(leftIRPin) < 0.25){
    checkIRLeft = true;
    }
    if (getVoltage(rightIRPin) < 0.25){
    checkIRRight = true;
    }
    if (getDistance() < 10){
    checkUltrasonic = true;
    }
    if (analogRead(bottomIRPin) > 0){
    checkBottomIR = true;
    }
    if (LeftEncoderCount > 10){
    checkEncLeft = true;
    }
    if (RightEncoderCount > 10){
    checkEncRight = true;
    }
    if(checkIRLeft && checkIRRight && checkUltrasonic && checkBottomIR && checkEncLeft && checkEncRight){
      break;
      }
    }
  LeftEncoderCount  = 0;
  RightEncoderCount = 0;
 }

//LEFT TURN
void TurnL90(){
  left_motor.run(FORWARD);
  right_motor.run(BACKWARD);

  LeftPWM = -INITLEFTPWM;
  RightPWM = (int)INITRIGHTPWM;

  LeftEncoderCount = 0;
  RightEncoderCount = 0;

  left_motor.setSpeed(LeftPWM);
  right_motor.setSpeed(RightPWM);

  while(LeftEncoderCount<15 && RightEncoderCount<15){ // Turn in progress
    delay(DELAYTIME);
  }

  Stop();
}

//RIGHT TURN
void TurnR90(){
  left_motor.run(BACKWARD);
  right_motor.run(FORWARD);

  RightPWM = -INITRIGHTPWM;
  LeftPWM = (int)INITLEFTPWM;

  LeftEncoderCount = 0;
  RightEncoderCount = 0;

  left_motor.setSpeed(LeftPWM);
  right_motor.setSpeed(RightPWM);

  while(LeftEncoderCount<20 && RightEncoderCount<20){ // Turn in progress
    delay(DELAYTIME);
  }

  Stop();
  }
