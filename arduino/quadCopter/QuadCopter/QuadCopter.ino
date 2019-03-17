/*
  Tornicopter v.1.0 SNAPSHOT
  Author - Juan López - 15/8/16
  Last Update - 07/12/16
*/

/*TODO --> CONFIGURATION.H*/

#include <Servo.h>
#include <PID_v1.h>
#include <Wire.h>
#include <ADXL345.h> 
#include <HMC5883L.h> 
#include <ITG3200.h>

//Angles
float rollX,pitchY,yawZ = 0.0;

// RX Signals
int throttle = 0;

// PID variables
double pid_roll_in,   pid_roll_out,   pid_roll_setpoint = 0;
double pid_pitch_in,  pid_pitch_out,  pid_pitch_setpoint = 0;
double pid_yaw_in,    pid_yaw_out,    pid_yaw_setpoint = 0;

#define PIN_POT A3 // Potentiometer (Analog)


// Motors
int m0, m1, m2, m3; // Front, Right, Back, Left

void setup(){
  
  Serial.begin(9600);
  configureIMU();
}

void loop(){
  imuUpdate();
}
