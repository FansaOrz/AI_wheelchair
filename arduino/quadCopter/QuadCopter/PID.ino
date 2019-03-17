/*
  PID Interface v.1.0 SNAPSHOT
  Author - Juan López
  Last Update - 07/12/16
*/

PID roll_controller(&pid_roll_in,   &pid_roll_out,  &pid_roll_setpoint,  0.250, 0.950, 0.011, REVERSE);
PID pitch_controller(&pid_pitch_in, &pid_pitch_out, &pid_pitch_setpoint, 0.250, 0.950, 0.011, REVERSE);
PID yaw_controller(&pid_yaw_in,     &pid_yaw_out,   &pid_yaw_setpoint,   1.0, 0.0, 0.0, DIRECT);  

void pid_initialize() {
  
  roll_controller.SetOutputLimits(-200.0, 200.0);
  pitch_controller.SetOutputLimits(-200.0,200.0);
  yaw_controller.SetOutputLimits(100.0,100.0);
  roll_controller.SetMode(AUTOMATIC);
  pitch_controller.SetMode(AUTOMATIC);
  yaw_controller.SetMode(AUTOMATIC);
  roll_controller.SetSampleTime(10);
  pitch_controller.SetSampleTime(10);
  yaw_controller.SetSampleTime(10);
  
}

void pid_update(){
  pid_roll_in = rollX;
  pid_pitch_in = pitchY;
  pid_yaw_in = yawZ; 
}

void pid_compute() {
   roll_controller.Compute();
   pitch_controller.Compute();
   yaw_controller.Compute();
}


