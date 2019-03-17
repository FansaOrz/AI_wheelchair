void controlUpdate(){
  
  int potIn = analogRead(PIN_POT);
  
  throttle=map(potIn, 0, 1023, 1000, 2000);
  
  pid_update();
  pid_compute();
  
  /*
  Serial.print (pid_roll_out);
      Serial.print(", ");

  Serial.print (pid_pitch_out);
        Serial.print(", ");

  Serial.println (pid_yaw_out);*/

  m0 = throttle - pid_roll_out - pid_pitch_out + pid_yaw_out;
  m1 = throttle + pid_roll_out - pid_pitch_out - pid_yaw_out;
  m2 = throttle + pid_roll_out + pid_pitch_out + pid_yaw_out;
  m3 = throttle - pid_roll_out + pid_pitch_out - pid_yaw_out;
  
  /*Serial.print(m0);
    Serial.print(", ");

  Serial.print(m1);
      Serial.print(", ");

  Serial.print(m2);
      Serial.print(", ");

  Serial.println(m3);*/

  updateMotors(m0,m1,m2,m3);
}


