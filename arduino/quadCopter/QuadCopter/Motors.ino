/*
  Brushless Motors Control v.1.0
  Author - Juan López
  Last Update - 15/8/16
*/
#define PIN_MOTOR1 2
#define PIN_MOTOR2 3 
#define PIN_MOTOR3 4 
#define PIN_MOTOR4 5 


Servo motor0;
Servo motor1;
Servo motor2;
Servo motor3;

void initializeMotors(){
  
  motor0.attach(PIN_MOTOR1);		
  motor1.attach(PIN_MOTOR2);
  motor2.attach(PIN_MOTOR3);
  motor3.attach(PIN_MOTOR4);
  motor0.write(0);				
  motor1.write(0);
  motor2.write(0);
  motor3.write(0);
 
}

void updateMotors(int m0, int m1, int m2, int m3)
{
  motor0.writeMicroseconds(m0);
  motor1.writeMicroseconds(m1);
  motor2.writeMicroseconds(m2);
  motor3.writeMicroseconds(m3);
}
