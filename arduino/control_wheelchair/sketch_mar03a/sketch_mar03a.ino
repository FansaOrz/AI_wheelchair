/*
control wheelchair motors
*/

#include <ros.h>
#include <rosserial_arduino/command.h>

ros::NodeHandle nh;
//front back
int X_pin = 9;
//left right
int Y_pin = 10;
void messageCb( const rosserial_arduino::command& command){
  float X = command.lwheel_vtarget;
  float Y = command.rwheel_vtarget; 
  int OUTPUT_X = 0;
  int OUTPUT_Y = 0;
  if(X == 0.0 || X == -0.0){
     pinMode(9, INPUT);
  }
  else{
    pinMode(9, OUTPUT);
    if(X > 0){
      OUTPUT_X = 50 - int(50 * X);
      analogWrite(X_pin, OUTPUT_X);
    }
    else{
      OUTPUT_X = -51 - int(50 * X);
      analogWrite(X_pin, OUTPUT_X);
    }
  }
  if(Y == 0.0 || Y == -0.0){
    pinMode(10, INPUT);
  }
  else{
    pinMode(10, OUTPUT);
    if(Y > 0){
      OUTPUT_Y = 51 - (50*Y);
      analogWrite(Y_pin, OUTPUT_Y);
    }
    else{
      OUTPUT_Y = -51 - (50 * Y);
      analogWrite(Y_pin, OUTPUT_Y);
    }
  }
}

ros::Subscriber<rosserial_arduino::command> sub("/command_velocity", &messageCb );

void setup()
{
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
