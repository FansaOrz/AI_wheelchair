/*
control wheelchair motors
*/

#include <ros.h>
#include <rosserial_arduino/command.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;
std_msgs::String str_msg;
 int encoder0PinA = 51;
 int encoder0PinB = 52;
 int n = LOW;
 int m = LOW;
 char msg[3] = "00";
//front back
int X_pin = 12;
//left right
int Y_pin = 13;
void messageCb( const rosserial_arduino::command& command){
  float X = command.lwheel_vtarget;
  float Y = command.rwheel_vtarget; 
  int OUTPUT_X = 0;
  int OUTPUT_Y = 0;
  if(X == 0.0 || X == -0.0){
     pinMode(X_pin, INPUT);
  }
  else{
    pinMode(X_pin, OUTPUT);
    if(X > 0){
      OUTPUT_X = 51 - int(50 * X);
      analogWrite(X_pin, OUTPUT_X);
    }
    else{
      OUTPUT_X = -51 - int(50 * X);
      analogWrite(X_pin, OUTPUT_X);
    }
  }
  if(Y == 0.0 || Y == -0.0){
    pinMode(Y_pin, INPUT);
  }
  else{
    pinMode(Y_pin, OUTPUT);
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
ros::Publisher encoder("encoder", &str_msg);

void setup()
{
  pinMode (encoder0PinA,INPUT);
  pinMode (encoder0PinB,INPUT);
  pinMode(X_pin, INPUT);
  pinMode(Y_pin, INPUT);
  nh.initNode();
  nh.advertise(encoder);
  nh.subscribe(sub);
}

void loop()
{
  delay(1);
   n = digitalRead(encoder0PinA);
   if(n == LOW)
     msg[0] = '0';
   else
     msg[0] = '1';
   //Serial.print(n);
   delay(1);
   m = digitalRead(encoder0PinB);
   if(m == LOW)
     msg[1] = '0';
   else
     msg[1] = '1';
   str_msg.data = msg;
   encoder.publish(&str_msg);
   //Serial.println(m);
  nh.spinOnce();
  delay(1);
}
