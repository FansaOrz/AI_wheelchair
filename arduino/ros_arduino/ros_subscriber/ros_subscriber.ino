#include<ros.h>
#include<rosserial_arduino/command.h>
#include<std_msgs/Float32MultiArray.h>
 
ros::NodeHandle nh;
 
void messageCb(const std_msgs::Float32MultiArray &msg){
Serial.println("aaaa");
}
 
ros::Subscriber<std_msgs::Float32MultiArray> sub("command_velocity", &messageCb);

void setup() {
    pinMode(9,OUTPUT);
    nh.initNode();
    nh.subscribe(sub);
}
 
void loop() {
  nh.spinOnce();
  delay(100);

}



