
 
#include <ros.h>
#include <std_msgs/String.h>
 
ros::NodeHandle nh;
 
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
 
char hello[13] = "hello world!";
 
void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  //Serial.begin(9600);
}
 
void loop()
{
  //Serial.println("Hello World!");
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}
