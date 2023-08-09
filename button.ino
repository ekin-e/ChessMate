#include <ros.h>
#include <std_msgs/Bool.h>

#define Buton 8

ros::NodeHandle  nh;

std_msgs::Bool num_msg;
ros::Publisher topic("arduinoTopic", &num_msg);

void setup()
{
  pinMode(Buton, INPUT);
  nh.initNode();
  nh.advertise(topic);

}
void loop()
{
  if (digitalRead(Buton) == 1){
    num_msg.data = 1;
    topic.publish(&num_msg);
    nh.spinOnce();
    delay(1000);
  }
  else{
    num_msg.data = 0;
    topic.publish(&num_msg);
    nh.spinOnce();
    delay(1000);
  }
  
}
