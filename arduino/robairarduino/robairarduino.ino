#include <ros.h>
#include <Robair.h>

ros::NodeHandle nh;
Robair robair(nh);

void setup()
{
	while(!Serial);
	nh.getHardware()->setBaud(57600);
	nh.initNode();

	robair.begin();


	pinMode(13,OUTPUT);
}

void loop()
{
	// ROBOT MOVEMENT
	nh.spinOnce();
	robair.spinOnce();
	delay(50);
}
