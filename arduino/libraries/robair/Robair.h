#ifndef ROBAIR_H
#define ROBAIR_H


// ROS
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <robairmain/MotorsCmd.h>
#include <robairmain/MotorsInfo.h>
#include <robairmain/EyesMat.h>

// Arduino libraries
#include <Adafruit_TiCoServo.h>
#include <Adafruit_NeoPixel.h>
#include <Eyes.h>
#include <md49.h>
#include <Papierlogik.h>

// Constantes Verin
#define CMD 0x00
#define GET_VERSION 0x29
#define SET_SPEED_2 0x32
#define SET_MODE 0x34


class Robair {
private:
	ros::NodeHandle & nh;

	std_msgs::String log_msg;
	ros::Publisher log_pub;

	// Verin test
	int ver_speed = 127;
	ros::Subscriber<std_msgs::Int8, Robair> sub_verin;


	// ========================  MOTORS  ========================
	const uint8_t PIN_RMD49 = 5;
	int cmd_speedL = 0;  //current_speed used by motors
	int cmd_speedR = 0;
	int cmd_msg_speedL = 0; //current command send by user
	int cmd_msg_speedR = 0;
	unsigned long last_cmdvel = 0;
	MD49 md49;

	ros::Subscriber<geometry_msgs::Twist, Robair> sub_cmdvel;

	robairmain::MotorsInfo motors_msg;
	ros::Publisher motors_pub;

	void cmdvelCb(const geometry_msgs::Twist& command_msg);
	void stop_motors(void);
	void speed_control(void);

	void powerMD49(bool on);

	// Verin Test
	MD49 md49_verin;


	// ========================  BATTERY  =======================
	std_msgs::Int32 battery_msg;
	ros::Publisher battery_pub;
	unsigned long last_timestamp_battery = 0;
	void check_battery(unsigned int delay_check);


	// =========================  EYES  =========================
	const uint8_t PIN_EYES = 4;
	Eyes eyes;
	std_msgs::UInt8 eyes_msg;
	ros::Publisher eyes_pub;
	ros::Subscriber<std_msgs::UInt8,Robair> sub_cmdeyes;
	ros::Subscriber<robairmain::EyesMat,Robair> sub_eyesmat;
	void setEyes(int id);
	void cmdEyesCb(const std_msgs::UInt8 &eyes_msg) ;
	void eyesMatCb(const robairmain::EyesMat &mat_msg) ;


	// =========================  HEAD  =========================
	const uint8_t PIN_HEAD = 7;
	Adafruit_TiCoServo servoHead;
	int cmd_msg_head = 0;
	int cmd_head = 0;

	int head_inc = 1;
	std_msgs::Int8 head_msg;
	ros::Publisher head_pub;

	ros::Subscriber<std_msgs::Int8, Robair> sub_cmdhead;

	void setHead(int degree);
	void cmdHeadCb(const std_msgs::Int8 &head_msg);


	// ========================  REBOOT  ========================
	ros::Subscriber<std_msgs::UInt8,Robair> sub_reboot;
	void rebootCb(const std_msgs::UInt8 &reboot_msg);

	// ========================  SERIE DEBUG  ========================
	
	void remote_control();

public:
	Robair(ros::NodeHandle &nh);

	void begin();
	void spinOnce();

	void log(String str);
};


#endif
