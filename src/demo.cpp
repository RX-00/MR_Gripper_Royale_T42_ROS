/* User demo program for the MassRobotics
 * Royale T42 Gripper
 * -Roy Xing, any questions please contact: roy@massrobotics.org
 */

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "MR_Gripper_Royale_T42_ROS/RPMSerialInterface.h" // servos
#include "MR_Gripper_Royale_T42_ROS/Utils.h"              // servo utility class for sleep & time methods


// ====================================================================================
// ====================================================================================


// TODO: hook up the maestro controller of the gripper to a computer and test the
//       channel MIN and MAX of the MG99 Servos, might be different from the values
//       here that were meant for the 35 kg*m servos. Double check the default starting
//       value of each demo here and if the servos are negative mirrored to each other.


// ====================================================================================
// ====================================================================================



//NOTE: modified the RPM library to allow for greater servo range
#define SRVO_MAX   9000
#define SRVO_MIN   5000
#define SRVO_NEUTRAL   7000
#define L_SERVO_OPEN   5000
#define L_SERVO_CLOSE  8500
#define R_SERVO_OPEN   5000
#define R_SERVO_CLOSE  8500
#define L_SERVO    0
#define R_SERVO    1

class Listener{
public:
  bool gripper_state; // open (false) and close (true) states
  unsigned char deviceNumber;
  unsigned char channelNumber;
  std::string portName;
  RPM::SerialInterface *servosInterface;

  Listener();
  ~Listener();

  // callback function for ros
  void sub_callback(const std_msgs::String::ConstPtr& msg);
};

Listener::Listener(){
  deviceNumber = 12;
  channelNumber = 1;
  portName = "/dev/ttyACM0";
  servosInterface = serialInterfaceInit(deviceNumber, channelNumber, portName);
  servosInterface -> SerialInterface::mMinChannelValue = SRVO_MIN;
  servosInterface -> SerialInterface::mMaxChannelValue = SRVO_MAX;
}

Listener::~Listener(){
  delete servosInterface;
  servosInterface = NULL;
}

void Listener::sub_callback(const std_msgs::String::ConstPtr& msg){
  const char* cmd = (msg->data.c_str());
  ROS_INFO("heard: [%s]", cmd);
  std::string gripper_cmd(cmd);
  if (!gripper_cmd.compare("open")){
    gripper_state = false;
  }
  if (!gripper_cmd.compare("close")){
    gripper_state = true;
  }
}

// automatic test for one individual servo
void servo_test(RPM::SerialInterface *servosInterface, unsigned char channelNumber){
  std::cout << "Testing servo number: " << 11 << std::endl;
  for (int i = 0; i < 2; i++){
    std::cout << "min position" << std::endl;
    servosInterface -> setTargetCP(channelNumber, SRVO_MIN);
    Utils::sleep(1000);
    std::cout << "middle position" << std::endl;
    servosInterface -> setTargetCP(channelNumber, SRVO_NEUTRAL);
    Utils::sleep(1000);
    std::cout << "max position" << std::endl;
    servosInterface -> setTargetCP(channelNumber, SRVO_MAX);
    Utils::sleep(1000);
  }
  std::cout << "min position" << std::endl;
  servosInterface -> setTargetCP(channelNumber, SRVO_MIN);
}

// function to test device over serial w/ sinusoidal signals
void sinusoid_signal(RPM::SerialInterface *serialInterface, unsigned char channelNumber){
  // Generate a sinusoid signal to send to the PololuInterface
  std::cout << "Sending sinusoidal signal to device to test device..." << std::endl;
	const float pi = 3.141592653589793f;
	const unsigned int channelMinValue = SRVO_MIN;
	const unsigned int channelMaxValue = SRVO_MAX;
	const unsigned int channelValueRange = channelMaxValue - channelMinValue;
	const unsigned int signalPeriodInMs = 2000;
	unsigned int time0 = Utils::getTimeAsMilliseconds();
	unsigned int timeSinceStart = 0;
	while ( timeSinceStart < 5000 ){
    float k = sin( (pi*2)/signalPeriodInMs * timeSinceStart ) * (float)(channelValueRange/2);
    float channelValue = (float)channelMinValue + (float)channelValueRange/2 + k;
    printf("\rchannelValue=%d", (unsigned int)channelValue );
    serialInterface->setTargetCP( channelNumber, (unsigned short)channelValue );
    timeSinceStart = Utils::getTimeAsMilliseconds() - time0;
    Utils::sleep(5);
  }
  printf("\n");
}

// function to create serial interface for the maestro servo controller
RPM::SerialInterface * serialInterfaceInit(unsigned char deviceNumber, unsigned char channelNumber, std::string portName){
  // create the interface for the maestro
  std::cout << "Serial interface init..." << std::endl;
	unsigned int baudRate = 115200;
	printf("Creating serial interface '%s' at %d bauds\n", portName.c_str(), baudRate);
	std::string errorMessage;
	RPM::SerialInterface* serialInterface = RPM::SerialInterface::createSerialInterface( portName, baudRate, &errorMessage );
	if ( !serialInterface ){
    printf("Failed to create serial interface. %s\n", errorMessage.c_str());
    std::cout << "Terminating program..." << std::endl;
    std::exit(EXIT_FAILURE);
  }
  std::cout << "Serial interface initiated\n" << std::endl;
  return serialInterface;
}

void init_srv_test(RPM::SerialInterface *servosInterface){
  //sinusoid_signal(servosInterface, 0);
  //sinusoid_signal(servosInterface, 1);

  std::cout << "Opening gripper..." << std::endl;
  servosInterface -> setTargetCP(L_SERVO, L_SERVO_OPEN);
  servosInterface -> setTargetCP(R_SERVO, R_SERVO_OPEN);
  Utils::sleep(3000);
  /*
  std::cout << "Closing gripper..." << std::endl;
  servosInterface -> setTargetCP(L_SERVO, L_SERVO_CLOSE);
  servosInterface -> setTargetCP(R_SERVO, R_SERVO_CLOSE);
  Utils::sleep(6000);
  servosInterface -> setTargetCP(L_SERVO, L_SERVO_OPEN);
  servosInterface -> setTargetCP(R_SERVO, R_SERVO_OPEN);
  Utils::sleep(1000);
  */
}



int main(int argc, char** argv){
  // Serial servo interface
  Listener listener;
  /*
  unsigned char deviceNumber = 12; // NOTE: might need to change to 6
  unsigned char channelNumber = 1;
  std::string portName = "/dev/ttyACM0";
  RPM::SerialInterface *servosInterface = serialInterfaceInit(deviceNumber, channelNumber, portName);
  servosInterface -> SerialInterface::mMinChannelValue = SRVO_MIN;
  servosInterface -> SerialInterface::mMaxChannelValue = SRVO_MAX;
  */
  //init_srv_test(servosInterface);

  // starting up ros node
  std::cout << "Setting up ros node..." << std::endl;
  ros::init(argc, argv, "listener");
  // main acccess point to communicate w/ ROS sys (this NodeHandle will init this node)
  ros::NodeHandle n;
  
  // subscriber call on the do_gripper topic, invokes call to ROS master node
  ros::Subscriber sub = n.subscribe("do_gripper", 1000, &Listener::sub_callback, &listener); // msg queue size: 1000

  /*
  if (listener.gripper_state == false){
    std::cout << "Opening gripper..." << std::endl;
    servosInterface -> setTargetCP(L_SERVO, L_SERVO_OPEN);
    servosInterface -> setTargetCP(R_SERVO, R_SERVO_OPEN);
    Utils::sleep(500);
  }
  if (listener.gripper_state == true){
    std::cout << "Closing gripper..." << std::endl;
    servosInterface -> setTargetCP(L_SERVO, L_SERVO_CLOSE);
    servosInterface -> setTargetCP(R_SERVO, R_SERVO_CLOSE);
    Utils::sleep(500);
  }
  */
  
  // ros::spin() will enter a loop calling callbacks, all callbacks will be called from within this main thread
  ros::spin();

  return 0;
}
