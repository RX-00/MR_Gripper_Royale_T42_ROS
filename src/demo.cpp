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

// open or close the gripper
int close_and_open(RPM::SerialInterface *servosInterface){
  bool cont = true;
  char option;
  while(cont){
    std::cout << "Close & Open gripper \n"
              << "A: open gripper \n"
              << "B: close gripper \n"
              << "any other input exits this demo option"
              << std::endl;
    std::cin >> option;
    if (!isalpha(option)) exit(0);
    switch(option){
    case 'A':
      std::cout << "opening gripper..." << std::endl;
      servosInterface -> setTargetCP(L_SERVO, L_SERVO_OPEN);
      servosInterface -> setTargetCP(R_SERVO, R_SERVO_OPEN);
      Utils::sleep(500);
      break;
    case 'B':
      std::cout << "closing gripper..." << std::endl;
      servosInterface -> setTargetCP(L_SERVO, L_SERVO_CLOSE);
      servosInterface -> setTargetCP(R_SERVO, R_SERVO_CLOSE);
      Utils::sleep(500);
      break;
    default:
      cont = false;
      return 0;
      break;
    }
  }
  return 0;
}

// move one individual servo
int move_servo(RPM::SerialInterface *servosInterface, unsigned char channelNumber){
  bool cont = true;
  char option;
  int val = SRVO_MIN;
  while(cont){
    std::cout << "Close & Open gripper \n"
              << "A: decrease servo pos value \n"
              << "B: increase servo pos value \n"
              << "any other input exits this demo option"
              << std::endl;
    std::cin >> option;
    if (!isalpha(option)) exit(0);
    switch(option){
    case 'A':
      val -= 5;
      servosInterface -> setTargetCP(channelNumber, val);
      Utils::sleep(500);
      break;
    case 'B':
      val += 5;
      servosInterface -> setTargetCP(channelNumber, val);
      Utils::sleep(500);
      break;
    default:
      cont = false;
      return 0;
      break;
    }
  }
  return 0;
}

// moves both servos at the same time
int move_servos(RPM::SerialInterface *servosInterface){
  bool cont = true;
  char option;
  int val0 = SRVO_MIN;
  int val1 = SRVO_MAX;
  while(cont){
    std::cout << "Close & Open gripper \n"
              << "A: decrease servos pos value \n"
              << "B: increase servos pos value \n"
              << "any other input exits this demo option"
              << std::endl;
    std::cin >> option;
    if (!isalpha(option)) exit(0);
    switch(option){
    case 0:
      val0 -= 5;
      val1 += 5;
      servosInterface -> setTargetCP(L_SERVO, val0);
      servosInterface -> setTargetCP(R_SERVO, val1);
      Utils::sleep(500);
      break;
    case 1:
      val0 += 5;
      val1 -= 5;
      servosInterface -> setTargetCP(L_SERVO, val0);
      servosInterface -> setTargetCP(R_SERVO, val1);
      Utils::sleep(500);
      break;
    default:
      return 0;
      cont = false;
      break;
    }
  }
  return 0;
}

void servo_control(RPM::SerialInterface *servosInterface){
  bool cont = true;
  char option;
  std::cout << "Starting demo for MassRobotics Royale T42 Gripper..." << std::endl;
  while (cont){
    std::cout << "\n\nPlease choose an option" << std::endl;
    std::cout << "A: close & open gripper\n"
              << "B: move servo num 0\n"
              << "C: move servo num 1\n"
              << "D: move both servos\n"
              << "E: exit demo\n"
              << std::endl;
    std::cin >> option;
    if (!isalpha(option)) exit(0);
    switch(option){
    case 'A':
      close_and_open(servosInterface);
      break;
    case 'B':
      move_servo(servosInterface, L_SERVO);
      break;
    case 'C':
      move_servo(servosInterface, R_SERVO);
      break;
    case 'D':
      move_servos(servosInterface);
      break;
    case 'E':
      cont = false;
      break;
    default:
      std::cout << "Invalid input" << std::endl;
      break;
    }
  }
  std::cout << "Terminating demo..." << std::endl;
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
  sinusoid_signal(servosInterface, 0);
  sinusoid_signal(servosInterface, 1);

  servosInterface -> setTargetCP(L_SERVO, L_SERVO_OPEN);
  servosInterface -> setTargetCP(R_SERVO, R_SERVO_OPEN);
  Utils::sleep(3000);
  std::cout << "Closing gripper..." << std::endl;
  servosInterface -> setTargetCP(L_SERVO, L_SERVO_CLOSE);
  servosInterface -> setTargetCP(R_SERVO, R_SERVO_CLOSE);
  Utils::sleep(6000);
  servosInterface -> setTargetCP(L_SERVO, L_SERVO_OPEN);
  servosInterface -> setTargetCP(R_SERVO, R_SERVO_OPEN);
  Utils::sleep(1000);
}


// ROS callback function for listening to topic data
void sub_callback(const std_msgs::String::ConstPtr& msg){
  std::string cmd = msg->data.c_str();
  ROS_INFO("heard: [%s]", cmd);
}


int main(int argc, char** argv){
  // Serial servo interface
  unsigned char deviceNumber = 12; // NOTE: might need to change to 6
  unsigned char channelNumber = 1;
  std::string portName = "/dev/ttyACM0";
  RPM::SerialInterface *servosInterface = serialInterfaceInit(deviceNumber, channelNumber, portName);
  servosInterface -> SerialInterface::mMinChannelValue = SRVO_MIN;
  servosInterface -> SerialInterface::mMaxChannelValue = SRVO_MAX;
  init_srv_test(servosInterface);

  // starting up ros node
  ros::init(argc, argv, "listener");
  // main acccess point to communicate w/ ROS sys (this NodeHandle will init this node)
  ros::NodeHandle n;
  // subscriber call on the do_gripper topic, invokes call to ROS master node
  ros::Subscriber sub = n.subscribe("do_gripper", 10, sub_callback); // msg queue size: 10

  // ros::spin() will enter a loop calling callbacks, all callbacks will be called from within this main thread
  // ros::spin() will exit when node is shutdown by the master or Ctr-C is executed
  ros::spin();

  delete servosInterface;
  servosInterface = NULL;
  return 0;
}
