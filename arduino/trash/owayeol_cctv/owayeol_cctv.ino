#include <Arduino.h>
#define USE_USBCON
//서보 라이브러리
#include <Servo.h> 
//로스 라이브러리
#include <ros.h>
//메시지 해더
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>

//ROS 노드 클래스 변수
ros::NodeHandle  nh;

//서보 클래스 변수
Servo robot_servos[2];
int servo_pins[2] = {6, 5}; // PWM Pins on Arduino Uno
int mid_positions[2] = {90, 90};
int SERVO_CURRENT_POSITIONS[2];

float TARGET_JOINT_POSITIONS[2] = {0,0};

void writeServos() {
  for (int j = 0; j < 2; j++) {
    int target_angle;
    if (j == 1) {
      // Due to difference in mounting directions
      target_angle = - TARGET_JOINT_POSITIONS[j]*(180/3.14) + mid_positions[j];
    } else {
      target_angle = TARGET_JOINT_POSITIONS[j]*(180/3.14) + mid_positions[j];
    }
    robot_servos[j].write(target_angle);
    SERVO_CURRENT_POSITIONS[j] = target_angle;
  }
  nh.spinOnce();
}

void servoControlSubscriberCallbackJointState(const sensor_msgs::JointState& msg) {
  TARGET_JOINT_POSITIONS[0] = msg.position[0];
  TARGET_JOINT_POSITIONS[1] = msg.position[1];
  // Call the method to write the joint positions to the servo motors
  writeServos();

}
ros::Subscriber<sensor_msgs::JointState> servo_control_subscriber_joint_state("joint_states", &servoControlSubscriberCallbackJointState);

void setup() {
  // Initial the servo motor connections and initialize them at home position
  for (unsigned int i = 0; i < 2; i++) {
    robot_servos[i].attach(servo_pins[i]);
    robot_servos[i].write(mid_positions[i]);
    SERVO_CURRENT_POSITIONS[i] = mid_positions[i];
  }

  // Set the communication BaudRate and start the node
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(servo_control_subscriber_joint_state);
}

void loop() {
  // Keep calling the spinOnce() method in this infinite loop to stay tightly coupled with the ROS Serial
  nh.spinOnce();
  delay(1);
}
