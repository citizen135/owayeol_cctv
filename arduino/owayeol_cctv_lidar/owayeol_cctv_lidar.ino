#define USE_USBCON
#include <Arduino.h>
#include <Servo.h> 

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>

//ROS 노드 클래스 변수
ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range("range_data", &range_msg);

float dist; //actual distance measurements of LiDAR
int strength; //signal strength of LiDAR
float temprature;
int check; //save check value
int i;
int uart[9]; //save data measured by LiDAR
const int HEADER=0x59; //frame header of data package
float distance;


//서보 클래스 변수
Servo robot_servos[2];
char servo_pins[2] = {6, 5}; // PWM Pins on Arduino Uno
int mid_positions[2] = {90, 90};
int SERVO_CURRENT_POSITIONS[2];
char frameid[] = "/link3";
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
    //range_msg.range=distance/10;
    //range_msg.header.stamp = nh.now();
    //pub_range.publish(&range_msg);
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
  nh.advertise(pub_range);
  
  range_msg.radiation_type = sensor_msgs::Range::INFRARED;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.01;
  range_msg.min_range = -12;
  range_msg.max_range = -0.1;
  Serial1.begin(115200);
}

void loop() {
  nh.spinOnce();
  if(Serial1.available()) { //check if serial port has data input
    if(Serial1.read() == HEADER) { //assess data package frame header 0x59
    uart[0]=HEADER;
    //Serial.println(uart[0]);
    //Serial.println(Serial1.read());
    //if (Serial1.read() == -1) { //assess data package frame header 0x59
    //uart[1] = -1;
    //Serial.println("sdf");
    for (i = 2; i < 9; i++) { //save data in array
      uart[i] = Serial1.read();
    }
    check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
    //Serial.println(check);
    //Serial.println(uart[8]);
    //if (uart[8] == (check & 0xff)){ //verify the received data as per protocol
    dist = uart[2] + uart[3] * 256; //calculate distance value
    strength = uart[4] + uart[5] * 256; //calculate signal strength value
    temprature = uart[6] + uart[7] *256;//calculate chip temprature
    temprature = temprature/8 - 256;
    
    if((dist>0)&&(dist<1200))
      distance = dist;       // 거리값을 cm단위로 불러옵니다.
    range_msg.range=distance/-100;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
    delay(1);
    
    }
  }

}
