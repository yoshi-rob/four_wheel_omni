#include "Cubic.controller.h"
#include "PID.h"
#include "cubic_arduino.h"
#include <Arduino.h>
#include <math.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

const int BAUD_RATE = 80000000;
const uint8_t motorNum[] = {0, 1, 2, 3};
const uint8_t incEncNum[] = {0, 1, 2, 3};
const uint16_t INC_CPR = 1024 * 4;
const double P = 0.8;
const double capableDuty = 0.5;
float velocity_target[] = {0, 0, 0, 0};
float Kp[] = {0.0, 0.0, 0.0, 0.0};
float Ki[] = {0.0, 0.0, 0.0, 0.0};
float Kd[] = {0.0, 0.0, 0.0, 0.0};
bool stopFlag = true;

void wheel_vel_callback(const std_msgs::Float32MultiArray &wheel_vel_msg) {
    velocity_target[0] = -wheel_vel_msg.data[0];
    velocity_target[1] = -wheel_vel_msg.data[1];
    velocity_target[2] = -wheel_vel_msg.data[2];
    velocity_target[3] = -wheel_vel_msg.data[3];
}

void safe_signal_cb(const std_msgs::Bool &safe_signal_msg) {
    if (safe_signal_msg.data == false) {
        stopFlag = true;
    } else {
        stopFlag = false;
    }
}

std_msgs::Float32MultiArray vel_msg;
ros::Publisher vel_pub("current_vel", &vel_msg);
ros::Subscriber<std_msgs::Float32MultiArray> wheel_vel_sub("wheel_vel",
                                                           wheel_vel_callback);
ros::Subscriber<std_msgs::Bool> safe_signal_sub("safe_signal", safe_signal_cb);

void setup() {
    Cubic::begin();
    nh.getHardware()->setBaud(BAUD_RATE);
    nh.initNode();

    while (!nh.connected()) {
        nh.spinOnce();
    }

    while (!nh.getParam("/Arduino/wheel/Kp", Kp, 4)) {
        nh.logerror("Failed to get Kp");
    }
    while (!nh.getParam("/Arduino/wheel/Ki", Ki, 4)) {
        nh.logerror("Failed to get Ki");
    }
    while (!nh.getParam("/Arduino/wheel/Kd", Kd, 4)) {
        nh.logerror("Failed to get Kd");
    }

    vel_msg.data = (float *)malloc(sizeof(float) * 4);
    vel_msg.data_length = 4;

    nh.subscribe(wheel_vel_sub);
    nh.subscribe(safe_signal_sub);
    nh.advertise(vel_pub);
}

void loop() {
    nh.spinOnce();
    using namespace Cubic_controller;
    static Velocity_PID velocityPID[] = {
        {motorNum[0], incEncNum[0], encoderType::inc, INC_CPR, capableDuty, P,
         Kp[0], Ki[0], Kd[0], velocity_target[0], true, false},
        {motorNum[1], incEncNum[1], encoderType::inc, INC_CPR, capableDuty, P,
         Kp[1], Ki[1], Kd[1], velocity_target[1], true, false},
        {motorNum[2], incEncNum[2], encoderType::inc, INC_CPR, capableDuty, P,
         Kp[2], Ki[2], Kd[2], velocity_target[2], true, false},
        {motorNum[3], incEncNum[3], encoderType::inc, INC_CPR, capableDuty, P,
         Kp[3], Ki[3], Kd[3], velocity_target[3], true, false}};

    if (stopFlag) {
        for (int i = 0; i < 4; i++) {
            DC_motor::put(motorNum[i], 0);
            vel_msg.data[i] = velocityPID[i].getCurrent();
        }
    } else {
        for (int i = 0; i < 4; i++) {
            velocityPID[i].setTarget(velocity_target[i]);
            velocityPID[i].compute();
            vel_msg.data[i] = velocityPID[i].getCurrent();
        }
    }
    vel_pub.publish(&vel_msg);
    Cubic::update();
}