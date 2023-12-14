#include <cmath>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

class FourWheelOmni {
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher wheel_pub_;
    ros::Subscriber twist_sub_;
    ros::Subscriber safe_signal_sub_;
    ros::Timer timer_;
    geometry_msgs::Twist last_twist_;

  public:
    FourWheelOmni() : nh_(), pnh_("~") {
        wheel_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("wheel_vel", 1);
        twist_sub_ =
            nh_.subscribe("cmd_vel", 1, &FourWheelOmni::twistCallback, this);
        timer_ = nh_.createTimer(ros::Duration(0.1),
                                 &FourWheelOmni::timerCallback, this);
    }

    void twistCallback(const geometry_msgs::Twist &twist_msg) {
        last_twist_ = twist_msg;
    }

    void timerCallback(const ros::TimerEvent &e) {
        std_msgs::Float32MultiArray wheel_vel;
        wheel_vel.data.resize(4);
        float vx = last_twist_.linear.x;
        float vy = last_twist_.linear.y;
        float vz = last_twist_.angular.z;

        // オムニホイールの角度
        float theta1 = M_PI / 4.0;
        float theta2 = 3.0 * M_PI / 4.0;
        float theta3 = 5.0 * M_PI / 4.0;
        float theta4 = 7.0 * M_PI / 4.0;

        // オムニホイールの速度計算
        wheel_vel.data[0] = vx * cos(theta1) + vy * sin(theta1) + vz; // 右前輪
        wheel_vel.data[1] = vx * cos(theta2) + vy * sin(theta2) + vz; // 左前輪
        wheel_vel.data[2] = vx * cos(theta3) + vy * sin(theta3) + vz; // 左後輪
        wheel_vel.data[3] = vx * cos(theta4) + vy * sin(theta4) + vz; // 右後輪

        wheel_pub_.publish(wheel_vel);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "four_wheel_omni");
    FourWheelOmni four_wheel_omni;
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
