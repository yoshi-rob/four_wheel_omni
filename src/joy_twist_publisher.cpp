#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

class TwistPublisher {
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher cmd_pub_;
    ros::Publisher safe_signal_pub_;
    ros::Subscriber joy_sub_;
    ros::Timer timer_;
    sensor_msgs::Joy last_joy_;
    int assign_x, assign_y, assign_z, safe_button;
    double max_x, max_y, max_z;

  public:
    TwistPublisher() : nh_(), pnh_("~") {
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        safe_signal_pub_ = nh_.advertise<std_msgs::Bool>("safe_signal", 1);
        joy_sub_ = nh_.subscribe("joy", 10, &TwistPublisher::joyCallback, this);
        timer_ = nh_.createTimer(ros::Duration(0.01),
                                 &TwistPublisher::timerCallback, this);
        pnh_.getParam("/joy/assign_x", assign_x);
        pnh_.getParam("/joy/assign_y", assign_y);
        pnh_.getParam("/joy/assign_z", assign_z);
        pnh_.getParam("/joy/safe_button", safe_button);
        pnh_.getParam("/joy/max_x", max_x);
        pnh_.getParam("/joy/max_y", max_y);
        pnh_.getParam("/joy/max_z", max_z);
    }

    void joyCallback(const sensor_msgs::Joy &joy_msg) { last_joy_ = joy_msg; }

    void timerCallback(const ros::TimerEvent &e) {
        geometry_msgs::Twist cmd_vel;
        if (0 <= assign_x && assign_x < last_joy_.axes.size()) {
            cmd_vel.linear.x = copysign(max_x, last_joy_.axes[assign_x]) *
                pow(last_joy_.axes[assign_x], 2);
        }
        if (0 <= assign_y && assign_y < last_joy_.axes.size()) {
            cmd_vel.linear.y = copysign(max_y, last_joy_.axes[assign_y]) *
                pow(last_joy_.axes[assign_y], 2);
        }
        if (0 <= assign_z && assign_z < last_joy_.axes.size()) {
            cmd_vel.angular.z = max_z * last_joy_.axes[assign_z];
        }

        cmd_pub_.publish(cmd_vel);

        /* 安全ボタンの処理 */
        std_msgs::Bool safe_signal;
        if (0 <= safe_button && safe_button < last_joy_.buttons.size()) {
            safe_signal.data = last_joy_.buttons[safe_button];

            if (last_joy_.buttons[safe_button]) {
                ROS_INFO("Moving");
            } else {
                ROS_INFO("Stopping");
            }
        }

        safe_signal_pub_.publish(safe_signal);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_twist_publisher");
    TwistPublisher twist_publisher;

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
