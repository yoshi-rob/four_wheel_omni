#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class Odometry {
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber wheel_sub_;
    ros::Publisher odom_pub_;
    tf2_ros::TransformBroadcaster odom_broadcaster_;
    geometry_msgs::TransformStamped tf_base_;
    nav_msgs::Odometry odom_;

    double radius_;
    double wheelBase_;
    ros::Time last_time = ros::Time::now();
    double x = 0.0, y = 0.0, theta = 0.0;
    double vx = 0.0, vy = 0.0, v_theta = 0.0;

  public:
    Odometry() : nh_(), pnh_("~") {
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
        wheel_sub_ =
            nh_.subscribe("current_vel", 1, &Odometry::currentVelCb, this);
        pnh_.getParam("/omniWheel/radius", radius_);
        pnh_.getParam("/omniWheel/wheelBase", wheelBase_);
    }

    void currentVelCb(const std_msgs::Float32MultiArray &vel_msg) {
        double v1 = -radius_ * vel_msg.data[0];
        double v2 = -radius_ * vel_msg.data[1];
        double v3 = -radius_ * vel_msg.data[2];
        double v4 = -radius_ * vel_msg.data[3];

        // オムニホイールの角度
        float theta1 = M_PI / 4.0;
        float theta2 = 3.0 * M_PI / 4.0;
        float theta3 = 5.0 * M_PI / 4.0;
        float theta4 = 7.0 * M_PI / 4.0;

        vx = (v1 * cos(theta1) + v2 * cos(theta2) + v3 * cos(theta3) +
              v4 * cos(theta4)) /
            4.0;
        vy = (v1 * sin(theta1) + v2 * sin(theta2) + v3 * sin(theta3) +
              v4 * sin(theta4)) /
            4.0;
        v_theta = (v1 + v2 + v3 + v4) / wheelBase_ / 4.0;
    }

    geometry_msgs::Pose setPose(double x, double y, double theta) {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = sin(theta / 2.0);
        pose.orientation.w = cos(theta / 2.0);
        return pose;
    }

    // オドメトリの計算
    void getOdom() {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;

        x += vx * dt;
        y += vy * dt;
        theta += v_theta * dt;

        tf_base_.header.stamp = current_time;
        tf_base_.header.frame_id = "odom";
        tf_base_.child_frame_id = "base_link";
        tf_base_.transform.translation.x = x;
        tf_base_.transform.translation.y = y;
        tf_base_.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        tf_base_.transform.rotation.x = q.x();
        tf_base_.transform.rotation.y = q.y();
        tf_base_.transform.rotation.z = q.z();
        tf_base_.transform.rotation.w = q.w();
        odom_broadcaster_.sendTransform(tf_base_);

        odom_.header.stamp = current_time;
        odom_.header.frame_id = "odom";
        odom_.child_frame_id = "base_link";
        odom_.pose.pose = setPose(x, y, theta);
        odom_.twist.twist.linear.x = vx;
        odom_.twist.twist.linear.y = vy;
        odom_.twist.twist.angular.z = v_theta;
        odom_pub_.publish(odom_);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry");
    Odometry odometry;
    ros::Rate loop_rate(100);
    while (ros::ok()) {
        odometry.getOdom();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}