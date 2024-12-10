/*
Referance: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html

*/


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


using namespace std::chrono_literals;

class RobotChase : public rclcpp::Node {

public:
  RobotChase() : Node("barista_robot_tf2_frame_listener") {
    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "morty/base_link");
    source_frame_ = this->declare_parameter<std::string>("source_frame", "rick/base_link");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create  velocity publisher
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 1);

    // Call on_timer function every second
    timer_ = this->create_wall_timer(200ms, std::bind(&RobotChase::on_timer, this));
  }

private:

  void on_timer() {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = source_frame_.c_str();

    // RCLCPP_INFO(this->get_logger(),"fromFrameRel [%s] toFrameRel [%s]",
    //                             fromFrameRel.c_str(), toFrameRel.c_str() );


    float error_distance;
    float distance_offset = 0.356;
    float error_yaw;
    const float kp_yaw = 0.9;
    const float kp_distance = 1.5;

    geometry_msgs::msg::TransformStamped ts;

    // Look up for the transformation between target_frame and source_frame 
    // and send velocity commands  to reach target_frame
    try {
        ts = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
        // RCLCPP_INFO(this->get_logger(),"Transfrom X[%f] Transfrom Y[%f]",
        //                                ts.transform.translation.x, ts.transform.translation.y );
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
        return;
    }

    geometry_msgs::msg::Twist msg;

    error_yaw = atan2(ts.transform.translation.y, ts.transform.translation.x);
    msg.angular.z = kp_yaw * error_yaw;

    error_distance =sqrt(pow(ts.transform.translation.x, 2) + pow(ts.transform.translation.y, 2));
    msg.linear.x = kp_distance * (error_distance - distance_offset);

    pub_cmd_vel_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "pub_cmd_vel_: X[%f]    Z[%f]",
    // msg.linear.x, msg.angular.z );
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
  std::string source_frame_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChase>());
  rclcpp::shutdown();
  return 0;
}