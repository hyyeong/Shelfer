#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <cstring>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class InitialPosePublisher : public rclcpp::Node
{
public:
  int status=0;
  using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;

  rclcpp::Publisher<Pose>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  explicit InitialPosePublisher(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("nav2_send_initial_pose",node_options)
  {
    const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    pose_publisher_ = 
      this->create_publisher<Pose>("initialpose",QOS_RKL10V);
    timer_=this->create_wall_timer(1s, std::bind(&InitialPosePublisher::sendGoal, this));
  }

  void sendGoal(void) 
  {
    Pose initpose;

    initpose.header.stamp = this->now();
    initpose.header.frame_id = "map";

    initpose.pose.pose.position.x=0.5;
    initpose.pose.pose.position.y=0.5;
    initpose.pose.pose.position.z=0.0;
    initpose.pose.pose.orientation.x = 0.0;
    initpose.pose.pose.orientation.y = 0.5;
    initpose.pose.pose.orientation.w = 0.0;
    initpose.pose.pose.orientation.z = 0.0;

    initpose.pose.covariance[6*0+0] = 0.5 * 0.5;
    initpose.pose.covariance[6*1+1] = 0.5 * 0.5;
    initpose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
    RCLCPP_INFO(get_logger(), "InitPose");

    
    pose_publisher_->publish(initpose);
    RCLCPP_INFO(get_logger(), "Init Pose Publish!");

  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InitialPosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
