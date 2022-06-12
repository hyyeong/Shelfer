#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <iostream>
#include <fstream>
#include <chrono>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/time.hpp"

using namespace std::chrono_literals;


class GetPositionFromAMCL : public rclcpp::Node
{
public:

  explicit GetPositionFromAMCL(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("get_position_from_amcl", node_options)
  {
    

  }
  ~GetPositionFromAMCL(){

  }
void create_odom()
  {
    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
    

    const auto QOS_RKL10V =
      rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    odom_position_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose",
      QOS_RKL10V,
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) -> void
      {
        

        RCLCPP_INFO(
          this->get_logger(),
          "Subscribed at: sec %ld nanosec %ld",
          msg->header.stamp.sec,
          msg->header.stamp.nanosec);

        RCLCPP_INFO(this->get_logger(), "Subscribed argument x : %.2f", msg->pose.pose.position.x);
        save_to_csv(msg);
        RCLCPP_INFO(this->get_logger(), "Subscribed argument y : %.2f", msg->pose.pose.position.y);
        
        rclcpp::shutdown();
      }
    );
    return;
  }

private:
  
  void save_to_csv(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::fstream csv_file("waypoints.txt",std::ios_base::app); //끝부터 추가 
    csv_file << msg->pose.pose.position.x << " " << msg->pose.pose.position.y <<" ";
    csv_file <<msg->pose.pose.position.z <<" ";
    csv_file << msg->pose.pose.orientation.x <<" " <<msg->pose.pose.orientation.y;
    csv_file <<" " << msg->pose.pose.orientation.w <<" " <<msg->pose.pose.orientation.z<<"\n";
    RCLCPP_INFO(this->get_logger(),"SaveFinish");
    return;
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    odom_position_subscriber_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto get_odom = std::make_shared<GetPositionFromAMCL>();
  get_odom->create_odom();
  rclcpp::spin(get_odom);

  rclcpp::shutdown();

  return 0;
}