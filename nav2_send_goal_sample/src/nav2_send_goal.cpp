#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/time.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Nav2Client : public rclcpp::Node
{
public:
  int status=0;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

  std::ifstream way_file;

  bool is_response;
  Nav2Client(): Node("nav2_send_goal")
  {
    way_file.open("waypoints.txt");
  }

  bool sendGoal(void) {
    this->client_ptr_  = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    while (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }
    
    is_response=false;
    std::string word;
      //setting Goal
    auto goal_msg = NavigateToPose::Goal();
      
      goal_msg.pose.header.stamp = this->now();
      goal_msg.pose.header.frame_id = "base_footprint";

      // goal_msg.pose.pose.position.x = get_float(way_file);
      // goal_msg.pose.pose.position.y = get_float(way_file);
      // RCLCPP_INFO(get_logger(), "get xy");

      // goal_msg.pose.pose.orientation.x = get_float(way_file);
      // goal_msg.pose.pose.orientation.y = get_float(way_file);
      // goal_msg.pose.pose.orientation.w = get_float(way_file);
      // goal_msg.pose.pose.orientation.z = get_float(way_file);

      way_file >> goal_msg.pose.pose.position.x >> goal_msg.pose.pose.position.y;
      way_file >> goal_msg.pose.pose.position.z;
      way_file >> goal_msg.pose.pose.orientation.x >> goal_msg.pose.pose.orientation.y;
      way_file >> goal_msg.pose.pose.orientation.w >> goal_msg.pose.pose.orientation.z;
      if(way_file.eof())
        return false;
      RCLCPP_INFO(get_logger(), "gogo");

      //Setting FeedBack
      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      send_goal_options.feedback_callback = std::bind(&Nav2Client::feedbackCallback, this, _1, _2);
      send_goal_options.result_callback = std::bind(&Nav2Client::resultCallback, this, _1);
      //Goal async
      client_ptr_->async_send_goal(goal_msg, send_goal_options);

      return true;
  }

  float get_float(std::ifstream &file)
  {
    std::string str;
    file >> str;
    RCLCPP_INFO(get_logger(), "Transition R = " + str);
    float k = std::stof(str);
    RCLCPP_INFO(get_logger(), "Transition = %f", k);
    return k;
  }
  //feedback
  void feedbackCallback(GoalHandleNavigateToPose::SharedPtr,const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Distance remaininf = %f", feedback->distance_remaining);
  }
  //result
  void resultCallback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Success!!!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }
    is_response=true;
    rclcpp::shutdown();
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2Client>();
  bool flag;
  flag = node->sendGoal();
  rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
