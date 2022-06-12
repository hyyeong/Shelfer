#include <memory>
#include <chrono>
#include <fstream>
#include <iostream>
#include <cstring>
#include <string>
#include <utility>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Nav2WaypointClient : public rclcpp::Node
{
public:
  int status=0;
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_ptr_;

  std::ifstream way_file;

  bool is_response;
  Nav2WaypointClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("nav2_send_waypoints",options)
  {
    RCLCPP_INFO(get_logger(),"Starting");
    this->client_ptr_  = rclcpp_action::create_client<FollowWaypoints>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "FollowWaypoints"
      );
    RCLCPP_INFO(get_logger(),"create Fin");
    way_file.open("waypoints.txt");
  }

  bool sendGoal(void) {

    RCLCPP_INFO(get_logger(), "Initial Func");
    if (!this->client_ptr_) {
    RCLCPP_WARN(this->get_logger(), "Action client not initialized");
  }
    while (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
      return false;
    }
    
    RCLCPP_INFO(get_logger(), "Create Client");
    is_response=false;
    std::string word;

      //setting Goal
    RCLCPP_INFO(get_logger(), "gog23");
    FollowWaypoints::Goal goal_msg;
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    geometry_msgs::msg::PoseStamped pose;
    RCLCPP_INFO(get_logger(), "gog2");
    while(!way_file.eof())
    {
      pose.header.stamp = this->now();
      pose.header.frame_id = "base_footprint";

        // goal_msg.pose.pose.position.x = get_float(way_file);
        // goal_msg.pose.pose.position.y = get_float(way_file);
        // RCLCPP_INFO(get_logger(), "get xy");

        // goal_msg.pose.pose.orientation.x = get_float(way_file);
        // goal_msg.pose.pose.orientation.y = get_float(way_file);
        // goal_msg.pose.pose.orientation.w = get_float(way_file);
        // goal_msg.pose.pose.orientation.z = get_float(way_file);
      RCLCPP_INFO(get_logger(), "gogo1");

      way_file >> pose.pose.position.x >> pose.pose.position.y;
      way_file >> pose.pose.position.z;
      way_file >> pose.pose.orientation.x >> pose.pose.orientation.y;
      way_file >> pose.pose.orientation.w >> pose.pose.orientation.z;
      poses.push_back(pose);
    }
    goal_msg.poses=poses;
    RCLCPP_INFO(get_logger(), "gogo");

      //Setting FeedBack
    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&Nav2WaypointClient::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&Nav2WaypointClient::resultCallback, this, _1);
      //Goal async
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
    if(way_file.eof()){
      way_file.close();
      return false;
    } 
      return true;
  }

  float get_float(std::ifstream &file)
  {
    std::string str;
    file >> str;
    RCLCPP_INFO(get_logger(), "Transition R = " + str);
    double k = std::stof(str);
    RCLCPP_INFO(get_logger(), "Transition = %f", k);
    return k;
  }
  //feedback
  void feedbackCallback(GoalHandleNavigateToPose::SharedPtr,const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Distance remaininf = %f", feedback);
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
  auto node = std::make_shared<Nav2WaypointClient>();
  node->sendGoal();
  rclcpp::spin(node);
  std::cout << "END" <<std::endl;
  rclcpp::shutdown();
  return 0;
}
