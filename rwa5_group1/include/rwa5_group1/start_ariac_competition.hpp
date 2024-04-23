
#pragma once
#include <rclcpp/rclcpp.hpp>
#include<iostream>
#include<string>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <map>

#include <std_srvs/srv/trigger.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/srv/move_agv.hpp>

class AriacCompetitionStart : public rclcpp::Node {
 public:

  AriacCompetitionStart(std::string node_name) : Node(node_name) {
   
    competition_state_subscriber_ = this->create_subscription<ariac_msgs::msg::CompetitionState>(
        "/ariac/competition_state", 10,
        std::bind(&AriacCompetitionStart::competition_state_subscriber_cb, this, std::placeholders::_1));
    start_competition_client_ = this->create_client<std_srvs::srv::Trigger>("/ariac/start_competition");
 
    competition_states[ariac_msgs::msg::CompetitionState::READY] = "READY";
    competition_states[ariac_msgs::msg::CompetitionState::IDLE] = "IDLE";
    competition_states[ariac_msgs::msg::CompetitionState::STARTED] = "STARTED";
    competition_states[ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE] = "ORDER_ANNOUNCEMENTS_DONE";
    competition_states[ariac_msgs::msg::CompetitionState::ENDED] = "ENDED";
  }
  
  
 private:
  rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_subscriber_;  
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_competition_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  ariac_msgs::msg::CompetitionState current_state;
  std::map<int,std::string> competition_states;
  void competition_state_subscriber_cb(const ariac_msgs::msg::CompetitionState::SharedPtr state);
  
  void start_competition();
  void start_competition_cb(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);

  // C++ Conversion backup for Task 6,7
  // void lock_tray(int agv_number);
  // void lock_tray_cb(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
  // void move_agv(int agv_number);
  // void move_agv_cb(rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture future);

};