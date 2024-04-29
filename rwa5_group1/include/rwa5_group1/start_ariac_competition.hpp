/**
 * @file start_ariac_competition.hpp
 * @author  Ankur Mahesh Chavan (achavan1@umd.edu),Datta Lohith Gannavarapu (gdatta@umd.edu), Shail Kiritkumar Shah (sshah115@umd.edu)
 * Vinay Krishna Bukka (vinay06@umd.edu), Vishnu Mandala (vishnum@umd.edu)
 * @brief This program is used to start the ariac competiton once Ariac Environment starts running
 * @version 0.1
 * @date 2024-04-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */
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

/**
 * @brief The Constructor is used to create the subscribers and clients. Also initialising map
 * 
 */
class AriacCompetitionStart : public rclcpp::Node {
 public:

  AriacCompetitionStart(std::string node_name) : Node(node_name) {
    /**
     * @brief Subscriber to competition state to get the exact state continuously
     * 
     */
    competition_state_subscriber_ = this->create_subscription<ariac_msgs::msg::CompetitionState>(
        "/ariac/competition_state", 10,
        std::bind(&AriacCompetitionStart::competition_state_subscriber_cb, this, std::placeholders::_1));
    /**
     * @brief A Client Creation to create the client to start competition
     * 
     */
    start_competition_client_ = this->create_client<std_srvs::srv::Trigger>("/ariac/start_competition");
 
    competition_states[ariac_msgs::msg::CompetitionState::READY] = "READY";
    competition_states[ariac_msgs::msg::CompetitionState::IDLE] = "IDLE";
    competition_states[ariac_msgs::msg::CompetitionState::STARTED] = "STARTED";
    competition_states[ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE] = "ORDER_ANNOUNCEMENTS_DONE";
    competition_states[ariac_msgs::msg::CompetitionState::ENDED] = "ENDED";
  }
  
  
 private:
  rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr competition_state_subscriber_;  // Competition state subscriber declaration
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_competition_client_;// Start Competition client
  rclcpp::TimerBase::SharedPtr timer_;
  ariac_msgs::msg::CompetitionState current_state; // Global variable to update current state
  std::map<int,std::string> competition_states; // Map to store competition states for Logging
  /**
   * @brief A subscriber callback continously getting competition state and updating global state
   * 
   * @param state Contains the state
   */
  void competition_state_subscriber_cb(const ariac_msgs::msg::CompetitionState::SharedPtr state);
  /**
   * @brief Client function used to call when Competition state is ready
   * 
   */
  void start_competition();
  /**
   * @brief Callback function to get the status of client requent sent to server
   * 
   * @param future Contains the status of request sent 
   */
  void start_competition_cb(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
};