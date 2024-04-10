/**
 * @file start_ariac_competition.cpp
 * @author Ankur Mahesh Chavan (achavan1@umd.edu),Datta Lohith Gannavarapu (gdatta@umd.edu), Shail Kiritkumar Shah (sshah115@umd.edu)
 * Vinay Krishna Bukka (vinay06@umd.edu), Vishnu Mandala (vishnum@umd.edu)
 * @brief This program contains implementation details for subscribers and clients
 * @version 0.1
 * @date 2024-03-25
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <map>
#include <string>

#include "rwa3_group1/start_ariac_competition.hpp"
#include <ariac_msgs/msg/competition_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <ariac_msgs/srv/move_agv.hpp>

using namespace std::chrono_literals;

 /**
 * @brief A subscriber callback continously getting competition state and updating global state
 * 
 * @param state Contains the state
 */
void AriacCompetitionStart::competition_state_subscriber_cb(const ariac_msgs::msg::CompetitionState::SharedPtr state){
    if (current_state.competition_state != state->competition_state){
    auto current_competition_state = competition_states[state->competition_state];
    RCLCPP_INFO(this->get_logger(),"The current state of competition is %s",current_competition_state.c_str());
    }
    if(state->competition_state == ariac_msgs::msg::CompetitionState::READY){
        start_competition();
    }
    
    current_state.competition_state = state->competition_state; 
}
/**
 * @brief Client function used to call when Competition state is ready
 * 
 */
void AriacCompetitionStart::start_competition(){
    if (current_state.competition_state ==  ariac_msgs::msg::CompetitionState::STARTED){
        return;
    }
    // std::shared_ptr<rclcpp::Node> node =
    //       rclcpp::Node::make_shared("trigger_client");
    RCLCPP_INFO(this->get_logger(),"Competition is ready. Starting..");
    while (!start_competition_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }
    
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // auto future = start_competition_client_->async_send_request(request);
    auto future_result = start_competition_client_->async_send_request(request, std::bind(&AriacCompetitionStart::start_competition_cb, this, std::placeholders::_1));

}

/**
 * @brief Callback function to get the status of client requent sent to server
 * 
 * @param future Contains the status of request sent 
 */
void AriacCompetitionStart::start_competition_cb(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
    auto status = future.wait_for(5s);
    if (status == std::future_status::ready)
    {
        RCLCPP_INFO(this->get_logger(),"Started Competition");
    } else {
        RCLCPP_INFO(this->get_logger(),"Still Waiting For Service Response");
    }

}


/**
 * @brief The main function creates object node and spins
 * 
 * @param argc Number of arguments
 * @param argv List of arguments from CLI
 * @return int 
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto start_node = std::make_shared<AriacCompetitionStart>("trial_start");
  rclcpp::spin(start_node);
//   start_node -> start_competition();  
  
  rclcpp::shutdown();
}












// void AriacCompetitionStart::start_competition(){
//     RCLCPP_INFO(this->get_logger(),"Waiting for competition to start");
//     if (current_state.competition_state ==  ariac_msgs::msg::CompetitionState::STARTED){
//         return;
//     }

//     auto current_node = shared_from_this();
//     while (current_state.competition_state != ariac_msgs::msg::CompetitionState::READY){
//         rclcpp::spin_some(current_node);
//         RCLCPP_INFO(this->get_logger(),"Its not ready yet");
//     }
//     RCLCPP_INFO(this->get_logger(),"Competition is ready. Starting..");
//     if (!start_competition_client_->wait_for_service(std::chrono::seconds((int)(2.0)))){
//         RCLCPP_INFO(this->get_logger(),"Service not available");
//     }

//     auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//     auto future = start_competition_client_->async_send_request(request);


//     if (rclcpp::spin_until_future_complete(current_node, future) == rclcpp::FutureReturnCode::SUCCESS)
//     {
//         RCLCPP_INFO(this->get_logger(),"Started Competition");
//     } else {
//         RCLCPP_INFO(this->get_logger(),"Unable to start Competition");
//     }

// }


// C++ Conversion backup 

// Lock Tray Client Creation and Callback

// void AriacCompetitionStart::lock_tray(int agv_number){
//     auto lock_tray_service_name = "/ariac/agv"+ std::to_string(agv_number)+"_lock_tray";
//     auto lock_agv_tray_client = this->create_client<std_srvs::srv::Trigger>(lock_tray_service_name);
//     RCLCPP_INFO(this->get_logger(),"AGV Tray Locking is Starting..");
//     while (!lock_agv_tray_client->wait_for_service(std::chrono::seconds(1)))
//     {
//         if (!rclcpp::ok())
//         {
//             RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service lock_tray. Exiting.");
//             return;
//         }
//         RCLCPP_INFO(this->get_logger(), "Service lock_tray not available, waiting again...");
//     }
    
//     auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//     // auto future = start_competition_client_->async_send_request(request);
//     auto future_result = lock_agv_tray_client->async_send_request(request, std::bind(&AriacCompetitionStart::lock_tray_cb, this, std::placeholders::_1));

// }

// void AriacCompetitionStart::lock_tray_cb(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
//     auto status = future.wait_for(5s);
//     if (status == std::future_status::ready)
//     {
//         RCLCPP_INFO(this->get_logger(),"AGV's Tray Locked");
//     } else {
//         RCLCPP_INFO(this->get_logger(),"Still Waiting For AGV Tray Locking Service Response");
//     }

// }


// // Move Agv Client Creation and Callback

// void AriacCompetitionStart::move_agv(int agv_number){
//     auto move_agv_service_name = "/ariac/move_agv"+ std::to_string(agv_number);
//     auto move_agv_client = this->create_client<ariac_msgs::srv::MoveAGV>(move_agv_service_name);
//     RCLCPP_INFO(this->get_logger(),"AGV Tray Locking is Starting..");
//     while (!move_agv_client->wait_for_service(std::chrono::seconds(1)))
//     {
//         if (!rclcpp::ok())
//         {
//             RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service lock_tray. Exiting.");
//             return;
//         }
//         RCLCPP_INFO(this->get_logger(), "Service lock_tray not available, waiting again...");
//     }
    
//     auto request = std::make_shared<ariac_msgs::srv::MoveAGV::Request>();
//     // auto future = start_competition_client_->async_send_request(request);
//     auto future_result = move_agv_client->async_send_request(request, std::bind(&AriacCompetitionStart::move_agv_cb, this, std::placeholders::_1));

// }

// void AriacCompetitionStart::move_agv_cb(rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture future){
//     auto status = future.wait_for(5s);
//     if (status == std::future_status::ready)
//     {
//         RCLCPP_INFO(this->get_logger(),"The respective AGV moved Locked");
//     } else {
//         RCLCPP_INFO(this->get_logger(),"Still Waiting For AGV Move Service Response");
//     }
// }