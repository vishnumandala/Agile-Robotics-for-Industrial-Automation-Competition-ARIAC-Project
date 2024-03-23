#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "rwa3_group1/start_ariac_competition.hpp"
#include <ariac_msgs/msg/competition_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <map>
#include <string>



void AriacCompetitionStart::competition_state_subscriber_cb(const ariac_msgs::msg::CompetitionState::SharedPtr state){

    std::string current_competition_state;
    // current_competition_state = competition_states.find(state->competition_state);
    if(state->competition_state == ariac_msgs::msg::CompetitionState::READY){
        current_competition_state = "READY";
    }else     if(state->competition_state == ariac_msgs::msg::CompetitionState::IDLE){
        current_competition_state = "IDLE";
    }else     if(state->competition_state == ariac_msgs::msg::CompetitionState::STARTED){
        current_competition_state = "STARTED";
    }else     if(state->competition_state == ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE){
        current_competition_state = "ORDER_ANNOUNCEMENTS_DONE";
    }else     if(state->competition_state == ariac_msgs::msg::CompetitionState::ENDED){
        current_competition_state = "ENDED";
    }
    RCLCPP_INFO(this->get_logger(),"The current state of competition is %s",current_competition_state.c_str());
    if(state->competition_state == ariac_msgs::msg::CompetitionState::READY){
        // current_state = 1;
        // RCLCPP_INFO_STREAM(this->get_logger(),"Current State: ");
        start_competition();
    }
    
    current_state.competition_state = state->competition_state;
    //   auto my_state = state->competition_state;
    //   RCLCPP_INFO_STREAM(this->get_logger(),"Current State: " << my_state);
    
    

}

// void AriacCompetitionStart::start_competition(){
//     RCLCPP_INFO(this->get_logger(),"Waiting for competition to start");
//     if (current_state.competition_state ==  ariac_msgs::msg::CompetitionState::STARTED){
//         return;
//     }
//     // std::shared_ptr<rclcpp::Node> node =
//     //       rclcpp::Node::make_shared("trigger_client");
//     auto start_node1 = std::make_shared<AriacCompetitionStart>("trial_start");
//     while (current_state.competition_state != ariac_msgs::msg::CompetitionState::READY){
//         rclcpp::spin_some(start_node);
//         RCLCPP_INFO(this->get_logger(),"Its not ready yet");
//     }
//     RCLCPP_INFO(this->get_logger(),"Competition is ready. Starting..");
//     if (!start_competition_client_->wait_for_service(std::chrono::milliseconds((int)(2000.0)))){
//         RCLCPP_INFO(this->get_logger(),"Service not available");
//     }

//     auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
//     auto future = start_competition_client_->async_send_request(request);

//     // rclcpp::spin_until_future_complete(node,future);

//     if (rclcpp::spin_until_future_complete(start_node1, future) ==
//         rclcpp::FutureReturnCode::SUCCESS)
//     {
//         RCLCPP_INFO(this->get_logger(),"Started Competition");
//     } else {
//         RCLCPP_INFO(this->get_logger(),"Unable to start Competition");
//     }

// }

void AriacCompetitionStart::start_competition(){
    if (current_state.competition_state ==  ariac_msgs::msg::CompetitionState::STARTED){
        return;
    }
    std::shared_ptr<rclcpp::Node> node =
          rclcpp::Node::make_shared("trigger_client");
    RCLCPP_INFO(this->get_logger(),"Competition is ready. Starting..");
    if (!start_competition_client_->wait_for_service(std::chrono::milliseconds((int)(2000.0)))){
        RCLCPP_INFO(this->get_logger(),"Service not available");
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = start_competition_client_->async_send_request(request);

    // rclcpp::spin_until_future_complete(node,future);

    if (rclcpp::spin_until_future_complete(node, future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(),"Started Competition");
    } else {
        RCLCPP_INFO(this->get_logger(),"Unable to start Competition");
    }

}
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto start_node = std::make_shared<AriacCompetitionStart>("trial_start");
  rclcpp::spin(start_node);
//   start_node -> start_competition();  
  
  rclcpp::shutdown();
}