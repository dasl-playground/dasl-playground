//
// Created by ghjang on 6/9/20.
//

#include "DRCVisionActionServer.h"
#include <thread>
#include <functional>


DRCVisionActionServer::DRCVisionActionServer(const rclcpp::NodeOptions &options) :
        Node("DRCVisionActionServer", options) {

    using namespace std::placeholders;
    mActionServer = rclcpp_action::create_server<DRCLidarAction>(
            get_node_base_interface(),
            get_node_clock_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            "DRCLidarAction",
            std::bind(&DRCVisionActionServer::handle_goal, this, _1, _2),
            std::bind(&DRCVisionActionServer::handle_cancel, this, _1),
            std::bind(&DRCVisionActionServer::handle_accepted, this, _1)
    );

    mPublisher = create_publisher<PointCloud2>("PointCloud2", 1);


}

rclcpp_action::GoalResponse DRCVisionActionServer::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        const std::shared_ptr<const DRCLidarAction::Goal>& goal) {

    RCLCPP_INFO(
            get_logger(),
            "Received goal request with command %s",
            goal->command.c_str());

    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
}

rclcpp_action::CancelResponse DRCVisionActionServer::handle_cancel(
        const std::shared_ptr<GoalHandleLidarAction>& goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

    return rclcpp_action::CancelResponse::ACCEPT;
}

void DRCVisionActionServer::execute(
        const std::shared_ptr<GoalHandleLidarAction> &goal_handle) {

    auto result = std::make_shared<DRCLidarAction ::Result>();

    auto &&command = goal_handle->get_goal()->command;
    auto &&feedback = std::make_shared<DRCLidarAction::Feedback>();
    auto &&status = feedback->status;
    auto &&param = goal_handle->get_goal()->parameters;

    RCLCPP_INFO(this->get_logger(),
                "Begin DRCVisionActionServer::execute(%s)",
                command.c_str());

    if(command == "open"){
        mLidar->open();
    }
    else if(command =="close"){
        mLidar->close();
    }
    else if(command == "find_home"){
        mLidar->findHome();
    }
    else if(command == "reset"){
        mLidar->reset();
    }
    else if(command == "scan"){
        RCLCPP_INFO(this->get_logger(),
                    "start scan");

        PointCloud2 message;
        mLidar->scan(-50,
                     50,
                     3,
                     1);
                     //0);


       // while(1){
            float posPan = mLidar->getPosition();
       // }
        //message = PointCloud2();
       // mPublisher->publish(message);
        RCLCPP_INFO(this->get_logger(),
                     "end scan");
    }
    goal_handle->publish_feedback(feedback);

    if (rclcpp::ok()) {
        result->result = "success";
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
    RCLCPP_INFO(this->get_logger(),
                "End DRCVisionActionServer::execute(%s)",
                command.c_str());

}

void DRCVisionActionServer::handle_accepted(
        const std::shared_ptr<GoalHandleLidarAction> &goal_handle) {
    mExecMutex.lock();
    RCLCPP_INFO(get_logger(),
                 "handle accepted");
    std::thread([&]() {
        execute(goal_handle);
    }).detach();
    mExecMutex.unlock();
}

DRCVisionActionServer::~DRCVisionActionServer() {


    mLidar->close();
    RCLCPP_INFO(get_logger(),
                "DRC Lidar Module Closed");
}


int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<DRCVisionActionServer>());

    rclcpp::shutdown();
}