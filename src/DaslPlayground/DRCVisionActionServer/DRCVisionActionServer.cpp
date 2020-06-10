//
// Created by ghjang on 6/9/20.
//

#include "DRCVisionActionServer.h"
#include <thread>
#include <functional>
#include <inttypes.h>


DRCVisionActionServer::DRCVisionActionServer(const rclcpp::NodeOptions &options) :
        Node("DRCVisionActionServer",options)
{

    using namespace std::placeholders;
    mActionServer = rclcpp_action::create_server<DRCLidarAction>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "DRCLidarAction",
            std::bind(&DRCVisionActionServer::handle_goal, this, _1,_2),
            std::bind(&DRCVisionActionServer::handle_cancel, this, _1),
            std::bind(&DRCVisionActionServer::handle_accepted, this, _1)
    );


}

rclcpp_action::GoalResponse DRCVisionActionServer::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                               std::shared_ptr<const DRCLidarAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with command %s", goal->command.c_str());



    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DRCVisionActionServer::handle_cancel(std::shared_ptr<GoalHandleLidarAction> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

    return rclcpp_action::CancelResponse::ACCEPT;
}
void DRCVisionActionServer::execute(const std::shared_ptr<GoalHandleLidarAction> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    //TODO:work...

    const auto command = goal_handle->get_goal();
    auto && feedback = std::make_shared<DRCLidarAction::Feedback>();
    auto && status = feedback->status;




    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");

}

void DRCVisionActionServer::handle_accepted(const std::shared_ptr<GoalHandleLidarAction> goal_handle) {

    using namespace std::placeholders;
    std::thread(
        std::bind(&DRCVisionActionServer::execute, this, _1), goal_handle
    ).detach();
}




int main(int argc, char ** argv){

    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<DRCVisionActionServer>());

    rclcpp::shutdown();


}