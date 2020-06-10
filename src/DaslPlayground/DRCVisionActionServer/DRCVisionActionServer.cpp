//
// Created by ghjang on 6/9/20.
//

#include "DRCVisionActionServer.h"
#include <inttypes.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <dasl_action/action/dasl_lidar_action.hpp>
#include "rclcpp_action/server.hpp"


class DRCVisionActionServer :public rclcpp::Node{

public:
    using DRCLidarAction = dasl_action::action::DaslLidarAction;
    using GoalHandleLidarAction = rclcpp_action::ServerGoalHandle<DRCLidarAction>;
//    rclcpp_action::Server<DRCLidarAction>::SharedPtr action_server_;

    explicit DRCVisionActionServer(
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions()):
            Node("DRCVisionActionServer",options)
    {

        using namespace std::placeholders;
       rclcpp_action::create_server<DRCLidarAction>(
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




    rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const DRCLidarAction::Goal> goal){

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
            std::shared_ptr<GoalHandleLidarAction> goal_handle){

        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void handle_accepted(const std::shared_ptr<GoalHandleLidarAction> goal_handle){


    }

};
