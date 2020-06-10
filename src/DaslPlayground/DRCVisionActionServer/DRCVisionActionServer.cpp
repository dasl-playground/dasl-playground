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


template<typename ActionT>
typename rclcpp_action::Server<ActionT>::SharedPtr
create_server2(
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
        rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
        rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
        const std::string & name,
        typename rclcpp_action::Server<ActionT>::GoalCallback handle_goal,
        typename rclcpp_action::Server<ActionT>::CancelCallback handle_cancel,
        typename rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted,
        const rcl_action_server_options_t & options = rcl_action_server_get_default_options(),
        rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
    std::weak_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> weak_node =
            node_waitables_interface;
    std::weak_ptr<rclcpp::CallbackGroup> weak_group = group;
    bool group_is_null = (nullptr == group.get());

    auto deleter = [weak_node, weak_group, group_is_null](rclcpp_action::Server<ActionT> * ptr)
    {
        if (nullptr == ptr) {
            return;
        }
        auto shared_node = weak_node.lock();
        if (!shared_node) {
            return;
        }
        // API expects a shared pointer, give it one with a deleter that does nothing.
        std::shared_ptr<rclcpp_action::Server<ActionT>> fake_shared_ptr(ptr, [](rclcpp_action::Server<ActionT> *) {});

        if (group_is_null) {
            // Was added to default group
            shared_node->remove_waitable(fake_shared_ptr, nullptr);
        } else {
            // Was added to a specfic group
            auto shared_group = weak_group.lock();
            if (shared_group) {
                shared_node->remove_waitable(fake_shared_ptr, shared_group);
            }
        }
        delete ptr;
    };

    std::shared_ptr<rclcpp_action::Server<ActionT>> action_server(new rclcpp_action::Server<ActionT>(
            node_base_interface,
            node_clock_interface,
            node_logging_interface,
            name,
            options,
            handle_goal,
            handle_cancel,
            handle_accepted), deleter);

    node_waitables_interface->add_waitable(action_server, group);
    return action_server;
};


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
      /*  this->action_server = create_server2<DRCLidarAction>(
                this->get_node_base_interface(),
                this->get_node_clock_interface(),
                this->get_node_logging_interface(),
                this->get_node_waitables_interface(),
                "DRCLidarAction",
                std::bind(&DRCVisionActionServer::handle_goal, this, _1,_2),
                std::bind(&DRCVisionActionServer::handle_cancel, this, _1),
                std::bind(&DRCVisionActionServer::handle_accepted, this, _1)
        );*/


         rclcpp_action::Server<DRCLidarAction>(
                this->get_node_base_interface(),
                this->get_node_clock_interface(),
                this->get_node_logging_interface(),
                "DRCLidarAction",
                rcl_action_server_get_default_options(),
                std::bind(&DRCVisionActionServer::handle_goal, this, _1,_2),
                std::bind(&DRCVisionActionServer::handle_cancel, this, _1),
                std::bind(&DRCVisionActionServer::handle_accepted, this, _1));

    }



    rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<DRCLidarAction::Goal> goal){

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
            std::shared_ptr<GoalHandleLidarAction> goal_handle){

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
            std::shared_ptr<GoalHandleLidarAction> goal_handle){


    }

};
