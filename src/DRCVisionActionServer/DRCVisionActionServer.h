// Copyright 2020 DASL@UNLV
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEV_WS_DRCVISIONACTIONSERVER_H
#define DEV_WS_DRCVISIONACTIONSERVER_H
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <dasl_interface/action/drc_lidar.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Dasl/Dasl>


class DRCVisionActionServer :public rclcpp::Node{

     Dasl::DRCLidar *mLidar = Dasl::DRCLidar::getInstance();

public:

    using DRCLidarAction = dasl_interface::action::DRCLidar;
    using GoalHandleLidarAction = rclcpp_action::ServerGoalHandle<DRCLidarAction>;

    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    rclcpp_action::Server<DRCLidarAction>::SharedPtr mActionServer;
    rclcpp::Publisher<PointCloud2>::SharedPtr mPublisher;


    explicit DRCVisionActionServer(
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            const std::shared_ptr<const DRCLidarAction::Goal>& goal);

    rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleLidarAction>& goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleLidarAction>& goal_handle);

    void execute(const std::shared_ptr<GoalHandleLidarAction>& goal_handle);


};


#endif //DEV_WS_DRCVISIONACTIONSERVER_H
