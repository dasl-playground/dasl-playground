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

    Eigen::Vector3d ret;
    Eigen::Vector3d u;
    u<< 0.001, 0, Dasl::DRCLidarZOffset;
    ret = Dasl::roty(60) *u;

    mQuitScanThread = false;

    mPublisher = create_publisher<PointCloud>("PointCloud", 1);
}

rclcpp_action::GoalResponse DRCVisionActionServer::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        const std::shared_ptr<const DRCLidarAction::Goal> goal) {

    RCLCPP_INFO(
            get_logger(),
            "Received goal request with command %s",
            goal->command.c_str());

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DRCVisionActionServer::handle_cancel(
        const std::shared_ptr<GoalHandleLidarAction> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

    return rclcpp_action::CancelResponse::ACCEPT;
}
bool DRCVisionActionServer::readScanPosThread(double endPose) {

    if(!mLidar->open()){
        return false;
    }
   // if(!mLidar->findHome()){
   //     return false;
   // }

    mQuitScanThread = false;
    mCurScanPos = mLidar->getPosition();

    std::thread([&](){
        while(!mQuitScanThread){
            mCurScanPos = mLidar->getPosition();
            usleep(25000);
        }

    }).detach();
}
bool DRCVisionActionServer::onScan(){


    PointCloud message;
    double posScanEnd = 45;
    double threshold = 1;            //Practically earned threshold
    mLidar->scan(-45,
                 posScanEnd,
                 7,
                 1);



    std::vector<double> panAngles;
    std::vector<std::vector<long>> rawLidarData;
    using namespace std::chrono_literals;
    rclcpp::WallRate loop_rate(25ms);
    //readScanPosThread(posScanEnd);
    std::vector<long> rawData;
    while(1){
        rawData.clear();
        mLidar->getLidarDistance(rawData, NULL);
        rawLidarData.push_back(rawData);

        mCurScanPos = mLidar->getPosition();
        panAngles.push_back(mCurScanPos);

        if(fabs(mCurScanPos - posScanEnd) <= threshold ){
            mQuitScanThread = true;
            break;
        }


        // std::ostringstream ss;
        // ss << posPan;
        // status = ss.str();
        // goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    message.header.stamp = rclcpp::Clock().now();
    message.header.frame_id = "Dasl_DRCLidar_frame";
    geometry_msgs::msg::Point32 pt;

    for (int i=0; i < panAngles.size(); i++){
        double posPan = panAngles[i] * M_PI / 180.0;
        auto && rawLineData = rawLidarData[i];
        double sp = sin( posPan);
        double cp = cos( posPan);
        for (int j=0; j <rawLineData.size();j++){
            double posTilt = ( -135.0 + 0.25 * j) * M_PI /180.0;

            Eigen::Vector3d u( rawLineData[j]*0.001, 0, Dasl::DRCLidarZOffset);

            Eigen::Vector3d ret;
            ret = Dasl::roty(posPan) * Dasl::rotz(posTilt) *u;
            pt.x = ret[0];
            pt.y = ret[1];
            pt.z = ret[2];


            // pt.x =  cp * (rawLineData[j]*cos(posTilt)) + 0 + sp *  Dasl::DRCLidarZOffset;
            // pt.y =  0 + rawLineData[j]*sin(posTilt) + 0;
            // pt.z =   -sp * rawLineData[j]*cos(posTilt) + 0 + cp * Dasl::DRCLidarZOffset;
            message.points.push_back(pt);
        }

    }

    mPublisher->publish(message);
}
void DRCVisionActionServer::execute(
        const std::shared_ptr<GoalHandleLidarAction> goal_handle) {

    auto result = std::make_shared<DRCLidarAction ::Result>();

    auto &&command = goal_handle->get_goal()->command;
    auto &&feedback = std::make_shared<DRCLidarAction::Feedback>();
    auto &&status = feedback->status;
    auto &&param = goal_handle->get_goal()->parameters;


//    auto && matz = Dasl::rotz(10);

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

        if(!onScan()){
            result->result = "failed";
        }
        RCLCPP_INFO(this->get_logger(),
                     "end scan");
    }

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
        const std::shared_ptr<GoalHandleLidarAction> goal_handle) {
   mExecMutex.lock();
    RCLCPP_INFO(get_logger(),
                 "handle accepted");

    std::thread([&,goal_handle]() {
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