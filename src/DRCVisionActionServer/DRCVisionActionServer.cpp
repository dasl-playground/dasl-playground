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

    mQuitScanThread = false;

    mPublisher = create_publisher<PointCloud>("PointCloud", 1);
    mPublisher2 = create_publisher<PointCloud2>("PointCloud2", 1);

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

    if (!mLidar->open()) {
        return false;
    }
    // if(!mLidar->findHome()){
    //     return false;
    // }

    mQuitScanThread = false;
    mCurScanPos = mLidar->getPosition();

    std::thread([&]() {
        while (!mQuitScanThread) {
            mCurScanPos = mLidar->getPosition();
            usleep(25000);
        }

    }).detach();
}

bool DRCVisionActionServer::onScan1() {

    mLidar->open();
    PointCloud message;

    double posScanEnd = 70;
    double threshold = 1;            //Practically earned threshold
    mLidar->scan(-15,
                 posScanEnd,
                 5,
                 1);


    std::vector<double> panAngles;
    std::vector<long> rawData;
    std::vector<std::vector<long>> rawLidarData;
    std::vector<unsigned short> rawIntensity;
    std::vector<std::vector<unsigned short>> rawIntensityData;

    using namespace std::chrono_literals;
    rclcpp::WallRate loop_rate(25ms);

    //readScanPosThread(posScanEnd);
    mLidar->startMeasurementDistanceIntensity();

    while (1) {
        rawData.clear();
        mLidar->getLidarDistanceIntensity(rawData, rawIntensity, NULL);
        rawLidarData.push_back(rawData);
        rawIntensityData.push_back(rawIntensity);

        mCurScanPos = mLidar->getPosition();
        panAngles.push_back(mCurScanPos);

        if (fabs(mCurScanPos - posScanEnd) <= threshold) {

            break;
        }

        // std::cout << "Distance" << rawData[0] << " Intensity" << rawIntensity[0] << std::endl;

        //printf("%f\n",mCurScanPos);
        // std::ostringstream ss;
        // ss << posPan;
        // status = ss.str();
        // goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }
    mLidar->stopMeasurement();
    mLidar->close();

    message.header.stamp = rclcpp::Clock().now();
    message.header.frame_id = "map";
    geometry_msgs::msg::Point32 pt;

    sensor_msgs::msg::ChannelFloat32 pclChannel;
    pclChannel.name = "intensities";

    auto filterIntensityMin = 100;
    auto filterIntensityMax = 4000;

    auto filterXMin = 0.0;
    auto filterXMax = 8.0;
    auto filterYMin = -4.0;
    auto filterYMax = 5.0;
    auto filterZMin = 0.0;
    auto filterZMax = 2.5;

    for (int i = 0; i < panAngles.size(); i++) {
        double posPan = panAngles[i] * M_PI / 180.0;
        auto &&rawLineData = rawLidarData[i];
        auto &&rawLineIntensity = rawIntensityData[i];

        for (int j = 0; j < rawLineData.size(); j++) {
            double posTilt = (-135.0 + 0.25 * j) * M_PI / 180.0;
            if (posTilt >= -35 && posTilt <= 35) {
                Eigen::Vector3d u(rawLineData[j] * 0.001, 0, Dasl::DRCLidarZOffset);
                Eigen::Vector3d ret;
                ret = Dasl::roty(posPan) * Dasl::rotz(posTilt) * u;
                pt.x = ret[0];
                pt.y = ret[1];
                pt.z = ret[2] + 1.32;
                if (pt.x <= filterXMax && pt.x >= filterXMin &&
                    pt.y <= filterYMax && pt.y >= filterYMin &&
                    pt.z <= filterZMax && pt.z >= filterZMin) {
                    if (rawLineIntensity[j] >= filterIntensityMin && rawLineIntensity[j] <= filterIntensityMax) {
                        pclChannel.values.push_back(rawLineIntensity[j]);
                        message.points.push_back(pt);
                    }
                }

            }
        }
    }
    printf("%d \n", message.points.size());
    message.channels.push_back(pclChannel);
    mPublisher->publish(message);
}

bool DRCVisionActionServer::onScan2() {

    mLidar->open();
    PointCloud2 message2;

    double posScanEnd = 45;
    double threshold = 1;            //Practically earned threshold
    mLidar->scan(-45,
                 posScanEnd,
                 7,
                 1);


    std::vector<double> panAngles;
    std::vector<long> rawData;
    std::vector<std::vector<long>> rawLidarData;
    std::vector<unsigned short> rawIntensity;
    std::vector<std::vector<unsigned short>> rawIntensityData;

    using namespace std::chrono_literals;
    rclcpp::WallRate loop_rate(25ms);

    //readScanPosThread(posScanEnd);
    mLidar->startMeasurementDistanceIntensity();

    while (1) {
        rawData.clear();
        mLidar->getLidarDistanceIntensity(rawData, rawIntensity, NULL);
        rawLidarData.push_back(rawData);
        rawIntensityData.push_back(rawIntensity);

        mCurScanPos = mLidar->getPosition();
        panAngles.push_back(mCurScanPos);

        if (fabs(mCurScanPos - posScanEnd) <= threshold) {

            break;
        }

        std::cout << "Distance" << rawData[0] << " Intensity" << rawIntensity[0] << std::endl;

        //printf("%f\n",mCurScanPos);
        // std::ostringstream ss;
        // ss << posPan;
        // status = ss.str();
        // goal_handle->publish_feedback(feedback);
        loop_rate.sleep();
    }
    mLidar->stopMeasurement();
    mLidar->close();

    message2.header.stamp = rclcpp::Clock().now();
    message2.header.frame_id = "map";
    message2.height = 1;

    message2.fields.resize(4); // 3 + channel size (1) = 4
    message2.fields[0].name = "x"; //point x
    message2.fields[1].name = "y"; //point y
    message2.fields[2].name = "z"; //point z
    message2.fields[3].name = "intensities"; //Channel name

    int offset = 0;
    for (size_t d = 0; d < message2.fields.size(); ++d, offset += 4) {
        message2.fields[d].offset = offset;
        message2.fields[d].count = 1;
        message2.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
    }

    message2.point_step = offset;
    message2.is_bigendian = false;
    message2.is_dense = false;

    std::vector<float> ptX;
    std::vector<float> ptY;
    std::vector<float> ptZ;
    std::vector<float> ptIntensity;

    auto filterIntensityMin = 100;
    auto filterIntensityMax = 4000;

    for (int i = 0; i < panAngles.size(); i++) {
        double posPan = panAngles[i] * M_PI / 180.0;
        auto &&rawLineData = rawLidarData[i];
        auto &&rawLineIntensity = rawIntensityData[i];

        for (int j = 0; j < rawLineData.size(); j++) {
            double posTilt = (-135.0 + 0.25 * j) * M_PI / 180.0;

            Eigen::Vector3d u(rawLineData[j] * 0.001, 0, Dasl::DRCLidarZOffset);
            Eigen::Vector3d ret;
            ret = Dasl::roty(posPan) * Dasl::rotz(posTilt) * u;

            if (rawLineIntensity[j] >= filterIntensityMin && rawLineIntensity[j] <= filterIntensityMax) {
                ptX.push_back(ret[0]);
                ptY.push_back(ret[1]);
                ptZ.push_back(ret[2]);
                ptIntensity.push_back(rawLineIntensity[j]);
            }
        }
    }

    message2.width = static_cast<uint32_t>(ptX.size());
    message2.row_step = message2.point_step * message2.width;
    message2.data.resize(ptX.size() * message2.point_step);

    for (size_t cp = 0; cp < ptX.size(); ++cp) {
        memcpy(&message2.data[cp * message2.point_step + message2.fields[0].offset], &ptX[cp],
               sizeof(float));
        memcpy(&message2.data[cp * message2.point_step + message2.fields[1].offset], &ptY[cp],
               sizeof(float));
        memcpy(&message2.data[cp * message2.point_step + message2.fields[2].offset], &ptZ[cp],
               sizeof(float));
        memcpy(&message2.data[cp * message2.point_step + message2.fields[3].offset], &ptIntensity[cp],
               sizeof(float));
    }
    mPublisher2->publish(message2);
}

void DRCVisionActionServer::execute(
        const std::shared_ptr<GoalHandleLidarAction> goal_handle) {

    auto result = std::make_shared<DRCLidarAction::Result>();

    auto &&command = goal_handle->get_goal()->command;
    auto &&feedback = std::make_shared<DRCLidarAction::Feedback>();
    auto &&status = feedback->status;
    auto &&param = goal_handle->get_goal()->parameters;

    RCLCPP_INFO(this->get_logger(),
                "Begin DRCVisionActionServer::execute(%s)",
                command.c_str());

    if (command == "open") {
        mLidar->open();
    } else if (command == "close") {
        mLidar->close();
    } else if (command == "find_home") {
        mLidar->findHome();
    } else if (command == "reset") {
        mLidar->reset();
    } else if (command == "scan1") {
        RCLCPP_INFO(this->get_logger(),
                    "start scan with PointCloud1");

        if (!onScan1()) {
            result->result = "failed";
        }
        RCLCPP_INFO(this->get_logger(),
                    "end scan with PointCloud1");
    } else if (command == "scan2") {
        RCLCPP_INFO(this->get_logger(),
                    "start scan with PointCloud2");

        if (!onScan2()) {
            result->result = "failed";
        }
        RCLCPP_INFO(this->get_logger(),
                    "end scan with PointCloud2");
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

    std::thread([&, goal_handle]() {
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