//
// Created by ghjang on 6/11/20.
//

#ifndef DEV_WS_DRCLIDAR_H
#define DEV_WS_DRCLIDAR_H

#include <rclcpp/rclcpp.hpp>
#include <urg_cpp/Urg_driver.h>
#include "DaslPanMotionController.h"

namespace Dasl{
class DRCLidar {

        qrk::Urg_driver mLidar;
        DaslPanMotionController *mMotion;

        std::vector<long> mRawData;
        long mLidarTimeStamp;

        DRCLidar(): mMotion(DaslPanMotionController::getInstance()) {
            mLidarTimeStamp = 0;
        }
    public:
        static DRCLidar * getInstance(){
            static DRCLidar obj;

            return &obj;
        }
        ~DRCLidar(){
            //RCLCPP_INFO(get_logger(), "Release Devices");
            close();
        }


        bool open(std::string ip_address = "10.19.3.10"){

            //RCLCPP_INFO(get_logger(), "mLidar.is_open()");
            if (mLidar.is_open()){
                return true;
            }
            //RCLCPP_INFO(get_logger(), "mMotion->connect()");
            mMotion->connect();

            //RCLCPP_INFO(get_logger(), "mLidar.open()");
            if (!mLidar.open(ip_address.c_str(), qrk::Urg_driver::Default_port, qrk::Urg_driver::Ethernet)){
                return false;
            }
            mLidar.stop_measurement();
            mLidar.start_measurement();
            return true;
        }
        bool reset(){
            mLidar.reboot();
        }
        bool close(){
            mLidar.close();
            mMotion->disconnect();
            return true;
        }
        bool findHome(){
            mMotion->findHome();
        }
        bool scan(int scan_start_deg=-50, int scan_end_deg=50, float scanning_period=3, float goto_time=1,int finished_pos=0){
            mMotion->scan(scan_start_deg,scan_end_deg,scanning_period,goto_time,finished_pos,0);
        }
        float getPosition(){
            return mMotion->getPosition();
        }
    };
}

#endif //DEV_WS_DRCLIDAR_H
