//
// Created by ghjang on 6/11/20.
//

#ifndef DEV_WS_DRCLIDAR_H
#define DEV_WS_DRCLIDAR_H

#include <rclcpp/rclcpp.hpp>
#include <urg_cpp/Urg_driver.h>
#include "DaslPanMotionController.h"

namespace Dasl{


const  double DRCLidarZOffset = 0.0445;


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

        bool isOpen(){
            if (mLidar.is_open() && mMotion->isOpen()){
                return true;
            }
            return false;
        }
        bool open(std::string ip_address = "10.19.3.10"){

            if (mLidar.is_open() && mMotion->isOpen()){
                return true;
            }

            if(!mMotion->isOpen()){
                mMotion->disconnect();
                mMotion->connect();
            }
            if(!mLidar.is_open()) {
                mLidar.close();
                if (!mLidar.open(ip_address.c_str(), qrk::Urg_driver::Default_port, qrk::Urg_driver::Ethernet)) {
                    return false;
                }
                mLidar.stop_measurement();
                mLidar.start_measurement();
            }
            return true;
        }
        bool reset(){

            mLidar.reboot();
        }
        bool close(){
            bool ret = true;

            if(!mMotion->isOpen() || !mLidar.is_open()){
                ret = false;
            }
            mLidar.close();
            mMotion->disconnect();

            return ret;
        }
        bool findHome(){

            //TODO: chk did find home....
            if(mMotion->isOpen()){
                mMotion->findHome();
                return true;
            }

            return false;

        }
        bool scan(int scan_start_deg=-50, int scan_end_deg=50, float scanning_period=3, float goto_time=1,int finished_pos=0){
            printf("[Trace] Called DRCLidar::scan\n");
            if(mMotion->isOpen()){
                mMotion->scan(scan_start_deg,scan_end_deg,scanning_period,goto_time,finished_pos,0);
                return true;
            }
            return false;
        }
        float getPosition(){
            if(mMotion->isOpen()) {
                return  mMotion->getPosition();
            }
            return 0.0;
        }
        bool  getLidarDistance(std::vector<long>& data, long *time_stamp = NULL){

            if(mLidar.is_open()){

                return  mLidar.get_distance(data, time_stamp);
            }
        }
    };
}

#endif //DEV_WS_DRCLIDAR_H
