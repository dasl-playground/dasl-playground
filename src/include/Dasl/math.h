//
// Created by ghjang on 6/12/20.
//

#ifndef DEV_WS_MATH_H
#define DEV_WS_MATH_H
#include <Eigen/Eigen>
#include <memory>
using Eigen::Matrix3Xd;

//Matrix3Xd mRoty(3,3);
//Matrix3Xd mRotz(3,3);

//float mThetap;
//float mThetat;
//
//Matrix3Xd mRoty, mRotz;
//mRoty << cos(mThetat),0,sin(mThetat),
//0, 1, 0,
//-sin(mThetat), 0, cos(mThetat);
//
//mRotz << cos(mThetap) -sin(mThetap), 0,
//sin(mThetap), cos(mThetap), 0;
//0, 0, 1;

namespace Dasl{
    inline Eigen::Matrix3d & roty(double angle,Eigen::Matrix3d & mat){
        mat << cos(angle),0,sin(angle),
                0, 1, 0,
                -sin(angle), 0, cos(angle);
        return mat;
    }
    inline Eigen::Matrix3d  roty(double angle){

        Eigen::Matrix3d mat;
        mat << cos(angle),0,sin(angle),
                0, 1, 0,
                -sin(angle), 0, cos(angle);
        return mat;
    }
    inline Eigen::Matrix3d & rotz(double angle,Eigen::Matrix3d & mat){
        mat << cos(angle) -sin(angle), 0,
                sin(angle), cos(angle), 0,
                0, 0, 1;
        return mat;
    }
    inline Eigen::Matrix3d  rotz(double angle){

        Eigen::Matrix3d  mat;
        mat << cos(angle) -sin(angle), 0,
                sin(angle), cos(angle), 0,
                0, 0, 1;
        return mat;
    }
}



#endif //DEV_WS_MATH_H
