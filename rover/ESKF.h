#pragma once

#include <stdint.h>
#include <eigen3/Eigen/Dense>
#include "Quaternion.h"

// 15 positions: 3 position + 3 velocity + 3 acceleration + 3 orientation error + 3 angular velocity
#define STATE_SIZE 15

#define MEASSUREMENT_GYR_SIZE 3
#define MEASSUREMENT_ACC_SIZE 3

namespace IMU_EKF
{

template <typename precision>
class ESKF
{
public:
    ESKF();
    void init();
    void initWithAcc(const float ax, const float ay, const float az);
    void predict(precision dt);
    void correctGyr(const float gx, const float gy, const float gz);
    void correctAcc(const float ax, const float ay, const float az);
    void reset();

    Eigen::Matrix<precision, STATE_SIZE, 1> getState() const;
    void getAttitude(float &roll, float &pitch, float &yaw) const;
    Quaternion<precision> getAttitude() const;
    void getAcceleration(float &x, float &y, float &z) const;

private:
    Eigen::Matrix<precision, STATE_SIZE, 1> x_;
    Quaternion<precision> qref_;
    Eigen::Matrix<precision, STATE_SIZE, STATE_SIZE> P_;
    Eigen::Matrix<precision, STATE_SIZE, STATE_SIZE> Q_;
    Eigen::Matrix<precision, MEASSUREMENT_GYR_SIZE, MEASSUREMENT_GYR_SIZE> R_Gyr_;
    Eigen::Matrix<precision, MEASSUREMENT_ACC_SIZE, MEASSUREMENT_ACC_SIZE> R_Acc_;
};
} // namespace IMU_EKF

// ugly but necessary — to get templates working
#include "ESKF.cpp"
