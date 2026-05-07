#include "ESKF.h"

static IMU_EKF::ESKF<float>* filter = nullptr;

extern "C" {

    void kalman_init(float ax, float ay, float az){
        filter = new IMU_EKF::ESKF<float>();
        filter->initWithAcc(ax, ay, az);
    }

    void kalman_update(float ax, float ay, float az, float gx, float gy, float gz, float dt){
        filter->predict(dt);
        filter->correctAcc(ax, ay, az);
        filter->correctGyr(gx, gy, gz);
        filter->reset();
    }

    void kalman_get_attitude(float *roll, float *pitch, float *yaw){
        filter->getAttitude(*roll, *pitch, *yaw);
    }

}
