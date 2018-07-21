#pragma once
#include "imu.h"
#include "synchronizer.h"
#include "accel_solver.h"
#include <queue>
class calibrated_IMU_exception
{
    string desc;
public:
    calibrated_IMU_exception(string desc) : desc(desc) {}
    string get_desc(){ return desc; }
};

class calibrated_IMU : public metric_IMU {
    synchronizer sync;

    const size_t queue_maxsize = 50;
    std::queue<metric_sample> samples;
    std::queue<camera_frame> frames;

    metric_sample get_sample_blocking();
    camera_frame get_framestamp_blocking();

    metric_sample transform_by_calibration(metric_sample s);

    struct {
        arma::vec accel_cal {0, 0, 0, 1, 1, 1};
        arma::vec gyro_cal {0, 0, 0};
    } calibration;

    void transform_and_process();
protected:
    virtual void callback_metric_sample(metric_sample smpl);
    virtual void callback_metric_framestamp(camera_frame frm);
    virtual uint64_t get_time_ns() = 0;
    virtual void callback_synced_calibrated_sample(metric_sample smpl) = 0;
    virtual void callback_synced_framestamp(camera_frame frm) = 0;
public:
    arma::vec calibrate_acc(int samples = 1000);
    arma::vec calibrate_gyr(int samples = 2000);
    bool load_calibration(string filename);
    bool save_calibration(string filename);

    void settle();
};