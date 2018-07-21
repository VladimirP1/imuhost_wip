#pragma once
#include "hid_mpu.h"
#include "util.h"
#include <array>

#define SMPL_TO_MSS (9.8 * 4.0 / 32768.0)
#define SMPL_TO_RADS (2000.0 / 32768.0)

struct metric_sample {
    std::array<double, 3> acc_mss;
    std::array<double, 3> gyr_rads;
    uint64_t time_us;
};
struct camera_frame {
    uint64_t time_us;
};

class metric_IMU : public MPU {
    struct {
        uint32_t time_last = 0;
        uint64_t time_correction = 0;
    } timewrap_data;
    uint64_t fix_time(uint32_t in);

protected:
    virtual void callback_raw_imu(uint8_t* ptr);
    virtual void callback_raw_framestamp(uint8_t* ptr);
    virtual void callback_raw_other(uint8_t* ptr);

    virtual void callback_metric_sample(metric_sample smpl) = 0;
    virtual void callback_metric_framestamp(camera_frame frm) = 0;
};