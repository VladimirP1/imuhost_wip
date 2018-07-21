#include "imu.h"

void metric_IMU::callback_raw_imu(uint8_t* ptr) {
    metric_sample smpl;
    ++ptr;
    for(int i = 0; i < 3; ++i) {
        smpl.acc_mss[i] = util::decode_short_at(2 * i + ptr) * SMPL_TO_MSS;
        smpl.gyr_rads[i] = util::decode_short_at(2 * i + ptr + 6) * SMPL_TO_RADS;
    }
    smpl.time_us = fix_time(util::decode_uword_at(ptr + 12)) / 120ULL;
    callback_metric_sample(smpl);
}

void metric_IMU::callback_raw_framestamp(uint8_t* ptr) {
    camera_frame frm;
    ++ptr;
    frm.time_us = fix_time(util::decode_uword_at(ptr + 12)) / 120ULL;
    callback_metric_framestamp(frm);
}

void metric_IMU::callback_raw_other(uint8_t* ptr) {
    
}

uint64_t metric_IMU::fix_time(uint32_t in) {
    if(in < timewrap_data.time_last) {
        timewrap_data.time_correction += 0x100000000ULL;
    }
    timewrap_data.time_last = in;
    return in + timewrap_data.time_correction;
}