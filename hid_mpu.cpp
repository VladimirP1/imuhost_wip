#include "hid_mpu.h"
#include <iostream>

void MPU::connect(uint16_t pid, uint16_t vid) {
    hid.open(pid, vid);
}

void MPU::disconnect() {
    hid.close();
}

void MPU::poll_once() {
    int a = hid.read(buf);
    route_packet(buf);
    route_packet(buf + PACKTSIZE);
}

void MPU::route_packet(uint8_t* ptr) {
    switch(ptr[0]) {
        case PACKET_IMU:
            callback_raw_imu(ptr);
            break;
        case PACKET_FRAMESTAMP:
            callback_raw_framestamp(ptr);
            break;
        default:
            callback_raw_other(ptr);
    }
}

uint32_t MPU::get_param(enum MPU::params param) {
    hid.get_feature(board_params.buf);
    switch(param) {
        case FRAME_INTERVAL:
            return board_params.params.interval_ms;
    }
}
void MPU::set_param(enum MPU::params param, uint32_t value) {
    switch(param) {
        case FRAME_INTERVAL:
            board_params.params.interval_ms = value;
            break;
    }
    hid.set_feature(board_params.buf);
}