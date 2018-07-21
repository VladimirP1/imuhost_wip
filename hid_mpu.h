#pragma once
#include "packet.h"
#include "hid.h"
#include <fstream>
#include <queue>
#include <array>

class MPU {
    HID hid;
    uint8_t buf[34];
    union {
        params_t params;
        uint8_t buf[sizeof(params_t)];
    } board_params;

    void route_packet(uint8_t* ptr);
protected:
    virtual void callback_raw_imu(uint8_t* ptr) = 0;
    virtual void callback_raw_framestamp(uint8_t* ptr) = 0;
    virtual void callback_raw_other(uint8_t* ptr) = 0;


public:
    enum params{
        FRAME_INTERVAL = 0
    };

    void connect(uint16_t pid, uint16_t vid);
    uint32_t get_param(enum params);
    void     set_param(enum params, uint32_t value);
    void disconnect();
    void poll_once();
};

