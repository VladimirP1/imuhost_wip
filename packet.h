#pragma once
#include <stdint.h>
struct params_t{
    uint8_t interval_ms;
    uint32_t  frame_cnt;
} __attribute__((packed, aligned(1)));

enum {
	PACKET_DUMMY = 0,
	PACKET_IMU,
	PACKET_FRAMESTAMP
};

#define PACKTSIZE 17
#define PACKTCOUNT 8