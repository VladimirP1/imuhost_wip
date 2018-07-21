#pragma once
#include <stdint.h>
#include <iostream>

class synchronizer {
    int64_t clk_offset = 0;
    int64_t error_lpf = 0;
    uint32_t corr_ptimer = 0;
    uint32_t cal_timer = 0;
    int64_t avg = 0;
public:
    uint64_t sync(uint64_t time_unix_ns, uint64_t time_board_ns) {
        int64_t offset_inst = time_unix_ns - time_board_ns;
        int64_t error_inst = offset_inst - clk_offset;
        if(cal_timer < 1000) {
            avg = avg + (offset_inst - avg)/(cal_timer + 1);
            if(cal_timer == 999) clk_offset = avg;
            cal_timer++;
        } else {
            error_lpf = error_lpf / 10000.0 * 9999.0 + error_inst / 10000.0;
            if(corr_ptimer % 100 == 0) {
                clk_offset += error_lpf / 100.0;
            }
            corr_ptimer++;

            if(abs(error_inst) > 4000000ULL) { // more than 4ms
                std::cout << "HIGH JITTER" " " << clk_offset << std::endl;
                if(abs(error_inst) > 400000000ULL) { // more than 400ms
                    std::cout << "VERY HIGH JITTER: reinit " << std::endl;
                    cal_timer = avg = 0;
                }
            }
        }
        return time_board_ns + clk_offset;
    }
};
