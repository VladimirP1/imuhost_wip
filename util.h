#pragma once
#include <stdint.h>
#include <armadillo>
#include <array>

struct util {
    static int16_t decode_short_at(uint8_t* ptr) {
        return ptr[0] << 8 | ptr[1];
    }

    static uint32_t decode_uword_at(uint8_t* ptr) {
        return ptr[0] << 24 | ptr[1] << 16 | ptr[2] << 8 | ptr[3];
    }

    static arma::vec to_arma(std::array<double, 3> col) {
        arma::vec ret {col[0], col[1], col[2]};
        return ret;
    }

    static std::array<double, 3> from_arma(arma::vec col) {
        return std::array<double, 3>{col[0], col[1], col[2]};
    }
};