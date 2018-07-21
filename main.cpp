#include <iostream>
#include <stdio.h>
#include "calibrated_imu.h"
#include "quaternion.h"
#include <time.h> 
#include "init_rt.h"
using namespace std;

class myIMU : public calibrated_IMU {
protected:
    virtual uint64_t get_time_ns() {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec * 1000000000ULL + ts.tv_nsec;
    }
    virtual void callback_synced_calibrated_sample(metric_sample smpl) {
        cout << arma::norm(util::to_arma(smpl.acc_mss)) << endl;
    }
    virtual void callback_synced_framestamp(camera_frame frm) {

    }
};

int main()
{
    realtime_priority::init();
    try{
        myIMU imu;
        imu.connect(0x0483, 0x5710);
        imu.set_param(myIMU::params::FRAME_INTERVAL, 50);
        cout << imu.get_param(myIMU::params::FRAME_INTERVAL) << endl;
        
        imu.load_calibration("calib_old.bin");
        imu.settle();
        //cout << imu.calibrate_acc() << endl;
        while(1) {
            imu.poll_once();
            for(int i = 0; i < 100; i++) {
                
            }
        }

        imu.disconnect();
    } catch(HID_exception e) {
        cout << e.get_desc() << endl;
    }
    /*
    vector<arma::vec> meas = {
        {9.8 + 4,0,0},
        {-9.8 + 4,0,0},
        {0 + 4,9.8,0},
        {0 + 4,-9.8,0},
        {0 + 4,0,9.8},
        {0 + 4,0,-9.8}
    };
    AccelSolver solver(meas);
    double err = 0;
    cout << solver.solve(&err) << endl;*/
    return 0;
}
