#include "calibrated_imu.h"

arma::vec calibrated_IMU::calibrate_acc(int sample_count) {
    string dummy;
    arma::vec sample(3);
    arma::vec accumulated(3);
    vector<arma::vec> measurements;
    while(1) {
        cout << "Now position your accel, do not move and press ENTER ..." << flush;
        getline(cin, dummy);
        if(dummy == "b") break;

        while(!samples.empty()) samples.pop();

        for(int j = 0; j < sample_count; j++) {
            sample = util::to_arma(get_sample_blocking().acc_mss);
            
            accumulated += sample;
        }

        accumulated /= sample_count;

        measurements.push_back(accumulated);

        cout << "added \n" << accumulated << endl;
    }
    cout << "Computing ..." << endl;

    AccelSolver solver(measurements);

    double error = 0;
    arma::vec solution = solver.solve(&error);

    cout << "Fitness: " << error << endl;

    calibration.accel_cal = solution;

    return solution;
}

metric_sample calibrated_IMU::get_sample_blocking() {
    while(samples.empty()) poll_once();
    metric_sample ret = samples.front();
    samples.pop();
    return ret;
}

camera_frame calibrated_IMU::get_framestamp_blocking() {
    while(frames.empty()) poll_once();
    camera_frame ret = frames.front();
    frames.pop();
    return ret;
}

arma::vec calibrated_IMU::calibrate_gyr(int samples) {
    metric_sample sample;
    arma::vec accumulated(3);

    for(int j = 0; j < samples; j++) {
        sample = get_sample_blocking();
        accumulated += util::to_arma(sample.gyr_rads);
    }

    accumulated /= -samples;

    calibration.gyro_cal = accumulated;

    return accumulated;
}

bool calibrated_IMU::load_calibration(string filename) {
    std::ifstream file(filename);
    bool ret = calibration.accel_cal.load(file);
    file.close();
    return ret;
}

bool calibrated_IMU::save_calibration(string filename) {
    std::ofstream file(filename);
    bool ret = calibration.accel_cal.save(file);
    file.close();
    return ret;
}

metric_sample calibrated_IMU::transform_by_calibration(metric_sample s) {
    s.acc_mss = util::from_arma(util::to_arma(s.acc_mss) - calibration.accel_cal.subvec(0,2));
    s.acc_mss = util::from_arma(util::to_arma(s.acc_mss) / calibration.accel_cal.subvec(3,5));
    s.gyr_rads = util::from_arma(util::to_arma(s.gyr_rads) + calibration.gyro_cal);
    return s;
}

void calibrated_IMU::callback_metric_sample(metric_sample smpl) {
    smpl.time_us = sync.sync(get_time_ns(), smpl.time_us * 1000ULL);
    if(samples.size() < queue_maxsize) samples.push(smpl);
    transform_and_process();
}

void calibrated_IMU::callback_metric_framestamp(camera_frame frm) {
    frm.time_us = sync.sync(get_time_ns(), frm.time_us * 1000ULL);
    if(frames.size() < queue_maxsize) frames.push(frm);
    transform_and_process();
}

void calibrated_IMU::settle() {
    for(int i = 0; i < 3000; i++) poll_once();
}

void calibrated_IMU::transform_and_process() {
    while(!samples.empty()) {
        auto sample = transform_by_calibration(samples.front());
        callback_synced_calibrated_sample(sample);
        samples.pop();
    }
    while(!frames.empty()) {
        callback_synced_framestamp(frames.front());
        frames.pop();
    }
}
