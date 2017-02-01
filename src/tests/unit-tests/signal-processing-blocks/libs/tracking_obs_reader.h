//
// Created by javier on 23/1/2017.
//

#ifndef GNSS_SIM_tracking_obs_reader_H
#define GNSS_SIM_tracking_obs_reader_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

class tracking_obs_reader {

public:
    ~tracking_obs_reader();
    bool read_binary_obs(double &signal_timestamp_s,
            double &acc_carrier_phase_cycles,
            double &doppler_l1_hz,
            double &prn_delay_chips,
            double &tow);
    bool restart();
    bool open_obs_file(std::string out_file);
    bool d_dump;

private:

    std::string d_dump_filename;
    std::ifstream d_dump_file;

};

#endif //GNSS_SIM_tracking_obs_reader_H
