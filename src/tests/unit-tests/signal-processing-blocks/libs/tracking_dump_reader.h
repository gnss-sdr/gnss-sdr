//
// Created by javier on 23/1/2017.
//

#ifndef GNSS_SIM_tracking_dump_reader_H
#define GNSS_SIM_tracking_dump_reader_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

class tracking_dump_reader {

public:
    ~tracking_dump_reader();
    bool read_binary_obs();
    bool restart();
    long int num_epochs();
    bool open_obs_file(std::string out_file);

    //tracking dump variables
    // EPR
    float abs_E;
    float abs_P;
    float abs_L;
    // PROMPT I and Q (to analyze navigation symbols)
    float prompt_I;
    float prompt_Q;
    // PRN start sample stamp
    unsigned long int PRN_start_sample_count;

    // accumulated carrier phase
    double acc_carrier_phase_rad;

    // carrier and code frequency
    double carrier_doppler_hz;
    double code_freq_chips;

    // PLL commands
    double carr_error_hz;
    double carr_error_filt_hz;

    // DLL commands
    double code_error_chips;
    double code_error_filt_chips;

    // CN0 and carrier lock test
    double CN0_SNV_dB_Hz;
    double carrier_lock_test;

    // AUX vars (for debug purposes)
    double aux1;
    double aux2;

private:

    std::string d_dump_filename;
    std::ifstream d_dump_file;

};

#endif //GNSS_SIM_tracking_dump_reader_H
