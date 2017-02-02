//
// Created by javier on 23/1/2017.
//

#ifndef GNSS_SIM_tlm_dump_reader_H
#define GNSS_SIM_tlm_dump_reader_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

class tlm_dump_reader {

public:
    ~tlm_dump_reader();
    bool read_binary_obs();
    bool restart();
    long int num_epochs();
    bool open_obs_file(std::string out_file);

    //telemetry decoder dump variables
    double TOW_at_current_symbol;
    double Prn_timestamp_ms;
    double d_TOW_at_Preamble;

private:

    std::string d_dump_filename;
    std::ifstream d_dump_file;

};

#endif //GNSS_SIM_tlm_dump_reader_H
