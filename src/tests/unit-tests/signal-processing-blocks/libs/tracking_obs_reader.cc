//
// Created by javier on 1/2/2017.
//

#include "tracking_obs_reader.h"

bool tracking_obs_reader::read_binary_obs(double &signal_timestamp_s,
        double &acc_carrier_phase_cycles,
        double &doppler_l1_hz,
        double &prn_delay_chips,
        double &tow)
    {
        try {
            d_dump_file.read((char *) &signal_timestamp_s, sizeof(double));
            d_dump_file.read((char *) &acc_carrier_phase_cycles, sizeof(double));
            d_dump_file.read((char *) &doppler_l1_hz, sizeof(double));
            d_dump_file.read((char *) &prn_delay_chips, sizeof(double));
            d_dump_file.read((char *) &tow, sizeof(double));
        }
        catch (const std::ifstream::failure &e) {
            std::cout << "Exception writing tracking obs dump file " << e.what() << std::endl;
        }
        return true;
    }

bool tracking_obs_reader::restart() {
    d_dump_file.clear();
    d_dump_file.seekg(0, std::ios::beg);
}

bool tracking_obs_reader::open_obs_file(std::string out_file) {
    if (d_dump_file.is_open() == false)
    {
        try
        {
            d_dump_filename=out_file;
            d_dump_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
            d_dump_file.open(d_dump_filename.c_str(), std::ios::in | std::ios::binary);
            std::cout << "Observables dump enabled, Log file: " << d_dump_filename.c_str()<< std::endl;
            return true;
        }
        catch (const std::ifstream::failure & e)
        {
            std::cout << "Problem opening Observables dump Log file: " << d_dump_filename.c_str()<< std::endl;
            return false;
        }
    }else{
        return false;
    }
}

tracking_obs_reader::~tracking_obs_reader() {
    if (d_dump_file.is_open() == true)
    {
        d_dump_file.close();
    }
}
