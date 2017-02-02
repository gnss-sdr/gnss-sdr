//
// Created by javier on 1/2/2017.
//

#include "tracking_true_obs_reader.h"

bool tracking_true_obs_reader::read_binary_obs()
    {
        try {
            d_dump_file.read((char *) &signal_timestamp_s, sizeof(double));
            d_dump_file.read((char *) &acc_carrier_phase_cycles, sizeof(double));
            d_dump_file.read((char *) &doppler_l1_hz, sizeof(double));
            d_dump_file.read((char *) &prn_delay_chips, sizeof(double));
            d_dump_file.read((char *) &tow, sizeof(double));
        }
        catch (const std::ifstream::failure &e) {
            return false;
        }
        return true;
    }

bool tracking_true_obs_reader::restart() {
    if (d_dump_file.is_open())
    {
        d_dump_file.clear();
        d_dump_file.seekg(0, std::ios::beg);
        return true;
    }else{
        return false;
    }
}

long int tracking_true_obs_reader::num_epochs()
{
    std::ifstream::pos_type size;
    int number_of_vars_in_epoch=5;
    int epoch_size_bytes=sizeof(double)*number_of_vars_in_epoch;
    std::ifstream tmpfile( d_dump_filename.c_str(), std::ios::binary | std::ios::ate);
    if (tmpfile.is_open())
        {
            size = tmpfile.tellg();
            long int  nepoch=size / epoch_size_bytes;
            return nepoch;
        }else{
            return 0;
        }
}

bool tracking_true_obs_reader::open_obs_file(std::string out_file) {
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

tracking_true_obs_reader::~tracking_true_obs_reader() {
    if (d_dump_file.is_open() == true)
    {
        d_dump_file.close();
    }
}
