//
// Created by javier on 1/2/2017.
//

#include "tracking_dump_reader.h"

bool tracking_dump_reader::read_binary_obs()
    {
        try {
            d_dump_file.read((char *) &abs_E, sizeof(float));
            d_dump_file.read((char *) &abs_P, sizeof(float));
            d_dump_file.read((char *) &abs_L, sizeof(float));
            d_dump_file.read((char *) &prompt_I, sizeof(float));
            d_dump_file.read((char *) &prompt_Q, sizeof(float));

            d_dump_file.read((char *) &PRN_start_sample_count, sizeof(unsigned long int));

            d_dump_file.read((char *) &acc_carrier_phase_rad, sizeof(double));
            d_dump_file.read((char *) &carrier_doppler_hz, sizeof(double));
            d_dump_file.read((char *) &code_freq_chips, sizeof(double));
            d_dump_file.read((char *) &carr_error_hz, sizeof(double));
            d_dump_file.read((char *) &carr_error_filt_hz, sizeof(double));
            d_dump_file.read((char *) &code_error_chips, sizeof(double));
            d_dump_file.read((char *) &code_error_filt_chips, sizeof(double));
            d_dump_file.read((char *) &CN0_SNV_dB_Hz, sizeof(double));
            d_dump_file.read((char *) &carrier_lock_test, sizeof(double));
            d_dump_file.read((char *) &aux1, sizeof(double));
            d_dump_file.read((char *) &aux2, sizeof(double));

        }
        catch (const std::ifstream::failure &e) {
            return false;
        }
        return true;
    }

bool tracking_dump_reader::restart() {
    if (d_dump_file.is_open())
    {
        d_dump_file.clear();
        d_dump_file.seekg(0, std::ios::beg);
        return true;
    }else{
        return false;
    }
}

long int tracking_dump_reader::num_epochs()
{
    std::ifstream::pos_type size;
    int number_of_double_vars=11;
    int number_of_float_vars=5;
    int epoch_size_bytes=sizeof(unsigned long int)+
            sizeof(double)*number_of_double_vars+
            sizeof(float)*number_of_float_vars;
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

bool tracking_dump_reader::open_obs_file(std::string out_file) {
    if (d_dump_file.is_open() == false)
    {
        try
        {
            d_dump_filename=out_file;
            d_dump_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
            d_dump_file.open(d_dump_filename.c_str(), std::ios::in | std::ios::binary);
            std::cout << "Tracking dump enabled, Log file: " << d_dump_filename.c_str()<< std::endl;
            return true;
        }
        catch (const std::ifstream::failure & e)
        {
            std::cout << "Problem opening Tracking dump Log file: " << d_dump_filename.c_str()<< std::endl;
            return false;
        }
    }else{
        return false;
    }
}

tracking_dump_reader::~tracking_dump_reader() {
    if (d_dump_file.is_open() == true)
    {
        d_dump_file.close();
    }
}
