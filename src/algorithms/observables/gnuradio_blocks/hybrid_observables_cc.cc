/*!
 * \file hybrid_observables_cc.cc
 * \brief Implementation of the pseudorange computation block for Galileo E1
 * \author Javier Arribas 2017. jarribas(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "hybrid_observables_cc.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <gnuradio/io_signature.h>
#include <gnuradio/block_detail.h>
#include <gnuradio/buffer.h>
#include <glog/logging.h>
#include <matio.h>
#include "Galileo_E1.h"
#include "GPS_L1_CA.h"

using google::LogMessage;


hybrid_observables_cc_sptr hybrid_make_observables_cc(unsigned int nchannels_in, unsigned int nchannels_out, bool dump, std::string dump_filename)
{
    return hybrid_observables_cc_sptr(new hybrid_observables_cc(nchannels_in, nchannels_out, dump, dump_filename));
}


hybrid_observables_cc::hybrid_observables_cc(unsigned int nchannels_in, unsigned int nchannels_out, bool dump, std::string dump_filename) :
                        gr::block("hybrid_observables_cc",
                                gr::io_signature::make(nchannels_in, nchannels_in, sizeof(Gnss_Synchro)),
                                gr::io_signature::make(nchannels_out, nchannels_out, sizeof(Gnss_Synchro)))
{
    set_max_noutput_items(1);
    set_max_output_buffer(1);
    d_dump = dump;
    set_T_rx_s = false;
    d_nchannels = nchannels_out;
    d_dump_filename = dump_filename;
    T_rx_s = 0.0;
    T_rx_step_s = 0.001; // 1 ms
    max_delta = 0.05; // 50 ms
    valid_channels.resize(d_nchannels, false);
    d_num_valid_channels = 0;

    for(unsigned int i = 0; i < d_nchannels; i++)
    {
        d_gnss_synchro_history.push_back(std::deque<Gnss_Synchro>());
    }

    // ############# ENABLE DATA FILE LOG #################
    if (d_dump)
    {
        if (!d_dump_file.is_open())
        {
            try
            {
                d_dump_file.exceptions (std::ifstream::failbit | std::ifstream::badbit );
                d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                LOG(INFO) << "Observables dump enabled Log file: " << d_dump_filename.c_str();
            }
            catch (const std::ifstream::failure & e)
            {
                LOG(WARNING) << "Exception opening observables dump file " << e.what();
                d_dump = false;
            }
        }
    }
}


hybrid_observables_cc::~hybrid_observables_cc()
{
    if (d_dump_file.is_open())
        {
            try { d_dump_file.close(); }
            catch(const std::exception & ex)
            {
                    LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
            }
        }
    if(d_dump)
        {
            std::cout << "Writing observables .mat files ...";
            save_matfile();
            std::cout << " done." << std::endl;
        }
}


int hybrid_observables_cc::save_matfile()
{
    // READ DUMP FILE
    std::ifstream::pos_type size;
    int number_of_double_vars = 7;
    int epoch_size_bytes = sizeof(double) * number_of_double_vars * d_nchannels;
    std::ifstream dump_file;
    dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try { dump_file.open(d_dump_filename.c_str(), std::ios::binary | std::ios::ate); }
    catch(const std::ifstream::failure &e)
    {
        std::cerr << "Problem opening dump file:" <<  e.what() << std::endl;
        return 1;
    }
    // count number of epochs and rewind
    long int num_epoch = 0;
    if (dump_file.is_open())
        {
            size = dump_file.tellg();
            num_epoch = static_cast<long int>(size) / static_cast<long int>(epoch_size_bytes);
            dump_file.seekg(0, std::ios::beg);
        }
    else { return 1; }
    double ** RX_time = new double * [d_nchannels];
    double ** TOW_at_current_symbol_s = new double * [d_nchannels];
    double ** Carrier_Doppler_hz = new double * [d_nchannels];
    double ** Carrier_phase_cycles = new double * [d_nchannels];
    double ** Pseudorange_m = new double * [d_nchannels];
    double ** PRN = new double * [d_nchannels];
    double ** Flag_valid_pseudorange = new double * [d_nchannels];

    for(unsigned int i = 0; i < d_nchannels; i++)
        {
            RX_time[i] = new double [num_epoch];
            TOW_at_current_symbol_s[i] = new double[num_epoch];
            Carrier_Doppler_hz[i] = new double[num_epoch];
            Carrier_phase_cycles[i] = new double[num_epoch];
            Pseudorange_m[i] = new double[num_epoch];
            PRN[i] = new double[num_epoch];
            Flag_valid_pseudorange[i] = new double[num_epoch];
        }

    try
    {
            if (dump_file.is_open())
                {
                    for(long int i = 0; i < num_epoch; i++)
                        {
                            for(unsigned int chan = 0; chan < d_nchannels; chan++)
                                {
                                    dump_file.read(reinterpret_cast<char *>(&RX_time[chan][i]), sizeof(double));
                                    dump_file.read(reinterpret_cast<char *>(&TOW_at_current_symbol_s[chan][i]), sizeof(double));
                                    dump_file.read(reinterpret_cast<char *>(&Carrier_Doppler_hz[chan][i]), sizeof(double));
                                    dump_file.read(reinterpret_cast<char *>(&Carrier_phase_cycles[chan][i]), sizeof(double));
                                    dump_file.read(reinterpret_cast<char *>(&Pseudorange_m[chan][i]), sizeof(double));
                                    dump_file.read(reinterpret_cast<char *>(&PRN[chan][i]), sizeof(double));
                                    dump_file.read(reinterpret_cast<char *>(&Flag_valid_pseudorange[chan][i]), sizeof(double));
                                }
                        }
                }
            dump_file.close();
    }
    catch (const std::ifstream::failure &e)
    {
            std::cerr << "Problem reading dump file:" <<  e.what() << std::endl;
            for(unsigned int i = 0; i < d_nchannels; i++)
                {
                    delete[] RX_time[i];
                    delete[] TOW_at_current_symbol_s[i];
                    delete[] Carrier_Doppler_hz[i];
                    delete[] Carrier_phase_cycles[i];
                    delete[] Pseudorange_m[i];
                    delete[] PRN[i];
                    delete[] Flag_valid_pseudorange[i];
                }
            delete[] RX_time;
            delete[] TOW_at_current_symbol_s;
            delete[] Carrier_Doppler_hz;
            delete[] Carrier_phase_cycles;
            delete[] Pseudorange_m;
            delete[] PRN;
            delete[] Flag_valid_pseudorange;

            return 1;
    }

    double * RX_time_aux = new double [d_nchannels * num_epoch];
    double * TOW_at_current_symbol_s_aux = new double [d_nchannels * num_epoch];
    double * Carrier_Doppler_hz_aux = new double [d_nchannels * num_epoch];
    double * Carrier_phase_cycles_aux = new double [d_nchannels * num_epoch];
    double * Pseudorange_m_aux = new double [d_nchannels * num_epoch];
    double * PRN_aux = new double [d_nchannels * num_epoch];
    double * Flag_valid_pseudorange_aux = new double[d_nchannels * num_epoch];
    unsigned int k = 0;
    for(long int j = 0; j < num_epoch; j++ )
        {
            for(unsigned int i = 0; i < d_nchannels; i++ )
                {
                    RX_time_aux[k] = RX_time[i][j];
                    TOW_at_current_symbol_s_aux[k] = TOW_at_current_symbol_s[i][j];
                    Carrier_Doppler_hz_aux[k] = Carrier_Doppler_hz[i][j];
                    Carrier_phase_cycles_aux[k] = Carrier_phase_cycles[i][j];
                    Pseudorange_m_aux[k] = Pseudorange_m[i][j];
                    PRN_aux[k] = PRN[i][j];
                    Flag_valid_pseudorange_aux[k] = Flag_valid_pseudorange[i][j];
                    k++;
                }
        }

    // WRITE MAT FILE
    mat_t *matfp;
    matvar_t *matvar;
    std::string filename = d_dump_filename;
    filename.erase(filename.length() - 4, 4);
    filename.append(".mat");
    matfp = Mat_CreateVer(filename.c_str(), NULL, MAT_FT_MAT73);
    if(reinterpret_cast<long*>(matfp) != NULL)
        {
            size_t dims[2] = {static_cast<size_t>(d_nchannels), static_cast<size_t>(num_epoch)};
            matvar = Mat_VarCreate("RX_time", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, RX_time_aux, MAT_F_DONT_COPY_DATA);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("TOW_at_current_symbol_s", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, TOW_at_current_symbol_s_aux, MAT_F_DONT_COPY_DATA);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Carrier_Doppler_hz", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, Carrier_Doppler_hz_aux, MAT_F_DONT_COPY_DATA);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Carrier_phase_cycles", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, Carrier_phase_cycles_aux, MAT_F_DONT_COPY_DATA);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Pseudorange_m", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, Pseudorange_m_aux, MAT_F_DONT_COPY_DATA);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("PRN", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, PRN_aux, MAT_F_DONT_COPY_DATA);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);

            matvar = Mat_VarCreate("Flag_valid_pseudorange", MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, Flag_valid_pseudorange_aux, MAT_F_DONT_COPY_DATA);
            Mat_VarWrite(matfp, matvar, MAT_COMPRESSION_ZLIB); // or MAT_COMPRESSION_NONE
            Mat_VarFree(matvar);
        }
    Mat_Close(matfp);

    for(unsigned int i = 0; i < d_nchannels; i++)
        {
            delete[] RX_time[i];
            delete[] TOW_at_current_symbol_s[i];
            delete[] Carrier_Doppler_hz[i];
            delete[] Carrier_phase_cycles[i];
            delete[] Pseudorange_m[i];
            delete[] PRN[i];
            delete[] Flag_valid_pseudorange[i];

        }
    delete[] RX_time;
    delete[] TOW_at_current_symbol_s;
    delete[] Carrier_Doppler_hz;
    delete[] Carrier_phase_cycles;
    delete[] Pseudorange_m;
    delete[] PRN;
    delete[] Flag_valid_pseudorange;

    delete[] RX_time_aux;
    delete[] TOW_at_current_symbol_s_aux;
    delete[] Carrier_Doppler_hz_aux;
    delete[] Carrier_phase_cycles_aux;
    delete[] Pseudorange_m_aux;
    delete[] PRN_aux;
    delete[] Flag_valid_pseudorange_aux;
    return 0;
}

double hybrid_observables_cc::interpolate_data(const std::pair<Gnss_Synchro, Gnss_Synchro>& a, const double& ti, int parameter)
{
    // x(ti) = m * ti + c
    // m = [x(t2) - x(t1)] / [t2 - t1]
    // c = x(t1) - m * t1

    double m = 0.0;
    double c = 0.0;

    if(!a.first.Flag_valid_word or !a.second.Flag_valid_word) { return 0.0; }

    switch(parameter)
    {
    case 0:// Doppler
        m = (a.first.Carrier_Doppler_hz - a.second.Carrier_Doppler_hz) / (a.first.RX_time - a.second.RX_time);
        c = a.second.Carrier_Doppler_hz - m * a.second.RX_time;
        break;

    case 1:// Carrier phase
        m = (a.first.Carrier_phase_rads - a.second.Carrier_phase_rads) / (a.first.RX_time - a.second.RX_time);
        c = a.second.Carrier_phase_rads - m * a.second.RX_time;
        break;

    case 2:// TOW
        m = (a.first.TOW_at_current_symbol_s - a.second.TOW_at_current_symbol_s) / (a.first.RX_time - a.second.RX_time);
        c = a.second.TOW_at_current_symbol_s - m * a.second.RX_time;
        break;

    case 3:// Code phase samples
        m = (a.first.Code_phase_samples - a.second.Code_phase_samples) / (a.first.RX_time - a.second.RX_time);
        c = a.second.Code_phase_samples - m * a.second.RX_time;
        break;
    }
    return(m * ti + c);
}

double hybrid_observables_cc::compute_T_rx_s(const Gnss_Synchro& a)
{
    if(a.Flag_valid_word)
    {
        return((static_cast<double>(a.Tracking_sample_counter) + a.Code_phase_samples) / static_cast<double>(a.fs));
    }
    else { return 0.0; }
}

/*
bool Hybrid_pairCompare_gnss_synchro_T_rx(const std::pair<Gnss_Synchro, Gnss_Synchro>& a, const std::pair<Gnss_Synchro, Gnss_Synchro>& b)
{
    if(a.second.Flag_valid_word and !b.second.Flag_valid_word) { return true; }
    else if(!a.second.Flag_valid_word and b.second.Flag_valid_word) { return false; }
    else if(!a.second.Flag_valid_word and !b.second.Flag_valid_word) {return false; }
    else
    {
        return(Hybrid_Compute_T_rx_s(a.second) < Hybrid_Compute_T_rx_s(b.second));
    }
}


bool Hybrid_pairCompare_gnss_synchro_sample_counter(const std::pair<Gnss_Synchro, Gnss_Synchro>& a, const std::pair<Gnss_Synchro, Gnss_Synchro>& b)
{
    if(a.second.Flag_valid_word and !b.second.Flag_valid_word) { return true; }
    else if(!a.second.Flag_valid_word and b.second.Flag_valid_word) { return false; }
    else if(!a.second.Flag_valid_word and !b.second.Flag_valid_word) {return false; }
    else
    {
        return(a.second.Tracking_sample_counter < b.second.Tracking_sample_counter);
    }
}


bool Hybrid_valueCompare_gnss_synchro_sample_counter(const Gnss_Synchro& a, unsigned long int b)
{
    return(a.Tracking_sample_counter < b);
}


bool Hybrid_valueCompare_gnss_synchro_receiver_time(const Gnss_Synchro& a, double b)
{
    return((static_cast<double>(a.Tracking_sample_counter) + static_cast<double>(a.Code_phase_samples)) / static_cast<double>(a.fs) ) < (b);
}


bool Hybrid_pairCompare_gnss_synchro_TOW(const std::pair<Gnss_Synchro, Gnss_Synchro>& a, const std::pair<Gnss_Synchro, Gnss_Synchro>& b)
{
    if(a.first.Flag_valid_word and !b.first.Flag_valid_word) { return true; }
    else if(!a.first.Flag_valid_word and b.first.Flag_valid_word) { return false; }
    else if(!a.first.Flag_valid_word and !b.first.Flag_valid_word) {return false; }
    else
    {
        return(a.first.TOW_at_current_symbol_s < b.second.TOW_at_current_symbol_s);
    }
}


bool Hybrid_valueCompare_gnss_synchro_d_TOW(const Gnss_Synchro& a, double b)
{
    return(a.TOW_at_current_symbol_s < b);
}
*/

void hybrid_observables_cc::forecast(int noutput_items __attribute__((unused)),
        gr_vector_int &ninput_items_required)
{
//    bool available_items = false;
//    for(unsigned int i = 0; i < d_nchannels; i++)
//    {
//        ninput_items_required[i] = 0;
//        if(detail()->input(i)->items_available() > 0) { available_items = true; }
//    }
//    if(available_items) { ninput_items_required[d_nchannels] = 0; }
//    else { ninput_items_required[d_nchannels] = 1; }
    for(unsigned int i = 0; i < d_nchannels; i++)
    {
        ninput_items_required[i] = 0;
    }
    ninput_items_required[d_nchannels] = 1;
}

void hybrid_observables_cc::clean_history(std::deque<Gnss_Synchro>& data)
{
    while(data.size() > 0)
    {
        if((T_rx_s - data.front().RX_time) > max_delta) { data.pop_front(); }
        else { return; }
    }
}

unsigned int hybrid_observables_cc::find_closest(std::deque<Gnss_Synchro>& data)
{
    unsigned int result = 0;
    double delta_t = std::numeric_limits<double>::max();
    std::deque<Gnss_Synchro>::iterator it;
    unsigned int aux = 0;
    for(it = data.begin(); it != data.end(); it++)
    {
        double instant_delta = T_rx_s - it->RX_time;
        if((instant_delta > 0) and (instant_delta < delta_t))
        {
            delta_t = instant_delta;
            result = aux;
        }
        aux++;
    }
    return result;
}

double hybrid_observables_cc::find_min_RX_time()
{
    if(d_num_valid_channels == 0) { return 0.0; }

    std::vector<std::deque<Gnss_Synchro>>::iterator it = d_gnss_synchro_history.begin();
    double result = std::numeric_limits<double>::max();
    for(unsigned int i = 0; i < d_nchannels; i++)
    {
        if(valid_channels[i])
        {
            if(it->front().RX_time < result) { result = it->front().RX_time; }
        }
        it++;
    }
    return(floor(result * 1000.0) / 1000.0);
}

void hybrid_observables_cc::correct_TOW_and_compute_prange(std::vector<Gnss_Synchro>& data)
{
    double TOW_ref = std::numeric_limits<double>::lowest();
    std::vector<Gnss_Synchro>::iterator it;
    for(it = data.begin(); it != data.end(); it++)
    {
        if(it->RX_time > TOW_ref) { TOW_ref = it->RX_time; }
    }
    for(it = data.begin(); it != data.end(); it++)
    {
        double traveltime_s = TOW_ref - it->RX_time + GPS_STARTOFFSET_ms / 1000.0;
        it->RX_time = TOW_ref + GPS_STARTOFFSET_ms / 1000.0;
        it->Pseudorange_m = traveltime_s * SPEED_OF_LIGHT;
    }
}


int hybrid_observables_cc::general_work(int noutput_items __attribute__((unused)),
        gr_vector_int &ninput_items, gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{
    const Gnss_Synchro** in = reinterpret_cast<const Gnss_Synchro**>(&input_items[0]);
    Gnss_Synchro** out = reinterpret_cast<Gnss_Synchro**>(&output_items[0]);

    unsigned int i;
    int total_input_items = 0;
    for(i = 0; i < d_nchannels; i++) { total_input_items += ninput_items[i]; }
    consume(d_nchannels, 1);

    //////////////////////////////////////////////////////////////////////////
    if((total_input_items == 0) and (d_num_valid_channels == 0))
    {
        return 0;
    }
    if(set_T_rx_s) { T_rx_s += T_rx_step_s; }
    //////////////////////////////////////////////////////////////////////////


    std::vector<std::deque<Gnss_Synchro>>::iterator it;
    if (total_input_items > 0)
    {
        i = 0;
        for(it = d_gnss_synchro_history.begin(); it != d_gnss_synchro_history.end(); it++)
        {
            if(ninput_items[i] > 0)
            {
                for(int aux = 0; aux < ninput_items[i]; aux++)
                {
                    if(in[i][aux].Flag_valid_word)
                    {
                        it->push_back(in[i][aux]);
                        it->back().RX_time = compute_T_rx_s(in[i][aux]);
                    }
                }
                consume(i, ninput_items[i]);
            }
            i++;
        }
    }
    for(i = 0; i < d_nchannels; i++)
    {
        if(d_gnss_synchro_history.at(i).size() > 2) { valid_channels[i] = true; }
        else { valid_channels[i] = false; }
    }
    d_num_valid_channels = valid_channels.count();
    // Check if there is any valid channel after reading the new incoming Gnss_Synchro data
    if(d_num_valid_channels == 0)
    {
        set_T_rx_s = false;
        return 0;
    }

    if(!set_T_rx_s) //Find the lowest RX_time among the valid observables in the history
    {
        T_rx_s = find_min_RX_time();
        set_T_rx_s = true;
    }

    for(i = 0; i < d_nchannels; i++) //Discard observables with T_rx higher than the threshold
    {
        if(valid_channels[i])
        {
            clean_history(d_gnss_synchro_history.at(i));
            if(d_gnss_synchro_history.at(i).size() < 2) { valid_channels[i] = false; }
        }
    }

    // Check if there is any valid channel after computing the time distance between the Gnss_Synchro data and the receiver time
    d_num_valid_channels = valid_channels.count();
    if(d_num_valid_channels == 0)
    {
        set_T_rx_s = false;
        return 0;
    }

    std::vector<Gnss_Synchro> epoch_data;
    i = 0;
    for(it = d_gnss_synchro_history.begin(); it != d_gnss_synchro_history.end(); it++)
    {
        if(valid_channels[i])
        {
            unsigned int index_closest = find_closest(*it);
            unsigned int index1;
            unsigned int index2;
            if(index_closest == (it->size() - 1))
            {
                index1 = index_closest - 1;
                index2 = index_closest;
            }
            else
            {
                index1 = index_closest;
                index2 = index_closest + 1;
            }
            Gnss_Synchro interpolated_gnss_synchro = it->at(index1);

            interpolated_gnss_synchro.Carrier_Doppler_hz = interpolate_data(
                                std::pair<Gnss_Synchro, Gnss_Synchro>(it->at(index2), it->at(index1)), T_rx_s, 0);

            interpolated_gnss_synchro.Carrier_phase_rads = interpolate_data(
                                std::pair<Gnss_Synchro, Gnss_Synchro>(it->at(index2), it->at(index1)), T_rx_s, 1);

            interpolated_gnss_synchro.RX_time = interpolate_data(
                                std::pair<Gnss_Synchro, Gnss_Synchro>(it->at(index2), it->at(index1)), T_rx_s, 2);

            //interpolated_gnss_synchro.Code_phase_samples = interpolate_data(
            //                    std::pair<Gnss_Synchro, Gnss_Synchro>(it->at(index2), it->at(index1)), T_rx_s, 3);

            epoch_data.push_back(interpolated_gnss_synchro);
        }
        i++;
    }

    correct_TOW_and_compute_prange(epoch_data);
    std::vector<Gnss_Synchro>::iterator it2 = epoch_data.begin();
    for(i = 0; i < d_nchannels; i++)
    {
        if(valid_channels[i])
        {
            out[i][0] = (*it2);
            out[i][0].Flag_valid_pseudorange = true;
            it2++;
        }
        else
        {
            out[i][0] = Gnss_Synchro();
            out[i][0].Flag_valid_pseudorange = false;
        }
    }
    return 1;

    /* ANTONIO
    it = d_gnss_synchro_history.begin();
    double TOW_ref = std::numeric_limits<double>::max();
    for(i = 0; i < d_nchannels; i++)
    {
        if(!valid_channels[i]) { out[i][0] = Gnss_Synchro(); }
        else
        {
            out[i][0] = it->first;
            out[i][0].Flag_valid_pseudorange = true;
            out[i][0].Carrier_Doppler_hz = Hybrid_Interpolate_data(*it, T_rx_s, 0);
            out[i][0].Carrier_phase_rads = Hybrid_Interpolate_data(*it, T_rx_s, 1);
            out[i][0].RX_time = Hybrid_Interpolate_data(*it, T_rx_s, 2);
            out[i][0].Code_phase_samples = Hybrid_Interpolate_data(*it, T_rx_s, 3);
            //std::cout<<"T2: "<< it->first.RX_time<<". T1: "<< it->second.RX_time <<" T i: " << T_rx_s <<std::endl;
            //std::cout<<"Doppler origin: "<< it->first.Carrier_Doppler_hz<<","<< it->second.Carrier_Doppler_hz<<" Doppler interp: " << out[i][0].Carrier_Doppler_hz <<std::endl;
            if(out[i][0].RX_time < TOW_ref) { TOW_ref = out[i][0].RX_time; }
        }
        it++;
    }
    for(i = 0; i < d_nchannels; i++)
    {
        if(valid_channels[i])
        {
            double traveltime_ms = (out[i][0].RX_time - TOW_ref) * 1000.0 + GPS_STARTOFFSET_ms;
            out[i][0].Pseudorange_m = traveltime_ms * GPS_C_m_ms;
            out[i][0].RX_time = TOW_ref + GPS_STARTOFFSET_ms / 1000.0;
            //std::cout << "Sat " << out[i][0].PRN << ". Prang = " << out[i][0].Pseudorange_m << ". TOW = " << out[i][0].RX_time << std::endl;
        }
    }
    return 1;

    */

    /*******************************                 OLD ALGORITHM            ********************************/

//    const Gnss_Synchro** in = reinterpret_cast<const Gnss_Synchro**>(&input_items[0]); // Get the input buffer pointer
//    Gnss_Synchro** out = reinterpret_cast<Gnss_Synchro**>(&output_items[0]);           // Get the output buffer pointer
//    int n_outputs = 0;
//    int n_consume[d_nchannels];
//    double past_history_s = 100e-3;
//
//    Gnss_Synchro current_gnss_synchro[d_nchannels];
//    Gnss_Synchro aux = Gnss_Synchro();
//    for(unsigned int i = 0; i < d_nchannels; i++)
//    {
//        current_gnss_synchro[i] = aux;
//    }
//    /*
//     * 1. Read the GNSS SYNCHRO objects from available channels.
//     *  Multi-rate GNURADIO Block. Read how many input items are avaliable in each channel
//     *  Record all synchronization data into queues
//     */
//    for (unsigned int i = 0; i < d_nchannels; i++)
//    {
//        n_consume[i] = ninput_items[i]; // full throttle
//        for(int j = 0; j < n_consume[i]; j++)
//        {
//            d_gnss_synchro_history_queue[i].push_back(in[i][j]);
//        }
//    }
//
//    bool channel_history_ok;
//
//    do
//    {
//
//    try
//    {
//
//        channel_history_ok = true;
//        for(unsigned int i = 0; i < d_nchannels; i++)
//        {
//            if (d_gnss_synchro_history_queue.at(i).size() < history_deep && !d_gnss_synchro_history_queue.at(i).empty())
//            {
//                channel_history_ok = false;
//            }
//        }
//        if (channel_history_ok == true)
//        {
//            std::map<int,Gnss_Synchro>::const_iterator gnss_synchro_map_iter;
//            std::deque<Gnss_Synchro>::const_iterator gnss_synchro_deque_iter;
//
//            // 1. If the RX time is not set, set the Rx time
//            if (T_rx_s == 0)
//            {
//                // 0. Read a gnss_synchro snapshot from the queue and store it in a map
//                std::map<int,Gnss_Synchro> gnss_synchro_map;
//                for (unsigned int i = 0; i < d_nchannels; i++)
//                {
//                    if (!d_gnss_synchro_history_queue.at(i).empty())
//                    {
//                        gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(d_gnss_synchro_history_queue.at(i).front().Channel_ID,
//                                d_gnss_synchro_history_queue.at(i).front()));
//                    }
//                }
//                if(gnss_synchro_map.empty()) { break; } // Breaks the do-while loop
//
//                gnss_synchro_map_iter = std::min_element(gnss_synchro_map.cbegin(),
//                        gnss_synchro_map.cend(),
//                        Hybrid_pairCompare_gnss_synchro_sample_counter);
//                T_rx_s = static_cast<double>(gnss_synchro_map_iter->second.Tracking_sample_counter) / static_cast<double>(gnss_synchro_map_iter->second.fs);
//                T_rx_s = floor(T_rx_s * 1000.0) / 1000.0; // truncate to ms
//                T_rx_s += past_history_s; // increase T_rx to have a minimum past history to interpolate
//            }
//
//            // 2. Realign RX time in all valid channels
//            std::map<int,Gnss_Synchro> realigned_gnss_synchro_map; // container for the aligned set of observables for the selected T_rx
//            std::map<int,Gnss_Synchro> adjacent_gnss_synchro_map;  // container for the previous observable values to interpolate
//            // shift channels history to match the reference TOW
//            for (unsigned int i = 0; i < d_nchannels; i++)
//            {
//                if (!d_gnss_synchro_history_queue.at(i).empty())
//                {
//                    gnss_synchro_deque_iter = std::lower_bound(d_gnss_synchro_history_queue.at(i).cbegin(),
//                            d_gnss_synchro_history_queue.at(i).cend(),
//                            T_rx_s,
//                            Hybrid_valueCompare_gnss_synchro_receiver_time);
//                    if (gnss_synchro_deque_iter != d_gnss_synchro_history_queue.at(i).cend())
//                    {
//                        if (gnss_synchro_deque_iter->Flag_valid_word == true)
//                        {
//                            double T_rx_channel = static_cast<double>(gnss_synchro_deque_iter->Tracking_sample_counter) / static_cast<double>(gnss_synchro_deque_iter->fs);
//                            double delta_T_rx_s = T_rx_channel - T_rx_s;
//
//                            // check that T_rx difference is less than a threshold (the correlation interval)
//                            if (delta_T_rx_s * 1000.0 < static_cast<double>(gnss_synchro_deque_iter->correlation_length_ms))
//                            {
//                                // record the word structure in a map for pseudorange computation
//                                // save the previous observable
//                                int distance = std::distance(d_gnss_synchro_history_queue.at(i).cbegin(), gnss_synchro_deque_iter);
//                                if (distance > 0)
//                                {
//                                    if (d_gnss_synchro_history_queue.at(i).at(distance - 1).Flag_valid_word)
//                                    {
//                                        double T_rx_channel_prev = static_cast<double>(d_gnss_synchro_history_queue.at(i).at(distance - 1).Tracking_sample_counter) / static_cast<double>(gnss_synchro_deque_iter->fs);
//                                        double delta_T_rx_s_prev = T_rx_channel_prev - T_rx_s;
//                                        if (fabs(delta_T_rx_s_prev) < fabs(delta_T_rx_s))
//                                        {
//                                            realigned_gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(d_gnss_synchro_history_queue.at(i).at(distance - 1).Channel_ID,
//                                                    d_gnss_synchro_history_queue.at(i).at(distance - 1)));
//                                            adjacent_gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(gnss_synchro_deque_iter->Channel_ID, *gnss_synchro_deque_iter));
//                                        }
//                                        else
//                                        {
//                                            realigned_gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(gnss_synchro_deque_iter->Channel_ID, *gnss_synchro_deque_iter));
//                                            adjacent_gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(d_gnss_synchro_history_queue.at(i).at(distance - 1).Channel_ID,
//                                                    d_gnss_synchro_history_queue.at(i).at(distance - 1)));
//                                        }
//                                    }
//
//                                }
//                                else
//                                {
//                                    realigned_gnss_synchro_map.insert(std::pair<int, Gnss_Synchro>(gnss_synchro_deque_iter->Channel_ID, *gnss_synchro_deque_iter));
//                                }
//
//                            }
//                        }
//                    }
//                }
//            }
//
//            if(!realigned_gnss_synchro_map.empty())
//            {
//                /*
//                 *  2.1 Use CURRENT set of measurements and find the nearest satellite
//                 *  common RX time algorithm
//                 */
//                // what is the most recent symbol TOW in the current set? -> this will be the reference symbol
//                gnss_synchro_map_iter = std::max_element(realigned_gnss_synchro_map.cbegin(),
//                        realigned_gnss_synchro_map.cend(),
//                        Hybrid_pairCompare_gnss_synchro_d_TOW);
//                double ref_fs_hz = static_cast<double>(gnss_synchro_map_iter->second.fs);
//
//                // compute interpolated TOW value at T_rx_s
//                int ref_channel_key = gnss_synchro_map_iter->second.Channel_ID;
//                Gnss_Synchro adj_obs;
//                adj_obs = adjacent_gnss_synchro_map.at(ref_channel_key);
//                double ref_adj_T_rx_s = static_cast<double>(adj_obs.Tracking_sample_counter) / ref_fs_hz + adj_obs.Code_phase_samples / ref_fs_hz;
//
//                double d_TOW_reference = gnss_synchro_map_iter->second.TOW_at_current_symbol_s;
//                double d_ref_T_rx_s = static_cast<double>(gnss_synchro_map_iter->second.Tracking_sample_counter) / ref_fs_hz + gnss_synchro_map_iter->second.Code_phase_samples / ref_fs_hz;
//
//                double selected_T_rx_s = T_rx_s;
//                // two points linear interpolation using adjacent (adj) values: y=y1+(x-x1)*(y2-y1)/(x2-x1)
//                double ref_TOW_at_T_rx_s = adj_obs.TOW_at_current_symbol_s +
//                        (selected_T_rx_s - ref_adj_T_rx_s) * (d_TOW_reference - adj_obs.TOW_at_current_symbol_s) / (d_ref_T_rx_s - ref_adj_T_rx_s);
//
//                // Now compute RX time differences due to the PRN alignment in the correlators
//                double traveltime_ms;
//                double pseudorange_m;
//                double channel_T_rx_s;
//                double channel_fs_hz;
//                double channel_TOW_s;
//                for(gnss_synchro_map_iter = realigned_gnss_synchro_map.cbegin(); gnss_synchro_map_iter != realigned_gnss_synchro_map.cend(); gnss_synchro_map_iter++)
//                {
//                    channel_fs_hz = static_cast<double>(gnss_synchro_map_iter->second.fs);
//                    channel_TOW_s = gnss_synchro_map_iter->second.TOW_at_current_symbol_s;
//                    channel_T_rx_s = static_cast<double>(gnss_synchro_map_iter->second.Tracking_sample_counter) / channel_fs_hz + gnss_synchro_map_iter->second.Code_phase_samples / channel_fs_hz;
//                    // compute interpolated observation values
//                    // two points linear interpolation using adjacent (adj) values: y=y1+(x-x1)*(y2-y1)/(x2-x1)
//                    // TOW at the selected receiver time T_rx_s
//                    int element_key = gnss_synchro_map_iter->second.Channel_ID;
//                    adj_obs = adjacent_gnss_synchro_map.at(element_key);
//
//                    double adj_T_rx_s = static_cast<double>(adj_obs.Tracking_sample_counter) / channel_fs_hz + adj_obs.Code_phase_samples / channel_fs_hz;
//
//                    double channel_TOW_at_T_rx_s = adj_obs.TOW_at_current_symbol_s + (selected_T_rx_s - adj_T_rx_s) * (channel_TOW_s - adj_obs.TOW_at_current_symbol_s) / (channel_T_rx_s - adj_T_rx_s);
//
//                    // Doppler and Accumulated carrier phase
//                    double Carrier_phase_lin_rads = adj_obs.Carrier_phase_rads + (selected_T_rx_s - adj_T_rx_s) * (gnss_synchro_map_iter->second.Carrier_phase_rads - adj_obs.Carrier_phase_rads) / (channel_T_rx_s - adj_T_rx_s);
//                    double Carrier_Doppler_lin_hz = adj_obs.Carrier_Doppler_hz + (selected_T_rx_s - adj_T_rx_s) * (gnss_synchro_map_iter->second.Carrier_Doppler_hz - adj_obs.Carrier_Doppler_hz) / (channel_T_rx_s - adj_T_rx_s);
//
//                    // compute the pseudorange (no rx time offset correction)
//                    traveltime_ms = (ref_TOW_at_T_rx_s - channel_TOW_at_T_rx_s) * 1000.0 + GPS_STARTOFFSET_ms;
//                    // convert to meters
//                    pseudorange_m = traveltime_ms * GPS_C_m_ms; // [m]
//                    // update the pseudorange object
//                    current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID] = gnss_synchro_map_iter->second;
//                    current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID].Pseudorange_m = pseudorange_m;
//                    current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID].Flag_valid_pseudorange = true;
//                    // Save the estimated RX time (no RX clock offset correction yet!)
//                    current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID].RX_time = ref_TOW_at_T_rx_s + GPS_STARTOFFSET_ms / 1000.0;
//
//                    current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID].Carrier_phase_rads = Carrier_phase_lin_rads;
//                    current_gnss_synchro[gnss_synchro_map_iter->second.Channel_ID].Carrier_Doppler_hz = Carrier_Doppler_lin_hz;
//                }
//
//                if(d_dump == true)
//                {
//                    // MULTIPLEXED FILE RECORDING - Record results to file
//                    try
//                    {
//                        double tmp_double;
//                        for (unsigned int i = 0; i < d_nchannels; i++)
//                        {
//                            tmp_double = current_gnss_synchro[i].RX_time;
//                            d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
//                            tmp_double = current_gnss_synchro[i].TOW_at_current_symbol_s;
//                            d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
//                            tmp_double = current_gnss_synchro[i].Carrier_Doppler_hz;
//                            d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
//                            tmp_double = current_gnss_synchro[i].Carrier_phase_rads / GPS_TWO_PI;
//                            d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
//                            tmp_double = current_gnss_synchro[i].Pseudorange_m;
//                            d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
//                            tmp_double = current_gnss_synchro[i].PRN;
//                            d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
//                            tmp_double = static_cast<double>(current_gnss_synchro[i].Flag_valid_pseudorange);
//                            d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
//                        }
//                    }
//                    catch (const std::ifstream::failure& e)
//                    {
//                        LOG(WARNING) << "Exception writing observables dump file " << e.what();
//                        d_dump = false;
//                    }
//                }
//
//                for (unsigned int i = 0; i < d_nchannels; i++)
//                {
//                    out[i][n_outputs] = current_gnss_synchro[i];
//                }
//
//                n_outputs++;
//            }
//
//            // Move RX time
//            T_rx_s += T_rx_step_s;
//            // pop old elements from queue
//            for (unsigned int i = 0; i < d_nchannels; i++)
//            {
//                if (!d_gnss_synchro_history_queue.at(i).empty())
//                {
//                    while (static_cast<double>(d_gnss_synchro_history_queue.at(i).front().Tracking_sample_counter) / static_cast<double>(d_gnss_synchro_history_queue.at(i).front().fs) < (T_rx_s - past_history_s))
//                    {
//                        d_gnss_synchro_history_queue.at(i).pop_front();
//                    }
//                }
//            }
//        }
//
//    }// End of try{...}
//    catch(const std::out_of_range& e)
//    {
//        LOG(WARNING) << "Out of range exception thrown by Hybrid Observables block. Exception message: " << e.what();
//        std::cout << "Out of range exception thrown by Hybrid Observables block. Exception message: " << e.what() << std::endl;
//        return gr::block::WORK_DONE;
//    }
//    catch(const std::exception& e)
//    {
//        LOG(WARNING) << "Exception thrown by Hybrid Observables block. Exception message: " << e.what();
//        std::cout << "Exception thrown by Hybrid Observables block. Exception message: " << e.what() << std::endl;
//        return gr::block::WORK_DONE;
//    }
//
//    }while(channel_history_ok == true && noutput_items > n_outputs);
//
//    // Multi-rate consume!
//    for (unsigned int i = 0; i < d_nchannels; i++)
//    {
//        consume(i, n_consume[i]); // which input, how many items
//    }
//
//    //consume monitor channel always
//    consume(d_nchannels, 1);
//    return n_outputs;
//
//
}

