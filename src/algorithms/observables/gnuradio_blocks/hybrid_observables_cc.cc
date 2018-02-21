/*!
 * \file hybrid_observables_cc.cc
 * \brief Implementation of the pseudorange computation block
 * \author Javier Arribas 2017. jarribas(at)cttc.es
 * \author Antonio Ramos  2018. antonio.ramos(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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


void hybrid_observables_cc::forecast(int noutput_items __attribute__((unused)),
        gr_vector_int &ninput_items_required)
{
    for(unsigned int i = 0; i < d_nchannels; i++) { ninput_items_required[i] = 0; }
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
                // Add the new Gnss_Synchros to their corresponding deque
                for(int aux = 0; aux < ninput_items[i]; aux++)
                {
                    if(in[i][aux].Flag_valid_word)
                    {
                        it->push_back(in[i][aux]);
                        it->back().RX_time = compute_T_rx_s(in[i][aux]);
                        // Check if the last Gnss_Synchro comes from the same satellite as the previous ones
                        if(it->size() > 1)
                        {
                            if(it->front().PRN != it->back().PRN) { it->clear(); }
                        }
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
            std::pair<Gnss_Synchro, Gnss_Synchro> gnss_pair(it->at(index2), it->at(index1));

            interpolated_gnss_synchro.Carrier_Doppler_hz = interpolate_data(gnss_pair, T_rx_s, 0);
            interpolated_gnss_synchro.Carrier_phase_rads = interpolate_data(gnss_pair, T_rx_s, 1);
            interpolated_gnss_synchro.RX_time = interpolate_data(gnss_pair, T_rx_s, 2);
            interpolated_gnss_synchro.TOW_at_current_symbol_s = interpolated_gnss_synchro.RX_time;

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
    if(d_dump)
    {
        // MULTIPLEXED FILE RECORDING - Record results to file
        try
        {
            double tmp_double;
            for (i = 0; i < d_nchannels; i++)
            {
                tmp_double = out[i][0].RX_time;
                d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                tmp_double = out[i][0].TOW_at_current_symbol_s;
                d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                tmp_double = out[i][0].Carrier_Doppler_hz;
                d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                tmp_double = out[i][0].Carrier_phase_rads / GPS_TWO_PI;
                d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                tmp_double = out[i][0].Pseudorange_m;
                d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                tmp_double = static_cast<double>(out[i][0].PRN);
                d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
                tmp_double = static_cast<double>(out[i][0].Flag_valid_pseudorange);
                d_dump_file.write(reinterpret_cast<char*>(&tmp_double), sizeof(double));
            }
        }
        catch (const std::ifstream::failure& e)
        {
            LOG(WARNING) << "Exception writing observables dump file " << e.what();
            d_dump = false;
        }
    }
    return 1;
}
