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
#include "GPS_L1_CA.h"
#include <armadillo>
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <matio.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include "display.h"


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
    d_dump = dump;
    d_nchannels = nchannels_out;
    d_dump_filename = dump_filename;
    T_rx_s = 0.0;
    T_rx_step_s = 0.001; // 1 ms
    max_delta = 0.15; // 150 ms
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
    if(filename.size() > 4) { filename.erase(filename.end() - 4, filename.end()); }
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

bool hybrid_observables_cc::interpolate_data(Gnss_Synchro& out, std::deque<Gnss_Synchro>& data, const double& ti)
{
    if((ti < data.front().RX_time) or (ti > data.back().RX_time)) { return false; }
    std::deque<Gnss_Synchro>::iterator it;

    arma::vec t      = arma::vec(data.size());
    arma::vec dop    = t;
    arma::vec cph    = t;
    arma::vec tow    = t;
    arma::vec tiv    = arma::vec(1);
    arma::vec result;
    tiv(0) = ti;

    unsigned int aux = 0;
    for(it = data.begin(); it != data.end(); it++)
    {
        t(aux)   = it->RX_time;
        dop(aux) = it->Carrier_Doppler_hz;
        cph(aux) = it->Carrier_phase_rads;
        tow(aux) = it->TOW_at_current_symbol_s;

        aux++;
    }
    arma::interp1(t, dop, tiv, result);
    out.Carrier_Doppler_hz      = result(0);
    arma::interp1(t, cph, tiv, result);
    out.Carrier_phase_rads      = result(0);
    arma::interp1(t, tow, tiv, result);
    out.TOW_at_current_symbol_s = result(0);

    return result.is_finite();
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

void hybrid_observables_cc::correct_TOW_and_compute_prange(std::vector<Gnss_Synchro>& data)
{
    std::vector<Gnss_Synchro>::iterator it;

/////////////////////// DEBUG //////////////////////////
//   Logs if there is a pseudorange difference between
//   signals of the same satellite higher than a threshold
////////////////////////////////////////////////////////
#ifndef NDEBUG
    std::vector<Gnss_Synchro>::iterator it2;
    double thr_ = 250.0 / SPEED_OF_LIGHT; // Maximum pseudorange difference = 250 meters
    for(it = data.begin(); it != (data.end() - 1); it++)
    {
        for(it2 = it + 1; it2 != data.end(); it2++)
        {
            if(it->PRN == it2->PRN and it->System == it2->System)
            {
                double tow_dif_ = std::fabs(it->TOW_at_current_symbol_s - it2->TOW_at_current_symbol_s);
                if(tow_dif_ > thr_)
                {
                    DLOG(INFO) << "System " << it->System << ". Signals " << it->Signal << " and " << it2->Signal
                               << ". TOW difference in PRN " << it->PRN
                               << " = " << tow_dif_ * 1e3 << "[ms]. Equivalent to " << tow_dif_ * SPEED_OF_LIGHT
                               << " meters in pseudorange";
                    std::cout << TEXT_RED << "System " << it->System << ". Signals " << it->Signal << " and " << it2->Signal
                              << ". TOW difference in PRN " << it->PRN
                              << " = " << tow_dif_ * 1e3 << "[ms]. Equivalent to " << tow_dif_ * SPEED_OF_LIGHT
                              << " meters in pseudorange" << TEXT_RESET << std::endl;
                }
            }
        }
    }
#endif
///////////////////////////////////////////////////////////

    double TOW_ref = std::numeric_limits<double>::lowest();
    for(it = data.begin(); it != data.end(); it++)
    {
        if(it->TOW_at_current_symbol_s > TOW_ref) { TOW_ref = it->TOW_at_current_symbol_s; }
    }
    for(it = data.begin(); it != data.end(); it++)
    {
        double traveltime_s = TOW_ref - it->TOW_at_current_symbol_s + GPS_STARTOFFSET_ms / 1000.0;
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
    T_rx_s += T_rx_step_s;

    //////////////////////////////////////////////////////////////////////////
    if((total_input_items == 0) and (d_num_valid_channels == 0))
    {
        return 0;
    }
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
    if(d_num_valid_channels == 0) { return 0; }

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
    double T_rx_s_out = T_rx_s - (max_delta / 2.0);
    if((d_num_valid_channels == 0) or (T_rx_s_out < 0.0)) { return 0; }

    std::vector<Gnss_Synchro> epoch_data;
    i = 0;
    for(it = d_gnss_synchro_history.begin(); it != d_gnss_synchro_history.end(); it++)
    {
        if(valid_channels[i])
        {
            Gnss_Synchro interpolated_gnss_synchro = it->back();
            if(interpolate_data(interpolated_gnss_synchro, *it, T_rx_s_out))
            {
                epoch_data.push_back(interpolated_gnss_synchro);
            }
            else
            {
                valid_channels[i] = false;
            }
        }
        i++;
    }
    d_num_valid_channels = valid_channels.count();
    if(d_num_valid_channels == 0) { return 0; }
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
