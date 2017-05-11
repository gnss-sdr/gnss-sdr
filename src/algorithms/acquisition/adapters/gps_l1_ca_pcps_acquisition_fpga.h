/*!
 * \file gps_l1_ca_pcps_acquisition_fpga.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L1 C/A signals. This file is based on the file gps_l1_ca_pcps_acquisition.h
 * \authors <ul>
 * 			<li> Marc Majoral, 2017. mmajoral(at)cttc.cat
 *          </ul>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#ifndef GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_FPGA_H_
#define GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_FPGA_H_

#include <string>
#include <gnuradio/blocks/stream_to_vector.h>
#include <gnuradio/blocks/float_to_complex.h>
#include "gnss_synchro.h"
#include "acquisition_interface.h"
#include "gps_pcps_acquisition_fpga_sc.h"
#include "complex_byte_to_float_x2.h"
#include <volk_gnsssdr/volk_gnsssdr.h>



class ConfigurationInterface;

/*!
 * \brief This class adapts a PCPS acquisition block to an AcquisitionInterface
 *  for GPS L1 C/A signals
 */
class GpsL1CaPcpsAcquisitionFpga: public AcquisitionInterface
{
public:
    GpsL1CaPcpsAcquisitionFpga(ConfigurationInterface* configuration,
            std::string role, unsigned int in_streams,
            unsigned int out_streams);

    virtual ~GpsL1CaPcpsAcquisitionFpga();

    std::string role()
    {
        return role_;
    }

    /*!
     * \brief Returns "GPS_L1_CA_PCPS_Acquisition"
     */
    std::string implementation()
    {
        return "GPS_L1_CA_PCPS_Acquisition_Fpga";
    }
    size_t item_size()
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and
     *  tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);

    /*!
     * \brief Set acquisition channel unique ID
     */
    void set_channel(unsigned int channel);

    /*!
     * \brief Set statistics threshold of PCPS algorithm
     */
    void set_threshold(float threshold);

    /*!
     * \brief Set maximum Doppler off grid search
     */
    void set_doppler_max(unsigned int doppler_max);

    /*!
     * \brief Set Doppler steps for the grid search
     */
    void set_doppler_step(unsigned int doppler_step);

    /*!
     * \brief Initializes acquisition algorithm.
     */
    void init();

    /*!
     * \brief Sets local code for GPS L1/CA PCPS acquisition algorithm.
     */
    void set_local_code();

    /*!
     * \brief Returns the maximum peak of grid search
     */
    signed int mag();

    /*!
     * \brief Restart acquisition algorithm
     */
    void reset();

    /*!
     * \brief If state = 1, it forces the block to start acquiring from the first sample
     */
    void set_state(int state);

private:
    ConfigurationInterface* configuration_;
    gps_pcps_acquisition_fpga_sc_sptr gps_acquisition_fpga_sc_;
    gr::blocks::stream_to_vector::sptr stream_to_vector_;
    gr::blocks::float_to_complex::sptr float_to_complex_;
    complex_byte_to_float_x2_sptr cbyte_to_float_x2_;
    size_t item_size_;
    std::string item_type_;
    unsigned int vector_length_;
    unsigned int code_length_;
    bool bit_transition_flag_;
    bool use_CFAR_algorithm_flag_;
    unsigned int channel_;
    float threshold_;
    unsigned int doppler_max_;
    unsigned int doppler_step_;
    unsigned int sampled_ms_;
    unsigned int max_dwells_;
    long fs_in_;
    long if_;
    bool dump_;
    std::string dump_filename_;
    std::complex<float> * code_;
    Gnss_Synchro * gnss_synchro_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;

    unsigned int nsamples_total_;

    unsigned int select_queue_Fpga_;

    float calculate_threshold(float pfa);
};

#endif /* GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_H_ */
