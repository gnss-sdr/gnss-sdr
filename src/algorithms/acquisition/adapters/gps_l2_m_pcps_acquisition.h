/*!
 * \file gps_l2_m_pcps_acquisition.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  GPS L2 M signals
 * \authors <ul>
 *          <li> Javier Arribas, 2015. jarribas(at)cttc.es
 *          </ul>
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

#ifndef GNSS_SDR_GPS_L2_M_PCPS_ACQUISITION_H_
#define GNSS_SDR_GPS_L2_M_PCPS_ACQUISITION_H_

#include <string>
#include <gnuradio/blocks/stream_to_vector.h>
#include <gnuradio/blocks/float_to_complex.h>
#include "gnss_synchro.h"
#include "acquisition_interface.h"
#include "pcps_acquisition_cc.h"
#include "complex_byte_to_float_x2.h"
#include <volk_gnsssdr/volk_gnsssdr.h>



class ConfigurationInterface;

/*!
 * \brief This class adapts a PCPS acquisition block to an AcquisitionInterface
 *  for GPS L2 M signals
 */
class GpsL2MPcpsAcquisition: public AcquisitionInterface
{
public:
    GpsL2MPcpsAcquisition(ConfigurationInterface* configuration,
            std::string role, unsigned int in_streams,
            unsigned int out_streams);

    virtual ~GpsL2MPcpsAcquisition();

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "GPS_L2_M_PCPS_Acquisition"
     */
    inline std::string implementation() override
    {
        return "GPS_L2_M_PCPS_Acquisition";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and
     *  tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

    /*!
     * \brief Set acquisition channel unique ID
     */
    void set_channel(unsigned int channel) override;

    /*!
     * \brief Set statistics threshold of PCPS algorithm
     */
    void set_threshold(float threshold) override;

    /*!
     * \brief Set maximum Doppler off grid search
     */
    void set_doppler_max(unsigned int doppler_max) override;

    /*!
     * \brief Set Doppler steps for the grid search
     */
    void set_doppler_step(unsigned int doppler_step) override;

    /*!
     * \brief Initializes acquisition algorithm.
     */
    void init() override;

    /*!
     * \brief Sets local code for GPS L2/M PCPS acquisition algorithm.
     */
    void set_local_code() override;

    /*!
     * \brief Returns the maximum peak of grid search
     */
    signed int mag() override;

    /*!
     * \brief Restart acquisition algorithm
     */
    void reset() override;

    /*!
     * \brief If state = 1, it forces the block to start acquiring from the first sample
     */
    void set_state(int state);

private:
    ConfigurationInterface* configuration_;
    pcps_acquisition_cc_sptr acquisition_cc_;
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
    unsigned int max_dwells_;
    long fs_in_;
    long if_;
    bool dump_;
    bool blocking_;
    std::string dump_filename_;
    std::complex<float> * code_;
    Gnss_Synchro * gnss_synchro_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;

    float calculate_threshold(float pfa);
};

#endif /* GNSS_SDR_GPS_L2_M_PCPS_ACQUISITION_H_ */
