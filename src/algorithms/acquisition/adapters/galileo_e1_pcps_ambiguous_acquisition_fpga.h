/*!
 * \file galileo_e1_pcps_ambiguous_acquisition.h
 * \brief Adapts a PCPS acquisition block to an AcquisitionInterface for
 *  Galileo E1 Signals
 * \author Luis Esteve, 2012. luis(at)epsilon-formacion.com
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

#ifndef GNSS_SDR_GALILEO_E1_PCPS_AMBIGUOUS_ACQUISITION_FPGA_H_
#define GNSS_SDR_GALILEO_E1_PCPS_AMBIGUOUS_ACQUISITION_FPGA_H_

#include "acquisition_interface.h"
#include "gnss_synchro.h"
#include "pcps_acquisition_fpga.h"
#include "complex_byte_to_float_x2.h"
#include <gnuradio/blocks/stream_to_vector.h>
#include <gnuradio/blocks/float_to_complex.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <string>


class ConfigurationInterface;

/*!
 * \brief This class adapts a PCPS acquisition block to an
 *  AcquisitionInterface for Galileo E1 Signals
 */
class GalileoE1PcpsAmbiguousAcquisitionFpga : public AcquisitionInterface
{
public:
    GalileoE1PcpsAmbiguousAcquisitionFpga(ConfigurationInterface* configuration,
        std::string role, unsigned int in_streams,
        unsigned int out_streams);

    virtual ~GalileoE1PcpsAmbiguousAcquisitionFpga();

    inline std::string role() override
    {
        //   printf("top acq role\n");
        return role_;
    }

    /*!
     * \brief Returns "Galileo_E1_PCPS_Ambiguous_Acquisition"
     */
    inline std::string implementation() override
    {
        //  printf("top acq implementation\n");
        return "Galileo_E1_PCPS_Ambiguous_Acquisition_Fpga";
    }

    size_t item_size() override
    {
        //   printf("top acq item size\n");
        size_t item_size = sizeof(lv_16sc_t);
        return item_size;
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
     * \brief Sets local code for Galileo E1 PCPS acquisition algorithm.
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
    void set_state(int state) override;

    /*!
      * \brief This function is only used in the unit tests
      */
     void set_single_doppler_flag(unsigned int single_doppler_flag);
     /*!
      * \brief This function is only used in the unit tests
      */
     void read_acquisition_results(uint32_t *max_index,
         float *max_magnitude, float *second_magnitude, uint64_t *initial_sample, uint32_t *doppler_index, uint32_t *total_fft_scaling_factor);


     /*!
      * \brief This function is only used in the unit tests
      */
     void reset_acquisition(void);

     /*!
      * \brief This function is only used in the unit tests
      */
     //void read_fpga_total_scale_factor(uint32_t *total_scale_factor, uint32_t *fw_scale_factor);

     /*!
      * \brief Stop running acquisition
      */
     void stop_acquisition() override;

private:
    ConfigurationInterface* configuration_;
    //pcps_acquisition_sptr acquisition_;
    pcps_acquisition_fpga_sptr acquisition_fpga_;
    gr::blocks::stream_to_vector::sptr stream_to_vector_;
    gr::blocks::float_to_complex::sptr float_to_complex_;
    complex_byte_to_float_x2_sptr cbyte_to_float_x2_;
    // size_t item_size_;
    // std::string item_type_;
    //unsigned int vector_length_;
    //unsigned int code_length_;
    bool bit_transition_flag_;
    bool use_CFAR_algorithm_flag_;
    bool acquire_pilot_;
    unsigned int channel_;
    //float threshold_;
    unsigned int doppler_max_;
    unsigned int doppler_step_;
    //unsigned int sampled_ms_;
    unsigned int max_dwells_;
    //long fs_in_;
    //long if_;
    bool dump_;
    bool blocking_;
    std::string dump_filename_;
    //std::complex<float>* code_;
    Gnss_Synchro* gnss_synchro_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    //float calculate_threshold(float pfa);

    // extra for the FPGA
    lv_16sc_t* d_all_fft_codes_;  // memory that contains all the code ffts
};

#endif /* GNSS_SDR_GALILEO_E1_PCPS_AMBIGUOUS_ACQUISITION_FPGA_H_ */
