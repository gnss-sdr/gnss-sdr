/*!
 * \file gps_l1_ca_pcps_acquisition_fpga.h
 * \brief Adapts a PCPS acquisition block that uses the FPGA to
 *  an AcquisitionInterface for GPS L1 C/A signals
 * \authors <ul>
 *          <li> Marc Majoral, 2018. mmajoral(at)cttc.es
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena(at)gmail.com
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

#ifndef GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_FPGA_H_
#define GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_FPGA_H_

#include "acquisition_interface.h"
#include "gnss_synchro.h"
#include "pcps_acquisition_fpga.h"
#include <string>


class ConfigurationInterface;

/*!
 * \brief This class adapts a PCPS acquisition block to an AcquisitionInterface
 *  for GPS L1 C/A signals
 */
class GpsL1CaPcpsAcquisitionFpga : public AcquisitionInterface
{
public:
    GpsL1CaPcpsAcquisitionFpga(ConfigurationInterface* configuration,
        std::string role, unsigned int in_streams,
        unsigned int out_streams);

    virtual ~GpsL1CaPcpsAcquisitionFpga();

    inline std::string role() override
    {
        return role_;
    }

    /*!
     * \brief Returns "GPS_L1_CA_PCPS_Acquisition"
     */
    inline std::string implementation() override
    {
        return "GPS_L1_CA_PCPS_Acquisition";
    }

    inline size_t item_size() override
    {
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
     * \brief Sets local code for GPS L1/CA PCPS acquisition algorithm.
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
    pcps_acquisition_fpga_sptr acquisition_fpga_;
    unsigned int vector_length_;
    unsigned int code_length_;
    unsigned int channel_;
    unsigned int doppler_max_;
    unsigned int doppler_step_;
    unsigned int sampled_ms_;
    long fs_in_;
    long if_;
    Gnss_Synchro* gnss_synchro_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    lv_16sc_t *d_all_fft_codes; // memory that contains all the code ffts

};

#endif /* GNSS_SDR_GPS_L1_CA_PCPS_ACQUISITION_FPGA_H_ */
