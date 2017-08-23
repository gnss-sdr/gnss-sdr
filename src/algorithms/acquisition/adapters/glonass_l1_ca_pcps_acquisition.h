
#ifndef GNSS_SDR_GLONASS_L1_CA_PCPS_ACQUISITION_H_
#define GNSS_SDR_GLONASS_L1_CA_PCPS_ACQUISITION_H_

#include <string>
#include <gnuradio/blocks/stream_to_vector.h>
#include <gnuradio/blocks/float_to_complex.h>
#include "gnss_synchro.h"
#include "acquisition_interface.h"
#include "pcps_acquisition_cc.h"
#include "pcps_acquisition_sc.h"
#include "complex_byte_to_float_x2.h"
#include <volk_gnsssdr/volk_gnsssdr.h>



class ConfigurationInterface;

/*!
 * \brief This class adapts a PCPS acquisition block to an AcquisitionInterface
 *  for GPS L1 C/A signals
 */
class GlonassL1CaPcpsAcquisition: public AcquisitionInterface
{
public:
    GlonassL1CaPcpsAcquisition(ConfigurationInterface* configuration,
            std::string role, unsigned int in_streams,
            unsigned int out_streams);

    virtual ~GlonassL1CaPcpsAcquisition();

    std::string role()
    {
        return role_;
    }

    /*!
     * \brief Returns "GLONASS_L1_CA_PCPS_Acquisition"
     */
    std::string implementation()
    {
        return "GLONASS_L1_CA_PCPS_Acquisition";
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
    pcps_acquisition_cc_sptr acquisition_cc_;
    pcps_acquisition_sc_sptr acquisition_sc_;
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

    float calculate_threshold(float pfa);
};

#endif /* GNSS_SDR_GLONASS_L1_CA_PCPS_ACQUISITION_H_ */
