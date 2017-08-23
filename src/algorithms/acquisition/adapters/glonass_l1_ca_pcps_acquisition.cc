#include "glonass_l1_ca_pcps_acquisition.h"
#include <boost/math/distributions/exponential.hpp>
#include <glog/logging.h>
#include "glonass_l1_signal_processing.h"
#include "Glonass_L1_CA.h"
#include "configuration_interface.h"


using google::LogMessage;
GlonassL1CaPcpsAcquisition::GlonassL1CaPcpsAcquisition(
        ConfigurationInterface* configuration, std::string role,
        unsigned int in_streams, unsigned int out_streams) :
    role_(role), in_streams_(in_streams), out_streams_(out_streams)
{
    configuration_ = configuration;
    std::string default_item_type = "gr_complex";
    std::string default_dump_filename = "./data/acquisition.dat";

    DLOG(INFO) << "role " << role;

    item_type_ = configuration_->property(role + ".item_type", default_item_type);

    fs_in_ = configuration_->property("GNSS-SDR.internal_fs_hz", 2048000);
    if_ = configuration_->property(role + ".if", 0);
    dump_ = configuration_->property(role + ".dump", false);
    doppler_max_ = configuration_->property(role + ".doppler_max", 5000);
    sampled_ms_ = configuration_->property(role + ".coherent_integration_time_ms", 1);

    bit_transition_flag_ = configuration_->property(role + ".bit_transition_flag", false);
    use_CFAR_algorithm_flag_=configuration_->property(role + ".use_CFAR_algorithm", true); //will be false in future versions

    max_dwells_ = configuration_->property(role + ".max_dwells", 1);

    dump_filename_ = configuration_->property(role + ".dump_filename", default_dump_filename);

    //--- Find number of samples per spreading code -------------------------
    code_length_ = round(fs_in_ / (GLONASS_L1_CA_CODE_RATE_HZ / GLONASS_L1_CA_CODE_LENGTH_CHIPS));

    vector_length_ = code_length_ * sampled_ms_;

    if( bit_transition_flag_ )
        {
            vector_length_ *= 2;
        }

    code_ = new gr_complex[vector_length_];

    if (item_type_.compare("cshort") == 0 )
        {
            item_size_ = sizeof(lv_16sc_t);
            acquisition_sc_ = pcps_make_acquisition_sc(sampled_ms_, max_dwells_,
                    doppler_max_, if_, fs_in_, code_length_, code_length_,
                    bit_transition_flag_, use_CFAR_algorithm_flag_, dump_, dump_filename_);
            DLOG(INFO) << "acquisition(" << acquisition_sc_->unique_id() << ")";

        }else{
                item_size_ = sizeof(gr_complex);
                acquisition_cc_ = pcps_make_acquisition_cc(sampled_ms_, max_dwells_,
                        doppler_max_, if_, fs_in_, code_length_, code_length_,
                        bit_transition_flag_, use_CFAR_algorithm_flag_, dump_, dump_filename_);
                DLOG(INFO) << "acquisition(" << acquisition_cc_->unique_id() << ")";
        }

    stream_to_vector_ = gr::blocks::stream_to_vector::make(item_size_, vector_length_);
    DLOG(INFO) << "stream_to_vector(" << stream_to_vector_->unique_id() << ")";
    
    if (item_type_.compare("cbyte") == 0)
        {
            cbyte_to_float_x2_ = make_complex_byte_to_float_x2();
            float_to_complex_ = gr::blocks::float_to_complex::make();
        }

    channel_ = 0;
    threshold_ = 0.0;
    doppler_step_ = 0;
    gnss_synchro_ = 0;
}


GlonassL1CaPcpsAcquisition::~GlonassL1CaPcpsAcquisition()
{
    delete[] code_;
}


void GlonassL1CaPcpsAcquisition::set_channel(unsigned int channel)
{
    channel_ = channel;
    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_channel(channel_);
        }
    else
        {
            acquisition_cc_->set_channel(channel_);
        }
}


void GlonassL1CaPcpsAcquisition::set_threshold(float threshold)
{
    float pfa = configuration_->property(role_ + ".pfa", 0.0);

    if(pfa == 0.0)
        {
            threshold_ = threshold;
        }
    else
        {
            threshold_ = calculate_threshold(pfa);
        }

    DLOG(INFO) << "Channel " << channel_ << " Threshold = " << threshold_;


    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_threshold(threshold_);
        }
    else
        {
            acquisition_cc_->set_threshold(threshold_);
        }
}


void GlonassL1CaPcpsAcquisition::set_doppler_max(unsigned int doppler_max)
{
    doppler_max_ = doppler_max;

    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_doppler_max(doppler_max_);
        }
    else
        {
            acquisition_cc_->set_doppler_max(doppler_max_);
        }
}


void GlonassL1CaPcpsAcquisition::set_doppler_step(unsigned int doppler_step)
{
    doppler_step_ = doppler_step;

    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_doppler_step(doppler_step_);
        }
    else
        {
            acquisition_cc_->set_doppler_step(doppler_step_);
        }
}


void GlonassL1CaPcpsAcquisition::set_gnss_synchro(Gnss_Synchro* gnss_synchro)
{
    gnss_synchro_ = gnss_synchro;

    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_gnss_synchro(gnss_synchro_);
        }
    else
        {
            acquisition_cc_->set_gnss_synchro(gnss_synchro_);
        }
}


signed int GlonassL1CaPcpsAcquisition::mag()
{
    if (item_type_.compare("cshort") == 0)
        {
            return acquisition_sc_->mag();
        }
    else
        {
            return acquisition_cc_->mag();
        }
}


void GlonassL1CaPcpsAcquisition::init()
{
    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->init();
        }
    else
        {
            acquisition_cc_->init();
        }

    set_local_code();
}


void GlonassL1CaPcpsAcquisition::set_local_code()
{

    std::complex<float>* code = new std::complex<float>[code_length_];

    glonass_l1_ca_code_gen_complex_sampled(code,/* gnss_synchro_->PRN,*/ fs_in_, 0);

    for (unsigned int i = 0; i < sampled_ms_; i++)
        {
            memcpy(&(code_[i*code_length_]), code,
                    sizeof(gr_complex)*code_length_);
        }

    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_local_code(code_);
        }
    else
        {
            acquisition_cc_->set_local_code(code_);
        }

    delete[] code;
}


void GlonassL1CaPcpsAcquisition::reset()
{
    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_active(true);
        }
    else
        {
            acquisition_cc_->set_active(true);
        }
}


void GlonassL1CaPcpsAcquisition::set_state(int state)
{
    if (item_type_.compare("cshort") == 0)
        {
            acquisition_sc_->set_state(state);
        }
    else
        {
            acquisition_cc_->set_state(state);
        }
}


float GlonassL1CaPcpsAcquisition::calculate_threshold(float pfa)
{
    //Calculate the threshold
    unsigned int frequency_bins = 0;
    /*
    for (int doppler = (int)(-doppler_max_); doppler <= (int)doppler_max_; doppler += doppler_step_)
        {
            frequency_bins++;
        }
    */

    frequency_bins = (2*doppler_max_ + doppler_step_)/doppler_step_;

    DLOG(INFO) << "Channel " << channel_ << "  Pfa = " << pfa;
    unsigned int ncells = vector_length_ * frequency_bins;
    double exponent = 1 / static_cast<double>(ncells);
    double val = pow(1.0 - pfa, exponent);
    double lambda = double(vector_length_);
    boost::math::exponential_distribution<double> mydist (lambda);
    float threshold = (float)quantile(mydist,val);

    return threshold;
}


void GlonassL1CaPcpsAcquisition::connect(gr::top_block_sptr top_block)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            top_block->connect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
    else if (item_type_.compare("cshort") == 0)
        {
            top_block->connect(stream_to_vector_, 0, acquisition_sc_, 0);
        }
    else if (item_type_.compare("cbyte") == 0)
        {
            top_block->connect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
            top_block->connect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
            top_block->connect(float_to_complex_, 0, stream_to_vector_, 0);
            top_block->connect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


void GlonassL1CaPcpsAcquisition::disconnect(gr::top_block_sptr top_block)
{
    if (item_type_.compare("gr_complex") == 0)
        {
            top_block->disconnect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
    else if (item_type_.compare("cshort") == 0)
        {
            top_block->disconnect(stream_to_vector_, 0, acquisition_sc_, 0);
        }
    else if (item_type_.compare("cbyte") == 0)
        {
            // Since a byte-based acq implementation is not available,
            // we just convert cshorts to gr_complex
            top_block->disconnect(cbyte_to_float_x2_, 0, float_to_complex_, 0);
            top_block->disconnect(cbyte_to_float_x2_, 1, float_to_complex_, 1);
            top_block->disconnect(float_to_complex_, 0, stream_to_vector_, 0);
            top_block->disconnect(stream_to_vector_, 0, acquisition_cc_, 0);
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
        }
}


gr::basic_block_sptr GlonassL1CaPcpsAcquisition::get_left_block()
{
    if (item_type_.compare("gr_complex") == 0)
        {
            return stream_to_vector_;
        }
    else if (item_type_.compare("cshort") == 0)
        {
            return stream_to_vector_;
        }
    else if (item_type_.compare("cbyte") == 0)
        {
            return cbyte_to_float_x2_;
        }
    else
        {
            LOG(WARNING) << item_type_ << " unknown acquisition item type";
            return nullptr;
        }
}


gr::basic_block_sptr GlonassL1CaPcpsAcquisition::get_right_block()
{
    if (item_type_.compare("cshort") == 0)
        {
            return acquisition_sc_;
        }
    else
        {
            return acquisition_cc_;
        }
}

