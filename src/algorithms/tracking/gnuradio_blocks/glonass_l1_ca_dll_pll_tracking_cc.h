#ifndef GNSS_SDR_GLONASS_L1_CA_DLL_PLL_TRACKING_CC_H
#define GNSS_SDR_GLONASS_L1_CA_DLL_PLL_TRACKING_CC_H

#include <fstream>
#include <map>
#include <string>
#include <gnuradio/block.h>
#include "gnss_synchro.h"
#include "tracking_2nd_DLL_filter.h"
#include "tracking_2nd_PLL_filter.h"
#include "cpu_multicorrelator.h"

class Glonass_L1_Ca_Dll_Pll_Tracking_cc;

typedef boost::shared_ptr<Glonass_L1_Ca_Dll_Pll_Tracking_cc>
        glonass_l1_ca_dll_pll_tracking_cc_sptr;

glonass_l1_ca_dll_pll_tracking_cc_sptr
glonass_l1_ca_dll_pll_make_tracking_cc(long if_freq,
                                   long fs_in, unsigned
                                   int vector_length,
                                   bool dump,
                                   std::string dump_filename,
                                   float pll_bw_hz,
                                   float dll_bw_hz,
                                   float early_late_space_chips);



/*!
 * \brief This class implements a DLL + PLL tracking loop block
 */
class Glonass_L1_Ca_Dll_Pll_Tracking_cc: public gr::block
{
public:
    ~Glonass_L1_Ca_Dll_Pll_Tracking_cc();

    void set_channel(unsigned int channel);
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);
    void start_tracking();

    int general_work (int noutput_items, gr_vector_int &ninput_items,
            gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

    void forecast (int noutput_items, gr_vector_int &ninput_items_required);

private:
    friend glonass_l1_ca_dll_pll_tracking_cc_sptr
    glonass_l1_ca_dll_pll_make_tracking_cc(long if_freq,
            long fs_in, unsigned
            int vector_length,
            bool dump,
            std::string dump_filename,
            float pll_bw_hz,
            float dll_bw_hz,
            float early_late_space_chips);

    Glonass_L1_Ca_Dll_Pll_Tracking_cc(long if_freq,
            long fs_in, unsigned
            int vector_length,
            bool dump,
            std::string dump_filename,
            float pll_bw_hz,
            float dll_bw_hz,
            float early_late_space_chips);

    // tracking configuration vars
    unsigned int d_vector_length;
    bool d_dump;

    Gnss_Synchro* d_acquisition_gnss_synchro;
    unsigned int d_channel;

    long d_if_freq;
    long d_fs_in;

    double d_early_late_spc_chips;

    // remaining code phase and carrier phase between tracking loops
    double d_rem_code_phase_samples;
    double d_rem_code_phase_chips;
    double d_rem_carr_phase_rad;

    // PLL and DLL filter library
    Tracking_2nd_DLL_filter d_code_loop_filter;
    Tracking_2nd_PLL_filter d_carrier_loop_filter;

    // acquisition
    double d_acq_code_phase_samples;
    double d_acq_carrier_doppler_hz;
    // correlator
    int d_n_correlator_taps;
    gr_complex* d_ca_code;
    float* d_local_code_shift_chips;
    gr_complex* d_correlator_outs;
    cpu_multicorrelator multicorrelator_cpu;


    // tracking vars
    double d_code_freq_chips;
    double d_code_phase_step_chips;
    double d_carrier_doppler_hz;
    double d_carrier_phase_step_rad;
    double d_acc_carrier_phase_rad;
    double d_code_phase_samples;

    //PRN period in samples
    int d_current_prn_length_samples;

    //processing samples counters
    unsigned long int d_sample_counter;
    unsigned long int d_acq_sample_stamp;

    // CN0 estimation and lock detector
    int d_cn0_estimation_counter;
    gr_complex* d_Prompt_buffer;
    double d_carrier_lock_test;
    double d_CN0_SNV_dB_Hz;
    double d_carrier_lock_threshold;
    int d_carrier_lock_fail_counter;

    // control vars
    bool d_enable_tracking;
    bool d_pull_in;

    // file dump
    std::string d_dump_filename;
    std::ofstream d_dump_file;

    std::map<std::string, std::string> systemName;
    std::string sys;
};

#endif //GNSS_SDR_GLONASS_L1_CA_DLL_PLL_TRACKING_CC_H
