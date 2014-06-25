/*
 * galileo_e5a_pilot_3ms_acquisition_cc.h
 *
 *  Created on: Jun 22, 2014
 *      Author: marc
 */

#ifndef GALILEO_E5A_PILOT_3MS_ACQUISITION_CC_H_
#define GALILEO_E5A_PILOT_3MS_ACQUISITION_CC_H_

#include <fstream>
#include <queue>
#include <string>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <gnuradio/block.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/gr_complex.h>
#include <gnuradio/fft/fft.h>
#include "concurrent_queue.h"
#include "gnss_synchro.h"

class galileo_e5a_pilot_3ms_acquisition_cc;

typedef boost::shared_ptr<galileo_e5a_pilot_3ms_acquisition_cc> galileo_e5a_pilot_3ms_acquisition_cc_sptr;

galileo_e5a_pilot_3ms_acquisition_cc_sptr
galileo_e5a_pilot_3ms_make_acquisition_cc(unsigned int max_dwells,
                         unsigned int doppler_max, long freq, long fs_in,
                         int samples_per_ms, int samples_per_code,
                         bool bit_transition_flag,
                         gr::msg_queue::sptr queue, bool dump,
                         std::string dump_filename);

/*!
 * \brief This class implements a Parallel Code Phase Search Acquisition.
 *
 * Check \ref Navitec2012 "An Open Source Galileo E1 Software Receiver",
 * Algorithm 1, for a pseudocode description of this implementation.
 */
class galileo_e5a_pilot_3ms_acquisition_cc: public gr::block
{
private:
    friend galileo_e5a_pilot_3ms_acquisition_cc_sptr
    galileo_e5a_pilot_3ms_make_acquisition_cc(unsigned int max_dwells,
            unsigned int doppler_max, long freq, long fs_in,
            int samples_per_ms, int samples_per_code,
            bool bit_transition_flag,
            gr::msg_queue::sptr queue, bool dump,
            std::string dump_filename);

    galileo_e5a_pilot_3ms_acquisition_cc(unsigned int max_dwells,
            unsigned int doppler_max, long freq, long fs_in,
            int samples_per_ms, int samples_per_code,
            bool bit_transition_flag,
            gr::msg_queue::sptr queue, bool dump,
            std::string dump_filename);

    void calculate_magnitudes(gr_complex* fft_begin, int doppler_shift,
            int doppler_offset);

    long d_fs_in;
    long d_freq;
    int d_samples_per_ms;
    int d_samples_per_code;
    unsigned int d_doppler_resolution;
    float d_threshold;
    std::string d_satellite_str;
    unsigned int d_doppler_max;
    unsigned int d_doppler_step;
    unsigned int d_max_dwells;
    unsigned int d_well_count;
    unsigned int d_fft_size;
    unsigned long int d_sample_counter;
    gr_complex** d_grid_doppler_wipeoffs;
    unsigned int d_num_doppler_bins;
    gr_complex* d_fft_code_A;
    gr_complex* d_fft_code_B;
    gr_complex* d_fft_code_C;
    gr_complex* d_fft_code_D;
    gr::fft::fft_complex* d_fft_if;
    gr::fft::fft_complex* d_ifft;
    Gnss_Synchro *d_gnss_synchro;
    unsigned int d_code_phase;
    float d_doppler_freq;
    float d_mag;
    float* d_magnitude;
    float d_input_power;
    float d_test_statistics;
    bool d_bit_transition_flag;
    gr::msg_queue::sptr d_queue;
    concurrent_queue<int> *d_channel_internal_queue;
    std::ofstream d_dump_file;
    bool d_active;
    int d_state;
    bool d_dump;
    unsigned int d_channel;
    std::string d_dump_filename;

public:
    /*!
     * \brief Default destructor.
     */
     ~galileo_e5a_pilot_3ms_acquisition_cc();

     /*!
      * \brief Set acquisition/tracking common Gnss_Synchro object pointer
      * to exchange synchronization data between acquisition and tracking blocks.
      * \param p_gnss_synchro Satellite information shared by the processing blocks.
      */
     void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro)
     {
         d_gnss_synchro = p_gnss_synchro;
     }

     /*!
      * \brief Returns the maximum peak of grid search.
      */
     unsigned int mag()
     {
         return d_mag;
     }

     /*!
      * \brief Initializes acquisition algorithm.
      */
     void init();

     /*!
      * \brief Sets local code for PCPS acquisition algorithm.
      * \param code - Pointer to the PRN code.
      */
     void set_local_code(std::complex<float> * code);

     /*!
      * \brief Starts acquisition algorithm, turning from standby mode to
      * active mode
      * \param active - bool that activates/deactivates the block.
      */
     void set_active(bool active)
     {
         d_active = active;
     }

     /*!
      * \brief Set acquisition channel unique ID
      * \param channel - receiver channel.
      */
     void set_channel(unsigned int channel)
     {
         d_channel = channel;
     }

     /*!
      * \brief Set statistics threshold of PCPS algorithm.
      * \param threshold - Threshold for signal detection (check \ref Navitec2012,
      * Algorithm 1, for a definition of this threshold).
      */
     void set_threshold(float threshold)
     {
         d_threshold = threshold;
     }

     /*!
      * \brief Set maximum Doppler grid search
      * \param doppler_max - Maximum Doppler shift considered in the grid search [Hz].
      */
     void set_doppler_max(unsigned int doppler_max)
     {
         d_doppler_max = doppler_max;
     }

     /*!
      * \brief Set Doppler steps for the grid search
      * \param doppler_step - Frequency bin of the search grid [Hz].
      */
     void set_doppler_step(unsigned int doppler_step)
     {
         d_doppler_step = doppler_step;
     }


     /*!
      * \brief Set tracking channel internal queue.
      * \param channel_internal_queue - Channel's internal blocks information queue.
      */
     void set_channel_queue(concurrent_queue<int> *channel_internal_queue)
     {
         d_channel_internal_queue = channel_internal_queue;
     }

     /*!
      * \brief Parallel Code Phase Search Acquisition signal processing.
      */
     int general_work(int noutput_items, gr_vector_int &ninput_items,
             gr_vector_const_void_star &input_items,
             gr_vector_void_star &output_items);
};

#endif /* GALILEO_E5A_PILOT_3MS_ACQUISITION_CC_H_ */
