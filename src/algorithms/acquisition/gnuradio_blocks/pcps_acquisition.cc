/*!
 * \file pcps_acquisition.cc
 * \brief This class implements a Parallel Code Phase Search Acquisition
 * \authors <ul>
 *          <li> Javier Arribas, 2011. jarribas(at)cttc.es
 *          <li> Luis Esteve, 2012. luis(at)epsilon-formacion.com
 *          <li> Marc Molina, 2013. marc.molina.pena@gmail.com
 *          <li> Cillian O'Driscoll, 2017. cillian(at)ieee.org
 *          </ul>
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "pcps_acquisition.h"
#include "GLONASS_L1_L2_CA.h"  // for GLONASS_PRN
#include "MATH_CONSTANTS.h"    // for TWO_PI
#include "gnss_frequencies.h"
#include "gnss_sdr_create_directory.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_synchro.h"
#include "matlab_writter_helper.h"
#include <boost/math/special_functions/gamma.hpp>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for from_long
#include <pmt/pmt_sugar.h>  // for mp
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <algorithm>  // for std::fill_n, std::min, std::copy
#include <array>
#include <cmath>  // for floor, fmod, rint, ceil
#include <exception>
#include <iostream>
#include <limits>
#include <map>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif


namespace
{
float compute_threshold(float pfa, uint32_t effective_fft_size, uint32_t num_doppler_bins, uint32_t max_dwells)
{
    const int num_bins = effective_fft_size * num_doppler_bins;
    return static_cast<float>(2.0 * boost::math::gamma_p_inv(2.0 * max_dwells, std::pow(1.0 - pfa, 1.0 / static_cast<float>(num_bins))));
}

std::string get_dump_filename(std::string dump_filename)
{
    std::string dump_path;

    // Get path
    if (dump_filename.find_last_of('/') != std::string::npos)
        {
            const auto last_slash_index = dump_filename.find_last_of('/');
            dump_path = dump_filename.substr(0, last_slash_index);
            dump_filename = dump_filename.substr(last_slash_index + 1);
        }
    else
        {
            dump_path = std::string(".");
        }

    if (dump_filename.empty())
        {
            dump_filename = "acquisition";
        }

    // remove extension if any
    if (dump_filename.substr(1).find_last_of('.') != std::string::npos)
        {
            dump_filename = dump_filename.substr(0, dump_filename.find_last_of('.'));
        }

    if (!gnss_sdr_create_directory(dump_path))
        {
            std::cerr << "GNSS-SDR cannot create dump file for the Acquisition block. Wrong permissions?\n";
            return {};
        }

    return dump_path + fs::path::preferred_separator + dump_filename;
}

template <typename T>
uint32_t aligned_row_stride(uint32_t row_size)
{
    const auto alignment = static_cast<size_t>(volk_get_alignment());
    if (alignment == 0)
        {
            return row_size;
        }

    uint32_t stride = row_size;
    while ((static_cast<size_t>(stride) * sizeof(T)) % alignment != 0)
        {
            ++stride;
        }
    return stride;
}
}  // namespace

pcps_acquisition_sptr pcps_make_acquisition(const Acq_Conf& conf_)
{
    return pcps_acquisition_sptr(new pcps_acquisition(conf_));
}


pcps_acquisition::pcps_acquisition(const Acq_Conf& conf_)
    : acquisition_impl_interface("pcps_acquisition",
          gr::io_signature::make(1, 1, conf_.it_size),
          gr::io_signature::make(0, 1, sizeof(Gnss_Synchro))),
      d_acq_parameters(conf_),
      d_dump_filename(conf_.dump ? get_dump_filename(conf_.dump_filename) : std::string{}),
      d_doppler_max(conf_.doppler_max),
      d_samplesPerChip(conf_.samples_per_chip),
      d_doppler_step(conf_.doppler_step),
      d_consumed_samples(conf_.sampled_ms * conf_.samples_per_ms * (conf_.bit_transition_flag ? 2.0 : 1.0)),
      d_fft_size(conf_.sampled_ms == conf_.ms_per_code ? d_consumed_samples : d_consumed_samples * 2),
      d_effective_fft_size(conf_.bit_transition_flag ? (d_fft_size / 2) : d_fft_size),
      d_magnitude_grid_stride(aligned_row_stride<float>(d_effective_fft_size)),
      d_doppler_wipeoffs_stride(aligned_row_stride<gr_complex>(d_fft_size)),
      d_num_doppler_bins(static_cast<uint32_t>(std::ceil(static_cast<double>(2 * d_doppler_max) / static_cast<double>(d_doppler_step)))),
      d_num_doppler_bins_step1_capacity(std::max(d_num_doppler_bins, 2U)),
      d_num_doppler_bins_step2(conf_.num_doppler_bins_step2),
      d_dump_channel(conf_.dump_channel),
      d_threshold(conf_.pfa > 0.0 ? compute_threshold(conf_.pfa, d_effective_fft_size, d_num_doppler_bins, conf_.bit_transition_flag ? 1 : conf_.max_dwells) : conf_.threshold),
      d_threshold_narrowed(conf_.pfa > 0.0 ? compute_threshold(conf_.pfa, d_effective_fft_size, 1U, conf_.bit_transition_flag ? 1 : conf_.max_dwells) : conf_.threshold),
      d_threshold_step_two(conf_.pfa2 > 0.0 ? compute_threshold(conf_.pfa2, d_effective_fft_size, d_num_doppler_bins_step2, conf_.bit_transition_flag ? 1 : conf_.max_dwells) : conf_.threshold),
      d_cshort(conf_.it_size != sizeof(gr_complex)),
      d_use_CFAR_algorithm_flag(conf_.use_CFAR_algorithm_flag),
      d_dump(!d_dump_filename.empty()),
      d_worker(nullptr),
      d_gnss_synchro(nullptr),
      d_state(0),
      d_doppler_center(0),
      d_doppler_bias(0),
      d_num_doppler_bins_active(d_num_doppler_bins),
      d_doppler_search_narrowed(false),
      d_buffer_count(0),
      d_channel(0),
      d_resampler_latency_samples(conf_.resampler_latency_samples),
      d_sample_count(0),
      d_step_two(false),
      d_active(false),
      d_worker_active(false),
      d_num_noncoherent_integrations_counter(0),
      d_dump_number(0),
      d_input_power(0),
      d_doppler_center_step_two(0),
      d_magnitude_grid(std::max(d_num_doppler_bins_step1_capacity, d_num_doppler_bins_step2) * d_magnitude_grid_stride),
      d_tmp_buffer(d_effective_fft_size),
      d_input_signal(d_fft_size),
      d_grid_doppler_wipeoffs_step_two(d_acq_parameters.make_2_steps ? d_num_doppler_bins_step2 * d_doppler_wipeoffs_stride : 0),
      d_ifft(gnss_fft_rev_make_unique(d_fft_size)),
      d_grid_doppler_wipeoffs(d_num_doppler_bins_step1_capacity * d_doppler_wipeoffs_stride),
      d_fft_codes(d_fft_size),
      d_data_buffer(d_consumed_samples),
      d_fft_if(gnss_fft_fwd_make_unique(d_fft_size))
{
    this->message_port_register_out(pmt::mp("events"));

    // d_fft_size = next power of two?  ////

    // COD:
    // Experimenting with the overlap/save technique for handling bit trannsitions
    // The problem: Circular correlation is asynchronous with the received code.
    // In effect the first code phase used in the correlation is the current
    // estimate of the code phase at the start of the input buffer. If this is 1/2
    // of the code period a bit transition would move all the signal energy into
    // adjacent frequency bands at +/- 1/T where T is the integration time.
    //
    // We can avoid this by doing linear correlation, effectively doubling the
    // size of the input buffer and padding the code with zeros.
    // if (d_acq_parameters.bit_transition_flag)
    // {
    //  d_fft_size = d_consumed_samples * 2;
    //  d_acq_parameters.max_dwells = 1;  // Activation of d_acq_parameters.bit_transition_flag invalidates the value of d_acq_parameters.max_dwells
    // }

    if (d_cshort)
        {
            d_data_buffer_sc = volk_gnsssdr::vector<lv_16sc_t>(d_consumed_samples);
        }

    std::fill(d_magnitude_grid.begin(), d_magnitude_grid.end(), 0.0F);

    update_grid_doppler_wipeoffs();
}


pcps_acquisition::~pcps_acquisition() noexcept
{
    try
        {
            wait_if_active();
        }
    catch (const std::exception& e)
        {
            LOG(WARNING) << "Exception while waiting for the acquisition worker in destructor: " << e.what();
        }
    catch (...)
        {
            LOG(WARNING) << "Unknown exception while waiting for the acquisition worker in destructor";
        }
}


void pcps_acquisition::set_active(bool active)
{
    {
        gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
        d_active = active;
    }

    if (!active)
        {
            wait_if_active();
        }
}


void pcps_acquisition::set_resampler_latency(uint32_t latency_samples)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    d_resampler_latency_samples = latency_samples;
}


void pcps_acquisition::set_local_code(std::complex<float>* code)
{
    // This will check if it's fdma, if yes will update the intermediate frequency and the doppler grid
    if (is_fdma())
        {
            update_grid_doppler_wipeoffs();
        }
    // COD
    // Here we want to create a buffer that looks like this:
    // [ 0 0 0 ... 0 c_0 c_1 ... c_L]
    // where c_i is the local code and there are L zeros and L chips
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    if (d_acq_parameters.bit_transition_flag)
        {
            const int32_t offset = d_fft_size / 2;
            std::fill_n(d_fft_if->get_inbuf(), offset, gr_complex(0.0, 0.0));
            std::copy(code, code + offset, d_fft_if->get_inbuf() + offset);
        }
    else
        {
            if (d_acq_parameters.sampled_ms == d_acq_parameters.ms_per_code)
                {
                    std::copy(code, code + d_consumed_samples, d_fft_if->get_inbuf());
                }
            else
                {
                    std::fill_n(d_fft_if->get_inbuf(), d_fft_size - d_consumed_samples, gr_complex(0.0, 0.0));
                    std::copy(code, code + d_consumed_samples, d_fft_if->get_inbuf() + d_consumed_samples);
                }
        }

    d_fft_if->execute();  // We need the FFT of local code
    volk_32fc_conjugate_32fc(d_fft_codes.data(), d_fft_if->get_outbuf(), d_fft_size);
}


bool pcps_acquisition::is_fdma()
{
    // reset the intermediate frequency
    d_doppler_bias = 0;

    // Dealing with FDMA system
    const auto is_1G = strcmp(d_gnss_synchro->Signal, "1G") == 0;
    const auto is_2G = strcmp(d_gnss_synchro->Signal, "2G") == 0;
    const auto is_glonass = is_1G || is_2G;

    if (is_glonass)
        {
            const auto freq = is_1G ? DFRQ1_GLO : DFRQ2_GLO;
            d_doppler_bias = static_cast<int32_t>(freq * GLONASS_PRN.at(d_gnss_synchro->PRN));
            DLOG(INFO) << "Trying to acquire SV PRN " << d_gnss_synchro->PRN << " with freq " << d_doppler_bias << " in Glonass Channel " << GLONASS_PRN.at(d_gnss_synchro->PRN) << '\n';
        }

    return is_glonass;
}


void pcps_acquisition::update_local_carrier(own::span<gr_complex> carrier_vector, float freq) const
{
    const auto fs_in = d_acq_parameters.use_automatic_resampler ? d_acq_parameters.resampled_fs : d_acq_parameters.fs_in;
    const auto phase_step_rad = static_cast<float>(TWO_PI) * freq / static_cast<float>(fs_in);
    std::array<float, 1> _phase{};
    volk_gnsssdr_s32f_sincos_32fc(carrier_vector.data(), -phase_step_rad, _phase.data(), carrier_vector.size());
}


void pcps_acquisition::update_grid_doppler_wipeoffs()
{
    if (d_doppler_search_narrowed)
        {
            // Assisted acquisition: the Doppler is exactly known from an already-tracked
            // primary frequency. Search just that bin, plus one reference bin offset by
            // the full configured Doppler half-width -- the same separation the full grid
            // search uses for its "opposite" bin -- so max_to_input_power_statistic can
            // keep using it as a noise-only reference without any change to its logic.
            update_local_carrier(own::span<gr_complex>(doppler_wipeoff_data(0), d_fft_size), static_cast<float>(d_doppler_bias + d_doppler_center));
            update_local_carrier(own::span<gr_complex>(doppler_wipeoff_data(1), d_fft_size), static_cast<float>(d_doppler_bias + d_doppler_center + static_cast<int32_t>(d_doppler_max)));
            return;
        }
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins_active; doppler_index++)
        {
            const int32_t doppler = -static_cast<int32_t>(d_doppler_max) + d_doppler_center + d_doppler_step * doppler_index;
            update_local_carrier(own::span<gr_complex>(doppler_wipeoff_data(doppler_index), d_fft_size), static_cast<float>(d_doppler_bias + doppler));
        }
}


void pcps_acquisition::update_grid_doppler_wipeoffs_step2()
{
    for (uint32_t doppler_index = 0; doppler_index < d_num_doppler_bins_step2; doppler_index++)
        {
            const float doppler = (static_cast<float>(doppler_index) - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0))) * d_acq_parameters.doppler_step2;
            // d_doppler_center_step_two holds the unbiased Doppler reported by the coarse
            // stage, so the FDMA frequency offset must be added back to the wipeoff
            update_local_carrier(own::span<gr_complex>(doppler_wipeoff_step_two_data(doppler_index), d_fft_size), static_cast<float>(d_doppler_bias) + d_doppler_center_step_two + doppler);
        }
}


void pcps_acquisition::log_acquisition(const AcquisitionResult& result) const
{
    DLOG(INFO) << (result.positive_acq ? "positive" : "negative") << " acquisition"
               << ", satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << ", sample_stamp " << result.sample_count
               << ", test statistics value " << result.test_statistics
               << ", test statistics threshold " << get_threshold()
               << ", code phase " << d_gnss_synchro->Acq_delay_samples
               << ", doppler " << static_cast<double>(result.doppler)
               << ", input signal power " << d_input_power
               << ", Assist doppler_center " << d_doppler_center;
}


void pcps_acquisition::send_positive_acquisition(const AcquisitionResult& result)
{
    log_acquisition(result);

    if (!d_channel_fsm.expired())
        {
            // the channel FSM is set, so, notify it directly the positive acquisition to minimize delays
            d_channel_fsm.lock()->Event_valid_acquisition();
        }
    else
        {
            // Declare positive acquisition using a message port
            // 0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
            this->message_port_pub(pmt::mp("events"), pmt::from_long(1));
        }
    LOG(INFO) << "Successful acquisition in channel " << d_channel
              << " for satellite " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN;

    // Copy and push current Gnss_Synchro to monitor queue
    if (d_acq_parameters.enable_monitor_output)
        {
            d_monitor_queue.push(*d_gnss_synchro);
        }
}


void pcps_acquisition::send_negative_acquisition(const AcquisitionResult& result)
{
    log_acquisition(result);

    // Declare negative acquisition using a message port
    // 0=STOP_CHANNEL 1=ACQ_SUCCEES 2=ACQ_FAIL
    this->message_port_pub(pmt::mp("events"), pmt::from_long(2));
}


void pcps_acquisition::dump_results(const AcquisitionResult& result)
{
    ensure_dump_grid_allocated();

    d_dump_number++;
    std::string filename = d_dump_filename;
    filename.append("_");
    filename.append(1, d_gnss_synchro->System);
    filename.append("_");
    filename.append(1, d_gnss_synchro->Signal[0]);
    filename.append(1, d_gnss_synchro->Signal[1]);
    filename.append("_ch_");
    filename.append(std::to_string(d_channel));
    filename.append("_");
    filename.append(std::to_string(d_dump_number));
    filename.append("_sat_");
    filename.append(std::to_string(d_gnss_synchro->PRN));
    filename.append(".mat");

    mat_t* matfp = Mat_CreateVer(filename.c_str(), nullptr, MAT_FT_MAT73);
    if (matfp == nullptr)
        {
            std::cout << "Unable to create or open Acquisition dump file\n";
        }
    else
        {
            std::array<size_t, 2> dims_1d{1, 1};
            std::array<size_t, 2> dims_2d{d_effective_fft_size, d_num_doppler_bins_active};

            // In narrowed mode, acq_grid is only d_num_doppler_bins_active (2) columns
            // wide -- column 0 is the known/aided Doppler, column 1 the noise-reference
            // bin at +doppler_max from it. doppler_max/doppler_step are given the same
            // {0, d_doppler_max} encoding compute_statistics() uses internally, so
            // doppler(col) = -doppler_max + doppler_center + doppler_step*col still
            // decodes both columns correctly; doppler_narrowed flags which encoding is
            // in effect for offline post-processing.
            const bool dump_narrowed = d_doppler_search_narrowed;
            const int32_t dump_doppler_max = dump_narrowed ? 0 : static_cast<int32_t>(d_doppler_max);
            const int32_t dump_doppler_step = dump_narrowed ? static_cast<int32_t>(d_doppler_max) : static_cast<int32_t>(d_doppler_step);

            write_matlab_var<2>("acq_grid", d_grid.memptr(), matfp, dims_2d);
            write_matlab_var<1>("doppler_max", dump_doppler_max, matfp, dims_1d);
            write_matlab_var<1>("doppler_step", dump_doppler_step, matfp, dims_1d);
            write_matlab_var<1>("doppler_center", d_doppler_center, matfp, dims_1d);
            write_matlab_var<1>("doppler_narrowed", static_cast<int32_t>(dump_narrowed ? 1 : 0), matfp, dims_1d);
            write_matlab_var<1>("positive_acq", static_cast<int32_t>(result.positive_acq ? 1 : 0), matfp, dims_1d);
            write_matlab_var<1>("acq_doppler_hz", static_cast<float>(d_gnss_synchro->Acq_doppler_hz), matfp, dims_1d);
            write_matlab_var<1>("acq_delay_samples", static_cast<float>(d_gnss_synchro->Acq_delay_samples), matfp, dims_1d);
            write_matlab_var<1>("test_statistic", result.test_statistics, matfp, dims_1d);
            write_matlab_var<1>("threshold", get_threshold(), matfp, dims_1d);
            write_matlab_var<1>("input_power", d_input_power, matfp, dims_1d);
            write_matlab_var<1>("sample_counter", result.sample_count, matfp, dims_1d);
            write_matlab_var<1>("PRN", d_gnss_synchro->PRN, matfp, dims_1d);
            write_matlab_var<1>("num_dwells", static_cast<int32_t>(d_num_noncoherent_integrations_counter), matfp, dims_1d);

            if (d_acq_parameters.make_2_steps)
                {
                    dims_2d = {d_effective_fft_size, d_num_doppler_bins_step2};

                    const float doppler_step_narrow = d_doppler_center_step_two - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0)) * d_acq_parameters.doppler_step2;
                    write_matlab_var<2>("acq_grid_narrow", d_narrow_grid.memptr(), matfp, dims_2d);
                    write_matlab_var<1>("doppler_step_narrow", d_acq_parameters.doppler_step2, matfp, dims_1d);
                    write_matlab_var<1>("doppler_grid_narrow_min", doppler_step_narrow, matfp, dims_1d);
                }

            Mat_Close(matfp);
        }
}


void pcps_acquisition::ensure_dump_grid_allocated()
{
    // Sized to the currently *active* bin count (2 when narrowed), not the full
    // configured d_num_doppler_bins -- matches what copy_magnitude_grid_to_dump_grid()
    // actually writes each cycle, and this size check doubles as the resize trigger
    // whenever narrowed/full mode changes between dumps (either direction).
    if ((d_grid.n_rows != d_effective_fft_size) || (d_grid.n_cols != d_num_doppler_bins_active))
        {
            d_grid.zeros(d_effective_fft_size, d_num_doppler_bins_active);
        }

    if (d_acq_parameters.make_2_steps &&
        ((d_narrow_grid.n_rows != d_effective_fft_size) || (d_narrow_grid.n_cols != d_num_doppler_bins_step2)))
        {
            d_narrow_grid.zeros(d_effective_fft_size, d_num_doppler_bins_step2);
        }
}


void pcps_acquisition::copy_magnitude_grid_to_dump_grid()
{
    ensure_dump_grid_allocated();

    const auto bin_count = d_step_two ? d_num_doppler_bins_step2 : d_num_doppler_bins_active;
    auto& grid = d_step_two ? d_narrow_grid : d_grid;

    for (uint32_t doppler_index = 0; doppler_index < bin_count; doppler_index++)
        {
            const auto* magnitude_grid = magnitude_grid_data(doppler_index);
            std::copy(magnitude_grid, magnitude_grid + d_effective_fft_size, grid.colptr(doppler_index));
        }
}


bool pcps_acquisition::should_dump_channel() const
{
    return d_dump && d_channel == d_dump_channel;
}


gr_complex* pcps_acquisition::doppler_wipeoff_data(uint32_t doppler_index)
{
    return d_grid_doppler_wipeoffs.data() + static_cast<size_t>(doppler_index) * d_doppler_wipeoffs_stride;
}


gr_complex* pcps_acquisition::doppler_wipeoff_step_two_data(uint32_t doppler_index)
{
    return d_grid_doppler_wipeoffs_step_two.data() + static_cast<size_t>(doppler_index) * d_doppler_wipeoffs_stride;
}


float* pcps_acquisition::magnitude_grid_data(uint32_t doppler_index)
{
    return d_magnitude_grid.data() + static_cast<size_t>(doppler_index) * d_magnitude_grid_stride;
}


const float* pcps_acquisition::magnitude_grid_data(uint32_t doppler_index) const
{
    return d_magnitude_grid.data() + static_cast<size_t>(doppler_index) * d_magnitude_grid_stride;
}


pcps_acquisition::AcquisitionResult pcps_acquisition::max_to_input_power_statistic(uint32_t num_doppler_bins, uint32_t candidate_count, int32_t doppler_max, int32_t doppler_step)
{
    AcquisitionResult result;
    float grid_maximum = 0.0;
    uint32_t index_doppler = 0U;
    uint32_t tmp_intex_t = 0U;

    // Find the correlation peak and the carrier frequency -- only among the
    // candidate rows (excludes any trailing noise-reference-only rows, e.g. the
    // narrowed-mode reference bin, from ever being selected as the result).
    for (uint32_t i = 0; i < candidate_count; i++)
        {
            const auto* magnitude_grid = magnitude_grid_data(i);
            volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, magnitude_grid, d_effective_fft_size);
            if (magnitude_grid[tmp_intex_t] > grid_maximum)
                {
                    grid_maximum = magnitude_grid[tmp_intex_t];
                    index_doppler = i;
                    result.index_time = tmp_intex_t;
                }
        }

    if (!d_step_two)
        {
            const auto index_opp = (index_doppler + num_doppler_bins / 2) % num_doppler_bins;
            const auto* magnitude_grid = magnitude_grid_data(index_opp);
            d_input_power = static_cast<float>(std::accumulate(magnitude_grid, magnitude_grid + d_effective_fft_size, static_cast<float>(0.0)) / d_effective_fft_size / 2.0 / d_num_noncoherent_integrations_counter);
            result.doppler = -static_cast<int32_t>(doppler_max) + d_doppler_center + doppler_step * static_cast<int32_t>(index_doppler);
        }
    else
        {
            result.doppler = static_cast<int32_t>(d_doppler_center_step_two + (static_cast<float>(index_doppler) - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0))) * d_acq_parameters.doppler_step2);
        }

    if (d_input_power < std::numeric_limits<float>::epsilon())
        {
            result.test_statistics = 0;
        }
    else
        {
            result.test_statistics = grid_maximum / d_input_power;
        }

    return result;
}


pcps_acquisition::AcquisitionResult pcps_acquisition::first_vs_second_peak_statistic(uint32_t candidate_count, int32_t doppler_max, int32_t doppler_step)
{
    // Look for correlation peaks in the results
    // Find the highest peak and compare it to the second highest peak
    // The second peak is chosen not closer than 1 chip to the highest peak

    AcquisitionResult result;
    float firstPeak = 0.0;
    uint32_t index_doppler = 0U;
    uint32_t tmp_intex_t = 0U;

    // Find the correlation peak and the carrier frequency -- only among the
    // candidate rows (excludes any trailing noise-reference-only rows, e.g. the
    // narrowed-mode reference bin, from ever being selected as the result).
    for (uint32_t i = 0; i < candidate_count; i++)
        {
            const auto* magnitude_grid = magnitude_grid_data(i);
            volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, magnitude_grid, d_effective_fft_size);
            if (magnitude_grid[tmp_intex_t] > firstPeak)
                {
                    firstPeak = magnitude_grid[tmp_intex_t];
                    index_doppler = i;
                    result.index_time = tmp_intex_t;
                }
        }

    if (!d_step_two)
        {
            result.doppler = -static_cast<int32_t>(doppler_max) + d_doppler_center + doppler_step * static_cast<int32_t>(index_doppler);
        }
    else
        {
            result.doppler = static_cast<int32_t>(d_doppler_center_step_two + (static_cast<float>(index_doppler) - static_cast<float>(floor(d_num_doppler_bins_step2 / 2.0))) * d_acq_parameters.doppler_step2);
        }

    // Find 1 chip wide code phase exclude range around the peak
    int32_t excludeRangeIndex1 = result.index_time - d_samplesPerChip;
    int32_t excludeRangeIndex2 = result.index_time + d_samplesPerChip;

    // Correct code phase exclude range if the range includes array boundaries
    if (excludeRangeIndex1 < 0)
        {
            excludeRangeIndex1 = d_effective_fft_size + excludeRangeIndex1;
        }
    else if (excludeRangeIndex2 >= static_cast<int32_t>(d_effective_fft_size))
        {
            excludeRangeIndex2 = excludeRangeIndex2 - d_effective_fft_size;
        }

    int32_t idx = excludeRangeIndex1;
    const auto* magnitude_grid = magnitude_grid_data(index_doppler);
    std::copy(magnitude_grid, magnitude_grid + d_effective_fft_size, d_tmp_buffer.data());
    do
        {
            d_tmp_buffer[idx] = 0.0;
            idx++;
            if (idx == static_cast<int32_t>(d_effective_fft_size))
                {
                    idx = 0;
                }
        }
    while (idx != excludeRangeIndex2);

    // Find the second highest correlation peak in the same freq. bin ---
    volk_gnsssdr_32f_index_max_32u(&tmp_intex_t, d_tmp_buffer.data(), d_effective_fft_size);
    const float secondPeak = d_tmp_buffer[tmp_intex_t];

    // Compute the test statistics and compare to the threshold
    result.test_statistics = firstPeak / secondPeak;

    return result;
}


void pcps_acquisition::doppler_grid(const gr_complex* in)
{
    const auto bin_count = d_step_two ? d_num_doppler_bins_step2 : d_num_doppler_bins_active;
    const auto* grid_doppler_wipeoffs = d_step_two ? d_grid_doppler_wipeoffs_step_two.data() : d_grid_doppler_wipeoffs.data();

    for (uint32_t doppler_index = 0; doppler_index < bin_count; doppler_index++)
        {
            auto* magnitude_grid = magnitude_grid_data(doppler_index);
            const auto* doppler_wipeoff = grid_doppler_wipeoffs + static_cast<size_t>(doppler_index) * d_doppler_wipeoffs_stride;

            // Remove Doppler
            volk_32fc_x2_multiply_32fc(d_fft_if->get_inbuf(), in, doppler_wipeoff, d_fft_size);

            // Perform the FFT-based convolution  (parallel time search)
            // Compute the FFT of the carrier wiped--off incoming signal
            d_fft_if->execute();

            // Multiply carrier wiped--off, Fourier transformed incoming signal with the local FFT'd code reference
            volk_32fc_x2_multiply_32fc(d_ifft->get_inbuf(), d_fft_if->get_outbuf(), d_fft_codes.data(), d_fft_size);

            // Compute the inverse FFT
            d_ifft->execute();

            // Compute squared magnitude (and accumulate in case of non-coherent integration)
            const size_t offset = (d_acq_parameters.bit_transition_flag ? d_effective_fft_size : 0);
            if (d_num_noncoherent_integrations_counter == 1)
                {
                    volk_32fc_magnitude_squared_32f(magnitude_grid, d_ifft->get_outbuf() + offset, d_effective_fft_size);
                }
            else
                {
                    volk_32fc_magnitude_squared_32f(d_tmp_buffer.data(), d_ifft->get_outbuf() + offset, d_effective_fft_size);
                    volk_32f_x2_add_32f(magnitude_grid, magnitude_grid, d_tmp_buffer.data(), d_effective_fft_size);
                }
        }
}


pcps_acquisition::AcquisitionResult pcps_acquisition::compute_statistics()
{
    const auto bin_count = d_step_two ? d_num_doppler_bins_step2 : d_num_doppler_bins_active;
    // Assisted acquisition (Doppler exactly known from an already-tracked primary
    // frequency): bin 0 is the known Doppler and bin 1 is the reference bin offset
    // by d_doppler_max (see update_grid_doppler_wipeoffs()). Passing doppler_max=0,
    // doppler_step=d_doppler_max makes the generic "-doppler_max + center +
    // doppler_step * index" decoding below map those two bins back to the right
    // Doppler values, same formula as the full grid search uses.
    const bool assisted = (!d_step_two) && d_doppler_search_narrowed;
    const auto doppler_step = d_step_two ? d_acq_parameters.doppler_step2 : (assisted ? static_cast<int32_t>(d_doppler_max) : d_doppler_step);
    const auto doppler_max = d_step_two ? static_cast<int32_t>(d_doppler_center_step_two - (static_cast<float>(bin_count) / 2.0) * doppler_step) : (assisted ? 0 : static_cast<int32_t>(d_doppler_max));
    // In assisted mode, bin_count computed rows exist (known bin + noise reference)
    // but only bin 0 is a real Doppler candidate -- the reference bin must never be
    // selected as the acquisition result (see the two statistic functions).
    const auto candidate_count = assisted ? 1U : bin_count;

    // The reference/"opposite" bin used by max_to_input_power_statistic as a
    // noise-only estimate works the same way whether the grid has the full
    // configured bin count or just the 2 bins built above for assisted mode.
    if (d_use_CFAR_algorithm_flag)
        {
            return max_to_input_power_statistic(bin_count, candidate_count, doppler_max, doppler_step);
        }
    else
        {
            return first_vs_second_peak_statistic(candidate_count, doppler_max, doppler_step);
        }
}


void pcps_acquisition::update_synchro(const AcquisitionResult& result)
{
    d_gnss_synchro->Acq_delay_samples = static_cast<double>(std::fmod(static_cast<float>(result.index_time), d_acq_parameters.samples_per_code));
    d_gnss_synchro->Acq_doppler_hz = static_cast<double>(result.doppler);

    if (d_acq_parameters.use_automatic_resampler)
        {
            // take into account the acquisition resampler ratio
            d_gnss_synchro->Acq_delay_samples = (d_gnss_synchro->Acq_delay_samples * d_acq_parameters.resampler_ratio) - static_cast<double>(d_resampler_latency_samples);
            d_gnss_synchro->Acq_samplestamp_samples = rint(static_cast<double>(result.sample_count) * d_acq_parameters.resampler_ratio);
            d_gnss_synchro->fs = d_acq_parameters.resampled_fs;
        }
    else
        {
            d_gnss_synchro->Acq_samplestamp_samples = result.sample_count;
            d_gnss_synchro->fs = d_acq_parameters.fs_in;
        }

    if (d_step_two)
        {
            d_gnss_synchro->Acq_doppler_step = d_acq_parameters.doppler_step2;
        }
}


void pcps_acquisition::handle_threshold_reached(AcquisitionResult& result)
{
    d_state = 0;

    if (d_acq_parameters.make_2_steps)
        {
            if (d_step_two)
                {
                    send_positive_acquisition(result);
                    result.positive_acq = true;
                    d_active = false;
                }
            else
                {
                    d_doppler_center_step_two = static_cast<float>(result.doppler);
                    update_grid_doppler_wipeoffs_step2();
                    d_num_noncoherent_integrations_counter = 0;
                }

            d_step_two = !d_step_two;
        }
    else
        {
            send_positive_acquisition(result);
            result.positive_acq = true;
            d_active = false;
        }
}


void pcps_acquisition::handle_integration_done(const AcquisitionResult& result)
{
    if (d_state != 0)
        {
            send_negative_acquisition(result);
        }

    d_active = false;
    d_state = 0;
    d_step_two = false;
}


void pcps_acquisition::acquisition_core(uint64_t sample_count)
{
    gr::thread::scoped_lock lk(d_setlock);

    // Initialize acquisition algorithm
    const gr_complex* in = nullptr;  // Get the input samples pointer
    if (d_cshort)
        {
            volk_gnsssdr_16ic_convert_32fc(d_input_signal.data(), d_data_buffer_sc.data(), d_consumed_samples);
            if (d_fft_size > d_consumed_samples)
                {
                    std::fill_n(d_input_signal.data() + d_consumed_samples, d_fft_size - d_consumed_samples, gr_complex(0.0, 0.0));
                }
            in = d_input_signal.data();
        }
    else if (d_fft_size == d_consumed_samples)
        {
            in = d_data_buffer.data();
        }
    else
        {
            std::copy(d_data_buffer.data(), d_data_buffer.data() + d_consumed_samples, d_input_signal.data());
            std::fill_n(d_input_signal.data() + d_consumed_samples, d_fft_size - d_consumed_samples, gr_complex(0.0, 0.0));
            in = d_input_signal.data();
        }

    d_num_noncoherent_integrations_counter++;

    DLOG(INFO) << "Channel: " << d_channel
               << " , doing acquisition of satellite: " << d_gnss_synchro->System << " " << d_gnss_synchro->PRN
               << " , sample stamp: " << sample_count
               << ", threshold: " << get_threshold()
               << ", doppler_max: " << d_doppler_max
               << ", doppler_step: " << d_doppler_step
               << ", use_CFAR_algorithm_flag: " << (d_use_CFAR_algorithm_flag ? "true" : "false");

    lk.unlock();

    // Doppler frequency grid loop, only access variables that doesn't need a lock
    doppler_grid(in);
    if (should_dump_channel())
        {
            copy_magnitude_grid_to_dump_grid();
        }
    auto result = compute_statistics();
    result.sample_count = sample_count;

    lk.lock();

    update_synchro(result);

    if (!d_acq_parameters.bit_transition_flag)
        {
            if (result.test_statistics > get_threshold())
                {
                    handle_threshold_reached(result);
                }
            else
                {
                    d_buffer_count = 0;
                    d_state = 1;
                }

            if (d_num_noncoherent_integrations_counter == d_acq_parameters.max_dwells)
                {
                    handle_integration_done(result);
                }
        }
    else
        {
            if (result.test_statistics > get_threshold())
                {
                    handle_threshold_reached(result);
                }
            else
                {
                    handle_integration_done(result);
                }
        }

    if ((d_num_noncoherent_integrations_counter == d_acq_parameters.max_dwells) || (result.positive_acq) || (d_acq_parameters.bit_transition_flag))
        {
            // Record results to file if required
            if (should_dump_channel())
                {
                    pcps_acquisition::dump_results(result);
                }
            d_num_noncoherent_integrations_counter = 0U;
        }

    d_worker_active = false;
}


float pcps_acquisition::get_threshold() const
{
    if (d_step_two)
        {
            return d_threshold_step_two;
        }
    return d_doppler_search_narrowed ? d_threshold_narrowed : d_threshold;
}


void pcps_acquisition::set_doppler_center(int32_t doppler_center)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    if (doppler_center != d_doppler_center)
        {
            DLOG(INFO) << " Doppler assistance for Channel: " << d_channel << " => Doppler: " << doppler_center << "[Hz]";
            d_doppler_center = doppler_center;
            update_grid_doppler_wipeoffs();
        }
}


void pcps_acquisition::set_doppler_uncertainty(uint32_t doppler_uncertainty)
{
    gr::thread::scoped_lock lock(d_setlock);  // require mutex with work function called by the scheduler
    const bool narrow = (doppler_uncertainty == 0) && d_acq_parameters.enable_doppler_narrowing;
    const uint32_t num_doppler_bins_active = narrow ? 2U : d_num_doppler_bins;
    if (narrow != d_doppler_search_narrowed || num_doppler_bins_active != d_num_doppler_bins_active)
        {
            d_doppler_search_narrowed = narrow;
            d_num_doppler_bins_active = num_doppler_bins_active;
            update_grid_doppler_wipeoffs();
            DLOG(INFO) << " Doppler uncertainty for Channel: " << d_channel << " => active Doppler bins: " << d_num_doppler_bins_active << ", Doppler center: " << d_doppler_center;
        }
}


void pcps_acquisition::wait_if_active()
{
    std::unique_ptr<gr::thread::thread> worker;

    {
        gr::thread::scoped_lock lk(d_setlock);
        worker = std::move(d_worker);
        d_worker = nullptr;
    }

    if (worker != nullptr)
        {
            if (worker->joinable())
                {
                    worker->join();
                }
        }
}


int pcps_acquisition::general_work(int noutput_items __attribute__((unused)),
    gr_vector_int& ninput_items,
    gr_vector_const_void_star& input_items,
    gr_vector_void_star& output_items)
{
    /*
     * By J.Arribas, L.Esteve and M.Molina
     * Acquisition strategy (Kay Borre book + CFAR threshold):
     * 1. Compute the input signal power estimation
     * 2. Doppler serial search loop
     * 3. Perform the FFT-based circular convolution (parallel time search)
     * 4. Record the maximum peak and the associated synchronization parameters
     * 5. Compute the test statistics and compare to the threshold
     * 6. Declare positive or negative acquisition using a message port
     */
    gr::thread::scoped_lock lk(d_setlock);
    if (!d_active || d_worker_active)
        {
            // do not consume samples while performing a non-coherent integration
            const bool consume_samples = ((!d_active) || (d_worker_active && (d_num_noncoherent_integrations_counter == d_acq_parameters.max_dwells)));
            if ((!d_acq_parameters.blocking_on_standby) && consume_samples)
                {
                    d_sample_count += static_cast<uint64_t>(ninput_items[0]);
                    consume_each(ninput_items[0]);
                }
            return 0;
        }

    switch (d_state)
        {
        case 0:
            {
                // Restart acquisition variables
                d_gnss_synchro->Acq_delay_samples = 0.0;
                d_gnss_synchro->Acq_doppler_hz = 0.0;
                d_gnss_synchro->Acq_samplestamp_samples = 0ULL;
                d_gnss_synchro->Acq_doppler_step = 0U;
                d_state = 1;
                d_buffer_count = 0U;
                break;
            }
        case 1:
            {
                const auto fit_in_buffer = (ninput_items[0] + d_buffer_count) <= d_consumed_samples;
                const uint32_t buff_increment = fit_in_buffer ? ninput_items[0] : d_consumed_samples - d_buffer_count;

                if (d_cshort)
                    {
                        const auto* in = reinterpret_cast<const lv_16sc_t*>(input_items[0]);  // Get the input samples pointer
                        std::copy(in, in + buff_increment, d_data_buffer_sc.begin() + d_buffer_count);
                    }
                else
                    {
                        const auto* in = reinterpret_cast<const gr_complex*>(input_items[0]);  // Get the input samples pointer
                        std::copy(in, in + buff_increment, d_data_buffer.begin() + d_buffer_count);
                    }

                // If buffer will be full in next iteration
                if (d_buffer_count >= d_consumed_samples)
                    {
                        d_state = 2;
                    }
                d_buffer_count += buff_increment;
                d_sample_count += static_cast<uint64_t>(buff_increment);
                consume_each(buff_increment);
                break;
            }
        case 2:
            {
                // Copy the data to the core and let it know that new data is available
                if (d_acq_parameters.blocking)
                    {
                        lk.unlock();
                        acquisition_core(d_sample_count);
                    }
                else
                    {
                        lk.unlock();
                        wait_if_active();
                        lk.lock();
                        d_worker = std::make_unique<gr::thread::thread>(&pcps_acquisition::acquisition_core, this, d_sample_count);
                        d_worker_active = true;
                    }
                consume_each(0);
                d_buffer_count = 0U;
                break;
            }
        }

    // Send outputs to the monitor
    if (d_acq_parameters.enable_monitor_output && !d_monitor_queue.empty())
        {
            auto** out = reinterpret_cast<Gnss_Synchro**>(&output_items[0]);
            const int num_gnss_synchro_objects = d_monitor_queue.size();
            for (int i = 0; i < num_gnss_synchro_objects; ++i)
                {
                    Gnss_Synchro current_synchro_data = d_monitor_queue.front();
                    d_monitor_queue.pop();
                    *out[i] = std::move(current_synchro_data);
                }
            return num_gnss_synchro_objects;
        }

    return 0;
}
