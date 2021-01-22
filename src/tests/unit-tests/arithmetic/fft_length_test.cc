/*!
 * \file fft_length_test.cc
 * \brief  This file implements timing tests for the FFT.
 * \author Carles Fernandez-Prades, 2016. cfernandez(at)cttc.es
 *
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

#include "gnss_sdr_fft.h"
#include "gnss_sdr_filesystem.h"
#include "gnuplot_i.h"
#include "test_flags.h"
#include <algorithm>
#include <chrono>
#include <functional>
#include <random>


DEFINE_int32(fft_iterations_test, 1000, "Number of averaged iterations in FFT length timing test");
DEFINE_bool(plot_fft_length_test, false, "Plots results of FFTLengthTest with gnuplot");

// Note from FFTW documentation: the standard FFTW distribution works most efficiently for arrays whose
// size can be factored into small primes (2, 3, 5, and 7), and otherwise it uses a slower general-purpose routine.

TEST(FFTLengthTest, MeasureExecutionTime)
{
    unsigned int fft_sizes[] = {512, 1000, 1024, 1100, 1297, 1400, 1500, 1960, 2000, 2048, 2221, 2500, 3000, 3500, 4000,
        4096, 4200, 4500, 4725, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8192, 8500, 9000, 9500, 10000, 10368, 11000,
        12000, 15000, 16000, 16384, 27000, 32768, 50000, 65536};

    std::chrono::time_point<std::chrono::system_clock> start, end;

    std::random_device r;
    std::default_random_engine e1(r());
    std::default_random_engine e2(r());
    std::uniform_real_distribution<float> uniform_dist(-1, 1);
    auto func = [](float a, float b) { return gr_complex(a, b); };  // Helper lambda function that returns a gr_complex
    auto random_number1 = std::bind(uniform_dist, e1);
    auto random_number2 = std::bind(uniform_dist, e2);
    auto gen = std::bind(func, random_number1, random_number2);  // Function that returns a random gr_complex

    std::vector<unsigned int> fft_sizes_v(fft_sizes, fft_sizes + sizeof(fft_sizes) / sizeof(unsigned int));
    std::sort(fft_sizes_v.begin(), fft_sizes_v.end());
    std::vector<unsigned int>::const_iterator it;
    unsigned int d_fft_size;
    std::vector<double> execution_times;
    std::vector<unsigned int> powers_of_two;
    std::vector<double> execution_times_powers_of_two;

    EXPECT_NO_THROW(
        for (it = fft_sizes_v.cbegin(); it != fft_sizes_v.cend(); ++it) {
            d_fft_size = *it;
            auto d_fft = gnss_fft_fwd_make_unique(d_fft_size);
            std::generate_n(d_fft->get_inbuf(), d_fft_size, gen);

            start = std::chrono::system_clock::now();
            for (int k = 0; k < FLAGS_fft_iterations_test; k++)
                {
                    d_fft->execute();
                }
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            double exec_time = elapsed_seconds.count() / static_cast<double>(FLAGS_fft_iterations_test);
            execution_times.push_back(exec_time * 1e3);
            std::cout << "FFT execution time for length=" << d_fft_size << " : " << exec_time << " [s]\n";

            if ((d_fft_size & (d_fft_size - 1)) == 0)  // if it is a power of two
                {
                    powers_of_two.push_back(d_fft_size);
                    execution_times_powers_of_two.push_back(exec_time / 1e-3);
                }
        });

    if (FLAGS_plot_fft_length_test == true)
        {
            const std::string gnuplot_executable(FLAGS_gnuplot_executable);
            if (gnuplot_executable.empty())
                {
                    std::cout << "WARNING: Although the flag plot_fft_length_test has been set to TRUE,\n";
                    std::cout << "gnuplot has not been found in your system.\n";
                    std::cout << "Test results will not be plotted.\n";
                }
            else
                {
                    try
                        {
                            fs::path p(gnuplot_executable);
                            fs::path dir = p.parent_path();
                            const std::string& gnuplot_path = dir.native();
                            Gnuplot::set_GNUPlotPath(gnuplot_path);

                            Gnuplot g1("linespoints");
                            if (FLAGS_show_plots)
                                {
                                    g1.showonscreen();  // window output
                                }
                            else
                                {
                                    g1.disablescreen();
                                }
                            g1.set_title("FFT execution times for different lengths");
                            g1.set_grid();
                            g1.set_xlabel("FFT length");
                            g1.set_ylabel("Execution time [ms]");
                            g1.plot_xy(fft_sizes_v, execution_times, "FFT execution time (averaged over " + std::to_string(FLAGS_fft_iterations_test) + " iterations)");
                            g1.set_style("points").plot_xy(powers_of_two, execution_times_powers_of_two, "Power of 2");
                            g1.savetops("FFT_execution_times_extended");
                            g1.savetopdf("FFT_execution_times_extended", 18);

                            Gnuplot g2("linespoints");
                            if (FLAGS_show_plots)
                                {
                                    g2.showonscreen();  // window output
                                }
                            else
                                {
                                    g2.disablescreen();
                                }
                            g2.set_title("FFT execution times for different lengths (up to 2^{14}=16384)");
                            g2.set_grid();
                            g2.set_xlabel("FFT length");
                            g2.set_ylabel("Execution time [ms]");
                            g2.set_xrange(0, 16384);
                            g2.plot_xy(fft_sizes_v, execution_times, "FFT execution time (averaged over " + std::to_string(FLAGS_fft_iterations_test) + " iterations)");
                            g2.set_style("points").plot_xy(powers_of_two, execution_times_powers_of_two, "Power of 2");
                            g2.savetops("FFT_execution_times");
                            g2.savetopdf("FFT_execution_times", 18);
                            if (FLAGS_show_plots)
                                {
                                    g2.showonscreen();  // window output
                                }
                        }
                    catch (const GnuplotException& ge)
                        {
                            std::cout << ge.what() << '\n';
                        }
                }
        }
}
