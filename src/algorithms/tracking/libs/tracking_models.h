/*!
 * \file tracking_models.h
 * \brief Implementation of linear and nonlinear tracking models.
 * \author Gerald LaMountain, 2019. gerald(at)ece.neu.edu
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_TRACKING_MODELS_H_
#define GNSS_SDR_TRACKING_MODELS_H_

#include "cpu_multicorrelator_real_codes.h"
#include "dll_pll_conf.h"
#include "exponential_smoother.h"
#include "tracking_FLL_PLL_filter.h"  // for PLL/FLL filter
#include "tracking_loop_filter.h"     // for DLL filter
#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>   // for boost::shared_ptr
#include <gnuradio/block.h>       // for block
#include <gnuradio/gr_complex.h>  // for gr_complex
#include <gnuradio/types.h>       // for gr_vector_int, gr_vector...
#include <pmt/pmt.h>
#include <cstdint>                // for int32_t
#include <fstream>                // for string, ofstream
#include <utility>                // for pair
#include <vector>

// Abstract model function
template <class OutputType>
class ModelFunction
{
public:
    ModelFunction(){};
    virtual OutputType operator()(const arma::vec& input) = 0;
    virtual ~ModelFunction() = default;
};

/*
class CxModelFunction
{
public:
    CxModelFunction(){};
    virtual arma::cx_vec operator()(const arma::cx_vec& input) = 0;
    virtual ~CxModelFunction() = default;
};
class CarrierTransitionModel : public ModelFunction
{
public:
    explicit CarrierTransitionModel(const float carrier_pdi) { pdi = carrier_pdi; };
    arma::vec operator()(const arma::vec& input) override { 
        arma::vec output = arma::zeros(3,1);
        output(0, 0) = input(0) + PI_2*pdi*input(1) + 0.5*PI_2*std::pow(pdi, 2)*input(2);
        output(0, 0) = input(1) + pdi*input(2);
        output(0, 0) = input(2);

        return output;
    };

private:
    arma::mat t_mat;
    float pdi;
};

class CarrierMeasurementModel : public ModelFunction
{
public:
    explicit CarrierMeasurementModel(const float carrier_pow) { alpha = carrier_pow; };
    arma::vec operator()(const arma::vec& input) override {
        using namespace std::complex_literals;
        std::complex<double> output_iq = static_cast<double>(alpha) * std::exp( 1i * static_cast<double>(input(0)) );

        arma::vec output = arma::zeros(2,1);
        output(0, 0) = static_cast<float>( std::real( output_iq ) );
        output(1, 0) = static_cast<float>( std::imag( output_iq ) );

        return output;
    };

private:
    float alpha;
};
*/
#endif
