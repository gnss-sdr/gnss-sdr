/*!
 * \file hybrid_observables.h
 * \brief Implementation of an adapter of an observables block accepting all kind
 * of signals to a ObservablesInterface
 * \author Mara Branzanti 2013. mara.branzanti(at)gmail.com
 * \author Javier Arribas 2013. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_HYBRID_OBSERVABLES_H
#define GNSS_SDR_HYBRID_OBSERVABLES_H

#include "gnss_synchro.h"
#include "hybrid_observables_gs.h"
#include "observables_interface.h"
#include <gnuradio/gr_complex.h>     // for gr_complex
#include <gnuradio/runtime_types.h>  // for basic_block_sptr, top_block_sptr
#include <cstddef>
#include <string>

/** \addtogroup Observables
 * Classes for the computation of GNSS observables
 * \{ */
/** \addtogroup Observables_adapters obs_adapters
 * Wrap GNU Radio observables blocks with an ObservablesInterface
 * \{ */

class ConfigurationInterface;

/*!
 * \brief This class implements an ObservablesInterface for observables of all kind of GNSS signals
 */
class HybridObservables : public ObservablesInterface
{
public:
    HybridObservables(const ConfigurationInterface* configuration,
        const std::string& role,
        unsigned int in_streams,
        unsigned int out_streams);

    ~HybridObservables() = default;

    inline std::string role() override
    {
        return role_;
    }

    //!  Returns "Hybrid_Observables"
    inline std::string implementation() override
    {
        return "Hybrid_Observables";
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    inline void reset() override
    {
        return;
    }

    //! All blocks must have an item_size() function implementation
    inline size_t item_size() override
    {
        return sizeof(Gnss_Synchro);
    }

private:
    hybrid_observables_gs_sptr observables_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    bool dump_;
    bool dump_mat_;
};

/** \} */
/** \} */
#endif
