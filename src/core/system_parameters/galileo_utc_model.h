/*!
 * \file galileo_utc_model.h
 * \brief  Interface of a Galileo UTC MODEL storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
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


#ifndef GNSS_SDR_GALILEO_UTC_MODEL_H
#define GNSS_SDR_GALILEO_UTC_MODEL_H

#include <boost/serialization/nvp.hpp>
#include <cstdint>

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the GALILEO UTC MODEL data as described in Galileo ICD
 * https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo-OS-SIS-ICD.pdf
 * paragraph 5.1.7
 */
class Galileo_Utc_Model
{
public:
    /*!
     * Default constructor
     */
    Galileo_Utc_Model() = default;

    // double TOW_6;
    double GST_to_UTC_time(double t_e, int32_t WN) const;  //!< GST-UTC Conversion Algorithm and Parameters

    // Word type 6: GST-UTC conversion parameters
    double A0_6{};
    double A1_6{};
    int32_t Delta_tLS_6{};
    int32_t t0t_6{};   //!< UTC data reference Time of Week [s]
    int32_t WNot_6{};  //!< UTC data reference Week number [week]
    int32_t WN_LSF_6{};
    int32_t DN_6{};
    int32_t Delta_tLSF_6{};

    // GPS to Galileo GST conversion parameters
    double A_0G_10{};
    double A_1G_10{};
    int32_t t_0G_10{};
    int32_t WN_0G_10{};

    bool flag_utc_model{};

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization.
     Here is used to save the UTC data on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };
        archive& make_nvp("A0_6", A0_6);
        archive& make_nvp("A1_6", A1_6);
        archive& make_nvp("Delta_tLS_6", Delta_tLS_6);
        archive& make_nvp("t0t_6", t0t_6);
        archive& make_nvp("WNot_6", WNot_6);
        archive& make_nvp("WN_LSF_6", WN_LSF_6);
        archive& make_nvp("DN_6", DN_6);
        archive& make_nvp("Delta_tLSF_6", Delta_tLSF_6);
        archive& make_nvp("flag_utc_model", flag_utc_model);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_UTC_MODEL_H
