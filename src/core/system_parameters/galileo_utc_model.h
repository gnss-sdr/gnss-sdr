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
 * https://www.gsc-europa.eu/sites/default/files/sites/all/files/Galileo_OS_SIS_ICD_v2.0.pdf
 * paragraph 5.1.7
 */
class Galileo_Utc_Model
{
public:
    /*!
     * Default constructor
     */
    Galileo_Utc_Model() = default;

    /**
     * @brief Initialize the Galileo_UTC_Model with the provided data.
     *
     * This function initializes the Galileo_UTC_Model object with the given data.
     *
     * @param data The Galileo_UTC_Model object containing the data.
     */
    static void init(const Galileo_Utc_Model& data);

    // double TOW;
    static double GST_to_UTC_time(double t_e, int32_t WN);  //!< GST-UTC Conversion Algorithm and Parameters
    double UTC_time_to_GST(double t_Utc, int32_t WN) const;

    // TODO - make them private?
    // Word type 6: GST-UTC conversion parameters
    static double A0;
    static double A1;
    static int32_t Delta_tLS;
    static int32_t tot;   //!< UTC data reference Time of Week [s]
    static int32_t WNot;  //!< UTC data reference Week number [week]
    static int32_t WN_LSF;
    static int32_t DN;
    static int32_t Delta_tLSF;

    // GPS to Galileo GST conversion parameters
    double A_0G{};
    double A_1G{};
    int32_t t_0G{};
    int32_t WN_0G{};

    bool flag_utc_model{};

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization.
     Here is used to save the UTC data on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        if (version)
            {
            };
        archive& BOOST_SERIALIZATION_NVP(A0);
        archive& BOOST_SERIALIZATION_NVP(A1);
        archive& BOOST_SERIALIZATION_NVP(Delta_tLS);
        archive& BOOST_SERIALIZATION_NVP(tot);
        archive& BOOST_SERIALIZATION_NVP(WNot);
        archive& BOOST_SERIALIZATION_NVP(WN_LSF);
        archive& BOOST_SERIALIZATION_NVP(DN);
        archive& BOOST_SERIALIZATION_NVP(Delta_tLSF);
        archive& BOOST_SERIALIZATION_NVP(flag_utc_model);
    }
};


/** \} */
/** \} */
#endif  // GNSS_SDR_GALILEO_UTC_MODEL_H
