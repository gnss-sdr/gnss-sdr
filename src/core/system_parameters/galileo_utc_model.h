/*!
 * \file galileo_utc_model.h
 * \brief  Interface of a Galileo UTC MODEL storage
 * \author Javier Arribas, 2013. jarribas(at)cttc.es
 *
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


#ifndef GNSS_SDR_GALILEO_UTC_MODEL_H_
#define GNSS_SDR_GALILEO_UTC_MODEL_H_

#include <boost/serialization/nvp.hpp>

/*!
 * \brief This class is a storage for the GALILEO UTC MODEL data as described in Galileo ICD
 * https://www.gsc-europa.eu/system/files/galileo_documents/Galileo_OS_SIS_ICD.pdf
 * paragraph 5.1.7
 */
class Galileo_Utc_Model
{
public:
    // Word type 6: GST-UTC conversion parameters
    double A0_6;
    double A1_6;
    double Delta_tLS_6;
    double t0t_6;   //!< UTC data reference Time of Week [s]
    double WNot_6;  //!< UTC data reference Week number [week]
    double WN_LSF_6;
    double DN_6;
    double Delta_tLSF_6;
    bool flag_utc_model;
    //double TOW_6;
    double GST_to_UTC_time(double t_e, int WN);  //!< GST-UTC Conversion Algorithm and Parameters
    /*!
     * Default constructor
     */
    Galileo_Utc_Model();

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
    }
};

#endif
