/*!
 * \file beidou_dnav_iono.h
 * \brief  Interface of a BEIDOU IONOSPHERIC MODEL storage
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
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


#ifndef GNSS_SDR_BEIDOU_DNAV_IONO_H
#define GNSS_SDR_BEIDOU_DNAV_IONO_H

#include "gps_iono.h"

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


/*!
 * \brief This class is a storage for the BEIDOU IONOSPHERIC data as described
 * in ICD v2.1
 */
class Beidou_Dnav_Iono : public Gps_Iono
{
public:
    Beidou_Dnav_Iono() = default;  //!< Default constructor
};


/** \} */
/** \} */
#endif  // GNSS_SDR_BEIDOU_DNAV_IONO_H
