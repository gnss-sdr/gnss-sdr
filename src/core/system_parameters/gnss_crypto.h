/*!
 * \file gnss_crypto.h
 * \brief Class for computing cryptografic functions
 * \author Carles Fernandez, 2023. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2023  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GNSS_CRYPTO_H
#define GNSS_SDR_GNSS_CRYPTO_H

#include <cstdint>
#include <string>
#include <vector>


/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */


class Gnss_Crypto
{
public:
    Gnss_Crypto() = default;
    explicit Gnss_Crypto(const std::string& filePath);
    bool have_public_key() const;
    std::vector<uint8_t> computeSHA256(const std::vector<uint8_t>& input) const;
    std::vector<uint8_t> computeSHA3_256(const std::vector<uint8_t>& input) const;
    std::vector<uint8_t> computeHMAC_SHA_256(const std::vector<uint8_t>& key, const std::vector<uint8_t>& input) const;
    std::vector<uint8_t> computeCMAC_AES(const std::vector<uint8_t>& key, const std::vector<uint8_t>& input) const;
    void set_public_key(const std::vector<uint8_t>& publickey);
    void readPublicKeyFromPEM(const std::string& filePath);

private:
    std::vector<uint8_t> base64Decode(const std::string& encoded_string);
    std::vector<uint8_t> d_PublicKey;
};

/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_CRYPTO_H