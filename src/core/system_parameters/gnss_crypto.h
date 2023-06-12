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
#if USE_OPENSSL_FALLBACK
#include <openssl/ec.h>
#else
#include <gnutls/gnutls.h>
#endif

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

class Gnss_Crypto
{
public:
    Gnss_Crypto() = default;
    ~Gnss_Crypto();
    explicit Gnss_Crypto(const std::string& filePath);
    bool have_public_key() const;
    std::vector<uint8_t> computeSHA256(const std::vector<uint8_t>& input) const;
    std::vector<uint8_t> computeSHA3_256(const std::vector<uint8_t>& input) const;
    std::vector<uint8_t> computeHMAC_SHA_256(const std::vector<uint8_t>& key, const std::vector<uint8_t>& input) const;
    std::vector<uint8_t> computeCMAC_AES(const std::vector<uint8_t>& key, const std::vector<uint8_t>& input) const;
    bool verify_signature(const std::vector<uint8_t>& message, const std::vector<uint8_t>& signature);

    void readPublicKeyFromPEM(const std::string& filePath);
    // void set_public_key(const std::vector<uint8_t>& publickey);

private:
#if USE_OPENSSL_FALLBACK
#if USE_OPENSSL_3
    EVP_PKEY* d_PublicKey;
#else
    EC_KEY* d_PublicKey = nullptr;
#endif
#else
    gnutls_pubkey_t d_PublicKey{};
#endif
};

/** \} */
/** \} */
#endif  // GNSS_SDR_GNSS_CRYPTO_H