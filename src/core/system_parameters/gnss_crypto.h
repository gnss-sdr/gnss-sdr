/*!
 * \file gnss_crypto.h
 * \brief Class for computing cryptographic functions
 * \author Carles Fernandez, 2023-2024. cfernandez(at)cttc.es
 *   Cesare Ghionoiu Martinez, 2023-2024. c.ghionoiu-martinez@tu-braunschweig.de
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2024  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_GNSS_CRYPTO_H
#define GNSS_SDR_GNSS_CRYPTO_H

#include <cstdint>
#include <string>
#include <vector>
#if USE_GNUTLS_FALLBACK
#include <gnutls/abstract.h>
#include <gnutls/gnutls.h>
#else  // OpenSSL
#include <openssl/ec.h>
#endif

/** \addtogroup Core
 * \{ */
/** \addtogroup System_Parameters
 * \{ */

class Gnss_Crypto
{
public:
    Gnss_Crypto();
    Gnss_Crypto(const std::string& certFilePath, const std::string& merkleTreePath);
    ~Gnss_Crypto();

    void set_public_key(const std::vector<uint8_t>& publickey);
    bool have_public_key() const;
    bool verify_signature(const std::vector<uint8_t>& message, const std::vector<uint8_t>& signature) const;
    bool store_public_key(const std::string& pubKeyFilePath) const;
    std::vector<uint8_t> computeSHA256(const std::vector<uint8_t>& input) const;
    std::vector<uint8_t> computeSHA3_256(const std::vector<uint8_t>& input) const;
    std::vector<uint8_t> computeHMAC_SHA_256(const std::vector<uint8_t>& key, const std::vector<uint8_t>& input) const;
    std::vector<uint8_t> computeCMAC_AES(const std::vector<uint8_t>& key, const std::vector<uint8_t>& input) const;
    std::vector<uint8_t> getMerkleRoot(const std::vector<std::vector<uint8_t>>& merkle) const;
    std::vector<uint8_t> getPublicKey() const;
    inline std::vector<uint8_t> getMerkleRoot() const
    {
        return d_x_4_0;
    }
    inline void setMerkleRoot(std::vector<uint8_t> v)
    {
        d_x_4_0 = v;
    }

private:
    void read_merkle_xml(const std::string& merkleFilePath);
    void readPublicKeyFromPEM(const std::string& pemFilePath);
    bool readPublicKeyFromCRT(const std::string& crtFilePath);
    bool convert_raw_to_der_ecdsa(const std::vector<uint8_t>& raw_signature, std::vector<uint8_t>& der_signature) const;
    std::vector<uint8_t> convert_from_hex_str(const std::string& input) const;
#if USE_GNUTLS_FALLBACK
    bool pubkey_copy(gnutls_pubkey_t src, gnutls_pubkey_t* dest);
    gnutls_pubkey_t d_PublicKey{};
#else  // OpenSSL
#if USE_OPENSSL_3
    bool pubkey_copy(EVP_PKEY* src, EVP_PKEY** dest);
    EVP_PKEY* d_PublicKey{};
#else  // OpenSSL 1.x
    bool pubkey_copy(EC_KEY* src, EC_KEY** dest);
    EC_KEY* d_PublicKey = nullptr;
#endif
#endif
    std::vector<uint8_t> d_x_4_0;
    std::vector<uint8_t> d_x_3_1;
    std::vector<uint8_t> d_x_2_1;
    std::vector<uint8_t> d_x_1_1;
    std::vector<uint8_t> d_x_0_0;
    std::vector<uint8_t> d_x_0_1;
};

/** \} */
/** \} */

#endif  // GNSS_SDR_GNSS_CRYPTO_H