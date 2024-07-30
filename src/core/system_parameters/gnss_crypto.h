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

/*!
 * \brief Class implementing cryptographic functions
 * for Navigation Message Authentication
 */
class Gnss_Crypto
{
public:
    Gnss_Crypto();  //!< Default constructor

    /*!
     * Constructor with a .crt or .pem file for the ECDSA Public Key
     * and a XML file for the Merkle Tree root.
     * Files can be downloaded by registering at https://www.gsc-europa.eu/
     */
    Gnss_Crypto(const std::string& certFilePath, const std::string& merkleTreePath);
    ~Gnss_Crypto();  //!< Default destructor

    bool have_public_key() const;  //!< Returns true if the ECDSA Public Key is already loaded

    /*!
     * Stores the ECDSA Public Key in a .pem file, which is read in a following run if the .crt file is not found
     */
    bool store_public_key(const std::string& pubKeyFilePath) const;

    bool verify_signature_ecdsa_p256(const std::vector<uint8_t>& message, const std::vector<uint8_t>& signature) const;  //!< Verify ECDSA-P256 signature (message in plain hex, signature in raw format)
    bool verify_signature_ecdsa_p521(const std::vector<uint8_t>& message, const std::vector<uint8_t>& signature) const;  //!< Verify ECDSA-P521 signature (message in plain hex, signature in raw format)

    std::vector<uint8_t> compute_SHA_256(const std::vector<uint8_t>& input) const;                                        //!< Computes SHA-256 hash
    std::vector<uint8_t> compute_SHA3_256(const std::vector<uint8_t>& input) const;                                       //!< Computes SHA3-256 hash
    std::vector<uint8_t> compute_HMAC_SHA_256(const std::vector<uint8_t>& key, const std::vector<uint8_t>& input) const;  //!< Computes HMAC-SHA-256 message authentication code
    std::vector<uint8_t> compute_CMAC_AES(const std::vector<uint8_t>& key, const std::vector<uint8_t>& input) const;      //!< Computes CMAC-AES message authentication code

    std::vector<uint8_t> get_public_key() const;   //!< Gets the ECDSA Public Key in PEM format
    std::vector<uint8_t> get_merkle_root() const;  //!< Gets the Merkle Tree root node (\f$ x_{4,0} \f$)

    void set_public_key(const std::vector<uint8_t>& publickey);  //!< Sets the ECDSA Public Key (publickey compressed format)
    void set_merkle_root(const std::vector<uint8_t>& v);         //!< Sets the Merkle Tree root node x(\f$ x_{4,0} \f$)
    void read_merkle_xml(const std::string& merkleFilePath);

private:
    void readPublicKeyFromPEM(const std::string& pemFilePath);
    bool readPublicKeyFromCRT(const std::string& crtFilePath);
    bool convert_raw_to_der_ecdsa(const std::vector<uint8_t>& raw_signature, std::vector<uint8_t>& der_signature) const;
    std::vector<uint8_t> convert_from_hex_str(const std::string& input) const;
#if USE_GNUTLS_FALLBACK
    void decompress_public_key_secp256r1(const std::vector<uint8_t>& compressed_key, std::vector<uint8_t>& x, std::vector<uint8_t>& y) const;
    void decompress_public_key_secp521r1(const std::vector<uint8_t>& compressed_key, std::vector<uint8_t>& x, std::vector<uint8_t>& y) const;
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
};

/** \} */
/** \} */

#endif  // GNSS_SDR_GNSS_CRYPTO_H