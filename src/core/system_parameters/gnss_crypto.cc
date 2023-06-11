/*!
 * \file gnss_crypto.cc
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

#include "gnss_crypto.h"
#include "Galileo_OSNMA.h"
#include <cstddef>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>

#if USE_OPENSSL_FALLBACK
#include <openssl/cmac.h>
#include <openssl/ecdsa.h>
#include <openssl/hmac.h>
#include <openssl/pem.h>
#if USE_OPENSSL_3
#include <openssl/evp.h>
#define OPENSSL_ENGINE nullptr
#else
#include <openssl/sha.h>
#endif
#else
#include <gnutls/abstract.h>
#include <gnutls/crypto.h>
#include <gnutls/x509.h>
#endif


Gnss_Crypto::Gnss_Crypto(const std::string& filePath)
{
#if USE_OPENSSL_FALLBACK
#else
    gnutls_global_init();
#endif
    readPublicKeyFromPEM(filePath);
}


Gnss_Crypto::~Gnss_Crypto()
{
#if USE_OPENSSL_FALLBACK
    if (d_PublicKey != nullptr)
        {
            EC_KEY_free(d_PublicKey);
        }
#else
    if (d_PublicKey != nullptr)
        {
            gnutls_pubkey_deinit(*d_PublicKey);
        }

    gnutls_global_deinit();
#endif
}


bool Gnss_Crypto::have_public_key() const
{
    return (d_PublicKey != nullptr);
}


// void Gnss_Crypto::set_public_key(const std::vector<uint8_t>& publickey)
// {
// }


std::vector<uint8_t> Gnss_Crypto::computeSHA256(const std::vector<uint8_t>& input) const
{
    std::vector<uint8_t> output(32);  // SHA256 hash size
#if USE_OPENSSL_FALLBACK
#if USE_OPENSSL_3
    unsigned int mdLen;
    EVP_MD_CTX* mdCtx = EVP_MD_CTX_new();
    if (!EVP_DigestInit_ex(mdCtx, EVP_sha256(), OPENSSL_ENGINE))
        {
            // LOG(WARNING) << "OSNMA SHA-256: Message digest initialization failed.";
            EVP_MD_CTX_free(mdCtx);
            return output;
        }
    if (!EVP_DigestUpdate(mdCtx, input.data(), input.size()))
        {
            // LOG(WARNING) << "OSNMA SHA-256: Message digest update failed.";
            EVP_MD_CTX_free(mdCtx);
            return output;
        }
    if (!EVP_DigestFinal_ex(mdCtx, output.data(), &mdLen))
        {
            // LOG(WARNING) << "OSNMA SHA-256: Message digest finalization failed.";
            EVP_MD_CTX_free(mdCtx);
            return output;
        }
    EVP_MD_CTX_free(mdCtx);
#else
    SHA256_CTX sha256Context;
    SHA256_Init(&sha256Context);
    SHA256_Update(&sha256Context, input.data(), input.size());
    SHA256_Final(output.data(), &sha256Context);
#endif
#else
    std::vector<uint8_t> output_aux(32);
    gnutls_hash_hd_t hashHandle;
    gnutls_hash_init(&hashHandle, GNUTLS_DIG_SHA256);
    gnutls_hash(hashHandle, input.data(), input.size());
    gnutls_hash_output(hashHandle, output_aux.data());
    output = output_aux;
    gnutls_hash_deinit(hashHandle, output_aux.data());
#endif
    return output;
}


std::vector<uint8_t> Gnss_Crypto::computeSHA3_256(const std::vector<uint8_t>& input) const
{
    std::vector<uint8_t> output(32);  // SHA256 hash size
#if USE_OPENSSL_FALLBACK
#if USE_OPENSSL_3
    EVP_MD_CTX* mdctx = EVP_MD_CTX_new();
    const EVP_MD* md = EVP_sha3_256();

    EVP_DigestInit_ex(mdctx, md, nullptr);
    EVP_DigestUpdate(mdctx, input.data(), input.size());
    EVP_DigestFinal_ex(mdctx, output.data(), nullptr);
    EVP_MD_CTX_free(mdctx);
#else
    // SHA3-256 not implemented in OpenSSL < 3.0
#endif
#else
    std::vector<uint8_t> output_aux(32);
    gnutls_hash_hd_t hashHandle;
    gnutls_hash_init(&hashHandle, GNUTLS_DIG_SHA3_256);
    gnutls_hash(hashHandle, input.data(), input.size());
    gnutls_hash_output(hashHandle, output_aux.data());
    output = output_aux;
    gnutls_hash_deinit(hashHandle, output_aux.data());
#endif
    return output;
}


std::vector<uint8_t> Gnss_Crypto::computeHMAC_SHA_256(const std::vector<uint8_t>& key, const std::vector<uint8_t>& input) const
{
    std::vector<uint8_t> output(32);
#if USE_OPENSSL_FALLBACK
#if USE_OPENSSL_3
    std::vector<uint8_t> hmac(EVP_MAX_MD_SIZE);

    // Create HMAC-SHA256 context
    EVP_MD_CTX* ctx = EVP_MD_CTX_new();
    EVP_PKEY* pkey = EVP_PKEY_new_mac_key(EVP_PKEY_HMAC, nullptr, key.data(), key.size());

    // Initialize HMAC-SHA256 context
    EVP_DigestSignInit(ctx, nullptr, EVP_sha256(), nullptr, pkey);

    // Compute HMAC-SHA256
    EVP_DigestSignUpdate(ctx, input.data(), input.size());
    size_t macLength;
    EVP_DigestSignFinal(ctx, hmac.data(), &macLength);

    EVP_PKEY_free(pkey);
    EVP_MD_CTX_free(ctx);

    hmac.resize(macLength);
    output = hmac;
#else
    std::vector<uint8_t> hmac(32);
    // Create HMAC context
    HMAC_CTX* ctx = HMAC_CTX_new();
    HMAC_Init_ex(ctx, key.data(), key.size(), EVP_sha256(), nullptr);

    // Update HMAC context with the message
    HMAC_Update(ctx, input.data(), input.size());

    // Finalize HMAC computation
    unsigned int hmacLen;
    HMAC_Final(ctx, hmac.data(), &hmacLen);

    // Clean up HMAC context
    HMAC_CTX_free(ctx);

    // Resize the HMAC vector to the actual length
    hmac.resize(hmacLen);
    output = hmac;
#endif
#else
    std::vector<uint8_t> output_aux(32);
    gnutls_hmac_hd_t hmac;
    gnutls_hmac_init(&hmac, GNUTLS_MAC_SHA256, key.data(), key.size());
    gnutls_hmac(hmac, input.data(), input.size());
    gnutls_hmac_output(hmac, output_aux.data());
    output = output_aux;
    gnutls_hmac_deinit(hmac, output_aux.data());
#endif
    return output;
}


std::vector<uint8_t> Gnss_Crypto::computeCMAC_AES(const std::vector<uint8_t>& key, const std::vector<uint8_t>& input) const
{
    std::vector<uint8_t> output(16);
#if USE_OPENSSL_FALLBACK
#if USE_OPENSSL_3
    std::vector<uint8_t> mac(EVP_MAX_MD_SIZE);  // CMAC-AES output size

    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    EVP_MAC* cmac = EVP_MAC_fetch(nullptr, "CMAC-AES", nullptr);

    EVP_MAC_CTX* cmacCtx = EVP_MAC_CTX_new(cmac);

    OSSL_PARAM params[4];
    params[0] = OSSL_PARAM_construct_utf8_string("key", (char*)key.data(), key.size());
    params[1] = OSSL_PARAM_construct_octet_string("iv", nullptr, 0);   // Set IV to nullptr
    params[2] = OSSL_PARAM_construct_octet_string("aad", nullptr, 0);  // Set AAD to nullptr
    params[3] = OSSL_PARAM_construct_end();

    // Set AES-128 CMAC cipher and key
    EVP_MAC_init(cmacCtx, nullptr, 0, params);

    // Compute CMAC-AES
    EVP_MAC_update(cmacCtx, input.data(), input.size());
    size_t macLength = mac.size();
    size_t outputLength = 16;

    EVP_MAC_final(cmacCtx, mac.data(), &macLength, outputLength);

    EVP_MAC_free(cmac);
    EVP_MAC_CTX_free(cmacCtx);
    EVP_CIPHER_CTX_free(ctx);

    mac.resize(macLength);
    output = mac;
#else
    std::vector<uint8_t> mac(16);  // CMAC-AES output size

    // Create CMAC context
    CMAC_CTX* cmacCtx = CMAC_CTX_new();
    CMAC_Init(cmacCtx, key.data(), key.size(), EVP_aes_128_cbc(), nullptr);

    // Compute CMAC-AES
    CMAC_Update(cmacCtx, input.data(), input.size());
    CMAC_Final(cmacCtx, mac.data(), nullptr);

    // Clean up CMAC context
    CMAC_CTX_free(cmacCtx);

    output = mac;
#endif
#else
    gnutls_cipher_hd_t cipher;
    std::vector<uint8_t> mac(16);
    std::vector<uint8_t> message = input;
    gnutls_datum_t key_data = {const_cast<uint8_t*>(key.data()), static_cast<unsigned int>(key.size())};
    gnutls_cipher_init(&cipher, GNUTLS_CIPHER_AES_128_CBC, &key_data, nullptr);
    gnutls_cipher_set_iv(cipher, nullptr, 16);                      // Set IV to zero
    gnutls_cipher_encrypt(cipher, message.data(), message.size());  // Encrypt the message with AES-128
    gnutls_cipher_tag(cipher, mac.data(), mac.size());              // Get the CMAC-AES tag
    output = mac;
    gnutls_cipher_deinit(cipher);
#endif
    return output;
}


void Gnss_Crypto::readPublicKeyFromPEM(const std::string& filePath)
{
    // Open the .pem file
    std::ifstream pemFile(filePath);
    if (!pemFile)
        {
            // PEM file not found
            // If it not was the default, maybe it is a configuration error
            if (filePath != PEMFILE_DEFAULT)
                {
                    std::cerr << "File " << filePath << " not found" << std::endl;
                }
            return;
        }
    std::vector<uint8_t> publicKey;
    std::string pemContent((std::istreambuf_iterator<char>(pemFile)), std::istreambuf_iterator<char>());
#if USE_OPENSSL_FALLBACK
    // Create a BIO object from the string data
    BIO* bio = BIO_new_mem_buf(pemContent.c_str(), pemContent.length());
    if (!bio)
        {
            std::cerr << "OpenSSL: error creating a BIO object with data read from file " << filePath << ". Aborting import" << std::endl;
            return;
        }
#if USE_OPENSSL_3
    // Read the PEM data from the BIO object
    EVP_PKEY* evpKey = PEM_read_bio_PUBKEY(bio, nullptr, nullptr, nullptr);
    BIO_free(bio);
    if (!evpKey)
        {
            std::cerr << "OpenSSL: error reading the Public Key from file " << filePath << ". Aborting import" << std::endl;
            return;
        }

    // Check if the public key is an EC key
    if (EVP_PKEY_base_id(evpKey) != EVP_PKEY_EC)
        {
            std::cerr << "OpenSSL: Public key imported from file " << filePath << " is not an EC key. Aborting import" << std::endl;
            EVP_PKEY_free(evpKey);
            return;
        }

    // Get the EC key from the EVP_PKEY object
    d_PublicKey = EVP_PKEY_get1_EC_KEY(evpKey);
    EVP_PKEY_free(evpKey);

    if (d_PublicKey == nullptr)
        {
            std::cout << "OpenSSL: Failed to get the EC key from file " << filePath << ". Aborting import" << std::endl;
            return;
        }

    // Get the EC group from the EC key
    const EC_GROUP* ecGroup = EC_KEY_get0_group(d_PublicKey);

    if (ecGroup == nullptr)
        {
            std::cout << "OpenSSL: Failed to extract the EC group from file " << filePath << ". Aborting import" << std::endl;
            EC_KEY_free(d_PublicKey);
            return;
        }

    // Check if it is ECDSA P-256
    if (EC_GROUP_get_curve_name(ecGroup) != NID_X9_62_prime256v1)
        {
            std::cerr << "Invalid curve name in file " << filePath << ". Expected P-256. Aborting import" << std::endl;
            EC_KEY_free(d_PublicKey);
            return;
        }
#else
    // Load the public key from the BIO
    d_PublicKey = PEM_read_bio_EC_PUBKEY(bio, nullptr, nullptr, nullptr);
    BIO_free(bio);
    if (d_PublicKey == nullptr)
        {
            std::cerr << "OpenSSL: error reading the Public Key from file " << filePath << ". Aborting import" << std::endl;
            return;
        }
#endif
#else
    gnutls_pubkey_t pubKey;
    gnutls_pubkey_init(&pubKey);
    d_PublicKey = &pubKey;
    // Import the PEM data
    gnutls_datum_t pemDatum = {const_cast<unsigned char*>(reinterpret_cast<unsigned char*>(pemContent.data())), static_cast<unsigned int>(pemContent.size())};
    int ret = gnutls_pubkey_import(*d_PublicKey, &pemDatum, GNUTLS_X509_FMT_PEM);
    if (ret < 0)
        {
            std::cerr << "GnuTLS: error reading the Public Key from file "
                      << filePath
                      << ". (Error: " << gnutls_strerror(ret) << "). Aborting import" << std::endl;
            gnutls_pubkey_deinit(*d_PublicKey);
            return;
        }
#endif
    std::cout << "Public key successfully read from file " << filePath << std::endl;
}


// bool verify_signature(const std::vector<uint8_t>& message, const std::vector<uint8_t>& signature)
// {
//     bool success = false;
// #if USE_OPENSSL_FALLBACK
/** Verifies that the given signature is valid ECDSA signature
 *  of the supplied hash value using the specified public key.
 *  \param  type     this parameter is ignored
 *  \param  dgst     pointer to the hash value
 *  \param  dgstlen  length of the hash value
 *  \param  sig      pointer to the DER encoded signature
 *  \param  siglen   length of the DER encoded signature
 *  \param  eckey    EC_KEY object containing a public EC key
 *  \return 1 if the signature is valid, 0 if the signature is invalid
 *          and -1 on error
 */
// int ECDSA_verify(int type, const unsigned char *dgst, int dgstlen, const unsigned char *sig, int siglen, EC_KEY *eckey);)
// int verification = ECDSA_verify(0, digest, SHA256_DIGEST_LENGTH, signature, signature_len, key_pair_obj);
// #else
//     gnutls_global_init();
//     int result = gnutls_pubkey_verify_data(publicKey.data(), GNUTLS_SIGN_ECDSA_SHA256, digest.data(), digest.size(), signature.data(), signature.size());
//     success = (result == GNUTLS_E_SUCCESS);
//     gnutls_global_deinit();
// pubkey: Holds the public key
// algo: The signature algorithm used
// flags: Zero or an OR list of gnutls_certificate_verify_flags
// data: holds the signed data
// signature: contains the signature
// This function will verify the given signed data, using the parameters from the certificate.
// gnutls_pubkey_verify_data2 (gnutls_pubkey_t pubkey, gnutls_sign_algorithm_t algo, unsigned int flags, const gnutls_datum_t * data, const gnutls_datum_t * signature)
// #endif
//     return success;
// }
// // bool verifyDigitalSignature(const unsigned char* signature, size_t signatureSize, const unsigned char* message, size_t messageSize, gnutls_pubkey_t publicKey)
// // {
// //     int verificationStatus = gnutls_pubkey_verify_data(publicKey, GNUTLS_DIG_SHA256, 0, message, messageSize, signature, signatureSize);
// //     return verificationStatus == 0;
