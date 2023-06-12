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
    readPublicKeyFromPEM(filePath);
}


Gnss_Crypto::~Gnss_Crypto()
{
#if USE_OPENSSL_FALLBACK
#if USE_OPENSSL_3
#else
    if (d_PublicKey != nullptr)
        {
            EC_KEY_free(d_PublicKey);
        }
#endif
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
    // Load the public key from the BIO
#if USE_OPENSSL_3
    d_PublicKey = PEM_read_bio_PUBKEY(bio, nullptr, nullptr, nullptr);
#else
    d_PublicKey = PEM_read_bio_EC_PUBKEY(bio, nullptr, nullptr, nullptr);
#endif
    BIO_free(bio);
    if (d_PublicKey == nullptr)
        {
            std::cerr << "OpenSSL: error reading the Public Key from file " << filePath << ". Aborting import" << std::endl;
            return;
        }
#else
    gnutls_global_init();
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
    gnutls_pubkey_deinit(pubKey);
#endif
    std::cout << "Public key successfully read from file " << filePath << std::endl;
}


bool Gnss_Crypto::verify_signature(const std::vector<uint8_t>& message, const std::vector<uint8_t>& signature)
{
    bool success = false;
#if USE_OPENSSL_FALLBACK
#if USE_OPENSSL_3
    EVP_PKEY_CTX* ctx;
    ctx = EVP_PKEY_CTX_new(d_PublicKey, NULL /* no engine */);
    bool do_operation = true;
    if (!ctx)
        {
            do_operation = false;
        }
    if (EVP_PKEY_verify_init(ctx) <= 0)
        {
            do_operation = false;
        }
    if (EVP_PKEY_CTX_set_signature_md(ctx, EVP_sha256()) <= 0)
        {
            do_operation = false;
        }
    int verification = 0;
    if (do_operation)
        {
            verification = EVP_PKEY_verify(ctx, signature.data(), signature.size(), message.data(), message.size());
        }
    if (verification == 1)
        {
            success = true;
        }
#else
    auto digest = this->computeSHA256(message);
    int verification = ECDSA_verify(0, digest.data(), SHA256_DIGEST_LENGTH, signature.data(), static_cast<int>(signature.size()), d_PublicKey);
    if (verification == 1)
        {
            success = true;
        }
    else if (verification == 0)
        {
            std::cerr << "OpenSSL: invalid signature found when verifying message" << std::endl;
        }

#endif
#else
    // Verify the dummy hash using the public key
    gnutls_datum_t dummyHash = {nullptr, 0};
    int ret2 = gnutls_pubkey_verify_hash2(*d_PublicKey, GNUTLS_SIGN_ECDSA_SHA256, 0, &dummyHash, &dummyHash);
    if (ret2 != GNUTLS_E_SUCCESS)
        {
            std::cout << "GnuTLS: The Public Key is invalid" << std::endl;
        }
    gnutls_datum_t signature_{};
    signature_.data = const_cast<uint8_t*>(signature.data());
    signature_.size = signature.size();
    gnutls_datum_t data_{};
    data_.data = const_cast<uint8_t*>(message.data());
    data_.size = message.size();
    int ret = gnutls_pubkey_verify_data2(*d_PublicKey, GNUTLS_SIGN_ECDSA_SHA256, 0, &data_, &signature_);
    if (ret == GNUTLS_E_SUCCESS)
        {
            success = true;
        }
#endif
    return success;
}