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
#include <cstddef>
#include <fstream>
#include <iostream>
#include <iterator>

#if USE_OPENSSL_FALLBACK
#include <openssl/cmac.h>
#include <openssl/hmac.h>
#include <openssl/pem.h>
#if USE_OPENSSL_3
#include <openssl/evp.h>
#define OPENSSL_ENGINE NULL
#else
#include <openssl/rsa.h>
#include <openssl/sha.h>
#endif
#else
#include <gnutls/crypto.h>
#include <gnutls/gnutls.h>
#include <gnutls/x509.h>
#endif

Gnss_Crypto::Gnss_Crypto(const std::string& filePath)
{
    d_PublicKey = readPublicKeyFromPEM(filePath);
}


std::vector<uint8_t> Gnss_Crypto::computeSHA256(const std::vector<uint8_t>& input)
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


std::vector<uint8_t> Gnss_Crypto::computeSHA3_256(const std::vector<uint8_t>& input)
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


std::vector<uint8_t> Gnss_Crypto::computeHMAC_SHA_256(const std::vector<uint8_t>& key, const std::vector<uint8_t>& input)
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


std::vector<uint8_t> Gnss_Crypto::computeCMAC_AES(const std::vector<uint8_t>& key, const std::vector<uint8_t>& input)
{
    std::vector<uint8_t> output(16);
#if USE_OPENSSL_FALLBACK
#if USE_OPENSSL_3
    std::vector<uint8_t> mac(EVP_MAX_MD_SIZE);  // CMAC-AES output size

    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    EVP_MAC* cmac = EVP_MAC_fetch(NULL, "CMAC-AES", NULL);

    EVP_MAC_CTX* cmacCtx = EVP_MAC_CTX_new(cmac);

    OSSL_PARAM params[4];
    params[0] = OSSL_PARAM_construct_utf8_string("key", (char*)key.data(), key.size());
    params[1] = OSSL_PARAM_construct_octet_string("iv", NULL, 0);   // Set IV to NULL
    params[2] = OSSL_PARAM_construct_octet_string("aad", NULL, 0);  // Set AAD to NULL
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


std::vector<uint8_t> Gnss_Crypto::readPublicKeyFromPEM(const std::string& filePath)
{
    std::vector<uint8_t> publicKey;
    // Open the .pem file
    std::ifstream pemFile(filePath);
    if (!pemFile)
        {
            std::cerr << "Failed to open the file: " << filePath << std::endl;
            return publicKey;
        }
#if USE_OPENSSL_FALLBACK
#if USE_OPENSSL_3
    // Read the contents of the file into a string
    std::string pemContent((std::istreambuf_iterator<char>(pemFile)), std::istreambuf_iterator<char>());

    // Create a BIO object from the string data
    BIO* bio = BIO_new_mem_buf(pemContent.c_str(), pemContent.length());
    if (!bio)
        {
            // Handle BIO creation error
            pemFile.close();
            // ...
        }

    // Read the PEM data from the BIO
    EVP_PKEY* evpKey = PEM_read_bio_PUBKEY(bio, nullptr, nullptr, nullptr);
    if (!evpKey)
        {
            // Handle PEM reading error
            BIO_free(bio);
            pemFile.close();
            // ...
        }

    // Create a memory BIO to write the public key data
    BIO* memBio = BIO_new(BIO_s_mem());
    if (!memBio)
        {
            // Handle memory BIO creation error
            EVP_PKEY_free(evpKey);
            BIO_free(bio);
            pemFile.close();
            // ...
        }

    // Write the public key to the memory BIO
    int result = PEM_write_bio_PUBKEY(memBio, evpKey);
    if (result != 1)
        {
            // Handle public key writing error
            BIO_free(memBio);
            EVP_PKEY_free(evpKey);
            BIO_free(bio);
            pemFile.close();
            // ...
        }

    // Get the pointer to the memory BIO data and its length
    char* bioData;
    long bioDataLength = BIO_get_mem_data(memBio, &bioData);

    // Copy the public key data to the vector
    publicKey.assign(bioData, bioData + bioDataLength);

    // Free resources
    BIO_free(memBio);
    EVP_PKEY_free(evpKey);
    BIO_free(bio);
    pemFile.close();
#else
    // Read the PEM file contents into a string
    std::string pemContents((std::istreambuf_iterator<char>(pemFile)), std::istreambuf_iterator<char>());

    // Create a BIO object to hold the PEM data
    BIO* bio = BIO_new_mem_buf(pemContents.c_str(), -1);

    // Load the public key from the BIO
    RSA* rsa = PEM_read_bio_RSA_PUBKEY(bio, nullptr, nullptr, nullptr);
    BIO_free(bio);

    if (rsa == nullptr)
        {
            // Handle error reading public key
            return {};
        }

    // Get the RSA modulus and convert it to a vector of uint8_t
    const BIGNUM* rsaModulus = nullptr;
    RSA_get0_key(rsa, &rsaModulus, nullptr, nullptr);

    BN_bn2bin(rsaModulus, publicKey.data());

    // Clean up the RSA object
    RSA_free(rsa);
#endif
#else
    // Read the contents of the .pem file into a string
    std::string pemContents((std::istreambuf_iterator<char>(pemFile)), std::istreambuf_iterator<char>());

    gnutls_x509_crt_t cert;
    gnutls_x509_crt_init(&cert);

    // Import the certificate from the PEM file
    gnutls_datum_t pemData;
    pemData.data = reinterpret_cast<unsigned char*>(const_cast<char*>(pemContents.data()));
    pemData.size = pemContents.size();
    int ret = gnutls_x509_crt_import(cert, &pemData, GNUTLS_X509_FMT_PEM);
    if (ret < 0)
        {
            std::cerr << "Failed to import certificate from PEM file" << std::endl;
            gnutls_x509_crt_deinit(cert);
            return publicKey;
        }

    // Export the public key data
    size_t pubkey_data_size = 0;
    ret = gnutls_x509_crt_export(cert, GNUTLS_X509_FMT_DER, nullptr, &pubkey_data_size);
    if (ret < 0)
        {
            std::cerr << "Failed to export public key data" << std::endl;
            gnutls_x509_crt_deinit(cert);
            return publicKey;
        }

    publicKey.resize(pubkey_data_size);
    ret = gnutls_x509_crt_export(cert, GNUTLS_X509_FMT_DER, publicKey.data(), &pubkey_data_size);
    if (ret < 0)
        {
            std::cerr << "Failed to export public key data" << std::endl;
            gnutls_x509_crt_deinit(cert);
            return publicKey;
        }

    gnutls_x509_crt_deinit(cert);
#endif
    return publicKey;
}


// // bool signature(const std::vector<uint8_t>& publicKey, const std::vector<uint8_t>& digest, std::vector<uint8_t>& signature)
// // {
// //     bool success = false;
// // #if USE_OPENSSL_FALLBACK
// // #else
// //     gnutls_global_init();
// //     int result = gnutls_pubkey_verify_data(publicKey.data(), GNUTLS_SIGN_ECDSA_SHA256, digest.data, digest.size(), signature.data(), signature.size());
// //     success = (result == GNUTLS_E_SUCCESS);
// // gnutls_global_deinit();
// // #endif
// //     return success;
// // }
// // bool verifyDigitalSignature(const unsigned char* signature, size_t signatureSize, const unsigned char* message, size_t messageSize, gnutls_pubkey_t publicKey)
// // {
// //     int verificationStatus = gnutls_pubkey_verify_data(publicKey, GNUTLS_DIG_SHA256, 0, message, messageSize, signature, signatureSize);
// //     return verificationStatus == 0;
