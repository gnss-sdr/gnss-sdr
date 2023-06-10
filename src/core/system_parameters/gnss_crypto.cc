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
#include <openssl/hmac.h>
#include <openssl/pem.h>
#if USE_OPENSSL_3
#include <openssl/evp.h>
#define OPENSSL_ENGINE nullptr
#else
#include <openssl/sha.h>
#endif
#else
#include <gnutls/crypto.h>
#include <gnutls/gnutls.h>
#include <gnutls/x509.h>
#endif

Gnss_Crypto::Gnss_Crypto(const std::string& filePath)
{
    readPublicKeyFromPEM(filePath);
}


bool Gnss_Crypto::have_public_key() const
{
    return !d_PublicKey.empty();
}


void Gnss_Crypto::set_public_key(const std::vector<uint8_t>& publickey)
{
    d_PublicKey = publickey;
}


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
    EC_KEY* ecKey = EVP_PKEY_get1_EC_KEY(evpKey);
    EVP_PKEY_free(evpKey);

    if (ecKey == nullptr)
        {
            std::cout << "OpenSSL: Failed to get the EC key from file " << filePath << ". Aborting import" << std::endl;
            return;
        }

    // Get the EC group from the EC key
    const EC_GROUP* ecGroup = EC_KEY_get0_group(ecKey);

    if (ecGroup == nullptr)
        {
            std::cout << "OpenSSL: Failed to extract the EC group from file " << filePath << ". Aborting import" << std::endl;
            EC_KEY_free(ecKey);
            return;
        }

    // Check if it is ECDSA P-256
    if (EC_GROUP_get_curve_name(ecGroup) != NID_X9_62_prime256v1)
        {
            std::cerr << "Invalid curve name in file " << filePath << ". Expected P-256. Aborting import" << std::endl;
            EC_KEY_free(ecKey);
            return;
        }
    // Convert the EC parameters to an octet string (raw binary)
    // size_t octetSize = i2o_ECPublicKey(ecKey, nullptr);
    // std::vector<uint8_t> ecParameters(octetSize);
    // unsigned char* p = ecParameters.data();
    // i2o_ECPublicKey(ecKey, &p);
    std::vector<uint8_t> ecParameters(EC_GROUP_get_degree(ecGroup) / 8);
    EC_POINT_point2oct(ecGroup, EC_KEY_get0_public_key(ecKey), POINT_CONVERSION_UNCOMPRESSED, ecParameters.data(), ecParameters.size(), nullptr);

    // Get the EC public key from the EC key
    const EC_POINT* ecPoint = EC_KEY_get0_public_key(ecKey);

    // Convert the EC public key to an octet string (raw binary)
    size_t pointSize = EC_POINT_point2oct(ecGroup, ecPoint, POINT_CONVERSION_UNCOMPRESSED, nullptr, 0, nullptr);
    publicKey = std::vector<uint8_t>(pointSize);
    EC_POINT_point2oct(ecGroup, ecPoint, POINT_CONVERSION_UNCOMPRESSED, publicKey.data(), pointSize, nullptr);
    size_t octetSize = i2o_ECPublicKey(ecKey, nullptr);
    std::vector<uint8_t> publicKey2(octetSize);
    unsigned char* p2 = publicKey2.data();
    i2o_ECPublicKey(ecKey, &p2);
    // Clean up the EC key
    EC_KEY_free(ecKey);

    std::cout << "EC parameters (size: " << ecParameters.size() << "):";
    for (auto k : ecParameters)
        {
            std::cout << " " << static_cast<uint32_t>(k);
        }
    std::cout << std::endl;
    std::cout << "Public Key (size: " << publicKey.size() << "):";
    for (auto k : publicKey)
        {
            std::cout << " " << static_cast<uint32_t>(k);
        }
    std::cout << "Public Key2: (size: " << publicKey2.size() << "):";
    for (auto k : publicKey2)
        {
            std::cout << " " << static_cast<uint32_t>(k);
        }
    std::cout << std::endl;
#else
    // Load the public key from the BIO
    EC_KEY* ecKeyPublic = PEM_read_bio_EC_PUBKEY(bio, nullptr, nullptr, nullptr);
    BIO_free(bio);
    if (ecKeyPublic == nullptr)
        {
            std::cerr << "OpenSSL: error reading the Public Key from file " << filePath << ". Aborting import" << std::endl;
            return;
        }

    // // Get the EC group and EC point from the EC key
    const EC_GROUP* ecGroup = EC_KEY_get0_group(ecKeyPublic);
    const EC_POINT* ecPoint = EC_KEY_get0_public_key(ecKeyPublic);
    // Convert the EC point to an octet string (raw binary)
    const size_t octetSize = EC_POINT_point2oct(
        ecGroup, ecPoint, POINT_CONVERSION_UNCOMPRESSED, nullptr, 0, nullptr);
    publicKey = std::vector<uint8_t>(octetSize);
    EC_POINT_point2oct(
        ecGroup, ecPoint, POINT_CONVERSION_UNCOMPRESSED, publicKey.data(), octetSize, nullptr);
    EC_KEY_free(ecKeyPublic);

    std::cout << "Public Key:";
    for (auto k : publicKey)
        {
            std::cout << " " << static_cast<uint32_t>(k);
        }
    std::cout << std::endl;
#endif
#else
    // Find the beginning and end of the EC PARAMETERS section
    std::size_t beginPos = pemContent.find("-----BEGIN EC PARAMETERS-----");
    std::size_t endPos = pemContent.find("-----END EC PARAMETERS-----");
    if (beginPos == std::string::npos || endPos == std::string::npos)
        {
            std::cerr << "No EC Parameters found in file " << filePath << ". Aborting import" << std::endl;
            return;
        }

    // Extract the EC parameters data
    std::string ecParamsBase64 = pemContent.substr(beginPos + 30, endPos - beginPos - 31);
    std::vector<uint8_t> ecParameters = base64Decode(ecParamsBase64);

    std::cout << ecParamsBase64 << std::endl;
    std::cout << "Size ecParamsBase64 : " << ecParamsBase64.size() << std::endl;
    std::cout << "Size EC : " << ecParameters.size() << std::endl;
    for (auto k : ecParameters)
        {
            std::cout << " " << static_cast<uint32_t>(k);
        }
    std::cout << std::endl;

    std::size_t beginPos2 = pemContent.find("-----BEGIN PUBLIC KEY-----");
    std::size_t endPos2 = pemContent.find("-----END PUBLIC KEY-----");
    if (beginPos2 == std::string::npos || endPos2 == std::string::npos)
        {
            std::cout << "No Public Key found in file " << filePath << ". Aborting import" << std::endl;
            return;
        }
    auto PublickeyBase64 = pemContent.substr(beginPos2 + 27, endPos2 - beginPos2 - 28);
    auto readpublickey_long = base64Decode(PublickeyBase64);
    publicKey = std::vector<uint8_t>(readpublickey_long.begin() + 26, readpublickey_long.end());  // ??

    std::cout << "Public Key (size: " << publicKey.size() << "):" << std::endl;
    for (auto k : publicKey)
        {
            std::cout << " " << static_cast<uint32_t>(k);
        }
    std::cout << std::endl;
#endif
    d_PublicKey = publicKey;
    std::cout << "Public key successfully read from file " << filePath << std::endl;
}


// bool signature(const std::vector<uint8_t>& publicKey, const std::vector<uint8_t>& digest, const std::vector<uint8_t>& signature)
// {
//     bool success = false;
// #if USE_OPENSSL_FALLBACK
// #else
//     gnutls_global_init();
//     int result = gnutls_pubkey_verify_data(publicKey.data(), GNUTLS_SIGN_ECDSA_SHA256, digest.data(), digest.size(), signature.data(), signature.size());
//     success = (result == GNUTLS_E_SUCCESS);
//     gnutls_global_deinit();
// #endif
//     return success;
// }
// // bool verifyDigitalSignature(const unsigned char* signature, size_t signatureSize, const unsigned char* message, size_t messageSize, gnutls_pubkey_t publicKey)
// // {
// //     int verificationStatus = gnutls_pubkey_verify_data(publicKey, GNUTLS_DIG_SHA256, 0, message, messageSize, signature, signatureSize);
// //     return verificationStatus == 0;


std::vector<uint8_t> Gnss_Crypto::base64Decode(const std::string& encoded_string)
{
    const std::string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
    int in_len = encoded_string.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    uint8_t char_array_4[4];
    uint8_t char_array_3[3];
    std::vector<uint8_t> decoded;

    while (in_len-- && (encoded_string[in_] != '=') &&
           (isalnum(encoded_string[in_]) || (encoded_string[in_] == '+') || (encoded_string[in_] == '/')))
        {
            char_array_4[i++] = encoded_string[in_];
            in_++;
            if (i == 4)
                {
                    for (i = 0; i < 4; i++)
                        {
                            char_array_4[i] = base64_chars.find(char_array_4[i]);
                        }

                    char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
                    char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
                    char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

                    for (i = 0; (i < 3); i++)
                        {
                            decoded.push_back(char_array_3[i]);
                        }
                    i = 0;
                }
        }

    if (i)
        {
            for (j = i; j < 4; j++)
                {
                    char_array_4[j] = 0;
                }

            for (j = 0; j < 4; j++)
                {
                    char_array_4[j] = base64_chars.find(char_array_4[j]);
                }

            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

            for (j = 0; (j < i - 1); j++)
                {
                    decoded.push_back(char_array_3[j]);
                }
        }

    return decoded;
}