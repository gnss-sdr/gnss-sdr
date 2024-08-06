/*!
 * \file gnss_crypto.cc
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

#include "gnss_crypto.h"
#include "Galileo_OSNMA.h"
#include <pugixml.hpp>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>

#if USE_GNUTLS_FALLBACK
#include <cstring>
#include <gmpxx.h>
#include <gnutls/crypto.h>
#include <gnutls/x509.h>
#else  // OpenSSL
#include <openssl/cmac.h>
#include <openssl/ecdsa.h>
#include <openssl/hmac.h>
#include <openssl/pem.h>
#include <openssl/x509.h>
#if USE_OPENSSL_3
#include <openssl/bio.h>
#include <openssl/bn.h>
#include <openssl/core_names.h>
#include <openssl/decoder.h>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/param_build.h>
#include <openssl/params.h>
#define OPENSSL_ENGINE nullptr
#else  // OpenSSL 1.x
#include <openssl/sha.h>
#endif
#endif

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>  // for DLOG
#else
#include <absl/log/log.h>
#endif


Gnss_Crypto::Gnss_Crypto()
{
#if USE_GNUTLS_FALLBACK
    gnutls_global_init();
#if !HAVE_GNUTLS_SIGN_ECDSA_SHA256
    LOG(WARNING) << "The GnuTLS library version you are linking against is too old for some OSNMA functions."
                 << " Please do not trust OSNMA ouputs or upgrade your system to a newer version of GnuTLS or OpenSSL"
                 << " and rebuild GNSS-SDR against it.";
#endif
#else  // OpenSSL
#if !(USE_OPENSSL_3 || USE_OPENSSL_111)
    LOG(WARNING) << "The OpenSSL library version you are linking against is too old for some OSNMA functions."
                 << " Please do not trust OSNMA ouputs or upgrade your system to a newer version of OpenSSL"
                 << " and rebuild GNSS-SDR against it.";
#endif
#endif
}


Gnss_Crypto::Gnss_Crypto(const std::string& certFilePath, const std::string& merkleTreePath)
{
#if USE_GNUTLS_FALLBACK
    gnutls_global_init();
#if !HAVE_GNUTLS_SIGN_ECDSA_SHA256
    LOG(WARNING) << "The GnuTLS library version you are linking against is too old for some OSNMA functions."
                 << " Please do not trust OSNMA ouputs or upgrade your system to a newer version of GnuTLS or OpenSSL"
                 << " and rebuild GNSS-SDR against it.";
#endif
#else  // OpenSSL
#if !(USE_OPENSSL_3 || USE_OPENSSL_111)
    LOG(WARNING) << "The OpenSSL library version you are linking against is too old for some OSNMA functions."
                 << " Please do not trust OSNMA ouputs or upgrade your system to a newer version of OpenSSL"
                 << " and rebuild GNSS-SDR against it.";
#endif
#endif
    if (!readPublicKeyFromCRT(certFilePath))
        {
            readPublicKeyFromPEM(certFilePath);
            if (!have_public_key())
                {
                    readPublicKeyFromPEM(PEMFILE_DEFAULT);
                }
        }
    read_merkle_xml(merkleTreePath);
}


Gnss_Crypto::~Gnss_Crypto()
{
#if USE_GNUTLS_FALLBACK
    if (d_PublicKey != nullptr)
        {
            gnutls_pubkey_deinit(d_PublicKey);
            d_PublicKey = nullptr;
        }
    gnutls_global_deinit();
#else  // OpenSSL
#if !USE_OPENSSL_3
    if (d_PublicKey != nullptr)
        {
            EC_KEY_free(d_PublicKey);
        }
#endif
#endif
}


bool Gnss_Crypto::have_public_key() const
{
#if USE_GNUTLS_FALLBACK
    return (d_PublicKey != gnutls_pubkey_t{});
#else  // OpenSSL
    return (d_PublicKey != nullptr);
#endif
}


bool Gnss_Crypto::store_public_key(const std::string& pubKeyFilePath) const
{
    if (!have_public_key())
        {
            return false;
        }
    std::ofstream pubKeyFile(pubKeyFilePath, std::ios::binary);
    if (!pubKeyFile.is_open())
        {
            LOG(WARNING) << "Unable to open file for storing the Public Key: " << pubKeyFilePath;
            return false;
        }
#if USE_GNUTLS_FALLBACK
    gnutls_datum_t pem_data;
#if HAVE_GNUTLS_PUBKEY_EXPORT2
    int ret = gnutls_pubkey_export2(d_PublicKey, GNUTLS_X509_FMT_PEM, &pem_data);
#else
    size_t output_stata_size;
    int ret = gnutls_pubkey_export(d_PublicKey, GNUTLS_X509_FMT_PEM, &pem_data, &output_stata_size);
#endif
    if (ret != GNUTLS_E_SUCCESS)
        {
            LOG(WARNING) << "GnuTLS: Failed to export public key: " << gnutls_strerror(ret);
            return false;
        }

    pubKeyFile.write(reinterpret_cast<const char*>(pem_data.data), pem_data.size);
    pubKeyFile.close();
    gnutls_free(pem_data.data);
#else  // OpenSSL
    BIO* bio = BIO_new(BIO_s_mem());
    if (!bio)
        {
            LOG(WARNING) << "OpenSSL: Failed to create BIO";
            return false;
        }
#if USE_OPENSSL_3
    if (!PEM_write_bio_PUBKEY(bio, d_PublicKey))
#else  // OpenSSL 1.x
    if (!PEM_write_bio_EC_PUBKEY(bio, d_PublicKey))
#endif
        {
            LOG(WARNING) << "OpenSSL: Failed to write public key to BIO";
            BIO_free(bio);
            return false;
        }

    char* bio_data;
    auto bio_len = BIO_get_mem_data(bio, &bio_data);
    if (bio_len <= 0)
        {
            LOG(WARNING) << "OpenSSL: Failed to get BIO data";
            BIO_free(bio);
            return false;
        }

    pubKeyFile.write(bio_data, bio_len);
    pubKeyFile.close();
    BIO_free(bio);
#endif
    return true;
}


bool Gnss_Crypto::verify_signature_ecdsa_p256(const std::vector<uint8_t>& message, const std::vector<uint8_t>& signature) const
{
    if (!have_public_key())
        {
            LOG(WARNING) << "Galileo OSNMA KROOT verification error: Public key is not available";
            return false;
        }
    std::vector<uint8_t> digest = this->compute_SHA_256(message);
    bool success = false;
#if USE_GNUTLS_FALLBACK
#if HAVE_GNUTLS_SIGN_ECDSA_SHA256
    // Convert signature to DER format
    std::vector<uint8_t> der_sig;
    if (!convert_raw_to_der_ecdsa(signature, der_sig))
        {
            LOG(WARNING) << "Failed to convert raw ECDSA signature to DER format";
            return false;
        }

    // Prepare the digest datum
    gnutls_datum_t digest_data = {const_cast<unsigned char*>(digest.data()), static_cast<unsigned int>(digest.size())};
    gnutls_datum_t der_sig_data = {der_sig.data(), static_cast<unsigned int>(der_sig.size())};

    // Verify the DER-encoded signature
    int ret = gnutls_pubkey_verify_hash2(d_PublicKey, GNUTLS_SIGN_ECDSA_SHA256, 0, &digest_data, &der_sig_data);
    success = (ret >= 0);
    if (success)
        {
            DLOG(INFO) << "GnuTLS: OSNMA signature authenticated successfully";
        }
    else
        {
            LOG(WARNING) << "GnuTLS: OSNMA message authentication failed: " << gnutls_strerror(ret);
        }
#else
    if (signature.empty())
        {
            // do nothing
        }
#endif
#else  // OpenSSL
#if USE_OPENSSL_3
    EVP_PKEY_CTX* ctx;
    ctx = EVP_PKEY_CTX_new(d_PublicKey, nullptr);
    bool do_operation = true;

    if (!ctx)
        {
            do_operation = false;
        }
    // convert raw signature into DER format
    size_t half_size = signature.size() / 2;
    std::vector<uint8_t> raw_r(signature.begin(), signature.begin() + half_size);
    std::vector<uint8_t> raw_s(signature.begin() + half_size, signature.end());

    // Convert raw R and S to BIGNUMs
    BIGNUM* r = BN_bin2bn(raw_r.data(), raw_r.size(), nullptr);
    BIGNUM* s = BN_bin2bn(raw_s.data(), raw_s.size(), nullptr);

    ECDSA_SIG* sig = ECDSA_SIG_new();
    if (r == nullptr || s == nullptr || sig == nullptr)
        {
            LOG(WARNING) << "OpenSSL: Failed to allocate memory for BIGNUMs or ECDSA_SIG";
            return false;
        }

    if (ECDSA_SIG_set0(sig, r, s) != 1)
        {
            LOG(WARNING) << "OpenSSL: Failed to set R and S values in ECDSA_SIG";
            ECDSA_SIG_free(sig);  // Free the ECDSA_SIG struct as it is no longer needed
            return false;
        }

    std::vector<uint8_t> derSignature;
    unsigned char* derSig = nullptr;
    int derSigLength = i2d_ECDSA_SIG(sig, &derSig);

    if (derSigLength <= 0)
        {
            LOG(WARNING) << "OpenSSL: Failed to convert ECDSA_SIG to DER format";
            return false;
        }

    derSignature.assign(derSig, derSig + derSigLength);

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
            verification = EVP_PKEY_verify(ctx, derSignature.data(), derSignature.size(), digest.data(), digest.size());
        }
    EVP_PKEY_CTX_free(ctx);
    OPENSSL_free(derSig);
    ECDSA_SIG_free(sig);
    if (verification == 1)
        {
            success = true;
            DLOG(INFO) << "OpenSSL: OSNMA signature authenticated successfully";
        }
    else
        {
            uint64_t errCode = ERR_get_error();
            char* err = ERR_error_string(errCode, nullptr);
            LOG(WARNING) << "OpenSSL: OSNMA message authentication failed: " << err;
        }
#else  // OpenSSL 1.x
    std::vector<uint8_t> der_sig;
    if (!convert_raw_to_der_ecdsa(signature, der_sig))
        {
            LOG(WARNING) << "OpenSSL: Failed to convert raw ECDSA signature to DER format";
            return false;
        }
    int verification = ECDSA_verify(0, digest.data(), SHA256_DIGEST_LENGTH, der_sig.data(), static_cast<int>(der_sig.size()), d_PublicKey);
    if (verification == 1)
        {
            success = true;
            DLOG(INFO) << "OpenSSL: OSNMA signature authenticated successfully";
        }
    else if (verification == 0)
        {
            LOG(WARNING) << "OpenSSL: invalid signature found when verifying message";
        }
    else
        {
            LOG(WARNING) << "OpenSSL: OSNMA message authentication failed";
        }
#endif
#endif
    return success;
}


bool Gnss_Crypto::verify_signature_ecdsa_p521(const std::vector<uint8_t>& message, const std::vector<uint8_t>& signature) const
{
    if (!have_public_key())
        {
            LOG(WARNING) << "Galileo OSNMA KROOT verification error: Public key is not available";
            return false;
        }

    if (signature.size() != 132)
        {
            LOG(WARNING) << "Invalid signature length for P-521. Expected 132 bytes, got " << signature.size();
            return false;
        }

    std::vector<uint8_t> der_sig;
    if (!convert_raw_to_der_ecdsa(signature, der_sig))
        {
            LOG(WARNING) << "Failed to convert raw ECDSA signature to DER format";
            return false;
        }
    bool success = false;
#if USE_GNUTLS_FALLBACK
#if HAVE_GNUTLS_SIGN_ECDSA_SHA512
    std::vector<uint8_t> digest(64);
    gnutls_hash_hd_t hash;
    int ret = gnutls_hash_init(&hash, GNUTLS_DIG_SHA512);
    if (ret != GNUTLS_E_SUCCESS)
        {
            LOG(WARNING) << "GnuTLS: gnutls_hash_init failed: " << gnutls_strerror(ret);
            return false;
        }

    gnutls_hash(hash, message.data(), message.size());
    gnutls_hash_deinit(hash, digest.data());

    gnutls_datum_t digest_data = {digest.data(), static_cast<unsigned int>(digest.size())};
    gnutls_datum_t signature_data = {der_sig.data(), static_cast<unsigned int>(der_sig.size())};

    // Verify the ECDSA signature
    ret = gnutls_pubkey_verify_hash2(d_PublicKey, GNUTLS_SIGN_ECDSA_SHA512, 0, &digest_data, &signature_data);

    if (ret >= 0)
        {
            DLOG(INFO) << "GnuTLS: OSNMA signature authenticated successfully";
            success = true;
        }
    else
        {
            LOG(WARNING) << "GnuTLS: OSNMA message authentication failed: " << gnutls_strerror(ret);
        }
#else
    if (message.empty())
        {
            // do nothing
        }
#endif
#else  // OpenSSL
    // Compute SHA-512 hash of the message
    std::vector<uint8_t> digest(SHA512_DIGEST_LENGTH);
    if (!EVP_Digest(message.data(), message.size(), digest.data(), nullptr, EVP_sha512(), nullptr))
        {
            LOG(WARNING) << "OpenSSL: EVP_Digest failed";
            return false;
        }
#if USE_OPENSSL_3
    // Verify the signature
    EVP_PKEY_CTX* pctx = EVP_PKEY_CTX_new(d_PublicKey, nullptr);
    if (pctx == nullptr)
        {
            LOG(WARNING) << "OpenSSL: EVP_PKEY_CTX_new failed";
            return false;
        }

    if (EVP_PKEY_verify_init(pctx) <= 0)
        {
            LOG(WARNING) << "OpenSSL: EVP_PKEY_verify_init failed";
            EVP_PKEY_CTX_free(pctx);
            return false;
        }

    if (EVP_PKEY_CTX_set_signature_md(pctx, EVP_sha512()) <= 0)
        {
            LOG(WARNING) << "OpenSSL: EVP_PKEY_CTX_set_signature_md failed";
            EVP_PKEY_CTX_free(pctx);
            return false;
        }

    int verification = EVP_PKEY_verify(pctx, der_sig.data(), der_sig.size(), digest.data(), digest.size());
    EVP_PKEY_CTX_free(pctx);

    if (verification == 1)
        {
            DLOG(INFO) << "OpenSSL: OSNMA signature authenticated successfully";
            success = true;
        }
    else if (verification == 0)
        {
            LOG(WARNING) << "OpenSSL: invalid signature found when verifying message";
        }
    else
        {
            LOG(WARNING) << "OpenSSL: OSNMA message authentication failed";
        }
#else  // OpenSSL 1.x
    const unsigned char* sig_ptr = der_sig.data();
    ECDSA_SIG* ecdsa_sig = d2i_ECDSA_SIG(nullptr, &sig_ptr, der_sig.size());
    if (ecdsa_sig == nullptr)
        {
            LOG(WARNING) << "OpenSSL: d2i_ECDSA_SIG failed";
            return false;
        }
    int verification = ECDSA_do_verify(digest.data(), digest.size(), ecdsa_sig, d_PublicKey);
    ECDSA_SIG_free(ecdsa_sig);
    if (verification == 1)
        {
            DLOG(INFO) << "OpenSSL: OSNMA signature authenticated successfully";
            success = true;
        }
    else if (verification == 0)
        {
            LOG(WARNING) << "OpenSSL: invalid signature found when verifying message";
        }
    else
        {
            LOG(WARNING) << "OpenSSL: OSNMA message authentication failed";
        }
#endif
#endif
    return success;
}


std::vector<uint8_t> Gnss_Crypto::compute_SHA_256(const std::vector<uint8_t>& input) const
{
    std::vector<uint8_t> output(32);  // SHA256 hash size
#if USE_GNUTLS_FALLBACK
    std::vector<uint8_t> output_aux(32);
    gnutls_hash_hd_t hashHandle;
    gnutls_hash_init(&hashHandle, GNUTLS_DIG_SHA256);
    gnutls_hash(hashHandle, input.data(), input.size());
    gnutls_hash_output(hashHandle, output_aux.data());
    output = output_aux;
    gnutls_hash_deinit(hashHandle, output_aux.data());
#else  // OpenSSL
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
#else  // OpenSSL 1.x
    SHA256_CTX sha256Context;
    SHA256_Init(&sha256Context);
    SHA256_Update(&sha256Context, input.data(), input.size());
    SHA256_Final(output.data(), &sha256Context);
#endif
#endif
    return output;
}


std::vector<uint8_t> Gnss_Crypto::compute_SHA3_256(const std::vector<uint8_t>& input) const
{
    std::vector<uint8_t> output(32);  // SHA256 hash size
#if USE_GNUTLS_FALLBACK
#if HAVE_GNUTLS_DIG_SHA3_256
    std::vector<uint8_t> output_aux(32);
    gnutls_hash_hd_t hashHandle;
    gnutls_hash_init(&hashHandle, GNUTLS_DIG_SHA3_256);
    gnutls_hash(hashHandle, input.data(), input.size());
    gnutls_hash_output(hashHandle, output_aux.data());
    output = output_aux;
    gnutls_hash_deinit(hashHandle, output_aux.data());
#else
    if (input.empty())
        {
            // do nothing
        }
#endif
#else  // OpenSSL
#if USE_OPENSSL_3 || USE_OPENSSL_111
    EVP_MD_CTX* mdctx = EVP_MD_CTX_new();
    const EVP_MD* md = EVP_sha3_256();

    EVP_DigestInit_ex(mdctx, md, nullptr);
    EVP_DigestUpdate(mdctx, input.data(), input.size());
    EVP_DigestFinal_ex(mdctx, output.data(), nullptr);
    EVP_MD_CTX_free(mdctx);
#else  // OpenSSL 1.x
    // SHA3-256 not implemented in OpenSSL 1.0, it was introduced in OpenSSL 1.1.1
    if (!input.empty())
        {
            // do nothing
        }
#endif
#endif
    return output;
}


std::vector<uint8_t> Gnss_Crypto::compute_HMAC_SHA_256(const std::vector<uint8_t>& key, const std::vector<uint8_t>& input) const
{
    std::vector<uint8_t> output(32);
#if USE_GNUTLS_FALLBACK
    std::vector<uint8_t> output_aux(32);
    gnutls_hmac_hd_t hmac;
#if HAVE_GNUTLS_HMAC_INIT_WITH_DIGEST
    gnutls_hmac_init(&hmac, GNUTLS_DIG_SHA256, key.data(), key.size());
#else
    gnutls_hmac_init(&hmac, GNUTLS_MAC_SHA256, key.data(), key.size());
#endif
    gnutls_hmac(hmac, input.data(), input.size());
    gnutls_hmac_output(hmac, output_aux.data());
    output = output_aux;
    gnutls_hmac_deinit(hmac, output_aux.data());
#else  // OpenSSL
#if USE_OPENSSL_3
    std::vector<uint8_t> hmac(EVP_MAX_MD_SIZE);
    size_t output_length = 0;
    // Create the context for the HMAC operation
    EVP_MAC* mac = EVP_MAC_fetch(nullptr, "HMAC", nullptr);
    if (!mac)
        {
            LOG(WARNING) << "OSNMA HMAC_SHA_256 computation failed to fetch HMAC";
            return output;
        }

    EVP_MAC_CTX* ctx = EVP_MAC_CTX_new(mac);
    if (!ctx)
        {
            EVP_MAC_free(mac);
            LOG(WARNING) << "OSNMA HMAC_SHA_256 computation failed to create HMAC context";
            return output;
        }

    // Initialize the HMAC context with the key and the SHA-256 algorithm
    OSSL_PARAM params[] = {
        OSSL_PARAM_construct_utf8_string(OSSL_ALG_PARAM_DIGEST, const_cast<char*>("SHA256"), 0),
        OSSL_PARAM_construct_end()};

    if (EVP_MAC_init(ctx, key.data(), key.size(), params) <= 0)
        {
            EVP_MAC_CTX_free(ctx);
            EVP_MAC_free(mac);
            LOG(WARNING) << "OSNMA HMAC_SHA_256 computation failed to initialize HMAC context";
            return output;
        }

    // Update the HMAC context with the input data
    if (EVP_MAC_update(ctx, input.data(), input.size()) <= 0)
        {
            EVP_MAC_CTX_free(ctx);
            EVP_MAC_free(mac);
            LOG(WARNING) << "OSNMA HMAC_SHA_256 computation failed to update HMAC context";
            return output;
        }

    // Finalize the HMAC and retrieve the output
    if (EVP_MAC_final(ctx, hmac.data(), &output_length, hmac.size()) <= 0)
        {
            EVP_MAC_CTX_free(ctx);
            EVP_MAC_free(mac);
            LOG(WARNING) << "OSNMA HMAC_SHA_256 computation failed to finalize HMAC";
            return output;
        }

    // Clean up the HMAC context
    EVP_MAC_CTX_free(ctx);
    EVP_MAC_free(mac);
    hmac.resize(output_length);
    output = hmac;
#else  // OpenSSL 1.x
    unsigned int outputLength = EVP_MAX_MD_SIZE;
    unsigned char* result = HMAC(EVP_sha256(), key.data(), key.size(), input.data(), input.size(), output.data(), &outputLength);
    if (result == nullptr)
        {
            LOG(WARNING) << "OSNMA HMAC_SHA_256 computation failed to compute HMAC-SHA256";
            return output;
        }

    // Resize the output vector to the actual length of the HMAC-SHA256 output
    output.resize(outputLength);
#endif
#endif
    return output;
}


std::vector<uint8_t> Gnss_Crypto::compute_CMAC_AES(const std::vector<uint8_t>& key, const std::vector<uint8_t>& input) const
{
    std::vector<uint8_t> output(16);
#if USE_GNUTLS_FALLBACK
#if HAVE_GNUTLS_MAC_AES_CMAC_128
    gnutls_hmac_hd_t hmac;

    // Initialize the HMAC context with the CMAC algorithm and key
    int ret = gnutls_hmac_init(&hmac, GNUTLS_MAC_AES_CMAC_128, key.data(), key.size());
    if (ret != GNUTLS_E_SUCCESS)
        {
            LOG(WARNING) << "OSNMA CMAC-AES: gnutls_hmac_init failed: " << gnutls_strerror(ret);
            return output;
        }

    // Update the HMAC context with the input data
    ret = gnutls_hmac(hmac, input.data(), input.size());
    if (ret != GNUTLS_E_SUCCESS)
        {
            LOG(WARNING) << "OSNMA CMAC-AES: gnutls_hmac failed: " << gnutls_strerror(ret);
            gnutls_hmac_deinit(hmac, nullptr);
            return output;
        }

    // Retrieve the HMAC output
    gnutls_hmac_output(hmac, output.data());

    // Clean up the HMAC context
    gnutls_hmac_deinit(hmac, nullptr);
#else
    if (!key.empty())
        {
            // do nothing
        }
    if (!input.empty())
        {
            // do nothing
        }
#endif
#else  // OpenSSL
#if USE_OPENSSL_3
    std::vector<uint8_t> aux(EVP_MAX_MD_SIZE);  // CMAC-AES output size
    size_t output_length = 0;

    // Create the context for the CMAC operation
    EVP_MAC* mac = EVP_MAC_fetch(nullptr, "CMAC", nullptr);
    if (!mac)
        {
            LOG(WARNING) << "OSNMA CMAC-AES: Failed to fetch CMAC";
            return output;
        }

    EVP_MAC_CTX* ctx = EVP_MAC_CTX_new(mac);
    if (!ctx)
        {
            LOG(WARNING) << "OSNMA CMAC-AES: Failed to create CMAC context";
            return output;
        }

    // Initialize the CMAC context with the key and the AES algorithm
    OSSL_PARAM params[] = {
        OSSL_PARAM_construct_utf8_string(OSSL_MAC_PARAM_CIPHER, const_cast<char*>("AES-128-CBC"), 0),
        OSSL_PARAM_construct_octet_string(OSSL_MAC_PARAM_KEY, const_cast<unsigned char*>(key.data()), key.size()),
        OSSL_PARAM_construct_end()};

    if (EVP_MAC_init(ctx, nullptr, 0, params) <= 0)
        {
            EVP_MAC_CTX_free(ctx);
            EVP_MAC_free(mac);
            LOG(WARNING) << "OSNMA CMAC-AES: Failed to initialize CMAC context";
            return output;
        }

    // Update the CMAC context with the input data
    if (EVP_MAC_update(ctx, input.data(), input.size()) <= 0)
        {
            EVP_MAC_CTX_free(ctx);
            EVP_MAC_free(mac);
            LOG(WARNING) << "OSNMA CMAC-AES: Failed to update CMAC context";
            return output;
        }

    // Finalize the CMAC and retrieve the output
    if (EVP_MAC_final(ctx, aux.data(), &output_length, aux.size()) <= 0)
        {
            EVP_MAC_CTX_free(ctx);
            EVP_MAC_free(mac);
            LOG(WARNING) << "OSNMA CMAC-AES: Failed to finalize CMAC";
            return output;
        }

    // Clean up the CMAC context
    EVP_MAC_CTX_free(ctx);
    EVP_MAC_free(mac);

    aux.resize(output_length);
    output = aux;
#else  // OpenSSL 1.x
    size_t mac_length = 0;  // to hold the length of the MAC output

    // Create CMAC context
    CMAC_CTX* cmacCtx = CMAC_CTX_new();
    if (!cmacCtx)
        {
            LOG(WARNING) << "OSNMA CMAC-AES: Failed to create CMAC context";
            return output;
        }

    // Initialize the CMAC context with the key and cipher
    if (CMAC_Init(cmacCtx, key.data(), key.size(), EVP_aes_128_cbc(), nullptr) != 1)
        {
            LOG(WARNING) << "OSNMA CMAC-AES: MAC_Init failed";
            CMAC_CTX_free(cmacCtx);
            return output;
        }

    // Compute the CMAC
    if (CMAC_Update(cmacCtx, input.data(), input.size()) != 1)
        {
            LOG(WARNING) << "OSNMA CMAC-AES: CMAC_Update failed";
            CMAC_CTX_free(cmacCtx);
            return output;
        }

    // Finalize the CMAC computation and retrieve the output
    if (CMAC_Final(cmacCtx, output.data(), &mac_length) != 1)
        {
            LOG(WARNING) << "OSNMA CMAC-AES: CMAC_Final failed";
            CMAC_CTX_free(cmacCtx);
            return output;
        }

    // Clean up CMAC context
    CMAC_CTX_free(cmacCtx);

    // Ensure the output vector is properly sized according to the actual MAC length
    output.resize(mac_length);
#endif
#endif
    return output;
}


std::vector<uint8_t> Gnss_Crypto::get_merkle_root() const
{
    return d_x_4_0;
}


std::string Gnss_Crypto::get_public_key_type() const
{
    if (d_PublicKeyType.empty())
        {
            return std::string("Unknown");
        }
    return d_PublicKeyType;
}


void Gnss_Crypto::set_public_key(const std::vector<uint8_t>& publicKey)
{
    d_PublicKeyType = std::string("Unknown");
#if USE_GNUTLS_FALLBACK
    gnutls_pubkey_t pubkey{};
    gnutls_ecc_curve_t curve;
    std::vector<uint8_t> x;
    std::vector<uint8_t> y;

    gnutls_pubkey_init(&pubkey);
    const size_t size_pk = publicKey.size();
    if (size_pk == 33)
        {
            curve = GNUTLS_ECC_CURVE_SECP256R1;
            d_PublicKeyType = std::string("ECDSA P-256");
            decompress_public_key_secp256r1(publicKey, x, y);
        }
    else if (size_pk == 67)
        {
            curve = GNUTLS_ECC_CURVE_SECP521R1;
            d_PublicKeyType = std::string("ECDSA P-521");
            decompress_public_key_secp521r1(publicKey, x, y);
        }
    else
        {
            LOG(WARNING) << "GnuTLS: Invalid public key size";
            gnutls_pubkey_deinit(pubkey);
            return;
        }

    gnutls_datum_t x_coord = {x.data(), static_cast<unsigned int>(x.size())};
    gnutls_datum_t y_coord = {y.data(), static_cast<unsigned int>(y.size())};

    int ret = gnutls_pubkey_import_ecc_raw(pubkey, curve, &x_coord, &y_coord);
    if (ret != GNUTLS_E_SUCCESS)
        {
            gnutls_pubkey_deinit(pubkey);
            LOG(WARNING) << "GnuTLS: error setting the OSNMA public key: " << gnutls_strerror(ret);
            d_PublicKeyType = std::string("Unknown");
            return;
        }
    pubkey_copy(pubkey, &d_PublicKey);
    gnutls_pubkey_deinit(pubkey);
#else  // OpenSSL

#if USE_OPENSSL_3
    // Uses the new EVP_PKEY envelope as well as the parameter builder functions
    // generate the uncompressed key, then add it into the EVP_PKEY* struct
    EVP_PKEY* pkey = nullptr;
    EVP_PKEY_CTX* ctx = nullptr;
    OSSL_PARAM_BLD* param_bld;
    OSSL_PARAM* params = nullptr;
    const size_t public_key_size = publicKey.size();

    param_bld = OSSL_PARAM_BLD_new();
    if (param_bld != nullptr &&
        OSSL_PARAM_BLD_push_utf8_string(param_bld, "group", (public_key_size == 33) ? "prime256v1" : "secp521r1", 0) &&
        OSSL_PARAM_BLD_push_octet_string(param_bld, "pub", publicKey.data(), public_key_size))
        {
            params = OSSL_PARAM_BLD_to_param(param_bld);
        }

    if (public_key_size == 33)
        {
            d_PublicKeyType = std::string("ECDSA P-256");
        }
    else if (public_key_size == 67)
        {
            d_PublicKeyType = std::string("ECDSA P-521");
        }

    ctx = EVP_PKEY_CTX_new_from_name(nullptr, "EC", nullptr);
    if (ctx == nullptr || params == nullptr || EVP_PKEY_fromdata_init(ctx) <= 0 || EVP_PKEY_fromdata(ctx, &pkey, EVP_PKEY_PUBLIC_KEY, params) <= 0)
        {
            EVP_PKEY_free(pkey);
            EVP_PKEY_CTX_free(ctx);
            OSSL_PARAM_free(params);
            OSSL_PARAM_BLD_free(param_bld);
            d_PublicKeyType = std::string("Unknown");
            return;
        }

    if (!pubkey_copy(pkey, &d_PublicKey))
        {
            EVP_PKEY_free(pkey);
            EVP_PKEY_CTX_free(ctx);
            OSSL_PARAM_free(params);
            OSSL_PARAM_BLD_free(param_bld);
            d_PublicKeyType = std::string("Unknown");
            return;
        }

    EVP_PKEY_free(pkey);
    EVP_PKEY_CTX_free(ctx);
    OSSL_PARAM_free(params);
    OSSL_PARAM_BLD_free(param_bld);
#else   // OpenSSL 1.x
    EC_KEY* ec_key = nullptr;    // ECC Key pair
    EC_POINT* point = nullptr;   // Represents the point in the EC the public key belongs to
    EC_GROUP* group = nullptr;   // Defines the curve the public key belongs
    if (publicKey.size() == 33)  // ECDSA-P-256
        {
            group = EC_GROUP_new_by_curve_name(NID_X9_62_prime256v1);
            d_PublicKeyType = std::string("ECDSA P-256");
        }
    else  // ECDSA-P-521
        {
            group = EC_GROUP_new_by_curve_name(NID_secp521r1);
            d_PublicKeyType = std::string("ECDSA P-521");
        }
    if (!group)
        {
            d_PublicKeyType = std::string("Unknown");
            return;
        }

    point = EC_POINT_new(group);
    if (!point)
        {
            EC_GROUP_free(group);
            d_PublicKeyType = std::string("Unknown");
            return;
        }

    if (!EC_POINT_oct2point(group, point, publicKey.data(), publicKey.size(), nullptr))
        {
            EC_GROUP_free(group);
            EC_POINT_free(point);
            d_PublicKeyType = std::string("Unknown");
            return;
        }

    if (publicKey.size() == 33)  // ECDSA-P-256
        {
            ec_key = EC_KEY_new_by_curve_name(NID_X9_62_prime256v1);
        }
    else  // ECDSA-P-521
        {
            ec_key = EC_KEY_new_by_curve_name(NID_secp521r1);
        }
    if (!ec_key)
        {
            EC_GROUP_free(group);
            EC_POINT_free(point);
            d_PublicKeyType = std::string("Unknown");
            return;
        }
    if (!EC_KEY_set_public_key(ec_key, point))
        {
            EC_KEY_free(ec_key);
            EC_POINT_free(point);
            EC_GROUP_free(group);
            d_PublicKeyType = std::string("Unknown");
            return;
        }
    if (!pubkey_copy(ec_key, &d_PublicKey))
        {
            EC_KEY_free(ec_key);
            EC_POINT_free(point);
            EC_GROUP_free(group);
            d_PublicKeyType = std::string("Unknown");
            return;
        }
    EC_KEY_free(ec_key);
    EC_POINT_free(point);
    EC_GROUP_free(group);
#endif  // OpenSSL 1.x
#endif
    DLOG(INFO) << "OSNMA Public Key successfully set up.";
}


void Gnss_Crypto::set_public_key_type(const std::string& public_key_type)
{
    if (public_key_type == "ECDSA P-256" || public_key_type == "ECDSA P-521")
        {
            d_PublicKeyType = public_key_type;
        }
}


void Gnss_Crypto::set_merkle_root(const std::vector<uint8_t>& v)
{
    d_x_4_0 = v;
}


void Gnss_Crypto::read_merkle_xml(const std::string& merkleFilePath)
{
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(merkleFilePath.c_str());
    if (!result)
        {
            // XML file not found
            // If it was not the default, maybe it is a configuration error, warn user
            if (merkleFilePath != MERKLEFILE_DEFAULT && !merkleFilePath.empty())
                {
                    LOG(WARNING) << "File " << merkleFilePath << " not found";
                }
            // fill default values
            d_x_4_0 = convert_from_hex_str("832E15EDE55655EAC6E399A539477B7C034CCE24C3C93FFC904ACD9BF842F04E");
            return;
        }
    try
        {
            pugi::xml_node root = doc.child("signalData");
            pugi::xml_node header = root.child("header");
            pugi::xml_node body = root.child("body");

            // Accessing data from the header
            pugi::xml_node galHeader = header.child("GAL-header");
            pugi::xml_node source = galHeader.child("source").child("GAL-EXT-GOC-SC-GLAd");
            pugi::xml_node destination = galHeader.child("destination").child("GAL-EXT-GOC-SC-GLAd");
            std::string issueDate = galHeader.child("issueDate").text().get();
            std::string signalVersion = galHeader.child("signalVersion").text().get();
            std::string dataVersion = galHeader.child("dataVersion").text().get();

            LOG(INFO) << "OSNMA Merkletree - Source: " << source.child_value("mission") << " - " << source.child_value("segment") << " - " << source.child_value("element");
            LOG(INFO) << "OSNMA Merkletree - Destination: " << destination.child_value("mission") << " - " << destination.child_value("segment") << " - " << destination.child_value("element");
            LOG(INFO) << "OSNMA Merkletree - Issue Date: " << issueDate;
            LOG(INFO) << "OSNMA Merkletree - Signal Version: " << signalVersion;
            LOG(INFO) << "OSNMA Merkletree - Data Version: " << dataVersion;

            // Accessing data from the body
            pugi::xml_node merkleTree = body.child("MerkleTree");

            int n = std::stoi(merkleTree.child_value("N"));
            std::string hashFunction = merkleTree.child_value("HashFunction");

            LOG(INFO) << "OSNMA Merkletree - N: " << n;
            LOG(INFO) << "OSNMA Merkletree - Hash Function: " << hashFunction;

            for (pugi::xml_node publicKey : merkleTree.children("PublicKey"))
                {
                    int i = std::stoi(publicKey.child_value("i"));
                    std::string pkid = publicKey.child_value("PKID");
                    int lengthInBits = std::stoi(publicKey.child_value("lengthInBits"));
                    std::string point = publicKey.child_value("point");
                    std::string pkType = publicKey.child_value("PKType");

                    LOG(INFO) << "OSNMA Merkletree - Public Key: " << i;
                    LOG(INFO) << "OSNMA Merkletree - PKID: " << pkid;
                    LOG(INFO) << "OSNMA Merkletree - Length in Bits: " << lengthInBits;
                    LOG(INFO) << "OSNMA Merkletree - Point: " << point;
                    LOG(INFO) << "OSNMA Merkletree - PK Type: " << pkType;
                    if (pkType == "ECDSA P-256/SHA-256")
                        {
                            d_PublicKeyType = std::string("ECDSA P-256");
                        }
                    else if (pkType == "ECDSA P-521/SHA-512")
                        {
                            d_PublicKeyType = std::string("ECDSA P-521");
                        }
                }
            for (pugi::xml_node treeNode : merkleTree.children("TreeNode"))
                {
                    int j = std::stoi(treeNode.child_value("j"));
                    int i = std::stoi(treeNode.child_value("i"));
                    int lengthInBits = std::stoi(treeNode.child_value("lengthInBits"));
                    LOG(INFO) << "OSNMA Merkletree - Node length (bits): " << lengthInBits;
                    std::string x_ji = treeNode.child_value("x_ji");
                    LOG(INFO) << "OSNMA Merkletree - Size string (bytes): " << x_ji.size();
                    LOG(INFO) << "OSNMA Merkletree - m_" << j << "_" << i << " = " << x_ji;
                    if (j == 4 && i == 0)
                        {
                            d_x_4_0 = convert_from_hex_str(x_ji);
                        }
                }
        }
    catch (const std::exception& e)
        {
            LOG(INFO) << "Exception raised reading the " << merkleFilePath << " file: " << e.what();
            d_x_4_0 = convert_from_hex_str("832E15EDE55655EAC6E399A539477B7C034CCE24C3C93FFC904ACD9BF842F04E");
            return;
        }
    std::cout << "OSNMA Merkle Tree successfully read from file " << merkleFilePath << std::endl;
    LOG(INFO) << "OSNMA Merkle Tree successfully read from file " << merkleFilePath;
}


void Gnss_Crypto::readPublicKeyFromPEM(const std::string& pemFilePath)
{
    // Open the .pem file
    std::ifstream pemFile(pemFilePath);
    if (!pemFile)
        {
            return;
        }
    d_PublicKeyType = std::string("Unknown");
    std::string pemContent((std::istreambuf_iterator<char>(pemFile)), std::istreambuf_iterator<char>());
#if USE_GNUTLS_FALLBACK
    // Import the PEM data
    gnutls_datum_t pemDatum = {const_cast<unsigned char*>(reinterpret_cast<unsigned char*>(const_cast<char*>(pemContent.data()))), static_cast<unsigned int>(pemContent.size())};
    gnutls_pubkey_t pubkey;
    gnutls_pubkey_init(&pubkey);

    int ret = gnutls_pubkey_import(pubkey, &pemDatum, GNUTLS_X509_FMT_PEM);
    if (ret != GNUTLS_E_SUCCESS)
        {
            gnutls_pubkey_deinit(pubkey);
            std::cerr << "GnuTLS: error reading the OSNMA Public Key from file "
                      << pemFilePath
                      << ". Aborting import" << std::endl;
            LOG(WARNING) << "GnuTLS: error reading the OSNMA Public Key from file "
                         << pemFilePath << ". Aborting import. Error " << gnutls_strerror(ret);
            return;
        }

    // store the key type - needed for the Kroot in case no DSM-PKR available
    gnutls_pk_algorithm_t pk_algorithm;
    unsigned int bits;

    ret = gnutls_pubkey_get_pk_algorithm(pubkey, &bits);
    if (ret < 0)
        {
            LOG(WARNING) << "GnuTLS: Failed to get public key algorithm from .pem file: " << gnutls_strerror(ret);
            gnutls_pubkey_deinit(pubkey);
            return;
        }

    pk_algorithm = static_cast<gnutls_pk_algorithm_t>(ret);
    if (pk_algorithm == GNUTLS_PK_ECC)
        {
            gnutls_ecc_curve_t curve;
            ret = gnutls_pubkey_export_ecc_raw(pubkey, &curve, nullptr, nullptr);
            if (ret < 0)
                {
                    LOG(WARNING) << "GnuTLS: Failed to get EC curve from .pem file: " << gnutls_strerror(ret);
                    gnutls_pubkey_deinit(pubkey);
                    return;
                }

            if (curve == GNUTLS_ECC_CURVE_SECP256R1)
                {
                    d_PublicKeyType = std::string("ECDSA P-256");
                }
            else if (curve == GNUTLS_ECC_CURVE_SECP521R1)
                {
                    d_PublicKeyType = std::string("ECDSA P-521");
                }
            else
                {
                    LOG(WARNING) << "GnuTLS: Trying to read unknown EC curve from .pem file";
                    gnutls_pubkey_deinit(pubkey);
                    return;
                }
        }
    else
        {
            LOG(WARNING) << "GnuTLS: Trying to read unknown key type from .pem file";
            gnutls_pubkey_deinit(pubkey);
            return;
        }
    pubkey_copy(pubkey, &d_PublicKey);
    gnutls_pubkey_deinit(pubkey);
#else  // OpenSSL
    // Create a BIO object from the string data
    BIO* bio = BIO_new_mem_buf(const_cast<char*>(pemContent.c_str()), pemContent.length());
    if (!bio)
        {
            LOG(WARNING) << "OpenSSL: error creating a BIO object with data read from file " << pemFilePath << ". Aborting import.";
            return;
        }
#if USE_OPENSSL_3
    EVP_PKEY* pubkey = nullptr;
    pubkey = PEM_read_bio_PUBKEY(bio, nullptr, nullptr, nullptr);

    // store the key type - needed for the Kroot in case no DSM-PKR available
    // Get the key type
    int key_type = EVP_PKEY_base_id(pubkey);
    if (key_type == EVP_PKEY_EC)
        {
            // It's an EC key, now we need to determine the curve
            char curve_name[256];
            size_t curve_name_len = sizeof(curve_name);

            if (EVP_PKEY_get_utf8_string_param(pubkey, OSSL_PKEY_PARAM_GROUP_NAME, curve_name, curve_name_len, &curve_name_len) == 1)
                {
                    if (strcmp(curve_name, "prime256v1") == 0 || strcmp(curve_name, "P-256") == 0)
                        {
                            d_PublicKeyType = std::string("ECDSA P-256");
                        }
                    else if (strcmp(curve_name, "secp521r1") == 0 || strcmp(curve_name, "P-521") == 0)
                        {
                            d_PublicKeyType = std::string("ECDSA P-521");
                        }
                    else
                        {
                            LOG(WARNING) << "OpenSSL: Trying to read an unknown EC curve from .pem file";
                            BIO_free(bio);
                            EVP_PKEY_free(pubkey);
                            return;
                        }
                }
            else
                {
                    LOG(WARNING) << "OpenSSL: Trying to read an unknown EC curve from .pem file";
                    BIO_free(bio);
                    EVP_PKEY_free(pubkey);
                    return;
                }
        }
    else
        {
            LOG(WARNING) << "OpenSSL: Trying to read an unknown key type from .pem file";
            BIO_free(bio);
            EVP_PKEY_free(pubkey);
            return;
        }
    pubkey_copy(pubkey, &d_PublicKey);
    EVP_PKEY_free(pubkey);
#else  // OpenSSL 1.x
    EC_KEY* pubkey = nullptr;
    pubkey = PEM_read_bio_EC_PUBKEY(bio, nullptr, nullptr, nullptr);
    if (!pubkey)
        {
            LOG(WARNING) << "OpenSSL: Failed to extract the public key from .pem file";
            BIO_free(bio);
            return;
        }
    const EC_GROUP* group = EC_KEY_get0_group(pubkey);
    int nid = EC_GROUP_get_curve_name(group);
    if (nid == 0)
        {
            BIGNUM* p = BN_new();
            if (EC_GROUP_get_curve_GFp(group, p, nullptr, nullptr, nullptr) == 1)
                {
                    char* p_str = BN_bn2hex(p);
                    const std::string pcstr(p_str);
                    if (pcstr.size() == 64)
                        {
                            d_PublicKeyType = std::string("ECDSA P-256");
                        }
                    else if (pcstr.size() == 132)
                        {
                            d_PublicKeyType = std::string("ECDSA P-521");
                        }
                    OPENSSL_free(p_str);
                }
            BN_free(p);
        }
    else
        {
            const char* curve_name = OBJ_nid2sn(nid);
            const std::string curve_str(curve_name);
            if (curve_str == "prime256v1")
                {
                    d_PublicKeyType = std::string("ECDSA P-256");
                }
            else if (curve_str == "secp521r1")
                {
                    d_PublicKeyType = std::string("ECDSA P-521");
                }
        }

    pubkey_copy(pubkey, &d_PublicKey);
    EC_KEY_free(pubkey);
#endif
    BIO_free(bio);
    if (d_PublicKey == nullptr)
        {
            std::cerr << "OpenSSL: error reading the OSNMA Public Key from file " << pemFilePath << ". Aborting import" << std::endl;
            LOG(WARNING) << "OpenSSL: error reading the OSNMA Public Key from file " << pemFilePath << ". Aborting import.";
            return;
        }
#endif
    std::cout << "OSNMA Public key successfully read from file " << pemFilePath << std::endl;
    LOG(INFO) << "OSNMA Public key successfully read from file " << pemFilePath;
}


bool Gnss_Crypto::readPublicKeyFromCRT(const std::string& crtFilePath)
{
    d_PublicKeyType = std::string("Unknown");
    // Open the .crt file
    std::ifstream crtFile(crtFilePath, std::ios::binary);
    if (!crtFile.is_open())
        {
            // CRT file not found
            // If it was not the default, maybe it is a configuration error
            if (crtFilePath != CRTFILE_DEFAULT && !crtFilePath.empty())
                {
                    std::cerr << "File " << crtFilePath << " not found" << std::endl;
                }
            return false;
        }

    const std::vector<unsigned char> buffer((std::istreambuf_iterator<char>(crtFile)), std::istreambuf_iterator<char>());
#if USE_GNUTLS_FALLBACK
    const gnutls_datum_t buffer_datum = {const_cast<unsigned char*>(buffer.data()), static_cast<unsigned int>(buffer.size())};
    gnutls_x509_crt_t cert;
    gnutls_x509_crt_init(&cert);
    int ret = gnutls_x509_crt_import(cert, &buffer_datum, GNUTLS_X509_FMT_PEM);
    if (ret < 0)
        {
            LOG(WARNING) << "GnuTLS: Failed to import certificate: " << gnutls_strerror(ret);
            gnutls_x509_crt_deinit(cert);
            return false;
        }

    gnutls_pubkey_t pubkey;
    gnutls_pubkey_init(&pubkey);

    ret = gnutls_pubkey_import_x509(pubkey, cert, 0);
    if (ret < 0)
        {
            LOG(WARNING) << "GnuTLS: Failed to import public key: " << gnutls_strerror(ret);
            gnutls_pubkey_deinit(pubkey);
            gnutls_x509_crt_deinit(cert);
            return false;
        }

    // store the key type - needed for the Kroot in case no DSM-PKR available
    gnutls_pk_algorithm_t pk_algorithm;
    unsigned int bits;

    ret = gnutls_pubkey_get_pk_algorithm(pubkey, &bits);
    if (ret < 0)
        {
            LOG(WARNING) << "GnuTLS: Failed to get public key algorithm: " << gnutls_strerror(ret);
            gnutls_pubkey_deinit(pubkey);
            gnutls_x509_crt_deinit(cert);
            return false;
        }

    pk_algorithm = static_cast<gnutls_pk_algorithm_t>(ret);
    if (pk_algorithm == GNUTLS_PK_ECC)
        {
            gnutls_ecc_curve_t curve;
            ret = gnutls_pubkey_export_ecc_raw(pubkey, &curve, nullptr, nullptr);
            if (ret < 0)
                {
                    LOG(WARNING) << "GnuTLS: Failed to get EC curve: " << gnutls_strerror(ret);
                    gnutls_pubkey_deinit(pubkey);
                    gnutls_x509_crt_deinit(cert);
                    return false;
                }

            if (curve == GNUTLS_ECC_CURVE_SECP256R1)
                {
                    d_PublicKeyType = std::string("ECDSA P-256");
                }
            else if (curve == GNUTLS_ECC_CURVE_SECP521R1)
                {
                    d_PublicKeyType = std::string("ECDSA P-521");
                }
            else
                {
                    LOG(WARNING) << "GnuTLS: Trying to read unknown EC curve";
                    gnutls_x509_crt_deinit(cert);
                    gnutls_pubkey_deinit(pubkey);
                    return false;
                }
        }
    else
        {
            LOG(WARNING) << "GnuTLS: Trying to read unknown key type";
            gnutls_x509_crt_deinit(cert);
            gnutls_pubkey_deinit(pubkey);
            return false;
        }

    pubkey_copy(pubkey, &d_PublicKey);
    gnutls_x509_crt_deinit(cert);
    gnutls_pubkey_deinit(pubkey);
#else  // OpenSSL
    // Read certificate
    BIO* bio = BIO_new_mem_buf(buffer.data(), buffer.size());
    if (!bio)
        {
            LOG(WARNING) << "OpenSSL: Unable to create BIO for file: " << crtFilePath;
            return false;
        }
    X509* cert = PEM_read_bio_X509(bio, nullptr, nullptr, nullptr);
    if (!cert)
        {
            LOG(WARNING) << "OpenSSL: Unable to read certificate from file: " << crtFilePath;
            BIO_free(bio);
            return false;
        }

    // Read the public key from the certificate
    EVP_PKEY* pubkey = X509_get_pubkey(cert);
    if (!pubkey)
        {
            LOG(WARNING) << "OpenSSL: Failed to extract the public key";
            X509_free(cert);
            BIO_free(bio);
            return false;
        }
#if USE_OPENSSL_3
    // store the key type - needed for the Kroot in case no DSM-PKR available
    // Get the key type
    int key_type = EVP_PKEY_base_id(pubkey);
    if (key_type == EVP_PKEY_EC)
        {
            // It's an EC key, now we need to determine the curve
            char curve_name[256];
            size_t curve_name_len = sizeof(curve_name);

            if (EVP_PKEY_get_utf8_string_param(pubkey, OSSL_PKEY_PARAM_GROUP_NAME, curve_name, curve_name_len, &curve_name_len) == 1)
                {
                    if (strcmp(curve_name, "prime256v1") == 0 || strcmp(curve_name, "P-256") == 0)
                        {
                            d_PublicKeyType = std::string("ECDSA P-256");
                        }
                    else if (strcmp(curve_name, "secp521r1") == 0 || strcmp(curve_name, "P-521") == 0)
                        {
                            d_PublicKeyType = std::string("ECDSA P-521");
                        }
                    else
                        {
                            LOG(WARNING) << "OpenSSL: Trying to read an unknown EC curve";
                            X509_free(cert);
                            BIO_free(bio);
                            return false;
                        }
                }
            else
                {
                    LOG(WARNING) << "OpenSSL: Trying to read an unknown EC curve";
                    X509_free(cert);
                    BIO_free(bio);
                    return false;
                }
        }
    else
        {
            LOG(WARNING) << "OpenSSL: Trying to read an unknown key type";
            X509_free(cert);
            BIO_free(bio);
            return false;
        }
    pubkey_copy(pubkey, &d_PublicKey);
    EVP_PKEY_free(pubkey);
#else  // OpenSSL 1.x
#if USE_OPENSSL_111
    // store the key type - needed for the Kroot in case no DSM-PKR available
    const auto ec_key = EVP_PKEY_get0_EC_KEY(pubkey);
    const EC_GROUP* group = EC_KEY_get0_group(ec_key);
    if (!group)
        {
            X509_free(cert);
            EC_KEY_free(ec_key);
            BIO_free(bio);
            return false;
        }
    const int nid = EC_GROUP_get_curve_name(group);
    if (nid == NID_X9_62_prime256v1)
        {
            d_PublicKeyType = std::string("ECDSA P-256");
        }
    else if (nid == NID_secp521r1)
        {
            d_PublicKeyType = std::string("ECDSA P-521");
        }
    EC_KEY_free(ec_key);
#else
    EC_KEY* ec_key = EVP_PKEY_get1_EC_KEY(pubkey);
    if (!ec_key)
        {
            X509_free(cert);
            BIO_free(bio);
            return false;
        }

    // Get the EC_GROUP from the EC_KEY
    const EC_GROUP* group = EC_KEY_get0_group(ec_key);
    if (!group)
        {
            X509_free(cert);
            EC_KEY_free(ec_key);
            BIO_free(bio);
            return false;
        }
    const int nid = EC_GROUP_get_curve_name(group);
    if (nid == 0)
        {
            BIGNUM* p = BN_new();
            if (EC_GROUP_get_curve_GFp(group, p, nullptr, nullptr, nullptr) == 1)
                {
                    char* p_str = BN_bn2hex(p);
                    const std::string pcstr(p_str);
                    if (pcstr.size() == 64)
                        {
                            d_PublicKeyType = std::string("ECDSA P-256");
                        }
                    else if (pcstr.size() == 132)
                        {
                            d_PublicKeyType = std::string("ECDSA P-521");
                        }
                    OPENSSL_free(p_str);
                }
            BN_free(p);
        }
    else
        {
            const char* curve_name = OBJ_nid2sn(nid);
            const std::string curve_str(curve_name);
            if (curve_str == "prime256v1")
                {
                    d_PublicKeyType = std::string("ECDSA P-256");
                }
            else if (curve_str == "secp521r1")
                {
                    d_PublicKeyType = std::string("ECDSA P-521");
                }
        }
    EC_KEY_free(ec_key);
#endif
    EC_KEY* ec_pubkey = EVP_PKEY_get1_EC_KEY(pubkey);
    EVP_PKEY_free(pubkey);
    if (!ec_pubkey)
        {
            LOG(WARNING) << "OpenSSL: Failed to extract the public key";
            X509_free(cert);
            return false;
        }
    pubkey_copy(ec_pubkey, &d_PublicKey);
    EC_KEY_free(ec_pubkey);
#endif
    BIO_free(bio);
    X509_free(cert);
#endif
    std::cout << "OSNMA Public key successfully read from file " << crtFilePath << std::endl;
    LOG(INFO) << "OSNMA Public key successfully read from file " << crtFilePath;
    return true;
}


bool Gnss_Crypto::convert_raw_to_der_ecdsa(const std::vector<uint8_t>& raw_signature, std::vector<uint8_t>& der_signature) const
{
    if (raw_signature.size() % 2 != 0)
        {
            LOG(WARNING) << "Invalid raw ECDSA signature size";
            return false;
        }

    size_t half_size = raw_signature.size() / 2;
    std::vector<uint8_t> raw_r(raw_signature.begin(), raw_signature.begin() + half_size);
    std::vector<uint8_t> raw_s(raw_signature.begin() + half_size, raw_signature.end());

    auto encode_asn1_integer = [](const std::vector<uint8_t>& value) -> std::vector<uint8_t> {
        std::vector<uint8_t> result;
        result.push_back(0x02);  // INTEGER tag

        if (value[0] & 0x80)
            {
                result.push_back(value.size() + 1);  // Length byte
                result.push_back(0x00);              // Add leading zero byte to ensure positive integer
            }
        else
            {
                result.push_back(value.size());  // Length byte
            }

        result.insert(result.end(), value.begin(), value.end());
        return result;
    };

    std::vector<uint8_t> der_r = encode_asn1_integer(raw_r);
    std::vector<uint8_t> der_s = encode_asn1_integer(raw_s);

    size_t total_length = der_r.size() + der_s.size();
    der_signature.push_back(0x30);  // SEQUENCE tag
    if (total_length > 127)
        {
            der_signature.push_back(0x81);  // Long form length
        }
    der_signature.push_back(static_cast<uint8_t>(total_length));

    der_signature.insert(der_signature.end(), der_r.begin(), der_r.end());
    der_signature.insert(der_signature.end(), der_s.begin(), der_s.end());

    return true;
}


std::vector<uint8_t> Gnss_Crypto::convert_from_hex_str(const std::string& input) const
{
    std::vector<uint8_t> result;

    // Iterate over the input string in pairs
    for (size_t i = 0; i < input.length(); i += 2)
        {
            // Extract two hexadecimal characters from the input string
            std::string hexByte = input.substr(i, 2);

            // Convert the hexadecimal string to an integer value
            auto value = static_cast<uint8_t>(std::stoul(hexByte, nullptr, 16));

            // Append the value to the result vector
            result.push_back(value);
        }

    return result;
}


#if USE_GNUTLS_FALLBACK  // GnuTLS-specific functions
bool Gnss_Crypto::pubkey_copy(gnutls_pubkey_t src, gnutls_pubkey_t* dest)
{
    gnutls_datum_t key_datum;

    // Export the public key from src to memory
#if HAVE_GNUTLS_PUBKEY_EXPORT2
    int ret = gnutls_pubkey_export2(src, GNUTLS_X509_FMT_PEM, &key_datum);
#else
    size_t output_stata_size;
    int ret = gnutls_pubkey_export(src, GNUTLS_X509_FMT_PEM, &key_datum, &output_stata_size);
#endif
    if (ret < 0)
        {
            gnutls_free(key_datum.data);
            return false;
        }

    // Initialize dest
    ret = gnutls_pubkey_init(dest);
    if (ret < 0)
        {
            gnutls_free(key_datum.data);
            return false;
        }

    // Import the public key data from key_datum to dest
    ret = gnutls_pubkey_import(*dest, &key_datum, GNUTLS_X509_FMT_PEM);
    gnutls_free(key_datum.data);

    if (ret < 0)
        {
            gnutls_pubkey_deinit(*dest);
            return false;
        }

    return true;
}


bool tonelli_shanks(mpz_t& res, const mpz_t& n, const mpz_t& p)
{
    if (mpz_legendre(n, p) != 1)
        {
            return false;
        }
    mpz_t q;
    mpz_t s;
    mpz_t z;
    mpz_t m;
    mpz_t c;
    mpz_t t;
    mpz_t r;
    mpz_t b;
    mpz_t two;
    mpz_t p_minus_one;
    mpz_inits(q, s, z, m, c, t, r, b, two, p_minus_one, nullptr);

    mpz_set_ui(two, 2);

    mpz_sub_ui(q, p, 1);
    mpz_set_ui(s, 0);
    while (mpz_even_p(q))
        {
            mpz_add_ui(s, s, 1);
            mpz_divexact_ui(q, q, 2);
        }

    mpz_set_ui(z, 2);
    while (mpz_legendre(z, p) != -1)
        {
            mpz_add_ui(z, z, 1);
        }

    mpz_powm(c, z, q, p);

    mpz_add_ui(p_minus_one, p, 1);
    mpz_divexact_ui(p_minus_one, p_minus_one, 4);
    mpz_powm(r, n, p_minus_one, p);
    mpz_powm(t, n, q, p);
    mpz_set(m, s);

    while (mpz_cmp_ui(t, 1) != 0)
        {
            mpz_set(b, t);
            unsigned int i;
            for (i = 0; mpz_cmp_ui(b, 1) != 0; i++)
                {
                    mpz_powm_ui(b, b, 2, p);
                }
            if (i == mpz_get_ui(m))
                {
                    mpz_clears(q, s, z, m, c, t, r, b, two, p_minus_one, nullptr);
                    return false;
                }

            mpz_powm_ui(b, two, mpz_get_ui(m) - i - 1, p);
            mpz_powm(c, c, b, p);
            mpz_mul(r, r, c);
            mpz_mod(r, r, p);
            mpz_powm_ui(c, c, 2, p);
            mpz_mul(t, t, c);
            mpz_mod(t, t, p);
            mpz_set_ui(m, mpz_get_ui(m) - i - 1);
        }

    mpz_set(res, r);
    mpz_clears(q, s, z, m, c, t, r, b, two, p_minus_one, nullptr);
    return true;
}


void Gnss_Crypto::decompress_public_key_secp256r1(const std::vector<uint8_t>& compressed_key, std::vector<uint8_t>& x, std::vector<uint8_t>& y) const
{
    // Define curve parameters for secp256r1
    const char* p_str = "FFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFF";
    const char* a_str = "FFFFFFFF00000001000000000000000000000000FFFFFFFFFFFFFFFFFFFFFFFC";
    const char* b_str = "5AC635D8AA3A93E7B3EBBD55769886BC651D06B0CC53B0F63BCE3C3E27D2604B";

    mpz_t p;
    mpz_t a;
    mpz_t b;
    mpz_t x_coord;
    mpz_t y_coord;
    mpz_t y_squared;
    mpz_t tmp;
    mpz_inits(p, a, b, x_coord, y_coord, y_squared, tmp, nullptr);

    // Initialize curve parameters
    mpz_set_str(p, p_str, 16);
    mpz_set_str(a, a_str, 16);
    mpz_set_str(b, b_str, 16);

    // Set x coordinate
    mpz_import(x_coord, 32, 1, 1, 1, 0, &compressed_key[1]);

    // Calculate y^2 = x^3 + ax + b (mod p)
    mpz_powm_ui(y_squared, x_coord, 3, p);  // y_squared = x^3
    mpz_mul(tmp, a, x_coord);               // tmp = ax
    mpz_add(y_squared, y_squared, tmp);     // y_squared = x^3 + ax
    mpz_add(y_squared, y_squared, b);       // y_squared = x^3 + ax + b
    mpz_mod(y_squared, y_squared, p);       // y_squared = (x^3 + ax + b) % p

    // Calculate the square root of y_squared to get y
    if (!tonelli_shanks(y_coord, y_squared, p))
        {
            mpz_clears(p, a, b, x_coord, y_coord, y_squared, tmp, nullptr);
            LOG(WARNING) << "GnuTLS: Failed to decompress public key: No valid y coordinate";
            return;
        }

    // Select the correct y coordinate based on the parity bit
    if ((compressed_key[0] & 1) != (mpz_tstbit(y_coord, 0) & 1))
        {
            mpz_sub(y_coord, p, y_coord);  // y = p - y
        }

    // Export the x and y coordinates to vectors
    x.resize(32);
    y.resize(32);
    mpz_export(x.data(), nullptr, 1, 1, 1, 0, x_coord);
    mpz_export(y.data(), nullptr, 1, 1, 1, 0, y_coord);

    mpz_clears(p, a, b, x_coord, y_coord, y_squared, tmp, nullptr);
}


void Gnss_Crypto::decompress_public_key_secp521r1(const std::vector<uint8_t>& compressed_key, std::vector<uint8_t>& x, std::vector<uint8_t>& y) const
{
    // Define curve parameters for secp521r1
    const char* p_str = "01FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF";
    const char* a_str = "01FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFC";
    const char* b_str = "0051953EB9618E1C9A1F929A21A0B68540EEA2DA725B99B315F3B8B489918EF109E156193951EC7E937B1652C0BD3BB1BF073573DF883D2C34F1EF451FD46B503F00";

    mpz_t p;
    mpz_t a;
    mpz_t b;
    mpz_t x_coord;
    mpz_t y_coord;
    mpz_t y_squared;
    mpz_t tmp;
    mpz_inits(p, a, b, x_coord, y_coord, y_squared, tmp, nullptr);

    // Initialize curve parameters
    mpz_set_str(p, p_str, 16);
    mpz_set_str(a, a_str, 16);
    mpz_set_str(b, b_str, 16);

    // Set x coordinate
    mpz_import(x_coord, 66, 1, 1, 1, 0, &compressed_key[1]);

    // Calculate y^2 = x^3 + ax + b (mod p)
    mpz_powm_ui(y_squared, x_coord, 3, p);  // y_squared = x^3
    mpz_mul(tmp, a, x_coord);               // tmp = ax
    mpz_add(y_squared, y_squared, tmp);     // y_squared = x^3 + ax
    mpz_add(y_squared, y_squared, b);       // y_squared = x^3 + ax + b
    mpz_mod(y_squared, y_squared, p);       // y_squared = (x^3 + ax + b) % p

    // Calculate the square root of y_squared to get y
    if (!tonelli_shanks(y_coord, y_squared, p))
        {
            mpz_clears(p, a, b, x_coord, y_coord, y_squared, tmp, nullptr);
            LOG(WARNING) << "GnuTLS: Failed to decompress public key: No valid y coordinate";
            return;
        }

    // Select the correct y coordinate based on the parity bit
    if ((compressed_key[0] & 1) != (mpz_tstbit(y_coord, 0) & 1))
        {
            mpz_sub(y_coord, p, y_coord);  // y = p - y
        }

    // Export the x and y coordinates to vectors
    x.resize(66, 0);  // Ensure 66 bytes with leading zeros if necessary
    y.resize(66, 0);
    mpz_export(x.data() + 1, nullptr, 1, 1, 1, 0, x_coord);
    mpz_export(y.data(), nullptr, 1, 1, 1, 0, y_coord);

    mpz_clears(p, a, b, x_coord, y_coord, y_squared, tmp, nullptr);
}
#else  // OpenSSL
#if USE_OPENSSL_3
bool Gnss_Crypto::pubkey_copy(EVP_PKEY* src, EVP_PKEY** dest)
{
    // Open a memory buffer
    BIO* mem_bio = BIO_new(BIO_s_mem());
    if (mem_bio == nullptr)
        {
            return false;
        }

    // Export the public key from src into the memory buffer in PEM format
    if (!PEM_write_bio_PUBKEY(mem_bio, src))
        {
            BIO_free(mem_bio);
            return false;
        }

    // Read the data from the memory buffer
    char* bio_data;
    int64_t data_len = BIO_get_mem_data(mem_bio, &bio_data);

    // Create a new memory buffer and load the data into it
    BIO* mem_bio2 = BIO_new_mem_buf(bio_data, data_len);
    if (mem_bio2 == nullptr)
        {
            BIO_free(mem_bio);
            return false;
        }

    // Read the public key from the new memory buffer
    *dest = PEM_read_bio_PUBKEY(mem_bio2, nullptr, nullptr, nullptr);
    if (*dest == nullptr)
        {
            BIO_free(mem_bio);
            BIO_free(mem_bio2);
            return false;
        }

    // Clean up
    BIO_free(mem_bio);
    BIO_free(mem_bio2);

    return true;
}
#else   // OpenSSL 1.x
bool Gnss_Crypto::pubkey_copy(EC_KEY* src, EC_KEY** dest)
{
    // Open a memory buffer
    BIO* mem_bio = BIO_new(BIO_s_mem());
    if (mem_bio == nullptr)
        {
            return false;
        }

    // Export the public key from src into the memory buffer in PEM format
    if (!PEM_write_bio_EC_PUBKEY(mem_bio, src))
        {
            BIO_free(mem_bio);
            return false;
        }

    // Read the data from the memory buffer
    char* bio_data;
    long data_len = BIO_get_mem_data(mem_bio, &bio_data);

    // Create a new memory buffer and load the data into it
    BIO* mem_bio2 = BIO_new_mem_buf(bio_data, data_len);
    if (mem_bio2 == nullptr)
        {
            BIO_free(mem_bio);
            return false;
        }

    // Read the public key from the new memory buffer
    *dest = PEM_read_bio_EC_PUBKEY(mem_bio2, nullptr, nullptr, nullptr);
    if (*dest == nullptr)
        {
            BIO_free(mem_bio);
            BIO_free(mem_bio2);
            return false;
        }

    // Clean up
    BIO_free(mem_bio);
    BIO_free(mem_bio2);

    return true;
}
#endif  // OpenSSL
#endif
