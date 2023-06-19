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
#include <pugixml.hpp>
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


Gnss_Crypto::Gnss_Crypto(const std::string& pemFilePath, const std::string& merkleTreePath)
{
#if USE_OPENSSL_FALLBACK
#else
    // gnutls_global_init();
#endif
    readPublicKeyFromPEM(pemFilePath);
    read_merkle_xml(merkleTreePath);
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
#endif
}


bool Gnss_Crypto::have_public_key() const
{
#if USE_OPENSSL_FALLBACK
    return (d_PublicKey != nullptr);
#else
    return (d_PublicKey != gnutls_pubkey_t{});
#endif
}


std::string Gnss_Crypto::convert_to_utf8_str(const std::vector<uint8_t>& input) const
{
    const char hex[] = "0123456789ABCDEF";
    std::string str(input.size() * 2, '0');
    for (size_t i = 0; i < input.size(); i++)
        {
            str[(i * 2) + 0] = hex[((input[i] & 0xF0) >> 4)];
            str[(i * 2) + 1] = hex[((input[i] & 0x0F))];
        }
    return str;
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


void Gnss_Crypto::read_merkle_xml(const std::string& merkleFilePath)
{
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(merkleFilePath.c_str());
    if (!result)
        {
            // XML file not found
            // If it was not the default, maybe it is a configuration error, warn user
            if (merkleFilePath != MERKLEFILE_DEFAULT)
                {
                    std::cerr << "File " << merkleFilePath << " not found" << std::endl;
                }
            // fill default values
            d_x_4_0 = convert_from_hex_str("C5B2A3BD24E819EF82B17ACE83C0E7F41D34AC9B488CB7CE4D765FDE7DCA0297");
            d_x_3_1 = convert_from_hex_str("C8314BA8084E0CA101E595E88F170012F1F5CE71EEEFAB27334283E15935E8E6");
            d_x_2_1 = convert_from_hex_str("6FB21E4DDF3F8E517A5C5B1C6D843F9236707FF11D96F9BA954BFEAA3A44E56B");
            d_x_1_1 = convert_from_hex_str("86E53A50D345FBDAD49835F3363EE4A7262DB738CBDFC399229AE2803679300D");
            d_x_0_0 = convert_from_hex_str("40CAA1D70F7B1D370219674A25721311170A49DE4E4A0CE4FE328674E01CF750");
            d_x_0_1 = convert_from_hex_str("AA1A8B68E5DB293106B5BC8806F9790E8ACF8DC2D28A6EF6C1AC7233A9813D3F");
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

            std::cout << "  Source: " << source.child_value("mission") << " - " << source.child_value("segment") << " - " << source.child_value("element") << std::endl;
            std::cout << "  Destination: " << destination.child_value("mission") << " - " << destination.child_value("segment") << " - " << destination.child_value("element") << std::endl;
            std::cout << "  Issue Date: " << issueDate << std::endl;
            std::cout << "  Signal Version: " << signalVersion << std::endl;
            std::cout << "  Data Version: " << dataVersion << std::endl;

            // Accessing data from the body
            pugi::xml_node merkleTree = body.child("MerkleTree");

            int n = std::stoi(merkleTree.child_value("N"));
            std::string hashFunction = merkleTree.child_value("HashFunction");

            std::cout << "  N: " << n << std::endl;
            std::cout << "  Hash Function: " << hashFunction << std::endl;

            for (pugi::xml_node publicKey : merkleTree.children("PublicKey"))
                {
                    int i = std::stoi(publicKey.child_value("i"));
                    std::string pkid = publicKey.child_value("PKID");
                    int lengthInBits = std::stoi(publicKey.child_value("lengthInBits"));
                    std::string point = publicKey.child_value("point");
                    std::string pkType = publicKey.child_value("PKType");

                    std::cout << "  Public Key: " << i << std::endl;
                    std::cout << "  PKID: " << pkid << std::endl;
                    std::cout << "  Length in Bits: " << lengthInBits << std::endl;
                    std::cout << "  Point: " << point << std::endl;
                    std::cout << "  PK Type: " << pkType << std::endl;
                }
            for (pugi::xml_node treeNode : merkleTree.children("TreeNode"))
                {
                    int j = std::stoi(treeNode.child_value("j"));
                    int i = std::stoi(treeNode.child_value("i"));
                    int lengthInBits = std::stoi(treeNode.child_value("lengthInBits"));
                    std::cout << "  Node length (bits): " << lengthInBits << std::endl;
                    std::string x_ji = treeNode.child_value("x_ji");
                    std::cout << "  Size string (bytes): " << x_ji.size() << std::endl;
                    std::cout << "  m_" << j << "_" << i << " = " << x_ji << std::endl;
                    if (j == 4 && i == 0)
                        {
                            d_x_4_0 = convert_from_hex_str(x_ji);
                        }
                    if (j == 3 && i == 1)
                        {
                            d_x_3_1 = convert_from_hex_str(x_ji);
                        }
                    if (j == 2 && i == 1)
                        {
                            d_x_2_1 = convert_from_hex_str(x_ji);
                        }
                    if (j == 1 && i == 1)
                        {
                            d_x_1_1 = convert_from_hex_str(x_ji);
                        }
                    if (j == 0 && i == 0)
                        {
                            d_x_0_0 = convert_from_hex_str(x_ji);
                        }
                    if (j == 0 && i == 1)
                        {
                            d_x_0_0 = convert_from_hex_str(x_ji);
                        }
                }
        }
    catch (const std::exception& e)
        {
            std::cerr << "Exception raised reading the " << merkleFilePath << " file: " << e.what() << '\n';
            d_x_4_0 = convert_from_hex_str("C5B2A3BD24E819EF82B17ACE83C0E7F41D34AC9B488CB7CE4D765FDE7DCA0297");
            d_x_3_1 = convert_from_hex_str("C8314BA8084E0CA101E595E88F170012F1F5CE71EEEFAB27334283E15935E8E6");
            d_x_2_1 = convert_from_hex_str("6FB21E4DDF3F8E517A5C5B1C6D843F9236707FF11D96F9BA954BFEAA3A44E56B");
            d_x_1_1 = convert_from_hex_str("86E53A50D345FBDAD49835F3363EE4A7262DB738CBDFC399229AE2803679300D");
            d_x_0_0 = convert_from_hex_str("40CAA1D70F7B1D370219674A25721311170A49DE4E4A0CE4FE328674E01CF750");
            d_x_0_1 = convert_from_hex_str("AA1A8B68E5DB293106B5BC8806F9790E8ACF8DC2D28A6EF6C1AC7233A9813D3F");
            return;
        }
    std::cout << "Merkle Tree successfully read from file " << merkleFilePath << std::endl;
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


void Gnss_Crypto::readPublicKeyFromPEM(const std::string& pemFilePath)
{
    // Open the .pem file
    std::ifstream pemFile(pemFilePath);
    if (!pemFile)
        {
            // PEM file not found
            // If it was not the default, maybe it is a configuration error
            if (pemFilePath != PEMFILE_DEFAULT)
                {
                    std::cerr << "File " << pemFilePath << " not found" << std::endl;
                }
            return;
        }
    std::string pemContent((std::istreambuf_iterator<char>(pemFile)), std::istreambuf_iterator<char>());
#if USE_OPENSSL_FALLBACK
    // Create a BIO object from the string data
    BIO* bio = BIO_new_mem_buf(pemContent.c_str(), pemContent.length());
    if (!bio)
        {
            std::cerr << "OpenSSL: error creating a BIO object with data read from file " << pemFilePath << ". Aborting import" << std::endl;
            return;
        }
#if USE_OPENSSL_3
    d_PublicKey = PEM_read_bio_PUBKEY(bio, nullptr, nullptr, nullptr);
#else
    d_PublicKey = PEM_read_bio_EC_PUBKEY(bio, nullptr, nullptr, nullptr);
#endif
    BIO_free(bio);
    if (d_PublicKey == nullptr)
        {
            std::cerr << "OpenSSL: error reading the Public Key from file " << pemFilePath << ". Aborting import" << std::endl;
            return;
        }
#else
    // Import the PEM data
    gnutls_datum_t pemDatum = {const_cast<unsigned char*>(reinterpret_cast<unsigned char*>(pemContent.data())), static_cast<unsigned int>(pemContent.size())};
    gnutls_pubkey_t pubkey;
    gnutls_pubkey_init(&pubkey);

    int ret = gnutls_pubkey_import(pubkey, &pemDatum, GNUTLS_X509_FMT_PEM);
    if (ret != GNUTLS_E_SUCCESS)
        {
            gnutls_pubkey_deinit(pubkey);
            std::cerr << "GnuTLS: error reading the Public Key from file "
                      << pemFilePath
                      << ". Aborting import" << std::endl;
            return;
        }
    d_PublicKey = pubkey;
    gnutls_pubkey_deinit(pubkey);
#endif
    std::cout << "Public key successfully read from file " << pemFilePath << std::endl;
}


bool Gnss_Crypto::verify_signature(const std::vector<uint8_t>& message, const std::vector<uint8_t>& signature)
{
    if (!have_public_key())
        {
            return false;
        }
    bool success = false;
#if USE_OPENSSL_FALLBACK
#if USE_OPENSSL_3
    EVP_PKEY_CTX* ctx;
    ctx = EVP_PKEY_CTX_new(d_PublicKey, nullptr /* no engine */);
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
    EVP_PKEY_CTX_free(ctx);
    if (verification == 1)
        {
            success = true;
        }
    else
        {
            std::cerr << "OpenSSL: message authentication failed" << std::endl;
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
    else
        {
            std::cerr << "OpenSSL: message authentication failed" << std::endl;
        }

#endif
#else
    unsigned int bit_size;
    if (gnutls_pubkey_get_pk_algorithm(d_PublicKey, &bit_size) != GNUTLS_PK_ECDSA)
        {
            std::cout << "GnuTLS: the Public Key does not contain a ECDSA key. Aborting signature verification" << std::endl;
        }
    gnutls_datum_t signature_{};
    signature_.data = const_cast<uint8_t*>(signature.data());
    signature_.size = signature.size();
    gnutls_datum_t data_{};
    data_.data = const_cast<uint8_t*>(message.data());
    data_.size = message.size();
    int ret = gnutls_pubkey_verify_data2(d_PublicKey, GNUTLS_SIGN_ECDSA_SHA256, 0, &data_, &signature_);
    if (ret >= 0)
        {
            success = true;
        }
    else
        {
            std::cerr << "GnuTLS: message authentication failed" << std::endl;
        }
#endif
    return success;
}


std::vector<uint8_t> Gnss_Crypto::getMerkleRoot(const std::vector<std::vector<uint8_t>>& merkle)
{
    if (merkle.empty())
        {
            return {};
        }
    else if (merkle.size() == 1)
        {
            return this->computeSHA3_256(merkle[0]);
        }

    std::vector<std::vector<uint8_t>> new_merkle = merkle;

    while (new_merkle.size() > 1)
        {
            if (new_merkle.size() % 2 == 1)
                {
                    new_merkle.push_back(merkle.back());
                }

            std::vector<std::vector<uint8_t>> result;

            for (size_t i = 0; i < new_merkle.size(); i += 2)
                {
                    std::vector<uint8_t> var1 = this->computeSHA3_256(new_merkle[i]);
                    std::vector<uint8_t> var2 = this->computeSHA3_256(new_merkle[i + 1]);
                    var1.insert(var1.end(), var2.begin(), var2.end());
                    std::vector<uint8_t> hash = this->computeSHA3_256(var1);
                    result.push_back(hash);
                }
            new_merkle = result;
        }
    return new_merkle[0];
}
