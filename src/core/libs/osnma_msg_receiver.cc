/*!
 * \file osnma_msg_receiver.cc
 * \brief GNU Radio block that processes Galileo OSNMA data received from
 * Galileo E1B telemetry blocks. After successful decoding, sends the content to
 * the PVT block.
 * \author Carles Fernandez-Prades, 2023-2026. cfernandez(at)cttc.es
 * Cesare Ghionoiu Martinez, 2023-2024. c.ghionoiu-martinez@tu-braunschweig.de
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */


#include "osnma_msg_receiver.h"
#include "Galileo_OSNMA.h"
#include "gnss_crypto.h"
#include "gnss_satellite.h"
#include "gnss_sdr_filesystem.h"
#include "gnss_sdr_make_unique.h"   // for std::make_unique in C++11
#include "osnma_dsm_reader.h"       // for OSNMA_DSM_Reader
#include "osnma_helper.h"           // for Osnma_Helper
#include <gnuradio/io_signature.h>  // for gr::io_signature::make
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <exception>
#include <fstream>  // for std::ifstream and std::ofstream
#include <iomanip>  // for std::setfill
#include <ios>      // for std::hex, std::uppercase
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>  // for std::accumulate
#include <sstream>  // std::stringstream
#include <tuple>
#include <typeinfo>  // for typeid
#include <utility>


#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>  // for DLOG
#else
#include <absl/log/log.h>
#endif

#if HAS_GENERIC_LAMBDA
#else
#include <boost/bind/bind.hpp>
#endif

#if PMT_USES_BOOST_ANY
#include <boost/any.hpp>
#include <iomanip>
namespace wht = boost;
#else
#include <any>
namespace wht = std;
#endif

#if USE_GNUTLS_FALLBACK
#include <gnutls/gnutls.h>
#include <gnutls/x509.h>
#else
#include <openssl/asn1.h>
#include <openssl/crypto.h>
#include <openssl/objects.h>
#if USE_OPENSSL_3
#include <openssl/store.h>
#else
#include <openssl/bio.h>
#include <openssl/pem.h>
#endif
#include <openssl/x509.h>
#endif


namespace
{
constexpr int32_t osnma_subframe_duration_s = 30;
constexpr int64_t dsm_kroot_expiration_s = 3600;
constexpr int64_t dsm_pkr_expiration_s = 13 * 3600;
constexpr int64_t mack_deferred_expiration_s = 330;
constexpr size_t max_deferred_mack_blocks = 500;
constexpr int64_t tesla_key_retention_s = mack_deferred_expiration_s + osnma_subframe_duration_s;

std::string kroot_metadata_path()
{
    return KROOTFILE_DEFAULT + ".meta";
}


bool compute_tesla_hash_count(int64_t gst_delta_seconds, bool include_kroot_step, uint32_t& hash_count)
{
    constexpr int64_t tesla_key_period_s = 30;
    constexpr int64_t max_safe_hash_count = static_cast<int64_t>(std::numeric_limits<uint32_t>::max()) - 1;
    if (gst_delta_seconds < 0)
        {
            return false;
        }

    const int64_t candidate_hash_count = gst_delta_seconds / tesla_key_period_s + (include_kroot_step ? 1 : 0);
    if (candidate_hash_count <= 0 || candidate_hash_count > max_safe_hash_count)
        {
            return false;
        }

    hash_count = static_cast<uint32_t>(candidate_hash_count);
    return true;
}


bool adkd_is_allowed(const std::vector<uint8_t>& allowed_adkds, uint8_t adkd)
{
    return std::find(allowed_adkds.cbegin(), allowed_adkds.cend(), adkd) != allowed_adkds.cend();
}


bool kroot_mack_parameters_equal(const DSM_KROOT_message& lhs, const DSM_KROOT_message& rhs)
{
    return lhs.verified == rhs.verified &&
           lhs.kroot == rhs.kroot &&
           lhs.alpha == rhs.alpha &&
           lhs.wn_k == rhs.wn_k &&
           lhs.nb_dk == rhs.nb_dk &&
           lhs.pkid == rhs.pkid &&
           lhs.cidkr == rhs.cidkr &&
           lhs.hf == rhs.hf &&
           lhs.mf == rhs.mf &&
           lhs.ks == rhs.ks &&
           lhs.ts == rhs.ts &&
           lhs.maclt == rhs.maclt &&
           lhs.towh_k == rhs.towh_k;
}


std::string trim_copy(const std::string& input)
{
    const auto first = std::find_if_not(input.cbegin(), input.cend(), [](unsigned char c) { return std::isspace(c) != 0; });
    const auto last = std::find_if_not(input.crbegin(), input.crend(), [](unsigned char c) { return std::isspace(c) != 0; }).base();
    if (first >= last)
        {
            return {};
        }
    return {first, last};
}


std::map<std::string, std::string> read_key_value_file(const std::string& path)
{
    std::ifstream file(path);
    std::map<std::string, std::string> values;
    if (!file)
        {
            return values;
        }

    std::string line;
    while (std::getline(file, line))
        {
            const auto comment_pos = line.find('#');
            if (comment_pos != std::string::npos)
                {
                    line.erase(comment_pos);
                }
            const auto separator = line.find('=');
            if (separator == std::string::npos)
                {
                    continue;
                }
            const auto key = trim_copy(line.substr(0, separator));
            const auto value = trim_copy(line.substr(separator + 1));
            if (!key.empty())
                {
                    values[key] = value;
                }
        }
    return values;
}


bool parse_uint32_value(const std::map<std::string, std::string>& values, const std::string& key, uint32_t& output)
{
    const auto it = values.find(key);
    if (it == values.cend())
        {
            return false;
        }
    try
        {
            size_t processed = 0;
            const auto parsed = std::stoul(it->second, &processed, 0);
            if (processed != it->second.size() || parsed > std::numeric_limits<uint32_t>::max())
                {
                    return false;
                }
            output = static_cast<uint32_t>(parsed);
            return true;
        }
    catch (const std::exception&)
        {
            return false;
        }
}


bool parse_uint16_value(const std::map<std::string, std::string>& values, const std::string& key, uint16_t& output)
{
    uint32_t parsed = 0;
    if (!parse_uint32_value(values, key, parsed) || parsed > std::numeric_limits<uint16_t>::max())
        {
            return false;
        }
    output = static_cast<uint16_t>(parsed);
    return true;
}


bool parse_uint8_value(const std::map<std::string, std::string>& values, const std::string& key, uint8_t& output)
{
    uint32_t parsed = 0;
    if (!parse_uint32_value(values, key, parsed) || parsed > std::numeric_limits<uint8_t>::max())
        {
            return false;
        }
    output = static_cast<uint8_t>(parsed);
    return true;
}


bool write_text_file_atomically(const std::string& path, const std::string& contents)
{
    if (path.empty())
        {
            return false;
        }

    const std::string tmp_path = path + ".tmp";
    {
        std::ofstream file(tmp_path, std::ios::binary | std::ios::trunc);
        if (!file)
            {
                return false;
            }
        file << contents;
        if (!file.good())
            {
                return false;
            }
    }

    const fs::path tmp(tmp_path);
    const fs::path destination(path);
    const fs::path backup(path + ".bak");
    errorlib::error_code ec;
    fs::remove(backup, ec);
    if (ec)
        {
            return false;
        }

    const bool destination_exists = fs::exists(destination);
    if (destination_exists)
        {
            ec.clear();
            fs::rename(destination, backup, ec);
            if (ec)
                {
                    return false;
                }
        }

    fs::rename(tmp, destination, ec);
    if (!ec)
        {
            if (destination_exists)
                {
                    errorlib::error_code remove_error;
                    fs::remove(backup, remove_error);
                }
            return true;
        }

    ec.clear();
    if (destination_exists)
        {
            fs::rename(backup, destination, ec);
        }
    return false;
}


std::vector<uint8_t> make_cached_kroot_raw(uint8_t nma_header, const std::vector<uint8_t>& dsm)
{
    std::vector<uint8_t> raw;
    raw.reserve(dsm.size() + 1);
    raw.push_back(nma_header);
    raw.insert(raw.end(), dsm.cbegin(), dsm.cend());
    return raw;
}


std::string trimmed_maclt_slot(const std::string& slot)
{
    const auto first = slot.find_first_not_of(' ');
    if (first == std::string::npos)
        {
            return {};
        }
    const auto last = slot.find_last_not_of(' ');
    return slot.substr(first, last - first + 1);
}


bool fixed_maclt_slot_matches(const MACK_tag_info& tag_info, uint32_t prna, const std::string& slot)
{
    const std::string maclt_slot = trimmed_maclt_slot(slot);
    if (maclt_slot.size() != 3 ||
        maclt_slot[0] < '0' || maclt_slot[0] > '9' ||
        maclt_slot[1] < '0' || maclt_slot[1] > '9')
        {
            return false;
        }

    const auto expected_adkd = static_cast<uint8_t>((maclt_slot[0] - '0') * 10 + (maclt_slot[1] - '0'));
    if (tag_info.ADKD != expected_adkd)
        {
            return false;
        }

    if (maclt_slot[2] == 'S')
        {
            return tag_info.PRN_d == prna;
        }
    if (maclt_slot[2] == 'E')
        {
            return tag_info.PRN_d >= 1 && tag_info.PRN_d <= 36 && tag_info.PRN_d != prna;
        }
    return false;
}


bool tag_info_uses_supported_prnd_adkd(const MACK_tag_info& tag_info)
{
    const bool supported_prnd = (tag_info.PRN_d >= 1 && tag_info.PRN_d <= 36) || tag_info.PRN_d == 255;
    const bool supported_adkd = tag_info.ADKD == 0 || tag_info.ADKD == 4 || tag_info.ADKD == 12;
    return supported_prnd && supported_adkd;
}


bool parse_osnma_pkid_from_subject_cn(const std::string& cn, uint8_t& pkid)
{
    static const std::string prefix("OSNMA-PublicKey-PKID-");

    pkid = 0;

    if (cn.compare(0, prefix.size(), prefix) != 0)
        {
            return false;
        }

    const std::string value = cn.substr(prefix.size());
    if (value.empty() || value.size() > 2)
        {
            return false;
        }

    unsigned int parsed = 0;
    for (char i : value)
        {
            if (i < '0' || i > '9')
                {
                    return false;
                }
            parsed = parsed * 10U + static_cast<unsigned int>(i - '0');
        }

    if (parsed > 15U)
        {
            return false;
        }

    pkid = static_cast<uint8_t>(parsed);
    return true;
}


bool file_is_readable(const std::string& path)
{
    std::ifstream file(path, std::ios::binary);
    return file.good();
}


std::string bytes_to_upper_hex(const std::vector<uint8_t>& bytes)
{
    std::ostringstream output;
    output << std::hex << std::uppercase << std::setfill('0');
    for (const auto byte : bytes)
        {
            output << std::setw(2) << static_cast<uint32_t>(byte);
        }
    return output.str();
}


bool osnma_msg_has_page_validity(const OSNMA_msg& msg)
{
    return msg.page_validity_available ||
           std::any_of(msg.page_valid.cbegin(), msg.page_valid.cend(), [](uint8_t value) { return value != 0; });
}


bool osnma_msg_page_available(const OSNMA_msg& msg, size_t page)
{
    if (page >= msg.page_valid.size())
        {
            return false;
        }
    return !osnma_msg_has_page_validity(msg) || msg.page_valid[page] != 0;
}


bool osnma_msg_all_pages_available(const OSNMA_msg& msg)
{
    return !osnma_msg_has_page_validity(msg) ||
           std::all_of(msg.page_valid.cbegin(), msg.page_valid.cend(), [](uint8_t value) { return value != 0; });
}
}  // namespace


osnma_msg_receiver_sptr osnma_msg_receiver_make(const std::string& pemFilePath, const std::string& merkleFilePath, bool strict_mode, bool replay_mode)
{
    return osnma_msg_receiver_sptr(new osnma_msg_receiver(pemFilePath, merkleFilePath, strict_mode, replay_mode));
}


bool osnma_msg_receiver::npkt_from_public_key_type(const std::string& key_type, uint8_t& npkt) const
{
    for (const auto& it : OSNMA_TABLE_5)
        {
            if (it.second == key_type)
                {
                    npkt = it.first;
                    return true;
                }
        }
    return false;
}


std::string osnma_msg_receiver::public_key_fingerprint_sha256(uint8_t pkid, uint8_t npkt, const std::vector<uint8_t>& compressed_key) const
{
    if (pkid > 15 || npkt > 15 || compressed_key.empty())
        {
            return {};
        }

    std::vector<uint8_t> input;
    input.reserve(1 + compressed_key.size());
    input.push_back(static_cast<uint8_t>(((npkt & 0x0F) << 4) | (pkid & 0x0F)));
    input.insert(input.end(), compressed_key.cbegin(), compressed_key.cend());
    return bytes_to_upper_hex(d_crypto->compute_SHA_256(input));
}


bool osnma_msg_receiver::resolve_configured_public_key_identity(const std::string& key_path, const std::vector<uint8_t>& compressed_key, const std::string& key_type, Osnma_Public_Key_Material& key_material) const
{
    uint8_t pkid = 0;
    uint8_t npkt = 0;

    if (!pkid_from_certificate_subject_cn(key_path, pkid))
        {
            return false;
        }

    if (!npkt_from_public_key_type(key_type, npkt))
        {
            return false;
        }

    const std::string fingerprint = public_key_fingerprint_sha256(pkid, npkt, compressed_key);
    if (fingerprint.empty())
        {
            return false;
        }

    const std::string source = key_material.source.empty() ? std::string("configured-public-key") : key_material.source;
    key_material = Osnma_Public_Key_Material();
    key_material.valid = true;
    key_material.pkid_valid = true;
    key_material.pkid = pkid;
    key_material.npkt_valid = true;
    key_material.npkt = npkt;
    key_material.key_type = key_type;
    key_material.source = source;
    key_material.pem_path = key_path;
    key_material.compressed_key = compressed_key;
    key_material.fingerprint_sha256 = fingerprint;

    return true;
}


bool osnma_msg_receiver::pkid_from_certificate_subject_cn(const std::string& crt_file_path, uint8_t& pkid) const
{
    pkid = 0;

    if (crt_file_path.empty())
        {
            return false;
        }

#if USE_GNUTLS_FALLBACK
    std::ifstream crt_file(crt_file_path.c_str(), std::ios::binary);
    if (!crt_file.is_open())
        {
            LOG(WARNING) << "Galileo OSNMA: unable to open certificate file "
                         << crt_file_path << " to read PKID";
            return false;
        }

    const std::vector<unsigned char> buffer(
        (std::istreambuf_iterator<char>(crt_file)),
        std::istreambuf_iterator<char>());

    if (buffer.empty())
        {
            LOG(WARNING) << "Galileo OSNMA: empty certificate file "
                         << crt_file_path;
            return false;
        }

    gnutls_datum_t buffer_datum = {
        const_cast<unsigned char*>(&buffer[0]),
        static_cast<unsigned int>(buffer.size())};

    gnutls_x509_crt_t cert;
    int ret = gnutls_x509_crt_init(&cert);
    if (ret < 0)
        {
            LOG(WARNING) << "GnuTLS: failed to initialize X.509 certificate: "
                         << gnutls_strerror(ret);
            return false;
        }

    ret = gnutls_x509_crt_import(
        cert,
        &buffer_datum,
        GNUTLS_X509_FMT_PEM);

    if (ret < 0)
        {
            LOG(WARNING) << "GnuTLS: failed to import certificate "
                         << crt_file_path << ": " << gnutls_strerror(ret);
            gnutls_x509_crt_deinit(cert);
            return false;
        }

    size_t cn_size = 0;
    ret = gnutls_x509_crt_get_dn_by_oid(
        cert,
        GNUTLS_OID_X520_COMMON_NAME,
        0,
        0,
        nullptr,
        &cn_size);

    if (ret != GNUTLS_E_SHORT_MEMORY_BUFFER)
        {
            LOG(WARNING) << "GnuTLS: unable to read certificate Subject CN "
                         << "from " << crt_file_path << ": "
                         << gnutls_strerror(ret);
            gnutls_x509_crt_deinit(cert);
            return false;
        }

    std::vector<char> cn_buffer(cn_size);
    ret = gnutls_x509_crt_get_dn_by_oid(
        cert,
        GNUTLS_OID_X520_COMMON_NAME,
        0,
        0,
        &cn_buffer[0],
        &cn_size);

    gnutls_x509_crt_deinit(cert);

    if (ret < 0)
        {
            LOG(WARNING) << "GnuTLS: unable to read certificate Subject CN "
                         << "from " << crt_file_path << ": "
                         << gnutls_strerror(ret);
            return false;
        }

    std::string cn(&cn_buffer[0], cn_size);
    while (!cn.empty() && cn[cn.size() - 1] == '\0')
        {
            cn.erase(cn.size() - 1);
        }

    if (!parse_osnma_pkid_from_subject_cn(cn, pkid))
        {
            LOG(WARNING) << "Galileo OSNMA: certificate Subject CN does not "
                         << "contain a valid OSNMA PKID: " << cn;
            return false;
        }

    return true;
#else  // OpenSSL
#if USE_OPENSSL_3
    OSSL_STORE_CTX* store_ctx = OSSL_STORE_open(
        crt_file_path.c_str(),
        nullptr,
        nullptr,
        nullptr,
        nullptr);

    if (store_ctx == nullptr)
        {
            LOG(WARNING) << "OpenSSL: unable to open certificate file "
                         << crt_file_path << " to read PKID";
            return false;
        }

    X509* cert = nullptr;

    while (!OSSL_STORE_eof(store_ctx))
        {
            OSSL_STORE_INFO* info = OSSL_STORE_load(store_ctx);
            if (info == nullptr)
                {
                    break;
                }

            if (OSSL_STORE_INFO_get_type(info) == OSSL_STORE_INFO_CERT)
                {
                    cert = OSSL_STORE_INFO_get1_CERT(info);
                    OSSL_STORE_INFO_free(info);
                    break;
                }

            OSSL_STORE_INFO_free(info);
        }

    OSSL_STORE_close(store_ctx);

    if (cert == nullptr)
        {
            LOG(WARNING) << "OpenSSL: unable to read X.509 certificate from "
                         << crt_file_path;
            return false;
        }
#else   // OpenSSL < 3
    BIO* bio = BIO_new_file(crt_file_path.c_str(), "rb");
    if (bio == nullptr)
        {
            LOG(WARNING) << "OpenSSL: unable to open certificate file "
                         << crt_file_path << " to read PKID";
            return false;
        }

    X509* cert = PEM_read_bio_X509(bio, nullptr, nullptr, nullptr);
    BIO_free(bio);

    if (cert == nullptr)
        {
            LOG(WARNING) << "OpenSSL: unable to read X.509 certificate from "
                         << crt_file_path;
            return false;
        }
#endif  // OpenSSL

    auto* subject = X509_get_subject_name(cert);
    if (subject == nullptr)
        {
            LOG(WARNING) << "OpenSSL: certificate has no Subject: "
                         << crt_file_path;
            X509_free(cert);
            return false;
        }

    bool found = false;
    uint8_t parsed_pkid = 0;
    int index = -1;

    while ((index = X509_NAME_get_index_by_NID(
                subject,
                NID_commonName,
                index)) >= 0)
        {
            auto* entry = X509_NAME_get_entry(subject, index);
            if (entry == nullptr)
                {
                    continue;
                }

            auto* asn1 = X509_NAME_ENTRY_get_data(entry);
            if (asn1 == nullptr)
                {
                    continue;
                }

            unsigned char* utf8 = nullptr;
            const int len = ASN1_STRING_to_UTF8(&utf8, asn1);
            if (len <= 0 || utf8 == nullptr)
                {
                    if (utf8 != nullptr)
                        {
                            OPENSSL_free(utf8);
                        }
                    continue;
                }

            const std::string cn(reinterpret_cast<char*>(utf8), len);
            OPENSSL_free(utf8);

            uint8_t candidate_pkid = 0;
            if (!parse_osnma_pkid_from_subject_cn(cn, candidate_pkid))
                {
                    continue;
                }

            if (found)
                {
                    LOG(WARNING) << "OpenSSL: ambiguous OSNMA PKID in "
                                 << "certificate Subject CN fields: "
                                 << crt_file_path;
                    X509_free(cert);
                    return false;
                }

            parsed_pkid = candidate_pkid;
            found = true;
        }

    X509_free(cert);

    if (!found)
        {
            LOG(WARNING) << "OpenSSL: certificate Subject CN does not contain "
                         << "a valid OSNMA PKID: " << crt_file_path;
            return false;
        }

    pkid = parsed_pkid;
    return true;
#endif  // USE_GNUTLS_FALLBACK
}


osnma_msg_receiver::osnma_msg_receiver(const std::string& crtFilePath,
    const std::string& merkleFilePath,
    bool strict_mode,
    bool replay_mode) : gr::block("osnma_msg_receiver",
                            gr::io_signature::make(0, 0, 0),
                            gr::io_signature::make(0, 0, 0)),
                        d_merkle_file_path(merkleFilePath),
                        d_strict_mode(strict_mode && !replay_mode),
                        d_replay_mode(replay_mode)
{
    d_dsm_reader = std::make_unique<OSNMA_DSM_Reader>();
    d_crypto = std::make_unique<Gnss_Crypto>(crtFilePath, merkleFilePath);
    d_material_manager = std::make_unique<Osnma_Crypto_Material_Manager>(".");
    d_helper = std::make_unique<Osnma_Helper>();
    d_nav_data_manager = std::make_unique<OSNMA_NavDataManager>();

    Osnma_Merkle_Tree_Material active_merkle_tree;
    active_merkle_tree.valid = !d_crypto->get_merkle_root().empty();
    active_merkle_tree.source = "configured-xml";
    active_merkle_tree.xml_path = merkleFilePath;
    active_merkle_tree.root = d_crypto->get_merkle_root();
    active_merkle_tree.hash_function = d_crypto->get_merkle_tree_hash_function();
    d_material_manager->set_active_merkle_tree(active_merkle_tree);

    if (d_crypto->have_public_key())
        {
            LOG(INFO) << "Galileo OSNMA Public Key available, trying to find DSM-KROOT saved";
            std::cout << "Galileo OSNMA Public Key available, trying to find DSM-KROOT saved" << std::endl;
            d_public_key_verified = true;
            const std::vector<uint8_t> compressed_key = d_crypto->get_public_key_compressed();
            const std::string key_type = d_crypto->get_public_key_type();
            Osnma_Public_Key_Material active_public_key;
            active_public_key.valid = true;
            active_public_key.key_type = key_type;
            active_public_key.compressed_key = compressed_key;
            std::string key_path;
            std::string key_source;
            if (file_is_readable(crtFilePath))
                {
                    key_path = crtFilePath;
                    key_source = "configured-public-key";
                }
            else if (file_is_readable(PEMFILE_DEFAULT))
                {
                    key_path = PEMFILE_DEFAULT;
                    key_source = "stored-public-key";
                }
            active_public_key.pem_path = key_path;
            active_public_key.source = key_source;

            bool public_key_identity_resolved = false;
            if (!key_path.empty() && key_path == PEMFILE_DEFAULT && key_source == "stored-public-key")
                {
                    const auto metadata = d_material_manager->load_active_public_key_cache();
                    uint8_t loaded_npkt = 0;
                    if (metadata.valid && metadata.pkid_valid && metadata.npkt_valid &&
                        npkt_from_public_key_type(key_type, loaded_npkt) &&
                        metadata.npkt == loaded_npkt)
                        {
                            const std::string fingerprint = public_key_fingerprint_sha256(metadata.pkid, metadata.npkt, compressed_key);
                            if (!fingerprint.empty())
                                {
                                    active_public_key = metadata;
                                    active_public_key.valid = true;
                                    active_public_key.pem_path = key_path;
                                    active_public_key.source = key_source;
                                    active_public_key.key_type = key_type;
                                    active_public_key.compressed_key = compressed_key;
                                    active_public_key.fingerprint_sha256 = fingerprint;
                                    public_key_identity_resolved = true;
                                    LOG(INFO) << "Galileo OSNMA: restored Public Key ID " << static_cast<uint32_t>(metadata.pkid)
                                              << " from " << PEMFILE_DEFAULT << ".meta";
                                }
                        }
                }
            else if (!key_path.empty())
                {
                    public_key_identity_resolved = resolve_configured_public_key_identity(key_path, compressed_key, key_type, active_public_key);
                }

            if (public_key_identity_resolved)
                {
                    d_active_public_key_id = active_public_key.pkid;
                    d_active_public_key_id_valid = true;
                }
            else
                {
                    if (!key_path.empty())
                        {
                            LOG(WARNING) << "Galileo OSNMA: configured Public Key loaded, but no trusted OSNMA PKID/NPKT binding is available. Hot start with cached DSM-KROOT will not be attempted with this key.";
                        }
                    active_public_key.valid = true;
                    active_public_key.pkid_valid = false;
                    active_public_key.npkt_valid = false;
                    active_public_key.key_type = key_type;
                    active_public_key.source = std::move(key_source);
                    active_public_key.pem_path = std::move(key_path);
                    active_public_key.compressed_key = compressed_key;
                    active_public_key.fingerprint_sha256.clear();
                }
            d_material_manager->set_active_public_key(active_public_key);

            if (load_pending_dsm_kroot_cache())
                {
                    LOG(INFO) << "Galileo OSNMA DSM-KROOT cache available. Hot-start eligibility will be checked after trusted GST is available.";
                    std::cout << "Galileo OSNMA DSM-KROOT cache available. Hot-start eligibility will be checked after trusted GST is available." << std::endl;
                }
            else
                {
                    LOG(INFO) << "Galileo OSNMA DSM-KROOT not available :: WARM START";
                    std::cout << "Galileo OSNMA DSM-KROOT not available :: WARM START" << std::endl;
                }
        }
    else
        {
            LOG(INFO) << "Galileo OSNMA Public Key not available :: COLD START";
            std::cout << "Galileo OSNMA Public Key not available :: COLD START" << std::endl;
        }

    //  register OSNMA input message port from telemetry blocks
    this->message_port_register_in(pmt::mp("OSNMA_from_TLM"));
    // register OSNMA output message port to PVT block
    this->message_port_register_out(pmt::mp("OSNMA_to_PVT"));

    this->set_msg_handler(pmt::mp("OSNMA_from_TLM"),
#if HAS_GENERIC_LAMBDA
        [this](auto&& PH1) { msg_handler_osnma(PH1); });
#else
#if USE_BOOST_BIND_PLACEHOLDERS
        boost::bind(&osnma_msg_receiver::msg_handler_osnma, this, boost::placeholders::_1));
#else
        boost::bind(&osnma_msg_receiver::msg_handler_osnma, this, _1));
#endif
#endif

    d_GST_Rx = d_helper->compute_gst_now();
    const auto WN = d_helper->get_WN(d_GST_Rx);
    const auto TOW = d_helper->get_TOW(d_GST_Rx);
    LOG(INFO) << "Galileo OSNMA: initial receiver time GST=[" << WN << " " << TOW << "]";
    std::cout << "Galileo OSNMA: initial receiver time GST=[" << WN << " " << TOW << "]" << std::endl;
    if (d_replay_mode)
        {
            LOG(WARNING) << "Galileo OSNMA: GNSS-SDR.osnma_mode=replay enabled. Receiver wall-clock GST alignment check is disabled; use only for captured-signal replay or post-processing.";
            std::cout << "Galileo OSNMA: GNSS-SDR.osnma_mode=replay enabled. Receiver wall-clock GST alignment check is disabled; use only for captured-signal replay or post-processing." << std::endl;
        }
    if (!d_strict_mode)
        {
            LOG(INFO) << "Galileo OSNMA: in non-strict mode, authenticated data are not enforced by PVT strict filtering.";
            std::cout << "Galileo OSNMA: in non-strict mode, authenticated data are not enforced by PVT strict filtering." << std::endl;
        }
}


void osnma_msg_receiver::msg_handler_osnma(const pmt::pmt_t& msg)
{
    // requires mutex with msg_handler_osnma function called by the scheduler
    gr::thread::scoped_lock lock(d_setlock);
    try
        {
            const size_t msg_type_hash_code = pmt::any_ref(msg).type().hash_code();
            if (msg_type_hash_code == typeid(std::shared_ptr<OSNMA_msg>).hash_code())
                {
                    const auto nma_msg = wht::any_cast<std::shared_ptr<OSNMA_msg>>(pmt::any_ref(msg));
                    const auto sat = Gnss_Satellite(std::string("Galileo"), nma_msg->PRN);

                    std::ostringstream output_message;
                    output_message << "Galileo OSNMA: data received started at "
                                   << "WN="
                                   << nma_msg->WN_sf0
                                   << ", TOW="
                                   << nma_msg->TOW_sf0
                                   << ", from satellite "
                                   << sat;
                    LOG(INFO) << output_message.str();
                    std::cout << output_message.str() << std::endl;

                    // Receiver time update
                    d_GST_SIS = d_helper->compute_gst(nma_msg->WN_sf0, nma_msg->TOW_sf0);
                    if (d_last_verified_key_GST == 0)
                        {
                            d_last_received_GST = d_GST_SIS;
                        }
                    else if (d_GST_SIS > d_last_received_GST)
                        {
                            d_last_received_GST = d_GST_SIS;
                        }
                    d_nav_data_manager->prune_old_navigation_data(nma_msg->WN_sf0, nma_msg->TOW_sf0);
                    prune_old_tesla_keys(d_GST_SIS);
                    if (!d_receiver_time_override)
                        {
                            d_GST_Rx = d_helper->compute_gst_now();
                        }
                    LOG(INFO) << "Galileo OSNMA: Local current Receiver Time GST=[" << d_helper->get_WN(d_GST_Rx) << " " << d_helper->get_TOW(d_GST_Rx) << "]";
                    std::cout << "Galileo OSNMA: Local current Receiver Time GST=[" << d_helper->get_WN(d_GST_Rx) << " " << d_helper->get_TOW(d_GST_Rx) << "]" << std::endl;

                    // time constraint verification
                    d_time_constraint_verified = false;
                    if (d_replay_mode)
                        {
                            d_tags_to_verify = {0, 4, 12};
                            DLOG(INFO) << "Galileo OSNMA: replay mode skips receiver wall-clock GST alignment check for this subframe.";
                        }
                    else
                        {
                            const uint32_t gst_sis_rx = gst_with_offset(nma_msg->WN_sf0, nma_msg->TOW_sf0, osnma_subframe_duration_s);
                            const int64_t time_delta_s = gst_delta_seconds(d_GST_Rx, gst_sis_rx);
                            const int64_t abs_time_delta_s = time_delta_s < 0 ? -time_delta_s : time_delta_s;
                            const int64_t fast_mac_limit_s = static_cast<int64_t>(d_T_L) / 2;
                            const int64_t slow_mac_limit_s = (static_cast<int64_t>(d_T_L) + 300) / 2;
                            if (abs_time_delta_s < fast_mac_limit_s)
                                {
                                    d_time_constraint_verified = true;
                                    d_tags_to_verify = {0, 4, 12};
                                    LOG(INFO) << "Galileo OSNMA: time constraint OK: |GST_Rx - GST_SIS_rx| = "
                                              << abs_time_delta_s << " s, expected < " << fast_mac_limit_s << " s";
                                }
                            else if (abs_time_delta_s < slow_mac_limit_s)
                                {
                                    d_time_constraint_verified = true;
                                    d_tags_to_verify = {12};
                                    std::ostringstream time_warning;
                                    time_warning << "Galileo OSNMA: time constraint violation for fast MACs: |GST_Rx - GST_SIS_rx| = "
                                                 << abs_time_delta_s << " s, expected < " << fast_mac_limit_s
                                                 << " s; processing only ADKD=12 slow MACs, expected < " << slow_mac_limit_s << " s";
                                    LOG(WARNING) << time_warning.str();
                                    std::cout << time_warning.str() << std::endl;
                                }
                            else
                                {
                                    d_tags_to_verify = {};
                                    std::ostringstream time_violation;
                                    time_violation << "Galileo OSNMA: time constraint violation: |GST_Rx - GST_SIS_rx| = "
                                                   << abs_time_delta_s << " s, expected < " << slow_mac_limit_s << " s";
                                    LOG(WARNING) << time_violation.str();
                                    std::cerr << time_violation.str() << std::endl;
                                    return;
                                }
                        }

                    process_osnma_message(nma_msg);
                }  // OSNMA frame received
            else if (msg_type_hash_code == typeid(std::shared_ptr<std::tuple<uint32_t, std::string, uint32_t, uint32_t>>).hash_code())  // Navigation data bits for OSNMA received
                {
                    const auto inav_data = wht::any_cast<std::shared_ptr<std::tuple<uint32_t, std::string, uint32_t, uint32_t>>>(pmt::any_ref(msg));
                    uint32_t PRNd = std::get<0>(*inav_data);
                    std::string nav_data = std::get<1>(*inav_data);
                    uint32_t WN = std::get<2>(*inav_data);
                    uint32_t TOW = std::get<3>(*inav_data);
                    d_nav_data_manager->add_navigation_data(nav_data, PRNd, WN, TOW);
                    try_verify_pending_tags(false);
                }
            else if (msg_type_hash_code == typeid(std::shared_ptr<std::tuple<uint32_t, std::string, uint32_t>>).hash_code())  // Legacy navigation data bits for OSNMA received
                {
                    const auto inav_data = wht::any_cast<std::shared_ptr<std::tuple<uint32_t, std::string, uint32_t>>>(pmt::any_ref(msg));
                    uint32_t PRNd = std::get<0>(*inav_data);
                    std::string nav_data = std::get<1>(*inav_data);
                    uint32_t TOW = std::get<2>(*inav_data);
                    uint32_t WN = d_helper->get_WN(d_GST_SIS);
                    d_nav_data_manager->add_navigation_data(nav_data, PRNd, WN, TOW);
                    try_verify_pending_tags(false);
                }
            else
                {
                    LOG(WARNING) << "Galileo OSNMA: osnma_msg_receiver received an unknown object type!";
                }
        }
    catch (const wht::bad_any_cast& e)
        {
            LOG(WARNING) << "Galileo OSNMA: osnma_msg_receiver Bad any_cast: " << e.what();
        }
}


void osnma_msg_receiver::read_merkle_xml(const std::string& merklepath)
{
    d_merkle_file_path = merklepath;
    auto merkle_tree = d_material_manager->load_configured_merkle_tree(merklepath);
    if (!merkle_tree.valid)
        {
            LOG(WARNING) << "Galileo OSNMA: Unable to load Merkle Tree material from " << merklepath;
            return;
        }

    if (d_flag_merkle_tree_renewal)
        {
            d_material_manager->set_candidate_merkle_tree(merkle_tree);
            d_new_merkle_tree_loaded = merkle_tree_differs_from_renewal_start(merkle_tree);
            return;
        }

    d_crypto->set_merkle_root(merkle_tree.root);
    d_crypto->set_merkle_tree_hash_function(merkle_tree.hash_function);
    d_material_manager->set_active_merkle_tree(merkle_tree);
}


int64_t osnma_msg_receiver::gst_delta_seconds(uint32_t lhs_gst, uint32_t rhs_gst) const
{
    return gst_to_seconds(lhs_gst) - gst_to_seconds(rhs_gst);
}


int64_t osnma_msg_receiver::gst_to_seconds(uint32_t gst) const
{
    constexpr int64_t seconds_per_week = 604800;
    return static_cast<int64_t>(d_helper->get_WN(gst)) * seconds_per_week + d_helper->get_TOW(gst);
}


uint32_t osnma_msg_receiver::gst_with_offset(uint32_t WN, uint32_t TOW, int32_t offset_seconds) const
{
    constexpr int64_t seconds_per_week = 604800;
    int64_t absolute_seconds = static_cast<int64_t>(WN) * seconds_per_week + static_cast<int64_t>(TOW) + offset_seconds;
    if (absolute_seconds < 0)
        {
            absolute_seconds = 0;
        }
    const auto week = static_cast<uint32_t>(absolute_seconds / seconds_per_week);
    const auto tow = static_cast<uint32_t>(absolute_seconds % seconds_per_week);
    return d_helper->compute_gst(week, tow);
}


bool osnma_msg_receiver::get_tesla_key(uint32_t WN, uint32_t TOW, int32_t offset_seconds, std::vector<uint8_t>& key) const
{
    const auto it = d_tesla_keys.find(gst_with_offset(WN, TOW, offset_seconds));
    if (it == d_tesla_keys.cend())
        {
            return false;
        }
    key = it->second.key;
    return true;
}


bool osnma_msg_receiver::get_tesla_key_for_adkd(uint32_t WN, uint32_t TOW, int32_t offset_seconds, uint8_t adkd, std::vector<uint8_t>& key) const
{
    const auto it = d_tesla_keys.find(gst_with_offset(WN, TOW, offset_seconds));
    if (it == d_tesla_keys.cend() || !adkd_is_allowed(it->second.allowed_adkds, adkd))
        {
            return false;
        }
    key = it->second.key;
    return true;
}


bool osnma_msg_receiver::has_tesla_key(uint32_t WN, uint32_t TOW, int32_t offset_seconds) const
{
    return d_tesla_keys.find(gst_with_offset(WN, TOW, offset_seconds)) != d_tesla_keys.cend();
}


bool osnma_msg_receiver::has_tesla_key_for_adkd(uint32_t WN, uint32_t TOW, int32_t offset_seconds, uint8_t adkd) const
{
    const auto it = d_tesla_keys.find(gst_with_offset(WN, TOW, offset_seconds));
    return it != d_tesla_keys.cend() && adkd_is_allowed(it->second.allowed_adkds, adkd);
}


bool osnma_msg_receiver::set_gst_sf_for_mack(uint32_t WN, uint32_t TOW)
{
    if (!d_kroot_verified || !d_osnma_data.d_dsm_kroot_message.verified)
        {
            return false;
        }

    d_GST_0 = d_helper->compute_gst(d_osnma_data.d_dsm_kroot_message.wn_k, d_osnma_data.d_dsm_kroot_message.towh_k * 3600);
    constexpr int64_t seconds_per_week = 604800;
    const int64_t gst0_seconds = gst_to_seconds(d_GST_0);
    const int64_t mack_seconds = gst_to_seconds(d_helper->compute_gst(WN, TOW));
    if (mack_seconds < gst0_seconds)
        {
            LOG(INFO) << "Galileo OSNMA: KROOT not applicable yet. Skipping MACK block.";
            return false;
        }

    const int64_t gst_sf_seconds = gst0_seconds + d_T_L * ((mack_seconds - gst0_seconds) / d_T_L);  // Eq. 3 R.G.
    const auto gst_sf_wn = static_cast<uint32_t>(gst_sf_seconds / seconds_per_week);
    const auto gst_sf_tow = static_cast<uint32_t>(gst_sf_seconds % seconds_per_week);
    d_GST_Sf = d_helper->compute_gst(gst_sf_wn, gst_sf_tow);
    return true;
}


bool osnma_msg_receiver::mack_bits_available(size_t bit_offset, size_t bit_length) const
{
    if (bit_length == 0)
        {
            return true;
        }
    const size_t last_bit = bit_offset + bit_length - 1;
    if (bit_offset >= 480 || last_bit >= 480 || last_bit < bit_offset)
        {
            return false;
        }
    if (!d_mack_page_validity_available)
        {
            return true;
        }
    const size_t first_page = bit_offset / 32;
    const size_t last_page = last_bit / 32;
    for (size_t page = first_page; page <= last_page; ++page)
        {
            if (page >= d_mack_page_received.size() || d_mack_page_received[page] == 0)
                {
                    return false;
                }
        }
    return true;
}


uint64_t osnma_msg_receiver::read_mack_bits(size_t bit_offset, size_t bit_length) const
{
    uint64_t value = 0;
    for (size_t bit = 0; bit < bit_length; ++bit)
        {
            const size_t absolute_bit = bit_offset + bit;
            const size_t byte_index = absolute_bit / 8;
            const size_t bit_index = 7 - (absolute_bit % 8);
            value = (value << 1) | ((d_mack_message[byte_index] >> bit_index) & 0x01);
        }
    return value;
}


bool osnma_msg_receiver::read_mack_byte(size_t bit_offset, uint8_t& value) const
{
    if (!mack_bits_available(bit_offset, 8))
        {
            return false;
        }
    value = static_cast<uint8_t>(read_mack_bits(bit_offset, 8));
    return true;
}


bool osnma_msg_receiver::merge_partial_tesla_key(uint32_t key_gst, const std::vector<uint8_t>& key_bytes, const std::vector<uint8_t>& key_byte_received, std::vector<uint8_t>& merged_key)
{
    if (key_bytes.empty() || key_bytes.size() != key_byte_received.size())
        {
            return false;
        }

    auto& partial_key = d_partial_tesla_keys[key_gst];
    if (partial_key.bytes.size() != key_bytes.size())
        {
            partial_key.bytes.assign(key_bytes.size(), 0);
            partial_key.received.assign(key_bytes.size(), 0);
        }

    for (size_t i = 0; i < key_bytes.size(); ++i)
        {
            if (key_byte_received[i] != 0)
                {
                    partial_key.bytes[i] = key_bytes[i];
                    partial_key.received[i] = 1;
                }
        }

    for (auto it = d_partial_tesla_keys.begin(); it != d_partial_tesla_keys.end();)
        {
            if (it->first != key_gst && gst_delta_seconds(key_gst, it->first) > 600)
                {
                    it = d_partial_tesla_keys.erase(it);
                }
            else
                {
                    ++it;
                }
        }

    const bool complete = std::all_of(partial_key.received.cbegin(), partial_key.received.cend(), [](uint8_t value) { return value != 0; });
    if (!complete)
        {
            return false;
        }

    merged_key = partial_key.bytes;
    d_partial_tesla_keys.erase(key_gst);
    return true;
}


void osnma_msg_receiver::process_osnma_message(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    if (d_flag_alert_message_verified)
        {
            return;
        }
    if (!osnma_msg_page_available(*osnma_msg, 0))
        {
            LOG(WARNING) << "Galileo OSNMA: Missing OSNMA page 0. Skipping subframe because NMA header is unavailable.";
            return;
        }
    read_nma_header(osnma_msg->hkroot[0]);

    if (d_osnma_data.d_nma_header.nmas == 0 /* RES */)
        {
            LOG(WARNING) << "Galileo OSNMA: NMAS invalid (RES), skipping osnma message";
            return;
        }
    if (d_time_constraint_verified)
        {
            evaluate_pending_dsm_kroot_cache(d_GST_SIS);
        }
    if (osnma_msg_all_pages_available(*osnma_msg))
        {
            read_dsm_header(osnma_msg->hkroot[1]);
            read_dsm_block(osnma_msg);
            process_dsm_block(osnma_msg);  // will process dsm block if received a complete one, then will call mack processing upon re-setting the dsm block to 0
        }
    else
        {
            LOG(INFO) << "Galileo OSNMA: Incomplete HKROOT block. Skipping DSM accumulation for this subframe.";
        }
    if (d_flag_alert_message_verified)
        {
            return;
        }
    const uint32_t current_gst = d_helper->compute_gst(osnma_msg->WN_sf0, osnma_msg->TOW_sf0);
    promote_verified_future_kroot_if_due(current_gst);
    expire_verified_kroot_if_needed(current_gst);
    if (d_kroot_verified && d_osnma_data.d_dsm_kroot_message.verified)
        {
            if (!set_gst_sf_for_mack(osnma_msg->WN_sf0, osnma_msg->TOW_sf0))
                {
                    return;
                }
            process_deferred_mack_blocks();
        }
    read_and_process_mack_block(osnma_msg);  // only process them if at least 3 available.
}


/**
 * @brief Reads the NMA header from the given input and stores the values in the d_osnma_data structure.
 *
 * The NMA header consists of several fields: d_nma_header.nmas, d_nma_header.cid, d_nma_header.cpks, and d_nma_header.reserved.
 * Each field is retrieved using the corresponding getter functions from the d_dsm_reader auxiliary object.
 *
 * @param nma_header The input containing the NMA header.
 */
void osnma_msg_receiver::read_nma_header(uint8_t nma_header)
{
    d_osnma_data.d_nma_header.nmas = d_dsm_reader->get_nmas(nma_header);
    d_osnma_data.d_nma_header.cid = d_dsm_reader->get_cid(nma_header);
    d_osnma_data.d_nma_header.cpks = d_dsm_reader->get_cpks(nma_header);
    d_osnma_data.d_nma_header.reserved = d_dsm_reader->get_nma_header_reserved(nma_header);
}


/**
 * @brief Read the DSM header from the given dsm_header and populate the d_osnma_data structure.
 *
 * @param dsm_header The DSM header.
 */
void osnma_msg_receiver::read_dsm_header(uint8_t dsm_header)
{
    d_osnma_data.d_dsm_header.dsm_id = d_dsm_reader->get_dsm_id(dsm_header);
    d_osnma_data.d_dsm_header.dsm_block_id = d_dsm_reader->get_dsm_block_id(dsm_header);  // BID
    LOG(INFO) << "Galileo OSNMA: Received block DSM_BID=" << static_cast<uint32_t>(d_osnma_data.d_dsm_header.dsm_block_id)
              << " with DSM_ID " << static_cast<uint32_t>(d_osnma_data.d_dsm_header.dsm_id);
}


/*
 * accumulates dsm messages
 * */
void osnma_msg_receiver::read_dsm_block(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    if (osnma_msg->hkroot.size() <= 2)
        {
            LOG(WARNING) << "Galileo OSNMA hkroot too short, skipping";
            return;
        }
    if (!osnma_msg_all_pages_available(*osnma_msg))
        {
            LOG(INFO) << "Galileo OSNMA: DSM block is incomplete at page level. Skipping it.";
            return;
        }
    // Fill d_dsm_message. dsm_block_id provides the offset within the dsm message.
    size_t index = 0;
    const auto dsm_id = d_osnma_data.d_dsm_header.dsm_id;
    const auto dsm_block_id = d_osnma_data.d_dsm_header.dsm_block_id;
    const size_t offset = SIZE_DSM_BLOCKS_BYTES * dsm_block_id;
    auto& dsm_buf = d_dsm_message[dsm_id];
    const uint32_t current_gst = d_helper->compute_gst(osnma_msg->WN_sf0, osnma_msg->TOW_sf0);
    const auto clear_dsm_block = [&](uint8_t block_id) {
        const size_t block_offset = SIZE_DSM_BLOCKS_BYTES * block_id;
        if (block_offset + SIZE_DSM_BLOCKS_BYTES <= dsm_buf.size())
            {
                std::fill(dsm_buf.begin() + block_offset,
                    dsm_buf.begin() + block_offset + SIZE_DSM_BLOCKS_BYTES,
                    0);
            }
        d_dsm_id_received[dsm_id][block_id] = 0;
        d_dsm_block_nma_header[dsm_id][block_id] = 0;
    };

    expire_dsm_accumulator_if_needed(dsm_id, current_gst);

    if (dsm_block_id == 0)
        {
            const bool block_zero_received = d_dsm_id_received[dsm_id][0] != 0;
            const bool repeated_block_zero = block_zero_received &&
                                             (d_dsm_nma_header[dsm_id] == osnma_msg->hkroot[0]) &&
                                             std::equal(osnma_msg->hkroot.cbegin() + 2, osnma_msg->hkroot.cend(), dsm_buf.cbegin());
            if (block_zero_received && !repeated_block_zero)
                {
                    reset_dsm_accumulator(dsm_id);
                }
        }
    else if (d_dsm_id_received[dsm_id][0] != 0 &&
             d_dsm_nma_header[dsm_id] != osnma_msg->hkroot[0])
        {
            LOG(WARNING) << "Galileo OSNMA: DSM_ID " << static_cast<uint32_t>(dsm_id)
                         << " accumulator NMA header changed before completion. Discarding anchored DSM blocks.";
            reset_dsm_accumulator(dsm_id);
        }
    if (dsm_block_id != 0 &&
        d_number_of_blocks[dsm_id] != 0 &&
        dsm_block_id >= d_number_of_blocks[dsm_id])
        {
            LOG(WARNING) << "Galileo OSNMA: DSM_ID " << static_cast<uint32_t>(dsm_id)
                         << " block " << static_cast<uint32_t>(dsm_block_id)
                         << " is outside the announced DSM length of "
                         << static_cast<uint32_t>(d_number_of_blocks[dsm_id])
                         << " blocks. Discarding it.";
            return;
        }
    if (!d_dsm_first_gst_valid[dsm_id])
        {
            d_dsm_first_gst[dsm_id] = current_gst;
            d_dsm_first_gst_valid[dsm_id] = true;
        }

    for (const auto* it = osnma_msg->hkroot.cbegin() + 2; it != osnma_msg->hkroot.cend(); ++it, ++index)
        {
            if (offset + index < dsm_buf.size())
                {
                    dsm_buf[offset + index] = *it;
                }
            else
                {
                    LOG(ERROR) << "Galileo OSNMA: DSM buffer overflow prevented";
                    return;
                }
        }
    // First block indicates number of blocks in DSM message
    if (dsm_block_id == 0)
        {
            d_dsm_nma_header[dsm_id] = osnma_msg->hkroot[0];
            d_dsm_block_nma_header[dsm_id][0] = osnma_msg->hkroot[0];
            uint8_t nb = d_dsm_reader->get_number_blocks_index(dsm_buf[0]);
            uint16_t number_of_blocks = 0;
            if (dsm_id < 12)
                {
                    // DSM-KROOT Table 7
                    const auto it = OSNMA_TABLE_7.find(nb);
                    if (it != OSNMA_TABLE_7.cend())
                        {
                            number_of_blocks = it->second.first;
                        }
                }
            else if (dsm_id < 16)
                {
                    // DSM-PKR Table 3
                    const auto it = OSNMA_TABLE_3.find(nb);
                    if (it != OSNMA_TABLE_3.cend())
                        {
                            number_of_blocks = it->second.first;
                        }
                }
            else
                {
                    LOG(WARNING) << "Galileo OSNMA: Wrong DSM ID";
                    return;
                }
            d_number_of_blocks[dsm_id] = number_of_blocks;
            LOG(INFO) << "Galileo OSNMA: number of blocks in this message: " << static_cast<uint32_t>(number_of_blocks);
            if (number_of_blocks == 0)
                {
                    // Something is wrong, start over
                    LOG(WARNING) << "Galileo OSNMA: Wrong number of blocks, start over";
                    reset_dsm_accumulator(dsm_id);
                    return;
                }
            // Block 0 anchors the DSM identity; keep only earlier blocks that match it.
            for (size_t block_id = 1; block_id < d_dsm_id_received[dsm_id].size(); ++block_id)
                {
                    if (d_dsm_id_received[dsm_id][block_id] != 0 &&
                        (block_id >= number_of_blocks ||
                            d_dsm_block_nma_header[dsm_id][block_id] != d_dsm_nma_header[dsm_id]))
                        {
                            clear_dsm_block(block_id);
                        }
                }
        }
    // Annotate bid
    d_dsm_id_received[dsm_id][dsm_block_id] = 1;
    d_dsm_block_nma_header[dsm_id][dsm_block_id] = osnma_msg->hkroot[0];

    // Build availability string
    std::ostringstream available_blocks;
    available_blocks << "Galileo OSNMA: Available blocks for DSM_ID "
                     << static_cast<uint32_t>(dsm_id) << ": [ ";

    const auto& blocks = d_dsm_id_received[dsm_id];
    uint16_t total_blocks = d_number_of_blocks[dsm_id];
    if (total_blocks == 0)
        {
            total_blocks = blocks.size();
        }

    for (uint16_t k = 0; k < total_blocks; k++)
        {
            available_blocks << (blocks[k] == 0 ? "- " : "X ");
        }

    available_blocks << "]";
    LOG(INFO) << available_blocks.str();
    std::cout << available_blocks.str() << std::endl;
}


/**
 * @brief Process DSM block of an OSNMA message.
 *
 * \details This function checks if all inner blocks of the DSM message are available and if so, calls process_dsm_message().
 * \post It creates a vector to hold the DSM message data, copies the data from the inner blocks into the vector,
 * resets the inner block arrays to empty
 *
 * @param osnma_msg The OSNMA message.
 */
void osnma_msg_receiver::process_dsm_block(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    // if all inner blocks available -> Process DSM message
    const auto dsm_id = d_osnma_data.d_dsm_header.dsm_id;
    if ((d_number_of_blocks[dsm_id] != 0) &&
        (d_number_of_blocks[dsm_id] == std::accumulate(d_dsm_id_received[dsm_id].cbegin(), d_dsm_id_received[dsm_id].cend(), 0)))
        {
            size_t len = std::size_t(d_number_of_blocks[dsm_id]) * SIZE_DSM_BLOCKS_BYTES;
            std::vector<uint8_t> dsm_msg(len, 0);
            for (uint32_t i = 0; i < d_number_of_blocks[dsm_id]; i++)
                {
                    for (size_t j = 0; j < SIZE_DSM_BLOCKS_BYTES; j++)
                        {
                            dsm_msg[i * SIZE_DSM_BLOCKS_BYTES + j] = d_dsm_message[dsm_id][i * SIZE_DSM_BLOCKS_BYTES + j];
                        }
                }
            const uint8_t nma_header = d_dsm_nma_header[dsm_id];
            reset_dsm_accumulator(dsm_id);
            LOG(INFO) << "Galileo OSNMA: DSM message completed :: start processing, GST=[" << osnma_msg->WN_sf0 << " " << osnma_msg->TOW_sf0 << "]";
            process_dsm_message(dsm_msg, nma_header);
        }
}


void osnma_msg_receiver::expire_dsm_accumulator_if_needed(uint8_t dsm_id, uint32_t current_gst)
{
    if (dsm_id >= d_dsm_first_gst.size() || !d_dsm_first_gst_valid[dsm_id])
        {
            return;
        }

    const int64_t timeout_s = dsm_id < 12 ? dsm_kroot_expiration_s : dsm_pkr_expiration_s;
    const int64_t age_s = gst_delta_seconds(current_gst, d_dsm_first_gst[dsm_id]);
    if (age_s >= timeout_s)
        {
            LOG(WARNING) << "Galileo OSNMA: DSM_ID " << static_cast<uint32_t>(dsm_id)
                         << " accumulator expired after " << age_s << " s. Discarding incomplete DSM blocks.";
            reset_dsm_accumulator(dsm_id);
        }
}


void osnma_msg_receiver::reset_dsm_accumulator(uint8_t dsm_id)
{
    if (dsm_id >= d_dsm_message.size())
        {
            return;
        }
    d_dsm_message[dsm_id] = {};
    d_dsm_id_received[dsm_id] = {};
    d_number_of_blocks[dsm_id] = 0;
    d_dsm_nma_header[dsm_id] = 0;
    d_dsm_block_nma_header[dsm_id] = {};
    d_dsm_first_gst[dsm_id] = 0;
    d_dsm_first_gst_valid[dsm_id] = false;
}


void osnma_msg_receiver::reset_dsm_accumulators()
{
    d_dsm_message = {};
    d_dsm_id_received = {};
    d_number_of_blocks = {};
    d_dsm_nma_header = {};
    d_dsm_block_nma_header = {};
    d_dsm_first_gst = {};
    d_dsm_first_gst_valid = {};
}


void osnma_msg_receiver::reset_tesla_chain_state(bool preserve_deferred_mack_blocks)
{
    d_tesla_keys.clear();
    d_partial_tesla_keys.clear();
    d_tags_awaiting_verify.clear();
    d_macks_awaiting_MACSEQ_verification.clear();
    if (!preserve_deferred_mack_blocks)
        {
            d_mack_blocks_awaiting_kroot.clear();
        }
    d_tesla_key_verified = false;
    d_last_verified_key_GST = 0;
}


void osnma_msg_receiver::promote_verified_future_kroot_if_due(uint32_t current_gst)
{
    const auto& future_kroot = d_osnma_data.d_dsm_kroot_new_message;
    const bool transition_pending = d_flag_chain_renewal || d_flag_chain_revocation || d_flag_PK_revocation;
    if (!transition_pending || !future_kroot.verified)
        {
            return;
        }

    const uint32_t future_kroot_gst0 = d_helper->compute_gst(future_kroot.wn_k, future_kroot.towh_k * 3600);
    if (gst_delta_seconds(current_gst, future_kroot_gst0) < 0)
        {
            return;
        }

    LOG(INFO) << "Galileo OSNMA: authenticated transition reached KROOT applicability. Promoting preverified CID="
              << static_cast<uint32_t>(future_kroot.cidkr) << " DSM-KROOT.";
    d_osnma_data.d_dsm_kroot_message = future_kroot;
    d_osnma_data.d_dsm_kroot_new_message = DSM_KROOT_message();
    d_flag_chain_renewal = false;
    d_flag_chain_revocation = false;
    d_flag_PK_revocation = false;
    d_GST_chain_renewal_start = 0;
    d_GST_chain_revocation_start = 0;
    d_GST_PKR_PKREV_start = 0;
    d_kroot_verified = true;
    d_kroot_loaded_from_cache = false;
    d_nav_data_manager->reset_tag_accumulations();
    reset_tesla_chain_state();
}


bool osnma_msg_receiver::verified_kroot_is_fresh(uint32_t current_gst) const
{
    if (!d_kroot_verified || d_last_verified_kroot_GST == 0)
        {
            return false;
        }
    return gst_delta_seconds(current_gst, d_last_verified_kroot_GST) < dsm_kroot_expiration_s;
}


void osnma_msg_receiver::invalidate_verified_kroot()
{
    d_kroot_verified = false;
    d_last_verified_kroot_GST = 0;
    d_osnma_data.d_dsm_kroot_message.verified = false;
    d_kroot_loaded_from_cache = false;
}


void osnma_msg_receiver::expire_verified_kroot_if_needed(uint32_t current_gst)
{
    if (!d_kroot_verified || verified_kroot_is_fresh(current_gst))
        {
            return;
        }

    if (d_last_verified_kroot_GST == 0)
        {
            LOG(WARNING) << "Galileo OSNMA: DSM-KROOT signature verification time is unknown. Reverification required before MACK processing.";
            std::cout << "Galileo OSNMA: DSM-KROOT signature verification time is unknown. Reverification required before MACK processing." << std::endl;
        }
    else
        {
            const int64_t age_s = gst_delta_seconds(current_gst, d_last_verified_kroot_GST);
            LOG(WARNING) << "Galileo OSNMA: DSM-KROOT signature verification is stale after "
                         << age_s << " s. Reverification required before MACK processing.";
            std::cout << "Galileo OSNMA: DSM-KROOT signature verification is stale after "
                      << age_s << " s. Reverification required before MACK processing." << std::endl;
        }

    invalidate_verified_kroot();
    d_nav_data_manager->reset_tag_accumulations();
    reset_tesla_chain_state();
}


void osnma_msg_receiver::handle_authenticated_revocation(bool chain_revocation, bool public_key_revocation, bool preserve_future_kroot, bool preserve_active_public_key)
{
    const DSM_KROOT_message verified_future_kroot = d_osnma_data.d_dsm_kroot_new_message;
    const bool keep_future_kroot = preserve_future_kroot && verified_future_kroot.verified;
    const uint32_t verified_future_kroot_gst = keep_future_kroot ? d_last_verified_kroot_GST : 0;
    const Osnma_Public_Key_Material active_public_key = d_material_manager->active_public_key();
    const bool keep_active_public_key = public_key_revocation && preserve_active_public_key && d_public_key_verified && d_active_public_key_id_valid;
    const uint8_t active_public_key_id = d_active_public_key_id;

    reset_dsm_accumulators();
    d_nav_data_manager->reset_tag_accumulations();
    reset_tesla_chain_state();
    d_osnma_data.d_dsm_kroot_message = DSM_KROOT_message();
    d_osnma_data.d_dsm_kroot_new_message = keep_future_kroot ? verified_future_kroot : DSM_KROOT_message();
    d_kroot_verified = false;
    d_last_verified_kroot_GST = verified_future_kroot_gst;
    d_kroot_loaded_from_cache = false;
    d_flag_merkle_tree_renewal = false;
    d_GST_merkle_tree_renewal_start = 0;
    d_new_merkle_tree_loaded = false;
    d_merkle_root_at_renewal_start.clear();
    d_merkle_hash_function_at_renewal_start.clear();
    d_material_manager->clear_candidate_merkle_tree();

    if (chain_revocation)
        {
            d_flag_chain_revocation = true;
            if (d_GST_chain_revocation_start == 0)
                {
                    d_GST_chain_revocation_start = d_GST_SIS;
                }
        }
    if (public_key_revocation)
        {
            d_flag_PK_revocation = true;
            d_flag_PK_renewal = false;
            d_flag_NPK_set = false;
            d_new_public_key.clear();
            d_new_public_key_id = 0;
            d_material_manager->clear_candidate_public_key();
            if (keep_active_public_key)
                {
                    d_material_manager->set_active_public_key(active_public_key);
                    d_public_key_verified = true;
                    d_active_public_key_id = active_public_key_id;
                    d_active_public_key_id_valid = true;
                }
            else
                {
                    d_material_manager->set_active_public_key(Osnma_Public_Key_Material());
                    d_public_key_verified = false;
                    d_active_public_key_id = 0;
                    d_active_public_key_id_valid = false;
                }
            if (d_GST_PKR_PKREV_start == 0)
                {
                    d_GST_PKR_PKREV_start = d_GST_SIS;
                }
        }
}


void osnma_msg_receiver::handle_verified_alert_message()
{
    d_flag_alert_message = true;
    d_flag_alert_message_verified = true;
    if (d_GST_PKR_AM_start == 0)
        {
            d_GST_PKR_AM_start = d_GST_SIS;
        }
    reset_dsm_accumulators();
    d_nav_data_manager->reset_tag_accumulations();
    reset_tesla_chain_state();
    d_osnma_data.d_dsm_kroot_message = DSM_KROOT_message();
    d_osnma_data.d_dsm_kroot_new_message = DSM_KROOT_message();
    d_new_public_key.clear();
    d_new_public_key_id = 0;
    d_material_manager->set_active_public_key(Osnma_Public_Key_Material());
    d_material_manager->set_active_merkle_tree(Osnma_Merkle_Tree_Material());
    d_material_manager->clear_candidate_public_key();
    d_material_manager->clear_candidate_merkle_tree();
    d_crypto->set_merkle_root({});
    d_public_key_verified = false;
    d_kroot_verified = false;
    d_last_verified_kroot_GST = 0;
    d_kroot_loaded_from_cache = false;
    d_flag_merkle_tree_renewal = false;
    d_GST_merkle_tree_renewal_start = 0;
    d_new_merkle_tree_loaded = false;
    d_merkle_root_at_renewal_start.clear();
    d_merkle_hash_function_at_renewal_start.clear();
    d_active_public_key_id = 0;
    d_active_public_key_id_valid = false;
    d_count_failed_tags = 0;
}


void osnma_msg_receiver::handle_authenticated_dont_use_status()
{
    LOG(WARNING) << "Galileo OSNMA: NMA Status 'Don't Use' authenticated. Navigation data authentication is suspended until DSM-KROOT is reverified.";
    std::cout << "Galileo OSNMA: NMA Status 'Don't Use' authenticated. Navigation data authentication is suspended until DSM-KROOT is reverified." << std::endl;
    reset_dsm_accumulators();
    d_nav_data_manager->reset_tag_accumulations();
    invalidate_verified_kroot();
    reset_tesla_chain_state();
}


/*
 * case DSM-Kroot:
 * - computes the padding and compares with received message
 * - if successful, tries to verify the digital signature
 * case DSM-PKR:
 * - calls verify_dsm_pkr to verify the public key
 * */
void osnma_msg_receiver::process_dsm_message(const std::vector<uint8_t>& dsm_msg, const uint8_t& nma_header)
{
    const uint8_t dsm_nmas = d_dsm_reader->get_nmas(nma_header);
    const uint8_t dsm_cid = d_dsm_reader->get_cid(nma_header);
    const uint8_t dsm_cpks = d_dsm_reader->get_cpks(nma_header);

    // DSM-KROOT message
    if ((d_osnma_data.d_dsm_header.dsm_id < 12 || d_flag_hot_start) && d_public_key_verified)
        {
            if (dsm_msg.size() < 13)
                {
                    LOG(WARNING) << "Galileo OSNMA: Failed length reading of DSM-KROOT message";
                    d_count_failed_Kroot++;
                    return;
                }
            const uint8_t nb_dk = d_dsm_reader->get_number_blocks_index(dsm_msg[0]);
            const uint8_t pkid = d_dsm_reader->get_pkid(dsm_msg);
            const uint8_t cidkr = d_dsm_reader->get_cidkr(dsm_msg);
            const uint8_t reserved1 = d_dsm_reader->get_dsm_reserved1(dsm_msg);
            const uint8_t hf = d_dsm_reader->get_hf(dsm_msg);
            const uint8_t mf = d_dsm_reader->get_mf(dsm_msg);
            const uint8_t ks = d_dsm_reader->get_ks(dsm_msg);
            const uint8_t ts = d_dsm_reader->get_ts(dsm_msg);
            const uint8_t maclt = d_dsm_reader->get_maclt(dsm_msg);
            const uint8_t reserved = d_dsm_reader->get_dsm_reserved(dsm_msg);
            const uint16_t wn_k = d_dsm_reader->get_wn_k(dsm_msg);
            const uint8_t towh_k = d_dsm_reader->get_towh_k(dsm_msg);
            const uint64_t alpha = d_dsm_reader->get_alpha(dsm_msg);
            const bool authenticated_operational_or_test = dsm_nmas == 1 /* Test */ || dsm_nmas == 2 /* OP */;
            const bool authenticated_eoc = authenticated_operational_or_test && dsm_cpks == 2 /* EOC */;
            const bool authenticated_nominal = authenticated_operational_or_test && dsm_cpks == 1 /* Nominal */;
            const bool authenticated_public_key_renewal = authenticated_operational_or_test && dsm_cpks == 4 /* NPK */;
            const bool authenticated_chain_revocation = dsm_nmas == 3 /* DU */ && dsm_cpks == 3 /* CREV */;
            const bool authenticated_public_key_revocation = dsm_nmas == 3 /* DU */ && dsm_cpks == 5 /* PKREV */;
            const bool authenticated_merkle_tree_renewal = authenticated_operational_or_test && dsm_cpks == 6 /* NMT */;
            const bool authenticated_alert_message = dsm_nmas == 3 /* DU */ && dsm_cpks == 7 /* AM */;
            const bool new_chain = (cidkr != dsm_cid) && (authenticated_eoc || authenticated_chain_revocation || authenticated_public_key_revocation);
            const auto candidate_public_key_material = d_material_manager->candidate_public_key(pkid);
            const bool verify_with_material_manager_candidate =
                candidate_public_key_material.valid && !candidate_public_key_material.compressed_key.empty();
            const bool verify_with_legacy_candidate = !d_new_public_key.empty() && pkid == d_new_public_key_id;
            const bool verify_with_new_public_key = verify_with_material_manager_candidate || verify_with_legacy_candidate;
            const auto& new_public_key_for_verification =
                verify_with_material_manager_candidate ? candidate_public_key_material.compressed_key : d_new_public_key;
            if (!verify_with_new_public_key && d_active_public_key_id_valid && pkid != d_active_public_key_id)
                {
                    const std::string warning =
                        "Galileo OSNMA: DSM-KROOT PKID=" + std::to_string(static_cast<uint32_t>(pkid)) +
                        " does not match active Public Key ID=" +
                        std::to_string(static_cast<uint32_t>(d_active_public_key_id)) +
                        ". Rejecting DSM-KROOT candidate and keeping active cryptographic material.";
                    LOG(WARNING) << warning;
                    std::cerr << warning << std::endl;
                    d_count_failed_Kroot++;
                    return;
                }

            // Parse Kroot message
            LOG(INFO) << "Galileo OSNMA: DSM-KROOT message received.";
            // Kroot field
            const uint16_t l_lk_bits = d_dsm_reader->get_lk_bits(ks);
            const size_t l_lk_bytes = l_lk_bits / 8;
            // DS field
            uint16_t l_ds_bits = 0;
            std::string public_key_type = d_crypto->get_public_key_type();
            if (verify_with_new_public_key)
                {
                    if (verify_with_material_manager_candidate && !candidate_public_key_material.key_type.empty())
                        {
                            public_key_type = candidate_public_key_material.key_type;
                        }
                    else if (new_public_key_for_verification.size() == 33)
                        {
                            public_key_type = "ECDSA P-256";
                        }
                    else if (new_public_key_for_verification.size() == 67)
                        {
                            public_key_type = "ECDSA P-521";
                        }
                    else
                        {
                            public_key_type = "Unknown";
                        }
                }
            const auto it = OSNMA_TABLE_15.find(public_key_type);
            if (it != OSNMA_TABLE_15.cend())
                {
                    l_ds_bits = it->second;
                }
            const size_t l_ds_bytes = l_ds_bits / 8;
            // Padding
            const uint16_t l_dk_bits = d_dsm_reader->get_l_dk_bits(nb_dk);
            const size_t l_dk_bytes = l_dk_bits / 8;
            const uint16_t check_l_dk = 104 * std::ceil(1.0 + static_cast<float>((l_lk_bytes * 8.0) + l_ds_bits) / 104.0);
            const size_t fields_length_bytes = 13 + l_lk_bytes + l_ds_bytes;
            if (l_lk_bits == 0 || l_ds_bits == 0 || l_dk_bits == 0 || l_dk_bits != check_l_dk ||
                l_dk_bytes < fields_length_bytes || dsm_msg.size() < l_dk_bytes)
                {
                    LOG(WARNING) << "Galileo OSNMA: Failed length reading of DSM-KROOT message";
                    d_count_failed_Kroot++;
                    return;
                }
            else
                {
                    const size_t l_pdk_bytes = l_dk_bytes - fields_length_bytes;
                    DSM_KROOT_message candidate_kroot_msg;
                    candidate_kroot_msg.nb_dk = nb_dk;
                    candidate_kroot_msg.pkid = pkid;
                    candidate_kroot_msg.cidkr = cidkr;
                    candidate_kroot_msg.reserved1 = reserved1;
                    candidate_kroot_msg.hf = hf;
                    candidate_kroot_msg.mf = mf;
                    candidate_kroot_msg.ks = ks;
                    candidate_kroot_msg.ts = ts;
                    candidate_kroot_msg.maclt = maclt;
                    candidate_kroot_msg.reserved = reserved;
                    candidate_kroot_msg.wn_k = wn_k;
                    candidate_kroot_msg.towh_k = towh_k;
                    candidate_kroot_msg.alpha = alpha;
                    candidate_kroot_msg.kroot.assign(dsm_msg.cbegin() + 13, dsm_msg.cbegin() + 13 + l_lk_bytes);
                    candidate_kroot_msg.ds.assign(dsm_msg.cbegin() + fields_length_bytes - l_ds_bytes, dsm_msg.cbegin() + fields_length_bytes);
                    candidate_kroot_msg.p_dk.assign(dsm_msg.cbegin() + fields_length_bytes, dsm_msg.cbegin() + l_dk_bytes);
                    if (!kroot_parameters_are_supported(candidate_kroot_msg))
                        {
                            LOG(WARNING) << "Galileo OSNMA: Unsupported DSM-KROOT parameters";
                            d_count_failed_Kroot++;
                            return;
                        }

                    // validation of padding
                    const size_t size_m = 13 + l_lk_bytes;
                    std::vector<uint8_t> MSG;
                    MSG.reserve(size_m + l_ds_bytes + 1);
                    MSG.push_back(nma_header);  // NMA header
                    for (size_t i = 1; i < size_m; i++)
                        {
                            MSG.push_back(dsm_msg[i]);
                        }
                    std::vector<uint8_t> message = MSG;  // MSG = (M | DS) from ICD. Eq. 7
                    for (size_t k = 0; k < l_ds_bytes; k++)
                        {
                            MSG.push_back(candidate_kroot_msg.ds[k]);
                        }

                    const std::vector<uint8_t> hash = d_crypto->compute_SHA_256(MSG);
                    // truncate hash
                    std::vector<uint8_t> p_dk_truncated;
                    p_dk_truncated.reserve(l_pdk_bytes);
                    for (size_t i = 0; i < l_pdk_bytes; i++)
                        {
                            p_dk_truncated.push_back(hash[i]);
                        }
                    // Check that the padding bits received match the computed values
                    if (candidate_kroot_msg.p_dk == p_dk_truncated)
                        {
                            LOG(INFO) << "Galileo OSNMA: DSM-KROOT message received ok.";
                            LOG(INFO) << "Galileo OSNMA: DSM-KROOT with CID=" << static_cast<uint32_t>(d_osnma_data.d_nma_header.cid)
                                      << ", PKID=" << static_cast<uint32_t>(candidate_kroot_msg.pkid)
                                      << ", WN=" << static_cast<uint32_t>(candidate_kroot_msg.wn_k)
                                      << ", TOW=" << static_cast<uint32_t>(candidate_kroot_msg.towh_k) * 3600;

                            const bool loaded_from_cache = d_flag_hot_start;
                            bool candidate_kroot_verified = false;
                            if (l_ds_bits == 512)
                                {
                                    if (verify_with_new_public_key)
                                        {
                                            Gnss_Crypto candidate_crypto;
                                            candidate_crypto.set_public_key(new_public_key_for_verification);
                                            candidate_kroot_verified = candidate_crypto.verify_signature_ecdsa_p256(message, candidate_kroot_msg.ds);
                                        }
                                    else
                                        {
                                            candidate_kroot_verified = d_crypto->verify_signature_ecdsa_p256(message, candidate_kroot_msg.ds);
                                        }
                                }
                            else if (l_ds_bits == 1056)
                                {
                                    if (verify_with_new_public_key)
                                        {
                                            Gnss_Crypto candidate_crypto;
                                            candidate_crypto.set_public_key(new_public_key_for_verification);
                                            candidate_kroot_verified = candidate_crypto.verify_signature_ecdsa_p521(message, candidate_kroot_msg.ds);
                                        }
                                    else
                                        {
                                            candidate_kroot_verified = d_crypto->verify_signature_ecdsa_p521(message, candidate_kroot_msg.ds);
                                        }
                                }
                            if (candidate_kroot_verified)
                                {
                                    candidate_kroot_msg.verified = true;
                                    const bool updates_active_kroot = !new_chain;
                                    const bool had_active_kroot = d_kroot_verified && d_osnma_data.d_dsm_kroot_message.verified;
                                    const bool active_kroot_changes = updates_active_kroot &&
                                                                      had_active_kroot &&
                                                                      !kroot_mack_parameters_equal(d_osnma_data.d_dsm_kroot_message, candidate_kroot_msg);
                                    const bool first_active_kroot = updates_active_kroot && !had_active_kroot;
                                    DSM_KROOT_message& applicable_kroot_msg = new_chain ? d_osnma_data.d_dsm_kroot_new_message : d_osnma_data.d_dsm_kroot_message;
                                    applicable_kroot_msg = std::move(candidate_kroot_msg);
                                    d_kroot_verified = true;
                                    d_kroot_loaded_from_cache = loaded_from_cache;
                                    if (active_kroot_changes)
                                        {
                                            LOG(INFO) << "Galileo OSNMA: active DSM-KROOT parameters changed. Clearing pending TESLA/MACK/tag state.";
                                            d_nav_data_manager->reset_tag_accumulations();
                                            reset_tesla_chain_state();
                                        }
                                    else if (first_active_kroot)
                                        {
                                            reset_tesla_chain_state(true);
                                        }
                                    if (loaded_from_cache && d_pending_dsm_kroot_cache.metadata_valid)
                                        {
                                            d_last_verified_kroot_GST = d_pending_dsm_kroot_cache.signature_verified_at_gst;
                                        }
                                    else if (d_GST_SIS != 0)
                                        {
                                            d_last_verified_kroot_GST = d_GST_SIS;
                                        }
                                    std::cout << "Galileo OSNMA: DSM-KROOT authentication successful!" << std::endl;
                                    LOG(INFO) << "Galileo OSNMA: DSM-KROOT authentication successful for WNk="
                                              << static_cast<uint32_t>(applicable_kroot_msg.wn_k)
                                              << " and TOWHk="
                                              << static_cast<uint32_t>(applicable_kroot_msg.towh_k) * 3600;
                                    if (authenticated_alert_message)
                                        {
                                            LOG(WARNING) << "Galileo OSNMA: DSM-KROOT :: Alert message verification :: SUCCESS. ";
                                        }
                                    else
                                        {
                                            LOG(INFO) << "Galileo OSNMA: NMA Status is " << d_dsm_reader->get_nmas_status(dsm_nmas) << ", "
                                                      << "Chain in force is " << static_cast<uint32_t>(dsm_cid) << ", "
                                                      << "Chain and Public Key Status is " << d_dsm_reader->get_cpks_status(dsm_cpks);
                                        }
                                    if (verify_with_new_public_key)
                                        {
                                            d_crypto->set_public_key(new_public_key_for_verification);
                                            d_crypto->store_public_key(PEMFILE_DEFAULT);
                                            d_active_public_key_id = applicable_kroot_msg.pkid;
                                            d_active_public_key_id_valid = true;
                                            if (verify_with_material_manager_candidate)
                                                {
                                                    d_material_manager->promote_candidate_for_pkid(applicable_kroot_msg.pkid);
                                                }
                                            else
                                                {
                                                    Osnma_Public_Key_Material active_public_key;
                                                    active_public_key.valid = true;
                                                    active_public_key.pkid_valid = true;
                                                    active_public_key.pkid = applicable_kroot_msg.pkid;
                                                    active_public_key.npkt_valid = candidate_public_key_material.npkt_valid;
                                                    active_public_key.npkt = candidate_public_key_material.npkt;
                                                    active_public_key.key_type = d_crypto->get_public_key_type();
                                                    active_public_key.source = "DSM-PKR";
                                                    active_public_key.compressed_key = new_public_key_for_verification;
                                                    active_public_key.fingerprint_sha256 = candidate_public_key_material.fingerprint_sha256;
                                                    d_material_manager->set_active_public_key(active_public_key);
                                                }
                                            const uint8_t applicable_npkt = candidate_public_key_material.npkt_valid
                                                                                ? candidate_public_key_material.npkt
                                                                                : d_osnma_data.d_dsm_pkr_message.npkt;
                                            store_public_key_metadata(applicable_kroot_msg.pkid, applicable_npkt, "DSM-PKR");
                                            if (d_flag_merkle_tree_renewal && d_new_merkle_tree_loaded && d_material_manager->promote_candidate_merkle_tree())
                                                {
                                                    const auto active_merkle_tree = d_material_manager->active_merkle_tree();
                                                    d_crypto->set_merkle_root(active_merkle_tree.root);
                                                    d_crypto->set_merkle_tree_hash_function(active_merkle_tree.hash_function);
                                                }
                                            d_flag_NPK_set = false;
                                            d_new_public_key_id = 0;
                                            d_new_public_key.clear();
                                        }
                                    else if (!d_active_public_key_id_valid)
                                        {
                                            d_active_public_key_id = applicable_kroot_msg.pkid;
                                            d_active_public_key_id_valid = true;
                                            auto active_public_key = d_material_manager->active_public_key();
                                            active_public_key.valid = true;
                                            active_public_key.pkid_valid = true;
                                            active_public_key.pkid = applicable_kroot_msg.pkid;
                                            active_public_key.key_type = d_crypto->get_public_key_type();
                                            uint8_t active_npkt = 0;
                                            if (npkt_from_public_key_type(active_public_key.key_type, active_npkt))
                                                {
                                                    active_public_key.npkt_valid = true;
                                                    active_public_key.npkt = active_npkt;
                                                }
                                            d_material_manager->set_active_public_key(active_public_key);
                                        }
                                    if (authenticated_public_key_renewal)
                                        {
                                            d_flag_PK_renewal = true;
                                            d_flag_NPK_set = !d_new_public_key.empty();
                                            if (d_GST_PKR_PKREV_start == 0)
                                                {
                                                    d_GST_PKR_PKREV_start = d_GST_SIS;
                                                }
                                        }
                                    if (authenticated_eoc)
                                        {
                                            d_flag_chain_renewal = true;
                                            if (d_GST_chain_renewal_start == 0)
                                                {
                                                    d_GST_chain_renewal_start = d_GST_SIS;
                                                }
                                        }
                                    if (authenticated_merkle_tree_renewal)
                                        {
                                            if (!d_flag_merkle_tree_renewal)
                                                {
                                                    d_merkle_root_at_renewal_start = d_crypto->get_merkle_root();
                                                    d_merkle_hash_function_at_renewal_start = d_crypto->get_merkle_tree_hash_function();
                                                    d_new_merkle_tree_loaded = false;
                                                }
                                            d_flag_merkle_tree_renewal = true;
                                            if (d_GST_merkle_tree_renewal_start == 0)
                                                {
                                                    d_GST_merkle_tree_renewal_start = d_GST_SIS;
                                                }
                                        }
                                    if (d_flag_chain_renewal && authenticated_nominal)
                                        {
                                            if (d_osnma_data.d_dsm_kroot_new_message.verified &&
                                                d_osnma_data.d_dsm_kroot_new_message.cidkr == dsm_cid)
                                                {
                                                    d_osnma_data.d_dsm_kroot_message = d_osnma_data.d_dsm_kroot_new_message;
                                                }
                                            d_flag_chain_renewal = false;
                                            d_osnma_data.d_dsm_kroot_new_message = DSM_KROOT_message();
                                            reset_tesla_chain_state();
                                            d_kroot_verified = d_osnma_data.d_dsm_kroot_message.verified;
                                            d_kroot_loaded_from_cache = false;
                                            d_GST_chain_renewal_start = 0;
                                        }
                                    if (authenticated_nominal)
                                        {
                                            if (d_flag_PK_renewal)
                                                {
                                                    d_flag_PK_renewal = false;
                                                    d_flag_NPK_set = false;
                                                    d_new_public_key.clear();
                                                    d_new_public_key_id = 0;
                                                    d_material_manager->clear_candidate_public_key();
                                                    d_GST_PKR_PKREV_start = 0;
                                                }
                                            if (d_flag_PK_revocation)
                                                {
                                                    d_flag_PK_revocation = false;
                                                    d_GST_PKR_PKREV_start = 0;
                                                }
                                            if (d_flag_chain_revocation)
                                                {
                                                    d_flag_chain_revocation = false;
                                                    d_GST_chain_revocation_start = 0;
                                                }
                                            if (d_flag_merkle_tree_renewal)
                                                {
                                                    d_flag_merkle_tree_renewal = false;
                                                    d_GST_merkle_tree_renewal_start = 0;
                                                    d_new_merkle_tree_loaded = false;
                                                    d_merkle_root_at_renewal_start.clear();
                                                    d_merkle_hash_function_at_renewal_start.clear();
                                                    d_material_manager->clear_candidate_merkle_tree();
                                                }
                                        }
                                    if (authenticated_chain_revocation || authenticated_public_key_revocation)
                                        {
                                            const bool preserve_active_public_key = d_active_public_key_id_valid && d_active_public_key_id == applicable_kroot_msg.pkid;
                                            handle_authenticated_revocation(authenticated_chain_revocation, authenticated_public_key_revocation, new_chain, preserve_active_public_key);
                                            return;
                                        }
                                    if (loaded_from_cache)
                                        {
                                            d_flag_hot_start = false;
                                            if (!d_active_public_key_id_valid)
                                                {
                                                    d_active_public_key_id = applicable_kroot_msg.pkid;
                                                    d_active_public_key_id_valid = true;
                                                }
                                            LOG(INFO) << "Galileo OSNMA: verified DSM-KROOT loaded from persistent storage. Using it as TESLA-chain anchor.";
                                            std::cout << "Galileo OSNMA: verified DSM-KROOT loaded from persistent storage. Using it as TESLA-chain anchor." << std::endl;
                                            return;
                                        }
                                    if (authenticated_alert_message)
                                        {
                                            handle_verified_alert_message();
                                            return;
                                        }
                                    // Save DSM-Kroot and NMA header into a permanent storage
                                    store_dsm_kroot(dsm_msg, nma_header, applicable_kroot_msg);
                                }
                            else
                                {
                                    LOG(WARNING) << "Galileo OSNMA: DSM-KROOT authentication failed.";
                                    std::cerr << "Galileo OSNMA: DSM-KROOT authentication failed." << std::endl;
                                    if (loaded_from_cache)
                                        {
                                            LOG(INFO) << "Galileo OSNMA: stored DSM-KROOT cache authentication failed. Ignoring cache and falling back to :: WARM START.";
                                            std::cout << "Galileo OSNMA: stored DSM-KROOT cache authentication failed. Ignoring cache and falling back to :: WARM START." << std::endl;
                                            d_count_failed_Kroot++;
                                            return;
                                        }
                                    if (!verify_with_new_public_key && d_active_public_key_id_valid && candidate_kroot_msg.pkid == d_active_public_key_id)
                                        {
                                            const std::string warning =
                                                "Galileo OSNMA: DSM-KROOT authentication failed with matching active Public Key ID=" +
                                                std::to_string(static_cast<uint32_t>(d_active_public_key_id)) +
                                                ". Rejecting DSM-KROOT candidate and keeping active cryptographic material.";
                                            LOG(WARNING) << warning;
                                            std::cerr << warning << std::endl;
                                        }
                                    if (d_flag_alert_message)
                                        {
                                            d_flag_alert_message = false;
                                        }
                                    d_count_failed_Kroot++;
                                }
                        }
                    else
                        {
                            LOG(WARNING) << "Galileo OSNMA: Error computing padding bits.";
                            d_count_failed_Kroot++;
                        }
                }
        }

    // DSM-PKR message
    else if (d_osnma_data.d_dsm_header.dsm_id >= 12 && d_osnma_data.d_dsm_header.dsm_id < 16)
        {
            LOG(INFO) << "Galileo OSNMA: DSM-PKR message received";
            constexpr size_t pkr_header_bytes = 130;
            if (dsm_msg.size() < pkr_header_bytes)
                {
                    LOG(WARNING) << "Galileo OSNMA: Failed length reading of DSM-PKR message";
                    d_flag_alert_message = false;
                    return;
                }
            // Save DSM-PKR message
            DSM_PKR_message dsm_pkr_message;
            dsm_pkr_message.nb_dp = d_dsm_reader->get_number_blocks_index(dsm_msg[0]);
            dsm_pkr_message.mid = d_dsm_reader->get_mid(dsm_msg);
            for (int k = 0; k < 128; k++)
                {
                    dsm_pkr_message.itn[k] = dsm_msg[k + 1];
                }
            dsm_pkr_message.npkt = d_dsm_reader->get_npkt(dsm_msg);
            uint8_t npktid = d_dsm_reader->get_npktid(dsm_msg);
            dsm_pkr_message.npktid = npktid;

            size_t l_npk_bytes = 0;
            std::string PKT;
            const auto it = OSNMA_TABLE_5.find(dsm_pkr_message.npkt);
            if (it != OSNMA_TABLE_5.cend())
                {
                    PKT = it->second;
                    const auto it2 = OSNMA_TABLE_6.find(it->second);
                    if (it2 != OSNMA_TABLE_6.cend())
                        {
                            l_npk_bytes = it2->second / 8;
                        }
                }
            const size_t l_dp_bytes = dsm_msg.size();
            const bool is_alert_dsm_pkr = dsm_pkr_message.npkt == 4 && dsm_pkr_message.npktid == 0;
            if (is_alert_dsm_pkr)
                {
                    LOG(WARNING) << "Galileo OSNMA: DSM-PKR :: Alert message received. Verifying it.";
                    std::cout << "Galileo OSNMA: DSM-PKR :: Alert message received. Verifying it." << std::endl;
                    l_npk_bytes = l_dp_bytes - pkr_header_bytes;  // bytes
                }

            const size_t check_l_dp_bytes = 104 * std::ceil(static_cast<float>(1040.0 + l_npk_bytes * 8.0) / 104.0) / 8;
            if (l_npk_bytes == 0 || l_npk_bytes > l_dp_bytes - pkr_header_bytes || l_dp_bytes != check_l_dp_bytes)
                {
                    LOG(WARNING) << "Galileo OSNMA: Failed length reading of DSM-PKR message";
                    d_flag_alert_message = false;
                    return;
                }
            else
                {
                    const size_t l_pd_bytes = l_dp_bytes - pkr_header_bytes - l_npk_bytes;
                    dsm_pkr_message.npk.assign(dsm_msg.cbegin() + pkr_header_bytes, dsm_msg.cbegin() + pkr_header_bytes + l_npk_bytes);
                    dsm_pkr_message.p_dp.assign(dsm_msg.cbegin() + l_dp_bytes - l_pd_bytes, dsm_msg.cend());
                    d_osnma_data.d_dsm_pkr_message = std::move(dsm_pkr_message);
                    if (!is_alert_dsm_pkr &&
                        d_active_public_key_id_valid &&
                        npktid < d_active_public_key_id &&
                        !d_flag_merkle_tree_renewal)
                        {
                            LOG(WARNING) << "Galileo OSNMA: DSM-PKR PKID="
                                         << static_cast<uint32_t>(npktid)
                                         << " is lower than active Public Key ID="
                                         << static_cast<uint32_t>(d_active_public_key_id)
                                         << ". Rejecting DSM-PKR.";
                            d_count_failed_pubKey++;
                            return;
                        }
                    LOG(INFO) << "Galileo OSNMA: DSM-PKR with CID=" << static_cast<uint32_t>(dsm_cid)
                              << ", PKID=" << static_cast<uint32_t>(d_osnma_data.d_dsm_pkr_message.npktid) << " received";
                    Osnma_Merkle_Tree_Material merkle_tree_for_pkr;
                    if (d_flag_merkle_tree_renewal && !ensure_new_merkle_tree_available())
                        {
                            LOG(WARNING) << "Galileo OSNMA: DSM-PKR verification postponed. New Merkle Tree status is authenticated, but no new Merkle tree root is loaded.";
                            return;
                        }
                    if (d_flag_merkle_tree_renewal)
                        {
                            merkle_tree_for_pkr = d_material_manager->candidate_merkle_tree();
                        }
                    else
                        {
                            merkle_tree_for_pkr = d_material_manager->active_merkle_tree();
                        }
                    // Public key verification against Merkle tree root.
                    bool verification = verify_dsm_pkr(d_osnma_data.d_dsm_pkr_message, merkle_tree_for_pkr);
                    if (verification)
                        {
                            LOG(INFO) << "Galileo OSNMA: DSM-PKR verification :: SUCCESS";
                            const bool had_public_key_verified = d_public_key_verified;
                            if (is_alert_dsm_pkr)
                                {
                                    LOG(WARNING) << "Galileo OSNMA: DSM-PKR verification :: Alert message verification :: SUCCESS. OSNMA disabled. Contact Galileo Service Centre";
                                    std::cout << "Galileo OSNMA: DSM-PKR verification :: Alert message verification :: SUCCESS. OSNMA disabled. Contact Galileo Service Centre" << std::endl;
                                    handle_verified_alert_message();
                                }
                            else if (had_public_key_verified || d_kroot_verified || d_flag_PK_renewal || d_flag_PK_revocation || d_flag_merkle_tree_renewal)
                                {
                                    d_public_key_verified = true;
                                    d_flag_NPK_set = true;
                                    d_new_public_key_id = npktid;
                                    d_new_public_key = d_osnma_data.d_dsm_pkr_message.npk;
                                    Osnma_Public_Key_Material candidate_public_key;
                                    candidate_public_key.pkid_valid = true;
                                    candidate_public_key.pkid = npktid;
                                    candidate_public_key.npkt_valid = true;
                                    candidate_public_key.npkt = d_osnma_data.d_dsm_pkr_message.npkt;
                                    candidate_public_key.key_type = PKT;
                                    candidate_public_key.source = "DSM-PKR";
                                    candidate_public_key.compressed_key = d_new_public_key;
                                    candidate_public_key.fingerprint_sha256 =
                                        public_key_fingerprint_sha256(npktid, d_osnma_data.d_dsm_pkr_message.npkt, d_osnma_data.d_dsm_pkr_message.npk);
                                    candidate_public_key.valid = !candidate_public_key.fingerprint_sha256.empty();
                                    d_material_manager->set_candidate_public_key(candidate_public_key);
                                }
                            else
                                {
                                    d_public_key_verified = true;
                                    d_crypto->set_public_key_type(PKT);
                                    d_crypto->set_public_key(d_osnma_data.d_dsm_pkr_message.npk);
                                    d_crypto->store_public_key(PEMFILE_DEFAULT);
                                    store_public_key_metadata(npktid, d_osnma_data.d_dsm_pkr_message.npkt, "DSM-PKR");
                                    d_active_public_key_id = npktid;
                                    d_active_public_key_id_valid = true;
                                }
                        }
                    else
                        {
                            LOG(ERROR) << "Galileo OSNMA: DSM-PKR verification :: FAILURE";
                            d_count_failed_pubKey++;
                            if (d_flag_alert_message)
                                {
                                    d_flag_alert_message = false;  // disregard message as its authenticity could not be verified.
                                }
                        }
                }
        }
    else
        {
            if (!d_public_key_verified && d_osnma_data.d_dsm_header.dsm_id < 12)
                {
                    LOG(INFO) << "Galileo OSNMA: DSM-KROOT message received but no Public Key available to authenticate the TESLA root key";
                    std::cerr << "Galileo OSNMA: DSM-KROOT message received but no Public Key available to authenticate the TESLA root key" << std::endl;
                }
            else
                {
                    LOG(INFO) << "Galileo OSNMA: Reserved message received";
                    std::cerr << "Galileo OSNMA: Reserved message received" << std::endl;
                }
        }
    if (d_osnma_data.d_dsm_header.dsm_id < d_number_of_blocks.size())
        {
            d_number_of_blocks[d_osnma_data.d_dsm_header.dsm_id] = 0;
        }
}


/**
 * @brief Reads the Mack message from the given OSNMA_msg object
 *
 * @details Conditions for MACK processing:
 * @param osnma_msg The OSNMA_msg object containing the Mack message.
 */
void osnma_msg_receiver::read_and_process_mack_block(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    const uint32_t current_gst = d_helper->compute_gst(osnma_msg->WN_sf0, osnma_msg->TOW_sf0);
    expire_verified_kroot_if_needed(current_gst);

    if (!process_mack_block(osnma_msg, d_tags_to_verify, d_osnma_data.d_nma_header.nmas))
        {
            store_deferred_mack_block(osnma_msg);
        }
}


void osnma_msg_receiver::store_deferred_mack_block(const std::shared_ptr<OSNMA_msg>& osnma_msg)
{
    const uint32_t current_gst = d_helper->compute_gst(osnma_msg->WN_sf0, osnma_msg->TOW_sf0);
    for (auto it = d_mack_blocks_awaiting_kroot.begin(); it != d_mack_blocks_awaiting_kroot.end();)
        {
            const uint32_t latest_useful_gst = gst_with_offset(it->WN_sf0, it->TOW_sf0, mack_deferred_expiration_s);
            if (gst_delta_seconds(current_gst, latest_useful_gst) > 0)
                {
                    it = d_mack_blocks_awaiting_kroot.erase(it);
                }
            else
                {
                    ++it;
                }
        }

    DeferredMackBlock block;
    block.mack = osnma_msg->mack;
    block.page_validity_available = osnma_msg_has_page_validity(*osnma_msg);
    if (block.page_validity_available)
        {
            block.page_valid = osnma_msg->page_valid;
        }
    else
        {
            block.page_valid.fill(1);
        }
    block.allowed_adkds = d_tags_to_verify;
    block.PRN = osnma_msg->PRN;
    block.WN_sf0 = osnma_msg->WN_sf0;
    block.TOW_sf0 = osnma_msg->TOW_sf0;
    block.nmas = d_osnma_data.d_nma_header.nmas;

    for (auto& deferred : d_mack_blocks_awaiting_kroot)
        {
            if (deferred.PRN == block.PRN &&
                deferred.WN_sf0 == block.WN_sf0 &&
                deferred.TOW_sf0 == block.TOW_sf0)
                {
                    deferred = std::move(block);
                    return;
                }
        }

    if (d_mack_blocks_awaiting_kroot.size() >= max_deferred_mack_blocks)
        {
            d_mack_blocks_awaiting_kroot.erase(d_mack_blocks_awaiting_kroot.begin());
        }
    d_mack_blocks_awaiting_kroot.push_back(std::move(block));

    LOG(INFO) << "Galileo OSNMA: queued MACK block until DSM-KROOT parameters are verified. WN="
              << osnma_msg->WN_sf0 << ", TOW=" << osnma_msg->TOW_sf0
              << ", PRNa=" << osnma_msg->PRN;
}


void osnma_msg_receiver::process_deferred_mack_blocks()
{
    if (d_mack_blocks_awaiting_kroot.empty())
        {
            return;
        }

    std::vector<DeferredMackBlock> deferred_blocks = std::move(d_mack_blocks_awaiting_kroot);
    d_mack_blocks_awaiting_kroot.clear();

    const uint32_t current_gst = d_GST_SIS != 0 ? d_GST_SIS : d_last_received_GST;
    for (const auto& block : deferred_blocks)
        {
            const uint32_t latest_useful_gst = gst_with_offset(block.WN_sf0, block.TOW_sf0, mack_deferred_expiration_s);
            if (current_gst != 0 && gst_delta_seconds(current_gst, latest_useful_gst) > 0)
                {
                    continue;
                }

            auto msg = std::make_shared<OSNMA_msg>();
            msg->mack = block.mack;
            msg->page_valid = block.page_valid;
            msg->page_validity_available = block.page_validity_available;
            msg->PRN = block.PRN;
            msg->WN_sf0 = block.WN_sf0;
            msg->TOW_sf0 = block.TOW_sf0;

            if (!process_mack_block(msg, block.allowed_adkds, block.nmas))
                {
                    d_mack_blocks_awaiting_kroot.push_back(block);
                }
            else if (d_flag_alert_message_verified || (!d_kroot_verified && !d_tesla_key_verified))
                {
                    d_mack_blocks_awaiting_kroot.clear();
                    break;
                }
        }
}


bool osnma_msg_receiver::process_mack_block(const std::shared_ptr<OSNMA_msg>& osnma_msg, const std::vector<uint8_t>& allowed_adkds, uint8_t nmas)
{
    // Retrieve Mack message
    d_mack_page_validity_available = osnma_msg_has_page_validity(*osnma_msg);
    if (d_mack_page_validity_available)
        {
            d_mack_page_received = osnma_msg->page_valid;
        }
    else
        {
            d_mack_page_received.fill(1);
        }
    uint32_t index = 0;
    for (uint32_t value : osnma_msg->mack)
        {
            d_mack_message[index] = static_cast<uint8_t>((value & 0xFF000000) >> 24);
            d_mack_message[index + 1] = static_cast<uint8_t>((value & 0x00FF0000) >> 16);
            d_mack_message[index + 2] = static_cast<uint8_t>((value & 0x0000FF00) >> 8);
            d_mack_message[index + 3] = static_cast<uint8_t>(value & 0x000000FF);
            index = index + 4;
        }

    d_osnma_data.d_nav_data.set_tow_sf0(osnma_msg->TOW_sf0);
    d_osnma_data.d_mack_message = MACK_message();

    const bool can_parse_tag_fields =
        kroot_parameters_are_supported(d_osnma_data.d_dsm_kroot_message);

    const bool have_verified_kroot_anchor =
        d_kroot_verified &&
        d_osnma_data.d_dsm_kroot_message.verified &&
        can_parse_tag_fields;

    const bool can_verify_tesla_key =
        d_tesla_key_verified || have_verified_kroot_anchor;

    if (can_verify_tesla_key && can_parse_tag_fields)
        {
            if (!read_mack_header())
                {
                    LOG(WARNING) << "Galileo OSNMA: Cannot parse MACK header. Skipping it.";
                    return true;
                }
            d_osnma_data.d_mack_message.PRNa = osnma_msg->PRN;  // FIXME this is ugly.
            d_osnma_data.d_mack_message.TOW = osnma_msg->TOW_sf0;
            d_osnma_data.d_mack_message.WN = osnma_msg->WN_sf0;
            d_osnma_data.d_mack_message.nmas = nmas;
            d_osnma_data.d_mack_message.allowed_adkds = allowed_adkds;
            if (d_kroot_verified && d_osnma_data.d_dsm_kroot_message.verified && !set_gst_sf_for_mack(osnma_msg->WN_sf0, osnma_msg->TOW_sf0))
                {
                    d_osnma_data.d_mack_message = MACK_message();
                    return true;
                }
            else if (!d_kroot_verified || !d_osnma_data.d_dsm_kroot_message.verified)
                {
                    d_GST_Sf = d_helper->compute_gst(osnma_msg->WN_sf0, osnma_msg->TOW_sf0);
                }
            if (!read_mack_body())
                {
                    LOG(WARNING) << "Galileo OSNMA: Cannot parse MACK body. Skipping it.";
                    d_osnma_data.d_mack_message = MACK_message();
                    return true;
                }
            process_mack_message();
            return true;
        }
    else
        {
            if (d_kroot_verified && !can_parse_tag_fields)
                {
                    LOG(WARNING) << "Galileo OSNMA: Verified DSM-KROOT has unsupported parameters. Skipping MACK block.";
                    std::cout << "Galileo OSNMA: Verified DSM-KROOT has unsupported parameters. Skipping MACK block." << std::endl;
                    return true;
                }
            else if (!d_kroot_verified && !d_tesla_key_verified)
                {
                    LOG(INFO) << "Galileo OSNMA: Cannot process MACK block yet. No verified DSM-KROOT or TESLA key available.";
                    return false;
                }
            else
                {
                    LOG(INFO) << "Galileo OSNMA: Cannot process MACK block yet. Waiting for DSM-KROOT parameters.";
                    return false;
                }
        }
    return true;
}


/**
 * \brief Reads the MACK header from the d_mack_message array and updates the d_osnma_data structure.
 * \details This function reads the message MACK header from the d_mack_message array and updates the d_osnma_data structure with the parsed data. The header consists of three fields
 *: tag0, macseq, and cop. The size of the fields is determined by the number of tag length (lt) bits specified in OSNMA_TABLE_11 for the corresponding tag size in d_osnma_data.d_dsm_k
 *root_message.ts. The lt_bits value is used to calculate tag0, MACSEQ, and COP.
 * \pre The d_mack_message array and d_osnma_data.d_dsm_kroot_message.ts field must be properly populated.
 * \post The d_osnma_data.d_mack_message.header.tag0, d_osnma_data.d_mack_message.header.macseq, and d_osnma_data.d_mack_message.header.cop fields are updated with the parsed values
 *.
 * \returns None.
 */
bool osnma_msg_receiver::read_mack_header()
{
    const auto it = OSNMA_TABLE_11.find(d_osnma_data.d_dsm_kroot_message.ts);
    const uint8_t lt_bits = it != OSNMA_TABLE_11.cend() ? it->second : 0;
    if (lt_bits < 16)
        {
            return false;  // The 16 is to avoid negative shifts if shorter tags were defined
        }
    auto& header = d_osnma_data.d_mack_message.header;
    header = MACK_header();
    if (mack_bits_available(0, lt_bits) &&
        mack_bits_available(static_cast<size_t>(lt_bits) + 12U, 4))
        {
            header.tag0 = read_mack_bits(0, lt_bits);
            header.cop = static_cast<uint8_t>(read_mack_bits(static_cast<size_t>(lt_bits) + 12U, 4));
            header.tag0_valid = true;
        }
    else
        {
            LOG(INFO) << "Galileo OSNMA: MACK Tag0 is incomplete at page level. Skipping Tag0.";
        }
    if (mack_bits_available(lt_bits, 12))
        {
            header.macseq = static_cast<uint16_t>(read_mack_bits(lt_bits, 12));
            header.macseq_valid = true;
        }
    else
        {
            LOG(INFO) << "Galileo OSNMA: MACK MACSEQ is incomplete at page level. FLX tags will be skipped.";
        }
    return true;
}


/**
 * @brief Reads the MACK message body
 *
 * \details It retrieves all the tags and tag-info associated, as well as the TESLA key.
 * \post populates d_osnma_data.d_mack_message with all tags and tag_info associated of MACK message, as well as the TESLA key into d_osnma_data.d_mack_message.key
 * @return None
 */
bool osnma_msg_receiver::read_mack_body()
{
    // retrieve tag length
    const auto it = OSNMA_TABLE_11.find(d_osnma_data.d_dsm_kroot_message.ts);
    const uint8_t lt_bits = it != OSNMA_TABLE_11.cend() ? it->second : 0;
    if (lt_bits == 0)
        {
            return false;
        }
    // retrieve key length
    const uint16_t lk_bits = d_dsm_reader->get_lk_bits(d_osnma_data.d_dsm_kroot_message.ks);
    if (lk_bits == 0 || lk_bits >= 480 || (lk_bits % 8) != 0)
        {
            return false;
        }
    // compute number  of tags in the given Mack message as per Eq. 8 ICD
    const uint16_t tag_and_info_bits = static_cast<uint16_t>(lt_bits) + 16U;
    auto nt = static_cast<uint16_t>((480U - lk_bits) / tag_and_info_bits);
    if (nt == 0)
        {
            return false;
        }
    const size_t key_bit_offset = static_cast<size_t>(nt) * tag_and_info_bits;
    if (key_bit_offset + lk_bits > 480)
        {
            return false;
        }
    d_osnma_data.d_mack_message.tag_and_info = std::vector<MACK_tag_and_info>(nt - 1);
    // retrieve tags and tag-info associated with the tags
    for (uint16_t k = 0; k < (nt - 1); k++)
        {
            const size_t tag_bit_offset = static_cast<size_t>(k + 1U) * tag_and_info_bits;
            auto& tag_and_info = d_osnma_data.d_mack_message.tag_and_info[k];
            tag_and_info.counter = k + 2;  // CTR==1 for Tag0, increases subsequently for all other tags.
            if (!mack_bits_available(tag_bit_offset, tag_and_info_bits))
                {
                    tag_and_info.valid = false;
                    LOG(INFO) << "Galileo OSNMA: MACK Tag&Info with CTR="
                              << static_cast<uint32_t>(tag_and_info.counter)
                              << " is incomplete at page level. Skipping this tag.";
                    continue;
                }
            tag_and_info.tag = read_mack_bits(tag_bit_offset, lt_bits);
            tag_and_info.tag_info.PRN_d = static_cast<uint8_t>(read_mack_bits(tag_bit_offset + lt_bits, 8));
            tag_and_info.tag_info.ADKD = static_cast<uint8_t>(read_mack_bits(tag_bit_offset + lt_bits + 8U, 4));
            tag_and_info.tag_info.cop = static_cast<uint8_t>(read_mack_bits(tag_bit_offset + lt_bits + 12U, 4));
            tag_and_info.valid = true;
        }

    const size_t key_bytes_count = lk_bits / 8;
    std::vector<uint8_t> key_bytes(key_bytes_count, 0);
    std::vector<uint8_t> key_byte_received(key_bytes_count, 0);
    for (size_t j = 0; j < key_bytes_count; j++)
        {
            uint8_t key_byte = 0;
            if (read_mack_byte(key_bit_offset + j * 8U, key_byte))
                {
                    key_bytes[j] = key_byte;
                    key_byte_received[j] = 1;
                }
        }
    if (std::all_of(key_byte_received.cbegin(), key_byte_received.cend(), [](uint8_t value) { return value != 0; }))
        {
            d_osnma_data.d_mack_message.key = std::move(key_bytes);
        }
    else
        {
            std::vector<uint8_t> merged_key;
            const uint32_t mack_key_gst = gst_with_offset(d_osnma_data.d_mack_message.WN, d_osnma_data.d_mack_message.TOW, 0);
            if (merge_partial_tesla_key(mack_key_gst, key_bytes, key_byte_received, merged_key))
                {
                    d_osnma_data.d_mack_message.key = std::move(merged_key);
                    LOG(INFO) << "Galileo OSNMA: TESLA key reconstructed from partial MACK pages.";
                }
            else
                {
                    LOG(INFO) << "Galileo OSNMA: TESLA key is incomplete at page level. Waiting for redundant pages.";
                }
        }
    // rest are padding bits, used for anything ?
    return true;
}


/**
 * @brief Verifies the tags transmitted in the past.
 *
 * \details This function is responsible for processing the MACK message received (480 bits) at time SF(i).
 * It stores the last 10 MACK messages and the last 11 OSNMA_NavData messages.
 * Then attempts to verify the Tesla Key by computing the number of hashes of distance between the key-to-verify and the
 * Kroot and iteratively hashing the result, until the required number of hashes is achieved.
 * The result is then compared with the Kroot. If the two values match, the Tesla key is verified.
 *  It also performs MACSEQ validation and compares the ADKD of Mack tags with MACLT defined ADKDs.
 *  Finally, it verifies the tags.
 * \pre Kroot or already a TESLA key shall be available. Depending on the ADKD of the tag, OSNMA_NavData of SF(i-2)...SF(i-11)
 * \post Number of tags bits verified for each ADKD. MACSEQ verification success
 * @param osnma_msg A reference to OSNMA_msg containing the MACK message to be processed.
 */
void osnma_msg_receiver::process_mack_message()
{
    if (!d_kroot_verified && !d_tesla_key_verified)
        {
            LOG(WARNING) << "Galileo OSNMA: MACK cannot be processed, "
                         << "no Kroot nor TESLA key available.";
            return;  // early return, cannot proceed further without one of the two verified. this equals to having Kroot but no TESLa key yet.
        }
    const uint32_t mack_key_gst = gst_with_offset(d_osnma_data.d_mack_message.WN, d_osnma_data.d_mack_message.TOW, 0);
    // verify tesla key and add it to the container of verified keys if successful
    if (d_tesla_keys.find(mack_key_gst) == d_tesla_keys.end())  // check if already available => no need to verify
        {
            if (!d_osnma_data.d_mack_message.key.empty())
                {
                    verify_tesla_key(d_osnma_data.d_mack_message.key, mack_key_gst);
                }
            else
                {
                    LOG(INFO) << "Galileo OSNMA: TESLA key not available in this MACK yet.";
                }
        }

    // MACSEQ - verify current macks, then add current retrieved mack to the end.
    auto mack = d_macks_awaiting_MACSEQ_verification.begin();
    while (mack != d_macks_awaiting_MACSEQ_verification.end())
        {
            if (has_tesla_key(mack->WN, mack->TOW, 30))
                {
                    if (mack->header.tag0_valid)
                        {
                            Tag tag0(*mack);
                            d_tags_awaiting_verify.insert(std::pair<uint32_t, Tag>(mack->TOW, tag0));
                            LOG(INFO) << "Galileo OSNMA: Add Tag0_Id="
                                      << tag0.tag_id
                                      << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                                      << tag0.received_tag << std::dec
                                      << ", TOW="
                                      << tag0.TOW
                                      << ", ADKD="
                                      << static_cast<unsigned>(tag0.ADKD)
                                      << ", PRNa="
                                      << static_cast<unsigned>(tag0.PRNa)
                                      << ", PRNd="
                                      << static_cast<unsigned>(tag0.PRN_d);
                        }
                    else
                        {
                            LOG(INFO) << "Galileo OSNMA: Tag0 unavailable for MACK TOW="
                                      << mack->TOW
                                      << ". Body tags will still be processed when allowed by MACLT/MACSEQ.";
                        }
                    std::vector<MACK_tag_and_info> macseq_verified_tags = verify_macseq(*mack);
                    for (auto& tag_and_info : macseq_verified_tags)
                        {
                            // add tags of current mack to the verification queue
                            Tag t(tag_and_info, mack->TOW, mack->WN, mack->PRNa, tag_and_info.counter, mack->nmas, mack->allowed_adkds);
                            d_tags_awaiting_verify.insert(std::pair<uint32_t, Tag>(mack->TOW, t));
                            LOG(INFO) << "Galileo OSNMA: Add Tag_Id="
                                      << t.tag_id
                                      << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                                      << t.received_tag << std::dec
                                      << ", TOW="
                                      << t.TOW
                                      << ", ADKD="
                                      << static_cast<unsigned>(t.ADKD)
                                      << ", PRNa="
                                      << static_cast<unsigned>(t.PRNa)
                                      << ", PRNd="
                                      << static_cast<unsigned>(t.PRN_d);
                        }
                    LOG(INFO) << "Galileo OSNMA: d_tags_awaiting_verify :: size: " << d_tags_awaiting_verify.size();
                    mack = d_macks_awaiting_MACSEQ_verification.erase(mack);
                }
            else
                {
                    const uint32_t latest_useful_key_gst = gst_with_offset(mack->WN, mack->TOW, 330);
                    if (gst_delta_seconds(d_GST_Sf, latest_useful_key_gst) > 0)
                        {
                            LOG(INFO) << "Galileo OSNMA: MACSEQ verification :: DELETE pending MACK due to expired TESLA disclosure window. TOW="
                                      << mack->TOW
                                      << ", WN="
                                      << mack->WN;
                            mack = d_macks_awaiting_MACSEQ_verification.erase(mack);
                        }
                    else
                        {
                            // key not yet available - keep in container until the disclosure window expires
                            ++mack;
                        }
                }
        }
    // add current received MACK to the container to be verified in the next iteration (on this one no key available)
    d_macks_awaiting_MACSEQ_verification.push_back(d_osnma_data.d_mack_message);

    try_verify_pending_tags(true);
}


void osnma_msg_receiver::try_verify_pending_tags(bool log_unavailable_tags)
{
    if (d_tags_awaiting_verify.empty())
        {
            return;
        }

    bool authenticated_dont_use_status = false;
    for (auto& it : d_tags_awaiting_verify)
        {
            bool ret;
            if (tag_is_allowed_by_time_constraint(it.second) &&
                tag_has_key_available(it.second) &&
                d_nav_data_manager->have_nav_data(it.second))  // tag_has_nav_data_available(it.second))
                {
                    ret = verify_tag(it.second);
                    if (ret)
                        {
                            d_count_successful_tags++;
                            if ((it.second.nmas & 0b00000011) == 0b00000011)
                                {
                                    it.second.status = Tag::AUTHENTICATED_DONT_USE;
                                    authenticated_dont_use_status = true;
                                    LOG(WARNING) << "Galileo OSNMA: Tag verification :: SUCCESS with authenticated NMA Status 'Don't Use' for tag Id="
                                                 << it.second.tag_id
                                                 << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                                                 << it.second.received_tag << std::dec
                                                 << ", TOW="
                                                 << it.second.TOW
                                                 << ", ADKD="
                                                 << static_cast<unsigned>(it.second.ADKD)
                                                 << ", PRNa="
                                                 << static_cast<unsigned>(it.second.PRNa)
                                                 << ", PRNd="
                                                 << static_cast<unsigned>(it.second.PRN_d);
                                }
                            else
                                {
                                    it.second.status = Tag::SUCCESS;
                                    LOG(INFO) << "Galileo OSNMA: Tag verification :: SUCCESS for tag Id="
                                              << it.second.tag_id
                                              << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                                              << it.second.received_tag << std::dec
                                              << ", TOW="
                                              << it.second.TOW
                                              << ", ADKD="
                                              << static_cast<unsigned>(it.second.ADKD)
                                              << ", PRNa="
                                              << static_cast<unsigned>(it.second.PRNa)
                                              << ", PRNd="
                                              << static_cast<unsigned>(it.second.PRN_d);
                                    std::cout << "Galileo OSNMA: Tag verification :: SUCCESS for tag ADKD="
                                              << static_cast<unsigned>(it.second.ADKD)
                                              << ", PRNa="
                                              << static_cast<unsigned>(it.second.PRNa)
                                              << ", PRNd="
                                              << static_cast<unsigned>(it.second.PRN_d) << std::endl;
                                }
                        }
                    else
                        {
                            d_count_failed_tags++;
                            it.second.status = Tag::FAIL;
                            LOG(WARNING) << "Galileo OSNMA: Tag verification :: FAILURE for tag Id="
                                         << it.second.tag_id
                                         << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                                         << it.second.received_tag << std::dec
                                         << ", TOW="
                                         << it.second.TOW
                                         << ", ADKD="
                                         << static_cast<unsigned>(it.second.ADKD)
                                         << ", PRNa="
                                         << static_cast<unsigned>(it.second.PRNa)
                                         << ", PRNd="
                                         << static_cast<unsigned>(it.second.PRN_d);
                            std::cerr << "Galileo OSNMA: Tag verification :: FAILURE for tag ADKD="
                                      << static_cast<unsigned>(it.second.ADKD)
                                      << ", PRNa="
                                      << static_cast<unsigned>(it.second.PRNa)
                                      << ", PRNd="
                                      << static_cast<unsigned>(it.second.PRN_d) << std::endl;
                        }
                }
            else if (log_unavailable_tags && it.second.TOW > d_osnma_data.d_nav_data.get_tow_sf0())
                {
                    it.second.skipped++;
                    LOG(WARNING) << "Galileo OSNMA: Tag verification :: SKIPPED (x" << it.second.skipped << ") for Tag_Id="
                                 << it.second.tag_id
                                 << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                                 << it.second.received_tag << std::dec
                                 << ", TOW="
                                 << it.second.TOW
                                 << ", ADKD="
                                 << static_cast<unsigned>(it.second.ADKD)
                                 << ", PRNa="
                                 << static_cast<unsigned>(it.second.PRNa)
                                 << ", PRNd="
                                 << static_cast<unsigned>(it.second.PRN_d)
                                 << ". Key available (" << tag_has_key_available(it.second) << "),  navData (" << tag_has_nav_data_available(it.second) << "). ";
                }
        }
    if (authenticated_dont_use_status)
        {
            handle_authenticated_dont_use_status();
            return;
        }
    uint8_t tag_size = 0;
    const auto it = OSNMA_TABLE_11.find(d_osnma_data.d_dsm_kroot_message.ts);
    if (it != OSNMA_TABLE_11.cend())
        {
            tag_size = it->second;
        }
    d_nav_data_manager->update_nav_data(d_tags_awaiting_verify, tag_size);
    auto data_to_send = d_nav_data_manager->get_verified_data();
    d_nav_data_manager->log_status();
    send_data_to_pvt(data_to_send);

    remove_verified_tags();

    control_tags_awaiting_verify_size();  // remove the oldest tags if size is too big.
}


/**
 * @brief Verify received DSM-PKR message
 *
 * \details This method provides the functionality to verify the DSM-PKR message. The verification includes generating the base leaf
 * and intermediate leafs, and comparing the computed merkle root leaf with the received one.
 * \pre DSM_PKR_message correctly filled in, especially the 1024-bit intermediate tree nodes fields
 * \returns true if computed merkle root matches received one, false otherwise
 */
bool osnma_msg_receiver::verify_dsm_pkr(const DSM_PKR_message& message) const
{
    Osnma_Merkle_Tree_Material active_merkle_tree;
    active_merkle_tree.valid = !d_crypto->get_merkle_root().empty();
    active_merkle_tree.root = d_crypto->get_merkle_root();
    active_merkle_tree.hash_function = d_crypto->get_merkle_tree_hash_function();
    active_merkle_tree.source = "active-crypto";
    return verify_dsm_pkr(message, active_merkle_tree);
}


bool osnma_msg_receiver::verify_dsm_pkr(const DSM_PKR_message& message, const Osnma_Merkle_Tree_Material& merkle_tree) const
{
    const auto msg_id = static_cast<int>(message.mid);
    if (!merkle_tree.valid)
        {
            LOG(WARNING) << "Galileo OSNMA: DSM-PKR verification for Message ID " << msg_id
                         << " :: FAILURE. No valid Merkle Tree material available.";
            return false;
        }

    const auto base_leaf = get_merkle_tree_leaves(message);                                                // m_i
    const auto computed_merkle_root = compute_merkle_root(message, base_leaf, merkle_tree.hash_function);  // x_4_0
    LOG(INFO) << "Galileo OSNMA: DSM-PKR verification :: leaf provided for Message ID " << msg_id;

    if (computed_merkle_root == merkle_tree.root)
        {
            LOG(INFO) << "Galileo OSNMA: DSM-PKR verification for Message ID " << msg_id << " :: SUCCESS. PKID=" << static_cast<unsigned>(message.npktid);
            std::cout << "Galileo OSNMA: DSM-PKR verification for Message ID " << msg_id << " :: SUCCESS. PKID=" << static_cast<unsigned>(message.npktid) << std::endl;
            return true;
        }
    else
        {
            LOG(WARNING) << "Galileo OSNMA: DSM-PKR verification for Message ID " << msg_id << " :: FAILURE.";
            std::cerr << "Galileo OSNMA: DSM-PKR verification for Message ID " << msg_id << " :: FAILURE." << std::endl;
            return false;
        }
}


std::vector<uint8_t> osnma_msg_receiver::compute_merkle_root(const DSM_PKR_message& dsm_pkr_message, const std::vector<uint8_t>& m_i) const
{
    return compute_merkle_root(dsm_pkr_message, m_i, d_crypto->get_merkle_tree_hash_function());
}


std::vector<uint8_t> osnma_msg_receiver::compute_merkle_root(const DSM_PKR_message& dsm_pkr_message, const std::vector<uint8_t>& m_i, const std::string& hash_function) const
{
    const auto compute_tree_hash = [this, &hash_function](const std::vector<uint8_t>& input) {
        if (hash_function == "SHA-256")
            {
                return d_crypto->compute_SHA_256(input);
            }
        if (hash_function == "SHA3-256")
            {
                return d_crypto->compute_SHA3_256(input);
            }
        LOG(WARNING) << "Galileo OSNMA: DSM-PKR verification :: Unsupported Merkle tree hash function " << hash_function;
        return std::vector<uint8_t>();
    };

    std::vector<uint8_t> x_next;
    std::vector<uint8_t> x_current = compute_tree_hash(m_i);
    if (x_current.empty())
        {
            return {};
        }
    for (size_t i = 0; i < 4; i++)
        {
            x_next.clear();
            x_next.reserve(64);  // we always append 32 + 32 bytes

            bool leaf_is_on_right = ((dsm_pkr_message.mid >> i) & 1) != 0;
            const auto* itn_start = dsm_pkr_message.itn.begin() + (32 * i);
            const auto* itn_end = itn_start + 32;

            if (leaf_is_on_right)
                {
                    // Leaf is on the right -> first the itn, then the leaf
                    x_next.insert(x_next.end(), itn_start, itn_end);
                    x_next.insert(x_next.end(), x_current.begin(), x_current.end());
                }
            else
                {
                    // Leaf is on the left -> first the leaf, then the itn
                    x_next.insert(x_next.end(), x_current.begin(), x_current.end());
                    x_next.insert(x_next.end(), itn_start, itn_end);
                }

            // Compute the next node
            x_current = compute_tree_hash(x_next);
            if (x_current.empty())
                {
                    return {};
                }
        }
    return x_current;
}


/**
 * @brief Get the Merkle tree base leave from a DSM_PKR_message.
 *
 * @param dsm_pkr_message The DSM_PKR_message object from which to retrieve the Merkle tree leave.
 * @return std::vector<uint8_t> The Merkle tree base leave from the DSM_PKR_message object.
 */
std::vector<uint8_t> osnma_msg_receiver::get_merkle_tree_leaves(const DSM_PKR_message& dsm_pkr_message) const
{
    // build base leaf m_i according to OSNMA SIS ICD v1.1, section 6.2 DSM-PKR Verification
    constexpr uint8_t MASK_4BITS = 0x0F;
    std::vector<uint8_t> m_i;
    m_i.reserve(1 + dsm_pkr_message.npk.size());
    m_i.emplace_back(static_cast<uint8_t>(((dsm_pkr_message.npkt & MASK_4BITS) << 4) | (dsm_pkr_message.npktid & MASK_4BITS)));
    m_i.insert(m_i.end(), dsm_pkr_message.npk.cbegin(), dsm_pkr_message.npk.cend());
    return m_i;
}


bool osnma_msg_receiver::verify_tag(Tag& tag) const
{
    // Debug
    //    LOG(INFO) << "Galileo OSNMA: Tag verification :: Start for tag Id= "
    //              << tag.tag_id
    //              << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
    //              << tag.received_tag << std::dec;
    // build message
    std::vector<uint8_t> m = build_message(tag);

    std::vector<uint8_t> mac;
    std::vector<uint8_t> applicable_key;
    if (tag.ADKD == 0 || tag.ADKD == 4)
        {
            if (!get_tesla_key_for_adkd(tag.WN, tag.TOW, 30, tag.ADKD, applicable_key))
                {
                    return false;
                }
            // LOG(INFO) << "|---> Galileo OSNMA :: applicable key: 0x" << d_helper->convert_to_hex_string(applicable_key) << "TOW="<<static_cast<int>(tag.TOW + 30);
        }
    else  // ADKD 12
        {
            if (!get_tesla_key_for_adkd(tag.WN, tag.TOW, 330, tag.ADKD, applicable_key))
                {
                    return false;
                }
            // LOG(INFO) << "|---> Galileo OSNMA :: applicable key: 0x" << d_helper->convert_to_hex_string(applicable_key) << "TOW="<<static_cast<int>(tag.TOW + 330);
        }

    if (d_osnma_data.d_dsm_kroot_message.mf == 0)  // C: HMAC-SHA-256
        {
            mac = d_crypto->compute_HMAC_SHA_256(applicable_key, m);
        }
    else if (d_osnma_data.d_dsm_kroot_message.mf == 1)  // C: CMAC-AES
        {
            mac = d_crypto->compute_CMAC_AES(applicable_key, m);
        }

    // truncate the computed mac: trunc(l_t, mac(K,m)) Eq. 23 ICD
    uint8_t lt_bits = 0;
    const auto it2 = OSNMA_TABLE_11.find(d_osnma_data.d_dsm_kroot_message.ts);
    if (it2 != OSNMA_TABLE_11.cend())
        {
            lt_bits = it2->second;
        }
    if (lt_bits < 16)
        {
            return false;
        }
    const size_t mac_bytes_needed = (static_cast<size_t>(lt_bits) + 7) / 8;
    if (mac.size() < mac_bytes_needed)
        {
            return false;
        }
    uint64_t computed_mac = static_cast<uint64_t>(mac[0]) << (lt_bits - 8);
    computed_mac += (static_cast<uint64_t>(mac[1]) << (lt_bits - 16));
    if (lt_bits == 20)
        {
            computed_mac += (static_cast<uint64_t>(mac[2] & 0xF0) >> 4);
        }
    else if (lt_bits == 24)
        {
            computed_mac += static_cast<uint64_t>(mac[2]);
        }
    else if (lt_bits == 28)
        {
            computed_mac += (static_cast<uint64_t>(mac[2]) << 4);
            computed_mac += (static_cast<uint64_t>(mac[3] & 0xF0) >> 4);
        }
    else if (lt_bits == 32)
        {
            computed_mac += (static_cast<uint64_t>(mac[2]) << 8);
            computed_mac += static_cast<uint64_t>(mac[3]);
        }
    else if (lt_bits == 40)
        {
            computed_mac += (static_cast<uint64_t>(mac[2]) << 16);
            computed_mac += (static_cast<uint64_t>(mac[3]) << 8);
            computed_mac += static_cast<uint64_t>(mac[4]);
        }

    tag.computed_tag = computed_mac;  // update with computed value
    // Compare computed tag with received one truncated
    if (tag.received_tag == computed_mac)
        {
            return true;
        }
    return false;
}


/**
 * \brief generates the message for computing the tag
 * \remarks It also sets some parameters to the Tag object, based on the verification process.
 *
 * \param tag The tag containing the information to be included in the message.
 *
 * \return The built OSNMA message as a vector of uint8_t.
 */
std::vector<uint8_t> osnma_msg_receiver::build_message(Tag& tag) const
{
    std::vector<uint8_t> m;
    if (tag.CTR != 1)
        {
            m.push_back(static_cast<uint8_t>(tag.PRN_d));
        }
    m.push_back(static_cast<uint8_t>(tag.PRNa));
    uint32_t GST = d_helper->compute_gst(tag.WN, tag.TOW);
    std::vector<uint8_t> GST_uint8 = d_helper->gst_to_uint8(GST);
    m.insert(m.end(), GST_uint8.begin(), GST_uint8.end());
    m.push_back(tag.CTR);
    // Extract only the two NMAS bits that were broadcast with this tag.
    uint8_t two_bits_nmas = tag.nmas & 0b00000011;
    two_bits_nmas = two_bits_nmas << 6;
    m.push_back(two_bits_nmas);

    // Add applicable NavData bits to message
    std::string applicable_nav_data;
    if (tag.cop == 0)
        {
            // SIS ICD 1.1 section 6.7: dummy tags keep PRN/GST/CTR/NMAS,
            // replacing only the navdata field with zero bits.
            if (tag.ADKD == 0 || tag.ADKD == 12)
                {
                    applicable_nav_data = std::string(549, '0');
                }
            else if (tag.ADKD == 4)
                {
                    applicable_nav_data = std::string(141, '0');
                }
        }
    else
        {
            applicable_nav_data = d_nav_data_manager->get_navigation_data(tag);
        }
    std::vector<uint8_t> applicable_nav_data_bytes = d_helper->bytes(applicable_nav_data);
    tag.nav_data = std::move(applicable_nav_data);  // update tag with applicable data

    // Convert and add OSNMA_NavData bytes into the message, taking care of that NMAS has only 2 bits
    for (uint8_t byte : applicable_nav_data_bytes)
        {
            m.back() |= (byte >> 2);  // First take the 6 MSB bits of byte and add to m
            m.push_back(byte << 6);   // Then take the last 2 bits of byte, shift them to MSB position and insert the new element into m
        }
    if (m.back() == 0)
        {
            m.pop_back();  // Remove the last element if its value is 0 (only padding was added)
        }
    else
        {
            // Pad with zeros if the last element wasn't full
            for (int bits = 2; bits < 8; bits += 2)
                {
                    // Check if the last element in the vector has 2 '00' bits in its LSB position
                    if ((m.back() & 0b00000011) == 0)
                        {
                            m.back() <<= 2;  // Shift the existing bits  to make room for new 2 bits
                        }
                    else
                        {
                            break;  // If it does not have 2 '00' bits in its LSB position, then the padding is complete
                        }
                }
        }
    return m;
}


bool osnma_msg_receiver::verify_tesla_key(const std::vector<uint8_t>& key, uint32_t key_gst)
{
    uint32_t num_of_hashes_needed{};
    uint32_t GST_SFi = gst_with_offset(d_helper->get_WN(d_GST_Sf), d_helper->get_TOW(d_GST_Sf), -30);  // GST of target key is to be used.
    std::vector<uint8_t> hash;
    const uint8_t lk_bytes = d_dsm_reader->get_lk_bits(d_osnma_data.d_dsm_kroot_message.ks) / 8;
    std::vector<uint8_t> validated_key;
    if (lk_bytes == 0 || key.empty())
        {
            LOG(WARNING) << "Galileo OSNMA: TESLA key verification :: FAILED. Invalid key length";
            return false;
        }
    if (d_tesla_key_verified)
        {  // have to go up to last verified key
            if (d_tesla_keys.empty())
                {
                    LOG(WARNING) << "Galileo OSNMA: TESLA key verification :: FAILED. No verified TESLA key available";
                    return false;
                }
            const int64_t gst_delta = gst_delta_seconds(d_GST_Sf, d_last_verified_key_GST);
            if (gst_delta <= 0)
                {
                    LOG(WARNING) << "Galileo OSNMA: TESLA key verification :: FAILED. Received a stale TESLA key";
                    return false;
                }
            validated_key = d_tesla_keys.rbegin()->second.key;
            if (!compute_tesla_hash_count(gst_delta, false, num_of_hashes_needed))  // Eq. 19 ICD modified
                {
                    LOG(WARNING) << "Galileo OSNMA: TESLA key verification :: FAILED. Invalid TESLA hash count";
                    return false;
                }
            LOG(INFO) << "Galileo OSNMA: TESLA verification (" << num_of_hashes_needed << " hashes) need to be performed up to closest verified TESLA key";

            hash = hash_chain(num_of_hashes_needed, key, GST_SFi, lk_bytes);
        }
    else
        {  // have to go until Kroot
            const int64_t gst_delta = gst_delta_seconds(d_GST_Sf, d_GST_0);
            if (gst_delta < 0)
                {
                    LOG(WARNING) << "Galileo OSNMA: TESLA key verification :: FAILED. TESLA key is before KROOT applicability";
                    return false;
                }
            validated_key = d_osnma_data.d_dsm_kroot_message.kroot;
            if (!compute_tesla_hash_count(gst_delta, true, num_of_hashes_needed))  // Eq. 19 ICD
                {
                    LOG(WARNING) << "Galileo OSNMA: TESLA key verification :: FAILED. Invalid TESLA hash count";
                    return false;
                }
            LOG(INFO) << "Galileo OSNMA: TESLA verification (" << num_of_hashes_needed << " hashes) need to be performed up to Kroot";

            hash = hash_chain(num_of_hashes_needed, key, GST_SFi, lk_bytes);
        }
    if (hash.size() < key.size())
        {
            LOG(WARNING) << "Galileo OSNMA: TESLA key verification :: FAILED. Invalid hash length";
            return false;
        }
    // truncate hash
    std::vector<uint8_t> computed_key;
    computed_key.reserve(key.size());
    for (size_t i = 0; i < key.size(); i++)
        {
            computed_key.push_back(hash[i]);
        }
    const bool current_key_verified = computed_key == validated_key && num_of_hashes_needed > 0;
    if (current_key_verified)
        {
            LOG(INFO) << "Galileo OSNMA: TESLA key verification :: SUCCESS!";
            std::cout << "Galileo OSNMA: TESLA key verification :: SUCCESS!" << std::endl;
            d_tesla_keys.insert(std::pair<uint32_t, VerifiedTeslaKey>(key_gst, VerifiedTeslaKey(key, d_osnma_data.d_mack_message.allowed_adkds)));
            d_tesla_key_verified = true;
            d_last_verified_key_GST = d_GST_Sf;
            prune_old_tesla_keys(d_last_verified_key_GST);
        }
    else if (num_of_hashes_needed > 0)
        {
            LOG(WARNING) << "Galileo OSNMA: TESLA key verification :: FAILED";
            std::cerr << "Galileo OSNMA: TESLA key verification :: FAILED" << std::endl;
            if (d_kroot_loaded_from_cache && !d_tesla_key_verified)
                {
                    LOG(WARNING) << "Galileo OSNMA: hot-start DSM-KROOT cache did not verify the received TESLA key. Retrieving DSM-KROOT from SIS.";
                    invalidate_verified_kroot();
                    d_nav_data_manager->reset_tag_accumulations();
                    reset_tesla_chain_state();
                }
        }
    return current_key_verified;
}


/**
 * @brief Removes the tags that have been verified from the multimap d_tags_awaiting_verify.
 *
 * This function iterates through the multimap d_tags_awaiting_verify, and removes the tags that have a status of SUCCESS or FAIL.
 * \remarks it also prints the current unverified tags
 */
void osnma_msg_receiver::remove_verified_tags()
{
    for (auto it = d_tags_awaiting_verify.begin(); it != d_tags_awaiting_verify.end();)
        {
            if (it->second.status == Tag::SUCCESS || it->second.status == Tag::FAIL || it->second.status == Tag::AUTHENTICATED_DONT_USE)
                {
                    LOG(INFO) << "Galileo OSNMA: Tag verification :: DELETE tag Id="
                              << it->second.tag_id
                              << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                              << it->second.received_tag << std::dec
                              << ", TOW="
                              << it->second.TOW
                              << ", ADKD="
                              << static_cast<unsigned>(it->second.ADKD)
                              << ", PRNa="
                              << static_cast<unsigned>(it->second.PRNa)
                              << ", PRNd="
                              << static_cast<unsigned>(it->second.PRN_d)
                              << ", status="
                              << d_helper->verification_status_str(it->second.status);
                    it = d_tags_awaiting_verify.erase(it);
                }
            else if (gst_delta_seconds(d_last_verified_key_GST, gst_with_offset(it->second.WN, it->second.TOW, 330)) > 0)
                {
                    LOG(INFO) << "Galileo OSNMA: Tag verification :: DELETE tag Id="
                              << it->second.tag_id
                              << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                              << it->second.received_tag << std::dec
                              << ", TOW="
                              << it->second.TOW
                              << ", ADKD="
                              << static_cast<unsigned>(it->second.ADKD)
                              << ", PRNa="
                              << static_cast<unsigned>(it->second.PRNa)
                              << ", PRNd="
                              << static_cast<unsigned>(it->second.PRN_d)
                              << ", status="
                              << d_helper->verification_status_str(it->second.status)
                              << ". Verification window expired.";
                    it = d_tags_awaiting_verify.erase(it);
                }
            else
                {
                    ++it;
                }
        }
    LOG(INFO) << "Galileo OSNMA: d_tags_awaiting_verify :: size: " << d_tags_awaiting_verify.size();
    for (const auto& it : d_tags_awaiting_verify)
        {
            LOG(INFO) << "Galileo OSNMA: Tag verification :: status tag Id="
                      << it.second.tag_id
                      << ", value=0x" << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                      << it.second.received_tag << std::dec
                      << ", TOW="
                      << it.second.TOW
                      << ", ADKD="
                      << static_cast<unsigned>(it.second.ADKD)
                      << ", PRNa="
                      << static_cast<unsigned>(it.second.PRNa)
                      << ", PRNd="
                      << static_cast<unsigned>(it.second.PRN_d)
                      << ", status="
                      << d_helper->verification_status_str(it.second.status);
        }
}


/**
 * @brief Control the size of the tags awaiting verification multimap.
 *
 * This function checks the size of the multimap `d_tags_awaiting_verify` and removes
 * elements from the beginning until the size is no longer greater than 60.
 * The purpose is to limit the size of the multimap and prevent it from consuming
 * excessive memory.
 */
void osnma_msg_receiver::control_tags_awaiting_verify_size()
{
    while (d_tags_awaiting_verify.size() > 500)
        {
            auto it = d_tags_awaiting_verify.begin();
            LOG(INFO) << "Galileo OSNMA: Tag verification :: DELETED tag due to exceeding buffer size. "
                      << "Tag_Id=" << it->second.tag_id
                      << ", TOW=" << it->first
                      << ", ADKD=" << static_cast<unsigned>(it->second.ADKD)
                      << ", from satellite " << it->second.PRNa;
            d_tags_awaiting_verify.erase(it);
        }
}


void osnma_msg_receiver::prune_old_tesla_keys(uint32_t reference_gst)
{
    if (reference_gst == 0)
        {
            return;
        }
    for (auto it = d_tesla_keys.begin(); it != d_tesla_keys.end();)
        {
            if (it->first != d_last_verified_key_GST &&
                gst_delta_seconds(reference_gst, it->first) > tesla_key_retention_s)
                {
                    it = d_tesla_keys.erase(it);
                }
            else
                {
                    ++it;
                }
        }
}


bool osnma_msg_receiver::tag_has_nav_data_available(const Tag& t) const
{
    return d_nav_data_manager->have_nav_data(t);
}


bool osnma_msg_receiver::tag_has_key_available(const Tag& t) const
{
    // check adkd of tag
    // if adkd = 0 or 4 => look for the key 30 seconds after the tag GST
    // if adkd = 12 => look for the key 330 seconds after the tag GST
    // return true if available, otherwise false

    if (t.ADKD == 0 || t.ADKD == 4)
        {
            if (has_tesla_key_for_adkd(t.WN, t.TOW, 30, t.ADKD))
                {
                    // LOG(INFO) << "Galileo OSNMA: hasKey = true " << std::endl;
                    return true;
                }
        }
    else if (t.ADKD == 12)
        {
            if (has_tesla_key_for_adkd(t.WN, t.TOW, 330, t.ADKD))
                {
                    // LOG(INFO) << "Galileo OSNMA: hasKey = true " << std::endl;
                    return true;
                }
        }
    // LOG(INFO) << "Galileo OSNMA: hasKey = false ";
    return false;
}


bool osnma_msg_receiver::tag_is_allowed_by_time_constraint(const Tag& tag) const
{
    return adkd_is_allowed(tag.allowed_adkds, tag.ADKD);
}


bool osnma_msg_receiver::kroot_parameters_are_supported(const DSM_KROOT_message& kroot) const
{
    const bool supported_hash_function = kroot.hf == 0 || kroot.hf == 2;
    const bool supported_mac_function = kroot.mf == 0 || kroot.mf == 1;
    const auto tag_size = OSNMA_TABLE_11.find(kroot.ts);
    const auto maclt = OSNMA_TABLE_16.find(kroot.maclt);
    const uint16_t lk_bits = d_dsm_reader->get_lk_bits(kroot.ks);
    if (!supported_hash_function ||
        !supported_mac_function ||
        tag_size == OSNMA_TABLE_11.cend() ||
        tag_size->second < 16 ||
        maclt == OSNMA_TABLE_16.cend() ||
        lk_bits == 0 ||
        lk_bits >= 480)
        {
            return false;
        }

    const auto nt = static_cast<size_t>(std::floor((480.0 - static_cast<double>(lk_bits)) / (static_cast<double>(tag_size->second) + 16.0)));
    return nt > 0 &&
           maclt->second.sequence1.size() == nt &&
           maclt->second.sequence2.size() == nt;
}


bool osnma_msg_receiver::merkle_tree_differs_from_renewal_start() const
{
    return d_crypto->get_merkle_root() != d_merkle_root_at_renewal_start ||
           d_crypto->get_merkle_tree_hash_function() != d_merkle_hash_function_at_renewal_start;
}


bool osnma_msg_receiver::merkle_tree_differs_from_renewal_start(const Osnma_Merkle_Tree_Material& merkle_tree) const
{
    return merkle_tree.root != d_merkle_root_at_renewal_start ||
           merkle_tree.hash_function != d_merkle_hash_function_at_renewal_start;
}


bool osnma_msg_receiver::ensure_new_merkle_tree_available()
{
    if (!d_flag_merkle_tree_renewal || d_new_merkle_tree_loaded)
        {
            return true;
        }

    const auto candidate_merkle_tree = d_material_manager->candidate_merkle_tree();
    if (candidate_merkle_tree.valid)
        {
            d_new_merkle_tree_loaded = merkle_tree_differs_from_renewal_start(candidate_merkle_tree);
            return d_new_merkle_tree_loaded;
        }

    if (!d_merkle_file_path.empty())
        {
            std::ifstream merkle_file(d_merkle_file_path);
            if (merkle_file.good())
                {
                    auto merkle_tree = d_material_manager->load_configured_merkle_tree(d_merkle_file_path);
                    if (merkle_tree.valid)
                        {
                            d_material_manager->set_candidate_merkle_tree(merkle_tree);
                            d_new_merkle_tree_loaded = merkle_tree_differs_from_renewal_start(merkle_tree);
                        }
                }
        }

    return d_new_merkle_tree_loaded;
}


std::vector<uint8_t> osnma_msg_receiver::hash_chain(uint32_t num_of_hashes_needed, const std::vector<uint8_t>& key, uint32_t GST_SFi, const uint8_t lk_bytes) const
{
    std::vector<uint8_t> K_II = key;
    std::vector<uint8_t> K_I;  // result of the recursive hash operations
    std::vector<uint8_t> msg;
    uint32_t last_hash_gst = GST_SFi;
    // compute the tesla key for current SF (GST_SFi and K_II change in each iteration)
    for (uint32_t i = 1; i <= num_of_hashes_needed; i++)
        {
            last_hash_gst = GST_SFi;
            // build message digest m = (K_I+1 || GST_SFi || alpha)
            msg.reserve(K_II.size() + sizeof(GST_SFi) + sizeof(d_osnma_data.d_dsm_kroot_message.alpha));
            std::copy(K_II.begin(), K_II.end(), std::back_inserter(msg));

            msg.push_back((GST_SFi & 0xFF000000) >> 24);
            msg.push_back((GST_SFi & 0x00FF0000) >> 16);
            msg.push_back((GST_SFi & 0x0000FF00) >> 8);
            msg.push_back(GST_SFi & 0x000000FF);
            // extract alpha
            //            d_osnma_data.d_dsm_kroot_message.alpha = 0xa06221261ad9;
            for (int k = 5; k >= 0; k--)
                {
                    msg.push_back(static_cast<uint8_t>((d_osnma_data.d_dsm_kroot_message.alpha >> (k * 8)) & 0xFF));  // extract first 6 bytes of alpha.
                }
            // compute hash
            std::vector<uint8_t> hash;
            if (d_osnma_data.d_dsm_kroot_message.hf == 0)  // Table 8.
                {
                    hash = d_crypto->compute_SHA_256(msg);
                }
            else if (d_osnma_data.d_dsm_kroot_message.hf == 2)
                {
                    hash = d_crypto->compute_SHA3_256(msg);
                }
            else
                {
                    hash = std::vector<uint8_t>(32);
                }
            // truncate hash
            K_I.reserve(lk_bytes);
            for (int k = 0; k < lk_bytes; k++)
                {
                    K_I.push_back(hash[k]);
                }
            // set parameters for next iteration
            GST_SFi = gst_with_offset(d_helper->get_WN(GST_SFi), d_helper->get_TOW(GST_SFi), -30);  // next SF time is the actual minus 30 seconds
            K_II = K_I;                                                                             // next key is the actual one
            K_I.clear();                                                                            // empty the actual one for a new computation
            msg.clear();
        }

    // check that the final time matches the Kroot time
    bool check;
    if (!d_tesla_key_verified)
        {
            check = last_hash_gst == gst_with_offset(d_helper->get_WN(d_GST_0), d_helper->get_TOW(d_GST_0), -30);
        }
    else
        {
            check = last_hash_gst == d_last_verified_key_GST;
        }
    if (!check)
        {
            LOG(WARNING) << "Galileo OSNMA: TESLA key chain verification error: KROOT time mismatch!";  // ICD. Eq. 18
            std::cerr << "Galileo OSNMA: TESLA key chain verification error: KROOT time mismatch!" << std::endl;
        }
    else
        {
            LOG(INFO) << "Galileo OSNMA: TESLA key chain verification: KROOT time matches.";  // ICD. Eq. 18
        }
    return K_II;
}


/**
 * @brief Verifies the MAC sequence of a received MACK message.
 *
 * This function is responsible for verifying the MAC sequence of a received MACK message.
 * It takes a reference to a constant MACK_message object as input and returns a vector containing
 * the tags for which the MACSEQ verification was successful
 *
 * @param mack The MACK message object to verify the MAC sequence for.
 * @return vector MACK_tag_and_info for which the MACSEQ was successful
 */
std::vector<MACK_tag_and_info> osnma_msg_receiver::verify_macseq(const MACK_message& mack)
{
    std::vector<MACK_tag_and_info> verified_tags{};

    // MACSEQ verification
    uint32_t GST_Sfi = gst_with_offset(mack.WN, mack.TOW, 0);  // time of the start of SF containing MACSEQ
    std::vector<uint8_t> applicable_key;
    get_tesla_key(mack.WN, mack.TOW, 30, applicable_key);  // current TESLA key is transmitted in the next subframe
    std::vector<std::string> sq1{};
    std::vector<std::string> sq2{};
    std::vector<std::string> applicable_sequence;
    const auto it = OSNMA_TABLE_16.find(d_osnma_data.d_dsm_kroot_message.maclt);
    if (it != OSNMA_TABLE_16.cend())
        {
            sq1 = it->second.sequence1;
            sq2 = it->second.sequence2;
        }
    else
        {
            LOG(WARNING) << "Galileo OSNMA: MACSEQ verification :: Unsupported MAC look-up table";
            return verified_tags;
        }

    // Assign relevant sequence based on subframe time
    if (mack.TOW % 60 < 30)  // tried GST_Sf and it does not support the data present.
        {
            applicable_sequence = std::move(sq1);
        }
    else if (mack.TOW % 60 >= 30)
        {
            applicable_sequence = std::move(sq2);
        }
    if (applicable_sequence.empty())
        {
            LOG(WARNING) << "Galileo OSNMA: MACSEQ verification :: Empty MAC look-up sequence";
            return verified_tags;
        }
    if (mack.tag_and_info.size() != applicable_sequence.size() - 1)
        {
            LOG(WARNING) << "Galileo OSNMA: Number of retrieved tags does not match MACLT sequence size!";
            d_count_failed_macseq += mack.tag_and_info.size();
            return verified_tags;
        }
    std::vector<uint8_t> flxTags{};
    std::string tempADKD;
    bool missing_flx_tag = false;
    // MACLT verification
    for (size_t i = 0; i < mack.tag_and_info.size(); i++)
        {
            tempADKD = applicable_sequence[i + 1];
            if (!mack.tag_and_info[i].valid)
                {
                    if (tempADKD == "FLX")
                        {
                            missing_flx_tag = true;
                        }
                    LOG(INFO) << "Galileo OSNMA: MACSEQ verification :: Tag with CTR="
                              << static_cast<uint32_t>(mack.tag_and_info[i].counter)
                              << " is incomplete at page level. Skipping it.";
                }
            else if (tempADKD == "FLX")
                {
                    flxTags.push_back(i);  // C: just need to save the index in the sequence
                }
            else if (fixed_maclt_slot_matches(mack.tag_and_info[i].tag_info, mack.PRNa, applicable_sequence[i + 1]))
                {
                    // fill index of tags failed
                    LOG(INFO) << "Galileo OSNMA: MACSEQ verification :: SUCCESS :: fixed MACLT slot match for Tag=0x"
                              << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                              << mack.tag_and_info[i].tag << std::dec;
                    verified_tags.push_back(mack.tag_and_info[i]);
                }
            else
                {
                    // discard tag
                    LOG(WARNING) << "Galileo OSNMA: MACSEQ verification :: FAILURE :: fixed MACLT slot mismatch for Tag=0x"
                                 << std::setfill('0') << std::setw(10) << std::hex << std::uppercase
                                 << mack.tag_and_info[i].tag << std::dec;
                    d_count_failed_macseq++;
                }
        }

    if (missing_flx_tag)
        {
            LOG(INFO) << "Galileo OSNMA: MACSEQ verification :: FLX tag incomplete at page level. Fixed tags, if any, were kept; FLX tags are skipped.";
            return verified_tags;
        }

    if (flxTags.empty())
        {
            LOG(INFO) << "Galileo OSNMA: MACSEQ verification :: No FLX tags to verify.";
            return verified_tags;
        }
    if (!mack.header.macseq_valid)
        {
            LOG(INFO) << "Galileo OSNMA: MACSEQ verification :: MACSEQ unavailable. Fixed tags were kept; FLX tags are skipped.";
            return verified_tags;
        }
    // Fixed as well as  FLX Tags share first part - Eq. 22 ICD
    std::vector<uint8_t> m(5 + 2 * flxTags.size());  // each flx tag brings two bytes
    m[0] = static_cast<uint8_t>(mack.PRNa);          // PRN_A - SVID of the satellite transmitting the tag
    m[1] = static_cast<uint8_t>((GST_Sfi & 0xFF000000) >> 24);
    m[2] = static_cast<uint8_t>((GST_Sfi & 0x00FF0000) >> 16);
    m[3] = static_cast<uint8_t>((GST_Sfi & 0x0000FF00) >> 8);
    m[4] = static_cast<uint8_t>(GST_Sfi & 0x000000FF);
    // Case tags flexible - Eq. 21 ICD
    for (size_t i = 0; i < flxTags.size(); i++)
        {
            m[2 * i + 5] = mack.tag_and_info[flxTags[i]].tag_info.PRN_d;
            m[2 * i + 6] = mack.tag_and_info[flxTags[i]].tag_info.ADKD << 4 |
                           mack.tag_and_info[flxTags[i]].tag_info.cop;
        }
    // compute mac
    std::vector<uint8_t> mac;
    if (d_osnma_data.d_dsm_kroot_message.mf == 0)  // C: HMAC-SHA-256
        {
            mac = d_crypto->compute_HMAC_SHA_256(applicable_key, m);
        }
    else if (d_osnma_data.d_dsm_kroot_message.mf == 1)  // C: CMAC-AES
        {
            mac = d_crypto->compute_CMAC_AES(applicable_key, m);
        }
    // Truncate the twelve MSBits and compare with received MACSEQ
    uint16_t mac_msb = 0;
    if (!mac.empty())
        {
            mac_msb = (mac[0] << 8) + mac[1];
        }
    uint16_t computed_macseq = (mac_msb & 0xFFF0) >> 4;
    if (computed_macseq == mack.header.macseq)
        {
            LOG(INFO) << "Galileo OSNMA: MACSEQ verification :: SUCCESS :: FLX tags verification OK";
            for (uint8_t flxTag : flxTags)
                {
                    const auto& tag_and_info = mack.tag_and_info[flxTag];
                    if (tag_info_uses_supported_prnd_adkd(tag_and_info.tag_info))
                        {
                            verified_tags.push_back(tag_and_info);
                        }
                    else
                        {
                            LOG(INFO) << "Galileo OSNMA: MACSEQ verification :: DISCARD :: FLX tag with reserved PRN_d="
                                      << static_cast<uint32_t>(tag_and_info.tag_info.PRN_d)
                                      << " or ADKD="
                                      << static_cast<uint32_t>(tag_and_info.tag_info.ADKD);
                        }
                }
            return verified_tags;
        }
    else
        {
            LOG(WARNING) << "Galileo OSNMA: MACSEQ verification :: FAILURE :: FLX tags verification failed";
            d_count_failed_macseq += flxTags.size();
            return verified_tags;
        }
}


void osnma_msg_receiver::send_data_to_pvt(const std::vector<OSNMA_NavData>& data)
{
    if (!data.empty())
        {
            for (const auto& i : data)
                {
                    const auto tmp_obj = std::make_shared<OSNMA_NavData>(i);
                    this->message_port_pub(pmt::mp("OSNMA_to_PVT"), pmt::make_any(tmp_obj));
                }
        }
}


bool osnma_msg_receiver::load_pending_dsm_kroot_cache()
{
    d_pending_dsm_kroot_cache = CachedDsmKroot();

    const auto cached = parse_dsm_kroot();
    if (cached.first.empty())
        {
            return false;
        }

    d_pending_dsm_kroot_cache.dsm = cached.first;
    d_pending_dsm_kroot_cache.nma_header = cached.second;
    d_pending_dsm_kroot_cache.valid = true;

    const auto raw = make_cached_kroot_raw(cached.second, cached.first);
    const std::string computed_raw_sha256 = bytes_to_upper_hex(d_crypto->compute_SHA_256(raw));
    if (!load_dsm_kroot_metadata())
        {
            LOG(INFO) << "Galileo OSNMA: stored DSM-KROOT cache has no valid metadata. Falling back to :: WARM START.";
            std::cout << "Galileo OSNMA: stored DSM-KROOT cache has no valid metadata. Falling back to :: WARM START." << std::endl;
            d_pending_dsm_kroot_cache = CachedDsmKroot();
            return false;
        }

    if (d_pending_dsm_kroot_cache.raw_sha256 != computed_raw_sha256)
        {
            LOG(INFO) << "Galileo OSNMA: stored DSM-KROOT cache metadata fingerprint mismatch. Falling back to :: WARM START.";
            std::cout << "Galileo OSNMA: stored DSM-KROOT cache metadata fingerprint mismatch. Falling back to :: WARM START." << std::endl;
            d_pending_dsm_kroot_cache = CachedDsmKroot();
            return false;
        }

    return true;
}


bool osnma_msg_receiver::load_dsm_kroot_metadata()
{
    const auto values = read_key_value_file(kroot_metadata_path());
    if (values.empty())
        {
            return false;
        }

    auto it = values.find("version");
    if (it == values.cend() || it->second != "1")
        {
            return false;
        }
    it = values.find("kind");
    if (it == values.cend() || it->second != "osnma-kroot-cache")
        {
            return false;
        }

    it = values.find("raw_sha256");
    if (it == values.cend() || it->second.empty())
        {
            return false;
        }
    d_pending_dsm_kroot_cache.raw_sha256 = it->second;

    it = values.find("public_key_sha256");
    if (it == values.cend() || it->second.empty())
        {
            return false;
        }
    d_pending_dsm_kroot_cache.public_key_fingerprint_sha256 = it->second;

    it = values.find("public_key_type");
    if (it == values.cend() || it->second.empty())
        {
            return false;
        }
    d_pending_dsm_kroot_cache.public_key_type = it->second;

    it = values.find("kroot_sha256");
    if (it != values.cend())
        {
            d_pending_dsm_kroot_cache.kroot_sha256 = it->second;
        }

    uint8_t public_key_npkt = 0;
    if (parse_uint8_value(values, "public_key_npkt", public_key_npkt) && public_key_npkt <= 15)
        {
            d_pending_dsm_kroot_cache.public_key_npkt = public_key_npkt;
            d_pending_dsm_kroot_cache.public_key_npkt_valid = true;
        }

    uint8_t metadata_nma_header = 0;
    if (!parse_uint8_value(values, "nma_header", metadata_nma_header) ||
        metadata_nma_header != d_pending_dsm_kroot_cache.nma_header ||
        !parse_uint8_value(values, "pkid", d_pending_dsm_kroot_cache.pkid) ||
        !parse_uint8_value(values, "cidkr", d_pending_dsm_kroot_cache.cidkr) ||
        !parse_uint8_value(values, "nmas", d_pending_dsm_kroot_cache.nmas) ||
        !parse_uint8_value(values, "cpks", d_pending_dsm_kroot_cache.cpks) ||
        !parse_uint8_value(values, "hf", d_pending_dsm_kroot_cache.hf) ||
        !parse_uint8_value(values, "mf", d_pending_dsm_kroot_cache.mf) ||
        !parse_uint8_value(values, "ks", d_pending_dsm_kroot_cache.ks) ||
        !parse_uint8_value(values, "ts", d_pending_dsm_kroot_cache.ts) ||
        !parse_uint8_value(values, "maclt", d_pending_dsm_kroot_cache.maclt) ||
        !parse_uint16_value(values, "wn_k", d_pending_dsm_kroot_cache.wn_k) ||
        !parse_uint8_value(values, "towh_k", d_pending_dsm_kroot_cache.towh_k) ||
        !parse_uint32_value(values, "gst0", d_pending_dsm_kroot_cache.gst0) ||
        !parse_uint32_value(values, "kroot_gst", d_pending_dsm_kroot_cache.kroot_gst) ||
        !parse_uint32_value(values, "signature_verified_at_gst", d_pending_dsm_kroot_cache.signature_verified_at_gst))
        {
            return false;
        }

    d_pending_dsm_kroot_cache.metadata_valid = true;
    return true;
}


void osnma_msg_receiver::evaluate_pending_dsm_kroot_cache(uint32_t current_gst)
{
    if (!d_pending_dsm_kroot_cache.valid)
        {
            return;
        }
    if (current_gst == 0)
        {
            return;
        }
    if (!d_public_key_verified)
        {
            LOG(INFO) << "Galileo OSNMA: stored DSM-KROOT cache cannot be used because no verified Public Key is active. Falling back to :: WARM START.";
            std::cout << "Galileo OSNMA: stored DSM-KROOT cache cannot be used because no verified Public Key is active. Falling back to :: WARM START." << std::endl;
            d_pending_dsm_kroot_cache = CachedDsmKroot();
            return;
        }
    if (!d_pending_dsm_kroot_cache.metadata_valid || d_pending_dsm_kroot_cache.signature_verified_at_gst == 0)
        {
            LOG(INFO) << "Galileo OSNMA: stored DSM-KROOT cache lacks freshness metadata. Falling back to :: WARM START.";
            std::cout << "Galileo OSNMA: stored DSM-KROOT cache lacks freshness metadata. Falling back to :: WARM START." << std::endl;
            d_pending_dsm_kroot_cache = CachedDsmKroot();
            return;
        }
    if (d_active_public_key_id_valid && d_pending_dsm_kroot_cache.pkid != d_active_public_key_id)
        {
            LOG(INFO) << "Galileo OSNMA: stored DSM-KROOT cache PKID="
                      << static_cast<uint32_t>(d_pending_dsm_kroot_cache.pkid)
                      << " does not match active Public Key ID="
                      << static_cast<uint32_t>(d_active_public_key_id)
                      << ". Falling back to :: WARM START.";
            std::cout << "Galileo OSNMA: stored DSM-KROOT cache PKID does not match active Public Key ID. Falling back to :: WARM START." << std::endl;
            d_pending_dsm_kroot_cache = CachedDsmKroot();
            return;
        }

    const std::string active_fingerprint = active_public_key_fingerprint();
    if (active_fingerprint.empty())
        {
            LOG(INFO) << "Galileo OSNMA: stored DSM-KROOT cache cannot be used because the active Public Key has no canonical OSNMA fingerprint. Falling back to :: WARM START.";
            std::cout << "Galileo OSNMA: stored DSM-KROOT cache cannot be used because the active Public Key has no canonical OSNMA fingerprint. Falling back to :: WARM START." << std::endl;
            d_pending_dsm_kroot_cache = CachedDsmKroot();
            return;
        }
    if (active_fingerprint != d_pending_dsm_kroot_cache.public_key_fingerprint_sha256)
        {
            LOG(INFO) << "Galileo OSNMA: stored DSM-KROOT cache Public Key fingerprint mismatch. "
                      << "active=" << active_fingerprint
                      << ", cached=" << d_pending_dsm_kroot_cache.public_key_fingerprint_sha256
                      << ". Falling back to :: WARM START.";
            std::cout << "Galileo OSNMA: stored DSM-KROOT cache Public Key fingerprint mismatch. Falling back to :: WARM START." << std::endl;
            d_pending_dsm_kroot_cache = CachedDsmKroot();
            return;
        }
    if (!d_pending_dsm_kroot_cache.public_key_type.empty() &&
        d_pending_dsm_kroot_cache.public_key_type != d_crypto->get_public_key_type())
        {
            LOG(INFO) << "Galileo OSNMA: stored DSM-KROOT cache Public Key type mismatch. Falling back to :: WARM START.";
            std::cout << "Galileo OSNMA: stored DSM-KROOT cache Public Key type mismatch. Falling back to :: WARM START." << std::endl;
            d_pending_dsm_kroot_cache = CachedDsmKroot();
            return;
        }

    const int64_t age_s = gst_delta_seconds(current_gst, d_pending_dsm_kroot_cache.signature_verified_at_gst);
    if (age_s < 0)
        {
            LOG(INFO) << "Galileo OSNMA: stored DSM-KROOT cache verification time is in the future. Falling back to :: WARM START.";
            std::cout << "Galileo OSNMA: stored DSM-KROOT cache verification time is in the future. Falling back to :: WARM START." << std::endl;
            d_pending_dsm_kroot_cache = CachedDsmKroot();
            return;
        }
    if (age_s >= dsm_kroot_expiration_s)
        {
            LOG(INFO) << "Galileo OSNMA: stored DSM-KROOT cache signature verification is stale after "
                      << age_s << " s. Retrieving DSM-KROOT from SIS.";
            std::cout << "Galileo OSNMA: stored DSM-KROOT cache signature verification is stale after "
                      << age_s << " s. Retrieving DSM-KROOT from SIS." << std::endl;
            d_pending_dsm_kroot_cache = CachedDsmKroot();
            return;
        }
    if (gst_delta_seconds(current_gst, d_pending_dsm_kroot_cache.gst0) < 0)
        {
            LOG(INFO) << "Galileo OSNMA: stored DSM-KROOT cache is verified but not applicable yet. Waiting for GST0.";
            return;
        }

    d_flag_hot_start = true;
    process_dsm_message(d_pending_dsm_kroot_cache.dsm, d_pending_dsm_kroot_cache.nma_header);
    d_flag_hot_start = false;

    if (d_kroot_verified && d_osnma_data.d_dsm_kroot_message.verified)
        {
            d_kroot_loaded_from_cache = true;
            LOG(INFO) << "Galileo OSNMA DSM-KROOT available :: HOT START";
            std::cout << "Galileo OSNMA DSM-KROOT available :: HOT START" << std::endl;
        }
    else
        {
            d_count_failed_Kroot = 0;
            d_osnma_data.d_dsm_kroot_message = DSM_KROOT_message();
            invalidate_verified_kroot();
            LOG(INFO) << "Galileo OSNMA DSM-KROOT saved data does not match the current public key :: WARM START";
            std::cout << "Galileo OSNMA DSM-KROOT saved data does not match the current public key :: WARM START" << std::endl;
        }

    d_pending_dsm_kroot_cache = CachedDsmKroot();
}


bool osnma_msg_receiver::store_dsm_kroot(const std::vector<uint8_t>& dsm, const uint8_t nma_header, const DSM_KROOT_message& kroot) const
{
    std::ofstream file(KROOTFILE_DEFAULT, std::ios::binary | std::ios::out);

    if (!file.is_open())
        {
            return false;
        }

    // NMA header
    file.write(reinterpret_cast<const char*>(&nma_header), 1);

    // Then writing the entire dsm_msg vector to the file
    file.write(reinterpret_cast<const char*>(dsm.data()), dsm.size());

    if (!file.good())
        {
            return false;
        }

    if (!store_dsm_kroot_metadata(dsm, nma_header, kroot))
        {
            LOG(WARNING) << "Galileo OSNMA: unable to store DSM-KROOT metadata in " << kroot_metadata_path();
            return false;
        }

    return true;
}


bool osnma_msg_receiver::store_dsm_kroot_metadata(const std::vector<uint8_t>& dsm, const uint8_t nma_header, const DSM_KROOT_message& kroot) const
{
    if (d_GST_SIS == 0 || !kroot.verified)
        {
            return false;
        }

    const std::string public_key_fingerprint = active_public_key_fingerprint();
    if (public_key_fingerprint.empty())
        {
            return false;
        }
    const auto active_public_key = d_material_manager->active_public_key();

    const auto raw = make_cached_kroot_raw(nma_header, dsm);
    const uint32_t gst0 = d_helper->compute_gst(kroot.wn_k, kroot.towh_k * 3600);
    const uint32_t kroot_gst = gst_with_offset(kroot.wn_k, kroot.towh_k * 3600, -osnma_subframe_duration_s);

    std::ostringstream output;
    output << "version=1\n";
    output << "kind=osnma-kroot-cache\n";
    output << "raw_sha256=" << bytes_to_upper_hex(d_crypto->compute_SHA_256(raw)) << "\n";
    output << "nma_header=" << static_cast<uint32_t>(nma_header) << "\n";
    output << "pkid=" << static_cast<uint32_t>(kroot.pkid) << "\n";
    output << "cidkr=" << static_cast<uint32_t>(kroot.cidkr) << "\n";
    output << "nmas=" << static_cast<uint32_t>(d_dsm_reader->get_nmas(nma_header)) << "\n";
    output << "cpks=" << static_cast<uint32_t>(d_dsm_reader->get_cpks(nma_header)) << "\n";
    output << "hf=" << static_cast<uint32_t>(kroot.hf) << "\n";
    output << "mf=" << static_cast<uint32_t>(kroot.mf) << "\n";
    output << "ks=" << static_cast<uint32_t>(kroot.ks) << "\n";
    output << "ts=" << static_cast<uint32_t>(kroot.ts) << "\n";
    output << "maclt=" << static_cast<uint32_t>(kroot.maclt) << "\n";
    output << "wn_k=" << static_cast<uint32_t>(kroot.wn_k) << "\n";
    output << "towh_k=" << static_cast<uint32_t>(kroot.towh_k) << "\n";
    output << "gst0=" << gst0 << "\n";
    output << "kroot_gst=" << kroot_gst << "\n";
    output << "kroot_sha256=" << bytes_to_upper_hex(d_crypto->compute_SHA_256(kroot.kroot)) << "\n";
    output << "public_key_sha256=" << public_key_fingerprint << "\n";
    if (active_public_key.npkt_valid)
        {
            output << "public_key_npkt=" << static_cast<uint32_t>(active_public_key.npkt) << "\n";
        }
    output << "public_key_type=" << d_crypto->get_public_key_type() << "\n";
    output << "signature_verified_at_gst=" << d_GST_SIS << "\n";
    output << "source=sis-dsm-kroot\n";

    return write_text_file_atomically(kroot_metadata_path(), output.str());
}


bool osnma_msg_receiver::store_public_key_metadata(uint8_t pkid, uint8_t npkt, const std::string& source)
{
    const std::vector<uint8_t> compressed_key = d_crypto->get_public_key_compressed();
    const std::string fingerprint = public_key_fingerprint_sha256(pkid, npkt, compressed_key);
    if (fingerprint.empty())
        {
            LOG(WARNING) << "Galileo OSNMA: Unable to compute canonical Public Key fingerprint";
            return false;
        }

    Osnma_Public_Key_Material active_public_key;
    active_public_key.valid = true;
    active_public_key.pkid_valid = true;
    active_public_key.pkid = pkid;
    active_public_key.npkt_valid = true;
    active_public_key.npkt = npkt;
    active_public_key.key_type = d_crypto->get_public_key_type();
    active_public_key.source = source;
    active_public_key.pem_path = PEMFILE_DEFAULT;
    active_public_key.compressed_key = compressed_key;
    active_public_key.fingerprint_sha256 = fingerprint;
    d_material_manager->set_active_public_key(active_public_key);
    if (!d_material_manager->store_active_public_key_cache(active_public_key))
        {
            LOG(WARNING) << "Galileo OSNMA: Unable to store Public Key metadata in " << PEMFILE_DEFAULT << ".meta";
            return false;
        }
    return true;
}


std::pair<std::vector<uint8_t>, uint8_t> osnma_msg_receiver::parse_dsm_kroot() const
{
    std::ifstream file(KROOTFILE_DEFAULT, std::ios::binary | std::ios::in);
    if (!file)
        {
            return {std::vector<uint8_t>(), 0};
        }

    // Read the first byte into hkroot[0]
    uint8_t nma_header;
    file.read(reinterpret_cast<char*>(&nma_header), 1);

    // Read the remaining file content into dsm_msg
    std::vector<uint8_t> dsm_msg((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    file.close();

    if (file.bad())
        {
            return {std::vector<uint8_t>(), 0};
        }

    return {dsm_msg, nma_header};
}


std::string osnma_msg_receiver::active_public_key_fingerprint() const
{
    const auto active_public_key = d_material_manager->active_public_key();
    if (active_public_key.valid &&
        active_public_key.pkid_valid &&
        active_public_key.npkt_valid &&
        !active_public_key.fingerprint_sha256.empty())
        {
            return active_public_key.fingerprint_sha256;
        }
    return {};
}


void osnma_msg_receiver::set_merkle_root(const std::vector<uint8_t>& v)
{
    Osnma_Merkle_Tree_Material merkle_tree;
    merkle_tree.valid = !v.empty();
    merkle_tree.source = "manual";
    merkle_tree.root = v;
    merkle_tree.hash_function = d_crypto->get_merkle_tree_hash_function();

    if (d_flag_merkle_tree_renewal)
        {
            d_material_manager->set_candidate_merkle_tree(merkle_tree);
            d_new_merkle_tree_loaded = merkle_tree_differs_from_renewal_start(merkle_tree);
            return;
        }

    d_crypto->set_merkle_root(v);
    d_material_manager->set_active_merkle_tree(merkle_tree);
}
