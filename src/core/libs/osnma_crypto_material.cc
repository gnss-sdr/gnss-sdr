/*!
 * \file osnma_crypto_material.cc
 * \brief OSNMA cryptographic material metadata and local cache manager.
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
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

#include "osnma_crypto_material.h"
#include "Galileo_OSNMA.h"
#include "gnss_sdr_filesystem.h"
#include <pugixml.hpp>
#include <algorithm>
#include <cctype>
#include <exception>
#include <fstream>
#include <iterator>
#include <map>
#include <sstream>

#if USE_GLOG_AND_GFLAGS
#include <glog/logging.h>
#else
#include <absl/log/log.h>
#endif

namespace
{
std::string trim_copy(const std::string& input)
{
    const auto first = std::find_if_not(input.cbegin(), input.cend(), [](unsigned char c) { return std::isspace(c) != 0; });
    const auto last = std::find_if_not(input.crbegin(), input.crend(), [](unsigned char c) { return std::isspace(c) != 0; }).base();
    if (first >= last)
        {
            return {};
        }
    return std::string(first, last);
}


std::string normalize_hash_function(const std::string& hash_function)
{
    if (hash_function == "SHA-256" || hash_function == "SHA256")
        {
            return "SHA-256";
        }
    if (hash_function == "SHA3-256" || hash_function == "SHA3_256")
        {
            return "SHA3-256";
        }
    return "Unknown";
}


std::string normalize_public_key_type(const std::string& key_type)
{
    if (key_type == "ECDSA P-256" || key_type == "ECDSA P-256/SHA-256")
        {
            return "ECDSA P-256";
        }
    if (key_type == "ECDSA P-521" || key_type == "ECDSA P-521/SHA-512")
        {
            return "ECDSA P-521";
        }
    return key_type.empty() ? std::string("Unknown") : key_type;
}


bool hex_nibble(char c, uint8_t& value)
{
    if (c >= '0' && c <= '9')
        {
            value = static_cast<uint8_t>(c - '0');
            return true;
        }
    if (c >= 'a' && c <= 'f')
        {
            value = static_cast<uint8_t>(c - 'a' + 10);
            return true;
        }
    if (c >= 'A' && c <= 'F')
        {
            value = static_cast<uint8_t>(c - 'A' + 10);
            return true;
        }
    return false;
}


bool convert_from_hex_string(const std::string& input, std::vector<uint8_t>& bytes)
{
    bytes.clear();
    const std::string trimmed_input = trim_copy(input);
    if (trimmed_input.empty() || (trimmed_input.size() % 2) != 0)
        {
            return false;
        }
    bytes.reserve(trimmed_input.size() / 2);
    for (size_t i = 0; i < trimmed_input.size(); i += 2)
        {
            uint8_t high = 0;
            uint8_t low = 0;
            if (!hex_nibble(trimmed_input[i], high) || !hex_nibble(trimmed_input[i + 1], low))
                {
                    bytes.clear();
                    return false;
                }
            bytes.push_back(static_cast<uint8_t>((high << 4U) | low));
        }
    return true;
}


bool parse_uint4_value(const std::string& value, uint8_t& output)
{
    try
        {
            size_t processed = 0;
            const auto parsed = static_cast<unsigned long>(std::stoul(value, &processed, 0));
            if (processed != value.size())
                {
                    return false;
                }
            if (parsed > 15UL)
                {
                    return false;
                }
            output = static_cast<uint8_t>(parsed);
            return true;
        }
    catch (const std::exception&)
        {
            return false;
        }
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


bool replace_file_with_temporary_file(const std::string& tmp_path, const std::string& destination_path)
{
    const fs::path tmp(tmp_path);
    const fs::path destination(destination_path);
    const fs::path backup(destination_path + ".bak");
    errorlib::error_code error;

    fs::remove(backup, error);
    if (error)
        {
            return false;
        }

    const bool destination_exists = fs::exists(destination);
    if (destination_exists)
        {
            error.clear();
            fs::rename(destination, backup, error);
            if (error)
                {
                    return false;
                }
        }

    fs::rename(tmp, destination, error);
    if (!error)
        {
            if (destination_exists)
                {
                    errorlib::error_code remove_error;
                    fs::remove(backup, remove_error);
                }
            return true;
        }

    error.clear();
    if (destination_exists)
        {
            fs::rename(backup, destination, error);
        }
    return false;
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
    return replace_file_with_temporary_file(tmp_path, path);
}


bool copy_file_atomically(const std::string& source_path, const std::string& destination_path)
{
    if (source_path.empty() || destination_path.empty())
        {
            return false;
        }

    std::ifstream source(source_path, std::ios::binary);
    if (!source)
        {
            return false;
        }

    const std::string tmp_path = destination_path + ".tmp";
    {
        std::ofstream destination(tmp_path, std::ios::binary | std::ios::trunc);
        if (!destination)
            {
                return false;
            }
        destination << source.rdbuf();
        if (!destination.good())
            {
                return false;
            }
    }

    return replace_file_with_temporary_file(tmp_path, destination_path);
}
}  // namespace


Osnma_Merkle_Tree_Material osnma_read_merkle_tree_xml(const std::string& merkle_file_path)
{
    Osnma_Merkle_Tree_Material material;
    material.xml_path = merkle_file_path;
    material.source = merkle_file_path.empty() ? std::string() : std::string("configured-xml");

    pugi::xml_document doc;
    const pugi::xml_parse_result result = doc.load_file(merkle_file_path.c_str());
    if (!result)
        {
            if (!merkle_file_path.empty())
                {
                    LOG(WARNING) << "Galileo OSNMA: Merkle Tree XML file " << merkle_file_path
                                 << " could not be read: " << result.description();
                }
            return material;
        }

    try
        {
            const pugi::xml_node root = doc.child("signalData");
            const pugi::xml_node header = root.child("header");
            const pugi::xml_node body = root.child("body");
            const pugi::xml_node gal_header = header.child("GAL-header");
            const pugi::xml_node merkle_tree = body.child("MerkleTree");

            material.uid = merkle_tree.child_value("UID");
            material.applicability = merkle_tree.child_value("Applicability");
            material.state = merkle_tree.child_value("State");
            material.hash_function = normalize_hash_function(merkle_tree.child_value("HashFunction"));

            const std::string issue_date = gal_header.child("issueDate").text().get();
            const std::string signal_version = gal_header.child("signalVersion").text().get();
            const std::string data_version = gal_header.child("dataVersion").text().get();
            LOG(INFO) << "Galileo OSNMA Merkletree - Issue Date: " << issue_date;
            LOG(INFO) << "Galileo OSNMA Merkletree - Signal Version: " << signal_version;
            LOG(INFO) << "Galileo OSNMA Merkletree - Data Version: " << data_version;
            LOG(INFO) << "Galileo OSNMA Merkletree - Hash Function: " << material.hash_function;

            for (pugi::xml_node public_key : merkle_tree.children("PublicKey"))
                {
                    Osnma_Merkle_Tree_Material::PublicKeyEntry entry;
                    entry.leaf_index = static_cast<uint32_t>(std::stoul(public_key.child_value("i")));
                    entry.length_bits = static_cast<uint32_t>(std::stoul(public_key.child_value("lengthInBits")));
                    entry.point = public_key.child_value("point");
                    entry.key_type = normalize_public_key_type(public_key.child_value("PKType"));
                    entry.pkid_valid = parse_uint4_value(public_key.child_value("PKID"), entry.pkid);
                    material.public_keys.push_back(entry);

                    LOG(INFO) << "Galileo OSNMA Merkletree - Public Key: " << entry.leaf_index;
                    LOG(INFO) << "Galileo OSNMA Merkletree - PKID: " << static_cast<uint32_t>(entry.pkid);
                    LOG(INFO) << "Galileo OSNMA Merkletree - PK Type: " << entry.key_type;
                }

            for (pugi::xml_node tree_node : merkle_tree.children("TreeNode"))
                {
                    const int j = std::stoi(tree_node.child_value("j"));
                    const int i = std::stoi(tree_node.child_value("i"));
                    const std::string x_ji = tree_node.child_value("x_ji");
                    LOG(INFO) << "Galileo OSNMA Merkletree - m_" << j << "_" << i << " = " << x_ji;
                    if (j == 4 && i == 0)
                        {
                            if (!convert_from_hex_string(x_ji, material.root))
                                {
                                    LOG(WARNING) << "Galileo OSNMA: invalid Merkle Tree root in " << merkle_file_path;
                                    material.root.clear();
                                }
                        }
                }

            material.valid = !material.root.empty() && material.hash_function != "Unknown";
        }
    catch (const std::exception& e)
        {
            LOG(INFO) << "Exception raised reading the " << merkle_file_path << " file: " << e.what();
            material = Osnma_Merkle_Tree_Material();
            material.xml_path = merkle_file_path;
        }
    return material;
}


Osnma_Crypto_Material_Manager::Osnma_Crypto_Material_Manager(const std::string& cache_dir) : d_cache_dir(cache_dir)
{
}


Osnma_Public_Key_Material Osnma_Crypto_Material_Manager::load_active_public_key_cache(const std::string& expected_fingerprint) const
{
    return load_public_key_metadata(PEMFILE_DEFAULT, expected_fingerprint);
}


Osnma_Public_Key_Material Osnma_Crypto_Material_Manager::load_public_key_metadata(const std::string& pem_path, const std::string& expected_fingerprint) const
{
    Osnma_Public_Key_Material key;
    key.pem_path = pem_path;

    const auto values = read_key_value_file(metadata_path_for_key(pem_path));
    if (values.empty())
        {
            return key;
        }

    auto it = values.find("version");
    if (it == values.cend() || it->second != "1")
        {
            return key;
        }

    it = values.find("pkid");
    if (it != values.cend())
        {
            key.pkid_valid = parse_uint4_value(it->second, key.pkid);
        }
    it = values.find("npkt");
    if (it != values.cend())
        {
            key.npkt_valid = parse_uint4_value(it->second, key.npkt);
        }
    it = values.find("pk_type");
    if (it != values.cend())
        {
            key.key_type = normalize_public_key_type(it->second);
        }
    it = values.find("source");
    if (it != values.cend())
        {
            key.source = it->second;
        }
    it = values.find("public_key_sha256");
    if (it != values.cend())
        {
            key.fingerprint_sha256 = it->second;
        }
    it = values.find("product_uid");
    if (it != values.cend())
        {
            key.product_uid = it->second;
        }
    it = values.find("applicability");
    if (it != values.cend())
        {
            key.applicability = it->second;
        }
    it = values.find("state");
    if (it != values.cend())
        {
            key.state = it->second;
        }

    if (!expected_fingerprint.empty() && key.fingerprint_sha256 != expected_fingerprint)
        {
            LOG(WARNING) << "Galileo OSNMA: Public Key metadata fingerprint mismatch for " << pem_path;
            return {};
        }

    key.valid = key.pkid_valid &&
                (key.key_type == "ECDSA P-256" || key.key_type == "ECDSA P-521") &&
                key.npkt_valid &&
                !key.fingerprint_sha256.empty();
    return key;
}


bool Osnma_Crypto_Material_Manager::store_active_public_key_cache(const Osnma_Public_Key_Material& key) const
{
    Osnma_Public_Key_Material stored_key = key;
    if (stored_key.pem_path.empty())
        {
            stored_key.pem_path = PEMFILE_DEFAULT;
        }
    return store_public_key_metadata(stored_key);
}


bool Osnma_Crypto_Material_Manager::store_public_key_metadata(const Osnma_Public_Key_Material& key) const
{
    if (key.pem_path.empty() || !key.valid || !key.pkid_valid || !key.npkt_valid || key.fingerprint_sha256.empty())
        {
            return false;
        }

    std::ostringstream output;
    output << "version=1\n";
    output << "pkid=" << static_cast<uint32_t>(key.pkid) << "\n";
    output << "npkt=" << static_cast<uint32_t>(key.npkt) << "\n";
    output << "pk_type=" << key.key_type << "\n";
    output << "public_key_sha256=" << key.fingerprint_sha256 << "\n";
    if (!key.source.empty())
        {
            output << "source=" << key.source << "\n";
        }
    if (!key.product_uid.empty())
        {
            output << "product_uid=" << key.product_uid << "\n";
        }
    if (!key.applicability.empty())
        {
            output << "applicability=" << key.applicability << "\n";
        }
    if (!key.state.empty())
        {
            output << "state=" << key.state << "\n";
        }

    return write_text_file_atomically(metadata_path_for_key(key.pem_path), output.str());
}


Osnma_Merkle_Tree_Material Osnma_Crypto_Material_Manager::load_configured_merkle_tree(const std::string& path) const
{
    return osnma_read_merkle_tree_xml(path);
}


Osnma_Merkle_Tree_Material Osnma_Crypto_Material_Manager::load_active_merkle_tree_cache() const
{
    return load_configured_merkle_tree(cache_path("OSNMA_MerkleTree.xml"));
}


Osnma_Merkle_Tree_Material Osnma_Crypto_Material_Manager::load_future_merkle_tree_cache() const
{
    return load_configured_merkle_tree(cache_path("OSNMA_MerkleTree_Future.xml"));
}


bool Osnma_Crypto_Material_Manager::store_active_merkle_tree_cache(const Osnma_Merkle_Tree_Material& tree) const
{
    if (!tree.valid || tree.xml_path.empty())
        {
            return false;
        }
    return copy_file_atomically(tree.xml_path, cache_path("OSNMA_MerkleTree.xml"));
}


bool Osnma_Crypto_Material_Manager::store_future_merkle_tree_cache(const Osnma_Merkle_Tree_Material& tree) const
{
    if (!tree.valid || tree.xml_path.empty())
        {
            return false;
        }
    return copy_file_atomically(tree.xml_path, cache_path("OSNMA_MerkleTree_Future.xml"));
}


bool Osnma_Crypto_Material_Manager::has_candidate_material_for_pkid(uint8_t pkid) const
{
    return d_candidate_public_key.valid &&
           d_candidate_public_key.pkid_valid &&
           d_candidate_public_key.pkid == pkid;
}


Osnma_Public_Key_Material Osnma_Crypto_Material_Manager::candidate_public_key(uint8_t pkid) const
{
    if (has_candidate_material_for_pkid(pkid))
        {
            return d_candidate_public_key;
        }
    return {};
}


Osnma_Merkle_Tree_Material Osnma_Crypto_Material_Manager::candidate_merkle_tree() const
{
    return d_candidate_merkle_tree;
}


void Osnma_Crypto_Material_Manager::set_active_public_key(const Osnma_Public_Key_Material& key)
{
    d_active_public_key = key;
}


void Osnma_Crypto_Material_Manager::set_active_merkle_tree(const Osnma_Merkle_Tree_Material& tree)
{
    d_active_merkle_tree = tree;
}


void Osnma_Crypto_Material_Manager::set_candidate_public_key(const Osnma_Public_Key_Material& key)
{
    d_candidate_public_key = key;
}


void Osnma_Crypto_Material_Manager::set_candidate_merkle_tree(const Osnma_Merkle_Tree_Material& tree)
{
    d_candidate_merkle_tree = tree;
}


void Osnma_Crypto_Material_Manager::clear_candidate_public_key()
{
    d_candidate_public_key = Osnma_Public_Key_Material();
}


void Osnma_Crypto_Material_Manager::clear_candidate_merkle_tree()
{
    d_candidate_merkle_tree = Osnma_Merkle_Tree_Material();
}


Osnma_Public_Key_Material Osnma_Crypto_Material_Manager::active_public_key() const
{
    return d_active_public_key;
}


Osnma_Merkle_Tree_Material Osnma_Crypto_Material_Manager::active_merkle_tree() const
{
    return d_active_merkle_tree;
}


bool Osnma_Crypto_Material_Manager::promote_candidate_for_pkid(uint8_t pkid)
{
    if (!has_candidate_material_for_pkid(pkid))
        {
            return false;
        }
    d_active_public_key = d_candidate_public_key;
    d_candidate_public_key = Osnma_Public_Key_Material();
    return true;
}


bool Osnma_Crypto_Material_Manager::promote_candidate_merkle_tree()
{
    if (!d_candidate_merkle_tree.valid)
        {
            return false;
        }
    d_active_merkle_tree = d_candidate_merkle_tree;
    d_candidate_merkle_tree = Osnma_Merkle_Tree_Material();
    return true;
}


std::string Osnma_Crypto_Material_Manager::metadata_path_for_key(const std::string& pem_path) const
{
    if (pem_path.empty())
        {
            return {};
        }
    return pem_path + ".meta";
}


std::string Osnma_Crypto_Material_Manager::cache_path(const std::string& file_name) const
{
    if (d_cache_dir.empty() || d_cache_dir == ".")
        {
            return "./" + file_name;
        }
    if (d_cache_dir.back() == '/')
        {
            return d_cache_dir + file_name;
        }
    return d_cache_dir + "/" + file_name;
}
