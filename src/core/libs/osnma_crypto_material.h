/*!
 * \file osnma_crypto_material.h
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

#ifndef GNSS_SDR_OSNMA_CRYPTO_MATERIAL_H
#define GNSS_SDR_OSNMA_CRYPTO_MATERIAL_H

#include <cstdint>
#include <string>
#include <vector>

struct Osnma_Public_Key_Material
{
    bool valid{false};
    bool pkid_valid{false};
    bool npkt_valid{false};
    uint8_t pkid{0};
    uint8_t npkt{0};
    std::string key_type;
    std::string source;
    std::string fingerprint_sha256;
    std::string pem_path;
    std::string product_uid;
    std::string applicability;
    std::string state;
    std::vector<uint8_t> compressed_key;
};


struct Osnma_Merkle_Tree_Material
{
    struct PublicKeyEntry
    {
        bool pkid_valid{false};
        uint8_t pkid{0};
        uint32_t leaf_index{0};
        uint32_t length_bits{0};
        std::string key_type;
        std::string point;
    };

    bool valid{false};
    std::string source;
    std::string uid;
    std::string applicability;
    std::string state;
    std::string hash_function;
    std::vector<uint8_t> root;
    std::string xml_path;
    std::vector<PublicKeyEntry> public_keys;
};


Osnma_Merkle_Tree_Material osnma_read_merkle_tree_xml(const std::string& merkle_file_path);


class Osnma_Crypto_Material_Manager
{
public:
    explicit Osnma_Crypto_Material_Manager(const std::string& cache_dir);

    Osnma_Public_Key_Material load_active_public_key_cache(const std::string& expected_fingerprint = std::string()) const;
    Osnma_Public_Key_Material load_public_key_metadata(const std::string& pem_path, const std::string& expected_fingerprint = std::string()) const;
    Osnma_Public_Key_Material candidate_public_key(uint8_t pkid) const;
    Osnma_Merkle_Tree_Material candidate_merkle_tree() const;
    Osnma_Merkle_Tree_Material load_configured_merkle_tree(const std::string& path) const;
    Osnma_Merkle_Tree_Material load_active_merkle_tree_cache() const;
    Osnma_Merkle_Tree_Material load_future_merkle_tree_cache() const;
    Osnma_Public_Key_Material active_public_key() const;
    Osnma_Merkle_Tree_Material active_merkle_tree() const;

    bool store_active_public_key_cache(const Osnma_Public_Key_Material& key) const;
    bool store_public_key_metadata(const Osnma_Public_Key_Material& key) const;
    bool store_active_merkle_tree_cache(const Osnma_Merkle_Tree_Material& tree) const;
    bool store_future_merkle_tree_cache(const Osnma_Merkle_Tree_Material& tree) const;
    bool has_candidate_material_for_pkid(uint8_t pkid) const;

    void set_active_public_key(const Osnma_Public_Key_Material& key);
    void set_active_merkle_tree(const Osnma_Merkle_Tree_Material& tree);
    void set_candidate_public_key(const Osnma_Public_Key_Material& key);
    void set_candidate_merkle_tree(const Osnma_Merkle_Tree_Material& tree);
    void clear_candidate_public_key();
    void clear_candidate_merkle_tree();

    bool promote_candidate_for_pkid(uint8_t pkid);
    bool promote_candidate_merkle_tree();

private:
    std::string metadata_path_for_key(const std::string& pem_path) const;
    std::string cache_path(const std::string& file_name) const;

    std::string d_cache_dir;
    Osnma_Public_Key_Material d_active_public_key;
    Osnma_Public_Key_Material d_candidate_public_key;
    Osnma_Merkle_Tree_Material d_active_merkle_tree;
    Osnma_Merkle_Tree_Material d_candidate_merkle_tree;
};

#endif  // GNSS_SDR_OSNMA_CRYPTO_MATERIAL_H
