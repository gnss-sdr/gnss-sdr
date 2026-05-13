/*!
 * \file osnma_msg_receiver.h
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

#ifndef GNSS_SDR_OSNMA_MSG_RECEIVER_H
#define GNSS_SDR_OSNMA_MSG_RECEIVER_H

#define FRIEND_TEST(test_case_name, test_name) \
    friend class test_case_name##_##test_name##_Test

#include "galileo_inav_message.h"    // for OSNMA_msg
#include "gnss_block_interface.h"    // for gnss_shared_ptr
#include "osnma_crypto_material.h"   // for OSNMA cryptographic material manager
#include "osnma_data.h"              // for OSNMA_data structures
#include "osnma_nav_data_manager.h"  // for OSNMA_NavDataManager
#include <gnuradio/block.h>          // for gr::block
#include <pmt/pmt.h>                 // for pmt::pmt_t
#include <array>                     // for std::array
#include <cstddef>                   // for size_t
#include <cstdint>                   // for uint8_t
#include <ctime>                     // for std::time_t
#include <initializer_list>          // for std::initializer_list
#include <map>                       // for std::map, std::multimap
#include <memory>                    // for std::shared_ptr
#include <string>                    // for std::string
#include <utility>                   // for std::pair
#include <vector>                    // for std::vector

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */

class OSNMA_DSM_Reader;
class Gnss_Crypto;
class Osnma_Helper;
class osnma_msg_receiver;

using osnma_msg_receiver_sptr = gnss_shared_ptr<osnma_msg_receiver>;

osnma_msg_receiver_sptr osnma_msg_receiver_make(const std::string& pemFilePath, const std::string& merkleFilePath, bool strict_mode = false, bool replay_mode = false);

/*!
 * \brief GNU Radio block that receives asynchronous OSNMA messages
 * from the telemetry blocks, stores them in memory, and decodes OSNMA info
 * when enough data have been received.
 * The decoded OSNMA data is sent to the PVT block.
 */
class osnma_msg_receiver : public gr::block
{
public:
    ~osnma_msg_receiver() = default;                            //!< Default destructor
    bool verify_dsm_pkr(const DSM_PKR_message& message) const;  //!< Public for benchmarking purposes
    bool verify_dsm_pkr(const DSM_PKR_message& message, const Osnma_Merkle_Tree_Material& merkle_tree) const;
    void msg_handler_osnma(const pmt::pmt_t& msg);        //!< For testing purposes
    void read_merkle_xml(const std::string& merklepath);  //!< Public for testing purposes
    void set_merkle_root(const std::vector<uint8_t>& v);  //!< Public for benchmarking purposes

private:
    friend osnma_msg_receiver_sptr osnma_msg_receiver_make(const std::string& pemFilePath, const std::string& merkleFilePath, bool strict_mode, bool replay_mode);
    osnma_msg_receiver(const std::string& crtFilePath, const std::string& merkleFilePath, bool strict_mode, bool replay_mode);

    void process_osnma_message(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void read_nma_header(uint8_t nma_header);
    void read_dsm_header(uint8_t dsm_header);
    void read_dsm_block(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void process_dsm_block(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void process_dsm_message(const std::vector<uint8_t>& dsm_msg, const uint8_t& nma_header);
    void expire_dsm_accumulator_if_needed(uint8_t dsm_id, uint32_t current_gst);
    void reset_dsm_accumulator(uint8_t dsm_id);
    void reset_dsm_accumulators();
    void reset_tesla_chain_state(bool preserve_deferred_mack_blocks = false);
    void promote_verified_future_kroot_if_due(uint32_t current_gst);
    void expire_verified_kroot_if_needed(uint32_t current_gst);
    void handle_authenticated_revocation(bool chain_revocation, bool public_key_revocation, bool preserve_future_kroot, bool preserve_active_public_key);
    void handle_verified_alert_message();
    void handle_authenticated_dont_use_status();
    void read_and_process_mack_block(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void process_mack_message();
    void try_verify_pending_tags(bool log_unavailable_tags);
    void process_deferred_mack_blocks();
    void store_deferred_mack_block(const std::shared_ptr<OSNMA_msg>& osnma_msg);
    void remove_verified_tags();
    void control_tags_awaiting_verify_size();
    void send_data_to_pvt(const std::vector<OSNMA_NavData>& data);
    void prune_old_tesla_keys(uint32_t reference_gst);

    bool read_mack_header();
    bool read_mack_body();
    bool process_mack_block(const std::shared_ptr<OSNMA_msg>& osnma_msg, const std::vector<uint8_t>& allowed_adkds, uint8_t nmas);
    bool set_gst_sf_for_mack(uint32_t WN, uint32_t TOW);
    bool verify_tesla_key(std::vector<uint8_t>& key, uint32_t key_gst);
    bool verify_tag(Tag& tag) const;
    bool tag_has_nav_data_available(const Tag& t) const;
    bool tag_has_key_available(const Tag& t) const;
    bool tag_is_allowed_by_time_constraint(const Tag& tag) const;
    bool kroot_parameters_are_supported(const DSM_KROOT_message& kroot) const;
    bool verified_kroot_is_fresh(uint32_t current_gst) const;
    bool load_pending_dsm_kroot_cache();
    bool load_dsm_kroot_metadata();
    bool store_dsm_kroot(const std::vector<uint8_t>& dsm, const uint8_t nma_header, const DSM_KROOT_message& kroot) const;
    bool store_dsm_kroot_metadata(const std::vector<uint8_t>& dsm, const uint8_t nma_header, const DSM_KROOT_message& kroot) const;
    bool store_public_key_metadata(uint8_t pkid, uint8_t npkt, const std::string& source);
    bool npkt_from_public_key_type(const std::string& key_type, uint8_t& npkt) const;
    bool pkid_from_certificate_subject_cn(const std::string& crt_file_path, uint8_t& pkid) const;
    bool resolve_configured_public_key_identity(const std::string& key_path, const std::vector<uint8_t>& compressed_key, const std::string& key_type, Osnma_Public_Key_Material& key_material) const;
    bool merkle_tree_differs_from_renewal_start() const;
    bool merkle_tree_differs_from_renewal_start(const Osnma_Merkle_Tree_Material& merkle_tree) const;
    bool ensure_new_merkle_tree_available();
    bool mack_bits_available(size_t bit_offset, size_t bit_length) const;
    bool read_mack_byte(size_t bit_offset, uint8_t& value) const;
    bool merge_partial_tesla_key(uint32_t key_gst, const std::vector<uint8_t>& key_bytes, const std::vector<uint8_t>& key_byte_received, std::vector<uint8_t>& merged_key);
    bool get_tesla_key(uint32_t WN, uint32_t TOW, int32_t offset_seconds, std::vector<uint8_t>& key) const;
    bool get_tesla_key_for_adkd(uint32_t WN, uint32_t TOW, int32_t offset_seconds, uint8_t adkd, std::vector<uint8_t>& key) const;
    bool has_tesla_key(uint32_t WN, uint32_t TOW, int32_t offset_seconds) const;
    bool has_tesla_key_for_adkd(uint32_t WN, uint32_t TOW, int32_t offset_seconds, uint8_t adkd) const;

    uint32_t gst_with_offset(uint32_t WN, uint32_t TOW, int32_t offset_seconds) const;
    int64_t gst_delta_seconds(uint32_t lhs_gst, uint32_t rhs_gst) const;
    int64_t gst_to_seconds(uint32_t gst) const;
    uint64_t read_mack_bits(size_t bit_offset, size_t bit_length) const;

    void evaluate_pending_dsm_kroot_cache(uint32_t current_gst);
    void invalidate_verified_kroot();
    std::pair<std::vector<uint8_t>, uint8_t> parse_dsm_kroot() const;
    std::string active_public_key_fingerprint() const;
    std::string public_key_fingerprint_sha256(uint8_t pkid, uint8_t npkt, const std::vector<uint8_t>& compressed_key) const;
    std::vector<uint8_t> get_merkle_tree_leaves(const DSM_PKR_message& dsm_pkr_message) const;
    std::vector<uint8_t> compute_merkle_root(const DSM_PKR_message& dsm_pkr_message, const std::vector<uint8_t>& m_i) const;
    std::vector<uint8_t> compute_merkle_root(const DSM_PKR_message& dsm_pkr_message, const std::vector<uint8_t>& m_i, const std::string& hash_function) const;
    std::vector<uint8_t> build_message(Tag& tag) const;
    std::vector<uint8_t> hash_chain(uint32_t num_of_hashes_needed, const std::vector<uint8_t>& key, uint32_t GST_SFi, const uint8_t lk_bytes) const;
    std::vector<MACK_tag_and_info> verify_macseq(const MACK_message& mack);
    struct PartialTeslaKey
    {
        std::vector<uint8_t> bytes;
        std::vector<uint8_t> received;
    };
    struct VerifiedTeslaKey
    {
        VerifiedTeslaKey() = default;
        explicit VerifiedTeslaKey(std::vector<uint8_t> key_bytes) : key(std::move(key_bytes)) {}
        explicit VerifiedTeslaKey(std::initializer_list<uint8_t> key_bytes) : key(key_bytes) {}
        VerifiedTeslaKey(std::vector<uint8_t> key_bytes, std::vector<uint8_t> adkds) : key(std::move(key_bytes)), allowed_adkds(std::move(adkds)) {}

        bool operator==(const std::vector<uint8_t>& key_bytes) const { return key == key_bytes; }
        VerifiedTeslaKey& operator=(const std::vector<uint8_t>& key_bytes)
        {
            key = key_bytes;
            allowed_adkds = {0, 4, 12};
            return *this;
        }
        VerifiedTeslaKey& operator=(std::initializer_list<uint8_t> key_bytes)
        {
            key = key_bytes;
            allowed_adkds = {0, 4, 12};
            return *this;
        }

        std::vector<uint8_t> key;
        std::vector<uint8_t> allowed_adkds{0, 4, 12};
    };
    struct DeferredMackBlock
    {
        std::array<uint32_t, 15> mack{};
        std::array<uint8_t, 15> page_valid{};
        std::vector<uint8_t> allowed_adkds{0, 4, 12};
        uint32_t PRN{};
        uint32_t WN_sf0{};
        uint32_t TOW_sf0{};
        uint8_t nmas{};
        bool page_validity_available{false};
    };
    struct CachedDsmKroot
    {
        std::vector<uint8_t> dsm;
        std::string public_key_fingerprint_sha256;
        std::string public_key_type;
        std::string raw_sha256;
        std::string kroot_sha256;
        uint32_t signature_verified_at_gst{0};
        uint32_t gst0{0};
        uint32_t kroot_gst{0};
        uint16_t wn_k{0};
        uint8_t nma_header{0};
        uint8_t pkid{0};
        uint8_t cidkr{0};
        uint8_t nmas{0};
        uint8_t cpks{0};
        uint8_t hf{0};
        uint8_t mf{0};
        uint8_t ks{0};
        uint8_t ts{0};
        uint8_t maclt{0};
        uint8_t towh_k{0};
        uint8_t public_key_npkt{0};
        bool valid{false};
        bool metadata_valid{false};
        bool public_key_npkt_valid{false};
    };

    std::map<uint32_t, VerifiedTeslaKey> d_tesla_keys;  // TESLA keys over time, sorted by GST
    std::map<uint32_t, PartialTeslaKey> d_partial_tesla_keys;
    std::multimap<uint32_t, Tag> d_tags_awaiting_verify;  // container with tags to verify from arbitrary SVIDs, sorted by TOW

    std::vector<uint8_t> d_new_public_key;
    std::vector<uint8_t> d_merkle_root_at_renewal_start;
    std::vector<uint8_t> d_tags_to_verify{0, 4, 12};
    std::vector<MACK_message> d_macks_awaiting_MACSEQ_verification;
    std::vector<DeferredMackBlock> d_mack_blocks_awaiting_kroot;
    std::string d_merkle_file_path;
    std::string d_merkle_hash_function_at_renewal_start;

    CachedDsmKroot d_pending_dsm_kroot_cache;

    std::array<std::array<uint8_t, 256>, 16> d_dsm_message{};  // structure for recording DSM blocks, when filled it sends them to parse and resets itself.
    std::array<std::array<uint8_t, 16>, 16> d_dsm_id_received{};
    std::array<uint16_t, 16> d_number_of_blocks{};
    std::array<uint8_t, 16> d_dsm_nma_header{};
    std::array<std::array<uint8_t, 16>, 16> d_dsm_block_nma_header{};
    std::array<uint32_t, 16> d_dsm_first_gst{};
    std::array<bool, 16> d_dsm_first_gst_valid{};
    std::array<uint8_t, 60> d_mack_message{};  // C: 480 b
    std::array<uint8_t, 15> d_mack_page_received{};
    bool d_mack_page_validity_available{false};

    std::unique_ptr<Gnss_Crypto> d_crypto;  // class for cryptographic functions
    std::unique_ptr<Osnma_Crypto_Material_Manager> d_material_manager;
    std::unique_ptr<OSNMA_DSM_Reader> d_dsm_reader;            // osnma parameters parser
    std::unique_ptr<Osnma_Helper> d_helper;                    // helper class with auxiliary functions
    std::unique_ptr<OSNMA_NavDataManager> d_nav_data_manager;  // refactor for holding and processing navigation data

    OSNMA_data d_osnma_data{};

    uint32_t d_last_received_GST{0};      // latest GST received
    uint32_t d_GST_Sf{};                  // Scaled GST time for cryptographic computations
    uint32_t d_GST_Rx{0};                 // local GST receiver time
    uint32_t d_last_verified_key_GST{0};  // GST for the latest verified TESLA key
    uint32_t d_last_verified_kroot_GST{0};
    uint32_t d_GST_0{};    // Time of applicability GST (KROOT + 30 s)
    uint32_t d_GST_SIS{};  // GST coming from W6 and W5 of SIS
    uint32_t d_GST_PKR_PKREV_start{};
    uint32_t d_GST_PKR_AM_start{};
    uint32_t d_GST_chain_renewal_start{};
    uint32_t d_GST_chain_revocation_start{};
    uint32_t d_GST_merkle_tree_renewal_start{};

    uint32_t d_count_successful_tags{0};
    uint32_t d_count_failed_tags{0};
    uint32_t d_count_failed_Kroot{0};
    uint32_t d_count_failed_pubKey{0};  // failed public key verifications against Merkle root
    uint32_t d_count_failed_macseq{0};

    uint8_t const d_T_L{30};  // s RG Section 2.1
    uint8_t d_new_public_key_id{};
    uint8_t d_active_public_key_id{};

    bool d_public_key_verified{false};
    bool d_kroot_verified{false};
    bool d_tesla_key_verified{false};
    bool d_strict_mode{false};
    bool d_replay_mode{false};
    bool d_flag_hot_start{false};
    bool d_flag_PK_renewal{false};
    bool d_flag_PK_revocation{false};
    bool d_flag_NPK_set{false};
    bool d_flag_alert_message{false};
    bool d_flag_alert_message_verified{false};
    bool d_flag_chain_renewal{false};
    bool d_flag_chain_revocation{false};
    bool d_flag_merkle_tree_renewal{false};
    bool d_receiver_time_override{false};
    bool d_active_public_key_id_valid{false};
    bool d_new_merkle_tree_loaded{false};
    bool d_time_constraint_verified{false};
    bool d_kroot_loaded_from_cache{false};

    // Provide access to inner functions to Gtest
    FRIEND_TEST(OsnmaMsgReceiverTest, TeslaKeyVerification);
    FRIEND_TEST(OsnmaMsgReceiverTest, TagVerification);
    FRIEND_TEST(OsnmaMsgReceiverTest, TagVerification20Bit);
    FRIEND_TEST(OsnmaMsgReceiverTest, TimeConstraintAllowsOnlySlowMac);
    FRIEND_TEST(OsnmaMsgReceiverTest, TimeConstraintDecisionIsStoredWithTag);
    FRIEND_TEST(OsnmaMsgReceiverTest, ReplayModeSkipsReceiverTimeConstraint);
    FRIEND_TEST(OsnmaMsgReceiverTest, TeslaKeyRetainsTimeConstraintDecision);
    FRIEND_TEST(OsnmaMsgReceiverTest, TeslaChainResetCanPreserveOrClearDeferredMackQueue);
    FRIEND_TEST(OsnmaMsgReceiverTest, PruneOldTeslaKeysDropsExpiredDisclosureKeys);
    FRIEND_TEST(OsnmaMsgReceiverTest, TimeConstraintUsesReceiverGuidelineBounds);
    FRIEND_TEST(OsnmaMsgReceiverTest, UnverifiedTransitionHeadersDoNotStartTransitions);
    FRIEND_TEST(OsnmaMsgReceiverTest, UnverifiedDontUseHeaderDoesNotSkipMackProcessing);
    FRIEND_TEST(OsnmaMsgReceiverTest, VerifiedDontUseTagStopsNavigationAuthentication);
    FRIEND_TEST(OsnmaMsgReceiverTest, AuthenticatedDontUseClearsDsmAndNavAccumulation);
    FRIEND_TEST(OsnmaMsgReceiverTest, StaleKrootStatusSkipsMackProcessing);
    FRIEND_TEST(OsnmaMsgReceiverTest, UnverifiedRevocationHeadersDoNotClearState);
    FRIEND_TEST(OsnmaMsgReceiverTest, AuthenticatedCrevPreservesVerifiedFutureKroot);
    FRIEND_TEST(OsnmaMsgReceiverTest, AuthenticatedRevocationPromotesPreservedFutureKrootAtApplicability);
    FRIEND_TEST(OsnmaMsgReceiverTest, AuthenticatedPkrevPreservesVerifiedFutureKrootAndCurrentPublicKey);
    FRIEND_TEST(OsnmaMsgReceiverTest, UnverifiedNominalHeaderDoesNotResetTransitionLatches);
    FRIEND_TEST(OsnmaMsgReceiverTest, DsmKrootAccumulatorExpiresAfterOneHour);
    FRIEND_TEST(OsnmaMsgReceiverTest, DsmPkrAccumulatorExpiresAfterThirteenHours);
    FRIEND_TEST(OsnmaMsgReceiverTest, KrootApplicabilitySkipsMackBeforeGst0);
    FRIEND_TEST(OsnmaMsgReceiverTest, KrootApplicabilityAcceptsTowhkZero);
    FRIEND_TEST(OsnmaMsgReceiverTest, AllZeroPageValidityMaskMeansNoMackPagesAvailable);
    FRIEND_TEST(OsnmaMsgReceiverTest, MackBlockWaitsForVerifiedKroot);
    FRIEND_TEST(OsnmaMsgReceiverTest, FailedTeslaKeyDoesNotReturnPersistentSuccess);
    FRIEND_TEST(OsnmaMsgReceiverTest, OutOfOrderTeslaKeyRejected);
    FRIEND_TEST(OsnmaMsgReceiverTest, TeslaKeyLookupUsesWeek);
    FRIEND_TEST(OsnmaMsgReceiverTest, FailedDsmPkrKeepsCurrentPublicKey);
    FRIEND_TEST(OsnmaMsgReceiverTest, LowerPkidDsmPkrIsRejectedWhilePublicKeyInForce);
    FRIEND_TEST(OsnmaMsgReceiverTest, VerifiedDsmPkrSetsActiveKeyIdOnColdStart);
    FRIEND_TEST(OsnmaMsgReceiverTest, VerifiedDsmPkrUpdatesPendingKeyIdAcrossRollover);
    FRIEND_TEST(OsnmaMsgReceiverTest, MalformedDsmPkrRejectedBeforeLengthCopies);
    FRIEND_TEST(OsnmaMsgReceiverTest, KrootWithMismatchedActivePkidIsRejected);
    FRIEND_TEST(OsnmaMsgReceiverTest, FailedKrootWithMatchingPkidKeepsActivePublicKey);
    FRIEND_TEST(OsnmaMsgReceiverTest, DsmKrootPdkUsesSha256WhenHfSha3);
    FRIEND_TEST(OsnmaMsgReceiverTest, VerifiedAlertMessageClearsMerkleMaterial);
    FRIEND_TEST(OsnmaMsgReceiverTest, DsmBlockZeroResetsStaleAccumulator);
    FRIEND_TEST(OsnmaMsgReceiverTest, DsmBlockZeroKeepsCompatibleUnanchoredBlocks);
    FRIEND_TEST(OsnmaMsgReceiverTest, DsmBlockZeroDropsIncompatibleUnanchoredBlocks);
    FRIEND_TEST(OsnmaMsgReceiverTest, AnchoredDsmBlockOutsideAnnouncedLengthIsIgnored);
    FRIEND_TEST(OsnmaMsgReceiverTest, DsmNonzeroBlockWithDifferentNmaHeaderResetsAnchoredAccumulator);
    FRIEND_TEST(OsnmaMsgReceiverTest, RepeatedDsmBlockZeroKeepsPartialAccumulator);
    FRIEND_TEST(OsnmaMsgReceiverTest, DsmMessageUsesSavedNmaHeaderForChainRouting);
    FRIEND_TEST(OsnmaMsgReceiverTest, FailedKrootDoesNotOverwriteActiveState);
    FRIEND_TEST(OsnmaMsgReceiverTest, ChainRenewalDoesNotPromoteUnverifiedKroot);
    FRIEND_TEST(OsnmaMsgReceiverTest, ChainRenewalPromotesVerifiedFutureKrootAtApplicability);
    FRIEND_TEST(OsnmaMsgReceiverTest, UnverifiedNominalHeaderDoesNotPromoteVerifiedKroot);
    FRIEND_TEST(OsnmaMsgReceiverTest, MalformedKrootRejectedBeforeLengthCopies);
    FRIEND_TEST(OsnmaMsgReceiverTest, TeslaKeyVerificationAcrossWeekRollover);
    FRIEND_TEST(OsnmaMsgReceiverTest, PendingMackQueueExpiresOldEntries);
    FRIEND_TEST(OsnmaMsgReceiverTest, FastTagsWaitForNavDataUntilVerificationWindowExpires);
    FRIEND_TEST(OsnmaMsgReceiverTest, GstDeltaSecondsIsSigned);
    FRIEND_TEST(OsnmaMsgReceiverTest, UnverifiedNominalHeaderKeepsPublicKeyRenewalPending);
    FRIEND_TEST(OsnmaMsgReceiverTest, BuildTagMessageM0);
    FRIEND_TEST(OsnmaMsgReceiverTest, NavDataArrivalRetriesPendingTagsImmediately);
    FRIEND_TEST(OsnmaMsgReceiverTest, MacseqVerificationUsesMackTime);
    FRIEND_TEST(OsnmaMsgReceiverTest, MacseqVerificationMissingMacseqKeepsFixedTagsOnly);
    FRIEND_TEST(OsnmaMsgReceiverTest, MacseqVerificationChecksFixedSelfAndCrossSlots);
    FRIEND_TEST(OsnmaMsgReceiverTest, MacseqVerificationDiscardsReservedFlexibleTags);
    FRIEND_TEST(OsnmaMsgReceiverTest, VerifyPublicKey);
    FRIEND_TEST(OsnmaMsgReceiverTest, ComputeBaseLeaf);
    FRIEND_TEST(OsnmaMsgReceiverTest, CanonicalPublicKeyFingerprintIncludesPkidNpktAndPoint);
    FRIEND_TEST(OsnmaMsgReceiverTest, ComputeMerkleRoot);
    FRIEND_TEST(OsnmaMsgReceiverTest, ComputeMerkleRootUsesConfiguredHashFunction);
    FRIEND_TEST(OsnmaMsgReceiverTest, DummyCopZeroTagUsesZeroNavigationData);
    FRIEND_TEST(OsnmaMsgReceiverTest, NmtDsmPkrWaitsForNewMerkleTree);
    FRIEND_TEST(OsnmaMsgReceiverTest, MerkleXmlDoesNotChangeActivePublicKeyType);
    FRIEND_TEST(OsnmaMsgReceiverTest, PublicKeyMetadataRestoresPkidForPemHotStart);
    FRIEND_TEST(OsnmaMsgReceiverTest, PublicKeyMetadataRejectsMismatchedPemFingerprint);
    FRIEND_TEST(OsnmaMsgReceiverTest, NmtDsmPkrUsesCandidateMerkleTree);
    FRIEND_TEST(OsnmaMsgReceiverTest, UnsupportedKrootParametersAreRejected);
    FRIEND_TEST(OsnmaMsgReceiverTest, UnsupportedKrootParametersSkipMackProcessing);
    FRIEND_TEST(OsnmaMsgReceiverTest, MackWithMissingTag0StillParsesMackBody);
    FRIEND_TEST(OsnmaMsgReceiverTest, PartialMackKeepsAvailableFixedTags);
    FRIEND_TEST(OsnmaMsgReceiverTest, PartialTeslaKeyReconstructedAcrossSatellites);
    FRIEND_TEST(OsnmaMsgReceiverTest, CachedKrootWithoutMetadataIsNotPromotedAtStartup);
    FRIEND_TEST(OsnmaMsgReceiverTest, CachedKrootMetadataStoresFreshnessAndApplicability);
    FRIEND_TEST(OsnmaMsgReceiverTest, StaleCachedKrootMetadataIsRejected);
    friend class OsnmaTestVectors;
    FRIEND_TEST(OsnmaTestVectors, NominalTestConf1);
    FRIEND_TEST(OsnmaTestVectors, NominalTestConf2);
    FRIEND_TEST(OsnmaTestVectors, PublicKeyRenewal);
    FRIEND_TEST(OsnmaTestVectors, PublicKeyRevocation);
    FRIEND_TEST(OsnmaTestVectors, ChainRenewal);
    FRIEND_TEST(OsnmaTestVectors, ChainRevocation);
    FRIEND_TEST(OsnmaTestVectors, AlertMessage);
};


/** \} */
/** \} */
#endif  // GNSS_SDR_OSNMA_MSG_RECEIVER_H
