#ifndef GNSS_SDR_GLONASS_L1_CA_DLL_PLL_C_AID_TRACKING_H_
#define GNSS_SDR_GLONASS_L1_CA_DLL_PLL_C_AID_TRACKING_H_

#include <string>
#include "tracking_interface.h"
#include "glonass_l1_ca_dll_pll_c_aid_tracking_cc.h"
#include "glonass_l1_ca_dll_pll_c_aid_tracking_sc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 */
class GlonassL1CaDllPllCAidTracking : public TrackingInterface
{
public:
  GlonassL1CaDllPllCAidTracking(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams);

    virtual ~GlonassL1CaDllPllCAidTracking();

    inline std::string role() override
    {
        return role_;
    }

    //! Returns "GLONASS_L1_CA_DLL_PLL_C_Aid_Tracking"
    inline std::string implementation() override
    {
        return "GLONASS_L1_CA_DLL_PLL_C_Aid_Tracking";
    }

    inline size_t item_size() override
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;

    /*!
     * \brief Set tracking channel unique ID
     */
    void set_channel(unsigned int channel) override;

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro) override;

    void start_tracking() override;

private:
    glonass_l1_ca_dll_pll_c_aid_tracking_cc_sptr tracking_cc;
    glonass_l1_ca_dll_pll_c_aid_tracking_sc_sptr tracking_sc;
    size_t item_size_;
    std::string item_type_;
    unsigned int channel_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif // GNSS_SDR_GLONASS_L1_CA_DLL_PLL_C_AID_TRACKING_H_
