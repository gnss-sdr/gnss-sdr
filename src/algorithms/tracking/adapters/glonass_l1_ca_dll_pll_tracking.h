#ifndef GNSS_SDR_GLONASS_L1_CA_DLL_PLL_TRACKING_H_
#define GNSS_SDR_GLONASS_L1_CA_DLL_PLL_TRACKING_H_

#include <string>
#include "tracking_interface.h"
#include "glonass_l1_ca_dll_pll_tracking_cc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 */
class GlonassL1CaDllPllTracking : public TrackingInterface
{
public:
    GlonassL1CaDllPllTracking(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams);

    virtual ~GlonassL1CaDllPllTracking();

    std::string role()
    {
        return role_;
    }

    //! Returns "GLONASS_L1_CA_DLL_PLL_Tracking"
    std::string implementation()
    {
        return "GLONASS_L1_CA_DLL_PLL_Tracking";
    }

    size_t item_size()
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();

    /*!
     * \brief Set tracking channel unique ID
     */
    void set_channel(unsigned int channel);

    /*!
     * \brief Set acquisition/tracking common Gnss_Synchro object pointer
     * to efficiently exchange synchronization data between acquisition and tracking blocks
     */
    void set_gnss_synchro(Gnss_Synchro* p_gnss_synchro);

    void start_tracking();

private:
    glonass_l1_ca_dll_pll_tracking_cc_sptr tracking_;
    size_t item_size_;
    unsigned int channel_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif // GNSS_SDR_GLONASS_L1_CA_DLL_PLL_TRACKING_H_
