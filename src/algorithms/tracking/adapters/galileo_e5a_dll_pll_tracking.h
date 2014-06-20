/*
 * galileo_e5a_dll_pll_tracking.h
 *
 *  Created on: Jun 19, 2014
 *      Author: marc
 */

#ifndef GNSS_SDR_GALILEO_E5A_DLL_PLL_TRACKING_H_
#define GNSS_SDR_GALILEO_E5A_DLL_PLL_TRACKING_H_

#include <string>
#include <gnuradio/msg_queue.h>
#include "tracking_interface.h"
#include "galileo_e5a_dll_pll_tracking_cc.h"


class ConfigurationInterface;

/*!
 * \brief This class implements a code DLL + carrier PLL tracking loop
 */
class GalileoE5aDllPllTracking : public TrackingInterface
{
public:

    GalileoE5aDllPllTracking(ConfigurationInterface* configuration,
            std::string role,
            unsigned int in_streams,
            unsigned int out_streams,
            boost::shared_ptr<gr::msg_queue> queue);

    virtual ~GalileoE5aDllPllTracking();

    std::string role()
    {
        return role_;
    }

    //! Returns "Galileo_E5a_DLL_PLL_Tracking"
    std::string implementation()
    {
        return "Galileo_E5a_DLL_PLL_Tracking";
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

    /*!
     * \brief Set tracking channel internal queue
     */
    void set_channel_queue(concurrent_queue<int> *channel_internal_queue);

    void start_tracking();

private:
    galileo_e5a_dll_pll_tracking_cc_sptr tracking_;
    size_t item_size_;
    unsigned int channel_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
    boost::shared_ptr<gr::msg_queue> queue_;
    concurrent_queue<int> *channel_internal_queue_;
};

#endif /* GNSS_SDR_GALILEO_E5A_DLL_PLL_TRACKING_H_ */
