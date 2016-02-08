#ifndef SIGNAL_GENERATOR_TEST_BEIDOU_H
#define SIGNAL_GENERATOR_TEST_BEIDOU_H

#include <string>
#include <vector>
#include <gnuradio/blocks/file_sink.h>
#include <gnuradio/hier_block2.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/vector_to_stream.h>
#include "gnss_block_interface.h"
#include "signal_generator_test_beidou_c.h"

class ConfigurationInterface;

/*!
* \brief This class generates synthesized GNSS signal.
*
*/
class SignalGenerator: public GNSSBlockInterface
{
public:
    SignalGenerator(ConfigurationInterface* configuration,
            std::string role, unsigned int in_stream,
            unsigned int out_stream, boost::shared_ptr<gr::msg_queue> queue);

    virtual ~SignalGenerator();
    std::string role()
    {
        return role_;
    }

    /*!
* \brief Returns "GNSSSignalGenerator".
*/
    std::string implementation()
    {
        return "GNSSSignalGenerator";
    }
    size_t item_size()
    {
        return item_size_;
    }

    void connect(gr::top_block_sptr top_block);
    void disconnect(gr::top_block_sptr top_block);
    gr::basic_block_sptr get_left_block();
    gr::basic_block_sptr get_right_block();

private:
    std::string role_;
    unsigned int in_stream_;
    unsigned int out_stream_;
    std::string item_type_;
    size_t item_size_;
    bool dump_;
    std::string dump_filename_;
    boost::shared_ptr<gr::block> gen_source_;
    gr::blocks::vector_to_stream::sptr vector_to_stream_;
    gr::blocks::file_sink::sptr file_sink_;
    boost::shared_ptr<gr::msg_queue> queue_;
};



#endif // SIGNAL_GENERATOR_TEST_BEIDOU_H
