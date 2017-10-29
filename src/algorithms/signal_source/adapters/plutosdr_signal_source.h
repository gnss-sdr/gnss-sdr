
#ifndef GNSS_SDR_PLUTOSDR_SIGNAL_SOURCE_H_
#define GNSS_SDR_PLUTOSDR_SIGNAL_SOURCE_H_

#include <string>
#include <boost/shared_ptr.hpp>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/file_sink.h>
#include <iio/pluto_source.h>
#include "gnss_block_interface.h"

class ConfigurationInterface;

/*!
 */
class PlutosdrSignalSource: public GNSSBlockInterface
{
public:
    PlutosdrSignalSource(ConfigurationInterface* configuration,
            std::string role, unsigned int in_stream,
            unsigned int out_stream, boost::shared_ptr<gr::msg_queue> queue);

    virtual ~PlutosdrSignalSource();

    std::string role()
    {
        return role_;
    }

    /*!
     * \brief Returns "Plutosdr_Signal_Source"
     */
    std::string implementation()
    {
        return "Plutosdr_Signal_Source";
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

    // Front-end settings
	std::string uri_;//device direction
	unsigned long freq_; //frequency of local oscilator
	unsigned long sample_rate_;
	unsigned long bandwidth_;
	unsigned long buffer_size_; //reception buffer
	unsigned int decimation_;
	bool quadrature_;
	bool rf_dc_;
	bool bb_dc_;
	std::string gain_mode_;
	double rf_gain_;
	std::string filter_file_;
	bool filter_auto_;

    unsigned int in_stream_;
    unsigned int out_stream_;

    std::string item_type_;
    size_t item_size_;
    long samples_;
    bool dump_;
    std::string dump_filename_;

	gr::iio::pluto_source::sptr plutosdr_source_;

    boost::shared_ptr<gr::block> valve_;
    gr::blocks::file_sink::sptr file_sink_;
    boost::shared_ptr<gr::msg_queue> queue_;
};

#endif /*GNSS_SDR_PLUTOSDR_SIGNAL_SOURCE_H_*/
