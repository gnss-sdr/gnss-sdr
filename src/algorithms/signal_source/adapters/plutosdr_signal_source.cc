#include "plutosdr_signal_source.h"
#include <iostream>
#include <boost/format.hpp>
#include <glog/logging.h>
#include <gnuradio/blocks/file_sink.h>
#include "configuration_interface.h"
#include "gnss_sdr_valve.h"
#include "GPS_L1_CA.h"


using google::LogMessage;


PlutosdrSignalSource::PlutosdrSignalSource(ConfigurationInterface* configuration,
        std::string role, unsigned int in_stream, unsigned int out_stream,
        boost::shared_ptr<gr::msg_queue> queue) :
                role_(role), in_stream_(in_stream), out_stream_(out_stream),
                queue_(queue)
{
	std::string default_item_type="gr_complex";
	std::string default_dump_file="./data/signal_source.dat";
	uri_ = configuration->property(role+".device_address",std::string("192.168.2.1"));
	freq_= configuration->property(role+".freq",GPS_L1_FREQ_HZ);
	sample_rate_=configuration->property(role+".sampling_frequency",3000000);
	bandwidth_ = configuration->property(role+".bandwidth",2000000);
	buffer_size_=configuration->property(role+".buffer_size",0xA0000);
	decimation_=configuration->property(role+".decimation",1);
	quadrature_=configuration->property(role+".quadrature",true);
	rf_dc_ =configuration->property(role+".rf_dc",true);
	bb_dc_ =configuration->property(role+".bb_dc",true);
	gain_mode_=configuration->property(role+".gain_mode",std::string("manual"));
	rf_gain_=configuration->property(role+".gain",50.0);
	filter_file_=configuration->property(role+".filter_file",std::string(""));
	filter_auto_=configuration->property(role+".filter_auto",true);

	item_type_=configuration->property(role+".item_type",default_item_type);
	samples_=configuration->property(role+".samples",0);
	dump_=configuration->property(role+".dump",false);
	dump_filename_=configuration->property(role+".dump_filename",default_dump_file);
	
	if(item_type_.compare("gr_complex") != 0)
	{
		std::cout<<"bad item_type!!"<<std::endl;
		LOG(FATAL) <<"Exception: item type must be gr_complex!";
	}
	
	item_size_=sizeof(gr_complex);


	std::cout<<"device address: "<<uri_<<std::endl;
	std::cout<<"frequency : "<<freq_<<"Hz"<<std::endl;
	std::cout<<"sample rate: "<<sample_rate_<<"Hz"<<std::endl;
	std::cout<<"gain mode: "<<gain_mode_<<std::endl;
	std::cout<<"item type: "<<item_type_<<std::endl;

	plutosdr_source_=gr::iio::pluto_source::make(uri_, freq_, sample_rate_,
		 decimation_, bandwidth_, buffer_size_, quadrature_, rf_dc_, bb_dc_, 
		gain_mode_.c_str(), rf_gain_,filter_file_.c_str(), filter_auto_);

	if (samples_ != 0)
    {
    	DLOG(INFO) << "Send STOP signal after " << samples_ << " samples";
        valve_ = gnss_sdr_make_valve(item_size_, samples_, queue_);
        DLOG(INFO) << "valve(" << valve_->unique_id() << ")";
    }

    if (dump_)
    {
    	DLOG(INFO) << "Dumping output into file " << dump_filename_;
        file_sink_ = gr::blocks::file_sink::make(item_size_, dump_filename_.c_str());
         DLOG(INFO) << "file_sink(" << file_sink_->unique_id() << ")";
        }
}

PlutosdrSignalSource::~PlutosdrSignalSource()
{}

void PlutosdrSignalSource::connect(gr::top_block_sptr top_block)
{
    if (samples_ != 0)
        {
            top_block->connect(plutosdr_source_, 0, valve_, 0);
            DLOG(INFO) << "connected plutosdr source to valve";
            if (dump_)
                {
                    top_block->connect(valve_, 0, file_sink_, 0);
                    DLOG(INFO) << "connected valve to file sink";
                }
        }
    else
        {
            if (dump_)
                {
                    top_block->connect(plutosdr_source_, 0, file_sink_, 0);
                    DLOG(INFO) << "connected plutosdr source to file sink";
                }
        }
}


void PlutosdrSignalSource::disconnect(gr::top_block_sptr top_block)
{
    if (samples_ != 0)
        {
            top_block->disconnect(plutosdr_source_, 0, valve_, 0);
            if (dump_)
                {
                    top_block->disconnect(valve_, 0, file_sink_, 0);
                }
        }
    else
        {
            if (dump_)
                {
                    top_block->disconnect(plutosdr_source_, 0, file_sink_, 0);
                }
        }
}


gr::basic_block_sptr PlutosdrSignalSource::get_left_block()
{
    LOG(WARNING) << "Trying to get signal source left block.";
    return gr::basic_block_sptr();
}


gr::basic_block_sptr PlutosdrSignalSource::get_right_block()
{
    if (samples_ != 0)
        {
            return valve_;
        }
    else
        {
            return plutosdr_source_;
        }
}
