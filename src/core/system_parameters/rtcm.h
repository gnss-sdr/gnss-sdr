/*!
 * \file rtcm.h
 * \brief  Interface for the RTCM 3.2 Standard
 * \author Carles Fernandez-Prades, 2015. cfernandez(at)cttc.es
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_RTCM_H_
#define GNSS_SDR_RTCM_H_


#include <bitset>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <boost/asio.hpp>
#include <boost/crc.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "concurrent_queue.h"
#include "gnss_synchro.h"
#include "galileo_fnav_message.h"
#include "gps_navigation_message.h"
#include "gps_cnav_navigation_message.h"


/*!
 * This class implements the generation and reading of some Message Types
 * defined in the RTCM 3.2 Standard, plus some utilities to handle messages.
 *
 * Generation of the following Message Types:
 *   1001, 1002, 1003, 1004, 1005, 1019, 1045
 *
 * Decoding of the following Message Types:
 *   1019, 1045
 *
 * Generation of the following Multiple Signal Messages:
 *   MSM1 (message types 1071, 1091)
 *   MSM2 (message types 1072, 1092)
 *   MSM3 (message types 1073, 1093)
 *   MSM4 (message types 1074, 1094)
 *   MSM5 (message types 1075, 1095)
 *   MSM6 (message types 1076, 1096)
 *   MSM7 (message types 1077, 1097)
 *
 * RTCM 3 message format (size in bits):
 *   +----------+--------+-----------+--------------------+----------+
 *   | preamble | 000000 |  length   |    data message    |  parity  |
 *   +----------+--------+-----------+--------------------+----------+
 *   |<-- 8 --->|<- 6 -->|<-- 10 --->|<--- length x 8 --->|<-- 24 -->|
 *   +----------+--------+-----------+--------------------+----------+
 *
 *
 *   (C) Carles Fernandez-Prades, 2015. cfernandez(at)cttc.es
 */
class Rtcm
{
public:
    Rtcm(); //<! Default constructor
    ~Rtcm();

    /*!
     * \brief Prints message type 1001 (L1-Only GPS RTK Observables)
     */
    std::string print_MT1001(const Gps_Ephemeris& gps_eph, double obs_time, const std::map<int, Gnss_Synchro> & pseudoranges);

    /*!
     * \brief Prints message type 1002 (Extended L1-Only GPS RTK Observables)
     */
    std::string print_MT1002(const Gps_Ephemeris & gps_eph, double obs_time, const std::map<int, Gnss_Synchro> & pseudoranges);

    /*!
     * \brief Prints message type 1003 (L1 & L2 GPS RTK Observables)
     */
    std::string print_MT1003(const Gps_Ephemeris & ephL1, const Gps_CNAV_Ephemeris & ephL2, double obs_time, const std::map<int, Gnss_Synchro> & pseudoranges);

    /*!
     * \brief Prints message type 1004 (Extended L1 & L2 GPS RTK Observables)
     */
    std::string print_MT1004(const Gps_Ephemeris & ephL1, const Gps_CNAV_Ephemeris & ephL2, double obs_time, const std::map<int, Gnss_Synchro> & pseudoranges);

    /*!
     * \brief Prints message type 1005 (Stationary Antenna Reference Point)
     */
    std::string print_MT1005(unsigned int ref_id, double ecef_x, double ecef_y, double ecef_z, bool gps, bool glonass, bool galileo, bool non_physical, bool single_oscillator, unsigned int quarter_cycle_indicator);

    /*!
     * \brief Verifies and reads messages of type 1005 (Stationary Antenna Reference Point). Returns 1 if anything goes wrong, 0 otherwise.
     */
    int read_MT1005(const std::string & message, unsigned int & ref_id, double & ecef_x, double & ecef_y, double & ecef_z, bool & gps, bool & glonass, bool & galileo);

    /*!
     * \brief Prints message type 1019 (GPS Ephemeris), should be broadcast in the event that
     * the IODC does not match the IODE, and every 2 minutes.
     */
    std::string print_MT1019(const Gps_Ephemeris & gps_eph);

    /*!
     * \brief Verifies and reads messages of type 1019 (GPS Ephemeris). Returns 1 if anything goes wrong, 0 otherwise.
     */
    int read_MT1019(const std::string & message, Gps_Ephemeris & gps_eph);

    /*!
     * \brief Prints message type 1045 (Galileo Ephemeris), should be broadcast every 2 minutes
     */
    std::string print_MT1045(const Galileo_Ephemeris & gal_eph);

    /*!
     * \brief Verifies and reads messages of type 1045 (Galileo Ephemeris). Returns 1 if anything goes wrong, 0 otherwise.
     */
    int read_MT1045(const std::string & message, Galileo_Ephemeris & gal_eph);

    /*!
     * \brief Prints messages of type MSM1 (Compact GNSS pseudoranges)
     */
    std::string print_MSM_1( const Gps_Ephemeris & gps_eph,
            const Gps_CNAV_Ephemeris & gps_cnav_eph,
            const Galileo_Ephemeris & gal_eph,
            double obs_time,
            const std::map<int, Gnss_Synchro> & pseudoranges,
            unsigned int ref_id,
            unsigned int clock_steering_indicator,
            unsigned int external_clock_indicator,
            int smooth_int,
            bool sync_flag,
            bool divergence_free,
            bool more_messages);

    /*!
     * \brief Prints messages of type MSM2 (Compact GNSS phaseranges)
     */
    std::string print_MSM_2( const Gps_Ephemeris & gps_eph,
            const Gps_CNAV_Ephemeris & gps_cnav_eph,
            const Galileo_Ephemeris & gal_eph,
            double obs_time,
            const std::map<int, Gnss_Synchro> & pseudoranges,
            unsigned int ref_id,
            unsigned int clock_steering_indicator,
            unsigned int external_clock_indicator,
            int smooth_int,
            bool sync_flag,
            bool divergence_free,
            bool more_messages);

    /*!
     * \brief Prints messages of type MSM3 (Compact GNSS pseudoranges and phaseranges)
     */
    std::string print_MSM_3( const Gps_Ephemeris & gps_eph,
            const Gps_CNAV_Ephemeris & gps_cnav_eph,
            const Galileo_Ephemeris & gal_eph,
            double obs_time,
            const std::map<int, Gnss_Synchro> & pseudoranges,
            unsigned int ref_id,
            unsigned int clock_steering_indicator,
            unsigned int external_clock_indicator,
            int smooth_int,
            bool sync_flag,
            bool divergence_free,
            bool more_messages);

    /*!
     * \brief Prints messages of type MSM4 (Full GNSS pseudoranges and phaseranges plus CNR)
     */
    std::string print_MSM_4( const Gps_Ephemeris & gps_eph,
            const Gps_CNAV_Ephemeris & gps_cnav_eph,
            const Galileo_Ephemeris & gal_eph,
            double obs_time,
            const std::map<int, Gnss_Synchro> & pseudoranges,
            unsigned int ref_id,
            unsigned int clock_steering_indicator,
            unsigned int external_clock_indicator,
            int smooth_int,
            bool sync_flag,
            bool divergence_free,
            bool more_messages);

    /*!
     * \brief Prints messages of type MSM5 (Full GNSS pseudoranges, phaseranges, phaserange rate and CNR)
     */
    std::string print_MSM_5( const Gps_Ephemeris & gps_eph,
            const Gps_CNAV_Ephemeris & gps_cnav_eph,
            const Galileo_Ephemeris & gal_eph,
            double obs_time,
            const std::map<int, Gnss_Synchro> & pseudoranges,
            unsigned int ref_id,
            unsigned int clock_steering_indicator,
            unsigned int external_clock_indicator,
            int smooth_int,
            bool sync_flag,
            bool divergence_free,
            bool more_messages);

    /*!
     * \brief Prints messages of type MSM6 (Full GNSS pseudoranges and phaseranges plus CNR, high resolution)
     */
    std::string print_MSM_6( const Gps_Ephemeris & gps_eph,
            const Gps_CNAV_Ephemeris & gps_cnav_eph,
            const Galileo_Ephemeris & gal_eph,
            double obs_time,
            const std::map<int, Gnss_Synchro> & pseudoranges,
            unsigned int ref_id,
            unsigned int clock_steering_indicator,
            unsigned int external_clock_indicator,
            int smooth_int,
            bool sync_flag,
            bool divergence_free,
            bool more_messages);

    /*!
     * \brief Prints messages of type MSM7 (Full GNSS pseudoranges, phaseranges, phaserange rate and CNR, high resolution)
     */
    std::string print_MSM_7( const Gps_Ephemeris & gps_eph,
            const Gps_CNAV_Ephemeris & gps_cnav_eph,
            const Galileo_Ephemeris & gal_eph,
            double obs_time,
            const std::map<int, Gnss_Synchro> & pseudoranges,
            unsigned int ref_id,
            unsigned int clock_steering_indicator,
            unsigned int external_clock_indicator,
            int smooth_int,
            bool sync_flag,
            bool divergence_free,
            bool more_messages);

    unsigned int lock_time(const Gps_Ephemeris & eph, double obs_time, const Gnss_Synchro & gnss_synchro); //<! Returns the time period in which GPS L1 signals have been continually tracked.
    unsigned int lock_time(const Gps_CNAV_Ephemeris & eph, double obs_time, const Gnss_Synchro & gnss_synchro); //<! Returns the time period in which GPS L2 signals have been continually tracked.
    unsigned int lock_time(const Galileo_Ephemeris & eph, double obs_time, const Gnss_Synchro & gnss_synchro); //<! Returns the time period in which Galileo signals have been continually tracked.

    std::string bin_to_hex(const std::string& s); //<! Returns a string of hexadecimal symbols from a string of binary symbols
    std::string hex_to_bin(const std::string& s); //<! Returns a string of binary symbols from a string of hexadecimal symbols

    unsigned long int bin_to_uint(const std::string& s); //<! Returns an unsigned long int from a string of binary symbols
    long int bin_to_int(const std::string& s);           //<! Returns a long int from a string of binary symbols

    unsigned long int hex_to_uint(const std::string& s); //<! Returns an unsigned long int from a string of hexadecimal symbols
    long int hex_to_int(const std::string& s);           //<! Returns a long int from a string of hexadecimal symbols

    double bin_to_double(const std::string& s);          //<! Returns double from a string of binary symbols
    std::string print_MT1005_test();                     //<! For testing purposes

    bool check_CRC(const std::string & message);         //<! Checks that the CRC of a RTCM package is correct

    void run_server();
    void stop_server();

    void run_client();
    void stop_client();

    void send_message(const std::string & message);

private:
    //
    // Generation of messages content
    //
    std::bitset<64> get_MT1001_4_header(unsigned int msg_number,
            const Gps_Ephemeris & gps_eph,
            double obs_time,
            const std::map<int, Gnss_Synchro> & pseudoranges,
            unsigned int ref_id,
            unsigned int smooth_int,
            bool sync_flag,
            bool divergence_free);

    std::bitset<58> get_MT1001_sat_content(const Gps_Ephemeris & eph, double obs_time, const Gnss_Synchro & gnss_synchro);
    std::bitset<74> get_MT1002_sat_content(const Gps_Ephemeris & eph, double obs_time, const Gnss_Synchro & gnss_synchro);
    std::bitset<101> get_MT1003_sat_content(const Gps_Ephemeris & ephL1, const Gps_CNAV_Ephemeris & ephL2, double obs_time, const Gnss_Synchro & gnss_synchroL1, const Gnss_Synchro & gnss_synchroL2);
    std::bitset<125> get_MT1004_sat_content(const Gps_Ephemeris & ephL1, const Gps_CNAV_Ephemeris & ephL2, double obs_time, const Gnss_Synchro & gnss_synchroL1, const Gnss_Synchro & gnss_synchroL2);

    std::bitset<152> get_MT1005_test();

    std::string get_MSM_header(unsigned int msg_number, const Gps_Ephemeris & gps_eph,
            const Gps_CNAV_Ephemeris & gps_cnav_eph,
            const Galileo_Ephemeris & gal_eph,
            double obs_time,
            const std::map<int, Gnss_Synchro> & pseudoranges,
            unsigned int ref_id,
            unsigned int clock_steering_indicator,
            unsigned int external_clock_indicator,
            int smooth_int,
            bool sync_flag,
            bool divergence_free,
            bool more_messages);

    std::string get_MSM_1_content_sat_data(const std::map<int, Gnss_Synchro> & pseudoranges);
    std::string get_MSM_4_content_sat_data(const std::map<int, Gnss_Synchro> & pseudoranges);
    std::string get_MSM_5_content_sat_data(const std::map<int, Gnss_Synchro> & pseudoranges);

    std::string get_MSM_1_content_signal_data(const std::map<int, Gnss_Synchro> & pseudoranges);
    std::string get_MSM_2_content_signal_data(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const std::map<int, Gnss_Synchro> & pseudoranges);
    std::string get_MSM_3_content_signal_data(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const std::map<int, Gnss_Synchro> & pseudoranges);
    std::string get_MSM_4_content_signal_data(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const std::map<int, Gnss_Synchro> & pseudoranges);
    std::string get_MSM_5_content_signal_data(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const std::map<int, Gnss_Synchro> & pseudoranges);
    std::string get_MSM_6_content_signal_data(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const std::map<int, Gnss_Synchro> & pseudoranges);
    std::string get_MSM_7_content_signal_data(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const std::map<int, Gnss_Synchro> & pseudoranges);

    //
    // Utilities
    //
    static std::map<std::string, int> galileo_signal_map;
    static std::map<std::string, int> gps_signal_map;
    std::vector<std::pair<int, Gnss_Synchro> > sort_by_signal(const std::vector<std::pair<int, Gnss_Synchro> >  & synchro_map);
    std::vector<std::pair<int, Gnss_Synchro> > sort_by_PRN_mask(const std::vector<std::pair<int, Gnss_Synchro> >  & synchro_map);
    boost::posix_time::ptime compute_GPS_time(const Gps_Ephemeris& eph, double obs_time);
    boost::posix_time::ptime compute_GPS_time(const Gps_CNAV_Ephemeris & eph, double obs_time);
    boost::posix_time::ptime compute_Galileo_time(const Galileo_Ephemeris& eph, double obs_time);
    boost::posix_time::ptime gps_L1_last_lock_time[64];
    boost::posix_time::ptime gps_L2_last_lock_time[64];
    boost::posix_time::ptime gal_E1_last_lock_time[64];
    boost::posix_time::ptime gal_E5_last_lock_time[64];
    unsigned int lock_time_indicator(unsigned int lock_time_period_s);
    unsigned int msm_lock_time_indicator(unsigned int lock_time_period_s);
    unsigned int msm_extended_lock_time_indicator(unsigned int lock_time_period_s);

    //
    // Classes for TCP communication
    //
    class Message_buffer
    {
    public:
        // Construct from a std::string.
        explicit Message_buffer(const std::string& data)
        : data_(new std::vector<char>(data.begin(), data.end())),
          buffer_(boost::asio::buffer(*data_))
        {
        }

        // Implement the ConstBufferSequence requirements.
        typedef boost::asio::const_buffer value_type;
        typedef const boost::asio::const_buffer* const_iterator;
        const boost::asio::const_buffer* begin() const { return &buffer_; }
        const boost::asio::const_buffer* end() const { return &buffer_ + 1; }

    private:
        std::shared_ptr<std::vector<char> > data_;
        boost::asio::const_buffer buffer_;
    };


    class Rtcm_listener
    {
    public:
        virtual ~Rtcm_listener() {}
        virtual void deliver(const Message_buffer & msg) = 0;
    };


    class Rtcm_session
            : public Rtcm_listener,
              public std::enable_shared_from_this<Rtcm_session>
    {
    public:
        Rtcm_session(boost::asio::ip::tcp::socket socket, std::shared_ptr< concurrent_queue<std::string> > & queue) : socket_(std::move(socket)) , queue_(queue)  { }

        void start()
        {
            std::cout << "Starting a session: " <<  std::endl;
            listeners_.insert(shared_from_this());
            do_send_messages();
        }

        void deliver(const Message_buffer & msg)
        {
            bool write_in_progress = !write_msgs_.empty();
            std::cout << "Pushing a message " <<  std::endl;
            write_msgs_.push_back(msg);
            if (!write_in_progress)
                {
                    do_write();
                }
        }

    private:
        void do_send_messages()
        {
            std::string message;
            queue_->wait_and_pop(message);
            Message_buffer buffer_out(message);
            for (auto listener: listeners_)
                {
                    listener->deliver(buffer_out);
                }
            do_send_messages();
        }

        void do_write()
        {
            auto self(shared_from_this());
            boost::asio::async_write(socket_, write_msgs_.front(), [this, self](boost::system::error_code ec, std::size_t /*length*/)
                    {
                if(!ec)
                    {
                        std::cout << "RTCM message sent." << std::endl;
                        write_msgs_.pop_front();
                        if(!write_msgs_.empty())
                            {
                                do_write();
                            }
                    }
                else
                    {
                        std::cout << "Erasing listener" << std::endl;
                        listeners_.erase(shared_from_this());
                    }
                    });
        }

        boost::asio::ip::tcp::socket socket_;
        std::shared_ptr< concurrent_queue<std::string> > & queue_;
        std::string read_msg_;
        std::deque<Message_buffer> write_msgs_;
        std::set< std::shared_ptr<Rtcm_listener> > listeners_;
    };


    class Tcp_server
    {
    public:
        Tcp_server(boost::asio::io_service& io_service, const boost::asio::ip::tcp::endpoint& endpoint, std::shared_ptr< concurrent_queue<std::string> >  & queue)
    : acceptor_(io_service), queue_(queue),
      socket_(io_service)
    {
            acceptor_.open(endpoint.protocol());
            acceptor_.set_option(boost::asio::ip::tcp::acceptor::reuse_address(true));
            acceptor_.bind(endpoint);
            acceptor_.listen();
            do_accept();
    }

    private:
        void do_accept()
        {
            acceptor_.async_accept(socket_, [this](boost::system::error_code ec)
                    {
                if (!ec)
                    {
                        std::cout << "Starting RTCM TCP server session..." << std::endl;
                        std::thread session([&]{ std::make_shared<Rtcm_session>(std::move(socket_), queue_)->start(); });
                        session.detach();
                        std::cout << "Accepting new connections " << std::endl;
                    }
                else
                    {
                        std::cout << "Error starting a new session: " << ec << std::endl;
                    }
                do_accept();
                    });
        }

        boost::asio::ip::tcp::acceptor acceptor_;
        boost::asio::ip::tcp::socket socket_;
        std::shared_ptr< concurrent_queue<std::string> >  & queue_;
    };


    class Tcp_client
    {
    public:
        Tcp_client(boost::asio::io_service& io_service,
                boost::asio::ip::tcp::resolver::iterator endpoint_iterator)
    : io_service_(io_service),
      socket_(io_service)
    {
            do_connect(endpoint_iterator);
    }

        void close()
        {
            io_service_.post([this]() { socket_.close(); });
        }

    private:
        void do_connect(boost::asio::ip::tcp::resolver::iterator endpoint_iterator)
        {
            std::cout << "Connecting to server..." << std::endl;
            boost::asio::async_connect(socket_, endpoint_iterator,
                    [this](boost::system::error_code ec, boost::asio::ip::tcp::resolver::iterator)
                    {
                if (!ec)
                    {
                        std::cout << "Connected." << std::endl;
                        do_read_message();
                    }
                else
                    {
                        std::cout << "Server is down." << std::endl;
                    }
                    });
        }

        void do_read_message()
        {
            // Waiting for data and reading forever, until connection is closed.
            for(;;)
                {
                    std::array<char, 10> buf;
                    boost::system::error_code error;

                    size_t len = socket_.read_some(boost::asio::buffer(buf), error);

                    if(error == boost::asio::error::eof)
                        break; // // Connection closed cleanly by peer.
                    else if(error)
                        {
                            std::cout << "Error: " << error << std::endl;
                            socket_.close();
                            break;
                        }
                    std::cout << "Received message: ";
                    std::cout.write(buf.data(), len);
                    std::cout << std::endl;
                }

            std::cout << std::endl;
            socket_.close();
            std::cout << "Connection closed by the server. Good bye." << std::endl;
        }

        boost::asio::io_service& io_service_;
        boost::asio::ip::tcp::socket socket_;
    };

    boost::asio::io_service io_service;
    std::shared_ptr< concurrent_queue<std::string> > rtcm_message_queue;
    std::thread t;
    std::list<Rtcm::Tcp_server> servers;
    std::list<Rtcm::Tcp_client> clients;
    bool server_is_running;
    void stop_service();

    //
    // Transport Layer
    //
    std::bitset<8> preamble;
    std::bitset<6> reserved_field;
    std::bitset<10> message_length;
    std::bitset<24> crc_frame;
    typedef boost::crc_optimal<24, 0x1864CFBu, 0x0, 0x0, false, false> crc_24_q_type;
    std::string add_CRC(const std::string& m);
    std::string build_message(std::string data); // adds 0s to complete a byte and adds the CRC

    //
    // Data Fields
    //
    int reset_data_fields();

    std::bitset<12> DF002;
    int set_DF002(unsigned int message_number);

    std::bitset<12> DF003;
    int set_DF003(unsigned int ref_station_ID);

    std::bitset<30> DF004;
    int set_DF004(const Gps_Ephemeris & gps_eph, double obs_time);
    int set_DF004(const Gps_CNAV_Ephemeris & gps_eph, double obs_time);

    std::bitset<1> DF005;
    int set_DF005(bool sync_flag);

    std::bitset<5> DF006;
    int set_DF006(const std::map<int, Gnss_Synchro> & pseudoranges);

    std::bitset<1> DF007;
    int set_DF007(bool divergence_free_smoothing_indicator); // 0 - Divergence-free smoothing not used 1 - Divergence-free smoothing used

    std::bitset<3> DF008;
    int set_DF008(short int smoothing_interval);

    std::bitset<6> DF009;
    int set_DF009(const Gnss_Synchro & gnss_synchro);
    int set_DF009(const Gps_Ephemeris & gps_eph);

    std::bitset<1> DF010;
    int set_DF010(bool code_indicator);

    std::bitset<24> DF011;
    int set_DF011(const Gnss_Synchro & gnss_synchro);

    std::bitset<20> DF012;
    int set_DF012(const Gnss_Synchro & gnss_synchro);

    std::bitset<7> DF013;
    int set_DF013(const Gps_Ephemeris & eph, double obs_time, const Gnss_Synchro & gnss_synchro);

    std::bitset<8> DF014;
    int set_DF014(const Gnss_Synchro & gnss_synchro);

    std::bitset<8> DF015;
    int set_DF015(const Gnss_Synchro & gnss_synchro);

    std::bitset<14> DF017;
    int set_DF017(const Gnss_Synchro & gnss_synchroL1, const Gnss_Synchro & gnss_synchroL2);

    std::bitset<20> DF018;
    int set_DF018(const Gnss_Synchro & gnss_synchroL1, const Gnss_Synchro & gnss_synchroL2);

    std::bitset<7> DF019;
    int set_DF019(const Gps_CNAV_Ephemeris & eph, double obs_time, const Gnss_Synchro & gnss_synchro);

    std::bitset<8> DF020;
    int set_DF020(const Gnss_Synchro & gnss_synchro);

    std::bitset<6> DF021;
    int set_DF021();

    std::bitset<1> DF022;
    int set_DF022(bool gps_indicator);

    std::bitset<1> DF023;
    int set_DF023(bool glonass_indicator);

    std::bitset<1> DF024;
    int set_DF024(bool galileo_indicator);

    std::bitset<38> DF025;
    int set_DF025(double antenna_ECEF_X_m);

    std::bitset<38> DF026;
    int set_DF026(double antenna_ECEF_Y_m);

    std::bitset<38> DF027;
    int set_DF027(double antenna_ECEF_Z_m);

    // Contents of GPS Satellite Ephemeris Data, Message Type 1019
    std::bitset<8> DF071;
    int set_DF071(const Gps_Ephemeris & gps_eph);

    std::bitset<10> DF076;
    int set_DF076(const Gps_Ephemeris & gps_eph);

    std::bitset<4> DF077;
    int set_DF077(const Gps_Ephemeris & gps_eph);

    std::bitset<2> DF078;
    int set_DF078(const Gps_Ephemeris & gps_eph);

    std::bitset<14> DF079;
    int set_DF079(const Gps_Ephemeris & gps_eph);

    std::bitset<8> DF080;
    int set_DF080(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF081;
    int set_DF081(const Gps_Ephemeris & gps_eph);

    std::bitset<8> DF082;
    int set_DF082(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF083;
    int set_DF083(const Gps_Ephemeris & gps_eph);

    std::bitset<22> DF084;
    int set_DF084(const Gps_Ephemeris & gps_eph);

    std::bitset<10> DF085;
    int set_DF085(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF086;
    int set_DF086(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF087;
    int set_DF087(const Gps_Ephemeris & gps_eph);

    std::bitset<32> DF088;
    int set_DF088(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF089;
    int set_DF089(const Gps_Ephemeris & gps_eph);

    std::bitset<32> DF090;
    int set_DF090(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF091;
    int set_DF091(const Gps_Ephemeris & gps_eph);

    std::bitset<32> DF092;
    int set_DF092(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF093;
    int set_DF093(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF094;
    int set_DF094(const Gps_Ephemeris & gps_eph);

    std::bitset<32> DF095;
    int set_DF095(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF096;
    int set_DF096(const Gps_Ephemeris & gps_eph);

    std::bitset<32> DF097;
    int set_DF097(const Gps_Ephemeris & gps_eph);

    std::bitset<16> DF098;
    int set_DF098(const Gps_Ephemeris & gps_eph);

    std::bitset<32> DF099;
    int set_DF099(const Gps_Ephemeris & gps_eph);

    std::bitset<24> DF100;
    int set_DF100(const Gps_Ephemeris & gps_eph);

    std::bitset<8> DF101;
    int set_DF101(const Gps_Ephemeris & gps_eph);

    std::bitset<6> DF102;
    int set_DF102(const Gps_Ephemeris & gps_eph);

    std::bitset<1> DF103;
    int set_DF103(const Gps_Ephemeris & gps_eph);

    std::bitset<1> DF137;
    int set_DF137(const Gps_Ephemeris & gps_eph);


    std::bitset<1> DF141;
    int set_DF141(const Gps_Ephemeris & gps_eph);

    std::bitset<1> DF142;
    int set_DF142(const Gps_Ephemeris & gps_eph);

    std::bitset<30> DF248;
    int set_DF248(const Galileo_Ephemeris & gal_eph, double obs_time);

    // Contents of Galileo F/NAV Satellite Ephemeris Data, Message Type 1045
    std::bitset<6> DF252;
    int set_DF252(const Galileo_Ephemeris & gal_eph);

    std::bitset<12> DF289;
    int set_DF289(const Galileo_Ephemeris & gal_eph);

    std::bitset<10> DF290;
    int set_DF290(const Galileo_Ephemeris & gal_eph);

    std::bitset<8> DF291;
    int set_DF291(const Galileo_Ephemeris & gal_eph);

    std::bitset<14> DF292;
    int set_DF292(const Galileo_Ephemeris & gal_eph);

    std::bitset<14> DF293;
    int set_DF293(const Galileo_Ephemeris & gal_eph);

    std::bitset<6> DF294;
    int set_DF294(const Galileo_Ephemeris & gal_eph);

    std::bitset<21> DF295;
    int set_DF295(const Galileo_Ephemeris & gal_eph);

    std::bitset<31> DF296;
    int set_DF296(const Galileo_Ephemeris & gal_eph);

    std::bitset<16> DF297;
    int set_DF297(const Galileo_Ephemeris & gal_eph);

    std::bitset<16> DF298;
    int set_DF298(const Galileo_Ephemeris & gal_eph);

    std::bitset<32> DF299;
    int set_DF299(const Galileo_Ephemeris & gal_eph);

    std::bitset<16> DF300;
    int set_DF300(const Galileo_Ephemeris & gal_eph);

    std::bitset<32> DF301;
    int set_DF301(const Galileo_Ephemeris & gal_eph);

    std::bitset<16> DF302;
    int set_DF302(const Galileo_Ephemeris & gal_eph);

    std::bitset<32> DF303;
    int set_DF303(const Galileo_Ephemeris & gal_eph);

    std::bitset<14> DF304;
    int set_DF304(const Galileo_Ephemeris & gal_eph);

    std::bitset<16> DF305;
    int set_DF305(const Galileo_Ephemeris & gal_eph);

    std::bitset<32> DF306;
    int set_DF306(const Galileo_Ephemeris & gal_eph);

    std::bitset<16> DF307;
    int set_DF307(const Galileo_Ephemeris & gal_eph);

    std::bitset<32> DF308;
    int set_DF308(const Galileo_Ephemeris & gal_eph);

    std::bitset<16> DF309;
    int set_DF309(const Galileo_Ephemeris & gal_eph);

    std::bitset<32> DF310;
    int set_DF310(const Galileo_Ephemeris & gal_eph);

    std::bitset<24> DF311;
    int set_DF311(const Galileo_Ephemeris & gal_eph);

    std::bitset<10> DF312;
    int set_DF312(const Galileo_Ephemeris & gal_eph);

    std::bitset<10> DF313;
    int set_DF313(const Galileo_Ephemeris & gal_eph);

    std::bitset<2> DF314;
    int set_DF314(const Galileo_Ephemeris & gal_eph);

    std::bitset<1> DF315;
    int set_DF315(const Galileo_Ephemeris & gal_eph);

    std::bitset<2> DF364;

    // Content of message header for MSM1, MSM2, MSM3, MSM4, MSM5, MSM6 and MSM7
    std::bitset<1> DF393;
    int set_DF393(bool more_messages); //1 indicates that more MSMs follow for given physical time and reference station ID

    std::bitset<64> DF394;
    int set_DF394(const std::map<int, Gnss_Synchro> & pseudoranges);

    std::bitset<32> DF395;
    int set_DF395(const std::map<int, Gnss_Synchro> & pseudoranges);

    std::string set_DF396(const std::map<int, Gnss_Synchro> & pseudoranges);

    std::bitset<8> DF397;
    int set_DF397(const Gnss_Synchro & gnss_synchro);

    std::bitset<10> DF398;
    int set_DF398(const Gnss_Synchro & gnss_synchro);

    std::bitset<14> DF399;
    int set_DF399(const Gnss_Synchro & gnss_synchro);

    std::bitset<15> DF400;
    int set_DF400(const Gnss_Synchro & gnss_synchro);

    std::bitset<22> DF401;
    int set_DF401(const Gnss_Synchro & gnss_synchro);

    std::bitset<4> DF402;
    int set_DF402(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const Gnss_Synchro & gnss_synchro);

    std::bitset<6> DF403;
    int set_DF403(const Gnss_Synchro & gnss_synchro);

    std::bitset<15> DF404;
    int set_DF404(const Gnss_Synchro & gnss_synchro);

    std::bitset<20> DF405;
    int set_DF405(const Gnss_Synchro & gnss_synchro);

    std::bitset<24> DF406;
    int set_DF406(const Gnss_Synchro & gnss_synchro);

    std::bitset<10> DF407;
    int set_DF407(const Gps_Ephemeris & ephNAV, const Gps_CNAV_Ephemeris & ephCNAV, const Galileo_Ephemeris & ephFNAV, double obs_time, const Gnss_Synchro & gnss_synchro);

    std::bitset<10> DF408;
    int set_DF408(const Gnss_Synchro & gnss_synchro);

    std::bitset<3> DF409;
    int set_DF409(unsigned int iods);

    std::bitset<2> DF411;
    int set_DF411(unsigned int clock_steering_indicator);

    std::bitset<2> DF412;
    int set_DF412(unsigned int external_clock_indicator);

    std::bitset<1> DF417;
    int set_DF417(bool using_divergence_free_smoothing);

    std::bitset<3> DF418;
    int set_DF418(int carrier_smoothing_interval_s);

    std::bitset<1> DF420;
    int set_DF420(const Gnss_Synchro & gnss_synchro);
};

#endif
