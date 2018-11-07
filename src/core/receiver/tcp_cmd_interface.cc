/*!
 * \file tcp_cmd_interface.cc
 *
 * \brief Class that implements a TCP telecontrol command line interface
 * for GNSS-SDR
 * \author Javier Arribas jarribas (at) cttc.es
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#include "tcp_cmd_interface.h"
#include "control_message_factory.h"
#include <functional>
#include <sstream>


void TcpCmdInterface::set_pvt(std::shared_ptr<PvtInterface> PVT_sptr)
{
    PVT_sptr_ = PVT_sptr;
}
time_t TcpCmdInterface::get_utc_time()
{
    return receiver_utc_time_;
}

arma::vec TcpCmdInterface::get_LLH()
{
    return arma::vec{rx_latitude_, rx_longitude_, rx_altitude_};
}
std::string TcpCmdInterface::reset(const std::vector<std::string> &commandLine)
{
    std::string response;
    std::unique_ptr<ControlMessageFactory> cmf(new ControlMessageFactory());
    if (control_queue_ != nullptr)
        {
            control_queue_->handle(cmf->GetQueueMessage(200, 1));  //send the restart message (who=200,what=1)
            response = "OK\n";
        }
    else
        {
            response = "ERROR\n";
        }

    return response;
}

std::string TcpCmdInterface::standby(const std::vector<std::string> &commandLine)
{
    std::string response;
    std::unique_ptr<ControlMessageFactory> cmf(new ControlMessageFactory());
    if (control_queue_ != nullptr)
        {
            control_queue_->handle(cmf->GetQueueMessage(300, 10));  //send the standby message (who=300,what=10)
            response = "OK\n";
        }
    else
        {
            response = "ERROR\n";
        }
    return response;
}

std::string TcpCmdInterface::status(const std::vector<std::string> &commandLine)
{
    std::stringstream str_stream;
    //todo: implement the receiver status report

    //    str_stream << "-------------------------------------------------------\n";
    //    str_stream << "ch | sys | sig | mode | Tlm | Eph | Doppler | CN0 |\n";
    //    str_stream << "   |     |     |      |     |     |  [Hz]   | [dB - Hz] |\n";
    //    str_stream << "-------------------------------------------------------\n";
    //    int n_ch = 10;
    //    for (int n = 0; n < n_ch; n++)
    //        {
    //            str_stream << n << "GPS | L1CA | TRK | YES | YES | 23412.4 | 44.3 |\n";
    //        }
    //    str_stream << "--------------------------------------------------------\n";

    double longitude_deg, latitude_deg, height_m, ground_speed_kmh, course_over_ground_deg;
    time_t UTC_time;
    if (PVT_sptr_->get_latest_PVT(&longitude_deg,
            &latitude_deg,
            &height_m,
            &ground_speed_kmh,
            &course_over_ground_deg,
            &UTC_time) == true)
        {
            struct tm tstruct;
            char buf1[80];
            tstruct = *gmtime(&UTC_time);
            strftime(buf1, sizeof(buf1), "%d/%m/%Y %H:%M:%S", &tstruct);
            std::string str_time = std::string(buf1);
            str_stream << "- Receiver UTC Time: " << str_time << std::endl;
            str_stream << std::setprecision(9);
            str_stream << "- Receiver Position WGS84 [Lat, Long, H]: "
                       << latitude_deg << ", "
                       << longitude_deg << ", ";
            str_stream << std::setprecision(3);
            str_stream << height_m << std::endl;
            str_stream << std::setprecision(1);
            str_stream << "- Receiver Speed over Ground [km/h]: " << ground_speed_kmh << std::endl;
            str_stream << "- Receiver Course over ground [deg]: " << course_over_ground_deg << std::endl;
        }
    else
        {
            str_stream << "No PVT information available.\n";
        }

    return str_stream.str();
}

std::string TcpCmdInterface::hotstart(const std::vector<std::string> &commandLine)
{
    std::string response;
    if (commandLine.size() > 5)
        {
            //read commandline time parameter
            struct tm tm;
            strptime(commandLine.at(1).c_str(), "%d/%m/%Y %H:%M:%S", &tm);
            receiver_utc_time_ = timegm(&tm);


            //read latitude, longitude, and height
            rx_latitude_ = std::stod(commandLine.at(3).c_str());
            rx_longitude_ = std::stod(commandLine.at(4).c_str());
            rx_altitude_ = std::stod(commandLine.at(5).c_str());

            if (std::isnan(rx_latitude_) || std::isnan(rx_longitude_) || std::isnan(rx_altitude_))
                {
                    response = "ERROR: position malformed\n";
                }
            else
                {
                    std::unique_ptr<ControlMessageFactory> cmf(new ControlMessageFactory());
                    if (control_queue_ != nullptr)
                        {
                            control_queue_->handle(cmf->GetQueueMessage(300, 12));  //send the standby message (who=300,what=12)
                            response = "OK\n";
                        }
                    else
                        {
                            response = "ERROR\n";
                        }
                }
        }
    else
        {
            response = "ERROR: time parameter not found, please use hotstart %d/%m/%Y %H:%M:%S\n";
        }
    return response;
}

std::string TcpCmdInterface::warmstart(const std::vector<std::string> &commandLine)
{
    std::string response;
    if (commandLine.size() > 5)
        {
            std::string tmp_str;
            //read commandline time parameter
            struct tm tm;
            tmp_str = commandLine.at(1) + commandLine.at(2);
            strptime(tmp_str.c_str(), "%d/%m/%Y %H:%M:%S", &tm);
            receiver_utc_time_ = timegm(&tm);

            //read latitude, longitude, and height
            rx_latitude_ = std::stod(commandLine.at(3).c_str());
            rx_longitude_ = std::stod(commandLine.at(4).c_str());
            rx_altitude_ = std::stod(commandLine.at(5).c_str());
            if (std::isnan(rx_latitude_) || std::isnan(rx_longitude_) || std::isnan(rx_altitude_))
                {
                    response = "ERROR: position malformed\n";
                }
            else
                {
                    std::unique_ptr<ControlMessageFactory> cmf(new ControlMessageFactory());
                    if (control_queue_ != nullptr)
                        {
                            control_queue_->handle(cmf->GetQueueMessage(300, 13));  //send the standby message (who=300,what=13)
                            response = "OK\n";
                        }
                    else
                        {
                            response = "ERROR\n";
                        }
                }
        }
    else
        {
            response = "ERROR: time parameter not found, please use warmstart %d/%m/%Y %H:%M:%S Lat Long H\n";
        }
    return response;
}


std::string TcpCmdInterface::coldstart(const std::vector<std::string> &commandLine)
{
    std::string response;
    std::unique_ptr<ControlMessageFactory> cmf(new ControlMessageFactory());
    if (control_queue_ != nullptr)
        {
            control_queue_->handle(cmf->GetQueueMessage(300, 11));  //send the standby message (who=300,what=11)
            response = "OK\n";
        }
    else
        {
            response = "ERROR\n";
        }
    return response;
}

std::string TcpCmdInterface::set_ch_satellite(const std::vector<std::string> &commandLine)
{
    std::string response;
    //todo: implement the set satellite command
    response = "Not implemented\n";
    return response;
}


void TcpCmdInterface::register_functions()
{
    functions["status"] = std::bind(&TcpCmdInterface::status, this, std::placeholders::_1);
    functions["standby"] = std::bind(&TcpCmdInterface::standby, this, std::placeholders::_1);
    functions["reset"] = std::bind(&TcpCmdInterface::reset, this, std::placeholders::_1);
    functions["hotstart"] = std::bind(&TcpCmdInterface::hotstart, this, std::placeholders::_1);
    functions["warmstart"] = std::bind(&TcpCmdInterface::warmstart, this, std::placeholders::_1);
    functions["coldstart"] = std::bind(&TcpCmdInterface::coldstart, this, std::placeholders::_1);
    functions["set_ch_satellite"] = std::bind(&TcpCmdInterface::set_ch_satellite, this, std::placeholders::_1);
}


TcpCmdInterface::TcpCmdInterface()
{
    register_functions();
    keep_running_ = true;
    control_queue_ = nullptr;
    rx_latitude_ = 0;
    rx_longitude_ = 0;
    rx_altitude_ = 0;
    receiver_utc_time_ = 0;
}


void TcpCmdInterface::set_msg_queue(gr::msg_queue::sptr control_queue)
{
    control_queue_ = control_queue;
}
void TcpCmdInterface::run_cmd_server(int tcp_port)
{
    // Get the port from the parameters
    uint16_t port = tcp_port;

    // Error to not throw exception
    boost::system::error_code not_throw;

    // Socket and acceptor
    boost::asio::io_service service;
    try
        {
            boost::asio::ip::tcp::acceptor acceptor(service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port));

            while (keep_running_)
                {
                    try
                        {
                            std::cout << "TcpCmdInterface: Telecommand TCP interface listening on port " << tcp_port << std::endl;

                            boost::asio::ip::tcp::socket socket(service);
                            acceptor.accept(socket, not_throw);
                            if (not_throw)
                                {
                                    std::cout << "TcpCmdInterface: Error when binding the port in the socket" << std::endl;
                                    continue;
                                }

                            // Read a message
                            boost::system::error_code error = boost::asio::error::eof;
                            do
                                {
                                    std::string response;
                                    boost::asio::streambuf b;
                                    boost::asio::read_until(socket, b, '\n', error);
                                    std::istream is(&b);
                                    std::string line;
                                    std::getline(is, line);
                                    std::istringstream iss(line);
                                    std::vector<std::string> cmd_vector(std::istream_iterator<std::string>{iss},
                                        std::istream_iterator<std::string>());

                                    if (cmd_vector.size() > 0)
                                        {
                                            try
                                                {
                                                    if (cmd_vector.at(0).compare("exit") == 0)
                                                        {
                                                            error = boost::asio::error::eof;
                                                            //send cmd response
                                                            socket.write_some(boost::asio::buffer("OK\n"), not_throw);
                                                        }
                                                    else
                                                        {
                                                            response = functions[cmd_vector.at(0)](cmd_vector);
                                                        }
                                                }
                                            catch (const std::bad_function_call &ex)
                                                {
                                                    response = "ERROR: command not found \n ";
                                                }
                                            catch (const std::exception &ex)
                                                {
                                                    response = "ERROR: command execution error: " + std::string(ex.what()) + "\n";
                                                }
                                        }
                                    else
                                        {
                                            response = "ERROR: empty command\n";
                                        }

                                    //send cmd response
                                    socket.write_some(boost::asio::buffer(response), not_throw);
                                    if (not_throw)
                                        {
                                            std::cout << "Error sending(" << not_throw.value() << "): " << not_throw.message() << std::endl;
                                            break;
                                        }
                                }
                            while (error != boost::asio::error::eof);

                            if (error == boost::asio::error::eof)
                                {
                                    std::cout << "TcpCmdInterface: EOF detected\n";
                                }
                            else
                                {
                                    std::cout << "TcpCmdInterface unexpected error: " << error << std::endl;
                                }

                            // Close socket
                            socket.close();
                        }
                    catch (const boost::exception &e)
                        {
                            std::cout << "TcpCmdInterface: Boost exception " << std::endl;
                        }
                    catch (const std::exception &ex)
                        {
                            std::cout << "TcpCmdInterface: Exception " << ex.what() << std::endl;
                        }
                }
        }
    catch (const boost::exception &e)
        {
            std::cout << "TCP Command Interface exception: address already in use" << std::endl;
        }
}


TcpCmdInterface::~TcpCmdInterface()
{
    // TODO Auto-generated destructor stub
}
