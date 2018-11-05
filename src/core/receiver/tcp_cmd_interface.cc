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
    std::string response;
    //todo: implement the receiver status report
    response = "Not implemented\n";
    return response;
}

std::string TcpCmdInterface::hotstart(const std::vector<std::string> &commandLine)
{
    std::string response;
    //todo: read and parse the command line parameter: dd/mm/yyyy HH:MM:SS
    //todo: store it in a member variable
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
    return response;
}

std::string TcpCmdInterface::warmstart(const std::vector<std::string> &commandLine)
{
    std::string response;
    //todo: read and parse the command line parameter: dd/mm/yyyy HH:MM:SS
    //todo: store it in a member variable
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
