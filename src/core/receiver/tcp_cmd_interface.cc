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


std::string TcpCmdInterface::stop(const std::vector<std::string> &commandLine)
{
    std::string response;
    response = "Not implemented\n";
    return response;
}

std::string TcpCmdInterface::status(const std::vector<std::string> &commandLine)
{
    std::string response;
    response = "Not implemented\n";
    return response;
}

std::string TcpCmdInterface::assistedstart(const std::vector<std::string> &commandLine)
{
    std::string response;
    response = "Not implemented\n";
    return response;
}

std::string TcpCmdInterface::warmstart(const std::vector<std::string> &commandLine)
{
    std::string response;
    response = "Not implemented\n";
    return response;
}


std::string TcpCmdInterface::coldstart(const std::vector<std::string> &commandLine)
{
    std::string response;
    response = "Not implemented\n";
    return response;
}

std::string TcpCmdInterface::set_ch_satellite(const std::vector<std::string> &commandLine)
{
    std::string response;
    response = "Not implemented\n";
    return response;
}


void TcpCmdInterface::register_functions()
{
    functions["status"] = status;
    functions["stop"] = stop;
    functions["assistedstart"] = assistedstart;
    functions["warmstart"] = warmstart;
    functions["coldstart"] = coldstart;
    functions["set_ch_satellite"] = set_ch_satellite;
}


TcpCmdInterface::TcpCmdInterface()
{
    register_functions();
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
            bool keep_running = true;
            while (keep_running)
                {
                    try
                        {
                            std::cout << "Telecommand TCP interface listening on port " << tcp_port << std::endl;

                            boost::asio::ip::tcp::socket socket(service);
                            acceptor.accept(socket, not_throw);
                            if (not_throw)
                                {
                                    std::cerr << "Error when binding the port in the socket" << std::endl;
                                    continue;
                                }

                            // Read a message
                            boost::system::error_code error = boost::asio::error::eof;
                            do
                                {
                                    std::string response;
                                    boost::asio::streambuf b;
                                    boost::asio::read_until(socket, b, '\n');
                                    std::istream is(&b);
                                    std::string line;
                                    std::getline(is, line);
                                    std::cout << "received command: " << line << std::endl;

                                    std::istringstream iss(line);
                                    std::vector<std::string> cmd_vector(std::istream_iterator<std::string>{iss},
                                        std::istream_iterator<std::string>());

                                    if (cmd_vector.size() > 0)
                                        {
                                            try
                                                {
                                                    response = functions[cmd_vector.at(0)](cmd_vector);
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
                                            std::cerr << "Error sending(" << not_throw.value() << "): " << not_throw.message() << std::endl;
                                            break;
                                        }
                                }
                            while (!error);  // && error != boost::asio::error::eof);

                            if (error == boost::asio::error::eof)
                                {
                                    std::cout << "EOF detected\n";
                                }
                            else
                                {
                                    std::cout << "error: " << error << std::endl;
                                }

                            // Close socket
                            socket.close();
                        }
                    catch (const boost::exception &e)
                        {
                            std::cout << "Boost exception " << std::endl;
                        }
                    catch (const std::exception &ex)
                        {
                            std::cout << "Exception " << ex.what() << std::endl;
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
