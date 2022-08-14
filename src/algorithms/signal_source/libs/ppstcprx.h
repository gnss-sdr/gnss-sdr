/*
 * ppstcprx.h
 *
 *  Created on: 28 feb 2022
 *      Author: javier
 */

#ifndef SRC_LIBS_PPSTCPRX_H_
#define SRC_LIBS_PPSTCPRX_H_
#include "concurrent_queue.h"
#include "pps_samplestamp.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
class pps_tcp_rx
{
private:
    std::shared_ptr<Concurrent_Queue<PpsSamplestamp>> Pps_queue;
    int clientSd;

public:
    volatile bool is_connected;
    pps_tcp_rx();
    virtual ~pps_tcp_rx();

    void receive_pps(std::string ip_address, int port);
    bool send_cmd(std::string cmd);

    void set_pps_samplestamp_queue(std::shared_ptr<Concurrent_Queue<PpsSamplestamp>> queue);
};

#endif /* SRC_LIBS_PPSTCPRX_H_ */
