/*!
 * \file control_thread.h
 * \brief This class represents the main thread of the application, aka control thread.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2011  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * at your option) any later version.
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

#ifndef GNSS_SDR_CONTROL_THREAD_H_
#define GNSS_SDR_CONTROL_THREAD_H_

#include <gnuradio/gr_msg_queue.h>
#include "control_message_factory.h"

class GNSSFlowgraph;
class ConfigurationInterface;


/*!
 * \brief This class represents the main thread of the application, aka Control Thread.
 */
class ControlThread
{

public:

    //! Default constructor
    ControlThread();

    /*!
     * \brief Constructor that initializes the class with parameters
     *
     * \param[in] configuration Pointer to a ConfigurationInterface
     */
    ControlThread(ConfigurationInterface *configuration);

    //! Virtual destructor. Derived classes must implement the destructor
    virtual ~ControlThread();

    /*!
     * \brief Runs the ControlThread
     */
    void run();

    /*!
     * \brief Sets the control_queue
     *
     * \param[in] gr_msg_queue_sptr control_queue
     */
    void set_control_queue(gr_msg_queue_sptr control_queue);

    unsigned int processed_control_messages()
    {
        return processed_control_messages_;
    }

    unsigned int applied_actions()
    {
        return applied_actions_;
    }

    /*!
     * \brief Instantiates a flowgraph
     *
     * \return Returns a flowgraph object
     */
    GNSSFlowgraph* flowgraph()
    {
        return flowgraph_;
	}


private:

    void init();
    void read_control_messages();
    void process_control_messages();
    void apply_action(unsigned int what);

    GNSSFlowgraph *flowgraph_;
    ConfigurationInterface *configuration_;
    gr_msg_queue_sptr control_queue_;
    ControlMessageFactory *control_message_factory_;
    std::vector<ControlMessage*> *control_messages_;

    bool stop_;
    bool delete_configuration_;

    unsigned int processed_control_messages_;
    unsigned int applied_actions_;
};

#endif /*GNSS_SDR_CONTROL_THREAD_H_*/
