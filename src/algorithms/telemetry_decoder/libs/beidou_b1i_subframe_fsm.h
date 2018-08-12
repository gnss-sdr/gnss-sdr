/*!
 * \file beidou_b1i_subframe_fsm.h
 * \brief  Interface of a BEIDOU NAV message word-to-subframe decoder state machine
 * \author Sergi Segura, 2018. sergi.segura.munoz(at)gmail.com
 *
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


#ifndef GNSS_SDR_BEIDOU_B1I_SUBFRAME_FSM_H_
#define GNSS_SDR_BEIDOU_B1I_SUBFRAME_FSM_H_

#include "beidou_b1I.h"
#include "beidou_navigation_message.h"
#include "beidou_ephemeris.h"
#include "beidou_iono.h"
#include "beidou_almanac.h"
#include "beidou_utc_model.h"
#include <boost/statechart/state_machine.hpp>

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

struct beidou_subframe_fsm_S0;
struct beidou_subframe_fsm_S1;
struct beidou_subframe_fsm_S2;
struct beidou_subframe_fsm_S3;
struct beidou_subframe_fsm_S4;
struct beidou_subframe_fsm_S5;
struct beidou_subframe_fsm_S6;
struct beidou_subframe_fsm_S7;
struct beidou_subframe_fsm_S8;
struct beidou_subframe_fsm_S9;
struct beidou_subframe_fsm_S10;
struct beidou_subframe_fsm_S11;


/*!
 * \brief This class implements a Finite State Machine that handles the decoding
 *  of the GPS L1 C/A NAV message
 */
class BeidouB1iSubframeFsm : public sc::state_machine<BeidouB1iSubframeFsm, beidou_subframe_fsm_S0>
{
public:
    BeidouB1iSubframeFsm();  //!< The constructor starts the Finite State Machine
    void clear_flag_new_subframe();
    // channel and satellite info
    int i_channel_ID;              //!< Channel id
    unsigned int i_satellite_PRN;  //!< Satellite PRN number

    Beidou_Navigation_Message_D1 d_nav;  //!< GPS L1 C/A navigation message object

    // GPS SV and System parameters
    Beidou_Ephemeris ephemeris;  //!< Object that handles GPS ephemeris parameters
    Beidou_Almanac almanac;      //!< Object that handles GPS almanac data
    Beidou_Utc_Model utc_model;  //!< Object that handles UTM model parameters
    Beidou_Iono iono;            //!< Object that handles ionospheric parameters

    char d_subframe[BEIDOU_SUBFRAME_LENGTH];
    int d_subframe_ID;
    bool d_flag_new_subframe;
    char d_BEIDOU_frame_4bytes[BEIDOU_WORD_LENGTH];
    //double d_preamble_time_ms;

    void beidou_word_to_subframe(int position);  //!< inserts the word in the correct position of the subframe

    /*!
     * \brief This function decodes a NAv message subframe and pushes the information to the right queues
     */
    void beidou_subframe_to_nav_msg();

    //FSM EVENTS
    void Event_beidou_word_valid();     //!< FSM event: the received word is valid
    void Event_beidou_word_invalid();   //!< FSM event: the received word is not valid
    void Event_beidou_word_preamble();  //!< FSM event: word preamble detected
};

#endif
