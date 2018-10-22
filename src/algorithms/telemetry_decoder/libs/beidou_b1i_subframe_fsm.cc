/*!
 * \file beidou_b1i_subframe_fsm.cc
 * \brief  Implementation of a BEIDOU NAV message word-to-subframe decoder state machine
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

#include "beidou_b1i_subframe_fsm.h"
#include "gnss_satellite.h"
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/mpl/list.hpp>
#include <cstring>
#include <iostream>

//************ GPS WORD TO SUBFRAME DECODER STATE MACHINE **********
struct Ev_beidou_word_valid : sc::event<Ev_beidou_word_valid>
{
};


struct Ev_beidou_word_invalid : sc::event<Ev_beidou_word_invalid>
{
};


struct Ev_beidou_word_preamble : sc::event<Ev_beidou_word_preamble>
{
};


struct beidou_subframe_fsm_S0 : public sc::state<beidou_subframe_fsm_S0, BeidouB1iSubframeFsm>
{
public:
    // sc::transition(event,next_status)
    typedef sc::transition<Ev_beidou_word_preamble, beidou_subframe_fsm_S1> reactions;
    beidou_subframe_fsm_S0(my_context ctx) : my_base(ctx)
    {
        //std::cout<<"Enter S0 "<<std::endl;
    }
};


struct beidou_subframe_fsm_S1 : public sc::state<beidou_subframe_fsm_S1, BeidouB1iSubframeFsm>
{
public:
    typedef mpl::list<sc::transition<Ev_beidou_word_invalid, beidou_subframe_fsm_S0>,
        sc::transition<Ev_beidou_word_valid, beidou_subframe_fsm_S2> >
        reactions;

    beidou_subframe_fsm_S1(my_context ctx) : my_base(ctx)
    {
        //std::cout<<"Enter S1 "<<std::endl;
    }
};


struct beidou_subframe_fsm_S2 : public sc::state<beidou_subframe_fsm_S2, BeidouB1iSubframeFsm>
{
public:
    typedef mpl::list<sc::transition<Ev_beidou_word_invalid, beidou_subframe_fsm_S0>,
        sc::transition<Ev_beidou_word_valid, beidou_subframe_fsm_S3> >
        reactions;

    beidou_subframe_fsm_S2(my_context ctx) : my_base(ctx)
    {
        //std::cout<<"Enter S2 "<<std::endl;
        context<BeidouB1iSubframeFsm>().beidou_word_to_subframe(0);
    }
};


struct beidou_subframe_fsm_S3 : public sc::state<beidou_subframe_fsm_S3, BeidouB1iSubframeFsm>
{
public:
    typedef mpl::list<sc::transition<Ev_beidou_word_invalid, beidou_subframe_fsm_S0>,
        sc::transition<Ev_beidou_word_valid, beidou_subframe_fsm_S4> >
        reactions;

    beidou_subframe_fsm_S3(my_context ctx) : my_base(ctx)
    {
        //std::cout<<"Enter S3 "<<std::endl;
        context<BeidouB1iSubframeFsm>().beidou_word_to_subframe(1);
    }
};


struct beidou_subframe_fsm_S4 : public sc::state<beidou_subframe_fsm_S4, BeidouB1iSubframeFsm>
{
public:
    typedef mpl::list<sc::transition<Ev_beidou_word_invalid, beidou_subframe_fsm_S0>,
        sc::transition<Ev_beidou_word_valid, beidou_subframe_fsm_S5> >
        reactions;

    beidou_subframe_fsm_S4(my_context ctx) : my_base(ctx)
    {
        //std::cout<<"Enter S4 "<<std::endl;
        context<BeidouB1iSubframeFsm>().beidou_word_to_subframe(2);
    }
};


struct beidou_subframe_fsm_S5 : public sc::state<beidou_subframe_fsm_S5, BeidouB1iSubframeFsm>
{
public:
    typedef mpl::list<sc::transition<Ev_beidou_word_invalid, beidou_subframe_fsm_S0>,
        sc::transition<Ev_beidou_word_valid, beidou_subframe_fsm_S6> >
        reactions;

    beidou_subframe_fsm_S5(my_context ctx) : my_base(ctx)
    {
        //std::cout<<"Enter S5 "<<std::endl;
        context<BeidouB1iSubframeFsm>().beidou_word_to_subframe(3);
    }
};


struct beidou_subframe_fsm_S6 : public sc::state<beidou_subframe_fsm_S6, BeidouB1iSubframeFsm>
{
public:
    typedef mpl::list<sc::transition<Ev_beidou_word_invalid, beidou_subframe_fsm_S0>,
        sc::transition<Ev_beidou_word_valid, beidou_subframe_fsm_S7> >
        reactions;

    beidou_subframe_fsm_S6(my_context ctx) : my_base(ctx)
    {
        //std::cout<<"Enter S6 "<<std::endl;
        context<BeidouB1iSubframeFsm>().beidou_word_to_subframe(4);
    }
};


struct beidou_subframe_fsm_S7 : public sc::state<beidou_subframe_fsm_S7, BeidouB1iSubframeFsm>
{
public:
    typedef mpl::list<sc::transition<Ev_beidou_word_invalid, beidou_subframe_fsm_S0>,
        sc::transition<Ev_beidou_word_valid, beidou_subframe_fsm_S8> >
        reactions;

    beidou_subframe_fsm_S7(my_context ctx) : my_base(ctx)
    {
        //std::cout<<"Enter S7 "<<std::endl;
        context<BeidouB1iSubframeFsm>().beidou_word_to_subframe(5);
    }
};


struct beidou_subframe_fsm_S8 : public sc::state<beidou_subframe_fsm_S8, BeidouB1iSubframeFsm>
{
public:
    typedef mpl::list<sc::transition<Ev_beidou_word_invalid, beidou_subframe_fsm_S0>,
        sc::transition<Ev_beidou_word_valid, beidou_subframe_fsm_S9> >
        reactions;

    beidou_subframe_fsm_S8(my_context ctx) : my_base(ctx)
    {
        //std::cout<<"Enter S8 "<<std::endl;
        context<BeidouB1iSubframeFsm>().beidou_word_to_subframe(6);
    }
};


struct beidou_subframe_fsm_S9 : public sc::state<beidou_subframe_fsm_S9, BeidouB1iSubframeFsm>
{
public:
    typedef mpl::list<sc::transition<Ev_beidou_word_invalid, beidou_subframe_fsm_S0>,
        sc::transition<Ev_beidou_word_valid, beidou_subframe_fsm_S10> >
        reactions;

    beidou_subframe_fsm_S9(my_context ctx) : my_base(ctx)
    {
        //std::cout<<"Enter S9 "<<std::endl;
        context<BeidouB1iSubframeFsm>().beidou_word_to_subframe(7);
    }
};


struct beidou_subframe_fsm_S10 : public sc::state<beidou_subframe_fsm_S10, BeidouB1iSubframeFsm>
{
public:
    typedef mpl::list<sc::transition<Ev_beidou_word_invalid, beidou_subframe_fsm_S0>,
        sc::transition<Ev_beidou_word_valid, beidou_subframe_fsm_S11> >
        reactions;

    beidou_subframe_fsm_S10(my_context ctx) : my_base(ctx)
    {
        //std::cout<<"Enter S10 "<<std::endl;
        context<BeidouB1iSubframeFsm>().beidou_word_to_subframe(8);
    }
};


struct beidou_subframe_fsm_S11 : public sc::state<beidou_subframe_fsm_S11, BeidouB1iSubframeFsm>
{
public:
    typedef sc::transition<Ev_beidou_word_preamble, beidou_subframe_fsm_S1> reactions;

    beidou_subframe_fsm_S11(my_context ctx) : my_base(ctx)
    {
        //std::cout<<"Completed GPS Subframe!"<<std::endl;
        context<BeidouB1iSubframeFsm>().beidou_word_to_subframe(9);
        context<BeidouB1iSubframeFsm>().beidou_subframe_to_nav_msg();  //decode the subframe
        // DECODE SUBFRAME
        //std::cout<<"Enter S11"<<std::endl;
    }
};


BeidouB1iSubframeFsm::BeidouB1iSubframeFsm()
{
    d_nav.reset();
    i_channel_ID = 0;
    i_satellite_PRN = 0;
    d_subframe_ID = 0;
    d_flag_new_subframe = false;
    initiate();  //start the FSM
}


void BeidouB1iSubframeFsm::beidou_word_to_subframe(int position)
{
    // insert the word in the correct position of the subframe
    std::memcpy(&d_subframe[position * BEIDOU_WORD_LENGTH], &d_BEIDOU_frame_4bytes, sizeof(char) * BEIDOU_WORD_LENGTH);
}


void BeidouB1iSubframeFsm::clear_flag_new_subframe()
{
    d_flag_new_subframe = false;
}


void BeidouB1iSubframeFsm::beidou_subframe_to_nav_msg()
{
    //int subframe_ID;
    // NEW GPS SUBFRAME HAS ARRIVED!
    d_subframe_ID = d_nav.subframe_decoder(this->d_subframe);  //decode the subframe
    std::cout << "New BEIDOU NAV message received in channel " << i_channel_ID << ": "
              << "subframe "
              << d_subframe_ID << " from satellite "
              << Gnss_Satellite(std::string("Beidou"), i_satellite_PRN) << std::endl;
    d_nav.i_satellite_PRN = i_satellite_PRN;
    d_nav.i_channel_ID = i_channel_ID;

    d_flag_new_subframe = true;
}


void BeidouB1iSubframeFsm::Event_beidou_word_valid()
{
    this->process_event(Ev_beidou_word_valid());
}


void BeidouB1iSubframeFsm::Event_beidou_word_invalid()
{
    this->process_event(Ev_beidou_word_invalid());
}


void BeidouB1iSubframeFsm::Event_beidou_word_preamble()
{
    this->process_event(Ev_beidou_word_preamble());
}
