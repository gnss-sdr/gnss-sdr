/*!
 * \file gnss_circular_deque.h
 * \brief This class implements a circular deque for Gnss_Synchro
 * \author Antonio Ramos, 2018. antonio.ramosdet(at)gmail.com
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_CIRCULAR_DEQUE_H
#define GNSS_SDR_CIRCULAR_DEQUE_H

#include <boost/circular_buffer.hpp>
#include <vector>

/** \addtogroup Algorithms_Library
 * \{ */
/** \addtogroup Algorithm_libs algorithms_libs
 * \{ */


template <class T>
class Gnss_circular_deque
{
public:
    Gnss_circular_deque();                                            //!< Default constructor
    Gnss_circular_deque(unsigned int max_size, unsigned int nchann);  //!< nchann = number of channels; max_size = channel capacity
    unsigned int size(unsigned int ch) const;                         //!< Returns the number of available elements in a channel
    T& at(unsigned int ch, unsigned int pos);                         //!< Returns a reference to an element with bound checking
    const T& get(unsigned int ch, unsigned int pos) const;            //!< Returns a const reference to an element without bound checking
    T& front(unsigned int ch);                                        //!< Returns a reference to the first element in the deque
    T& back(unsigned int ch);                                         //!< Returns a reference to the last element in the deque
    void push_back(unsigned int ch, const T& new_data);               //!< Inserts an element at the end of the deque
    void pop_front(unsigned int ch);                                  //!< Removes the first element of the deque
    void clear(unsigned int ch);                                      //!< Removes all the elements of the deque (Sets size to 0). Capacity is not modified
    void reset(unsigned int max_size, unsigned int nchann);           //!< Removes all the elements in all the channels. Re-sets the number of channels and their capacity
    void reset();                                                     //!< Removes all the channels (Sets nchann to 0)

private:
    std::vector<boost::circular_buffer<T>> d_data;
};


template <class T>
Gnss_circular_deque<T>::Gnss_circular_deque()
{
    reset();
}


template <class T>
Gnss_circular_deque<T>::Gnss_circular_deque(unsigned int max_size, unsigned int nchann)
{
    reset(max_size, nchann);
}


template <class T>
unsigned int Gnss_circular_deque<T>::size(unsigned int ch) const
{
    return d_data[ch].size();
}


template <class T>
T& Gnss_circular_deque<T>::back(unsigned int ch)
{
    return d_data[ch].back();
}


template <class T>
T& Gnss_circular_deque<T>::front(unsigned int ch)
{
    return d_data[ch].front();
}


template <class T>
T& Gnss_circular_deque<T>::at(unsigned int ch, unsigned int pos)
{
    return d_data.at(ch).at(pos);
}


template <class T>
const T& Gnss_circular_deque<T>::get(unsigned int ch, unsigned int pos) const
{
    return d_data[ch][pos];
}


template <class T>
void Gnss_circular_deque<T>::clear(unsigned int ch)
{
    d_data[ch].clear();
}


template <class T>
void Gnss_circular_deque<T>::reset(unsigned int max_size, unsigned int nchann)
{
    d_data.clear();
    if (max_size > 0 and nchann > 0)
        {
            for (unsigned int i = 0; i < nchann; i++)
                {
                    d_data.push_back(boost::circular_buffer<T>(max_size));
                }
        }
}


template <class T>
void Gnss_circular_deque<T>::reset()
{
    d_data.clear();
}


template <class T>
void Gnss_circular_deque<T>::pop_front(unsigned int ch)
{
    d_data[ch].pop_front();
}


template <class T>
void Gnss_circular_deque<T>::push_back(unsigned int ch, const T& new_data)
{
    d_data[ch].push_back(new_data);
}


/** \} */
/** \} */
#endif  // GNSS_SDR_CIRCULAR_DEQUE_H
