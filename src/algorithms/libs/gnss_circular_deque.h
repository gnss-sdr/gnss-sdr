/*!
 * \file gnss_circular_deque.h
 * \brief This class implements a circular deque for Gnss_Synchro
 *
 * \author Luis Esteve, 2018. antonio.ramos(at)cttc.es
 *
 * Detailed description of the file here if needed.
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_CIRCULAR_DEQUE_H_
#define GNSS_SDR_CIRCULAR_DEQUE_H_
#include <vector>
#include <boost/circular_buffer.hpp>

template <class T>
class Gnss_circular_deque
{
public:
    Gnss_circular_deque();                                                        // Default constructor
    Gnss_circular_deque(const unsigned int max_size, const unsigned int nchann);  // nchann = number of channels; max_size = channel capacity
    unsigned int size(const unsigned int ch);                                     // Returns the number of available elements in a channel
    T& at(const unsigned int ch, const unsigned int pos);                         // Returns a reference to an element
    T& front(const unsigned int ch);                                              // Returns a reference to the first element in the deque
    T& back(const unsigned int ch);                                               // Returns a reference to the last element in the deque
    void push_back(const unsigned int ch, const T& new_data);                     // Inserts an element at the end of the deque
    void pop_front(const unsigned int ch);                                        // Removes the first element of the deque
    void clear(const unsigned int ch);                                            // Removes all the elements of the deque (Sets size to 0). Capacity is not modified
    void reset(const unsigned int max_size, const unsigned int nchann);           // Removes all the elements in all the channels. Re-sets the number of channels and their capacity
    void reset();                                                                 // Removes all the channels (Sets nchann to 0)

private:
    std::vector<boost::circular_buffer<T>> d_data;
};


template <class T>
Gnss_circular_deque<T>::Gnss_circular_deque()
{
    reset();
}

template <class T>
Gnss_circular_deque<T>::Gnss_circular_deque(const unsigned int max_size, const unsigned int nchann)
{
    reset(max_size, nchann);
}

template <class T>
unsigned int Gnss_circular_deque<T>::size(const unsigned int ch)
{
    return d_data.at(ch).size();
}

template <class T>
T& Gnss_circular_deque<T>::back(const unsigned int ch)
{
    return d_data.at(ch).back();
}


template <class T>
T& Gnss_circular_deque<T>::front(const unsigned int ch)
{
    return d_data.at(ch).front();
}


template <class T>
T& Gnss_circular_deque<T>::at(const unsigned int ch, const unsigned int pos)
{
    return d_data.at(ch).at(pos);
}

template <class T>
void Gnss_circular_deque<T>::clear(const unsigned int ch)
{
    d_data.at(ch).clear();
}

template <class T>
void Gnss_circular_deque<T>::reset(const unsigned int max_size, const unsigned int nchann)
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
void Gnss_circular_deque<T>::pop_front(const unsigned int ch)
{
    d_data.at(ch).pop_front();
}

template <class T>
void Gnss_circular_deque<T>::push_back(const unsigned int ch, const T& new_data)
{
    d_data.at(ch).push_back(new_data);
}

#endif /* GNSS_SDR_CIRCULAR_DEQUE_H_ */
