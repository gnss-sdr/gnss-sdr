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
#include <exception>

template <class T>
class Gnss_circular_deque
{
public:
    Gnss_circular_deque();
    Gnss_circular_deque(const unsigned int max_size, const unsigned int nchann);
    ~Gnss_circular_deque();
    unsigned int size(const unsigned int ch);
    T& at(const unsigned int ch, const unsigned int pos);
    T& front(const unsigned int ch);
    T& back(const unsigned int ch);
    void push_back(const unsigned int ch, const T& new_data);
    T pop_front(const unsigned int ch);
    void clear(const unsigned int ch);
    T* get_vector(const unsigned int ch);

private:
    T** d_history;
    T d_return_void;  // Void object for avoid compiler errors
    unsigned int* d_index_pop;
    unsigned int* d_index_push;
    unsigned int* d_size;
    unsigned int d_max_size;
    unsigned int d_nchannels;
};


template <class T>
Gnss_circular_deque<T>::Gnss_circular_deque()
{
    d_max_size = 0;
    d_nchannels = 0;
    d_size = nullptr;
    d_index_pop = nullptr;
    d_index_push = nullptr;
    d_history = nullptr;
}

template <class T>
Gnss_circular_deque<T>::Gnss_circular_deque(const unsigned int max_size, const unsigned int nchann)
{
    d_max_size = max_size;
    d_nchannels = nchann;
    if (d_max_size > 0 and d_nchannels > 0)
        {
            d_size = new unsigned int[d_nchannels];
            d_index_pop = new unsigned int[d_nchannels];
            d_index_push = new unsigned int[d_nchannels];
            d_history = new T*[d_nchannels];
            for (unsigned int i = 0; i < d_nchannels; i++)
                {
                    d_size[i] = 0;
                    d_index_pop[i] = 0;
                    d_index_push[i] = 0;
                    d_history[i] = new T[d_max_size];
                    for (unsigned int ii = 0; ii < d_max_size; ii++)
                        {
                            d_history[i][ii] = d_return_void;
                        }
                }
        }
}

template <class T>
Gnss_circular_deque<T>::~Gnss_circular_deque()
{
    if (d_max_size > 0 and d_nchannels > 0)
        {
            delete[] d_size;
            delete[] d_index_pop;
            delete[] d_index_push;
            for (unsigned int i = 0; i < d_nchannels; i++)
                {
                    delete[] d_history[i];
                }
            delete[] d_history;
        }
}

template <class T>
unsigned int Gnss_circular_deque<T>::size(const unsigned int ch)
{
    return d_size[ch];
}

template <class T>
T& Gnss_circular_deque<T>::back(const unsigned int ch)
{
    if (d_size[ch] > 0)
        {
            unsigned int index = 0;
            if (d_index_push[ch] > 0)
                {
                    index = d_index_push[ch] - 1;
                }
            else
                {
                    index = d_max_size - 1;
                }
            return d_history[ch][index];
        }
    else
        {
            std::exception ex;
            throw ex;
            return d_return_void;
        }
}

template <class T>
T& Gnss_circular_deque<T>::front(const unsigned int ch)
{
    if (d_size[ch] > 0)
        {
            return d_history[ch][d_index_pop[ch]];
        }
    else
        {
            std::exception ex;
            throw ex;
            return d_return_void;
        }
}

template <class T>
T& Gnss_circular_deque<T>::at(const unsigned int ch, const unsigned int pos)
{
    if (d_size[ch] > 0 and pos < d_size[ch])
        {
            unsigned int index = ((d_index_pop[ch] + pos) % d_max_size);
            return d_history[ch][index];
        }
    else
        {
            std::exception ex;
            throw ex;
            return d_return_void;
        }
}

template <class T>
void Gnss_circular_deque<T>::clear(const unsigned int ch)
{
    d_size[ch] = 0;
    d_index_pop[ch] = 0;
    d_index_push[ch] = 0;
}

template <class T>
T Gnss_circular_deque<T>::pop_front(const unsigned int ch)
{
    if (d_size[ch] > 0)
        {
            d_size[ch]--;
            T result = d_history[ch][d_index_pop[ch]];
            if (d_size[ch] > 0)
                {
                    d_index_pop[ch]++;
                    d_index_pop[ch] %= d_max_size;
                }
            else
                {
                    clear(ch);
                }
            return result;
        }
    else
        {
            std::exception ex;
            throw ex;
            return d_return_void;
        }
}

template <class T>
void Gnss_circular_deque<T>::push_back(const unsigned int ch, const T& new_data)
{
    bool increment_pop = true;
    if (d_size[ch] < d_max_size)
        {
            increment_pop = false;
            d_size[ch]++;
        }

    d_history[ch][d_index_push[ch]] = new_data;
    d_index_push[ch]++;
    d_index_push[ch] %= d_max_size;
    if (increment_pop)
        {
            d_index_pop[ch] = d_index_push[ch];
        }
}

template <class T>
T* Gnss_circular_deque<T>::get_vector(const unsigned int ch)
{
    return d_history[ch];
}


#endif /* GNSS_SDR_CIRCULAR_DEQUE_H_ */
