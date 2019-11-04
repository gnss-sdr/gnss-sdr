/*!
 * \file item_type_helpers.h
 * \brief Utility functions for converting between item types
 * \authors <ul>
 *          <li> Cillian O'Driscoll, 2017. cillian.odriscoll(at)gmail.com
 *          </ul>
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2017  (see AUTHORS file for a list of contributors)
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

#include "item_type_helpers.h"

#include <cstring> // memcpy
#include <volk/volk.h>
#include <volk_gnsssdr/volk_gnsssdr.h>

std::string external_item_type_to_internal( const std::string &external_item_type )
{
    std::string internal_item_type( "" );

    if( external_item_type == "byte" )
    {
        internal_item_type = "i8";
    }
    else if( external_item_type == "ibyte" || external_item_type == "cbyte" )
    {
        internal_item_type = "ic8";
    }
    else if( external_item_type == "short" )
    {
        internal_item_type = "i16";
    }
    else if( external_item_type == "ishort" || external_item_type == "cshort" )
    {
        internal_item_type = "ic16";
    }
    else if( external_item_type == "float" )
    {
        internal_item_type = "f32";
    }
    else if( external_item_type == "gr_complex" )
    {
        internal_item_type = "fc32";
    }


    return internal_item_type;

}

bool item_type_valid( const std::string & item_type )
{
    if( item_type != "i8" and item_type != "ic8" and
            item_type != "i16" and item_type != "ic16" and
            item_type != "i32" and item_type != "ic32" and
            item_type != "f32" and item_type != "fc32" )
    {
        return false;
    }

    return true;
}

size_t item_type_size( const std::string & item_type )
{
    if( item_type == "i8" )
    {
        return sizeof( int8_t );
    }
    else if( item_type == "ic8" )
    {
        return 2*sizeof( int8_t );
    }
    else if( item_type == "i16" )
    {
        return sizeof( int16_t );
    }
    else if( item_type == "ic16" )
    {
        return 2*sizeof( int16_t );
    }
    else if( item_type == "i32" )
    {
        return sizeof( int32_t );
    }
    else if( item_type == "ic32" )
    {
        return 2*sizeof( int32_t );
    }
    else if( item_type == "f32" )
    {
        return sizeof( float );
    }
    else if( item_type == "fc32" )
    {
        return 2*sizeof(float);
    }
    else
    {
        return 0;
    }
}

// VOLK doesnt do 32 bit integer converters
template< typename OT >
void convert_32i_generic( OT *dest, const int32_t *src, unsigned int num_items )
{
    for( unsigned int i = 0; i < num_items; ++i )
    {
        dest[i] = static_cast< OT >( src[i] );
    }
}

void copy_converter( void *dest, const void *src, unsigned int num_items, size_t item_size )
{
    std::memcpy( dest, src, num_items*item_size );
}

void convert_8i_16i( void *dest, const void *src, unsigned int num_items )
{
    volk_8i_convert_16i( reinterpret_cast< int16_t * >(dest),
            reinterpret_cast< const int8_t *>(src), num_items );
}

void convert_8i_32f( void *dest, const void *src, unsigned int num_items )
{
    volk_8i_s32f_convert_32f( reinterpret_cast< float * >(dest),
            reinterpret_cast< const int8_t *>(src), 1.0f, num_items );
}

void convert_8ic_16ic( void *dest, const void *src, unsigned int num_items )
{
    volk_8i_convert_16i( reinterpret_cast< int16_t * >(dest),
            reinterpret_cast< const int8_t *>(src), 2*num_items );
}

void convert_8ic_32fc( void *dest, const void *src, unsigned int num_items )
{
    volk_8i_s32f_convert_32f( reinterpret_cast< float * >(dest),
            reinterpret_cast< const int8_t *>(src), 1.0f, 2*num_items );
}

void convert_16i_8i( void *dest, const void *src, unsigned int num_items )
{
    volk_16i_convert_8i( reinterpret_cast< int8_t * >(dest),
            reinterpret_cast< const int16_t *>(src), num_items );
}

void convert_16i_32f( void *dest, const void *src, unsigned int num_items )
{
    volk_16i_s32f_convert_32f( reinterpret_cast< float * >(dest),
            reinterpret_cast< const int16_t *>(src), 1.0f, num_items );
}

void convert_16ic_8ic( void *dest, const void *src, unsigned int num_items )
{
    volk_16i_convert_8i( reinterpret_cast< int8_t * >(dest),
            reinterpret_cast< const int16_t *>(src), 2*num_items );
}

void convert_16ic_32fc( void *dest, const void *src, unsigned int num_items )
{
    volk_16i_s32f_convert_32f( reinterpret_cast< float * >(dest),
            reinterpret_cast< const int16_t *>(src), 1.0f, 2*num_items );
}

void convert_32i_8i( void *dest, const void *src, unsigned int num_items )
{
    convert_32i_generic<int8_t>( reinterpret_cast< int8_t *>(dest), 
            reinterpret_cast<const int32_t*>(src), num_items );
}

void convert_32i_16i( void *dest, const void *src, unsigned int num_items )
{
    convert_32i_generic<int16_t>( reinterpret_cast< int16_t *>(dest), 
            reinterpret_cast<const int32_t*>(src), num_items );
}

void convert_32i_32f( void *dest, const void *src, unsigned int num_items )
{
    convert_32i_generic<float>( reinterpret_cast< float *>(dest), 
            reinterpret_cast<const int32_t*>(src), num_items );
}

void convert_32ic_8ic( void *dest, const void *src, unsigned int num_items )
{
    convert_32i_generic<int8_t>( reinterpret_cast< int8_t *>(dest), 
            reinterpret_cast<const int32_t*>(src), 2*num_items );
}

void convert_32ic_16ic( void *dest, const void *src, unsigned int num_items )
{
    convert_32i_generic<int16_t>( reinterpret_cast< int16_t *>(dest), 
            reinterpret_cast<const int32_t*>(src), 2*num_items );
}

void convert_32ic_32fc( void *dest, const void *src, unsigned int num_items )
{
    convert_32i_generic<float>( reinterpret_cast< float *>(dest), 
            reinterpret_cast<const int32_t*>(src), 2*num_items );
}

void convert_32f_8i( void *dest, const void *src, unsigned int num_items )
{
    volk_32f_s32f_convert_8i( reinterpret_cast< int8_t * >(dest),
            reinterpret_cast< const float *>(src), 1.0f, num_items );
}

void convert_32f_16i( void *dest, const void *src, unsigned int num_items )
{
    volk_32f_s32f_convert_16i( reinterpret_cast< int16_t * >(dest),
            reinterpret_cast< const float *>(src), 1.0f, num_items );
}

void convert_32fc_8ic( void *dest, const void *src, unsigned int num_items )
{
    volk_32f_s32f_convert_8i( reinterpret_cast< int8_t * >(dest),
            reinterpret_cast< const float *>(src), 1.0f, 2*num_items );
}

void convert_32fc_16ic( void *dest, const void *src, unsigned int num_items )
{
    volk_32f_s32f_convert_16i( reinterpret_cast< int16_t * >(dest),
            reinterpret_cast< const float *>(src), 1.0f, 2*num_items );
}

/*!
 * \brief Create a function to convert an array of input_type to an array of output_type
 *
 * \description Provides a generic interface to generate conversion functions for mapping
 * arrays of items.
 *
 * \param input_type - String representation of the input item type
 * \param output_type - String representation of the output item type
 *
 * The item types accepted are:
 *
 *  1. "i8" for 8 bit integers
 *  2. "ic8" for complex (interleaved) 8 bit integers
 *  3. "i16" for 16 bit integers
 *  4. "ic16" for complex (interleaved) 16 bit integers
 *  5. "f32" for 32 bit floating point values
 *  6. "fc32" for complex (interleaved) 32 bit floating point values
 *
 * \returns A function object with the following prototype:
 *  void convert_fun( void *dest, void *src, int num_items );
 *
 *  
 */
item_type_converter_t make_vector_converter( std::string input_type,
        std::string output_type )
{
    if( not item_type_valid( input_type ) or not item_type_valid( output_type ) )
    {
        throw std::runtime_error( "make_vector_converter: invalid item types : " 
                + input_type + " " + output_type );
    }

    if( input_type == output_type )
    {
        size_t input_size = item_type_size( input_type );
        return std::bind( copy_converter, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3, input_size );
    }

    if( input_type == "i8" )
    {
        if( output_type == "i16" )
        {
            return std::bind( convert_8i_16i, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
        else if( output_type == "f32" )
        {
            return std::bind( convert_8i_32f, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }

    }
    else if( input_type == "ic8" )
    {
        if( output_type == "ic16" )
        {
            return std::bind( convert_8ic_16ic, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
        else if( output_type == "fc32" )
        {
            return std::bind( convert_8ic_32fc, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
    }
    else if( input_type == "i16" )
    {
        if( output_type == "i8" )
        {
            return std::bind( convert_16i_8i, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
        else if( output_type == "f32" )
        {
            return std::bind( convert_16i_32f, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
    }
    else if( input_type == "ic16" )
    {
        if( output_type == "ic8" )
        {
            return std::bind( convert_16ic_8ic, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
        else 
        if( output_type == "fc32" )
        {
            return std::bind( convert_16ic_32fc, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
    }
    else if( input_type == "i32" )
    {
        if( output_type == "i8" )
        {
            return std::bind( convert_32i_8i, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
        else if( output_type == "i16" )
        {
            return std::bind( convert_32i_16i, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
        else if( output_type == "f32" )
        {
            return std::bind( convert_32i_32f, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
    }
    else if( input_type == "ic32" )
    {
        if( output_type == "ic8" )
        {
            return std::bind( convert_32ic_8ic, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
        else 
        if( output_type == "ic16" )
        {
            return std::bind( convert_32ic_16ic, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
        else 
        if( output_type == "fc32" )
        {
            return std::bind( convert_32ic_32fc, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
    }
    else if( input_type == "f32" )
    {
        if( output_type == "i8" )
        {
            return std::bind( convert_32f_8i, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
        else if( output_type == "i16" )
        {
            return std::bind( convert_32f_16i, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
    }
    else if( input_type == "fc32" )
    {
        if( output_type == "ic8" )
        {
            return std::bind( convert_32fc_8ic, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
        else if( output_type == "ic16" )
        {
            return std::bind( convert_32fc_16ic, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3 );
        }
    }

    throw std::runtime_error( "make_vector_converter: invalid conversion : "
            + input_type + " to " + output_type );

}


