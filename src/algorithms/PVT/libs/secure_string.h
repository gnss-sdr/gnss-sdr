/*!
 * \file secure_string.h
 * \brief Self-scrubbing value type for credential-bearing configuration fields
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_SECURE_STRING_H
#define GNSS_SDR_SECURE_STRING_H

#include <cstddef>
#include <string>
#include <utility>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */

/* Single wipe primitive of the code base, defined in rtklib_stream.cc. It is
 * declared here directly so this header stays free of the heavy rtklib
 * includes; the compiler rejects any signature divergence in translation
 * units that also see rtklib_stream.h. */
void secure_wipe(void* data, size_t size);

/*!
 * \brief Value type for credential-bearing configuration fields.
 *
 * Wipes its storage with volatile stores before every release - destruction,
 * assignment, clear() - so every copy of a configuration struct that carries
 * one scrubs itself, and a new holder of such a copy needs no manual scrub
 * site. Assign-once usage is assumed: in-place growth of the underlying
 * std::string is not protected.
 */
class Secure_String
{
public:
    Secure_String() = default;

    // Implicit by design: a drop-in for std::string configuration fields
    Secure_String(std::string value) : d_value(std::move(value)) {}  // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)
    Secure_String(const char* value) : d_value(value) {}             // NOLINT(google-explicit-constructor,hicpp-explicit-conversions)

    Secure_String(const Secure_String& other) = default;

    Secure_String(Secure_String&& other) noexcept : d_value(std::move(other.d_value))
    {
        other.clear();
    }

    Secure_String& operator=(const Secure_String& other)
    {
        if (this != &other)
            {
                clear();
                d_value = other.d_value;
            }
        return *this;
    }

    Secure_String& operator=(Secure_String&& other) noexcept
    {
        if (this != &other)
            {
                clear();
                d_value = std::move(other.d_value);
                other.clear();
            }
        return *this;
    }

    ~Secure_String()
    {
        clear();
    }

    void clear() noexcept
    {
        if (!d_value.empty())
            {
                secure_wipe(&d_value[0], d_value.size());
            }
        d_value.clear();
    }

    const std::string& value() const noexcept
    {
        return d_value;
    }

    bool empty() const noexcept
    {
        return d_value.empty();
    }

private:
    std::string d_value;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SECURE_STRING_H
