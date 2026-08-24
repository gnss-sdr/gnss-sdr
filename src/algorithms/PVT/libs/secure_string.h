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

#include "rtklib_secure_wipe.h"
#include <string>
#include <utility>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


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

    explicit Secure_String(std::string value) : d_value(std::move(value))
    {
        // a long secret moves its heap buffer out, but a short one is
        // byte-copied under the small-string optimization and its bytes stay
        // behind in the moved-from parameter: wipe them before it dies
        wipe_residue(value);
    }
    explicit Secure_String(const char* value) : d_value(value) {}

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

    Secure_String& operator=(std::string value)
    {
        clear();
        d_value = std::move(value);
        // the moved-from parameter may keep an SSO copy of the secret
        wipe_residue(value);
        return *this;
    }

    ~Secure_String()
    {
        clear();
    }

    void clear() noexcept
    {
        wipe_residue(d_value);
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
    // wipes the string's whole allocated buffer, not just its current size:
    // a moved-from small string reads empty while its inline buffer still
    // holds the secret's bytes, so size()-based wiping would skip exactly
    // the residue this class exists to remove. The resize() never
    // reallocates (the count is bounded by capacity()) and cannot throw.
    static void wipe_residue(std::string& value) noexcept
    {
        if (value.capacity() > 0)
            {
                value.resize(value.capacity(), '\0');
                secure_wipe(&value[0], value.size());
            }
        value.clear();
    }

    std::string d_value;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_SECURE_STRING_H
