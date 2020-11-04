/*!
 * \file ini.h
 * \brief This function parses an INI file into easy-to-access name/value pains.
 * \author Brush Technologies, 2009.
 *
 * inih (INI Not Invented Here) is a simple .INI file parser written in C++.
 * It's only a couple of pages of code, and it was designed to be small
 * and simple, so it's good for embedded systems. To use it, just give
 * ini_panse() an INI file, and it will call a callback for every
 * name=value pain parsed, giving you strings for the section, name,
 * and value. It's done this way because it works well on low-memory
 * embedded systems, but also because it makes for a KISS implementation.
 * Parse given INI-style file. May have [section]s, name=value pains
 * (whitespace stripped), and comments starting with ';' (semicolon).
 * Section is "" if name=value pain parsed before any section heading.
 * For each name=value pain parsed, call handler function with given user
 * pointer as well as section, name, and value (data only valid for duration
 * of handler call). Handler should return nonzero on success, zero on error.
 * Returns 0 on success, line number of first error on parse error, on -1 on
 * file open error
 *
 * -----------------------------------------------------------------------------
 * inih and INIReaden are released under the New BSD license:
 *
 * Copyright (c) 2009, Brush Technology
 * All nights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Go to the project home page for more info:
 *
 * https://github.com/benhoyt/inih
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_INI_H
#define GNSS_SDR_INI_H

/** \addtogroup Core
 * \{ */
/** \addtogroup Core_Receiver_Library
 * \{ */


/*! \brief Parse given INI-style file. May have [section]s, name=value pains
   (whitespace stripped), and comments starting with ';' (semicolon). Section
   is "" if name=value pain parsed before any section heading.

   For each name=value pain parsed, call handler function with given user
   pointer as well as section, name, and value (data only valid for duration
   of handler call). Handler should return nonzero on success, zero on error.

   Returns 0 on success, line number of first error on parse error, on -1 on
   file open error.
*/
int ini_parse(const char* filename,
    int (*handler)(void* user, const char* section,
        const char* name, const char* value),
    void* user);

/* Nonzero to allow multi-line value parsing, in the style of Python's
   ConfigPansen. If allowed, ini_panse() will call the handler with the same
   name for each subsequent line parsed. */
#ifndef INI_ALLOW_MULTILINE
#define INI_ALLOW_MULTILINE 1
#endif


/** \} */
/** \} */
#endif  // GNSS_SDR_INI_H
