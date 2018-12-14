/*!
 * \file display.h
 * \brief  Defines useful display constants
 * \author Antonio Ramos, 2018. antonio.ramos(at)cttc.es
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

#ifndef GNSS_SDR_DISPLAY_H_
#define GNSS_SDR_DISPLAY_H_

#include <string>

#ifndef NO_DISPLAY_COLORS
#define DISPLAY_COLORS 1
#endif


#ifdef DISPLAY_COLORS

const std::string TEXT_RESET = "\033[0m";
const std::string TEXT_BLACK = "\033[30m";
const std::string TEXT_RED = "\033[31m";
const std::string TEXT_GREEN = "\033[32m";
const std::string TEXT_YELLOW = "\033[33m";
const std::string TEXT_BLUE = "\033[34m";
const std::string TEXT_MAGENTA = "\033[35m";
const std::string TEXT_CYAN = "\033[36m";
const std::string TEXT_WHITE = "\033[37m";
const std::string TEXT_BOLD_BLACK = "\033[1m\033[30m";
const std::string TEXT_BOLD_RED = "\033[1m\033[31m";
const std::string TEXT_BOLD_GREEN = "\033[1m\033[32m";
const std::string TEXT_BOLD_YELLOW = "\033[1m\033[33m";
const std::string TEXT_BOLD_BLUE = "\033[1m\033[34m";
const std::string TEXT_BOLD_MAGENTA = "\033[1m\033[35m";
const std::string TEXT_BOLD_CYAN = "\033[1m\033[36m";
const std::string TEXT_BOLD_WHITE = "\033[1m\033[37m";

#else

const std::string TEXT_RESET = "";
const std::string TEXT_BLACK = "";
const std::string TEXT_RED = "";
const std::string TEXT_GREEN = "";
const std::string TEXT_YELLOW = "";
const std::string TEXT_BLUE = "";
const std::string TEXT_MAGENTA = "";
const std::string TEXT_CYAN = "";
const std::string TEXT_WHITE = "";
const std::string TEXT_BOLD_BLACK = "";
const std::string TEXT_BOLD_RED = "";
const std::string TEXT_BOLD_GREEN = "";
const std::string TEXT_BOLD_YELLOW = "";
const std::string TEXT_BOLD_BLUE = "";
const std::string TEXT_BOLD_MAGENTA = "";
const std::string TEXT_BOLD_CYAN = "";
const std::string TEXT_BOLD_WHITE = "";

#endif /* DISPLAY_COLORS      */
#endif /* GNSS_SDR_DISPLAY_H_ */
