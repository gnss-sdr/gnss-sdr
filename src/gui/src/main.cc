/*!
 * \file main.cc
 * \brief Main file of the GNSS-SDR GUI.
 * \author Usman Haider, 2017. usmanhaider89(at)gmail.com
 *
 *
 * -----------------------------------------------------------------------
 *
 * Copyright (C) 2010-2016  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *      Satellite Systems receiver
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
 * -----------------------------------------------------------------------
 */

#include "gnssgui.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Gnss_GUI w;
    //For minimize and maximize butttons
    w.setWindowFlags(Qt::Window);
    //By default show maximized screen
    w.showMaximized();
    w.show();
    return a.exec();
}
