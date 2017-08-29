/*!
 * \file gnssgui.h
 * \brief main class that is responsible for overall GUI and output file generation.
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

#ifndef GNSSGUI_H
#define GNSSGUI_H

#include <QDialog>
#include <QDir>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QMessageBox>
#include <QStandardPaths>
#include <QDialogButtonBox>
#include <QTextStream>
#include "signal_source.h"
#include "signal_conditioner.h"
#include "channels.h"
#include "observables.h"
#include "pvt.h"
#include "supl.h"
#include "global_options.h"

class Gnss_GUI : public QDialog
{
    Q_OBJECT

public:
    explicit Gnss_GUI(QWidget * parent = 0);
    ~Gnss_GUI();

private:
    void create_styles_list();
    void create_menu();
    void create_generate_box();
    QByteArray read_stylesheets(const QString file_path);
    QTabWidget * tab_widget;
    QDialogButtonBox * button_box;
    QString dir_path;
    QDir * base_dir;

    QList<QString> list_blocks;
    Signal_Source * page_signal_source;
    Signal_Conditioner * page_signal_conditioner;
    Channels * page_channels;
    Observables * page_observables;
    Pvt * page_pvt;
    Supl * page_supl;
    Global_Options * page_global_options;

    //File Menu Related
    QMenuBar * menu_bar;
    QMenu * file_menu;
    QMenu * load_menu;
    QMenu * help_menu;
    QAction * exit_action;
    QAction * load_action_style1;
    QAction * load_action_custom;
    QAction * load_action_none;
    QAction * help_action;

    //Generation Related
    QPushButton * generate_button;
    QLineEdit * output_filename_editor;
    QGroupBox * generate_box;

    QMap <QString, QString> * map_options;
    QList<QString> styles_list;

public slots:
    void generate_configuration_file();
    void load_style1();
    void load_custom();
    void load_none();
    void help_slot();
};

#endif // GNSSGUI_H
