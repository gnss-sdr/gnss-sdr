/*!
 * \file gnssgui.cc
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


#include "gnssgui.h"

Gnss_GUI::Gnss_GUI(QWidget *parent): QDialog(parent)
{
    #ifdef GUI_FILES_DIR
        {
            dir_path.append(GUI_FILES_DIR);
            if (dir_path.isEmpty())
                {
                    dir_path.append(QStandardPaths::locate(QStandardPaths::HomeLocation, QString("gui_ini_files"), QStandardPaths::LocateDirectory));
                }
            else
                {
                    if (!QDir(dir_path+"gui_ini_files").exists())
                        {
                            dir_path = "";
                        }
                    else
                        {
                            dir_path = dir_path + "gui_ini_files";
                        }
                }
        }
    #else
        {
            dir_path.append(QStandardPaths::locate(QStandardPaths::HomeLocation, QString("gui_ini_files"), QStandardPaths::LocateDirectory));
        }
    #endif
    if (dir_path.isEmpty())
        {
            QString home_location = QStandardPaths::locate(QStandardPaths::HomeLocation, "", QStandardPaths::LocateDirectory);
            QMessageBox::information(this, tr("Error"),
                    "<qt>Folder <b>gui_ini_files</b> cannot be located.<br>"
                    "Please make sure it is located in the location "
                    "specified by qmake <b>GUI_FILES_LOCATION</b> variable if you set it. Else it should be"
                    "present in your <b>"+ home_location+"</b>.<br> GUI will not be populated.</qt>");
            return;
        }
    map_options = new QMap <QString, QString>;
    base_dir = new QDir(dir_path);
    base_dir->setFilter(QDir::NoDotAndDotDot | QDir::NoSymLinks | QDir::Dirs);
    foreach (QFileInfo mItem, base_dir->entryInfoList())
        {
            if (mItem.isDir())
                {
                    list_blocks.append(mItem.fileName());
                }
        }
    create_styles_list();
    create_menu();
    create_generate_box();
    //Create a tab_widget
    tab_widget = new QTabWidget;
    page_global_options = new Global_Options(this, "Global", dir_path + "/Global");
    tab_widget->addTab(page_global_options, "Global");
    page_supl = new Supl(this, "Supl", dir_path + "/Supl");
    tab_widget->addTab(page_supl, "Supl");
    page_signal_source = new Signal_Source(this, "SignalSource", dir_path + "/SignalSource");
    tab_widget->addTab(page_signal_source, "SignalSource");
    page_signal_conditioner = new Signal_Conditioner(this, "SignalConditioner", dir_path + "/SignalConditioner");
    tab_widget->addTab(page_signal_conditioner, "SignalConditioner");
    page_channels = new Channels(this, "Channels", dir_path + "/Channels");
    tab_widget->addTab(page_channels, "Channels");
    page_observables = new Observables(this, "Observables", dir_path + "/Observables");
    tab_widget->addTab(page_observables, "Observables");
    page_pvt = new Pvt(this, "Pvt", dir_path + "/Pvt");
    tab_widget->addTab(page_pvt, "Pvt");
    connect(page_signal_source, SIGNAL(share_source_count_value(QString,int)), page_signal_conditioner, SLOT(listener_source_count(QString,int)));
    connect(page_signal_source, SIGNAL(share_rf_channels(QString,int)), page_signal_conditioner, SLOT(listener_rf_channel(QString,int)));
    connect(page_signal_source, SIGNAL(share_source_count_value(QString,int)), page_channels, SLOT(listener_source_count(QString,int)));
    connect(page_signal_source, SIGNAL(share_rf_channels(QString,int)), page_channels, SLOT(listener_rf_channel(QString,int)));

    //Vertical Layout is the main Layout
    QVBoxLayout *main_layout = new QVBoxLayout;
    //Add all the widgets to the  main Layout
    main_layout->addWidget(menu_bar);
    main_layout->addWidget(tab_widget);
    main_layout->addWidget(generate_box);
    setLayout(main_layout);
    load_style1();
}


void Gnss_GUI::create_menu()
{
    menu_bar = new QMenuBar;

    file_menu = new QMenu(tr("&File"), this);
    load_menu = new QMenu(tr("&Load Style"), this);
    help_menu = new QMenu(tr("&Help"), this);
    exit_action = file_menu->addAction(tr("E&xit"));
    if (styles_list.contains("Style1"))
        {
            load_action_style1 = load_menu->addAction(tr("S&tyle1"));
            connect(load_action_style1, SIGNAL(triggered()), this, SLOT(load_style1()));
        }
    if (styles_list.contains("Custom"))
        {
            load_action_custom = load_menu->addAction(tr("C&ustom"));
            connect(load_action_custom, SIGNAL(triggered()), this, SLOT(load_custom()));
        }
    load_action_none = load_menu->addAction(tr("N&one"));
    connect(load_action_none, SIGNAL(triggered()), this, SLOT(load_none()));
    help_action = help_menu->addAction(tr("A&bout"));

    menu_bar->addMenu(file_menu);
    menu_bar->addMenu(load_menu);
    menu_bar->addMenu(help_menu);

    connect(exit_action, SIGNAL(triggered()), this, SLOT(accept()));
    connect(help_action, SIGNAL(triggered()), this, SLOT(help_slot()));
}


void Gnss_GUI::create_generate_box()
{
    generate_box = new QGroupBox("Generate");
    QHBoxLayout * generate_layout = new QHBoxLayout();
    generate_button = new QPushButton("Generate");
    connect(generate_button, SIGNAL(clicked()), this, SLOT(generate_configuration_file()));
    output_filename_editor = new QLineEdit("default_conf");
    output_filename_editor->setMaximumWidth(200);
    //output_filename_editor->setFixedWidth(30);
    generate_layout->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Expanding, QSizePolicy::Minimum));
    generate_layout->addWidget(new QLabel("Filename:"));
    generate_layout->addWidget(output_filename_editor);
    generate_layout->addWidget(generate_button);
    generate_box->setLayout(generate_layout);
}


void Gnss_GUI::generate_configuration_file()
{
    QString file_name = output_filename_editor->text().split(".").at(0);
    QFile conf_file(file_name + ".conf");
    QList <QMap <QString, QString> *> * list_map_options;
    list_map_options = new QList <QMap <QString, QString> *>;
    QMap <QString, QString> * map_temp;
    map_temp = new QMap <QString, QString>;
    list_map_options->append(page_global_options->get_options());
    list_map_options->append(page_supl->get_options());
    list_map_options->append(page_signal_source->get_options());
    list_map_options->append(page_signal_conditioner->get_options());
    list_map_options->append(page_channels->get_options());
    list_map_options->append(page_observables->get_options());
    list_map_options->append(page_pvt->get_options());
    for (int i = 0; i < list_map_options->count(); i++)
        {
            map_temp = list_map_options->at(i);
            if (!conf_file.isOpen())
                {
                    if (conf_file.open(QFile::WriteOnly | QFile::Truncate))
                        {
                            if (conf_file.isOpen())
                                {
                                    QTextStream out(&conf_file);
                                    out << "[GNSS-SDR]" << endl;
                                    foreach (QString key, map_temp->keys())
                                        {
                                            if (!map_temp->value(key).isEmpty())
                                                {
                                                    out << key << "=" << map_temp->value(key) << endl;
                                                }
                                         }
                                }
                        }
                }
            else
                {
                    QTextStream out(&conf_file);
                    foreach (QString key, map_temp->keys())
                        {
                            if (!map_temp->value(key).isEmpty())
                                {
                                    out << key << "=" << map_temp->value(key) << endl;
                                }
                        }
                }
        }
    if (conf_file.isOpen())
        {
            conf_file.close();
        }

}


void Gnss_GUI::create_styles_list()
{
    if (list_blocks.contains("Styles"))
        {
            QDir *styles_dir;
            styles_dir = new QDir(dir_path + "/Styles");
            styles_dir->setFilter(QDir::NoDotAndDotDot | QDir::NoSymLinks | QDir::Files);
            QStringList name_filters;
            name_filters << "*.qss";
            styles_dir->setNameFilters(name_filters);
            foreach (QFileInfo item, styles_dir->entryInfoList())
                {
                    styles_list.append(item.baseName());
                }
        }
    else
        {
            styles_list.clear();
        }
}


QByteArray Gnss_GUI::read_stylesheets(const QString file_path)
{
    QFile input_file(file_path);
    QByteArray input_data;
    if (input_file.open(QIODevice::Text | QIODevice::Unbuffered | QIODevice::ReadOnly))
        {
            input_data = input_file.readAll();
            input_file.close();
            return input_data;
        }
    else
        {
            return QByteArray();
        }
}


void Gnss_GUI::load_style1()
{
    QString custom_style_sheet = read_stylesheets(dir_path + "/Styles/Style1.qss");
    this->setStyleSheet(custom_style_sheet);
}


void Gnss_GUI::load_custom()
{
    QString custom_style_sheet = read_stylesheets(dir_path + "/Styles/Custom.qss");
    this->setStyleSheet(custom_style_sheet);
}


void Gnss_GUI::load_none()
{
    this->setStyleSheet("");
}


void Gnss_GUI::help_slot()
{
    QMessageBox::about(this, tr("About Menu"),
            tr("This is the <b>GUI</b> for generating the configuration files for <b>GNSS-SDR</b>. <br>"
                    "Please visit <b>GNSS-SDR</b> website by clicking <a href='http://gnss-sdr.org'>GNSS-SDR.</a><br>"
                    "Please see <b>GNSS-SDR</b> Dcoumentation by clicking <a href='http://gnss-sdr.org/docs/'>Docs.</a><br>"
                    "Please see <b>GNSS-SDR-GUI</b> Dcoumentation by clicking <a href='https://github.com/UHaider/gnss_sdr_gui'>GUI-Docs.</a><br>"));
}


Gnss_GUI::~Gnss_GUI()
{
}
