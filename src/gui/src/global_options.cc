/*!
 * \file global_options.cc
 * \brief class that populates the GUI for Global Options
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

#include "global_options.h"

Global_Options::Global_Options(QWidget *parent, QString block_name_, QString dir_path_) : QWidget(parent), block_name(block_name_), dir_path(dir_path_)
{

    block_dir = new QDir(dir_path);
    block_dir->setFilter(QDir::NoDotAndDotDot | QDir::NoSymLinks | QDir::Files);
    QStringList name_filters;
    name_filters<<"*.ini";
    block_dir->setNameFilters(name_filters);
    foreach (QFileInfo item, block_dir->entryInfoList())
        {
            if (item.isFile() && (item.fileName() != "generic.ini") )
                {
                    block_implentation_list_path.append(item.filePath());
                }
        }
    map_implementation = new QMap <QString, QLineEdit *>;
    block_tab_widget = new QTabWidget();
    block_scroll_area_widget = new QWidget();
    block_scroll_area_widget_layout = new QVBoxLayout;
    block_scroll_area_widget->setLayout(block_scroll_area_widget_layout);
    block_scroll_area = new QScrollArea();
    block_scroll_area->setWidget(block_scroll_area_widget);
    block_scroll_area->setWidgetResizable(true);
    block_scroll_area_widget_layout->addWidget(block_tab_widget);
    QVBoxLayout * layout = new QVBoxLayout;
    layout->addWidget(block_scroll_area);
    setLayout(layout);
    setup_page();

}

void Global_Options::setup_page()
{
    block_tab_widget->addTab(new QWidget (),block_name);
    block_tab_widget->widget(0)->setLayout(new QVBoxLayout());
    update_implementaion();
    block_tab_widget->widget(0)->layout()->addItem(new QSpacerItem(10, 10, QSizePolicy::Minimum, QSizePolicy::Expanding));

}

void Global_Options::update_implementaion()
{
    QString source_settings;
    QString selected_implementation="global";
    source_settings = block_dir->path()+"/"+selected_implementation+".ini";
    if (!block_implentation_list_path.contains(source_settings))
        {
            return;
        }
    QSettings * implementation_options;
    implementation_options = new QSettings(source_settings, QSettings::IniFormat);
    QStringList group_list = implementation_options->childGroups();
    if (group_list.empty())
        {
            return;
        }
    QString main_group= group_list.at(0);
    implementation_options->beginGroup(main_group);
    QStringList main_keys = implementation_options->childKeys();
    implementation_options->endGroup();
    map_implementation->clear();
    QGroupBox * option_box = box_implementation(main_group,main_keys);
    block_tab_widget->widget(0)->layout()->addWidget(option_box);
}

QGroupBox* Global_Options::box_implementation(QString box_name, QStringList current_group_keys)
{
    QGroupBox * grid_groupbox;
    grid_groupbox = new QGroupBox(box_name);
    QGridLayout *layout = new QGridLayout;
    uint max_col = 4;
    uint row=0;
    uint col=0;
    foreach(QString key,current_group_keys)
        {
            //Insert Items in main map
            if (key == "GNSS-SDR.internal_fs_sps")
                {
                    map_implementation->insert(key,new QLineEdit("2000000"));
                    map_implementation->value(key)->setToolTip("After Conditioner Internal Frequency in [sps]");
                }
            else
                {
                    map_implementation->insert(key,new QLineEdit());
                }
            layout->addWidget(new QLabel(key),row,col++);
            layout->addWidget(map_implementation->value(key),row,col++);
            if ((col % max_col) == 0)
                {
                    col=0;
                    row++;
                }
        }
    grid_groupbox->setLayout(layout);
    grid_groupbox->setObjectName("MainGroupBox");
    return grid_groupbox;
}

QMap<QString, QString > * Global_Options::get_options()
{
    QMap<QString, QString > * map_options;
    map_options  = new QMap<QString, QString >;
    foreach (QString key, map_implementation->keys())
        {
            map_options->insert(key,map_implementation->value(key)->text());
        }
    return map_options;
}

