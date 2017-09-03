/*!
 * \file signal_conditioner.h
 * \brief class that populates the GUI options for DataTypeAdapter, InputFilter
 * and Resampler.
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


#ifndef GNSS_SDR_GUI_SIGNAL_CONDITIONER_H
#define GNSS_SDR_GUI_SIGNAL_CONDITIONER_H

#include <QWidget>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtCore/qmath.h>
#include <QDir>
#include <QSettings>

class Signal_Conditioner : public QWidget
{
    Q_OBJECT
public:
    explicit Signal_Conditioner(QWidget * parent = 0, QString block_name_ = "Signal_Source", QString dir_path_ = "");
    QMap<QString, QString> * get_options();
    QString block_name;

private:
    void populate_page(int index);
    void add_subtab(int index);
    void populate_subtab(int index);
    QGroupBox * box_implementation(QString boxname, QStringList current_group_Keys, int map_index, bool add_button);
    QGroupBox * sub_box_implementation(QString boxname, QStringList current_group_keys);
    QGroupBox * box_pass_through(QString boxname, int current_source);
    QMap <QString,int> * map_source_channel;
    QString dir_path;
    QList <QDir *> * list_subdirectory;
    QMap <int,bool> * map_pass_through_flag;
    QStringList * subdirectories_paths;
    QStringList * subdirectories_names;
    QDir * block_directory;
    QList <QStringList*> * blockImplementationList;
    QList <QStringList*> * block_implementation_list_path;
    QList < QMap<QString, QLineEdit *> *> * list_map_implementation;
    QList < QMap<QString, QLineEdit *> *> * list_map_sub;
    QList < QMap<QString, QLineEdit *> *> * list_map_pass;
    QList <QSpacerItem *> * list_spacer;
    QMap<int, QStringList*> * map_subgroup_list;
    QList < QMap<QString, QStringList> *> * list_map_subgroup_child_keys;
    QList < QMap<QString, QString > *> * list_map_subgroup_keys;

    QTabWidget * block_tab_widget;
    QList <QTabWidget *> * list_sub_tabwidget;
    QWidget * block_scroll_area_widget;
    QVBoxLayout * block_scroll_area_widget_layout;
    QScrollArea * block_scroll_area;

    bool multiple_conditioners;
signals:

public slots:
    void listener_source_count(QString, int);
    void listener_rf_channel(QString, int key_value);
    void update_data_adapter(QString);
    void update_resampler(QString);
    void update_input_filter(QString);
    void add_sub_blocks();
    void enable_disable(int);
};

#endif // GNSS_SDR_GUI_SIGNAL_CONDITIONER_H
