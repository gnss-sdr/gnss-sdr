/*!
 * \file channels.h
 * \brief class that populates the GUI options for Acquistion,
 * Tracking and Telemetry Decoder.
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

#ifndef GNSS_SDR_GUI_CHANNELS_H
#define GNSS_SDR_GUI_CHANNELS_H

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
#include <QDir>
#include <QSettings>

class Channels : public QWidget
{
    Q_OBJECT
public:
    explicit Channels(QWidget *parent = 0, QString block_name_ = "Channels", QString dir_path_ = "");
    QMap<QString, QString> *get_options();
    QString block_name;

private:
    void add_pages();
    void populate_page(int index);
    void populate_channel_page();
    QGroupBox *box_channel_types();
    QGroupBox *box_generic(int index, QStringList keys);
    QGroupBox *sub_box(QString boxname, QStringList keys);
    QGroupBox *box_channel(QString boxname, QStringList keys);
    QGroupBox *box_specfic_channel(QString boxname, int index, QStringList keys);
    void update_channel_boxes();

    //variable for keeping number of signal sources
    int sources_count;
    //MultiSource
    bool multi_source;
    //Multiband
    bool multi_band;
    //multiConstellation
    bool multi_constellation;

    QString dir_path;
    QList<QDir *> *list_subdirectory;
    QStringList *subdirectories_names;
    QStringList channel_types;
    QStringList channel_types_keys;
    QStringList dependent_keys;
    QStringList generic_keys;
    QStringList multiband_keys;
    QStringList multiconstellation_keys;
    QStringList multisource_keys;

    QDir *block_directory;
    QList<QTabWidget *> *list_sub_tabwidget;
    QList<QStringList *> *block_implementation_list;
    QList<QStringList *> *block_implementation_list_path;
    QMap<QString, QLineEdit *> *map_implementation;
    QList<QMap<QString, QLineEdit *> *> *list_map_generic;
    QList<QMap<QString, QLineEdit *> *> *list_map_subdirectory;
    QList<QMap<QString, QLineEdit *> *> *list_map_channels;
    QMap<QString, QLineEdit *> *map_channel_types;
    QMap<QString, int> *map_source_channel;
    QMap<QString, int> *map_channel_values;
    QMap<QString, int> *map_channel_tab_index;

    QList<QSpacerItem *> *list_spacer;
    QTabWidget *block_tab_widget;
    QWidget *block_scroll_area_widget;
    QVBoxLayout *block_scroll_area_widget_layout;
    QScrollArea *block_scroll_area;

signals:
    void fire_update_channels();

public slots:
    void update_channels();
    void listener_source_count(QString key, int value);
    void listener_rf_channel(QString key, int value);
    void update_subtabs();
    void generic_page_slot(QString imp);
    void channel_page_slot(QString imp);
};

#endif  // GNSS_SDR_GUI_CHANNELS_H
