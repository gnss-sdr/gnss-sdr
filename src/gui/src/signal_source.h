/*!
 * \file signal_source.h
 * \brief class that populates the GUI options for SignalSource
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

#ifndef GNSS_SDR_GUI_SIGNAL_SOURCE_H
#define GNSS_SDR_GUI_SIGNAL_SOURCE_H

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
#include <QStandardPaths>

class Signal_Source : public QWidget
{
    Q_OBJECT
public:
    explicit Signal_Source(QWidget *parent = 0, QString block_name_ = "Signal_Source", QString dir_path_ = "");
    QMap<QString, QString> *get_options();
    QString block_name;

private:
    QGroupBox *box_generic(QString boxname, QSettings *iniSettings);
    QGroupBox *box_implementation(QString boxname, QStringList currGroupKeys);
    QGroupBox *sub_box_implementation(QString boxname, QStringList currGroupKeys);
    QDir *block_dir;
    QString dir_path;

    QSettings *generic_settings;
    QMap<QString, QLineEdit *> *map_generic;

    QList<QMap<QString, QLineEdit *> *> *list_map_implementation;
    QList<QMap<QString, QLineEdit *> *> *list_map_sub;
    QList<QMap<QString, QComboBox *> *> *list_map_comboboxes;
    QList<QSpacerItem *> *list_spacer;

    // List of subgroups for each source implementation
    QMap<int, QStringList *> *map_subgroup_list;
    QList<QMap<QString, QString> *> *list_map_subgroup_keys;
    QList<QMap<QString, QStringList> *> *list_map_subgroup_child_keys;

    int sources_count;
    QList<QString> block_implementation_list;
    QList<QString> block_implementation_list_path;
    QList<QGroupBox *> block_box_list;
    QTabWidget *block_tab_widget;
    QScrollArea *block_scroll_area;
    QWidget *block_scroll_area_widget;
    QVBoxLayout *block_scroll_area_widget_layout;

signals:
    void share_source_count_value(QString, int);
    void share_rf_channels(QString, int);

public slots:
    void update_sources();
    void update_source_pages();
    void update_implementation(QString);
    void add_sub_blocks();
};

#endif  // GNSS_SDR_GUI_SIGNAL_SOURCE_H
