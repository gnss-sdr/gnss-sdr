/*!
 * \file observables.h
 * \brief class that populates the GUI options for Observables.
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

#ifndef OBSERVABLES_H
#define OBSERVABLES_H

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


class Observables : public QWidget
{
    Q_OBJECT
public:
    explicit Observables(QWidget *parent = 0, QString block_name_="Observables", QString dir_path_="");
    QMap<QString, QString> * get_options();
    QString block_name;

private:
    void setup_page();
    QGroupBox * box_generic(QString box_name, QSettings *ini_settings);
    QGroupBox * box_implementation(QString box_name, QStringList current_group_keys);
    QDir * block_dir;
    QString dir_path;
    QTabWidget * block_tab_widget;
    QScrollArea * block_scroll_area;
    QWidget * block_scroll_area_widget;
    QVBoxLayout * block_scroll_area_widget_layout;
    QList<QString> block_implentation_list;
    QList<QString> block_implentation_list_path;
    QMap <QString, QLineEdit *> * map_implementation;
    QList <QSpacerItem *> * list_spacer;

signals:

public slots:
    void update_implementaion(QString);
};

#endif // OBSERVABLES_H
