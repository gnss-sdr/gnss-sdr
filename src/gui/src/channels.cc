/*!
 * \file channels.cc
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

#include "channels.h"

Channels::Channels(QWidget *parent, QString block_name_, QString dir_path_) : QWidget(parent), block_name(block_name_), dir_path(dir_path_)
{
    //By Default sourceCount=1
    sources_count = 1;
    //By Default multiSource=fasle
    multi_source = false;
    //By Default multiband=false
    multi_band = false;
    //By Default multiConstellation=false
    multi_constellation = false;
    //Dependent Keys
    dependent_keys << "Channel.RF_channel_ID" << "Channel.SignalSource_ID" << "Channel.signal";
    list_sub_tabwidget = new QList <QTabWidget*>;
    map_implementation = new QMap<QString, QLineEdit *>;
    list_map_generic = new QList < QMap<QString, QLineEdit *> *>;
    list_map_subdirectory = new QList < QMap<QString, QLineEdit *> *>;
    list_map_channels = new QList < QMap<QString, QLineEdit *> *>;
    map_source_channel = new QMap <QString,int> ;
    map_channel_types =  new QMap<QString, QLineEdit *> ;
    map_channel_values = new QMap <QString,int>;
    map_channel_tab_index = new QMap <QString,int>;;
    list_spacer = new QList <QSpacerItem *>;
    block_directory = new QDir(dir_path);
    list_subdirectory = new QList <QDir *>;
    block_implementation_list = new QList <QStringList*>;
    block_implementation_list_path = new QList <QStringList*>;
    block_directory->setFilter(QDir::NoDotAndDotDot | QDir::NoSymLinks | QDir::Dirs);
    QStringList directory_name_filters;
    directory_name_filters << "Acquisition" << "TelemetryDecoder" << "Tracking";
    block_directory->setNameFilters(directory_name_filters);
    QSettings * implementation_options;
    implementation_options = new QSettings(dir_path + "/channel_types.ini", QSettings::IniFormat);
    QStringList group_list = implementation_options->childGroups();
    if (group_list.empty())
        {
            return;
        }
    QString main_group = group_list.at(0);
    implementation_options->beginGroup(main_group);
    channel_types_keys = implementation_options->childKeys();
    implementation_options->endGroup();
    int tabindex = 0;
    foreach (QString key, channel_types_keys)
        {
            map_channel_values->insert(key, 0);
            map_channel_tab_index->insert(key, tabindex);
            tabindex++;
        }
    QSettings * implementation_options_generic;
    implementation_options_generic = new QSettings(dir_path + "/generic.ini", QSettings::IniFormat);
    QStringList group_list_generic = implementation_options_generic->childGroups();
    if (group_list_generic.empty())
        {
            return;
        }
    QString main_group_generic = group_list_generic.at(0);
    implementation_options_generic->beginGroup(main_group_generic);
    foreach(QString key,implementation_options_generic->childKeys())
        {
            if (!(dependent_keys.contains(key)))
                {
                    generic_keys.append(key);
                }
        }
    implementation_options_generic->endGroup();
    foreach (QString key, channel_types_keys)
        {
            QStringList temp = key.split(".");
            QStringList temp1 = temp.at(0).split("_");
            channel_types.append(temp1.at(1));
        }
    subdirectories_names = new QStringList;
    QStringList name_filters;
    name_filters << "*.ini";
    foreach (QFileInfo item, block_directory->entryInfoList())
        {
            if (item.isDir())
                {
                    subdirectories_names->append(item.baseName());
                    list_subdirectory->append(new QDir(item.filePath()));
                }
        }
    for (int i = 0; i < list_subdirectory->count(); i++)
        {
            block_implementation_list->append(new QStringList);
            block_implementation_list_path->append(new QStringList);
        }
    int directory_index = 0;
    foreach(QDir * dir,*list_subdirectory)
        {
            dir->setFilter(QDir::NoDotAndDotDot | QDir::NoSymLinks | QDir::Files);
            dir->setNameFilters(name_filters);
            foreach (QFileInfo item, dir->entryInfoList())
                {
                    if (item.isFile() && (item.fileName() != "generic.ini") )
                        {
                            block_implementation_list->at(directory_index)->append(item.baseName());
                            block_implementation_list_path->at(directory_index)->append(item.filePath());
                        }
                }
            directory_index++;
        }
    block_tab_widget = new QTabWidget();
    block_scroll_area_widget = new QWidget();
    block_scroll_area_widget_layout = new QVBoxLayout;
    block_scroll_area_widget->setLayout(block_scroll_area_widget_layout);
    block_scroll_area = new QScrollArea();
    block_scroll_area->setWidget(block_scroll_area_widget);
    block_scroll_area->setWidgetResizable(true);
    block_scroll_area_widget_layout->addWidget(block_tab_widget);
    connect(this, SIGNAL(fire_update_channels()), this, SLOT(update_channels()));

    QVBoxLayout * layout = new QVBoxLayout;
    layout->addWidget(block_scroll_area);
    setLayout(layout);
    add_pages();
    populate_channel_page();
}


void Channels::add_pages()
{
    block_tab_widget->addTab(new QWidget(),"Channels");
    list_map_generic->append(new QMap<QString, QLineEdit *>);
    list_spacer->append(new QSpacerItem(10, 10, QSizePolicy::Minimum, QSizePolicy::Expanding));
    foreach (QString subdirectory_name, *subdirectories_names)
        {
            block_tab_widget->addTab(new QWidget(), subdirectory_name);
            list_sub_tabwidget->append(new QTabWidget);
            list_map_generic->append(new QMap<QString, QLineEdit *>);
            populate_page(block_tab_widget->count() - 1);
        }
}


void Channels::populate_channel_page()
{
    int index = 0;
    QVBoxLayout * layout = new QVBoxLayout();
    if (!block_tab_widget->widget(index)->layout())
        {
            block_tab_widget->widget(index)->setLayout(layout);
            layout->addWidget(box_channel_types());
            layout->addWidget(box_generic(0, generic_keys));
            //Add a spacer item at end
            block_tab_widget->widget(index)->layout()->addItem(list_spacer->at(index));
        }
}


QGroupBox* Channels::box_channel_types()
{
    QGroupBox * grid_groupbox;
    grid_groupbox = new QGroupBox("Channels");
    QPushButton * update_button = new QPushButton("Update");
    QGridLayout * layout = new QGridLayout;
    uint max_col = 4;
    uint row  =0;
    uint col = 0;
    foreach(QString key, channel_types_keys)
        {
            map_channel_types->insert(key, new QLineEdit());
        }
    foreach (const QString &key, map_channel_types->keys())
        {
            layout->addWidget(new QLabel(key), row, col++);
            layout->addWidget(map_channel_types->value(key), row, col++);
            if ((col % max_col) == 0)
                {
                    col = 0;
                    row++;
                }
        }
    layout->addWidget(update_button, row, max_col - 1, 1, 1, Qt::AlignRight);
    grid_groupbox->setLayout(layout);
    connect(update_button, SIGNAL(clicked()), this, SLOT(update_channels()));
    connect(update_button, SIGNAL(clicked()), this, SLOT(update_subtabs()));
    return grid_groupbox;
}


void Channels::update_channels()
{
    int count = 0;
    int temp;
    bool flag = false;
    multi_constellation = false;
    foreach(QString key, map_channel_types->keys())
        {
            temp = map_channel_types->value(key)->text().toInt();
            if (temp > 0)
                {
                    if (flag)
                        {
                            multi_constellation = true;
                        }
                    else
                        {
                            flag = true;
                        }
                }
            count += temp;
        }
    if (count > 0)
        {
            multiband_keys.clear();
            multiconstellation_keys.clear();
            multisource_keys.clear();
            if (multi_band)
                {
                    for (int i = 0; i < count; i++)
                        {
                            multiband_keys.append("Channel" + QString::number(i) + ".RF_channel_ID");
                        }
                }
            if (multi_source)
                {
                    for (int i = 0; i < count; i++)
                        {
                            multisource_keys.append("Channel" + QString::number(i) + ".SignalSource_ID");
                        }
                }
            if (multi_constellation)
                {
                    for (int i = 0; i < count; i++)
                        {
                            multiconstellation_keys.append("Channel" + QString::number(i) + ".signal");
                        }
                }
            else
                {
                    multiconstellation_keys.append("Channel.signal");
                }
        }
    update_channel_boxes();
}


void Channels::update_channel_boxes()
{
    block_tab_widget->widget(0)->layout()->removeItem(list_spacer->at(0));
    map_implementation->clear();
    QList<QGroupBox*> box_list = block_tab_widget->widget(0)->findChildren<QGroupBox*>("SubBox", Qt::FindDirectChildrenOnly);
    foreach(QGroupBox * box, box_list)
        {
            block_tab_widget->widget(0)->layout()->removeWidget(box);
            delete box;
        }
    block_tab_widget->widget(0)->layout()->addWidget(sub_box("MultiBand", multiband_keys));
    block_tab_widget->widget(0)->layout()->addWidget(sub_box("MultiSource", multisource_keys));
    block_tab_widget->widget(0)->layout()->addWidget(sub_box("MultiConstellation", multiconstellation_keys));
    block_tab_widget->widget(0)->layout()->addItem(list_spacer->at(0));
}


QGroupBox* Channels::sub_box(QString boxname,  QStringList keys)
{
    QGroupBox * grid_groupbox;
    grid_groupbox = new QGroupBox(boxname);
    QGridLayout * layout = new QGridLayout;
    uint max_col = 4;
    uint row = 0;
    uint col = 0;
    foreach(QString key, keys)
        {
            map_implementation->insert(key, new QLineEdit());
            layout->addWidget(new QLabel(key), row, col++);
            layout->addWidget(map_implementation->value(key), row, col++);
            if ((col % max_col) == 0)
                {
                    col = 0;
                    row++;
                }
        }
    grid_groupbox->setLayout(layout);
    grid_groupbox->setObjectName("SubBox");
    return grid_groupbox;
}


QGroupBox* Channels::box_generic(int index, QStringList keys)
{
    QGroupBox * grid_groupbox;
    grid_groupbox = new QGroupBox("Generic");
    QGridLayout * layout = new QGridLayout;
    uint max_col = 4;
    uint row = 0;
    uint col = 0;
    foreach(QString key, keys)
        {
            list_map_generic->at(index)->insert(key, new QLineEdit());
        }
    foreach (const QString &key, list_map_generic->at(index)->keys())
        {
            layout->addWidget(new QLabel(key), row, col++);
            layout->addWidget(list_map_generic->at(index)->value(key), row, col++);
            if ((col % max_col) == 0)
                {
                    col = 0;
                    row++;
                }
        }
    grid_groupbox->setLayout(layout);
    return grid_groupbox;
}


void Channels::populate_page(int index)
{
    QVBoxLayout * layout = new QVBoxLayout();
    if (!block_tab_widget->widget(index)->layout())
        {
            block_tab_widget->widget(index)->setLayout(layout);
            layout->addWidget(list_sub_tabwidget->at(index-1));
            foreach(QString key,channel_types)
                {
                    list_sub_tabwidget->at(index-1)->addTab(new QWidget(), key);
                }
        }
}


void Channels::update_subtabs()
{
    if (list_map_channels->count() > 0)
        {
            list_map_channels->clear();
        }
    if (list_map_subdirectory->count() > 0)
        {
            list_map_subdirectory->clear();
        }
    //delete all childrens widget from subtabs
    for(int i = 0; i < list_sub_tabwidget->count(); i++)
        {
            for(int j = list_sub_tabwidget->at(i)->count(); j > 0; j--)
                {
                    list_map_subdirectory->append(new QMap<QString, QLineEdit *>);
                    list_sub_tabwidget->at(i)->widget(j - 1)->deleteLater();
                    list_sub_tabwidget->at(i)->removeTab(j - 1);
                }
    }
    for(int i = 0; i < list_sub_tabwidget->count(); i++)
        {
            foreach(QString key, channel_types)
                {
                    list_sub_tabwidget->at(i)->addTab(new QWidget(), key);
                }
        }
    //Append a map for each channel specific configurations
    foreach (QString key, map_channel_types->keys())
        {
            for(int i = 0; i < map_channel_types->value(key)->text().toInt() * list_subdirectory->count(); i++)
                {
                    list_map_channels->append(new QMap<QString, QLineEdit *>);
                }
        }
    foreach (QString key, map_channel_types->keys())
        {
            int sub_index =  map_channel_tab_index->value(key);
            int channel_count = map_channel_types->value(key)->text().toInt();
            if (channel_count > 0)
                {
                    //add generic box
                    for(int i = 0; i < list_sub_tabwidget->count(); i++)
                        {
                            QVBoxLayout * layout =  new QVBoxLayout;
                            list_sub_tabwidget->at(i)->widget(sub_index)->setLayout(layout);
                            layout->addWidget(box_channel("Generic", *block_implementation_list->at(i)));
                            //add channel count number of boxes
                            for (int j = 0; j < channel_count; j++)
                                {
                                    layout->addWidget(box_specfic_channel("Channel Specific", j, *block_implementation_list->at(i)));
                                }
                            layout->addItem(new QSpacerItem(10, 10, QSizePolicy::Minimum, QSizePolicy::Expanding));
                        }
                }
            else
                {
                    //remove all boxes along with generic
                }
            //Update the value
            map_channel_values->remove(key);
            map_channel_values->insert(key, channel_count);
        }
}


QGroupBox* Channels::box_channel(QString boxname, QStringList keys)
{
    QGroupBox * grid_groupbox;
    grid_groupbox = new QGroupBox(boxname);
    QVBoxLayout * main_layout = new QVBoxLayout;
    QGroupBox * grid_groupboxinner;
    grid_groupboxinner = new QGroupBox("optionBox");
    QGridLayout * layout = new QGridLayout;
    QComboBox * source_combobox;
    source_combobox = new QComboBox();
    source_combobox->setObjectName("sourceComboBox");
    source_combobox->addItem("Select");
    foreach (QString mImp, keys)
        {
            source_combobox->addItem(mImp);
        }
    main_layout->addWidget(source_combobox);
    main_layout->addWidget(grid_groupboxinner);
    connect(source_combobox, SIGNAL(currentIndexChanged(QString)), this, SLOT(generic_page_slot(QString)));
    grid_groupboxinner->setLayout(layout);
    grid_groupbox->setLayout(main_layout);
    return grid_groupbox;
}


void Channels::generic_page_slot(QString imp)
{
    QComboBox * sender_combobox = qobject_cast<QComboBox*>(sender());
    if( sender_combobox != NULL )
        {
            QList<QGroupBox*> box_list = sender_combobox->parentWidget()->findChildren<QGroupBox*>(QString(), Qt::FindDirectChildrenOnly);
            foreach(QGroupBox * box, box_list)
                {
                    foreach(QLabel * obj, box->findChildren<QLabel*>(QString(), Qt::FindDirectChildrenOnly))
                        {
                            box->layout()->removeWidget(obj);
                            delete obj;
                        }
                    foreach(QLineEdit * obj, box->findChildren<QLineEdit*>(QString(), Qt::FindDirectChildrenOnly))
                        {
                            box->layout()->removeWidget(obj);
                            delete obj;
                        }
                }
            int tab_index = block_tab_widget->currentIndex();
            int map_index = list_sub_tabwidget->at(tab_index - 1)->currentIndex() + (tab_index - 1) * 4;
            int main_map_index = list_sub_tabwidget->at(tab_index - 1)->currentIndex();
            list_map_subdirectory->at(map_index)->clear();
            QString temp_block = block_tab_widget->tabText(block_tab_widget->currentIndex());
            QString imp_path = dir_path + "/" + temp_block + "/" + imp + ".ini";
            QSettings * implementation_options;
            implementation_options = new QSettings(imp_path, QSettings::IniFormat);
            QStringList group_list = implementation_options->childGroups();
            if (group_list.empty())
                {
                    return;
                }
            QString main_group = group_list.at(0);
            QString imp_value = group_list.at(0);
            implementation_options->beginGroup(main_group);
            QStringList all_keys = implementation_options->childKeys();
            implementation_options->endGroup();
            QString imp_key;
            imp_key = block_tab_widget->tabText(tab_index) + "_" + list_sub_tabwidget->at(tab_index - 1)->tabText(main_map_index) + ".implementation";
            list_map_subdirectory->at(map_index)->insert(imp_key, new QLineEdit(imp_value));
            uint max_col = 4;
            uint row = 0;
            uint col = 0;
            foreach(QString key, all_keys)
                {
                    list_map_subdirectory->at(map_index)->insert(key, new QLineEdit());
                    QRegularExpression key_re1( "^(Acquisition_)(1B|1C|2S|5X)[0-9]{0,}.(doppler_)(max|min|step)$" );
                    QRegularExpressionMatch match1 = key_re1.match(key);
                    if (match1.hasMatch())
                        {
                            list_map_subdirectory->at(map_index)->value(key)->setToolTip("Specify in [Hz]");
                        }

                    QRegularExpression key_re2( "^(Acquisition_|Tracking_)(1B|1C|2S|5X)[0-9]{0,}.(if)$" );
                    QRegularExpressionMatch match2 = key_re2.match(key);
                    if (match2.hasMatch())
                        {
                            list_map_subdirectory->at(map_index)->value(key)->setToolTip("Intermediate Frequency in [Hz]");
                        }

                    QRegularExpression key_re3( "^(Tracking_)(1B|1C|2S|5X)[0-9]{0,}.(dll|pll)(_bw_)(hz|init_hz|narrow_hz)$" );
                    QRegularExpressionMatch match3 = key_re3.match(key);
                    if (match3.hasMatch())
                        {
                            list_map_subdirectory->at(map_index)->value(key)->setToolTip("Bandwidth in [Hz]");
                        }

                    QGroupBox * box = sender_combobox->parentWidget()->findChild<QGroupBox*>(QString(), Qt::FindDirectChildrenOnly);
                    qobject_cast<QGridLayout*>( box->layout())->addWidget(new QLabel(key), row, col++);
                    qobject_cast<QGridLayout*>( box->layout())->addWidget(list_map_subdirectory->at(map_index)->value(key), row, col++);
                    if ((col % max_col) == 0)
                        {
                            col = 0;
                            row++;
                        }
                }
        }
}


QGroupBox* Channels::box_specfic_channel(QString boxname, int index,  QStringList keys)
{
    QGroupBox * grid_groupbox;
    grid_groupbox = new QGroupBox(boxname);
    QVBoxLayout * main_layout = new QVBoxLayout;
    QGroupBox * grid_groupboxinner;
    grid_groupboxinner = new QGroupBox("Channel:" + QString::number(index));
    QGridLayout * layout = new QGridLayout;
    QComboBox * source_combobox;
    source_combobox = new QComboBox();
    source_combobox->setObjectName("sourceComboBox");
    source_combobox->addItem("Select");
    foreach (QString mImp, keys)
        {
            source_combobox->addItem(mImp);
        }
    main_layout->addWidget(source_combobox);
    main_layout->addWidget(grid_groupboxinner);
    connect(source_combobox, SIGNAL(currentIndexChanged(QString)), this, SLOT(channel_page_slot(QString)));
    grid_groupboxinner->setLayout(layout);
    grid_groupbox->setLayout(main_layout);
    return grid_groupbox;
}


void Channels::channel_page_slot(QString imp)
{
    QComboBox * sender_combobox = qobject_cast<QComboBox*>(sender());
    if( sender_combobox != NULL )
        {
            QList<QGroupBox*> box_list = sender_combobox->parentWidget()->findChildren<QGroupBox*>(QString(), Qt::FindDirectChildrenOnly);
            foreach(QGroupBox * box, box_list)
                {
                    foreach(QLabel *obj, box->findChildren<QLabel*>(QString(), Qt::FindDirectChildrenOnly))
                        {
                            box->layout()->removeWidget(obj);
                            delete obj;
                        }
                    foreach(QLineEdit *obj, box->findChildren<QLineEdit*>(QString(), Qt::FindDirectChildrenOnly))
                        {
                            box->layout()->removeWidget(obj);
                            delete obj;
                        }
                }
            QList<int> cumm_channel;
            int tab_index = block_tab_widget->currentIndex();
            int sub_index = list_sub_tabwidget->at(tab_index - 1)->currentIndex();
            int total_channels = 0;
            foreach (QString key, map_channel_types->keys())
                {
                    cumm_channel.append(total_channels);
                    total_channels += map_channel_types->value(key)->text().toInt();
                }
            QGroupBox * box = sender_combobox->parentWidget()->findChild<QGroupBox*>(QString(), Qt::FindDirectChildrenOnly);
            QString box_title = box->title();
            QString channel_number = box_title.split(":").at(1);
            int map_index = (tab_index - 1) * total_channels + cumm_channel.at(sub_index) + channel_number.toInt();
            list_map_channels->at(map_index)->clear();
            QString temp_block = block_tab_widget->tabText(block_tab_widget->currentIndex());
            QString imp_path = dir_path + "/" + temp_block + "/" + imp + ".ini";
            QSettings * implementation_options;
            implementation_options = new QSettings(imp_path, QSettings::IniFormat);
            QStringList group_list = implementation_options->childGroups();
            if (group_list.empty())
                {
                    return;
                }
            QString main_group = group_list.at(0);
            QString imp_value = group_list.at(0);
            implementation_options->beginGroup(main_group);
            QStringList all_keys = implementation_options->childKeys();
            implementation_options->endGroup();
            int main_map_index = list_sub_tabwidget->at(tab_index-1)->currentIndex();
            QString imp_key;
            imp_key = block_tab_widget->tabText(tab_index) + "_" + list_sub_tabwidget->at(tab_index - 1)->tabText(main_map_index) + channel_number + ".implementation";
            list_map_channels->at(map_index)->insert(imp_key, new QLineEdit(imp_value));
            uint max_col = 4;
            uint row = 0;
            uint col = 0;
            foreach(QString key, all_keys)
                {
                    if (!key.contains('.'))
                        {
                            continue;
                        }
                    QStringList temp = key.split(".");
                    key = temp.at(0) + channel_number + "." + temp.at(1);
                    list_map_channels->at(map_index)->insert(key, new QLineEdit());
                    QRegularExpression key_re1( "^(Acquisition_)(1B|1C|2S|5X)[0-9]{0,}.(doppler_)(max|min|step)$" );
                    QRegularExpressionMatch match1 = key_re1.match(key);
                    if (match1.hasMatch())
                        {
                            list_map_channels->at(map_index)->value(key)->setToolTip("Specify in [Hz]");
                        }

                    QRegularExpression key_re2( "^(Acquisition_|Tracking_)(1B|1C|2S|5X)[0-9]{0,}.(if)$" );
                    QRegularExpressionMatch match2 = key_re2.match(key);
                    if (match2.hasMatch())
                        {
                            list_map_channels->at(map_index)->value(key)->setToolTip("Intermediate Frequency in [Hz]");
                        }

                    QRegularExpression key_re3( "^(Tracking_)(1B|1C|2S|5X)[0-9]{0,}.(dll|pll)(_bw_)(hz|init_hz|narrow_hz)$" );
                    QRegularExpressionMatch match3 = key_re3.match(key);
                    if (match3.hasMatch())
                        {
                            list_map_channels->at(map_index)->value(key)->setToolTip("Bandwidth in [Hz]");
                        }
                    qobject_cast<QGridLayout*>( box->layout())->addWidget(new QLabel(key), row, col++);
                    qobject_cast<QGridLayout*>( box->layout())->addWidget(list_map_channels->at(map_index)->value(key), row, col++);
                    if ((col % max_col) == 0)
                        {
                            col = 0;
                            row++;
                        }
                }
        }
}


void Channels::listener_source_count(QString key, int value)
{
    if (key == "Receiver.sources_count")
        {
            if (value > 0)
                {
                    sources_count = value;
                    multi_band = false;
                    if (value > 1)
                        {
                            multi_source = true;
                        }
                    else
                        {
                            multi_source = false;
                        }
                    map_source_channel->clear();
                    if (value > 1)
                        {
                            for (int i = 0; i < value; i++)
                                {
                                    //By default 1 RF channel for each source
                                    map_source_channel->insert("SignalSource" + QString::number(i) + ".RF_channels", 1);
                                }
                        }
                    else
                        {
                            //By default 1 RF channel for each source
                            map_source_channel->insert("SignalSource.RF_channels", 1);
                        }
                }
        }
    emit fire_update_channels();
}


void Channels::listener_rf_channel(QString key, int value)
{
    if (map_source_channel->contains(key))
        {
            if(value > 1)
                {
                    multi_band = true;
                }
            else
                {
                    multi_band = false;
                }
            map_source_channel->remove(key);
            map_source_channel->insert(key, value);
        }
    emit fire_update_channels();
}


QMap<QString, QString >* Channels::get_options()
{
    QMap<QString, QString > * map_options;
    map_options  = new QMap<QString, QString >;
    foreach (QString key, map_channel_types->keys())
        {
            map_options->insert(key, map_channel_types->value(key)->text());
        }
    foreach (QString key, map_implementation->keys())
        {
            map_options->insert(key, map_implementation->value(key)->text());
        }
    for (int i = 0; i < list_map_generic->count(); i++)
        {
            foreach (QString key, list_map_generic->at(i)->keys())
                {
                    map_options->insert(key, list_map_generic->at(i)->value(key)->text());
                }
        }
    for (int i = 0; i < list_map_subdirectory->count(); i++)
        {
            foreach (QString key, list_map_subdirectory->at(i)->keys())
                {
                    map_options->insert(key, list_map_subdirectory->at(i)->value(key)->text());
                }
        }
    for (int i = 0; i < list_map_channels->count(); i++)
        {
            foreach (QString key, list_map_channels->at(i)->keys())
                {
                    map_options->insert(key, list_map_channels->at(i)->value(key)->text());
                }
        }
    return map_options;
}

