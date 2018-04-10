/*!
 * \file signal_source.cc
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

#include "signal_source.h"
#include <iostream>
Signal_Source::Signal_Source(QWidget *parent, QString block_name_, QString dir_path_) : QWidget(parent), block_name(block_name_), dir_path(dir_path_)
{
    map_generic = new QMap<QString, QLineEdit *>;
    map_subgroup_list = new QMap<int, QStringList *>;
    list_map_implementation = new QList<QMap<QString, QLineEdit *> *>;
    list_map_sub = new QList<QMap<QString, QLineEdit *> *>;
    list_map_subgroup_keys = new QList<QMap<QString, QString> *>;
    list_map_subgroup_child_keys = new QList<QMap<QString, QStringList> *>;
    list_map_comboboxes = new QList<QMap<QString, QComboBox *> *>;
    list_spacer = new QList<QSpacerItem *>;
    block_dir = new QDir(dir_path);
    block_dir->setFilter(QDir::NoDotAndDotDot | QDir::NoSymLinks | QDir::Files);
    QStringList name_filters;
    name_filters << "*.ini";
    block_dir->setNameFilters(name_filters);
    foreach (QFileInfo item, block_dir->entryInfoList())
        {
            if (item.isFile() && (item.fileName() != "generic.ini"))
                {
                    block_implementation_list.append(item.baseName());
                    block_implementation_list_path.append(item.filePath());
                }
        }
    generic_settings = new QSettings(dir_path + "/generic.ini", QSettings::IniFormat);
    block_box_list.append(box_generic("Generic", generic_settings));
    block_tab_widget = new QTabWidget();
    block_scroll_area_widget = new QWidget();
    block_scroll_area_widget_layout = new QVBoxLayout;
    block_scroll_area_widget->setLayout(block_scroll_area_widget_layout);
    block_scroll_area = new QScrollArea();
    block_scroll_area->setWidget(block_scroll_area_widget);
    block_scroll_area->setWidgetResizable(true);
    foreach (QGroupBox *item, block_box_list)
        {
            block_scroll_area_widget_layout->addWidget(item);
        }
    block_scroll_area_widget_layout->addWidget(block_tab_widget);
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(block_scroll_area);
    QString homeLocation = QStandardPaths::locate(QStandardPaths::DocumentsLocation, QString("test"), QStandardPaths::LocateDirectory);
    setLayout(layout);
}


QGroupBox *Signal_Source::box_generic(QString boxname, QSettings *ini_settings)
{
    QGroupBox *grid_groupbox;
    grid_groupbox = new QGroupBox(boxname);
    QPushButton *update_button = new QPushButton("Update");
    QGridLayout *layout = new QGridLayout;
    uint max_col = 4;
    uint row = 0;
    uint col = 0;

    ini_settings->beginGroup("Generic");
    //map_generic = new QMap<QString, QLineEdit *>;
    foreach (QString key, ini_settings->allKeys())
        {
            //if (key == "Receiver.sources_count")
            //    {
            //        map_generic->insert(key,new QLineEdit("1"));
            //        map_generic->value(key)->setToolTip("Number of signal sources");
            //    }
            //else
            //    {
            map_generic->insert(key, new QLineEdit());
            //    }
        }
    foreach (const QString &key, map_generic->keys())
        {
            layout->addWidget(new QLabel(key), row, col++);
            layout->addWidget(map_generic->value(key), row, col++);
            if ((col % max_col) == 0)
                {
                    col = 0;
                    row++;
                }
        }
    ini_settings->endGroup();
    layout->addWidget(update_button, row, max_col - 1, 1, 1, Qt::AlignRight);
    grid_groupbox->setLayout(layout);
    connect(update_button, SIGNAL(clicked()), this, SLOT(update_sources()));
    return grid_groupbox;
}


void Signal_Source::update_sources()
{
    if (map_generic->contains("Receiver.sources_count"))
        {
            if (map_generic->value("Receiver.sources_count")->text().isEmpty())
                {
                    emit share_source_count_value("Receiver.sources_count", 0);
                }
            else
                {
                    emit share_source_count_value("Receiver.sources_count", map_generic->value("Receiver.sources_count")->text().toInt());
                }
        }
    if (map_generic->contains("Receiver.sources_count"))
        {
            sources_count = map_generic->value("Receiver.sources_count")->text().toInt();
        }
    if (sources_count != 0)
        {
            for (int i = block_tab_widget->count(); i > 0; i--)
                {
                    list_map_implementation->clear();
                    list_map_sub->clear();
                    list_map_comboboxes->clear();
                    map_subgroup_list->clear();
                    list_map_subgroup_keys->clear();
                    list_map_subgroup_child_keys->clear();
                    list_spacer->clear();
                    block_tab_widget->widget(i - 1)->deleteLater();
                    block_tab_widget->removeTab(i - 1);
                }
            for (int i = 0; i < sources_count; i++)
                {
                    block_tab_widget->addTab(new QWidget(), block_name + QString::number(i));
                    //create a main map for each source
                    list_map_implementation->append(new QMap<QString, QLineEdit *>);
                    //create a sub map for each source
                    list_map_sub->append(new QMap<QString, QLineEdit *>);
                    list_map_comboboxes->append(new QMap<QString, QComboBox *>);
                    //insert a QStringList in map for each source for keeping the subgrouplist
                    map_subgroup_list->insert(i, new QStringList);
                    list_map_subgroup_keys->append(new QMap<QString, QString>);
                    list_map_subgroup_child_keys->append(new QMap<QString, QStringList>);
                    list_spacer->append(new QSpacerItem(10, 10, QSizePolicy::Minimum, QSizePolicy::Expanding));
                }
        }
    update_source_pages();
}


void Signal_Source::update_source_pages()
{
    for (int sourceind = 0; sourceind < block_tab_widget->count(); sourceind++)
        {
            if (!block_tab_widget->widget(sourceind)->layout())
                {
                    block_tab_widget->widget(sourceind)->setLayout(new QVBoxLayout());
                    QComboBox *source_combobox;
                    source_combobox = new QComboBox();
                    source_combobox->setObjectName("sourceComboBox");
                    source_combobox->addItem("Select");
                    foreach (QString implementation, block_implementation_list)
                        {
                            source_combobox->addItem(implementation);
                        }
                    connect(source_combobox, SIGNAL(currentIndexChanged(QString)), this, SLOT(update_implementation(QString)));
                    block_tab_widget->widget(sourceind)->layout()->addWidget(source_combobox);
                    block_tab_widget->widget(sourceind)->layout()->setAlignment(source_combobox, Qt::AlignTop);
                    //Add a spacer item at end
                    block_tab_widget->widget(sourceind)->layout()->addItem(list_spacer->at(sourceind));
                }
        }
}


void Signal_Source::update_implementation(QString sourceImpl)
{
    QString share_key = "SignalSource" + QString::number(block_tab_widget->currentIndex()) + ".RF_channels";
    QString imp_value;
    emit share_rf_channels(share_key, 1);
    QString source_settings;
    source_settings = block_dir->path() + "/" + sourceImpl + ".ini";
    if (!block_implementation_list_path.contains(source_settings))
        {
            return;
        }
    QSettings *implementation_options;
    implementation_options = new QSettings(source_settings, QSettings::IniFormat);
    QStringList groupList = implementation_options->childGroups();
    if (groupList.empty())
        {
            return;
        }
    QString top_group = groupList.at(0);
    imp_value = groupList.at(0);
    implementation_options->beginGroup(top_group);
    QStringList main_keys = implementation_options->childKeys();
    QStringList main_keys_updated;
    QStringList sub_keys_updated;
    int current_source = block_tab_widget->currentIndex();
    if (block_tab_widget->count() > 1)
        {
            foreach (QString key, main_keys)
                {
                    QStringList temp = key.split(".");
                    QString temp_key = temp.at(0) + QString::number(current_source) + "." + temp.at(1);
                    main_keys_updated.append(temp_key);
                }
        }
    else
        {
            main_keys_updated = main_keys;
        }

    QComboBox *sender_combobox = qobject_cast<QComboBox *>(sender());
    if (sender_combobox != NULL)
        {
            //clear the main map for current source
            list_map_implementation->at(current_source)->clear();
            //clear the subgroup map for current source
            list_map_sub->at(current_source)->clear();
            list_map_subgroup_keys->at(current_source)->clear();
            list_map_subgroup_child_keys->at(current_source)->clear();
            if (block_tab_widget->count() > 1)
                {
                    list_map_implementation->at(current_source)->insert("SignalSource" + QString::number(current_source) + ".implementation", new QLineEdit(imp_value));
                }
            else
                {
                    list_map_implementation->at(current_source)->insert("SignalSource.implementation", new QLineEdit(imp_value));
                }
            QGroupBox *mainBox = box_implementation(top_group, main_keys_updated);
            QList<QGroupBox *> boxList = sender_combobox->parentWidget()->layout()->parentWidget()->findChildren<QGroupBox *>(QString(), Qt::FindDirectChildrenOnly);
            foreach (QGroupBox *box, boxList)
                {
                    sender_combobox->parentWidget()->layout()->removeWidget(box);
                    delete box;
                }
            //Remove spacer
            sender_combobox->parentWidget()->layout()->removeItem(list_spacer->at(current_source));
            sender_combobox->parentWidget()->layout()->addWidget(mainBox);
            sender_combobox->parentWidget()->layout()->setAlignment(mainBox, Qt::AlignTop);
            //Add Spacer
            sender_combobox->parentWidget()->layout()->addItem(list_spacer->at(current_source));
        }
    map_subgroup_list->value(current_source)->operator=(implementation_options->childGroups());
    if (!((*map_subgroup_list->value(current_source)).empty()))
        {
            foreach (QString sub_group, *map_subgroup_list->value(current_source))
                {
                    QStringList temp = main_keys_updated.filter(sub_group);
                    list_map_subgroup_keys->at(current_source)->insert(temp.at(0), temp.at(0));
                    implementation_options->beginGroup(sub_group);
                    QStringList sub_keys = implementation_options->childKeys();
                    if (block_tab_widget->count() > 1)
                        {
                            foreach (QString key, sub_keys)
                                {
                                    QStringList temp = key.split(".");
                                    QString temp_key = temp.at(0) + QString::number(current_source) + "." + temp.at(1);
                                    sub_keys_updated.append(temp_key);
                                }
                        }
                    else
                        {
                            sub_keys_updated = sub_keys;
                        }
                    list_map_subgroup_child_keys->at(current_source)->insert(temp.at(0), sub_keys_updated);
                }
        }
    implementation_options->endGroup();
}


QGroupBox *Signal_Source::box_implementation(QString boxname, QStringList group_keys)
{
    QGroupBox *grid_groupbox;
    grid_groupbox = new QGroupBox(boxname);
    QPushButton *update_button = new QPushButton("Update");
    QGridLayout *layout = new QGridLayout;
    uint max_col = 4;
    QComboBox *dumpComboBox;
    QComboBox *repeatComboBox;
    QComboBox *enable_throttle_controlComboBox;
    QComboBox *item_typeComboBox;
    QComboBox *subdeviceComboBox;
    QComboBox *clock_sourceComboBox;
    QComboBox *AGCComboBox;

    uint row = 0;
    uint col = 0;
    int current_source = block_tab_widget->currentIndex();

    QStringList group_main_pars;
    QStringList group_pars;
    QStringList group_dump;

    QRegularExpression key_File_Signal_Source("File_Signal_Source");
    QRegularExpression key_UHD_Signal_Source("UHD_Signal_Source");
    QRegularExpression key_Osmosdr_Signal_Source("Osmosdr_Signal_Source");
    QRegularExpression key_RtlTcp_Signal_Source("RtlTcp_Signal_Source");

    QRegularExpression key_dump("dump(?!(_filename))");
    QRegularExpression key_dump_filename("dump_filename");
    QRegularExpression key_filename("\\.filename");
    QRegularExpression key_sampling_frequency("sampling_freq");
    QRegularExpression key_device_address("device_address");
    QRegularExpression key_subdevice("\\.subdevice");
    QRegularExpression key_samples("\\.samples");
    QRegularExpression key_item_type("\\.item_type");
    QRegularExpression key_repeat("\\.repeat");
    QRegularExpression key_enable_throttle_control("\\.enable_throttle_control");
    QRegularExpression key_implementation("\\.implementation");
    QRegularExpression key_clock_source("\\.clock_source");
    QRegularExpression key_freq("^(SignalSource)[0-9]{0,}.(freq)$");
    QRegularExpression key_gain("\\.gain$");
    QRegularExpression key_rf_gain("\\.rf_gain$");
    QRegularExpression key_if_gain("\\.if_gain$");
    QRegularExpression key_AGC_enabled("\\.AGC_enabled");

    QString current_implementation;
    for (int i = 0; i < list_map_implementation->count(); i++)
        {
            foreach (QString key, list_map_implementation->at(i)->keys())
                {
                    QRegularExpressionMatch match_implementation = key_implementation.match(key);
                    if (match_implementation.hasMatch())
                        {
                            current_implementation = list_map_implementation->at(i)->value(key)->text();
                        }
                }
        }
    QRegularExpressionMatch match_File_Signal_Source = key_File_Signal_Source.match(current_implementation);
    QRegularExpressionMatch match_UHD_Signal_Source = key_UHD_Signal_Source.match(current_implementation);
    QRegularExpressionMatch match_Osmosdr_Signal_Source = key_Osmosdr_Signal_Source.match(current_implementation);
    QRegularExpressionMatch match_RtlTcp_Signal_Source = key_RtlTcp_Signal_Source.match(current_implementation);

    foreach (QString key, group_keys)
        {
            //Insert Items in main map for this source
            QRegularExpressionMatch match_dump = key_dump.match(key);
            QRegularExpressionMatch match_dump_filename = key_dump_filename.match(key);
            QRegularExpressionMatch match_filename = key_filename.match(key);
            QRegularExpressionMatch match_sampling_frequency = key_sampling_frequency.match(key);
            QRegularExpressionMatch match_device_address = key_device_address.match(key);
            QRegularExpressionMatch match_samples = key_samples.match(key);
            QRegularExpressionMatch match_item_type = key_item_type.match(key);
            QRegularExpressionMatch match_repeat = key_repeat.match(key);
            QRegularExpressionMatch match_enable_throttle_control = key_enable_throttle_control.match(key);
            QRegularExpressionMatch match_subdevice = key_subdevice.match(key);
            QRegularExpressionMatch match_clock_source = key_clock_source.match(key);
            QRegularExpressionMatch match_freq = key_freq.match(key);
            QRegularExpressionMatch match_gain = key_gain.match(key);
            QRegularExpressionMatch match_rf_gain = key_rf_gain.match(key);
            QRegularExpressionMatch match_if_gain = key_if_gain.match(key);
            QRegularExpressionMatch match_AGC_enabled = key_AGC_enabled.match(key);

            if (match_dump_filename.hasMatch())
                {
                    group_dump << key;
                }
            if (match_filename.hasMatch())
                {
                    group_main_pars << key;
                }
            if (match_sampling_frequency.hasMatch())
                {
                    group_main_pars << key;
                }
            if (match_device_address.hasMatch())
                {
                    group_main_pars << key;
                }
            if (match_freq.hasMatch())
                {
                    group_main_pars << key;
                }

            if (match_subdevice.hasMatch())
                {
                    subdeviceComboBox = new QComboBox();
                    subdeviceComboBox->setObjectName("subdeviceComboBox");
                    subdeviceComboBox->addItem(tr("A:0"));
                    subdeviceComboBox->addItem(tr("B:0"));
                    subdeviceComboBox->setCurrentIndex(0);
                    list_map_comboboxes->at(current_source)->insert(key, subdeviceComboBox);
                }
            else if (match_clock_source.hasMatch())
                {
                    clock_sourceComboBox = new QComboBox();
                    clock_sourceComboBox->setObjectName("clock_sourceComboBox");
                    clock_sourceComboBox->addItem(tr("internal"));
                    clock_sourceComboBox->addItem(tr("external"));
                    clock_sourceComboBox->addItem(tr("MIMO"));
                    clock_sourceComboBox->setCurrentIndex(0);
                    list_map_comboboxes->at(current_source)->insert(key, clock_sourceComboBox);
                }
            else if (match_dump.hasMatch())
                {
                    dumpComboBox = new QComboBox();
                    dumpComboBox->setObjectName("dumpComboBox");
                    dumpComboBox->addItem(tr("false"));
                    dumpComboBox->addItem(tr("true"));
                    dumpComboBox->setCurrentIndex(0);
                    list_map_comboboxes->at(current_source)->insert(key, dumpComboBox);
                    group_dump << key;
                }
            else if (match_repeat.hasMatch())
                {
                    repeatComboBox = new QComboBox();
                    repeatComboBox->setObjectName("repeatComboBox");
                    repeatComboBox->addItem(tr("false"));
                    repeatComboBox->addItem(tr("true"));
                    repeatComboBox->setCurrentIndex(0);
                    list_map_comboboxes->at(current_source)->insert(key, repeatComboBox);
                }
            else if (match_enable_throttle_control.hasMatch())
                {
                    enable_throttle_controlComboBox = new QComboBox();
                    enable_throttle_controlComboBox->setObjectName("enable_throttle_controlComboBox");
                    enable_throttle_controlComboBox->addItem(tr("false"));
                    enable_throttle_controlComboBox->addItem(tr("true"));
                    enable_throttle_controlComboBox->setCurrentIndex(0);
                    list_map_comboboxes->at(current_source)->insert(key, enable_throttle_controlComboBox);
                }
            else if (match_AGC_enabled.hasMatch())
                {
                    AGCComboBox = new QComboBox();
                    AGCComboBox->setObjectName("AGC_enabledComboBox");
                    AGCComboBox->addItem(tr("false"));
                    AGCComboBox->addItem(tr("true"));
                    AGCComboBox->setCurrentIndex(0);
                    list_map_comboboxes->at(current_source)->insert(key, AGCComboBox);
                }
            else if (match_item_type.hasMatch())
                {
                    if (match_File_Signal_Source.hasMatch())
                        {
                            item_typeComboBox = new QComboBox();
                            item_typeComboBox->setObjectName("item_typeComboBox");
                            item_typeComboBox->addItem(tr("byte"));
                            item_typeComboBox->addItem(tr("ibyte"));
                            item_typeComboBox->addItem(tr("short"));
                            item_typeComboBox->addItem(tr("cshort"));
                            item_typeComboBox->addItem(tr("float"));
                            item_typeComboBox->addItem(tr("gr_complex"));
                            item_typeComboBox->setCurrentIndex(5);
                            list_map_comboboxes->at(current_source)->insert(key, item_typeComboBox);
                        }
                    else if (match_UHD_Signal_Source.hasMatch())
                        {
                            item_typeComboBox = new QComboBox();
                            item_typeComboBox->setObjectName("item_typeComboBox");
                            item_typeComboBox->addItem(tr("cbyte"));
                            item_typeComboBox->addItem(tr("cshort"));
                            item_typeComboBox->addItem(tr("gr_complex"));
                            item_typeComboBox->setCurrentIndex(1);
                            list_map_comboboxes->at(current_source)->insert(key, item_typeComboBox);
                        }
                    else
                        {
                            item_typeComboBox = new QComboBox();
                            item_typeComboBox->setObjectName("item_typeComboBox");
                            item_typeComboBox->addItem(tr("byte"));
                            item_typeComboBox->addItem(tr("ibyte"));
                            item_typeComboBox->addItem(tr("short"));
                            item_typeComboBox->addItem(tr("cshort"));
                            item_typeComboBox->addItem(tr("float"));
                            item_typeComboBox->addItem(tr("gr_complex"));
                            item_typeComboBox->setCurrentIndex(5);
                            list_map_comboboxes->at(current_source)->insert(key, item_typeComboBox);
                        }
                }
            else
                {
                    if (match_dump_filename.hasMatch())
                        {
                            list_map_implementation->at(current_source)->insert(key, new QLineEdit("./my_capture.dat"));
                        }
                    else if (match_samples.hasMatch())
                        {
                            list_map_implementation->at(current_source)->insert(key, new QLineEdit("0"));
                        }
                    else if (match_gain.hasMatch())
                        {
                            if (match_UHD_Signal_Source.hasMatch())
                                {
                                    list_map_implementation->at(current_source)->insert(key, new QLineEdit("50"));
                                }
                            if (match_Osmosdr_Signal_Source.hasMatch() || match_RtlTcp_Signal_Source.hasMatch())
                                {
                                    list_map_implementation->at(current_source)->insert(key, new QLineEdit("40"));
                                }
                        }
                    else if (match_rf_gain.hasMatch())
                        {
                            list_map_implementation->at(current_source)->insert(key, new QLineEdit("40"));
                        }
                    else if (match_if_gain.hasMatch())
                        {
                            list_map_implementation->at(current_source)->insert(key, new QLineEdit("40"));
                        }
                    else
                        {
                            list_map_implementation->at(current_source)->insert(key, new QLineEdit());
                        }

                    if (match_freq.hasMatch())
                        {
                            list_map_implementation->at(current_source)->value(key)->setToolTip("RF front-end center frequency in [Hz]");
                        }
                    QRegularExpressionMatch match_sampling_frequency2 = key_sampling_frequency.match(key);
                    if (match_sampling_frequency2.hasMatch())
                        {
                            list_map_implementation->at(current_source)->value(key)->setToolTip("Original Signal sampling frequency in [sps]");
                        }
                }

            if (!match_dump_filename.hasMatch() && !match_filename.hasMatch() && !match_dump.hasMatch() && !match_sampling_frequency.hasMatch() && !match_device_address.hasMatch() && !match_freq.hasMatch())
                {
                    group_pars << key;
                }
        }

    foreach (const QString key, group_main_pars)
        {
            layout->addWidget(new QLabel(key), row, col++);
            layout->addWidget(list_map_implementation->at(current_source)->value(key), row, col++);
            if ((col % max_col) == 0)
                {
                    col = 0;
                    row++;
                }
        }

    foreach (const QString key, group_pars)
        {
            QRegularExpressionMatch match_repeat2 = key_repeat.match(key);
            QRegularExpressionMatch match_enable_throttle_control2 = key_enable_throttle_control.match(key);
            QRegularExpressionMatch match_item_type2 = key_item_type.match(key);
            QRegularExpressionMatch match_subdevice2 = key_subdevice.match(key);
            QRegularExpressionMatch match_clock_source2 = key_clock_source.match(key);
            QRegularExpressionMatch match_AGC_enabled2 = key_AGC_enabled.match(key);

            if (match_repeat2.hasMatch() || match_enable_throttle_control2.hasMatch() || match_item_type2.hasMatch() || match_subdevice2.hasMatch() || match_clock_source2.hasMatch() || match_AGC_enabled2.hasMatch())
                {
                    layout->addWidget(new QLabel(key), row, col++);
                    layout->addWidget(list_map_comboboxes->at(current_source)->value(key), row, col++);
                }
            else
                {
                    layout->addWidget(new QLabel(key), row, col++);
                    layout->addWidget(list_map_implementation->at(current_source)->value(key), row, col++);
                }

            if ((col % max_col) == 0)
                {
                    col = 0;
                    row++;
                }
        }

    foreach (const QString key, group_dump)
        {
            QRegularExpressionMatch match4 = key_dump.match(key);
            if (!match4.hasMatch())
                {
                    layout->addWidget(new QLabel(key), row, col++);
                    layout->addWidget(list_map_implementation->at(current_source)->value(key), row, col++);
                }
            else
                {
                    layout->addWidget(new QLabel(key), row, col++);
                    layout->addWidget(list_map_comboboxes->at(current_source)->value(key), row, col++);
                }

            if ((col % max_col) == 0)
                {
                    col = 0;
                    row++;
                }
        }
    layout->addWidget(update_button, row, max_col - 1, 1, 1, Qt::AlignRight);
    grid_groupbox->setLayout(layout);
    connect(update_button, SIGNAL(clicked()), this, SLOT(add_sub_blocks()));
    grid_groupbox->setObjectName("MainGroupBox");
    return grid_groupbox;
}


void Signal_Source::add_sub_blocks()
{
    QPushButton *sender_button = qobject_cast<QPushButton *>(sender());
    QWidget *page_widget = new (QWidget);
    int current_source = block_tab_widget->currentIndex();
    QString share_key;
    if (block_tab_widget->count() > 1)
        {
            share_key = "SignalSource" + QString::number(current_source) + ".RF_channels";
        }
    else
        {
            share_key = "SignalSource.RF_channels";
        }
    if (list_map_implementation->at(current_source)->contains(share_key))
        {
            int value = list_map_implementation->at(current_source)->value(share_key)->text().toInt();
            if (value == 0)
                {
                    value = 1;
                }
            emit share_rf_channels(share_key, value);
        }
    else
        {
            emit share_rf_channels(share_key, 1);
        }
    page_widget = sender_button->parentWidget()->parentWidget();
    if (sender_button != NULL)
        {
            //Remove Spacer
            sender_button->parentWidget()->parentWidget()->layout()->removeItem(list_spacer->at(current_source));
            QList<QGroupBox *> subbox_list = page_widget->findChildren<QGroupBox *>("SuBGroupBox");
            list_map_sub->at(current_source)->clear();
            //mapSub->clear();
            foreach (QGroupBox *p, subbox_list)
                {
                    p->setParent(NULL);
                    p->deleteLater();
                }
            if ((list_map_subgroup_keys->count() - 1 >= current_source))
                {
                    foreach (const QString subgroup, list_map_subgroup_keys->at(current_source)->keys())
                        {
                            if (list_map_implementation->at(current_source)->contains(subgroup))
                                {
                                    int sub_group_count = list_map_implementation->at(current_source)->value(subgroup)->text().toInt();
                                    if (sub_group_count > 1)
                                        {
                                            for (int i = 0; i < sub_group_count; i++)
                                                {
                                                    //list_map_sub->append(new QMap<QString, QLineEdit *>);
                                                    QStringList sub_keys;
                                                    QStringList sub_keys_updated;
                                                    sub_keys = list_map_subgroup_child_keys->at(current_source)->value(subgroup);
                                                    foreach (QString key, sub_keys)
                                                        {
                                                            sub_keys_updated.append(key + QString::number(i));
                                                        }
                                                    QGroupBox *sub_box = sub_box_implementation(subgroup + QString::number(i), sub_keys_updated);
                                                    sender_button->parentWidget()->parentWidget()->layout()->addWidget(sub_box);
                                                }
                                        }
                                    else
                                        {
                                            if (sub_group_count == 1)
                                                {
                                                    //list_map_sub->append(new QMap<QString, QLineEdit *>);
                                                    QStringList sub_keys;
                                                    sub_keys = list_map_subgroup_child_keys->at(current_source)->value(subgroup);
                                                    QGroupBox *sub_box = sub_box_implementation(subgroup, sub_keys);
                                                    sender_button->parentWidget()->parentWidget()->layout()->addWidget(sub_box);
                                                }
                                        }
                                }
                        }
                }
            //Add spacer
            sender_button->parentWidget()->parentWidget()->layout()->addItem(list_spacer->at(current_source));
        }
}


QGroupBox *Signal_Source::sub_box_implementation(QString boxname, QStringList group_keys)
{
    QGroupBox *grid_groupbox;
    grid_groupbox = new QGroupBox(boxname);
    QGridLayout *layout = new QGridLayout;
    uint max_col = 4;
    uint row = 0;
    uint col = 0;
    int current_source = block_tab_widget->currentIndex();
    foreach (QString key, group_keys)
        {
            list_map_sub->at(current_source)->insert(key, new QLineEdit());
            QRegularExpression key_re1("^(SignalSource)[0-9]{0,}.(freq)[0-9]{0,}$");
            QRegularExpressionMatch match = key_re1.match(key);
            if (match.hasMatch())
                {
                    list_map_sub->at(current_source)->value(key)->setToolTip("RF front-end center frequency in [Hz]");
                }
        }
    foreach (const QString key, group_keys)
        {
            layout->addWidget(new QLabel(key), row, col++);
            layout->addWidget(list_map_sub->at(current_source)->value(key), row, col++);
            if ((col % max_col) == 0)
                {
                    col = 0;
                    row++;
                }
        }
    grid_groupbox->setLayout(layout);
    grid_groupbox->setObjectName("SuBGroupBox");
    return grid_groupbox;
}


QMap<QString, QString> *Signal_Source::get_options()
{
    QMap<QString, QString> *map_options;
    map_options = new QMap<QString, QString>;
    foreach (QString key, map_generic->keys())
        {
            map_options->insert(key, map_generic->value(key)->text());
        }
    for (int i = 0; i < list_map_implementation->count(); i++)
        {
            foreach (QString key, list_map_implementation->at(i)->keys())
                {
                    map_options->insert(key, list_map_implementation->at(i)->value(key)->text());
                }
        }
    for (int i = 0; i < list_map_sub->count(); i++)
        {
            foreach (QString key, list_map_sub->at(i)->keys())
                {
                    map_options->insert(key, list_map_sub->at(i)->value(key)->text());
                }
        }
    for (int i = 0; i < list_map_comboboxes->count(); i++)
        {
            foreach (QString key, list_map_comboboxes->at(i)->keys())
                {
                    map_options->insert(key, list_map_comboboxes->at(i)->value(key)->currentText());
                }
        }
    return map_options;
}
