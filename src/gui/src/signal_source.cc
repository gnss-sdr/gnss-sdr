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

Signal_Source::Signal_Source(QWidget * parent, QString block_name_, QString dir_path_) : QWidget(parent), block_name(block_name_), dir_path(dir_path_)
{
    map_generic = new QMap<QString, QLineEdit *>;
    map_subgroup_list = new QMap<int, QStringList*>;
    list_map_implementation = new QList < QMap<QString, QLineEdit *> *>;
    list_map_sub = new QList < QMap<QString, QLineEdit *> *>;
    list_map_subgroup_keys = new QList < QMap<QString, QString > *>;
    list_map_subgroup_child_keys = new QList < QMap<QString, QStringList> *>;
    list_map_dump = new QList < QMap<QString, QComboBox *> *>;
    list_spacer = new QList <QSpacerItem *>;
    block_dir = new QDir(dir_path);
    block_dir->setFilter(QDir::NoDotAndDotDot | QDir::NoSymLinks | QDir::Files);
    QStringList name_filters;
    name_filters << "*.ini";
    block_dir->setNameFilters(name_filters);
    foreach (QFileInfo item, block_dir->entryInfoList())
        {
            if (item.isFile() && (item.fileName() != "generic.ini") )
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
    foreach(QGroupBox *item, block_box_list)
        {
            block_scroll_area_widget_layout->addWidget(item);
        }
    block_scroll_area_widget_layout->addWidget(block_tab_widget);
    QVBoxLayout * layout = new QVBoxLayout;
    layout->addWidget(block_scroll_area);
    QString homeLocation = QStandardPaths::locate(QStandardPaths::DocumentsLocation, QString("test"), QStandardPaths::LocateDirectory);
    setLayout(layout);
}


QGroupBox* Signal_Source::box_generic(QString boxname, QSettings *ini_settings)
{
    QGroupBox * grid_groupbox;
    grid_groupbox = new QGroupBox(boxname);
    QPushButton * update_button = new QPushButton("Update");
    QGridLayout * layout = new QGridLayout;
    uint max_col = 4;
    uint row = 0;
    uint col = 0;

    ini_settings->beginGroup("Generic");
    //map_generic = new QMap<QString, QLineEdit *>;
    foreach(QString key,ini_settings->allKeys())
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
    layout->addWidget(update_button, row, max_col-1, 1, 1, Qt::AlignRight);
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
                    emit share_source_count_value("Receiver.sources_count",0);
                }
            else
                {
                    emit share_source_count_value("Receiver.sources_count",map_generic->value("Receiver.sources_count")->text().toInt());
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
                    list_map_dump->clear();
                    map_subgroup_list->clear();
                    list_map_subgroup_keys->clear();
                    list_map_subgroup_child_keys->clear();
                    list_spacer->clear();
                    block_tab_widget->widget(i-1)->deleteLater();
                    block_tab_widget->removeTab(i-1);
                }
            for (int i = 0; i < sources_count; i++)
                {
                    block_tab_widget->addTab(new QWidget (),block_name+QString::number(i));
                    //create a main map for each source
                    list_map_implementation->append(new QMap<QString, QLineEdit *>);
                    //create a sub map for each source
                    list_map_sub->append(new QMap<QString, QLineEdit *>);
                    list_map_dump->append(new QMap<QString, QComboBox *>);
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
                    QComboBox * source_combobox;
                    source_combobox = new QComboBox();
                    source_combobox->setObjectName("sourceComboBox");
                    source_combobox->addItem("Select");
                    foreach (QString implentation, block_implementation_list)
                        {
                            source_combobox->addItem(implentation);
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
    QSettings * implementation_options;
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
    if (block_tab_widget->count() > 1 )
        {
            foreach(QString key, main_keys)
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

    QComboBox * sender_combobox = qobject_cast<QComboBox*>(sender());
    if( sender_combobox != NULL )
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
            QGroupBox * mainBox = box_implementation(top_group,main_keys_updated);
            QList<QGroupBox*> boxList = sender_combobox->parentWidget()->layout()->parentWidget()->findChildren<QGroupBox*>(QString(), Qt::FindDirectChildrenOnly);
            foreach(QGroupBox * box, boxList)
                {
                    sender_combobox->parentWidget()->layout()->removeWidget(box);
                    delete box;
                }
            //Remove spacer
            sender_combobox->parentWidget()->layout()->removeItem(list_spacer->at(current_source));
            sender_combobox->parentWidget()->layout()->addWidget(mainBox);
            sender_combobox->parentWidget()->layout()->setAlignment(mainBox,Qt::AlignTop);
            //Add Spacer
            sender_combobox->parentWidget()->layout()->addItem(list_spacer->at(current_source));
        }
    map_subgroup_list->value(current_source)->operator =(implementation_options->childGroups());
    if (!((*map_subgroup_list->value(current_source)).empty()))
        {
            foreach (QString sub_group,*map_subgroup_list->value(current_source))
                {
                    QStringList temp = main_keys_updated.filter(sub_group);
                    list_map_subgroup_keys->at(current_source)->insert(temp.at(0), temp.at(0));
                    implementation_options->beginGroup(sub_group);
                    QStringList sub_keys = implementation_options->childKeys();
                    if (block_tab_widget->count() > 1 )
                        {
                            foreach(QString key,sub_keys)
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


QGroupBox* Signal_Source::box_implementation(QString boxname, QStringList group_keys)
{
    QGroupBox * grid_groupbox;
    grid_groupbox = new QGroupBox(boxname);
    QPushButton * update_button = new QPushButton("Update");
    QGridLayout * layout = new QGridLayout;
    uint max_col = 4;
    QComboBox * dumpComboBox;
    uint row = 0;
    uint col = 0;
    int current_source = block_tab_widget->currentIndex();
    QRegularExpression key_dump( "dump(?!(_filename))" );
    foreach(QString key,group_keys)
        {
            //Insert Items in main map for this source
            QRegularExpressionMatch match_dump = key_dump.match(key);
            if (match_dump.hasMatch())
                {
                    dumpComboBox = new QComboBox();
                    dumpComboBox->setObjectName("dumpComboBox");
                    dumpComboBox->addItem(tr("false"));
                    dumpComboBox->addItem(tr("true"));
                    dumpComboBox->setCurrentIndex(0);
                    list_map_dump->at(current_source)->insert(key, dumpComboBox);
                }
            else
                {
                    list_map_implementation->at(current_source)->insert(key, new QLineEdit());
                    QRegularExpression key_re1( "^(SignalSource)[0-9]{0,}.(freq)$" );
                    QRegularExpressionMatch match1 = key_re1.match(key);
                    if (match1.hasMatch())
                        {
                            list_map_implementation->at(current_source)->value(key)->setToolTip("RF front-end center frequency in [sps]");
                        }
                    QRegularExpression key_re2( "^(SignalSource)[0-9]{0,}.(sampling_frequency)$" );
                    QRegularExpressionMatch match2 = key_re2.match(key);
                    if (match2.hasMatch())
                        {
                            list_map_implementation->at(current_source)->value(key)->setToolTip("Original Signal sampling frequency in [sps]");
                        }
                }
        }
    foreach (const QString key, group_keys)
        {
            layout->addWidget(new QLabel(key), row, col++);

            QRegularExpressionMatch match4 = key_dump.match(key);
            if (match4.hasMatch())
                {
                    layout->addWidget(list_map_dump->at(current_source)->value(key), row, col++);
                }
            else
                {
                    layout->addWidget(list_map_implementation->at(current_source)->value(key), row, col++);
                }

            //layout->addWidget(mapImp->value(mapKey),row,col++);
            if ((col % max_col) == 0)
                {
                    col = 0;
                    row++;
                }
        }
    layout->addWidget(update_button, row, max_col-1, 1, 1, Qt::AlignRight);
    grid_groupbox->setLayout(layout);
    connect(update_button, SIGNAL(clicked()), this, SLOT(add_sub_blocks()));
    grid_groupbox->setObjectName("MainGroupBox");
    return grid_groupbox;
}


void Signal_Source::add_sub_blocks()
{
    QPushButton * sender_button = qobject_cast<QPushButton*>(sender());
    QWidget * page_widget=  new (QWidget);
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
            emit share_rf_channels(share_key,value);
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
            foreach(QGroupBox * p, subbox_list)
                {
                    p->setParent(NULL);
                    p->deleteLater();
                }
            if ((list_map_subgroup_keys->count() - 1 >= current_source))
                {
                    foreach(const QString subgroup,list_map_subgroup_keys->at(current_source)->keys())
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
                                                    QGroupBox * sub_box = sub_box_implementation(subgroup + QString::number(i), sub_keys_updated);
                                                    sender_button->parentWidget()->parentWidget()->layout()->addWidget(sub_box);
                                                }
                                        }
                                    else
                                        {
                                            if(sub_group_count == 1)
                                                {
                                                    //list_map_sub->append(new QMap<QString, QLineEdit *>);
                                                    QStringList sub_keys;
                                                    sub_keys = list_map_subgroup_child_keys->at(current_source)->value(subgroup);
                                                    QGroupBox * sub_box = sub_box_implementation(subgroup, sub_keys);
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


QGroupBox* Signal_Source::sub_box_implementation(QString boxname, QStringList group_keys)
{
    QGroupBox * grid_groupbox;
    grid_groupbox = new QGroupBox(boxname);
    QGridLayout * layout = new QGridLayout;
    uint max_col = 4;
    uint row = 0;
    uint col = 0;
    int current_source = block_tab_widget->currentIndex();
    foreach(QString key,group_keys)
        {
            list_map_sub->at(current_source)->insert(key, new QLineEdit());
            QRegularExpression key_re1( "^(SignalSource)[0-9]{0,}.(freq)[0-9]{0,}$" );
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


QMap<QString, QString >* Signal_Source::get_options()
{
    QMap<QString, QString > * map_options;
    map_options  = new QMap<QString, QString >;
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
    for (int i = 0; i < list_map_dump->count(); i++)
        {
            foreach (QString key, list_map_dump->at(i)->keys())
                {
                    map_options->insert(key, list_map_dump->at(i)->value(key)->currentText());
                }
        }
    return map_options;
}
