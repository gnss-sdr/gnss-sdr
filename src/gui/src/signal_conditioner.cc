/*!
 * \file signal_conditioner.cc
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

#include "signal_conditioner.h"
#include "signal_source.h"

Signal_Conditioner::Signal_Conditioner(QWidget *parent, QString block_name_, QString dir_path_) : QWidget(parent), block_name(block_name_), dir_path(dir_path_)
{

    map_source_channel = new QMap <QString,int> ;
    list_map_implementation = new QList < QMap<QString, QLineEdit *> *>;
    list_map_sub = new QList < QMap<QString, QLineEdit *> *>;
    list_map_pass = new QList < QMap<QString, QLineEdit *> *>;
    list_sub_tabwidget = new QList <QTabWidget *>;
    list_spacer = new QList <QSpacerItem *>;
    map_pass_through_flag =  new QMap <int,bool>;
    //Only used by input_filter
    map_subgroup_list =  new QMap<int, QStringList*>;
    //Only used by input_filter
    list_map_subgroup_child_keys = new QList < QMap<QString, QStringList> *>;
    list_map_subgroup_keys = new QList < QMap<QString, QString > *>;
    block_directory = new QDir(dir_path);
    list_subdirectory = new QList <QDir *>;
    blockImplentationList = new QList <QStringList*>;
    block_implentation_list_path = new QList <QStringList*>;
    block_directory->setFilter(QDir::NoDotAndDotDot | QDir::NoSymLinks | QDir::Dirs);
    QStringList directory_name_filters;
    directory_name_filters <<"DataTypeAdapter" << "InputFilter" <<"Resampler";
    block_directory->setNameFilters(directory_name_filters);
    subdirectories_paths = new QStringList;
    subdirectories_names = new QStringList;
    QStringList name_filters;
    name_filters<<"*.ini";
    foreach (QFileInfo item,block_directory->entryInfoList())
        {
            if (item.isDir() )
                {
                    subdirectories_names->append(item.baseName());
                    subdirectories_paths->append(item.filePath());
                    list_subdirectory->append(new QDir(item.filePath()));
                }
        }
    for (int i= 0 ; i< list_subdirectory->count();i++)
        {
            blockImplentationList->append(new QStringList);
            block_implentation_list_path->append(new QStringList);
        }
    int directory_index=0;
    foreach(QDir * dir,*list_subdirectory)
        {
            dir->setFilter(QDir::NoDotAndDotDot | QDir::NoSymLinks | QDir::Files);
            dir->setNameFilters(name_filters);
            foreach (QFileInfo item, dir->entryInfoList())
                {
                    if (item.isFile() && (item.fileName() != "generic.ini") )
                        {
                            blockImplentationList->at(directory_index)->append(item.baseName());
                            block_implentation_list_path->at(directory_index)->append(item.filePath());
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
    QVBoxLayout * layout = new QVBoxLayout;
    layout->addWidget(block_scroll_area);
    setLayout(layout);
}

void Signal_Conditioner::listener_source_count(QString key, int value)
{
    if (key == "Receiver.sources_count")
        {
            if (value > 0)
                {
                    map_pass_through_flag->clear();
                    map_source_channel->clear();
                    list_map_implementation->clear();
                    list_map_pass->clear();
                    list_map_sub->clear();
                    list_sub_tabwidget->clear();
                    list_spacer->clear();
                    map_subgroup_list->clear();
                    list_map_subgroup_child_keys->clear();
                    list_map_subgroup_keys->clear();
                    for (int i=block_tab_widget->count();i>0;i--)
                        {
                            block_tab_widget->widget(i-1)->deleteLater();
                            block_tab_widget->removeTab(i-1);
                        }
                    if (value > 1)
                        {
                            multiple_conditioners=true;
                            for (int i=0;i<value;i++)
                                {
                                    block_tab_widget->addTab(new QWidget (),block_name+QString::number(i));
                                    //By default 1 Signal_Conditioner for each source
                                    map_source_channel->insert("SignalSource"+QString::number(i)+".RF_channels",1);
                                    //insert a QStringList in map for each Signal_Conditioner for keeping the subgrouplist
                                    map_subgroup_list->insert(i, new QStringList);
                                    list_map_subgroup_child_keys->append(new QMap<QString, QStringList>);
                                    list_map_subgroup_keys->append(new QMap<QString, QString>);
                                   //create a sub map for each source
                                   list_map_sub->append(new QMap<QString, QLineEdit *>);
                                   //create a map for pass through
                                   list_map_pass->append(new QMap<QString, QLineEdit *>);
                                   map_pass_through_flag->insert(i,true);
                                   populate_page(i);
                                }
                        }
                    else
                        {
                            multiple_conditioners=false;
                            //By default 1 Signal_Conditioner for each source
                            map_source_channel->insert("SignalSource.RF_channels",1);
                            for (int i=0;i<value;i++)
                                {
                                   //block_tab_widget->addTab(new QWidget (),block_name+QString::number(i));
                                   block_tab_widget->addTab(new QWidget (),block_name);
                                   //insert a QStringList in map for each Signal_Conditioner for keeping the subgrouplist
                                   map_subgroup_list->insert(i, new QStringList);
                                   list_map_subgroup_child_keys->append(new QMap<QString, QStringList>);
                                   list_map_subgroup_keys->append(new QMap<QString, QString>);
                                   //create a sub map for each source
                                   list_map_sub->append(new QMap<QString, QLineEdit *>);
                                   //create a map for pass through
                                   list_map_pass->append(new QMap<QString, QLineEdit *>);
                                   map_pass_through_flag->insert(i,true);
                                   populate_page(i);
                                }
                        }
                }
        }
}
void Signal_Conditioner::listener_rf_channel(QString key, int key_value)
{
    if (map_source_channel->contains(key))
        {
            if (key_value > map_source_channel->value(key) )
                {
                    int current_count = block_tab_widget->count();
                    int to_add = key_value - map_source_channel->value(key);
                    int value = current_count + to_add;
                    //###
                    map_pass_through_flag->clear();
                    map_source_channel->clear();
                    list_map_implementation->clear();
                    list_map_pass->clear();
                    list_map_sub->clear();
                    list_sub_tabwidget->clear();
                    list_spacer->clear();
                    map_subgroup_list->clear();
                    list_map_subgroup_child_keys->clear();
                    list_map_subgroup_keys->clear();
                    for (int i=block_tab_widget->count();i>0;i--)
                        {
                            block_tab_widget->widget(i-1)->deleteLater();
                            block_tab_widget->removeTab(i-1);
                        }

                    if (value > 1)
                        {
                            multiple_conditioners=true;
                            for (int i=0;i<value;i++)
                                {
                                    block_tab_widget->addTab(new QWidget (),block_name+QString::number(i));
                                    //By default 1 Signal_Conditioner for each source
                                    map_source_channel->insert("SignalSource"+QString::number(i)+".RF_channels",1);
                                    //insert a QStringList in map for each Signal_Conditioner for keeping the subgrouplist
                                    map_subgroup_list->insert(i, new QStringList);
                                    list_map_subgroup_child_keys->append(new QMap<QString, QStringList>);
                                    list_map_subgroup_keys->append(new QMap<QString, QString>);
                                   //create a sub map for each source
                                   list_map_sub->append(new QMap<QString, QLineEdit *>);
                                   //create a map for pass through
                                   list_map_pass->append(new QMap<QString, QLineEdit *>);
                                   map_pass_through_flag->insert(i,true);
                                   populate_page(i);
                                }
                        }
                    else
                        {
                            multiple_conditioners=false;
                            //By default 1 Signal_Conditioner for each source
                            map_source_channel->insert("SignalSource.RF_channels",1);
                            for (int i=0;i<value;i++)
                                {
                                   //block_tab_widget->addTab(new QWidget (),block_name+QString::number(i));
                                   block_tab_widget->addTab(new QWidget (),block_name);
                                   //insert a QStringList in map for each Signal_Conditioner for keeping the subgrouplist
                                   map_subgroup_list->insert(i, new QStringList);
                                   list_map_subgroup_child_keys->append(new QMap<QString, QStringList>);
                                   list_map_subgroup_keys->append(new QMap<QString, QString>);
                                   //create a sub map for each source
                                   list_map_sub->append(new QMap<QString, QLineEdit *>);
                                   //create a map for pass through
                                   list_map_pass->append(new QMap<QString, QLineEdit *>);
                                   map_pass_through_flag->insert(i,true);
                                   populate_page(i);
                                }
                        }
                    //###
                    map_source_channel->remove(key);
                    map_source_channel->insert(key,key_value);
                }
            else if (key_value < map_source_channel->value(key))
                {
                    int current_count = block_tab_widget->count();
                    int to_del = map_source_channel->value(key)-key_value;
                    int value =  current_count - to_del;
                    //###
                    map_pass_through_flag->clear();
                    map_source_channel->clear();
                    list_map_implementation->clear();
                    list_map_pass->clear();
                    list_map_sub->clear();
                    list_sub_tabwidget->clear();
                    list_spacer->clear();
                    map_subgroup_list->clear();
                    list_map_subgroup_child_keys->clear();
                    list_map_subgroup_keys->clear();
                    for (int i=block_tab_widget->count();i>0;i--)
                        {
                            block_tab_widget->widget(i-1)->deleteLater();
                            block_tab_widget->removeTab(i-1);
                        }

                    if (value > 1)
                        {
                            multiple_conditioners=true;
                            for (int i=0;i<value;i++)
                                {
                                    block_tab_widget->addTab(new QWidget (),block_name+QString::number(i));
                                    //By default 1 Signal_Conditioner for each source
                                    map_source_channel->insert("SignalSource"+QString::number(i)+".RF_channels",1);
                                    //insert a QStringList in map for each Signal_Conditioner for keeping the subgrouplist
                                    map_subgroup_list->insert(i, new QStringList);
                                    list_map_subgroup_child_keys->append(new QMap<QString, QStringList>);
                                    list_map_subgroup_keys->append(new QMap<QString, QString>);
                                   //create a sub map for each source
                                   list_map_sub->append(new QMap<QString, QLineEdit *>);
                                   //create a map for pass through
                                   list_map_pass->append(new QMap<QString, QLineEdit *>);
                                   map_pass_through_flag->insert(i,true);
                                   populate_page(i);
                                }
                        }
                    else
                        {
                            multiple_conditioners=false;
                            //By default 1 Signal_Conditioner for each source
                            map_source_channel->insert("SignalSource.RF_channels",1);
                            for (int i=0;i<value;i++)
                                {
                                   //block_tab_widget->addTab(new QWidget (),block_name+QString::number(i));
                                   block_tab_widget->addTab(new QWidget (),block_name);
                                   //insert a QStringList in map for each Signal_Conditioner for keeping the subgrouplist
                                   map_subgroup_list->insert(i, new QStringList);
                                   list_map_subgroup_child_keys->append(new QMap<QString, QStringList>);
                                   list_map_subgroup_keys->append(new QMap<QString, QString>);
                                   //create a sub map for each source
                                   list_map_sub->append(new QMap<QString, QLineEdit *>);
                                   //create a map for pass through
                                   list_map_pass->append(new QMap<QString, QLineEdit *>);
                                   map_pass_through_flag->insert(i,true);
                                   populate_page(i);
                                }
                        }

                    map_source_channel->remove(key);
                    map_source_channel->insert(key,key_value);
                }
        }
}

void Signal_Conditioner::populate_page(int index)
{
    QVBoxLayout * layout;
    layout = new QVBoxLayout();
    block_tab_widget->widget(index)->setLayout(layout);
    QGroupBox * pass_groupbox= box_pass_through("Pass_Through",index);
    //Add a combobox
    QComboBox * select_combobox;
    select_combobox= new QComboBox();
    select_combobox->setObjectName("selectComboBox");
    select_combobox->addItem("Signal_Conditioner");
    select_combobox->addItem("Pass_Through");
    select_combobox->setCurrentIndex(1);
    layout->addWidget(select_combobox);
    layout->addWidget(pass_groupbox);
    pass_groupbox->setEnabled(false);
    connect(select_combobox,SIGNAL(currentIndexChanged(int)),this,SLOT(enable_disable(int)));
    add_subtab(index);
    populate_subtab(index);
    for (int i = 0 ; i < list_sub_tabwidget->count();i++)
        {
            list_sub_tabwidget->at(i)->setEnabled(false);
        }
}

void Signal_Conditioner::add_subtab(int index)
{
    QTabWidget * sub_tabwidget;
    sub_tabwidget =  new QTabWidget();
    sub_tabwidget->setObjectName("SubTab");
    if (multiple_conditioners == true)
        {
            for (int i=0;i<subdirectories_names->count();i++)
                {
                    sub_tabwidget->addTab(new QWidget(),subdirectories_names->at(i)+QString::number(index));
                    //create a main map for each page of sub_tab
                    list_map_implementation->append(new QMap<QString, QLineEdit *>);
                    //Add spacer for each page
                    list_spacer->append(new QSpacerItem(10, 10, QSizePolicy::Minimum, QSizePolicy::Expanding));
                }
        }
    else
        {
            for (int i=0;i<subdirectories_names->count();i++)
                {
                    sub_tabwidget->addTab(new QWidget(),subdirectories_names->at(i));
                    //create a main map for each page of sub_tab
                    list_map_implementation->append(new QMap<QString, QLineEdit *>);
                    //Add spacer for each page
                    list_spacer->append(new QSpacerItem(10, 10, QSizePolicy::Minimum, QSizePolicy::Expanding));
                }
        }
    list_sub_tabwidget->append(sub_tabwidget);
    block_tab_widget->widget(index)->layout()->addWidget(sub_tabwidget);
}

void Signal_Conditioner::populate_subtab(int index)
{
    int sub_pages_count = list_sub_tabwidget->at(index)->count();
    for (int i=0;i<sub_pages_count;i++)
        {
            QVBoxLayout * layout = new QVBoxLayout();
            list_sub_tabwidget->at(index)->widget(i)->setLayout(layout);
            //Add a combobox
            QComboBox * source_combobox;
            source_combobox= new QComboBox();
            source_combobox->setObjectName("sourceComboBox");
            source_combobox->addItem("Select");
            foreach (QString implementation, *(blockImplentationList->at(i)))
                {
                    source_combobox->addItem(implementation);
                }
            layout->addWidget(source_combobox);
            if (i==0)
                {
                    connect(source_combobox,SIGNAL(currentIndexChanged(QString)),this,SLOT(update_data_adapter(QString)));
                }
            else if (i==1)
                {
                    connect(source_combobox,SIGNAL(currentIndexChanged(QString)),this,SLOT(update_input_filter(QString)));
                }
            else if (i==2)
                {
                    connect(source_combobox,SIGNAL(currentIndexChanged(QString)),this,SLOT(update_resampler(QString)));
                }
            layout->addItem(list_spacer->at(index*3+i));
        }
}

void Signal_Conditioner::update_data_adapter(QString sourceImpl)
{
    //basic
    int current_source = block_tab_widget->currentIndex();
    int sub_tab_index = list_sub_tabwidget->at(current_source)->currentIndex();
    int map_index = current_source*3 + sub_tab_index;
    QString main_group;
    QStringList main_keys_updated;
    QStringList main_keys;
    QString implementation_value;
    QString source_settings;
    source_settings = list_subdirectory->at(0)->path()+"/"+sourceImpl+".ini";
    if (!block_implentation_list_path->at(0)->contains(source_settings))
        {
            return;
        }
    else
        {
            QSettings * implementation_options;
            implementation_options = new QSettings(source_settings, QSettings::IniFormat);
            QStringList groupList = implementation_options->childGroups();
            if (groupList.empty())
                {
                    return;
                }
            main_group= groupList.at(0);
            implementation_value = groupList.at(0);
            implementation_options->beginGroup(main_group);
            main_keys = implementation_options->childKeys();
            implementation_options->endGroup();
            if (multiple_conditioners == true)
                {
                    foreach(QString key,main_keys)
                        {
                            QStringList temp = key.split(".");
                            QString temp_key=temp.at(0)+QString::number(current_source)+"."+temp.at(1);
                            main_keys_updated.append(temp_key);
                        }
                }
            else
                {
                    main_keys_updated = main_keys;
                }
        }
    QComboBox * sender_combobox = qobject_cast<QComboBox*>(sender());
    if( sender_combobox != NULL )
        {
            //clear the main map for current source
            list_map_implementation->at(map_index)->clear();
            if (multiple_conditioners==true)
                {
                    list_map_implementation->at(map_index)->insert("DataTypeAdapter"+QString::number(current_source)+".implementation",new QLineEdit(implementation_value));
                }
            else
                {
                    list_map_implementation->at(map_index)->insert("DataTypeAdapter.implementation",new QLineEdit(implementation_value));
                }
            QGroupBox * main_box = box_implementation(main_group,main_keys_updated,map_index,false);
            QList<QGroupBox*> box_list = sender_combobox->parentWidget()->layout()->parentWidget()->findChildren<QGroupBox*>(QString(),Qt::FindDirectChildrenOnly);
            foreach(QGroupBox * box, box_list)
                {
                    sender_combobox->parentWidget()->layout()->removeWidget(box);
                    delete box;
                }
            //Remove spacer
            sender_combobox->parentWidget()->layout()->removeItem(list_spacer->at(map_index));
            sender_combobox->parentWidget()->layout()->addWidget(main_box);
            sender_combobox->parentWidget()->layout()->setAlignment(main_box,Qt::AlignTop);
            //Add Spacer
            sender_combobox->parentWidget()->layout()->addItem(list_spacer->at(map_index));
        }
}

void Signal_Conditioner::update_input_filter(QString sourceImpl)
{
    //basic
    int current_source = block_tab_widget->currentIndex();
    int sub_tab_index = list_sub_tabwidget->at(current_source)->currentIndex();
    int map_index = current_source*3 + sub_tab_index;
    QString block = list_sub_tabwidget->at(current_source)->tabText(sub_tab_index);
    QString main_group;
    QStringList main_keys_updated;
    QStringList main_keys;
    QString source_settings;
    QString implementation_value;
    QStringList sub_keys_updated;
    source_settings = list_subdirectory->at(1)->path()+"/"+sourceImpl+".ini";
    if (!block_implentation_list_path->at(1)->contains(source_settings))
        {
            return;
        }
    else
        {
            QSettings * implementation_options;
            implementation_options = new QSettings(source_settings, QSettings::IniFormat);
            QStringList group_list = implementation_options->childGroups();
            if (group_list.empty())
                {
                    return;
                }
            main_group= group_list.at(0);
            implementation_value = group_list.at(0);
            implementation_options->beginGroup(main_group);
            main_keys = implementation_options->childKeys();
            if (multiple_conditioners==true)
                {
                    foreach(QString key,main_keys)
                        {
                            QStringList temp = key.split(".");
                            QString temp_key=temp.at(0)+QString::number(current_source)+"."+temp.at(1);
                            main_keys_updated.append(temp_key);
                        }
                }
            else
                {
                    main_keys_updated=main_keys;
                }
            QComboBox * sender_combobox = qobject_cast<QComboBox*>(sender());
            if( sender_combobox != NULL )
                {
                    //clear the main map for current source
                    list_map_implementation->at(map_index)->clear();
                    //clear the subgroup map for current source
                    list_map_sub->at(current_source)->clear();
                    list_map_subgroup_keys->at(current_source)->clear();
                    list_map_subgroup_child_keys->at(current_source)->clear();
                    if (multiple_conditioners==true)
                        {
                            list_map_implementation->at(map_index)->insert("InputFilter"+QString::number(current_source)+".implementation",new QLineEdit(implementation_value));
                        }
                    else
                        {
                            list_map_implementation->at(map_index)->insert("InputFilter.implementation",new QLineEdit(implementation_value));
                        }
                    QGroupBox * main_box = box_implementation(main_group,main_keys_updated,map_index,true);
                    QList<QGroupBox*> box_list = sender_combobox->parentWidget()->layout()->parentWidget()->findChildren<QGroupBox*>(QString(),Qt::FindDirectChildrenOnly);
                    foreach(QGroupBox * box, box_list)
                        {
                            sender_combobox->parentWidget()->layout()->removeWidget(box);
                            delete box;
                        }
                    //Remove spacer
                    sender_combobox->parentWidget()->layout()->removeItem(list_spacer->at(map_index));
                    sender_combobox->parentWidget()->layout()->addWidget(main_box);
                    sender_combobox->parentWidget()->layout()->setAlignment(main_box,Qt::AlignTop);
                    //Add Spacer
                    sender_combobox->parentWidget()->layout()->addItem(list_spacer->at(map_index));
                }
            //Work for subgroups
            map_subgroup_list->value(current_source)->operator =(implementation_options->childGroups());
            if (!((*map_subgroup_list->value(current_source)).empty()))
                {
                    foreach (QString subGroup,*map_subgroup_list->value(current_source))
                        {
                            QStringList temp = main_keys_updated.filter(subGroup);
                            list_map_subgroup_keys->at(current_source)->insert(temp.at(0),temp.at(0));
                            implementation_options->beginGroup(subGroup);
                            QStringList subKeys = implementation_options->childKeys();
                            foreach(QString key,subKeys)
                                {
                                    QStringList temp = key.split(".");
                                    QString temp_key=temp.at(0)+QString::number(current_source)+"."+temp.at(1);
                                    sub_keys_updated.append(temp_key);
                                }
                            list_map_subgroup_child_keys->at(current_source)->insert(temp.at(0),sub_keys_updated);
                        }
                }
        }
}
void Signal_Conditioner::update_resampler(QString sourceImpl)
{
    //basic
    int current_source = block_tab_widget->currentIndex();
    int sub_tab_index = list_sub_tabwidget->at(current_source)->currentIndex();
    int map_index = current_source*3 + sub_tab_index;
    QString block = list_sub_tabwidget->at(current_source)->tabText(sub_tab_index);
    QString main_group;
    QStringList main_keys_updated;
    QStringList main_keys;
    QString implementation_value;
    QString source_settings;
    source_settings = list_subdirectory->at(2)->path()+"/"+sourceImpl+".ini";
    if (!block_implentation_list_path->at(2)->contains(source_settings))
        {
            return;
        }
    else
        {
            QSettings * implementation_options;
            implementation_options = new QSettings(source_settings, QSettings::IniFormat);
            QStringList group_list = implementation_options->childGroups();
            if (group_list.empty())
                {
                    return;
                }
            main_group= group_list.at(0);
            implementation_value = group_list.at(0);
            implementation_options->beginGroup(main_group);
            main_keys = implementation_options->childKeys();
            if (multiple_conditioners==true)
                {
                    foreach(QString key,main_keys)
                        {
                            QStringList temp = key.split(".");
                            QString temp_key=temp.at(0)+QString::number(current_source)+"."+temp.at(1);
                            main_keys_updated.append(temp_key);
                        }
                }
            else
                {
                    main_keys_updated = main_keys;
                }
        }
    QComboBox * source_combobox = qobject_cast<QComboBox*>(sender());
        if( source_combobox != NULL )
            {
                //clear the main map for current source
                list_map_implementation->at(map_index)->clear();
                if (multiple_conditioners==true)
                    {
                        list_map_implementation->at(map_index)->insert("Resampler"+QString::number(current_source)+".implementation",new QLineEdit(implementation_value));
                    }
                else
                    {
                        list_map_implementation->at(map_index)->insert("Resampler.implementation",new QLineEdit(implementation_value));
                    }
                QGroupBox * main_box = box_implementation(main_group,main_keys_updated,map_index,false);
                QList<QGroupBox*> boxList = source_combobox->parentWidget()->layout()->parentWidget()->findChildren<QGroupBox*>(QString(),Qt::FindDirectChildrenOnly);
                foreach(QGroupBox * box, boxList)
                    {
                        source_combobox->parentWidget()->layout()->removeWidget(box);
                        delete box;
                    }
                //Remove spacer
                source_combobox->parentWidget()->layout()->removeItem(list_spacer->at(map_index));
                source_combobox->parentWidget()->layout()->addWidget(main_box);
                source_combobox->parentWidget()->layout()->setAlignment(main_box,Qt::AlignTop);
                //Add Spacer
                source_combobox->parentWidget()->layout()->addItem(list_spacer->at(map_index));
            }
}

QGroupBox* Signal_Conditioner::box_implementation(QString boxname, QStringList current_group_Keys, int map_index, bool add_button)
{
    QGroupBox * grid_groupbox;
    grid_groupbox = new QGroupBox(boxname);
    QGridLayout * layout = new QGridLayout;
    uint max_col = 4;
    uint row=0;
    uint col=0;
    foreach(QString key,current_group_Keys)
    {
        //Insert Items in main map for this source
        list_map_implementation->at(map_index)->insert(key,new QLineEdit());
        QRegularExpression key_re1( "^(InputFilter)[0-9]{0,}.(sampling_frequency)$" );
        QRegularExpressionMatch match1 = key_re1.match(key);
        if (match1.hasMatch())
            {
                list_map_implementation->at(map_index)->value(key)->setToolTip("Filter input sampling frequency in [Hz]");
            }

        QRegularExpression key_re2( "^(Resampler)[0-9]{0,}.(sample_freq_)(in|out)$" );
        QRegularExpressionMatch match2 = key_re2.match(key);
        if (match2.hasMatch())
            {
                list_map_implementation->at(map_index)->value(key)->setToolTip("Sampling frequency in [Hz]");
            }
        QRegularExpression key_re3( "^(InputFilter)[0-9]{0,}.(IF)$" );
        QRegularExpressionMatch match3 = key_re3.match(key);
        if (match3.hasMatch())
            {
                list_map_implementation->at(map_index)->value(key)->setToolTip("Intermediate frequency in [Hz]");
            }
    }
    foreach (const QString key, current_group_Keys)
        {
            layout->addWidget(new QLabel(key),row,col++);
            layout->addWidget(list_map_implementation->at(map_index)->value(key),row,col++);
            if ((col % max_col) == 0)
                {
                    col=0;
                    row++;
                }
        }
    if (add_button)
        {
            QPushButton * update_button = new QPushButton("Update");
            layout->addWidget(update_button,row,max_col-1,1,1,Qt::AlignRight);
            connect(update_button,SIGNAL(clicked()),this,SLOT(add_sub_blocks()));
        }
    grid_groupbox->setLayout(layout);
    grid_groupbox->setObjectName("MainGroupBox");
    return grid_groupbox;
}

void Signal_Conditioner::add_sub_blocks()
{
    //basic
    int current_source = block_tab_widget->currentIndex();
    int sub_tab_index = list_sub_tabwidget->at(current_source)->currentIndex();
    int map_index = current_source*3 + sub_tab_index;
    QString block = list_sub_tabwidget->at(current_source)->tabText(sub_tab_index);
    QPushButton * sender_button = qobject_cast<QPushButton*>(sender());
    QWidget * page_widget=  new (QWidget);
    page_widget = sender_button->parentWidget()->parentWidget();
    if (sender_button != NULL)
        {
            //Remove Spacer
            sender_button->parentWidget()->parentWidget()->layout()->removeItem(list_spacer->at(map_index));
            QList<QGroupBox *> sub_box_list = page_widget->findChildren<QGroupBox *>("SuBGroupBox");
            list_map_sub->at(current_source)->clear();
            foreach(QGroupBox * p,sub_box_list)
                {
                    p->setParent(NULL);
                    p->deleteLater();
                }
            if ((list_map_subgroup_keys->count()-1 >= current_source))
                {
                    foreach(const QString subgroup,list_map_subgroup_keys->at(current_source)->keys())
                        {
                            if (list_map_implementation->at(map_index)->contains(subgroup))
                                {
                                    int subgroup_count = list_map_implementation->at(map_index)->value(subgroup)->text().toInt();
                                        if (subgroup_count > 1)
                                            {
                                                for (int i=0;i<subgroup_count;i++)
                                                    {
                                                        QStringList sub_keys;
                                                        QStringList sub_keys_updated;
                                                        sub_keys= list_map_subgroup_child_keys->at(current_source)->value(subgroup);
                                                        foreach (QString key,sub_keys)
                                                            {
                                                                sub_keys_updated.append(key+QString::number(i));
                                                            }
                                                        QGroupBox * sub_box =  sub_box_implementation(subgroup+QString::number(i),sub_keys_updated);
                                                        sender_button->parentWidget()->parentWidget()->layout()->addWidget(sub_box);
                                                    }
                                            }
                                        else if(subgroup_count == 1)
                                            {
                                                QStringList sub_keys;
                                                sub_keys= list_map_subgroup_child_keys->at(current_source)->value(subgroup);
                                                QGroupBox * sub_box =  sub_box_implementation(subgroup,sub_keys);
                                                sender_button->parentWidget()->parentWidget()->layout()->addWidget(sub_box);
                                            }
                                }
                        }
                }
            //Add spacer
            sender_button->parentWidget()->parentWidget()->layout()->addItem(list_spacer->at(map_index));
        }
}

QGroupBox* Signal_Conditioner::sub_box_implementation(QString boxname, QStringList current_group_keys)
{
    QGroupBox * grid_groupbox;
    grid_groupbox = new QGroupBox(boxname);
    QGridLayout * layout = new QGridLayout;
    uint max_col = 4;
    uint row=0;
    uint col=0;
    int current_source=block_tab_widget->currentIndex();
    foreach(QString key,current_group_keys)
        {
            list_map_sub->at(current_source)->insert(key,new QLineEdit());
            QRegularExpression key_re1( "^(InputFilter)[0-9]{0,}.(band_begin|band_end)[0-9]{0,}$" );
            QRegularExpressionMatch match1 = key_re1.match(key);
            if (match1.hasMatch())
                {
                    list_map_sub->at(current_source)->value(key)->setToolTip("Normalized frequency in range [0-1]");
                }
            QRegularExpression key_re2( "^(InputFilter)[0-9]{0,}.(ampl_begin|ampl_end)[0-9]{0,}$" );
            QRegularExpressionMatch match2 = key_re2.match(key);
            if (match2.hasMatch())
                {
                    list_map_sub->at(current_source)->value(key)->setToolTip("Desired amplitude in range [0-1]");
                }
        }
    foreach (const QString key, current_group_keys)
        {
            layout->addWidget(new QLabel(key),row,col++);
            layout->addWidget(list_map_sub->at(current_source)->value(key),row,col++);
            if ((col % max_col) == 0)
                {
                    col=0;
                    row++;
                }
        }
    grid_groupbox->setLayout(layout);
    grid_groupbox->setObjectName("SuBGroupBox");
    return grid_groupbox;
}

QGroupBox* Signal_Conditioner::box_pass_through(QString boxname, int current_source)
{
    QGroupBox * grid_groupbox;
    grid_groupbox = new QGroupBox(boxname);
    QString source_settings;
    QStringList main_keys;
    QStringList main_keys_updated;
    source_settings = block_directory->absolutePath()+"/pass_through.ini";
    QSettings * implementation_options;
    implementation_options = new QSettings(source_settings, QSettings::IniFormat);
    QStringList group_list = implementation_options->childGroups();
    if (group_list.empty())
        {
            return grid_groupbox;
        }
    QString main_group= group_list.at(0);
    implementation_options->beginGroup(main_group);
    main_keys = implementation_options->childKeys();
    implementation_options->endGroup();
    if (multiple_conditioners==true)
        {
            foreach(QString mkey,main_keys)
                {
                    QStringList temp = mkey.split(".");
                    QString temp_key=temp.at(0)+QString::number(current_source)+"."+temp.at(1);
                    main_keys_updated.append(temp_key);
                }
        }
    else
        {
            main_keys_updated=main_keys;
        }
    QGridLayout * layout = new QGridLayout;
    uint max_col = 4;
    uint row=0;
    uint col=0;
    foreach(QString key,main_keys_updated)
        {
            //Insert Items in main map for this source
            list_map_pass->at(current_source)->insert(key,new QLineEdit());
        }
    foreach (const QString key, main_keys_updated)
        {
            layout->addWidget(new QLabel(key),row,col++);
            layout->addWidget(list_map_pass->at(current_source)->value(key),row,col++);
            if ((col % max_col) == 0)
                {
                    col=0;
                    row++;
                }
        }
    grid_groupbox->setLayout(layout);
    grid_groupbox->setObjectName("PassGroupBox");
    return grid_groupbox;
}

void Signal_Conditioner::enable_disable(int index)
{
    int current_source = block_tab_widget->currentIndex();
    if (index==0)
        {
            map_pass_through_flag->remove(current_source);
            map_pass_through_flag->insert(current_source,false);
        }
    else
        {
            map_pass_through_flag->remove(current_source);
            map_pass_through_flag->insert(current_source,true);
        }
    QComboBox * sender_combobox = qobject_cast<QComboBox*>(sender());
    if( sender_combobox != NULL )
        {
            QList<QGroupBox*> box_list= sender_combobox->parentWidget()->findChildren<QGroupBox*>(QString(),Qt::FindDirectChildrenOnly);
            QList<QTabWidget*> tab_list= sender_combobox->parentWidget()->findChildren<QTabWidget*>(QString(),Qt::FindDirectChildrenOnly);
            foreach(QGroupBox * box, box_list)
                {
                    if (index==0)
                        {
                            box->setEnabled(false);
                        }
                    else
                        {
                            box->setEnabled(true);
                        }

                }
            foreach(QTabWidget * tab, tab_list)
                {
                    if (index==0)
                        {
                            tab->setEnabled(true);
                        }
                    else
                        {
                            tab->setEnabled(false);
                        }
                }
        }
}

QMap<QString, QString >* Signal_Conditioner::get_options()
{
    QMap<QString, QString > * map_options;
    map_options  = new QMap<QString, QString >;
    for (int i=0;i<list_map_implementation->count();i++)
        {
            int index= static_cast<int>(qFloor(i/list_subdirectory->count()));
            QString base_key;
            if (multiple_conditioners==true)
                {
                    base_key ="SignalConditioner"+QString::number(index)+".implementation";
                }
            else
                {
                    base_key ="SignalConditioner.implementation";
                }
            if (!map_pass_through_flag->value(index))
                {
                    if (!map_options->contains(base_key))
                        {
                            map_options->insert(base_key,"Signal_Conditioner");
                        }
                    foreach (QString key, list_map_implementation->at(i)->keys())
                        {
                            map_options->insert(key,list_map_implementation->at(i)->value(key)->text());
                        }
                }
            else
                {
                    if (!map_options->contains(base_key))
                        {
                            map_options->insert(base_key,"Pass_Through");
                        }
                }
        }
    for (int i=0;i<list_map_sub->count();i++)
        {
            if (!map_pass_through_flag->value(i))
                {
                    foreach (QString key, list_map_sub->at(i)->keys())
                        {
                            map_options->insert(key,list_map_sub->at(i)->value(key)->text());
                        }
                }
        }
    return map_options;
}
