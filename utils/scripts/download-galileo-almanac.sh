#!/bin/sh
# GNSS-SDR shell script that tries to download the latest Galileo Almanac file
# published by the European GNSS Service Centre.
#
# Usage: ./download-galileo-almanac.sh
#
# SPDX-FileCopyrightText: 2022 Carles Fernandez-Prades <cfernandez(at)cttc.es>
# SPDX-License-Identifier: GPL-3.0-or-later

if ! [ -x "$(command -v wget)" ]; then
    echo "Please install wget before using this script."
    exit 1
fi

help()
{
    echo "This script tries to download the most recent Galileo Almanac XML file"
    echo "published by the European GNSS Service Centre."
    echo "More info at https://www.gsc-europa.eu/gsc-products/almanac"
    echo "If today there is no published file, the script will look up to one week ago."
    echo ""
    echo "Usage:"
    echo "./download-galileo-almanac.sh [OPTION]"
    echo "  Options:"
    echo "  -h, --help        Prints this message"
    echo "  -r, --rename      Gets latest Galileo Almanac XML file and saves it as gal_almanac.xml"
    echo "  -d, --date [date] Retrieves file for a specific date, with format YYYY-MM-DD"
    echo "  -rd [date]        Retrieves file for a specific date, with format YYYY-MM-DD"
    echo "                    and saves it as gal_almanac.xml"
    echo ""
    echo "  Examples:"
    echo "  ./download-galileo-almanac.sh                # Gets latest Galileo Almanac XML file"
    echo "  ./download-galileo-almanac.sh -r             # Gets latest Galileo Almanac XML file, stores it as gal_almanac.xml"
    echo "  ./download-galileo-almanac.sh -d 2022-03-15  # Gets Galileo Almanac XML file for that day"
    echo "  ./download-galileo-almanac.sh -rd 2022-03-15 # Gets Galileo Almanac XML file for that day, stores it as gal_almanac.xml"
}

if [ "$1" = "-h" ] || [ "$1" = "--help" ] ; then
    help
    exit 0
fi

BASE_URL="https://www.gsc-europa.eu/sites/default/files/sites/all/files/"
YEAR=$(date '+%Y')
SPACING="-"
MONTH=$(date '+%m')
DAY=$(date '+%d')

TERMINATION1="_0.xml"
TERMINATION2=".xml"

COUNTER=1
MAX_COUNTER=7

if [ "$1" = "-d" ] || [ "$1" = "--date" ] ; then
    if wget "$BASE_URL$2$TERMINATION2" >/dev/null 2>&1 ; then
        echo "Downloaded latest Galileo almanac from $BASE_URL$2$TERMINATION2"
        exit 0
    else
        echo "Couldn't find an XML file for that date."
        exit 1
    fi
elif [ "$1" = "-rd" ] ; then
    if wget -O gal_almanac.xml "$BASE_URL$2$TERMINATION2" >/dev/null 2>&1 ; then
        echo "Downloaded latest Galileo almanac from $BASE_URL$2$TERMINATION2"
        exit 0
    else
        echo "Couldn't find an XML file for that date."
        rm gal_almanac.xml
        exit 1
    fi
else
    echo "According to system time, today is $(date '+%Y-%m-%d'). Searching for the latest Galileo almanac ..."
fi

if [ "$1" = "-r" ] || [ "$1" = "--rename" ]; then
    RENAME="yes"
fi

lowercase()
{
    echo "$1" | sed "y/ABCDEFGHIJKLMNOPQRSTUVWXYZ/abcdefghijklmnopqrstuvwxyz/"
}

OS=$(lowercase "$(uname)")

date_before()
{
    if [ "$OS" = "darwin" ]; then
        YEAR=$(date -v -"$COUNTER"d '+%Y')
        MONTH=$(date -v -"$COUNTER"d '+%m')
        DAY=$(date -v -"$COUNTER"d '+%d')
    else
        YEAR=$(date -d "$COUNTER day ago" '+%Y')
        MONTH=$(date -d "$COUNTER day ago" '+%m')
        DAY=$(date -d "$COUNTER day ago" '+%d')
    fi
    COUNTER=$((COUNTER+1))
}

download_rename_file()
{
    [ "$RENAME" = "yes" ] && set -- -O gal_almanac.xml "$@"
    wget "$@"
}

try_download()
{
    while [ $COUNTER -le $MAX_COUNTER ]
    do
        url="$BASE_URL$YEAR$SPACING$MONTH$SPACING$DAY$TERMINATION2"
        if download_rename_file "$url" >/dev/null 2>&1 ; then
        	echo "Downloaded latest Galileo almanac from $url"
            exit 0
        else
            date_before
            try_download
        fi
    done
}

url="$BASE_URL$YEAR$SPACING$MONTH$SPACING$DAY$TERMINATION1"
if download_rename_file "$url" >/dev/null 2>&1 ; then
	echo "Downloaded latest Galileo almanac from $url"
else
    try_download
    echo "Couldn't find a recent Galileo almanac."
    exit 1
fi
