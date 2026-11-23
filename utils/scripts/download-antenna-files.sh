#!/bin/sh
# GNSS-SDR shell script that downloads the latest antenna calibration (ANTEX)
# and satellite metadata (SINEX) files used by the PPP positioning modes,
# published by the International GNSS Service (IGS) and by the European GNSS
# Service Centre (GSC).
#
# Usage: ./download-antenna-files.sh [-o directory]
#
# SPDX-FileCopyrightText: 2026 Carles Fernandez-Prades <cfernandez(at)cttc.es>
# SPDX-License-Identifier: GPL-3.0-or-later

IGS_URL="https://files.igs.org/pub/station/general"
GSC_URL="https://www.gsc-europa.eu"
GSC_PAGE="$GSC_URL/support-to-developers/galileo-satellite-metadata"
# Galileo satellite antennas, if the GSC web page does not tell a later file
GSC_ANTEX="/sites/default/files/sites/all/files/GSAT_2409.atx"

OUTPUT_DIR="."
FAILED=""

help()
{
    echo "This script downloads the latest antenna calibration (ANTEX) and satellite"
    echo "metadata (SINEX) files for the PPP positioning modes of GNSS-SDR:"
    echo ""
    echo "  From the International GNSS Service, $IGS_URL"
    echo "    igs20.atx                   Satellite and receiver antennas, ANTEX 1.4"
    echo "    igs20_sat.atx2              Satellite antennas, ANTEX 2.0"
    echo "    igs20_rcv.atx2              Receiver antennas, ANTEX 2.0"
    echo "    igs_satellite_metadata.snx  Satellite metadata, needed by igs20_sat.atx2"
    echo ""
    echo "  From the European GNSS Service Centre, $GSC_PAGE"
    echo "    GSAT_[number].atx           Galileo satellite antennas, ANTEX 1.4"
    echo "    igs20_[week].atx            IGS file in use by the Galileo High Accuracy"
    echo "                                Service, if its calibrations are not those"
    echo "                                of the latest igs20.atx"
    echo ""
    echo "Files are stored uncompressed, replacing those of a previous download."
    echo ""
    echo "Usage:"
    echo "./download-antenna-files.sh [OPTION]"
    echo "  Options:"
    echo "  -h, --help          Prints this message"
    echo "  -o, --output [dir]  Stores the files in that directory, which is created"
    echo "                      if it does not exist (default: current directory)"
    echo ""
    echo "  Examples:"
    echo "  ./download-antenna-files.sh             # Gets the files in the current directory"
    echo "  ./download-antenna-files.sh -o antenna  # Gets the files in ./antenna"
    echo ""
    echo "Then, in the configuration file of a PPP_Static or PPP_Kinematic receiver:"
    echo "  PVT.antex_file=/path/to/igs20.atx"
    echo "or, with the ANTEX 2.0 files:"
    echo "  PVT.antex_file=/path/to/igs20_sat.atx2"
    echo "  PVT.satellite_metadata_file=/path/to/igs_satellite_metadata.snx"
    echo "  PVT.receiver_antex_file=/path/to/igs20_rcv.atx2"
}

while [ $# -gt 0 ]
do
    case "$1" in
        -h|--help)
            help
            exit 0
            ;;
        -o|--output)
            if [ -z "$2" ]; then
                echo "Option $1 requires a directory."
                exit 1
            fi
            OUTPUT_DIR="$2"
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Try ./download-antenna-files.sh --help"
            exit 1
            ;;
    esac
    shift
done

if [ -x "$(command -v curl)" ]; then
    DOWNLOADER="curl"
elif [ -x "$(command -v wget)" ]; then
    DOWNLOADER="wget"
else
    echo "Please install curl or wget before using this script."
    exit 1
fi

if ! [ -x "$(command -v gzip)" ]; then
    echo "Please install gzip before using this script."
    exit 1
fi

if ! mkdir -p "$OUTPUT_DIR" || ! [ -w "$OUTPUT_DIR" ]; then
    echo "Cannot write in directory $OUTPUT_DIR"
    exit 1
fi

# Downloads are written to a temporary name, so a failed one leaves no file
PARTIAL="$OUTPUT_DIR/.download-antenna-files.$$.part"
trap 'rm -f "$PARTIAL" "$PARTIAL.gz"' EXIT
trap 'exit 1' HUP INT TERM

download()
{
    # $1: address, $2: file
    if [ "$DOWNLOADER" = "curl" ]; then
        curl --fail --silent --show-error --location --connect-timeout 30 \
            --output "$2" "$1"
    else
        wget --quiet --timeout=30 --tries=1 --output-document="$2" "$1"
    fi
}

get_file()
{
    # $1: address, $2: text in the first line of a file of its format
    # Stores the file in OUTPUT_DIR, uncompressed, and sets FILE to its name
    FILE=$(basename "$1")
    if [ "${FILE%.gz}" != "$FILE" ]; then
        FILE="${FILE%.gz}"
        download "$1" "$PARTIAL.gz" && gzip -d -c "$PARTIAL.gz" > "$PARTIAL"
        STATUS=$?
    else
        download "$1" "$PARTIAL"
        STATUS=$?
    fi
    if [ "$STATUS" -ne 0 ]; then
        echo "Could not download $1"
    elif ! head -n 1 "$PARTIAL" | grep "$2" >/dev/null 2>&1; then
        echo "The file at $1 does not have the expected format, so it is not stored."
    elif mv -f "$PARTIAL" "$OUTPUT_DIR/$FILE"; then
        echo "Downloaded $FILE from $1"
        rm -f "$PARTIAL.gz"
        return 0
    fi
    rm -f "$PARTIAL" "$PARTIAL.gz"
    FAILED="$FAILED $FILE"
    return 1
}

latest_link()
{
    # $1: file with a web page, $2: basic regular expression of a file name
    # Prints the link in that page to the file of that name with the highest
    # number (and revision, as in igs20_2415_2.atx.gz) after its first underscore
    tr '"' '\n' < "$1" | grep "/$2\$" |
        awk -F/ '{ n = $NF; sub(/\..*$/, "", n); split(n, a, "_"); print a[2] + 0, a[3] + 0, $0 }' |
        sort -k 1,1n -k 2,2n | tail -n 1 | cut -d ' ' -f 3
}

gsc_address()
{
    # $1: link found in the GSC web page. Prints its address if it is in that site
    case "$1" in
        "$GSC_URL"/*) echo "$1" ;;
        /*) echo "$GSC_URL$1" ;;
    esac
}

echo "Downloading antenna and satellite metadata files into $OUTPUT_DIR ..."

# International GNSS Service: these names always hold the latest release
get_file "$IGS_URL/igs20.atx.gz" "ANTEX VERSION"
get_file "$IGS_URL/igs20_sat.atx2.gz" "ANTEX VERSION"
get_file "$IGS_URL/igs20_rcv.atx2.gz" "ANTEX VERSION"
get_file "$IGS_URL/igs_satellite_metadata.snx" "%=SNX"

# European GNSS Service Centre: the file names carry a number, and the web page
# of the Galileo satellite metadata links the ones in force
GSC_HAS_ANTEX=""
if download "$GSC_PAGE" "$PARTIAL" 2>/dev/null; then
    LINK=$(gsc_address "$(latest_link "$PARTIAL" 'GSAT_[0-9][0-9]*\.atx')")
    if [ -n "$LINK" ]; then
        GSC_ANTEX="$LINK"
    fi
    GSC_HAS_ANTEX=$(gsc_address "$(latest_link "$PARTIAL" 'igs[0-9][0-9]_[0-9][0-9_]*\.atx\.gz')")
else
    echo "Could not read $GSC_PAGE"
fi
rm -f "$PARTIAL"

get_file "$(gsc_address "$GSC_ANTEX")" "ANTEX VERSION"

if [ -n "$GSC_HAS_ANTEX" ]; then
    if get_file "$GSC_HAS_ANTEX" "ANTEX VERSION"; then
        echo "$FILE is the IGS file in use by the Galileo High Accuracy Service."
        # The GSC copy differs from the IGS one in the label of its SINEX CODE records
        if [ -f "$OUTPUT_DIR/igs20.atx" ] &&
            grep -v "SINEX CODE" "$OUTPUT_DIR/igs20.atx" > "$PARTIAL" &&
            grep -v "SINEX CODE" "$OUTPUT_DIR/$FILE" | cmp -s - "$PARTIAL"; then
            rm -f "$OUTPUT_DIR/$FILE"
            echo "It has the calibrations of the latest igs20.atx, so it is not kept."
        fi
        rm -f "$PARTIAL"
    fi
else
    echo "Could not find the IGS file in use by the Galileo High Accuracy Service."
fi

if [ -n "$FAILED" ]; then
    echo "Files not retrieved:$FAILED"
    exit 1
fi
echo "Done. See ./download-antenna-files.sh --help for their use in the receiver configuration."
