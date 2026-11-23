/*!
 * \file antex_reader.cc
 * \brief Implementation of a class that reads antenna phase center offsets and
 * variations from a file in the Antenna Exchange Format (ANTEX)
 * \author Carles Fernandez-Prades, 2026. cfernandez(at)cttc.es
 *
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2026  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#include "antex_reader.h"
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <utility>

namespace
{
const std::size_t ANTEX_LABEL_COLUMN = 60;
const double ANTEX_MM_TO_M = 1e-3;
const double RECEIVER_ZENITH_STEP_DEG = 5.0;
const double SATELLITE_NADIR_STEP_DEG = 1.0;


std::string trim(const std::string& text)
{
    const std::size_t first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
        {
            return {};
        }
    const std::size_t last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}


std::string field(const std::string& line, std::size_t position, std::size_t length)
{
    return position < line.size() ? line.substr(position, length) : std::string();
}


std::vector<double> read_numbers(const std::string& text)
{
    std::vector<double> numbers;
    std::istringstream stream(text);
    double value = 0.0;
    while (stream >> value)
        {
            numbers.push_back(value);
        }
    return numbers;
}


// Days from 1970-01-01 of a date in the proleptic Gregorian calendar
int64_t days_from_civil(int64_t year, int64_t month, int64_t day)
{
    year -= month <= 2 ? 1 : 0;
    const int64_t era = (year >= 0 ? year : year - 399) / 400;
    const int64_t year_of_era = year - era * 400;
    const int64_t day_of_year = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    const int64_t day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    return era * 146097 + day_of_era - 719468;
}


// Date of a VALID FROM / VALID UNTIL record, in s since 1970. Zero if it cannot be read
int64_t read_date(const std::string& line)
{
    const std::vector<double> date = read_numbers(field(line, 0, 43));
    if (date.size() < 3)
        {
            return 0;
        }
    int64_t seconds = days_from_civil(static_cast<int64_t>(date[0]), static_cast<int64_t>(date[1]), static_cast<int64_t>(date[2])) * 86400;
    if (date.size() >= 6)
        {
            seconds += static_cast<int64_t>(date[3]) * 3600 + static_cast<int64_t>(date[4]) * 60 + static_cast<int64_t>(date[5]);
        }
    return seconds;
}


// "G01" for the records of a frequency ("G01", "G 1") or a satellite code. Empty if it is not one
std::string system_and_number(const std::string& text)
{
    const std::string code = trim(text);
    if (code.size() < 2 || code.size() > 3 || std::string("GRECJIS").find(code[0]) == std::string::npos)
        {
            return {};
        }
    const std::string number = trim(code.substr(1));
    if (number.empty() || number.size() > 2)
        {
            return {};
        }
    for (const char digit : number)
        {
            if (std::isdigit(static_cast<unsigned char>(digit)) == 0)
                {
                    return {};
                }
        }
    return std::string(1, code[0]) + (number.size() == 1 ? "0" : "") + number;
}


// Receiver antenna name and radome, with a single space in between
std::string receiver_key(const std::string& antenna_type, std::string* antenna = nullptr, std::string* radome = nullptr)
{
    std::istringstream stream(antenna_type);
    std::string name;
    std::string cover;
    stream >> name >> cover;
    if (cover.empty())
        {
            cover = "NONE";
        }
    if (antenna != nullptr)
        {
            *antenna = name;
        }
    if (radome != nullptr)
        {
            *radome = cover;
        }
    return name + ' ' + cover;
}


// Values given from zen1 in steps of dzen, at 19 angles from 0 in steps of step
std::array<double, 19> resample(const std::vector<double>& values, double zen1, double dzen, double step)
{
    std::array<double, 19> output = std::array<double, 19>();
    if (values.empty())
        {
            return output;
        }
    for (std::size_t i = 0; i < output.size(); i++)
        {
            const double position = dzen > 0.0 ? (static_cast<double>(i) * step - zen1) / dzen : 0.0;
            if (position <= 0.0)
                {
                    output[i] = values.front();
                }
            else if (position >= static_cast<double>(values.size() - 1))
                {
                    output[i] = values.back();
                }
            else
                {
                    const auto index = static_cast<std::size_t>(std::floor(position));
                    const double fraction = position - static_cast<double>(index);
                    output[i] = values[index] * (1.0 - fraction) + values[index + 1] * fraction;
                }
            output[i] *= ANTEX_MM_TO_M;
        }
    return output;
}

// Time of the satellite metadata files, YYYY:DDD:SSSSS, in s since 1970. Zero
// for 0000:000:00000 (not bounded) or if it cannot be read
int64_t read_sinex_time(const std::string& text)
{
    std::string numbers(text);
    for (char& character : numbers)
        {
            if (character == ':')
                {
                    character = ' ';
                }
        }
    const std::vector<double> time = read_numbers(numbers);
    if (time.size() < 3 || (time[0] == 0.0 && time[1] == 0.0 && time[2] == 0.0))
        {
            return 0;
        }
    auto year = static_cast<int64_t>(time[0]);
    if (year < 100)
        {
            year += year <= 50 ? 2000 : 1900;
        }
    return (days_from_civil(year, 1, 1) + static_cast<int64_t>(time[1]) - 1) * 86400 + static_cast<int64_t>(time[2]);
}


bool is_valid_at(const Antex_Antenna& antenna, int64_t time_s)
{
    return (antenna.valid_from == 0 || time_s >= antenna.valid_from) &&
           (antenna.valid_until == 0 || time_s <= antenna.valid_until);
}
}  // namespace


bool Antex_Reader::read(const std::string& filename)
{
    d_satellites.clear();
    d_satellites_by_svn.clear();
    d_receivers.clear();
    d_number_of_satellite_antennas = 0;

    std::ifstream file(filename.c_str());
    if (!file.is_open())
        {
            return false;
        }

    // The first record has the version of the format
    std::string line;
    if (!std::getline(file, line))
        {
            return false;
        }
    const std::vector<double> version = read_numbers(field(line, 0, 8));
    if (!version.empty() && version[0] >= 2.0)
        {
            if (trim(field(line, ANTEX_LABEL_COLUMN, 20)) != "ANTEX VERSION")
                {
                    return false;
                }
            read_version_2(file);
        }
    else
        {
            read_version_1(file);
        }
    return d_number_of_satellite_antennas > 0 || !d_receivers.empty();
}


void Antex_Reader::add_receiver(const Antex_Antenna& antenna)
{
    // calibration of the type (no serial number) before the ones of individual antennas
    const std::string key = receiver_key(antenna.type);
    const auto stored = d_receivers.find(key);
    if (stored == d_receivers.cend() || (!stored->second.serial.empty() && antenna.serial.empty()))
        {
            d_receivers[key] = antenna;
        }
}


void Antex_Reader::read_version_1(std::istream& file)
{
    Antex_Antenna antenna = Antex_Antenna();
    antenna.valid_from = 0;
    antenna.valid_until = 0;
    Antex_Frequency frequency = Antex_Frequency();
    std::string frequency_code;
    std::string satellite;
    std::vector<double> variation;
    double zen1 = 0.0;
    double dzen = 0.0;
    bool in_antenna = false;
    bool is_satellite = false;

    std::string line;
    while (std::getline(file, line))
        {
            const std::string label = trim(field(line, ANTEX_LABEL_COLUMN, 20));
            if (label == "START OF ANTENNA")
                {
                    antenna = Antex_Antenna();
                    antenna.valid_from = 0;
                    antenna.valid_until = 0;
                    frequency_code.clear();
                    satellite.clear();
                    zen1 = 0.0;
                    dzen = 0.0;
                    in_antenna = true;
                    is_satellite = false;
                    continue;
                }
            if (!in_antenna)
                {
                    continue;
                }
            if (label == "TYPE / SERIAL NO")
                {
                    antenna.type = trim(field(line, 0, 20));
                    antenna.serial = trim(field(line, 20, 20));
                    // a satellite antenna has the satellite code in place of the serial number
                    satellite = system_and_number(field(line, 20, 20));
                    is_satellite = !satellite.empty();
                }
            else if (label == "ZEN1 / ZEN2 / DZEN")
                {
                    const std::vector<double> grid = read_numbers(field(line, 0, 20));
                    if (grid.size() >= 3)
                        {
                            zen1 = grid[0];
                            dzen = grid[2];
                        }
                }
            else if (label == "VALID FROM")
                {
                    antenna.valid_from = read_date(line);
                }
            else if (label == "VALID UNTIL")
                {
                    antenna.valid_until = read_date(line);
                }
            else if (label == "START OF FREQUENCY")
                {
                    frequency_code = system_and_number(field(line, 3, 3));
                    frequency = Antex_Frequency();
                    variation.clear();
                }
            else if (label == "NORTH / EAST / UP")
                {
                    const std::vector<double> offset = read_numbers(field(line, 0, 30));
                    if (offset.size() >= 3)
                        {
                            for (std::size_t i = 0; i < 3; i++)
                                {
                                    frequency.offset[i] = offset[i] * ANTEX_MM_TO_M;
                                }
                        }
                }
            else if (label == "END OF FREQUENCY")
                {
                    if (!frequency_code.empty())
                        {
                            frequency.variation = resample(variation, zen1, dzen, is_satellite ? SATELLITE_NADIR_STEP_DEG : RECEIVER_ZENITH_STEP_DEG);
                            antenna.frequencies[frequency_code] = frequency;
                        }
                    frequency_code.clear();
                }
            else if (label == "END OF ANTENNA")
                {
                    in_antenna = false;
                    if (antenna.frequencies.empty())
                        {
                            continue;
                        }
                    if (is_satellite)
                        {
                            d_satellites[satellite].push_back(antenna);
                            d_number_of_satellite_antennas++;
                        }
                    else
                        {
                            add_receiver(antenna);
                        }
                }
            else if (!frequency_code.empty() && trim(field(line, 0, 8)) == "NOAZI")
                {
                    variation = read_numbers(field(line, 8, std::string::npos));
                }
        }
}


void Antex_Reader::read_version_2(std::istream& file)
{
    // An antenna record has calibration records, each with its own validity
    // and grid. The ones for the carrier phase are kept, grouped by validity
    std::vector<Antex_Antenna> calibrations;
    Antex_Antenna calibration = Antex_Antenna();
    Antex_Frequency frequency = Antex_Frequency();
    std::vector<std::string> frequency_codes;
    std::vector<double> sums;
    std::vector<int> counts;
    std::string type;
    std::string serial;
    std::string svn;
    double zen1 = 0.0;
    double dzen = 0.0;
    int rows = 0;
    bool in_antenna = false;
    bool is_satellite = false;
    bool phase_calibration = false;
    bool in_phase_pattern = false;
    bool in_table = false;

    std::string line;
    while (std::getline(file, line))
        {
            const std::string label = trim(field(line, ANTEX_LABEL_COLUMN, 20));
            if (in_table && label != "END OF PHASE")
                {
                    // Row of the pattern: azimuth, and a value per off-boresight
                    // angle in fields of eight characters, blank if not known.
                    // The row at 360 degrees repeats the one at 0 degrees
                    const std::vector<double> azimuth = read_numbers(field(line, 0, 8));
                    if (azimuth.empty() || (azimuth[0] > 359.999 && rows > 0))
                        {
                            continue;
                        }
                    rows++;
                    for (std::size_t column = 0; 8 * (column + 1) < line.size(); column++)
                        {
                            const std::vector<double> value = read_numbers(field(line, 8 * (column + 1), 8));
                            if (value.empty())
                                {
                                    continue;
                                }
                            if (column >= sums.size())
                                {
                                    sums.resize(column + 1, 0.0);
                                    counts.resize(column + 1, 0);
                                }
                            sums[column] += value[0];
                            counts[column]++;
                        }
                    continue;
                }
            if (label == "START OF ANTENNA")
                {
                    calibrations.clear();
                    type.clear();
                    serial.clear();
                    svn.clear();
                    in_antenna = true;
                    is_satellite = false;
                    phase_calibration = false;
                    in_phase_pattern = false;
                    continue;
                }
            if (!in_antenna)
                {
                    continue;
                }
            if (label == "TYPE / SVN")
                {
                    type = trim(field(line, 0, 20));
                    svn = trim(field(line, 40, 4));
                    is_satellite = true;
                }
            else if (label == "TYPE / SN")
                {
                    type = trim(field(line, 0, 20));
                    serial = trim(field(line, 20, 20));
                    is_satellite = false;
                }
            else if (label == "START OF CALIB")
                {
                    calibration = Antex_Antenna();
                    calibration.valid_from = 0;
                    calibration.valid_until = 0;
                    zen1 = 0.0;
                    dzen = 0.0;
                    phase_calibration = false;
                }
            else if (label == "TYPE / # OF FREQS")
                {
                    phase_calibration = trim(field(line, 0, 10)) == "PHASE";
                }
            else if (label == "VALID FROM")
                {
                    calibration.valid_from = read_date(line);
                }
            else if (label == "VALID UNTIL")
                {
                    calibration.valid_until = read_date(line);
                }
            else if (label == "ZEN1 / ZEN2 / DZEN")
                {
                    const std::vector<double> grid = read_numbers(field(line, 0, 20));
                    if (grid.size() >= 3)
                        {
                            zen1 = grid[0];
                            dzen = grid[2];
                        }
                }
            else if (label == "START OF PHASE")
                {
                    // the bands that share the offset and the pattern
                    frequency_codes.clear();
                    std::istringstream bands(field(line, 0, ANTEX_LABEL_COLUMN));
                    std::string band;
                    while (bands >> band)
                        {
                            const std::string code = system_and_number(band);
                            if (!code.empty())
                                {
                                    frequency_codes.push_back(code);
                                }
                        }
                    frequency = Antex_Frequency();
                    sums.clear();
                    counts.clear();
                    rows = 0;
                    in_phase_pattern = phase_calibration;
                }
            else if (label == "X / Y / Z" && in_phase_pattern)
                {
                    // Receiver antennas: x is east and y is north
                    const std::vector<double> offset = read_numbers(field(line, 0, 30));
                    if (offset.size() >= 3)
                        {
                            frequency.offset[0] = (is_satellite ? offset[0] : offset[1]) * ANTEX_MM_TO_M;
                            frequency.offset[1] = (is_satellite ? offset[1] : offset[0]) * ANTEX_MM_TO_M;
                            frequency.offset[2] = offset[2] * ANTEX_MM_TO_M;
                        }
                    in_table = true;
                }
            else if (label == "END OF PHASE")
                {
                    if (in_phase_pattern)
                        {
                            // mean over the azimuth: the pattern that only depends on the off-boresight angle
                            std::vector<double> variation(sums.size(), 0.0);
                            for (std::size_t i = 0; i < sums.size(); i++)
                                {
                                    variation[i] = counts[i] > 0 ? sums[i] / static_cast<double>(counts[i]) : 0.0;
                                }
                            frequency.variation = resample(variation, zen1, dzen, is_satellite ? SATELLITE_NADIR_STEP_DEG : RECEIVER_ZENITH_STEP_DEG);
                            for (const std::string& code : frequency_codes)
                                {
                                    calibration.frequencies.insert(std::make_pair(code, frequency));
                                }
                        }
                    in_phase_pattern = false;
                    in_table = false;
                }
            else if (label == "END OF CALIB")
                {
                    if (phase_calibration && !calibration.frequencies.empty())
                        {
                            // calibration records of the same validity make one set. The
                            // ones of a receiver antenna are not told apart by validity
                            bool merged = false;
                            for (Antex_Antenna& stored : calibrations)
                                {
                                    if (!is_satellite || (stored.valid_from == calibration.valid_from && stored.valid_until == calibration.valid_until))
                                        {
                                            stored.frequencies.insert(calibration.frequencies.cbegin(), calibration.frequencies.cend());
                                            merged = true;
                                            break;
                                        }
                                }
                            if (!merged)
                                {
                                    calibrations.push_back(calibration);
                                }
                        }
                    phase_calibration = false;
                }
            else if (label == "END OF ANTENNA")
                {
                    in_antenna = false;
                    for (Antex_Antenna& antenna : calibrations)
                        {
                            antenna.type = type;
                            antenna.serial = is_satellite ? svn : serial;
                            if (!is_satellite)
                                {
                                    add_receiver(antenna);
                                }
                            else if (!svn.empty())
                                {
                                    d_satellites_by_svn[svn].push_back(antenna);
                                    d_number_of_satellite_antennas++;
                                }
                            // A satellite antenna with no SVN is the one of a type of
                            // satellite. The satellites that carry it are not known here
                        }
                }
        }
}


bool Antex_Reader::read_satellite_metadata(const std::string& filename)
{
    d_prn_assignments.clear();
    std::ifstream file(filename.c_str());
    if (!file.is_open())
        {
            return false;
        }
    bool in_block = false;
    std::string line;
    while (std::getline(file, line))
        {
            const std::string text = trim(line);
            if (text == "+SATELLITE/PRN")
                {
                    in_block = true;
                }
            else if (text == "-SATELLITE/PRN")
                {
                    in_block = false;
                }
            else if (in_block && !text.empty() && text[0] != '*')
                {
                    // SVN, start and end of the assignment, PRN
                    std::istringstream stream(text);
                    Prn_Assignment assignment;
                    std::string valid_from;
                    std::string valid_until;
                    std::string prn;
                    if (stream >> assignment.svn >> valid_from >> valid_until >> prn)
                        {
                            const std::string satellite = system_and_number(prn);
                            if (!satellite.empty())
                                {
                                    assignment.valid_from = read_sinex_time(valid_from);
                                    assignment.valid_until = read_sinex_time(valid_until);
                                    d_prn_assignments[satellite].push_back(assignment);
                                }
                        }
                }
        }
    return !d_prn_assignments.empty();
}


std::string Antex_Reader::find_svn(const std::string& satellite, int64_t time_s) const
{
    const auto assignments = d_prn_assignments.find(satellite);
    if (assignments != d_prn_assignments.cend())
        {
            for (const Prn_Assignment& assignment : assignments->second)
                {
                    if (time_s >= assignment.valid_from && (assignment.valid_until == 0 || time_s <= assignment.valid_until))
                        {
                            return assignment.svn;
                        }
                }
        }
    return {};
}


const Antex_Antenna* Antex_Reader::find_satellite(const std::string& satellite, int64_t time_s) const
{
    const auto antennas = d_satellites.find(satellite);
    if (antennas != d_satellites.cend())
        {
            for (const Antex_Antenna& antenna : antennas->second)
                {
                    if (is_valid_at(antenna, time_s))
                        {
                            return &antenna;
                        }
                }
        }
    // Antennas identified by the SVN of the satellite
    const auto svn_antennas = d_satellites_by_svn.find(find_svn(satellite, time_s));
    if (svn_antennas != d_satellites_by_svn.cend())
        {
            for (const Antex_Antenna& antenna : svn_antennas->second)
                {
                    if (is_valid_at(antenna, time_s))
                        {
                            return &antenna;
                        }
                }
        }
    return nullptr;
}


const Antex_Antenna* Antex_Reader::find_receiver(const std::string& antenna_type) const
{
    std::string antenna;
    std::string radome;
    const std::string key = receiver_key(antenna_type, &antenna, &radome);
    if (antenna.empty())
        {
            return nullptr;
        }
    auto found = d_receivers.find(key);
    if (found == d_receivers.cend() && radome != "NONE")
        {
            found = d_receivers.find(antenna + " NONE");
        }
    return found == d_receivers.cend() ? nullptr : &found->second;
}
