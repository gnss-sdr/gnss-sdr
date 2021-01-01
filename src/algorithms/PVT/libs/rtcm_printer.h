/*!
 * \file rtcm_printer.h
 * \brief Interface of a RTCM 3.2 printer for GNSS-SDR
 * This class provides a implementation of a subset of the RTCM Standard 10403.2
 * for Differential GNSS Services
 *
 * \author Carles Fernandez-Prades, 2014. cfernandez(at)cttc.es
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * Copyright (C) 2010-2020  (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -----------------------------------------------------------------------------
 */

#ifndef GNSS_SDR_RTCM_PRINTER_H
#define GNSS_SDR_RTCM_PRINTER_H

#include <cstdint>  // for int32_t
#include <fstream>  // for std::ofstream
#include <map>      // for std::map
#include <memory>   // std::shared_ptr
#include <string>

/** \addtogroup PVT
 * \{ */
/** \addtogroup PVT_libs
 * \{ */


class Galileo_Ephemeris;
class Glonass_Gnav_Ephemeris;
class Glonass_Gnav_Utc_Model;
class Gnss_Synchro;
class Gps_CNAV_Ephemeris;
class Gps_Ephemeris;
class Rtcm;
class Rtklib_Solver;

/*!
 * \brief This class provides a implementation of a subset of the RTCM Standard 10403.2 messages
 */
class Rtcm_Printer
{
public:
    /*!
     * \brief Default constructor.
     */
    Rtcm_Printer(const std::string& filename,
        bool flag_rtcm_file_dump,
        bool flag_rtcm_server,
        bool flag_rtcm_tty_port,
        uint16_t rtcm_tcp_port,
        uint16_t rtcm_station_id,
        const std::string& rtcm_dump_devname,
        bool time_tag_name = true,
        const std::string& base_path = ".");

    /*!
     * \brief Default destructor.
     */
    ~Rtcm_Printer();

    /*!
     * \brief Print RTCM messages.
     */
    void Print_Rtcm_Messages(const Rtklib_Solver* pvt_solver,
        const std::map<int, Gnss_Synchro>& gnss_observables_map,
        double rx_time,
        int32_t type_of_rx,
        int32_t rtcm_MSM_rate_ms,
        int32_t rtcm_MT1019_rate_ms,
        int32_t rtcm_MT1020_rate_ms,
        int32_t rtcm_MT1045_rate_ms,
        int32_t rtcm_MT1077_rate_ms,
        int32_t rtcm_MT1097_rate_ms,
        bool flag_write_RTCM_MSM_output,
        bool flag_write_RTCM_1019_output,
        bool flag_write_RTCM_1020_output,
        bool flag_write_RTCM_1045_output,
        bool enable_rx_clock_correction);

    uint32_t lock_time(const Gps_Ephemeris& eph, double obs_time, const Gnss_Synchro& gnss_synchro);
    uint32_t lock_time(const Gps_CNAV_Ephemeris& eph, double obs_time, const Gnss_Synchro& gnss_synchro);
    uint32_t lock_time(const Galileo_Ephemeris& eph, double obs_time, const Gnss_Synchro& gnss_synchro);

    /*!
     * \brief Locks time for logging given GLONASS GNAV Broadcast Ephemeris
     * \note Code added as part of GSoC 2017 program
     * \params glonass_gnav_eph GLONASS GNAV Broadcast Ephemeris
     * \params obs_time Time of observation at the moment of printing
     * \params observables Set of observables as defined by the platform
     * \return locked time during logging process
     */
    uint32_t lock_time(const Glonass_Gnav_Ephemeris& eph, double obs_time, const Gnss_Synchro& gnss_synchro);

    std::string print_MT1005_test();  //!<  For testing purposes

private:
    bool Print_Rtcm_MT1001(const Gps_Ephemeris& gps_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables);
    bool Print_Rtcm_MT1002(const Gps_Ephemeris& gps_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables);
    bool Print_Rtcm_MT1003(const Gps_Ephemeris& gps_eph, const Gps_CNAV_Ephemeris& cnav_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables);
    bool Print_Rtcm_MT1004(const Gps_Ephemeris& gps_eph, const Gps_CNAV_Ephemeris& cnav_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables);

    /*
     * \brief Prints L1-Only GLONASS RTK Observables
     * \details This GLONASS message type is not generally used or supported; type 1012 is to be preferred.
     * \note Code added as part of GSoC 2017 program
     * \param glonass_gnav_eph GLONASS GNAV Broadcast Ephemeris
     * \param obs_time Time of observation at the moment of printing
     * \param observables Set of observables as defined by the platform
     * \return true or false upon operation success
     */
    bool Print_Rtcm_MT1009(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables);

    /*
     * \brief Prints Extended L1-Only GLONASS RTK Observables
     * \details This GLONASS message type is used when only L1 data is present and bandwidth is very tight, often 1012 is used in such cases.
     * \note Code added as part of GSoC 2017 program
     * \param glonass_gnav_eph GLONASS GNAV Broadcast Ephemeris
     * \param obs_time Time of observation at the moment of printing
     * \param observables Set of observables as defined by the platform
     * \return true or false upon operation success
     */
    bool Print_Rtcm_MT1010(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables);

    /*
     * \brief Prints L1&L2 GLONASS RTK Observables
     * \details This GLONASS message type is not generally used or supported; type 1012 is to be preferred
     * \note Code added as part of GSoC 2017 program
     * \param glonass_gnav_ephL1 GLONASS L1 GNAV Broadcast Ephemeris for satellite
     * \param glonass_gnav_ephL2 GLONASS L2 GNAV Broadcast Ephemeris for satellite
     * \param obs_time Time of observation at the moment of printing
     * \param observables Set of observables as defined by the platform
     * \return true or false upon operation success
     */
    bool Print_Rtcm_MT1011(const Glonass_Gnav_Ephemeris& glonass_gnav_ephL1, const Glonass_Gnav_Ephemeris& glonass_gnav_ephL2, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables);

    /*
     * \brief Prints Extended L1&L2 GLONASS RTK Observables
     * \details This GLONASS message type is the most common observational message type, with L1/L2/SNR content.  This is one of the most common messages found.
     * \note Code added as part of GSoC 2017 program
     * \param glonass_gnav_ephL1 GLONASS L1 GNAV Broadcast Ephemeris for satellite
     * \param glonass_gnav_ephL2 GLONASS L2 GNAV Broadcast Ephemeris for satellite
     * \param obs_time Time of observation at the moment of printing
     * \param observables Set of observables as defined by the platform
     * \return true or false upon operation success
     */
    bool Print_Rtcm_MT1012(const Glonass_Gnav_Ephemeris& glonass_gnav_ephL1, const Glonass_Gnav_Ephemeris& glonass_gnav_ephL2, double obs_time, const std::map<int32_t, Gnss_Synchro>& observables);

    bool Print_Rtcm_MT1019(const Gps_Ephemeris& gps_eph);      // GPS Ephemeris, should be broadcast in the event that the IODC does not match the IODE, and every 2 minutes.
    bool Print_Rtcm_MT1045(const Galileo_Ephemeris& gal_eph);  // Galileo Ephemeris, should be broadcast every 2 minutes

    /*
     * \brief Prints GLONASS GNAV Ephemeris
     * \details This GLONASS message should be broadcast every 2 minutes
     * \note Code added as part of GSoC 2017 program
     * \param glonass_gnav_eph GLONASS GNAV Broadcast Ephemeris
     * \param utc_model GLONASS GNAV Clock Information broadcast in string 5
     * \return true or false upon operation success
     */
    bool Print_Rtcm_MT1020(const Glonass_Gnav_Ephemeris& glonass_gnav_eph, const Glonass_Gnav_Utc_Model& utc_model);

    bool Print_Rtcm_MSM(uint32_t msm_number,
        const Gps_Ephemeris& gps_eph,
        const Gps_CNAV_Ephemeris& gps_cnav_eph,
        const Galileo_Ephemeris& gal_eph,
        const Glonass_Gnav_Ephemeris& glo_gnav_eph,
        double obs_time,
        const std::map<int32_t, Gnss_Synchro>& observables,
        uint32_t clock_steering_indicator,
        uint32_t external_clock_indicator,
        int32_t smooth_int,
        bool divergence_free,
        bool more_messages);

    int32_t init_serial(const std::string& serial_device);  // serial port control
    void close_serial() const;
    bool Print_Message(const std::string& message);

    std::unique_ptr<Rtcm> rtcm;
    std::ofstream rtcm_file_descriptor;  // Output file stream for RTCM log file
    std::string rtcm_filename;           // String with the RTCM log filename
    std::string rtcm_base_path;
    std::string rtcm_devname;
    int32_t rtcm_dev_descriptor;  // RTCM serial device descriptor (i.e. COM port)
    uint16_t port;
    uint16_t station_id;
    bool d_rtcm_writing_started;
    bool d_rtcm_file_dump;
};


/** \} */
/** \} */
#endif  // GNSS_SDR_RTCM_PRINTER_H
