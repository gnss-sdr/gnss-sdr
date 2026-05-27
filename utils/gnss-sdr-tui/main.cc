/*!
 * \file main.cc
 * \brief Entry point for the GNSS-SDR Terminal User Interface.
 * \author Generated for GNSS-SDR contributors
 *
 * -----------------------------------------------------------------------------
 *
 * GNSS-SDR is a Global Navigation Satellite System software-defined receiver.
 * This file is part of GNSS-SDR.
 *
 * SPDX-FileCopyrightText: 2025 (see AUTHORS file for a list of contributors)
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * -----------------------------------------------------------------------------
 */

#include "data_store.h"
#include "udp_listener.h"
#include "screens/event_log.h"
#include "screens/help_screen.h"
#include "screens/pvt_panel.h"
#include "screens/satellite_detail.h"
#include "screens/satellite_table.h"
#include "screens/signal_overview.h"
#include <ftxui/component/component.hpp>
#include <ftxui/component/screen_interactive.hpp>
#include <ftxui/dom/elements.hpp>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

std::atomic<bool> g_running{true};

void handle_signal(int)
{
    g_running = false;
}

int main(int argc, char* argv[])
{
    std::string address = "127.0.0.1";
    unsigned short gnss_port = 1234;
    unsigned short pvt_port = 1234;

    for (int i = 1; i < argc; ++i)
        {
            const std::string arg(argv[i]);
            if (arg == "--gnss-port" && i + 1 < argc)
                {
                    gnss_port = static_cast<unsigned short>(std::stoi(argv[++i]));
                }
            else if (arg == "--pvt-port" && i + 1 < argc)
                {
                    pvt_port = static_cast<unsigned short>(std::stoi(argv[++i]));
                }
            else if (arg == "--address" && i + 1 < argc)
                {
                    address = argv[++i];
                }
            else if (arg == "--help")
                {
                    std::cout << "Usage: gnss_sdr_tui [options]\n\n"
                              << "  --gnss-port PORT    UDP port for GnssSynchro (default: 1234)\n"
                              << "  --pvt-port PORT     UDP port for MonitorPvt (default: 1234)\n"
                              << "  --address ADDR      UDP address (default: 127.0.0.1)\n"
                              << "  --help              Show this help\n";
                    return 0;
                }
        }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    DataStore data_store;
    UdpListener listener(data_store, address, gnss_port, pvt_port);
    listener.start();

    auto screen = ftxui::ScreenInteractive::Fullscreen();

    std::atomic<bool> show_help{false};
    std::atomic<bool> show_detail{false};
    std::atomic<int> selected_row{1};
    std::atomic<int> detail_channel{0};
    std::atomic<int> refresh_ms{200};

    auto renderer = ftxui::Renderer([&] {
        if (show_help)
            {
                return render_help_screen();
            }

        if (show_detail)
            {
                return render_satellite_detail(data_store, detail_channel.load());
            }

        auto overview = render_signal_overview(data_store);
        auto table = render_satellite_table(data_store, selected_row.load());
        auto pvt = render_pvt_panel(data_store);
        auto log = render_event_log(data_store);

        auto right = ftxui::vbox({
            table | ftxui::flex,
            ftxui::separator(),
            pvt,
        });

        auto top = ftxui::hbox({
            overview,
            ftxui::separator(),
            right | ftxui::flex,
        }) | ftxui::flex;

        std::ostringstream status;
        status << " Sats: " << data_store.satellite_count()
               << "  |  Refresh: " << refresh_ms.load() << "ms";
        auto pvt_opt = data_store.get_pvt();
        if (pvt_opt)
            {
                status << "  |  HDOP: " << std::to_string(pvt_opt->hdop).substr(0, 4)
                       << "  |  " << pvt_opt->utc_time.substr(0, 19);
            }
        status << "  |  \xE2\x86\x91\xE2\x86\x93:select  Enter:detail  h:help  q:quit  +/-:speed";
        auto status_bar = ftxui::text(status.str()) | ftxui::dim | ftxui::center;

        ftxui::Elements main_el;
        main_el.push_back(top | ftxui::flex);
        main_el.push_back(ftxui::separator());
        main_el.push_back(status_bar);
        return ftxui::vbox(std::move(main_el));
    });

    auto app = renderer | ftxui::CatchEvent([&](const ftxui::Event& event) {
        if (event == ftxui::Event::Character('q'))
            {
                screen.Exit();
                return true;
            }
        if (event == ftxui::Event::Character('h'))
            {
                show_help = !show_help;
                return true;
            }
        if (event == ftxui::Event::Character('+') || event == ftxui::Event::Character('='))
            {
                refresh_ms = std::min(1000, refresh_ms.load() + 50);
                return true;
            }
        if (event == ftxui::Event::Character('-') || event == ftxui::Event::Character('_'))
            {
                refresh_ms = std::max(50, refresh_ms.load() - 50);
                return true;
            }
        if (event == ftxui::Event::Character('e') || event == ftxui::Event::Character('\r') || event == ftxui::Event::Character('\n'))
            {
                if (show_detail)
                    {
                        show_detail = false;
                    }
                else
                    {
                        const auto sorted = data_store.get_sorted_satellites();
                        const int idx = selected_row.load() - 1;
                        if (idx >= 0 && idx < static_cast<int>(sorted.size()))
                            {
                                detail_channel = sorted[idx].first;
                                show_detail = true;
                            }
                    }
                return true;
            }
        if (event == ftxui::Event::Escape)
            {
                if (show_detail || show_help)
                    {
                        show_detail = false;
                        show_help = false;
                        return true;
                    }
                return false;
            }
        if (event == ftxui::Event::ArrowUp || event == ftxui::Event::Character('k'))
            {
                if (!show_detail && !show_help)
                    {
                        const auto n = static_cast<int>(data_store.satellite_count());
                        selected_row = std::max(1, selected_row.load() - 1);
                        if (selected_row > n) selected_row = n;
                    }
                return true;
            }
        if (event == ftxui::Event::ArrowDown || event == ftxui::Event::Character('j'))
            {
                if (!show_detail && !show_help)
                    {
                        const auto n = static_cast<int>(data_store.satellite_count());
                        selected_row = std::min(std::max(1, n), selected_row.load() + 1);
                    }
                return true;
            }
        return false;
    });

    std::thread refresh([&] {
        while (g_running)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(refresh_ms.load()));
                screen.PostEvent(ftxui::Event::Custom);
            }
    });

    screen.Loop(app);

    g_running = false;
    refresh.join();
    listener.stop();
    return 0;
}
