
#include <gflags/gflags.h>


DEFINE_string(generator_binary, std::string(SW_GENERATOR_BIN), "Path of software-defined signal generator binary");
DEFINE_string(rinex_nav_file, std::string(DEFAULT_RINEX_NAV), "Input RINEX navigation file");
DEFINE_int32(duration, 100, "Duration of the experiment [in seconds, max = 300]");
DEFINE_string(static_position, "30.286502,120.032669,100", "Static receiver position [log,lat,height]");
DEFINE_string(dynamic_position, "", "Observer positions file, in .csv or .nmea format");
DEFINE_string(filename_rinex_obs, "sim.16o", "Filename of output RINEX navigation file");
DEFINE_string(filename_raw_data, "signal_out.bin", "Filename of output raw data file");
DEFINE_int32(fs_gen_hz, 2600000, "Samppling frequency [Hz]");
DEFINE_int32(test_satellite_PRN, 1, "PRN of the satellite under test (must be visible during the observation time)");
