#include "dll_pll_conf.h"
#include <cstring>

Dll_Pll_Conf::Dll_Pll_Conf()
{
    /* DLL/PLL tracking configuration */
    fs_in = 0.0;
    vector_length = 0;
    dump = false;
    dump_filename = "./dll_pll_dump.dat";
    pll_bw_hz = 40.0;
    dll_bw_hz = 2.0;
    pll_bw_narrow_hz = 5.0;
    dll_bw_narrow_hz = 0.75;
    early_late_space_chips = 0.5;
    very_early_late_space_chips = 0.5;
    early_late_space_narrow_chips = 0.1;
    very_early_late_space_narrow_chips = 0.1;
    extend_correlation_symbols = 5;
    cn0_samples = 20;
    carrier_lock_det_mav_samples = 20;
    cn0_min = 25;
    max_lock_fail = 50;
    carrier_lock_th = 0.85;
    track_pilot = false;
    system = 'G';
    char sig_[3] = "1C";
    std::memcpy(signal, sig_, 3);
}
