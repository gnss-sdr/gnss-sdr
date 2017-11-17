#include "conjugate_sc.h"
#include <gnuradio/io_signature.h>
#include <volk_gnsssdr/volk_gnsssdr.h>


conjugate_sc_sptr make_conjugate_sc()
{
    return conjugate_sc_sptr(new conjugate_sc());
}


conjugate_sc::conjugate_sc() : gr::sync_block("conjugate_sc",
        gr::io_signature::make (1, 1, sizeof(lv_16sc_t)),
        gr::io_signature::make (1, 1, sizeof(lv_16sc_t)))
{
    const int alignment_multiple = volk_gnsssdr_get_alignment() / sizeof(lv_16sc_t);
    set_alignment(std::max(1, alignment_multiple));
}


int conjugate_sc::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{
    const lv_16sc_t *in = reinterpret_cast<const lv_16sc_t *>(input_items[0]);
    lv_16sc_t *out = reinterpret_cast<lv_16sc_t *>(output_items[0]);
    volk_gnsssdr_16ic_conjugate_16ic(out, in, noutput_items);
    return noutput_items;
}
