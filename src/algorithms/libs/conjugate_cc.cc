#include "conjugate_cc.h"
#include <gnuradio/io_signature.h>
#include <volk/volk.h>


conjugate_cc_sptr make_conjugate_cc()
{
    return conjugate_cc_sptr(new conjugate_cc());
}


conjugate_cc::conjugate_cc() : gr::sync_block("conjugate_cc",
        gr::io_signature::make (1, 1, sizeof(gr_complex)),
        gr::io_signature::make (1, 1, sizeof(gr_complex)))
{
    const int alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
    set_alignment(std::max(1, alignment_multiple));
}


int conjugate_cc::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
{
    const gr_complex *in = reinterpret_cast<const gr_complex *>(input_items[0]);
    gr_complex *out = reinterpret_cast<gr_complex *>(output_items[0]);
    volk_32fc_conjugate_32fc(out, in, noutput_items);
    return noutput_items;
}
