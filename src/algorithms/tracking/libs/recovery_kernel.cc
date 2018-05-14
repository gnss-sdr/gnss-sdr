#include "recover_kernel.h"
#include <cmath>


void recovery_kernel::set_params(
				 float init_phase,
                                 float incm,
				 unsigned chunk_length,
				 const lv_32fc_t corr_result,
                                 std::shared_ptr<lv_32fc_t> ref_sig
                                  )
{
  d_init_phase = std::exp(lv_32fc_t(0,init_phase));
  d_incm_phase = std::exp(lv_32fc_t(0,incm));
  d_chunk_length = chunk_length;
  d_ref_sig_sptr = ref_sig;
  d_corr_result = corr_result;
}

void recovery_kernel::put(lv_32fc_t* out,int length)
{
  memset(out,0,sizeof(lv_32fc_t) * length);
}

void recovery_kernel::put(lv_32fc_t* out, bool valid)
{
  if (valid)
    {
            lv_32fc_t  norm;
	    lv_32fc_t* in =  d_ref_sig_sptr.get();
	    
	    volk_32fc_x2_conjugate_dot_prod_32fc(
				                  &norm,
				                  in,
				                  in,
				                  d_chunk_length
				                );
	    
	    volk_32fc_s32fc_multiply_32fc(out,
					  in,
					  d_corr_result/ real(result),
					  d_chunk_length
					 );
	    
	    volk_32fc_s32fc_x2_rotator_32fc(out,
					    out,
					    &d_init_phase,
					    &d_incm_phase,
					    d_chunk_length
					   );
    }
  else
    {
      put(out, d_chunk_length);
    }
}






