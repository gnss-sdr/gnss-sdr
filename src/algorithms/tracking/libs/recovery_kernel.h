
#include <volk/volk.h>
#include <memory>

class recovery_kernel
{
   public:
     void set_params(
                 float init_phase,
                 float incm,
                 unsigned chunk_length,
                 const lv_32fc_t corr_res,
                 std::shared_ptr<lv_32fc_t> ref_sig
                      );
     void put(lv_32fc_t* out,bool valid);
     void put(lv_32fc_t* out,int length);
     recover_kernel()  {}
     ~recover_kernel() {}
   private:
     lv_32fc_t d_init_phase;
     lv_32fc_t d_incm_phase;
     lv_32fc_t d_corr_result;
     unsigned d_chunk_length;
     std::shared_ptr<lv_32fc_t> d_ref_sig_sptr; 
};
