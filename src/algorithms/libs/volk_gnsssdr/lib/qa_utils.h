#ifndef VOLK_QA_UTILS_H
#define VOLK_QA_UTILS_H

#include <cstdlib>
#include <string>
#include <vector>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_common.h>

struct volk_gnsssdr_type_t {
    bool is_float;
    bool is_scalar;
    bool is_signed;
    bool is_complex;
    int size;
    std::string str;
};

volk_gnsssdr_type_t volk_gnsssdr_type_from_string(std::string);

float uniform(void);
void random_floats(float *buf, unsigned n);

bool run_volk_gnsssdr_tests(volk_gnsssdr_func_desc_t, void(*)(), std::string, float, lv_32fc_t, int, int, std::vector<std::string> *, std::string, bool benchmark_mode=false, std::string kernel_regex="");


#define VOLK_RUN_TESTS(func, tol, scalar, len, iter) BOOST_AUTO_TEST_CASE(func##_test) { BOOST_CHECK_EQUAL(run_volk_gnsssdr_tests(func##_get_func_desc(), (void (*)())func##_manual, std::string(#func), tol, scalar, len, iter, 0, "NULL"), 0); }
#define VOLK_PROFILE(func, tol, scalar, len, iter, results, bnmode, kernel_regex) run_volk_gnsssdr_tests(func##_get_func_desc(), (void (*)())func##_manual, std::string(#func), tol, scalar, len, iter, results, "NULL", bnmode, kernel_regex)
#define VOLK_PUPPET_PROFILE(func, puppet_master_func, tol, scalar, len, iter, results, bnmode, kernel_regex) run_volk_gnsssdr_tests(func##_get_func_desc(), (void (*)())func##_manual, std::string(#func), tol, scalar, len, iter, results, std::string(#puppet_master_func), bnmode, kernel_regex)
typedef void (*volk_gnsssdr_fn_1arg)(void *, unsigned int, const char*); //one input, operate in place
typedef void (*volk_gnsssdr_fn_2arg)(void *, void *, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_3arg)(void *, void *, void *, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_4arg)(void *, void *, void *, void *, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_1arg_s32f)(void *, float, unsigned int, const char*); //one input vector, one scalar float input
typedef void (*volk_gnsssdr_fn_2arg_s32f)(void *, void *, float, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_3arg_s32f)(void *, void *, void *, float, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_1arg_s32fc)(void *, lv_32fc_t, unsigned int, const char*); //one input vector, one scalar float input
typedef void (*volk_gnsssdr_fn_2arg_s32fc)(void *, void *, lv_32fc_t, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_3arg_s32fc)(void *, void *, void *, lv_32fc_t, unsigned int, const char*);

//ADDED BY GNSS-SDR. START
typedef void (*volk_gnsssdr_fn_1arg_s8i)(void *, char, unsigned int, const char*); //one input vector, one scalar char input
typedef void (*volk_gnsssdr_fn_2arg_s8i)(void *, void *, char, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_3arg_s8i)(void *, void *, void *, char, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_1arg_s8ic)(void *, lv_8sc_t, unsigned int, const char*); //one input vector, one scalar lv_8sc_t vector input
typedef void (*volk_gnsssdr_fn_2arg_s8ic)(void *, void *, lv_8sc_t, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_3arg_s8ic)(void *, void *, void *, lv_8sc_t, unsigned int, const char*);

typedef void (*volk_gnsssdr_fn_8arg)(void *, void *, void *, void *, void *, void *, void *, void *, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_8arg_s32f)(void *, void *, void *, void *, void *, void *, void *, void *, float, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_8arg_s32fc)(void *, void *, void *, void *, void *, void *, void *, void *, lv_32fc_t, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_8arg_s8i)(void *, void *, void *, void *, void *, void *, void *, void *, char, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_8arg_s8ic)(void *, void *, void *, void *, void *, void *, void *, void *, lv_8sc_t, unsigned int, const char*);

typedef void (*volk_gnsssdr_fn_12arg)(void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_12arg_s32f)(void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, float, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_12arg_s32fc)(void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, lv_32fc_t, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_12arg_s8i)(void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, char, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_12arg_s8ic)(void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, void *, lv_8sc_t, unsigned int, const char*);
//ADDED BY GNSS-SDR. END

#endif //VOLK_QA_UTILS_H
