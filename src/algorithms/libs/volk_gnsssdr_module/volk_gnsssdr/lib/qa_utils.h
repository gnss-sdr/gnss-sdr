/* Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef GNSS_SDR_VOLK_QA_UTILS_H
#define GNSS_SDR_VOLK_QA_UTILS_H

#ifdef __APPLE__
#define _DARWIN_C_SOURCE
#endif

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <volk_gnsssdr/volk_gnsssdr_common.h>

/************************************************
 * VOLK QA type definitions                     *
 ************************************************/
struct volk_gnsssdr_type_t {
    bool is_float;
    bool is_scalar;
    bool is_signed;
    bool is_complex;
    int size;
    std::string str;
};

class volk_gnsssdr_test_time_t {
    public:
        std::string name;
        double time;
        std::string units;
        bool pass;
};

class volk_gnsssdr_test_results_t {
    public:
        std::string name;
        std::string config_name;
        unsigned int vlen;
        unsigned int iter;
        std::map<std::string, volk_gnsssdr_test_time_t> results;
        std::string best_arch_a;
        std::string best_arch_u;
};

class volk_gnsssdr_test_params_t {
    private:
        float _tol;
        lv_32fc_t _scalar;
        unsigned int _vlen;
        unsigned int _iter;
        bool _benchmark_mode;
        std::string _kernel_regex;
    public:
        // ctor
        volk_gnsssdr_test_params_t(float tol, lv_32fc_t scalar, unsigned int vlen, unsigned int iter,
                           bool benchmark_mode, std::string kernel_regex) :
            _tol(tol), _scalar(scalar), _vlen(vlen), _iter(iter),
            _benchmark_mode(benchmark_mode), _kernel_regex(kernel_regex) {};
        // setters
        void set_tol(float tol) {_tol=tol;};
        void set_scalar(lv_32fc_t scalar) {_scalar=scalar;};
        void set_vlen(unsigned int vlen) {_vlen=vlen;};
        void set_iter(unsigned int iter) {_iter=iter;};
        void set_benchmark(bool benchmark) {_benchmark_mode=benchmark;};
        void set_regex(std::string regex) {_kernel_regex=regex;};
        // getters
        float tol() {return _tol;};
        lv_32fc_t scalar() {return _scalar;};
        unsigned int vlen() {return _vlen;};
        unsigned int iter() {return _iter;};
        bool benchmark_mode() {return _benchmark_mode;};
        std::string kernel_regex() {return _kernel_regex;};
};

class volk_gnsssdr_test_case_t {
    private:
        volk_gnsssdr_func_desc_t _desc;
        void(*_kernel_ptr)();
        std::string _name;
        volk_gnsssdr_test_params_t _test_parameters;
        std::string _puppet_master_name;
    public:
        volk_gnsssdr_func_desc_t desc() {return _desc;};
        void (*kernel_ptr()) () {return _kernel_ptr;};
        std::string name() {return _name;};
        std::string puppet_master_name() {return _puppet_master_name;};
        volk_gnsssdr_test_params_t test_parameters() {return _test_parameters;};
        // normal ctor
        volk_gnsssdr_test_case_t(volk_gnsssdr_func_desc_t desc, void(*kernel_ptr)(), std::string name,
            volk_gnsssdr_test_params_t test_parameters) :
            _desc(desc), _kernel_ptr(kernel_ptr), _name(name), _test_parameters(test_parameters),
            _puppet_master_name("NULL")
            {};
        // ctor for puppets
        volk_gnsssdr_test_case_t(volk_gnsssdr_func_desc_t desc, void(*kernel_ptr)(), std::string name,
            std::string puppet_master_name, volk_gnsssdr_test_params_t test_parameters) :
            _desc(desc), _kernel_ptr(kernel_ptr), _name(name), _test_parameters(test_parameters),
            _puppet_master_name(puppet_master_name)
            {};
};

/************************************************
 * VOLK QA functions                            *
 ************************************************/
volk_gnsssdr_type_t volk_gnsssdr_type_from_string(std::string);

float uniform(void);
void random_floats(float *buf, unsigned n);

bool run_volk_gnsssdr_tests(
    volk_gnsssdr_func_desc_t,
    void(*)(),
    std::string,
    volk_gnsssdr_test_params_t,
    std::vector<volk_gnsssdr_test_results_t> *results = NULL,
    std::string puppet_master_name = "NULL"
    );

bool run_volk_gnsssdr_tests(
        volk_gnsssdr_func_desc_t,
        void(*)(),
        std::string,
        float,
        lv_32fc_t,
        unsigned int,
        unsigned int,
        std::vector<volk_gnsssdr_test_results_t> *results = NULL,
        std::string puppet_master_name = "NULL",
        bool benchmark_mode = false
);


#define VOLK_RUN_TESTS(func, tol, scalar, len, iter) \
    BOOST_AUTO_TEST_CASE(func##_test) { \
        BOOST_CHECK_EQUAL(run_volk_gnsssdr_tests( \
            func##_get_func_desc(), (void (*)())func##_manual, \
            std::string(#func), tol, scalar, len, iter, 0, "NULL"), \
          0); \
    }
#define VOLK_PROFILE(func, test_params, results) run_volk_gnsssdr_tests(func##_get_func_desc(), (void (*)())func##_manual, std::string(#func), test_params, results, "NULL")
#define VOLK_PUPPET_PROFILE(func, puppet_master_func, test_params, results) run_volk_gnsssdr_tests(func##_get_func_desc(), (void (*)())func##_manual, std::string(#func), test_params, results, std::string(#puppet_master_func))
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
typedef void (*volk_gnsssdr_fn_1arg_s16ic)(void *, lv_16sc_t, unsigned int, const char*); //one input vector, one scalar lv_16sc_t vector input
typedef void (*volk_gnsssdr_fn_2arg_s16ic)(void *, void *, lv_16sc_t, unsigned int, const char*);
typedef void (*volk_gnsssdr_fn_3arg_s16ic)(void *, void *, void *, lv_16sc_t, unsigned int, const char*);
//ADDED BY GNSS-SDR. END


#endif // GNSS_SDR_VOLK_QA_UTILS_H
