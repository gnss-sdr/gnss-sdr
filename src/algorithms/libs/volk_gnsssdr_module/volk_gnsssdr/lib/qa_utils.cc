/* Copyright (C) 2010-2018 (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 */

#include "volk_gnsssdr/volk_gnsssdr.h"  // for volk_gnsssdr_func_desc_t
#include "qa_utils.h"

#include <volk_gnsssdr/volk_gnsssdr_malloc.h>  // for volk_gnsssdr_free, volk_gnsssdr_malloc
#include <cassert>                             // for assert
#include <chrono>                              // for system_clock, duration,...
#include <cmath>                               // for sqrt, fabs, abs
#include <cstdint>                             // for uint16_t, uint64_t,int16_t, int32_t
#include <cstring>                             // for memcpy, memset
#include <fstream>                             // for operator<<
#include <iostream>                            // for cout, cerr
#include <limits>                              // for numeric_limits
#include <map>                                 // for map
#include <random>                              // for random_device, default_random_engine, uniform_real_distribution
#include <vector>                              // for vector


float uniform()
{
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_real_distribution<float> uniform_dist(-1, 1);
    return uniform_dist(e1);  // uniformly (-1, 1)
}

template <class t>
void random_floats(t *buf, unsigned n)
{
    for (unsigned i = 0; i < n; i++)
        buf[i] = uniform();
}

void load_random_data(void *data, volk_gnsssdr_type_t type, unsigned int n)
{
    std::random_device r;
    std::default_random_engine e2(r());

    if (type.is_complex) n *= 2;

    if (type.is_float)
        {
            if (type.size == 8)
                random_floats<double>((double *)data, n);
            else
                random_floats<float>((float *)data, n);
        }
    else
        {
            float int_max = float(uint64_t(2) << (type.size * 8));
            if (type.is_signed) int_max /= 2.0;
            std::uniform_real_distribution<float> uniform_dist(-int_max, int_max);
            for (unsigned int i = 0; i < n; i++)
                {
                    float scaled_rand = uniform_dist(e2);

                    switch (type.size)
                        {
                        case 8:
                            if (type.is_signed)
                                ((int64_t *)data)[i] = (int64_t)scaled_rand;
                            else
                                ((uint64_t *)data)[i] = (uint64_t)scaled_rand;
                            break;
                        case 4:
                            if (type.is_signed)
                                ((int32_t *)data)[i] = (int32_t)scaled_rand;
                            else
                                ((uint32_t *)data)[i] = (uint32_t)scaled_rand;
                            break;
                        case 2:
                            // 16 bit multiplication saturates very fast
                            // we produce here only 3 bits input range
                            if (type.is_signed)
                                ((int16_t *)data)[i] = (int16_t)((int16_t)scaled_rand % 8);
                            else
                                ((uint16_t *)data)[i] = (uint16_t)(int16_t)((int16_t)scaled_rand % 8);
                            break;
                        case 1:
                            if (type.is_signed)
                                ((int8_t *)data)[i] = (int8_t)scaled_rand;
                            else
                                ((uint8_t *)data)[i] = (uint8_t)scaled_rand;
                            break;
                        default:
                            throw "load_random_data: no support for data size > 8 or < 1";  //no shenanigans here
                        }
                }
        }
}

static std::vector<std::string> get_arch_list(volk_gnsssdr_func_desc_t desc)
{
    std::vector<std::string> archlist;

    for (size_t i = 0; i < desc.n_impls; i++)
        {
            archlist.push_back(std::string(desc.impl_names[i]));
        }

    return archlist;
}

template <typename T>
T volk_lexical_cast(const std::string &str)
{
    for (unsigned int c_index = 0; c_index < str.size(); ++c_index)
        {
            if (str.at(c_index) < '0' || str.at(c_index) > '9')
                {
                    throw "not all numbers!";
                }
        }
    T var;
    std::istringstream iss;
    iss.str(str);
    iss >> var;
    // deal with any error bits that may have been set on the stream
    return var;
}

volk_gnsssdr_type_t volk_gnsssdr_type_from_string(std::string name)
{
    volk_gnsssdr_type_t type;
    type.is_float = false;
    type.is_scalar = false;
    type.is_complex = false;
    type.is_signed = false;
    type.size = 0;
    type.str = name;

    if (name.size() < 2)
        {
            throw std::string("name too short to be a datatype");
        }

    //is it a scalar?
    if (name[0] == 's')
        {
            type.is_scalar = true;
            name = name.substr(1, name.size() - 1);
        }

    //get the data size
    size_t last_size_pos = name.find_last_of("0123456789");
    if (last_size_pos == std::string::npos)
        {
            throw std::string("no size spec in type ").append(name);
        }
    //will throw if malformed
    int size = volk_lexical_cast<int>(name.substr(0, last_size_pos + 1));

    assert(((size % 8) == 0) && (size <= 64) && (size != 0));
    type.size = size / 8;  //in bytes

    for (size_t i = last_size_pos + 1; i < name.size(); i++)
        {
            switch (name[i])
                {
                case 'f':
                    type.is_float = true;
                    break;
                case 'i':
                    type.is_signed = true;
                    break;
                case 'c':
                    type.is_complex = true;
                    break;
                case 'u':
                    type.is_signed = false;
                    break;
                default:
                    throw;
                }
        }

    return type;
}

std::vector<std::string> split_signature(const std::string &protokernel_signature)
{
    std::vector<std::string> signature_tokens;
    std::string token;
    for (unsigned int loc = 0; loc < protokernel_signature.size(); ++loc)
        {
            if (protokernel_signature.at(loc) == '_')
                {
                    if (protokernel_signature.substr(loc + 1, 7).compare("gnsssdr") == 0)  // jump the "gnsssdr" part of "volk_gnsssdr"
                        {
                            loc += 7;
                        }
                    else
                        {
                            // this is a break
                            signature_tokens.push_back(token);
                            token = "";
                        }
                }
            else
                {
                    token.push_back(protokernel_signature.at(loc));
                }
        }
    // Get the last one to the end of the string
    signature_tokens.push_back(token);
    return signature_tokens;
}

static void get_signatures_from_name(std::vector<volk_gnsssdr_type_t> &inputsig,
    std::vector<volk_gnsssdr_type_t> &outputsig,
    std::string name)
{
    std::vector<std::string> toked = split_signature(name);
    assert(toked[0] == "volk");
    toked.erase(toked.begin());

    enum
    {
        SIDE_INPUT,
        SIDE_NAME,
        SIDE_OUTPUT
    } side = SIDE_INPUT;
    std::string fn_name;
    volk_gnsssdr_type_t type;
    for (unsigned int token_index = 0; token_index < toked.size(); ++token_index)
        {
            std::string token = toked[token_index];
            try
                {
                    type = volk_gnsssdr_type_from_string(token);
                    if (side == SIDE_NAME) side = SIDE_OUTPUT;  //if this is the first one after the name...

                    if (side == SIDE_INPUT)
                        inputsig.push_back(type);
                    else
                        outputsig.push_back(type);
                }
            catch (...)
                {
                    if (token[0] == 'x' && (token.size() > 1) && (token[1] > '0' || token[1] < '9'))
                        {
                            if (side == SIDE_INPUT)
                                assert(inputsig.size() > 0);
                            else
                                assert(outputsig.size() > 0);
                            int multiplier = volk_lexical_cast<int>(token.substr(1, token.size() - 1));  // will throw if invalid
                            for (int i = 1; i < multiplier; i++)
                                {
                                    if (side == SIDE_INPUT)
                                        inputsig.push_back(inputsig.back());
                                    else
                                        outputsig.push_back(outputsig.back());
                                }
                        }

                    else if (side == SIDE_INPUT)
                        {  //it's the function name, at least it better be
                            side = SIDE_NAME;
                            fn_name.append("_");
                            fn_name.append(token);
                        }
                    else if (side == SIDE_OUTPUT)
                        {
                            if (token != toked.back()) throw;  //the last token in the name is the alignment
                        }
                }
        }
    //we don't need an output signature (some fn's operate on the input data, "in place"), but we do need at least one input!
    assert(inputsig.size() != 0);
}

inline void run_cast_test1(volk_gnsssdr_fn_1arg func, std::vector<void *> &buffs, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], vlen, arch.c_str());
}

inline void run_cast_test2(volk_gnsssdr_fn_2arg func, std::vector<void *> &buffs, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], buffs[1], vlen, arch.c_str());
}

inline void run_cast_test3(volk_gnsssdr_fn_3arg func, std::vector<void *> &buffs, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], buffs[1], buffs[2], vlen, arch.c_str());
}

inline void run_cast_test4(volk_gnsssdr_fn_4arg func, std::vector<void *> &buffs, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], buffs[1], buffs[2], buffs[3], vlen, arch.c_str());
}

inline void run_cast_test1_s32f(volk_gnsssdr_fn_1arg_s32f func, std::vector<void *> &buffs, float scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], scalar, vlen, arch.c_str());
}

inline void run_cast_test2_s32f(volk_gnsssdr_fn_2arg_s32f func, std::vector<void *> &buffs, float scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], buffs[1], scalar, vlen, arch.c_str());
}

inline void run_cast_test3_s32f(volk_gnsssdr_fn_3arg_s32f func, std::vector<void *> &buffs, float scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], buffs[1], buffs[2], scalar, vlen, arch.c_str());
}

inline void run_cast_test1_s32fc(volk_gnsssdr_fn_1arg_s32fc func, std::vector<void *> &buffs, lv_32fc_t scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], scalar, vlen, arch.c_str());
}

inline void run_cast_test2_s32fc(volk_gnsssdr_fn_2arg_s32fc func, std::vector<void *> &buffs, lv_32fc_t scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], buffs[1], scalar, vlen, arch.c_str());
}

inline void run_cast_test3_s32fc(volk_gnsssdr_fn_3arg_s32fc func, std::vector<void *> &buffs, lv_32fc_t scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], buffs[1], buffs[2], scalar, vlen, arch.c_str());
}

// *************** ADDED BY GNSS-SDR. START
inline void run_cast_test1_s8i(volk_gnsssdr_fn_1arg_s8i func, std::vector<void *> &buffs, char scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], scalar, vlen, arch.c_str());
}

inline void run_cast_test2_s8i(volk_gnsssdr_fn_2arg_s8i func, std::vector<void *> &buffs, char scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], buffs[1], scalar, vlen, arch.c_str());
}

inline void run_cast_test3_s8i(volk_gnsssdr_fn_3arg_s8i func, std::vector<void *> &buffs, char scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], buffs[1], buffs[2], scalar, vlen, arch.c_str());
}
inline void run_cast_test1_s8ic(volk_gnsssdr_fn_1arg_s8ic func, std::vector<void *> &buffs, lv_8sc_t scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], scalar, vlen, arch.c_str());
}

inline void run_cast_test2_s8ic(volk_gnsssdr_fn_2arg_s8ic func, std::vector<void *> &buffs, lv_8sc_t scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], buffs[1], scalar, vlen, arch.c_str());
}

inline void run_cast_test3_s8ic(volk_gnsssdr_fn_3arg_s8ic func, std::vector<void *> &buffs, lv_8sc_t scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], buffs[1], buffs[2], scalar, vlen, arch.c_str());
}

inline void run_cast_test1_s16ic(volk_gnsssdr_fn_1arg_s16ic func, std::vector<void *> &buffs, lv_16sc_t scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], scalar, vlen, arch.c_str());
}

inline void run_cast_test2_s16ic(volk_gnsssdr_fn_2arg_s16ic func, std::vector<void *> &buffs, lv_16sc_t scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], buffs[1], scalar, vlen, arch.c_str());
}

inline void run_cast_test3_s16ic(volk_gnsssdr_fn_3arg_s16ic func, std::vector<void *> &buffs, lv_16sc_t scalar, unsigned int vlen, unsigned int iter, std::string arch)
{
    while (iter--) func(buffs[0], buffs[1], buffs[2], scalar, vlen, arch.c_str());
}
// *************** ADDED BY GNSS-SDR. END

template <class t>
bool fcompare(t *in1, t *in2, unsigned int vlen, float tol)
{
    bool fail = false;
    int print_max_errs = 10;
    for (unsigned int i = 0; i < vlen; i++)
        {
            // for very small numbers we'll see round off errors due to limited
            // precision. So a special test case...
            if (fabs(((t *)(in1))[i]) < 1e-30)
                {
                    if (fabs(((t *)(in2))[i]) > tol)
                        {
                            fail = true;
                            if (print_max_errs-- > 0)
                                {
                                    std::cout << "offset " << i << " in1: " << t(((t *)(in1))[i]) << " in2: " << t(((t *)(in2))[i]);
                                    std::cout << " tolerance was: " << tol << std::endl;
                                }
                        }
                }
            // the primary test is the percent different greater than given tol
            else if (fabs(((t *)(in1))[i] - ((t *)(in2))[i]) / fabs(((t *)in1)[i]) > tol)
                {
                    fail = true;
                    if (print_max_errs-- > 0)
                        {
                            std::cout << "offset " << i << " in1: " << t(((t *)(in1))[i]) << " in2: " << t(((t *)(in2))[i]);
                            std::cout << " tolerance was: " << tol << std::endl;
                        }
                }
        }

    return fail;
}

template <class t>
bool ccompare(t *in1, t *in2, unsigned int vlen, float tol)
{
    bool fail = false;
    int print_max_errs = 10;
    for (unsigned int i = 0; i < 2 * vlen; i += 2)
        {
            t diff[2] = {in1[i] - in2[i], in1[i + 1] - in2[i + 1]};
            t err = std::sqrt(diff[0] * diff[0] + diff[1] * diff[1]);
            t norm = std::sqrt(in1[i] * in1[i] + in1[i + 1] * in1[i + 1]);

            // for very small numbers we'll see round off errors due to limited
            // precision. So a special test case...
            if (norm < 1e-30)
                {
                    if (err > tol)
                        {
                            fail = true;
                            if (print_max_errs-- > 0)
                                {
                                    std::cout << "offset " << i / 2 << " in1: " << in1[i] << " + " << in1[i + 1] << "j  in2: " << in2[i] << " + " << in2[i + 1] << "j";
                                    std::cout << " tolerance was: " << tol << std::endl;
                                }
                        }
                }
            // the primary test is the percent different greater than given tol
            else if ((err / norm) > tol)
                {
                    fail = true;
                    if (print_max_errs-- > 0)
                        {
                            std::cout << "offset " << i / 2 << " in1: " << in1[i] << " + " << in1[i + 1] << "j  in2: " << in2[i] << " + " << in2[i + 1] << "j";
                            std::cout << " tolerance was: " << tol << std::endl;
                        }
                }
        }

    return fail;
}

template <class t>
bool icompare(t *in1, t *in2, unsigned int vlen, unsigned int tol)
{
    bool fail = false;
    int print_max_errs = 10;
    for (unsigned int i = 0; i < vlen; i++)
        {
            if (((unsigned int)abs(int(((t *)(in1))[i]) - int(((t *)(in2))[i]))) > tol)
                {
                    fail = true;
                    if (print_max_errs-- > 0)
                        {
                            std::cout << "offset " << i << " in1: " << static_cast<int>(t(((t *)(in1))[i])) << " in2: " << static_cast<int>(t(((t *)(in2))[i]));
                            std::cout << " tolerance was: " << tol << std::endl;
                        }
                }
        }

    return fail;
}

class volk_gnsssdr_qa_aligned_mem_pool
{
public:
    void *get_new(size_t size)
    {
        size_t alignment = volk_gnsssdr_get_alignment();
        void *ptr = volk_gnsssdr_malloc(size, alignment);
        memset(ptr, 0x00, size);
        _mems.push_back(ptr);
        return ptr;
    }
    ~volk_gnsssdr_qa_aligned_mem_pool()
    {
        for (unsigned int ii = 0; ii < _mems.size(); ++ii)
            {
                volk_gnsssdr_free(_mems[ii]);
            }
    }

private:
    std::vector<void *> _mems;
};

bool run_volk_gnsssdr_tests(volk_gnsssdr_func_desc_t desc,
    void (*manual_func)(),
    std::string name,
    volk_gnsssdr_test_params_t test_params,
    std::vector<volk_gnsssdr_test_results_t> *results,
    std::string puppet_master_name)
{
    return run_volk_gnsssdr_tests(desc, manual_func, name, test_params.tol(), test_params.scalar(),
        test_params.vlen(), test_params.iter(), results, puppet_master_name,
        test_params.benchmark_mode());
}

bool run_volk_gnsssdr_tests(volk_gnsssdr_func_desc_t desc,
    void (*manual_func)(),
    std::string name,
    float tol,
    lv_32fc_t scalar,
    unsigned int vlen,
    unsigned int iter,
    std::vector<volk_gnsssdr_test_results_t> *results,
    std::string puppet_master_name,
    bool benchmark_mode)
{
    // Initialize this entry in results vector
    results->push_back(volk_gnsssdr_test_results_t());
    results->back().name = name;
    results->back().vlen = vlen;
    results->back().iter = iter;
    std::cout << "RUN_VOLK_GNSSSDR_TESTS: " << name << "(" << vlen << "," << iter << ")" << std::endl;

    // vlen_twiddle will increase vlen for malloc and data generation
    // but kernels will still be called with the user provided vlen.
    // This is useful for causing errors in kernels that do bad reads
    const unsigned int vlen_twiddle = 5;
    vlen = vlen + vlen_twiddle;

    const float tol_f = tol;
    const unsigned int tol_i = static_cast<unsigned int>(tol);

    //first let's get a list of available architectures for the test
    std::vector<std::string> arch_list = get_arch_list(desc);

    if ((!benchmark_mode) && (arch_list.size() < 2))
        {
            std::cout << "no architectures to test" << std::endl;
            return false;
        }

    //something that can hang onto memory and cleanup when this function exits
    volk_gnsssdr_qa_aligned_mem_pool mem_pool;

    //now we have to get a function signature by parsing the name
    std::vector<volk_gnsssdr_type_t> inputsig, outputsig;
    try
        {
            get_signatures_from_name(inputsig, outputsig, name);
        }
    catch (std::exception &error)
        {
            std::cerr << "Error: unable to get function signature from kernel name" << std::endl;
            std::cerr << "  - " << name << std::endl;
            return false;
        }
    catch (std::string s)
        {
            std::cerr << "Error: " << s << std::endl;
            return false;
        }

    //pull the input scalars into their own vector
    std::vector<volk_gnsssdr_type_t> inputsc;
    for (size_t i = 0; i < inputsig.size(); i++)
        {
            if (inputsig[i].is_scalar)
                {
                    inputsc.push_back(inputsig[i]);
                    inputsig.erase(inputsig.begin() + i);
                    i -= 1;
                }
        }
    std::vector<void *> inbuffs;
    for (unsigned int inputsig_index = 0; inputsig_index < inputsig.size(); ++inputsig_index)
        {
            volk_gnsssdr_type_t sig = inputsig[inputsig_index];
            if (!sig.is_scalar)  //we don't make buffers for scalars
                inbuffs.push_back(mem_pool.get_new(vlen * sig.size * (sig.is_complex ? 2 : 1)));
        }
    for (size_t i = 0; i < inbuffs.size(); i++)
        {
            load_random_data(inbuffs[i], inputsig[i], vlen);
        }

    //ok let's make a vector of vector of void buffers, which holds the input/output vectors for each arch
    std::vector<std::vector<void *> > test_data;
    for (size_t i = 0; i < arch_list.size(); i++)
        {
            std::vector<void *> arch_buffs;
            for (size_t j = 0; j < outputsig.size(); j++)
                {
                    arch_buffs.push_back(mem_pool.get_new(vlen * outputsig[j].size * (outputsig[j].is_complex ? 2 : 1)));
                }
            for (size_t j = 0; j < inputsig.size(); j++)
                {
                    void *arch_inbuff = mem_pool.get_new(vlen * inputsig[j].size * (inputsig[j].is_complex ? 2 : 1));
                    memcpy(arch_inbuff, inbuffs[j], vlen * inputsig[j].size * (inputsig[j].is_complex ? 2 : 1));
                    arch_buffs.push_back(arch_inbuff);
                }
            test_data.push_back(arch_buffs);
        }

    std::vector<volk_gnsssdr_type_t> both_sigs;
    both_sigs.insert(both_sigs.end(), outputsig.begin(), outputsig.end());
    both_sigs.insert(both_sigs.end(), inputsig.begin(), inputsig.end());

    //now run the test
    vlen = vlen - vlen_twiddle;
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::vector<double> profile_times;
    for (size_t i = 0; i < arch_list.size(); i++)
        {
            start = std::chrono::system_clock::now();

            switch (both_sigs.size())
                {
                case 1:
                    if (inputsc.size() == 0)
                        {
                            run_cast_test1((volk_gnsssdr_fn_1arg)(manual_func), test_data[i], vlen, iter, arch_list[i]);
                        }
                    else if (inputsc.size() == 1 && inputsc[0].is_float)
                        {
                            if (inputsc[0].is_complex)
                                {
                                    run_cast_test1_s32fc((volk_gnsssdr_fn_1arg_s32fc)(manual_func), test_data[i], scalar, vlen, iter, arch_list[i]);
                                }
                            else
                                {
                                    run_cast_test1_s32f((volk_gnsssdr_fn_1arg_s32f)(manual_func), test_data[i], scalar.real(), vlen, iter, arch_list[i]);
                                }
                        }
                    //ADDED BY GNSS-SDR. START
                    else if (inputsc.size() == 1 && !inputsc[0].is_float)
                        {
                            if (inputsc[0].is_complex)
                                {
                                    if (inputsc[0].size == 2)
                                        {
                                            run_cast_test1_s16ic((volk_gnsssdr_fn_1arg_s16ic)(manual_func), test_data[i], scalar, vlen, iter, arch_list[i]);
                                        }
                                    else
                                        {
                                            run_cast_test1_s8ic((volk_gnsssdr_fn_1arg_s8ic)(manual_func), test_data[i], scalar, vlen, iter, arch_list[i]);
                                        }
                                }
                            else
                                {
                                    run_cast_test1_s8i((volk_gnsssdr_fn_1arg_s8i)(manual_func), test_data[i], scalar.real(), vlen, iter, arch_list[i]);
                                }
                        }
                    //ADDED BY GNSS-SDR. END
                    else
                        throw "unsupported 1 arg function >1 scalars";
                    break;
                case 2:
                    if (inputsc.size() == 0)
                        {
                            run_cast_test2((volk_gnsssdr_fn_2arg)(manual_func), test_data[i], vlen, iter, arch_list[i]);
                        }
                    else if (inputsc.size() == 1 && inputsc[0].is_float)
                        {
                            if (inputsc[0].is_complex)
                                {
                                    run_cast_test2_s32fc((volk_gnsssdr_fn_2arg_s32fc)(manual_func), test_data[i], scalar, vlen, iter, arch_list[i]);
                                }
                            else
                                {
                                    run_cast_test2_s32f((volk_gnsssdr_fn_2arg_s32f)(manual_func), test_data[i], scalar.real(), vlen, iter, arch_list[i]);
                                }
                        }
                    //ADDED BY GNSS-SDR. START
                    else if (inputsc.size() == 1 && !inputsc[0].is_float)
                        {
                            if (inputsc[0].is_complex)
                                {
                                    if (inputsc[0].size == 2)
                                        {
                                            run_cast_test2_s16ic((volk_gnsssdr_fn_2arg_s16ic)(manual_func), test_data[i], scalar, vlen, iter, arch_list[i]);
                                        }
                                    else
                                        {
                                            run_cast_test2_s8ic((volk_gnsssdr_fn_2arg_s8ic)(manual_func), test_data[i], scalar, vlen, iter, arch_list[i]);
                                        }
                                }
                            else
                                {
                                    run_cast_test2_s8i((volk_gnsssdr_fn_2arg_s8i)(manual_func), test_data[i], scalar.real(), vlen, iter, arch_list[i]);
                                }
                        }
                    //ADDED BY GNSS-SDR. END
                    else
                        throw "unsupported 2 arg function >1 scalars";
                    break;
                case 3:
                    if (inputsc.size() == 0)
                        {
                            run_cast_test3((volk_gnsssdr_fn_3arg)(manual_func), test_data[i], vlen, iter, arch_list[i]);
                        }
                    else if (inputsc.size() == 1 && inputsc[0].is_float)
                        {
                            if (inputsc[0].is_complex)
                                {
                                    run_cast_test3_s32fc((volk_gnsssdr_fn_3arg_s32fc)(manual_func), test_data[i], scalar, vlen, iter, arch_list[i]);
                                }
                            else
                                {
                                    run_cast_test3_s32f((volk_gnsssdr_fn_3arg_s32f)(manual_func), test_data[i], scalar.real(), vlen, iter, arch_list[i]);
                                }
                        }
                    //ADDED BY GNSS-SDR. START
                    else if (inputsc.size() == 1 && !inputsc[0].is_float)
                        {
                            if (inputsc[0].is_complex)
                                {
                                    {
                                        if (inputsc[0].size == 4)
                                            {
                                                run_cast_test3_s16ic((volk_gnsssdr_fn_3arg_s16ic)(manual_func), test_data[i], scalar, vlen, iter, arch_list[i]);
                                            }
                                        else
                                            {
                                                run_cast_test3_s8ic((volk_gnsssdr_fn_3arg_s8ic)(manual_func), test_data[i], scalar, vlen, iter, arch_list[i]);
                                            }
                                    }
                                }
                            else
                                {
                                    run_cast_test3_s8i((volk_gnsssdr_fn_3arg_s8i)(manual_func), test_data[i], scalar.real(), vlen, iter, arch_list[i]);
                                }
                        }
                    //ADDED BY GNSS-SDR. END
                    else
                        throw "unsupported 3 arg function >1 scalars";
                    break;
                case 4:
                    run_cast_test4((volk_gnsssdr_fn_4arg)(manual_func), test_data[i], vlen, iter, arch_list[i]);
                    break;
                default:
                    throw "no function handler for this signature";
                    break;
                }

            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            double arch_time = 1000.0 * elapsed_seconds.count();
            std::cout << arch_list[i] << " completed in " << arch_time << " ms" << std::endl;
            volk_gnsssdr_test_time_t result;
            result.name = arch_list[i];
            result.time = arch_time;
            result.units = "ms";
            result.pass = true;
            results->back().results[result.name] = result;

            profile_times.push_back(arch_time);
        }

    //and now compare each output to the generic output
    //first we have to know which output is the generic one, they aren't in order...
    size_t generic_offset = 0;
    for (size_t i = 0; i < arch_list.size(); i++)
        {
            if (arch_list[i] == "generic")
                {
                    generic_offset = i;
                }
        }

    // Just in case a kernel wrote to OOB memory, use the twiddled vlen
    vlen = vlen + vlen_twiddle;
    bool fail;
    bool fail_global = false;
    std::vector<bool> arch_results;
    for (size_t i = 0; i < arch_list.size(); i++)
        {
            fail = false;
            if (i != generic_offset)
                {
                    for (size_t j = 0; j < both_sigs.size(); j++)
                        {
                            if (both_sigs[j].is_float)
                                {
                                    if (both_sigs[j].size == 8)
                                        {
                                            if (both_sigs[j].is_complex)
                                                {
                                                    fail = ccompare((double *)test_data[generic_offset][j], (double *)test_data[i][j], vlen, tol_f);
                                                }
                                            else
                                                {
                                                    fail = fcompare((double *)test_data[generic_offset][j], (double *)test_data[i][j], vlen, tol_f);
                                                }
                                        }
                                    else
                                        {
                                            if (both_sigs[j].is_complex)
                                                {
                                                    fail = ccompare((float *)test_data[generic_offset][j], (float *)test_data[i][j], vlen, tol_f);
                                                }
                                            else
                                                {
                                                    fail = fcompare((float *)test_data[generic_offset][j], (float *)test_data[i][j], vlen, tol_f);
                                                }
                                        }
                                }
                            else
                                {
                                    //i could replace this whole switch statement with a memcmp if i wasn't interested in printing the outputs where they differ
                                    switch (both_sigs[j].size)
                                        {
                                        case 8:
                                            if (both_sigs[j].is_signed)
                                                {
                                                    fail = icompare((int64_t *)test_data[generic_offset][j], (int64_t *)test_data[i][j], vlen * (both_sigs[j].is_complex ? 2 : 1), tol_i);
                                                }
                                            else
                                                {
                                                    fail = icompare((uint64_t *)test_data[generic_offset][j], (uint64_t *)test_data[i][j], vlen * (both_sigs[j].is_complex ? 2 : 1), tol_i);
                                                }
                                            break;
                                        case 4:
                                            if (both_sigs[j].is_complex)  // ADDED BY GNSS_SDR
                                                {
                                                    if (both_sigs[j].is_signed)
                                                        {
                                                            fail = icompare((int16_t *)test_data[generic_offset][j], (int16_t *)test_data[i][j], vlen * (both_sigs[j].is_complex ? 2 : 1), tol_i);
                                                        }
                                                    else
                                                        {
                                                            fail = icompare((uint16_t *)test_data[generic_offset][j], (uint16_t *)test_data[i][j], vlen * (both_sigs[j].is_complex ? 2 : 1), tol_i);
                                                        }
                                                }
                                            else
                                                {
                                                    if (both_sigs[j].is_signed)
                                                        {
                                                            fail = icompare((int32_t *)test_data[generic_offset][j], (int32_t *)test_data[i][j], vlen * (both_sigs[j].is_complex ? 2 : 1), tol_i);
                                                        }
                                                    else
                                                        {
                                                            fail = icompare((uint32_t *)test_data[generic_offset][j], (uint32_t *)test_data[i][j], vlen * (both_sigs[j].is_complex ? 2 : 1), tol_i);
                                                        }
                                                }
                                            break;
                                        case 2:
                                            if (both_sigs[j].is_complex)  // ADDED BY GNSS_SDR
                                                {
                                                    if (both_sigs[j].is_signed)
                                                        {
                                                            fail = icompare((int8_t *)test_data[generic_offset][j], (int8_t *)test_data[i][j], vlen * (both_sigs[j].is_complex ? 2 : 1), tol_i);
                                                        }
                                                    else
                                                        {
                                                            fail = icompare((uint8_t *)test_data[generic_offset][j], (uint8_t *)test_data[i][j], vlen * (both_sigs[j].is_complex ? 2 : 1), tol_i);
                                                        }
                                                }
                                            else
                                                {
                                                    if (both_sigs[j].is_signed)
                                                        {
                                                            fail = icompare((int16_t *)test_data[generic_offset][j], (int16_t *)test_data[i][j], vlen * (both_sigs[j].is_complex ? 2 : 1), tol_i);  //
                                                        }
                                                    else
                                                        {
                                                            fail = icompare((uint16_t *)test_data[generic_offset][j], (uint16_t *)test_data[i][j], vlen * (both_sigs[j].is_complex ? 2 : 1), tol_i);
                                                        }
                                                }
                                            break;
                                        case 1:
                                            if (both_sigs[j].is_signed)
                                                {
                                                    fail = icompare((int8_t *)test_data[generic_offset][j], (int8_t *)test_data[i][j], vlen * (both_sigs[j].is_complex ? 2 : 1), tol_i);  // check volk_gnsssdr_32fc_convert_8ic !
                                                }
                                            else
                                                {
                                                    fail = icompare((uint8_t *)test_data[generic_offset][j], (uint8_t *)test_data[i][j], vlen * (both_sigs[j].is_complex ? 2 : 1), tol_i);
                                                }
                                            break;
                                        default:
                                            fail = 1;
                                        }
                                }
                            if (fail)
                                {
                                    volk_gnsssdr_test_time_t *result = &results->back().results[arch_list[i]];
                                    result->pass = !fail;
                                    fail_global = true;
                                    std::cout << name << ": fail on arch " << arch_list[i] << std::endl;
                                }
                        }
                }
            arch_results.push_back(!fail);
        }

    double best_time_a = std::numeric_limits<double>::max();
    double best_time_u = std::numeric_limits<double>::max();
    std::string best_arch_a = "generic";
    std::string best_arch_u = "generic";
    for (size_t i = 0; i < arch_list.size(); i++)
        {
            if ((profile_times[i] < best_time_u) && arch_results[i] && desc.impl_alignment[i] == 0)
                {
                    best_time_u = profile_times[i];
                    best_arch_u = arch_list[i];
                }
            if ((profile_times[i] < best_time_a) && arch_results[i])
                {
                    best_time_a = profile_times[i];
                    best_arch_a = arch_list[i];
                }
        }

    std::cout << "Best aligned arch: " << best_arch_a << std::endl;
    std::cout << "Best unaligned arch: " << best_arch_u << std::endl;

    if (puppet_master_name == "NULL")
        {
            results->back().config_name = name;
        }
    else
        {
            results->back().config_name = puppet_master_name;
        }
    results->back().best_arch_a = best_arch_a;
    results->back().best_arch_u = best_arch_u;

    return fail_global;
}
