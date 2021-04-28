//
// gsl-lite is based on GSL: Guidelines Support Library.
// For more information see https://github.com/gsl-lite/gsl-lite
//
// Copyright (c) 2015-2019 Martin Moene
// Copyright (c) 2019-2021 Moritz Beutel
// Copyright (c) 2015-2018 Microsoft Corporation. All rights reserved.
//
// SPDX-License-Identifier: MIT
//


#ifndef GSL_GSL_LITE_HPP_INCLUDED
#define GSL_GSL_LITE_HPP_INCLUDED

#include <cstddef>    // for size_t, ptrdiff_t, nullptr_t
#include <exception>  // for exception, terminate(), uncaught_exceptions()
#include <ios>        // for ios_base, streamsize
#include <iosfwd>     // for basic_ostream<>
#include <limits>
#include <memory>     // for addressof(), unique_ptr<>, shared_ptr<>
#include <stdexcept>  // for logic_error
#include <string>
#include <utility>  // for move(), forward<>(), swap()

#define gsl_lite_MAJOR 0
#define gsl_lite_MINOR 38
#define gsl_lite_PATCH 1

#define gsl_lite_VERSION gsl_STRINGIFY(gsl_lite_MAJOR) "." gsl_STRINGIFY(gsl_lite_MINOR) "." gsl_STRINGIFY(gsl_lite_PATCH)

#define gsl_STRINGIFY(x) gsl_STRINGIFY_(x)
#define gsl_STRINGIFY_(x) #x
#define gsl_CONCAT_(a, b) gsl_CONCAT2_(a, b)
#define gsl_CONCAT2_(a, b) a##b
#define gsl_EVALF_(f) f()

// configuration argument checking:

#define gsl_DETAIL_CFG_TOGGLE_VALUE_1 1
#define gsl_DETAIL_CFG_TOGGLE_VALUE_0 1
#define gsl_DETAIL_CFG_DEFAULTS_VERSION_VALUE_1 1
#define gsl_DETAIL_CFG_DEFAULTS_VERSION_VALUE_0 1
#define gsl_DETAIL_CFG_STD_VALUE_98 1
#define gsl_DETAIL_CFG_STD_VALUE_3 1
#define gsl_DETAIL_CFG_STD_VALUE_03 1
#define gsl_DETAIL_CFG_STD_VALUE_11 1
#define gsl_DETAIL_CFG_STD_VALUE_14 1
#define gsl_DETAIL_CFG_STD_VALUE_17 1
#define gsl_DETAIL_CFG_STD_VALUE_20 1
#define gsl_DETAIL_CFG_NO_VALUE_ 1
#define gsl_DETAIL_CFG_NO_VALUE_1 1  // many compilers treat the command-line parameter "-Dfoo" as equivalent to "-Dfoo=1", so we tolerate that
#define gsl_CHECK_CFG_TOGGLE_VALUE_(x) gsl_CONCAT_(gsl_DETAIL_CFG_TOGGLE_VALUE_, x)
#define gsl_CHECK_CFG_DEFAULTS_VERSION_VALUE_(x) gsl_CONCAT_(gsl_DETAIL_CFG_DEFAULTS_VERSION_VALUE_, x)
#define gsl_CHECK_CFG_STD_VALUE_(x) gsl_CONCAT_(gsl_DETAIL_CFG_STD_VALUE_, x)
#define gsl_CHECK_CFG_NO_VALUE_(x) gsl_CONCAT_(gsl_DETAIL_CFG_NO_VALUE, gsl_CONCAT_(_, x))

// gsl-lite backward compatibility:

#if defined(gsl_CONFIG_DEFAULTS_VERSION)
#if !gsl_CHECK_CFG_DEFAULTS_VERSION_VALUE_(gsl_CONFIG_DEFAULTS_VERSION)
#pragma message("invalid configuration value gsl_CONFIG_DEFAULTS_VERSION=" gsl_STRINGIFY(gsl_CONFIG_DEFAULTS_VERSION) ", must be 0 or 1")
#endif
#else
#define gsl_CONFIG_DEFAULTS_VERSION gsl_lite_MAJOR  // default
#endif
#define gsl_CONFIG_DEFAULTS_VERSION_() gsl_CONFIG_DEFAULTS_VERSION

#if defined(gsl_CONFIG_ALLOWS_SPAN_CONTAINER_CTOR)
#if !gsl_CHECK_CFG_TOGGLE_VALUE_(gsl_CONFIG_ALLOWS_SPAN_CONTAINER_CTOR)
#pragma message("invalid configuration value gsl_CONFIG_ALLOWS_SPAN_CONTAINER_CTOR=" gsl_STRINGIFY(gsl_CONFIG_ALLOWS_SPAN_CONTAINER_CTOR) ", must be 0 or 1")
#endif
#define gsl_CONFIG_ALLOWS_UNCONSTRAINED_SPAN_CONTAINER_CTOR gsl_CONFIG_ALLOWS_SPAN_CONTAINER_CTOR
#pragma message("gsl_CONFIG_ALLOWS_SPAN_CONTAINER_CTOR is deprecated since gsl-lite 0.7; replace with gsl_CONFIG_ALLOWS_UNCONSTRAINED_SPAN_CONTAINER_CTOR, or consider span(with_container, cont).")
#endif

#if defined(gsl_CONFIG_CONTRACT_LEVEL_ON)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_LEVEL_ON)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_LEVEL_ON=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_LEVEL_ON) "; macro must be defined without value")
#endif
#pragma message("gsl_CONFIG_CONTRACT_LEVEL_ON is deprecated since gsl-lite 0.36; replace with gsl_CONFIG_CONTRACT_CHECKING_ON.")
#define gsl_CONFIG_CONTRACT_CHECKING_ON
#endif
#if defined(gsl_CONFIG_CONTRACT_LEVEL_OFF)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_LEVEL_OFF)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_LEVEL_OFF=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_LEVEL_OFF) "; macro must be defined without value")
#endif
#pragma message("gsl_CONFIG_CONTRACT_LEVEL_OFF is deprecated since gsl-lite 0.36; replace with gsl_CONFIG_CONTRACT_CHECKING_OFF.")
#define gsl_CONFIG_CONTRACT_CHECKING_OFF
#endif
#if defined(gsl_CONFIG_CONTRACT_LEVEL_EXPECTS_ONLY)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_LEVEL_EXPECTS_ONLY)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_LEVEL_EXPECTS_ONLY=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_LEVEL_EXPECTS_ONLY) "; macro must be defined without value")
#endif
#pragma message("gsl_CONFIG_CONTRACT_LEVEL_EXPECTS_ONLY is deprecated since gsl-lite 0.36; replace with gsl_CONFIG_CONTRACT_CHECKING_ENSURES_OFF and gsl_CONFIG_CONTRACT_CHECKING_ASSERT_OFF.")
#define gsl_CONFIG_CONTRACT_CHECKING_ON
#define gsl_CONFIG_CONTRACT_CHECKING_ENSURES_OFF
#define gsl_CONFIG_CONTRACT_CHECKING_ASSERT_OFF
#elif defined(gsl_CONFIG_CONTRACT_LEVEL_ENSURES_ONLY)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_LEVEL_ENSURES_ONLY)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_LEVEL_ENSURES_ONLY=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_LEVEL_ENSURES_ONLY) "; macro must be defined without value")
#endif
#pragma message("gsl_CONFIG_CONTRACT_LEVEL_ENSURES_ONLY is deprecated since gsl-lite 0.36; replace with gsl_CONFIG_CONTRACT_CHECKING_EXPECTS_OFF and gsl_CONFIG_CONTRACT_CHECKING_ASSERT_OFF.")
#define gsl_CONFIG_CONTRACT_CHECKING_ON
#define gsl_CONFIG_CONTRACT_CHECKING_EXPECTS_OFF
#define gsl_CONFIG_CONTRACT_CHECKING_ASSERT_OFF
#endif

// M-GSL compatibility:

#if defined(GSL_THROW_ON_CONTRACT_VIOLATION)
#if !gsl_CHECK_CFG_NO_VALUE_(GSL_THROW_ON_CONTRACT_VIOLATION)
#pragma message("invalid configuration value GSL_THROW_ON_CONTRACT_VIOLATION=" gsl_STRINGIFY(GSL_THROW_ON_CONTRACT_VIOLATION) "; macro must be defined without value")
#endif
#define gsl_CONFIG_CONTRACT_VIOLATION_THROWS
#endif

#if defined(GSL_TERMINATE_ON_CONTRACT_VIOLATION)
#if !gsl_CHECK_CFG_NO_VALUE_(GSL_TERMINATE_ON_CONTRACT_VIOLATION)
#pragma message("invalid configuration value GSL_TERMINATE_ON_CONTRACT_VIOLATION=" gsl_STRINGIFY(GSL_TERMINATE_ON_CONTRACT_VIOLATION) "; macro must be defined without value")
#endif
#define gsl_CONFIG_CONTRACT_VIOLATION_TERMINATES
#endif

#if defined(GSL_UNENFORCED_ON_CONTRACT_VIOLATION)
#if !gsl_CHECK_CFG_NO_VALUE_(GSL_UNENFORCED_ON_CONTRACT_VIOLATION)
#pragma message("invalid configuration value GSL_UNENFORCED_ON_CONTRACT_VIOLATION=" gsl_STRINGIFY(GSL_UNENFORCED_ON_CONTRACT_VIOLATION) "; macro must be defined without value")
#endif
#define gsl_CONFIG_CONTRACT_CHECKING_OFF
#endif

// Configuration: Features

#if defined(gsl_FEATURE_WITH_CONTAINER_TO_STD)
#if !gsl_CHECK_CFG_STD_VALUE_(gsl_FEATURE_WITH_CONTAINER_TO_STD)
#pragma message("invalid configuration value gsl_FEATURE_WITH_CONTAINER_TO_STD=" gsl_STRINGIFY(gsl_FEATURE_WITH_CONTAINER_TO_STD) ", must be 98, 3, 11, 14, 17, or 20")
#endif
#else
#define gsl_FEATURE_WITH_CONTAINER_TO_STD 99  // default
#endif
#define gsl_FEATURE_WITH_CONTAINER_TO_STD_() gsl_FEATURE_WITH_CONTAINER_TO_STD

#if defined(gsl_FEATURE_MAKE_SPAN_TO_STD)
#if !gsl_CHECK_CFG_STD_VALUE_(gsl_FEATURE_MAKE_SPAN_TO_STD)
#pragma message("invalid configuration value gsl_FEATURE_MAKE_SPAN_TO_STD=" gsl_STRINGIFY(gsl_FEATURE_MAKE_SPAN_TO_STD) ", must be 98, 3, 11, 14, 17, or 20")
#endif
#else
#define gsl_FEATURE_MAKE_SPAN_TO_STD 99  // default
#endif
#define gsl_FEATURE_MAKE_SPAN_TO_STD_() gsl_FEATURE_MAKE_SPAN_TO_STD

#if defined(gsl_FEATURE_BYTE_SPAN_TO_STD)
#if !gsl_CHECK_CFG_STD_VALUE_(gsl_FEATURE_BYTE_SPAN_TO_STD)
#pragma message("invalid configuration value gsl_FEATURE_BYTE_SPAN_TO_STD=" gsl_STRINGIFY(gsl_FEATURE_BYTE_SPAN_TO_STD) ", must be 98, 3, 11, 14, 17, or 20")
#endif
#else
#define gsl_FEATURE_BYTE_SPAN_TO_STD 99  // default
#endif
#define gsl_FEATURE_BYTE_SPAN_TO_STD_() gsl_FEATURE_BYTE_SPAN_TO_STD

#if defined(gsl_FEATURE_IMPLICIT_MACRO)
#if !gsl_CHECK_CFG_TOGGLE_VALUE_(gsl_FEATURE_IMPLICIT_MACRO)
#pragma message("invalid configuration value gsl_FEATURE_IMPLICIT_MACRO=" gsl_STRINGIFY(gsl_FEATURE_IMPLICIT_MACRO) ", must be 0 or 1")
#endif
#else
#define gsl_FEATURE_IMPLICIT_MACRO 0  // default
#endif
#define gsl_FEATURE_IMPLICIT_MACRO_() gsl_FEATURE_IMPLICIT_MACRO

#if defined(gsl_FEATURE_OWNER_MACRO)
#if !gsl_CHECK_CFG_TOGGLE_VALUE_(gsl_FEATURE_OWNER_MACRO)
#pragma message("invalid configuration value gsl_FEATURE_OWNER_MACRO=" gsl_STRINGIFY(gsl_FEATURE_OWNER_MACRO) ", must be 0 or 1")
#endif
#else
#define gsl_FEATURE_OWNER_MACRO (gsl_CONFIG_DEFAULTS_VERSION == 0)  // default
#endif
#define gsl_FEATURE_OWNER_MACRO_() gsl_FEATURE_OWNER_MACRO

#if defined(gsl_FEATURE_EXPERIMENTAL_RETURN_GUARD)
#if !gsl_CHECK_CFG_TOGGLE_VALUE_(gsl_FEATURE_EXPERIMENTAL_RETURN_GUARD)
#pragma message("invalid configuration value gsl_FEATURE_EXPERIMENTAL_RETURN_GUARD=" gsl_STRINGIFY(gsl_FEATURE_EXPERIMENTAL_RETURN_GUARD) ", must be 0 or 1")
#endif
#else
#define gsl_FEATURE_EXPERIMENTAL_RETURN_GUARD 0  // default
#endif
#define gsl_FEATURE_EXPERIMENTAL_RETURN_GUARD_() gsl_FEATURE_EXPERIMENTAL_RETURN_GUARD

#if defined(gsl_FEATURE_GSL_LITE_NAMESPACE)
#if !gsl_CHECK_CFG_TOGGLE_VALUE_(gsl_FEATURE_GSL_LITE_NAMESPACE)
#pragma message("invalid configuration value gsl_FEATURE_GSL_LITE_NAMESPACE=" gsl_STRINGIFY(gsl_FEATURE_GSL_LITE_NAMESPACE) ", must be 0 or 1")
#endif
#else
#define gsl_FEATURE_GSL_LITE_NAMESPACE (gsl_CONFIG_DEFAULTS_VERSION >= 1)  // default
#endif
#define gsl_FEATURE_GSL_LITE_NAMESPACE_() gsl_FEATURE_GSL_LITE_NAMESPACE

// Configuration: Other

#if defined(gsl_CONFIG_TRANSPARENT_NOT_NULL)
#if !gsl_CHECK_CFG_TOGGLE_VALUE_(gsl_CONFIG_TRANSPARENT_NOT_NULL)
#pragma message("invalid configuration value gsl_CONFIG_TRANSPARENT_NOT_NULL=" gsl_STRINGIFY(gsl_CONFIG_TRANSPARENT_NOT_NULL) ", must be 0 or 1")
#endif
#if gsl_CONFIG_TRANSPARENT_NOT_NULL && defined(gsl_CONFIG_NOT_NULL_GET_BY_CONST_REF)
#error configuration option gsl_CONFIG_NOT_NULL_GET_BY_CONST_REF is meaningless if gsl_CONFIG_TRANSPARENT_NOT_NULL=1
#endif
#else
#define gsl_CONFIG_TRANSPARENT_NOT_NULL (gsl_CONFIG_DEFAULTS_VERSION >= 1)  // default
#endif
#define gsl_CONFIG_TRANSPARENT_NOT_NULL_() gsl_CONFIG_TRANSPARENT_NOT_NULL

#if !defined(gsl_CONFIG_DEPRECATE_TO_LEVEL)
#if gsl_CONFIG_DEFAULTS_VERSION >= 1
#define gsl_CONFIG_DEPRECATE_TO_LEVEL 6
#else
#define gsl_CONFIG_DEPRECATE_TO_LEVEL 0
#endif
#endif

#if !defined(gsl_CONFIG_SPAN_INDEX_TYPE)
#define gsl_CONFIG_SPAN_INDEX_TYPE std::size_t
#endif
#define gsl_CONFIG_SPAN_INDEX_TYPE_() gsl_CONFIG_SPAN_INDEX_TYPE

#if !defined(gsl_CONFIG_INDEX_TYPE)
#if gsl_CONFIG_DEFAULTS_VERSION >= 1
// p0122r3 uses std::ptrdiff_t
#define gsl_CONFIG_INDEX_TYPE std::ptrdiff_t
#else
#define gsl_CONFIG_INDEX_TYPE gsl_CONFIG_SPAN_INDEX_TYPE
#endif
#endif
#define gsl_CONFIG_INDEX_TYPE_() gsl_CONFIG_INDEX_TYPE

#if defined(gsl_CONFIG_NOT_NULL_EXPLICIT_CTOR)
#if !gsl_CHECK_CFG_TOGGLE_VALUE_(gsl_CONFIG_NOT_NULL_EXPLICIT_CTOR)
#pragma message("invalid configuration value gsl_CONFIG_NOT_NULL_EXPLICIT_CTOR=" gsl_STRINGIFY(gsl_CONFIG_NOT_NULL_EXPLICIT_CTOR) ", must be 0 or 1")
#endif
#else
#define gsl_CONFIG_NOT_NULL_EXPLICIT_CTOR (gsl_CONFIG_DEFAULTS_VERSION >= 1)  // default
#endif
#define gsl_CONFIG_NOT_NULL_EXPLICIT_CTOR_() gsl_CONFIG_NOT_NULL_EXPLICIT_CTOR

#if defined(gsl_CONFIG_NOT_NULL_GET_BY_CONST_REF)
#if !gsl_CHECK_CFG_TOGGLE_VALUE_(gsl_CONFIG_NOT_NULL_GET_BY_CONST_REF)
#pragma message("invalid configuration value gsl_CONFIG_NOT_NULL_GET_BY_CONST_REF=" gsl_STRINGIFY(gsl_CONFIG_NOT_NULL_GET_BY_CONST_REF) ", must be 0 or 1")
#endif
#else
#define gsl_CONFIG_NOT_NULL_GET_BY_CONST_REF 0  // default
#endif
#define gsl_CONFIG_NOT_NULL_GET_BY_CONST_REF_() gsl_CONFIG_NOT_NULL_GET_BY_CONST_REF

#if defined(gsl_CONFIG_CONFIRMS_COMPILATION_ERRORS)
#if !gsl_CHECK_CFG_TOGGLE_VALUE_(gsl_CONFIG_CONFIRMS_COMPILATION_ERRORS)
#pragma message("invalid configuration value gsl_CONFIG_CONFIRMS_COMPILATION_ERRORS=" gsl_STRINGIFY(gsl_CONFIG_CONFIRMS_COMPILATION_ERRORS) ", must be 0 or 1")
#endif
#else
#define gsl_CONFIG_CONFIRMS_COMPILATION_ERRORS 0  // default
#endif
#define gsl_CONFIG_CONFIRMS_COMPILATION_ERRORS_() gsl_CONFIG_CONFIRMS_COMPILATION_ERRORS

#if defined(gsl_CONFIG_ALLOWS_SPAN_COMPARISON)
#if !gsl_CHECK_CFG_TOGGLE_VALUE_(gsl_CONFIG_ALLOWS_SPAN_COMPARISON)
#pragma message("invalid configuration value gsl_CONFIG_ALLOWS_SPAN_COMPARISON=" gsl_STRINGIFY(gsl_CONFIG_ALLOWS_SPAN_COMPARISON) ", must be 0 or 1")
#endif
#else
#define gsl_CONFIG_ALLOWS_SPAN_COMPARISON (gsl_CONFIG_DEFAULTS_VERSION == 0)  // default
#endif
#define gsl_CONFIG_ALLOWS_SPAN_COMPARISON_() gsl_CONFIG_ALLOWS_SPAN_COMPARISON

#if defined(gsl_CONFIG_ALLOWS_NONSTRICT_SPAN_COMPARISON)
#if !gsl_CHECK_CFG_TOGGLE_VALUE_(gsl_CONFIG_ALLOWS_NONSTRICT_SPAN_COMPARISON)
#pragma message("invalid configuration value gsl_CONFIG_ALLOWS_NONSTRICT_SPAN_COMPARISON=" gsl_STRINGIFY(gsl_CONFIG_ALLOWS_NONSTRICT_SPAN_COMPARISON) ", must be 0 or 1")
#endif
#else
#define gsl_CONFIG_ALLOWS_NONSTRICT_SPAN_COMPARISON 1  // default
#endif
#define gsl_CONFIG_ALLOWS_NONSTRICT_SPAN_COMPARISON_() gsl_CONFIG_ALLOWS_NONSTRICT_SPAN_COMPARISON

#if defined(gsl_CONFIG_ALLOWS_UNCONSTRAINED_SPAN_CONTAINER_CTOR)
#if !gsl_CHECK_CFG_TOGGLE_VALUE_(gsl_CONFIG_ALLOWS_UNCONSTRAINED_SPAN_CONTAINER_CTOR)
#pragma message("invalid configuration value gsl_CONFIG_ALLOWS_UNCONSTRAINED_SPAN_CONTAINER_CTOR=" gsl_STRINGIFY(gsl_CONFIG_ALLOWS_UNCONSTRAINED_SPAN_CONTAINER_CTOR) ", must be 0 or 1")
#endif
#else
#define gsl_CONFIG_ALLOWS_UNCONSTRAINED_SPAN_CONTAINER_CTOR 0  // default
#endif
#define gsl_CONFIG_ALLOWS_UNCONSTRAINED_SPAN_CONTAINER_CTOR_() gsl_CONFIG_ALLOWS_UNCONSTRAINED_SPAN_CONTAINER_CTOR

#if defined(gsl_CONFIG_NARROW_THROWS_ON_TRUNCATION)
#if !gsl_CHECK_CFG_TOGGLE_VALUE_(gsl_CONFIG_NARROW_THROWS_ON_TRUNCATION)
#pragma message("invalid configuration value gsl_CONFIG_NARROW_THROWS_ON_TRUNCATION=" gsl_STRINGIFY(gsl_CONFIG_NARROW_THROWS_ON_TRUNCATION) ", must be 0 or 1")
#endif
#else
#define gsl_CONFIG_NARROW_THROWS_ON_TRUNCATION (gsl_CONFIG_DEFAULTS_VERSION >= 1)  // default
#endif
#define gsl_CONFIG_NARROW_THROWS_ON_TRUNCATION_() gsl_CONFIG_NARROW_THROWS_ON_TRUNCATION

#if defined(gsl_CONFIG_CONTRACT_CHECKING_EXPECTS_OFF)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_CHECKING_EXPECTS_OFF)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_CHECKING_EXPECTS_OFF=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_CHECKING_EXPECTS_OFF) "; macro must be defined without value")
#endif
#endif
#if defined(gsl_CONFIG_CONTRACT_CHECKING_ENSURES_OFF)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_CHECKING_ENSURES_OFF)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_CHECKING_ENSURES_OFF=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_CHECKING_ENSURES_OFF) "; macro must be defined without value")
#endif
#endif
#if defined(gsl_CONFIG_CONTRACT_CHECKING_ASSERT_OFF)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_CHECKING_ASSERT_OFF)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_CHECKING_ASSERT_OFF=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_CHECKING_ASSERT_OFF) "; macro must be defined without value")
#endif
#endif
#if defined(gsl_CONFIG_CONTRACT_CHECKING_AUDIT)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_CHECKING_AUDIT)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_CHECKING_AUDIT=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_CHECKING_AUDIT) "; macro must be defined without value")
#endif
#endif
#if defined(gsl_CONFIG_CONTRACT_CHECKING_ON)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_CHECKING_ON)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_CHECKING_ON=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_CHECKING_ON) "; macro must be defined without value")
#endif
#endif
#if defined(gsl_CONFIG_CONTRACT_CHECKING_OFF)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_CHECKING_OFF)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_CHECKING_OFF=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_CHECKING_OFF) "; macro must be defined without value")
#endif
#endif
#if defined(gsl_CONFIG_CONTRACT_VIOLATION_THROWS)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_VIOLATION_THROWS)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_VIOLATION_THROWS=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_VIOLATION_THROWS) "; macro must be defined without value")
#endif
#endif
#if defined(gsl_CONFIG_CONTRACT_VIOLATION_TERMINATES)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_VIOLATION_TERMINATES)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_VIOLATION_TERMINATES=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_VIOLATION_TERMINATES) "; macro must be defined without value")
#endif
#endif
#if defined(gsl_CONFIG_CONTRACT_VIOLATION_ASSERTS)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_VIOLATION_ASSERTS)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_VIOLATION_ASSERTS=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_VIOLATION_ASSERTS) "; macro must be defined without value")
#endif
#endif
#if defined(gsl_CONFIG_CONTRACT_VIOLATION_TRAPS)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_VIOLATION_TRAPS)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_VIOLATION_TRAPS=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_VIOLATION_TRAPS) "; macro must be defined without value")
#endif
#endif
#if defined(gsl_CONFIG_CONTRACT_VIOLATION_CALLS_HANDLER)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_CONTRACT_VIOLATION_CALLS_HANDLER)
#pragma message("invalid configuration value gsl_CONFIG_CONTRACT_VIOLATION_CALLS_HANDLER=" gsl_STRINGIFY(gsl_CONFIG_CONTRACT_VIOLATION_CALLS_HANDLER) "; macro must be defined without value")
#endif
#endif
#if defined(gsl_CONFIG_UNENFORCED_CONTRACTS_ASSUME)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_UNENFORCED_CONTRACTS_ASSUME)
#pragma message("invalid configuration value gsl_CONFIG_UNENFORCED_CONTRACTS_ASSUME=" gsl_STRINGIFY(gsl_CONFIG_UNENFORCED_CONTRACTS_ASSUME) "; macro must be defined without value")
#endif
#endif
#if defined(gsl_CONFIG_UNENFORCED_CONTRACTS_ELIDE)
#if !gsl_CHECK_CFG_NO_VALUE_(gsl_CONFIG_UNENFORCED_CONTRACTS_ELIDE)
#pragma message("invalid configuration value gsl_CONFIG_UNENFORCED_CONTRACTS_ELIDE=" gsl_STRINGIFY(gsl_CONFIG_UNENFORCED_CONTRACTS_ELIDE) "; macro must be defined without value")
#endif
#endif

#if 1 < defined(gsl_CONFIG_CONTRACT_CHECKING_AUDIT) + defined(gsl_CONFIG_CONTRACT_CHECKING_ON) + defined(gsl_CONFIG_CONTRACT_CHECKING_OFF)
#error only one of gsl_CONFIG_CONTRACT_CHECKING_AUDIT, gsl_CONFIG_CONTRACT_CHECKING_ON, and gsl_CONFIG_CONTRACT_CHECKING_OFF may be defined
#endif
#if 1 < defined(gsl_CONFIG_CONTRACT_VIOLATION_THROWS) + defined(gsl_CONFIG_CONTRACT_VIOLATION_TERMINATES) + defined(gsl_CONFIG_CONTRACT_VIOLATION_ASSERTS) + defined(gsl_CONFIG_CONTRACT_VIOLATION_TRAPS) + defined(gsl_CONFIG_CONTRACT_VIOLATION_CALLS_HANDLER)
#error only one of gsl_CONFIG_CONTRACT_VIOLATION_THROWS, gsl_CONFIG_CONTRACT_VIOLATION_TERMINATES, gsl_CONFIG_CONTRACT_VIOLATION_ASSERTS, gsl_CONFIG_CONTRACT_VIOLATION_TRAPS, and gsl_CONFIG_CONTRACT_VIOLATION_CALLS_HANDLER may be defined
#endif
#if 1 < defined(gsl_CONFIG_UNENFORCED_CONTRACTS_ASSUME) + defined(gsl_CONFIG_UNENFORCED_CONTRACTS_ELIDE)
#error only one of gsl_CONFIG_UNENFORCED_CONTRACTS_ASSUME and gsl_CONFIG_UNENFORCED_CONTRACTS_ELIDE may be defined
#endif

#if 0 == defined(gsl_CONFIG_CONTRACT_CHECKING_AUDIT) + defined(gsl_CONFIG_CONTRACT_CHECKING_ON) + defined(gsl_CONFIG_CONTRACT_CHECKING_OFF)
// select default
#define gsl_CONFIG_CONTRACT_CHECKING_ON
#endif
#if 0 == defined(gsl_CONFIG_CONTRACT_VIOLATION_THROWS) + defined(gsl_CONFIG_CONTRACT_VIOLATION_TERMINATES) + defined(gsl_CONFIG_CONTRACT_VIOLATION_ASSERTS) + defined(gsl_CONFIG_CONTRACT_VIOLATION_TRAPS) + defined(gsl_CONFIG_CONTRACT_VIOLATION_CALLS_HANDLER)
// select default
#define gsl_CONFIG_CONTRACT_VIOLATION_TERMINATES
#endif
#if 0 == defined(gsl_CONFIG_UNENFORCED_CONTRACTS_ASSUME) + defined(gsl_CONFIG_UNENFORCED_CONTRACTS_ELIDE)
// select default
#define gsl_CONFIG_UNENFORCED_CONTRACTS_ELIDE
#endif

// C++ language version detection (C++20 is speculative):
// Note: VC14.0/1900 (VS2015) lacks too much from C++14.

#ifndef gsl_CPLUSPLUS
#if defined(_MSVC_LANG) && !defined(__clang__)
#define gsl_CPLUSPLUS (_MSC_VER == 1900 ? 201103L : _MSVC_LANG)
#else
#define gsl_CPLUSPLUS __cplusplus
#endif
#endif

// C++ standard library version:

#ifndef gsl_CPLUSPLUS_STDLIB
#define gsl_CPLUSPLUS_STDLIB gsl_CPLUSPLUS
#endif

#define gsl_CPP98_OR_GREATER (gsl_CPLUSPLUS >= 199711L)
#define gsl_CPP11_OR_GREATER (gsl_CPLUSPLUS >= 201103L)
#define gsl_CPP14_OR_GREATER (gsl_CPLUSPLUS >= 201402L)
#define gsl_CPP17_OR_GREATER (gsl_CPLUSPLUS >= 201703L)
#define gsl_CPP20_OR_GREATER (gsl_CPLUSPLUS >= 202000L)

// C++ language version (represent 98 as 3):

#define gsl_CPLUSPLUS_V (gsl_CPLUSPLUS / 100 - (gsl_CPLUSPLUS > 200000 ? 2000 : 1994))

// half-open range [lo..hi):
#define gsl_BETWEEN(v, lo, hi) ((lo) <= (v) && (v) < (hi))

// Compiler versions:

// MSVC++  6.0  _MSC_VER == 1200  gsl_COMPILER_MSVC_VERSION ==  60  (Visual Studio 6.0)
// MSVC++  7.0  _MSC_VER == 1300  gsl_COMPILER_MSVC_VERSION ==  70  (Visual Studio .NET 2002)
// MSVC++  7.1  _MSC_VER == 1310  gsl_COMPILER_MSVC_VERSION ==  71  (Visual Studio .NET 2003)
// MSVC++  8.0  _MSC_VER == 1400  gsl_COMPILER_MSVC_VERSION ==  80  (Visual Studio 2005)
// MSVC++  9.0  _MSC_VER == 1500  gsl_COMPILER_MSVC_VERSION ==  90  (Visual Studio 2008)
// MSVC++ 10.0  _MSC_VER == 1600  gsl_COMPILER_MSVC_VERSION == 100  (Visual Studio 2010)
// MSVC++ 11.0  _MSC_VER == 1700  gsl_COMPILER_MSVC_VERSION == 110  (Visual Studio 2012)
// MSVC++ 12.0  _MSC_VER == 1800  gsl_COMPILER_MSVC_VERSION == 120  (Visual Studio 2013)
// MSVC++ 14.0  _MSC_VER == 1900  gsl_COMPILER_MSVC_VERSION == 140  (Visual Studio 2015)
// MSVC++ 14.1  _MSC_VER >= 1910  gsl_COMPILER_MSVC_VERSION == 141  (Visual Studio 2017)
// MSVC++ 14.2  _MSC_VER >= 1920  gsl_COMPILER_MSVC_VERSION == 142  (Visual Studio 2019)

#if defined(_MSC_VER) && !defined(__clang__)
#define gsl_COMPILER_MSVC_VER (_MSC_VER)
#define gsl_COMPILER_MSVC_VERSION (_MSC_VER / 10 - 10 * (5 + (_MSC_VER < 1900)))
#define gsl_COMPILER_MSVC_VERSION_FULL (_MSC_VER - 100 * (5 + (_MSC_VER < 1900)))
#else
#define gsl_COMPILER_MSVC_VER 0
#define gsl_COMPILER_MSVC_VERSION 0
#define gsl_COMPILER_MSVC_VERSION_FULL 0
#endif

#define gsl_COMPILER_VERSION(major, minor, patch) (10 * (10 * (major) + (minor)) + (patch))

// AppleClang  7.0.0  __apple_build_version__ ==  7000172  gsl_COMPILER_APPLECLANG_VERSION ==  700  (Xcode 7.0, 7.0.1)               (LLVM  3.7.0)
// AppleClang  7.0.0  __apple_build_version__ ==  7000176  gsl_COMPILER_APPLECLANG_VERSION ==  700  (Xcode 7.1)                      (LLVM  3.7.0)
// AppleClang  7.0.2  __apple_build_version__ ==  7000181  gsl_COMPILER_APPLECLANG_VERSION ==  702  (Xcode 7.2, 7.2.1)               (LLVM  3.7.0)
// AppleClang  7.3.0  __apple_build_version__ ==  7030029  gsl_COMPILER_APPLECLANG_VERSION ==  730  (Xcode 7.3)                      (LLVM  3.8.0)
// AppleClang  7.3.0  __apple_build_version__ ==  7030031  gsl_COMPILER_APPLECLANG_VERSION ==  730  (Xcode 7.3.1)                    (LLVM  3.8.0)
// AppleClang  8.0.0  __apple_build_version__ ==  8000038  gsl_COMPILER_APPLECLANG_VERSION ==  800  (Xcode 8.0)                      (LLVM  3.9.0)
// AppleClang  8.0.0  __apple_build_version__ ==  8000042  gsl_COMPILER_APPLECLANG_VERSION ==  800  (Xcode 8.1, 8.2, 8.2.1)          (LLVM  3.9.0)
// AppleClang  8.1.0  __apple_build_version__ ==  8020038  gsl_COMPILER_APPLECLANG_VERSION ==  810  (Xcode 8.3)                      (LLVM  3.9.0)
// AppleClang  8.1.0  __apple_build_version__ ==  8020041  gsl_COMPILER_APPLECLANG_VERSION ==  810  (Xcode 8.3.1)                    (LLVM  3.9.0)
// AppleClang  8.1.0  __apple_build_version__ ==  8020042  gsl_COMPILER_APPLECLANG_VERSION ==  810  (Xcode 8.3.2, 8.3.3)             (LLVM  3.9.0)
// AppleClang  9.0.0  __apple_build_version__ ==  9000037  gsl_COMPILER_APPLECLANG_VERSION ==  900  (Xcode 9.0)                      (LLVM  4.0.0)
// AppleClang  9.0.0  __apple_build_version__ ==  9000038  gsl_COMPILER_APPLECLANG_VERSION ==  900  (Xcode 9.1)                      (LLVM  4.0.0)
// AppleClang  9.0.0  __apple_build_version__ ==  9000039  gsl_COMPILER_APPLECLANG_VERSION ==  900  (Xcode 9.2)                      (LLVM  4.0.0)
// AppleClang  9.1.0  __apple_build_version__ ==  9020039  gsl_COMPILER_APPLECLANG_VERSION ==  910  (Xcode 9.3, 9.3.1)               (LLVM  5.0.2)
// AppleClang  9.1.0  __apple_build_version__ ==  9020039  gsl_COMPILER_APPLECLANG_VERSION ==  910  (Xcode 9.4, 9.4.1)               (LLVM  5.0.2)
// AppleClang 10.0.0  __apple_build_version__ == 10001145  gsl_COMPILER_APPLECLANG_VERSION == 1000  (Xcode 10.0, 10.1)               (LLVM  6.0.1)
// AppleClang 10.0.1  __apple_build_version__ == 10010046  gsl_COMPILER_APPLECLANG_VERSION == 1001  (Xcode 10.2, 10.2.1, 10.3)       (LLVM  7.0.0)
// AppleClang 11.0.0  __apple_build_version__ == 11000033  gsl_COMPILER_APPLECLANG_VERSION == 1100  (Xcode 11.1, 11.2, 11.3, 11.3.1) (LLVM  8.0.0)
// AppleClang 11.0.3  __apple_build_version__ == 11030032  gsl_COMPILER_APPLECLANG_VERSION == 1103  (Xcode 11.4, 11.4.1, 11.5, 11.6) (LLVM  9.0.0)
// AppleClang 12.0.0  __apple_build_version__ == 12000032  gsl_COMPILER_APPLECLANG_VERSION == 1200  (Xcode 12.0–12.4)                (LLVM 10.0.0)
// AppleClang 12.0.5  __apple_build_version__ == 12050022  gsl_COMPILER_APPLECLANG_VERSION == 1205  (Xcode 12.5)                     (LLVM 10.0.0)

#if defined(__apple_build_version__)
#define gsl_COMPILER_APPLECLANG_VERSION gsl_COMPILER_VERSION(__clang_major__, __clang_minor__, __clang_patchlevel__)
#define gsl_COMPILER_CLANG_VERSION 0
#elif defined(__clang__)
#define gsl_COMPILER_APPLECLANG_VERSION 0
#define gsl_COMPILER_CLANG_VERSION gsl_COMPILER_VERSION(__clang_major__, __clang_minor__, __clang_patchlevel__)
#else
#define gsl_COMPILER_APPLECLANG_VERSION 0
#define gsl_COMPILER_CLANG_VERSION 0
#endif

#if defined(__GNUC__) && !defined(__clang__)
#define gsl_COMPILER_GNUC_VERSION gsl_COMPILER_VERSION(__GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__)
#else
#define gsl_COMPILER_GNUC_VERSION 0
#endif

#if defined(__NVCC__)
#define gsl_COMPILER_NVCC_VERSION (__CUDACC_VER_MAJOR__ * 10 + __CUDACC_VER_MINOR__)
#else
#define gsl_COMPILER_NVCC_VERSION 0
#endif

#if defined(__ARMCC_VERSION)
#define gsl_COMPILER_ARMCC_VERSION (__ARMCC_VERSION / 10000)
#define gsl_COMPILER_ARMCC_VERSION_FULL __ARMCC_VERSION
#else
#define gsl_COMPILER_ARMCC_VERSION 0
#define gsl_COMPILER_ARMCC_VERSION_FULL 0
#endif

// Compiler non-strict aliasing:

#if defined(__clang__) || defined(__GNUC__)
#define gsl_may_alias __attribute__((__may_alias__))
#else
#define gsl_may_alias
#endif

// Presence of gsl, language and library features:

#define gsl_IN_STD(v) (((v) == 98 ? 3 : (v)) >= gsl_CPLUSPLUS_V)

#define gsl_DEPRECATE_TO_LEVEL(level) (level <= gsl_CONFIG_DEPRECATE_TO_LEVEL)
#define gsl_FEATURE_TO_STD(feature) gsl_IN_STD(gsl_FEATURE(feature##_TO_STD))
#define gsl_FEATURE(feature) gsl_EVALF_(gsl_FEATURE_##feature##_)
#define gsl_CONFIG(feature) gsl_EVALF_(gsl_CONFIG_##feature##_)
#define gsl_HAVE(feature) gsl_EVALF_(gsl_HAVE_##feature##_)

// Presence of wide character support:

#ifdef __DJGPP__
#define gsl_HAVE_WCHAR 0
#else
#define gsl_HAVE_WCHAR 1
#endif
#define gsl_HAVE_WCHAR_() gsl_HAVE_WCHAR

// Presence of language & library features:

#if gsl_COMPILER_CLANG_VERSION || gsl_COMPILER_APPLECLANG_VERSION
#ifdef __OBJC__
// There are a bunch of inconsistencies about __EXCEPTIONS and __has_feature(cxx_exceptions) in Clang 3.4/3.5/3.6.
// We're interested in C++ exceptions, which can be checked by __has_feature(cxx_exceptions) in 3.5+.
// In pre-3.5, __has_feature(cxx_exceptions) can be true if ObjC exceptions are enabled, but C++ exceptions are disabled.
// The recommended way to check is `__EXCEPTIONS && __has_feature(cxx_exceptions)`.
// See https://releases.llvm.org/3.6.0/tools/clang/docs/ReleaseNotes.html#the-exceptions-macro
// Note: this is only relevant in Objective-C++, thus the ifdef.
#if __EXCEPTIONS && __has_feature(cxx_exceptions)
#define gsl_HAVE_EXCEPTIONS 1
#else
#define gsl_HAVE_EXCEPTIONS 0
#endif  // __EXCEPTIONS && __has_feature(cxx_exceptions)
#else
// clang-cl doesn't define __EXCEPTIONS for MSVC compatibility (see https://reviews.llvm.org/D4065).
// Neither does Clang in MS-compatiblity mode.
// Let's hope no one tries to build Objective-C++ code using MS-compatibility mode or clang-cl.
#if __has_feature(cxx_exceptions)
#define gsl_HAVE_EXCEPTIONS 1
#else
#define gsl_HAVE_EXCEPTIONS 0
#endif
#endif
#elif gsl_COMPILER_GNUC_VERSION
#if gsl_BETWEEN(gsl_COMPILER_GNUC_VERSION, 1, 500)
#ifdef __EXCEPTIONS
#define gsl_HAVE_EXCEPTIONS 1
#else
#define gsl_HAVE_EXCEPTIONS 0
#endif  // __EXCEPTIONS
#else
#ifdef __cpp_exceptions
#define gsl_HAVE_EXCEPTIONS 1
#else
#define gsl_HAVE_EXCEPTIONS 0
#endif  // __cpp_exceptions
#endif  // gsl_BETWEEN(gsl_COMPILER_GNUC_VERSION, 1, 500)
#elif gsl_COMPILER_MSVC_VERSION
#ifdef _CPPUNWIND
#define gsl_HAVE_EXCEPTIONS 1
#else
#define gsl_HAVE_EXCEPTIONS 0
#endif  // _CPPUNWIND
#else
// For all other compilers, assume exceptions are always enabled.
#define gsl_HAVE_EXCEPTIONS 1
#endif
#define gsl_HAVE_EXCEPTIONS_() gsl_HAVE_EXCEPTIONS

#if defined(gsl_CONFIG_CONTRACT_VIOLATION_THROWS) && !gsl_HAVE_EXCEPTIONS
#error Cannot use gsl_CONFIG_CONTRACT_VIOLATION_THROWS if exceptions are disabled.
#endif  // defined( gsl_CONFIG_CONTRACT_VIOLATION_THROWS ) && !gsl_HAVE( EXCEPTIONS )

#ifdef _HAS_CPP0X
#define gsl_HAS_CPP0X _HAS_CPP0X
#else
#define gsl_HAS_CPP0X 0
#endif

#define gsl_CPP11_100 (gsl_CPP11_OR_GREATER || gsl_COMPILER_MSVC_VER >= 1600)
#define gsl_CPP11_110 (gsl_CPP11_OR_GREATER || gsl_COMPILER_MSVC_VER >= 1700)
#define gsl_CPP11_120 (gsl_CPP11_OR_GREATER || gsl_COMPILER_MSVC_VER >= 1800)
#define gsl_CPP11_140 (gsl_CPP11_OR_GREATER || gsl_COMPILER_MSVC_VER >= 1900)

#define gsl_CPP14_000 (gsl_CPP14_OR_GREATER)
#define gsl_CPP14_120 (gsl_CPP14_OR_GREATER || gsl_COMPILER_MSVC_VER >= 1800)
#define gsl_CPP14_140 (gsl_CPP14_OR_GREATER || gsl_COMPILER_MSVC_VER >= 1900)

#define gsl_CPP17_000 (gsl_CPP17_OR_GREATER)
#define gsl_CPP17_140 (gsl_CPP17_OR_GREATER || gsl_COMPILER_MSVC_VER >= 1900)

#define gsl_CPP11_140_CPP0X_90 (gsl_CPP11_140 || (gsl_COMPILER_MSVC_VER >= 1500 && gsl_HAS_CPP0X))
#define gsl_CPP11_140_CPP0X_100 (gsl_CPP11_140 || (gsl_COMPILER_MSVC_VER >= 1600 && gsl_HAS_CPP0X))

// Presence of C++11 language features:

#define gsl_HAVE_C99_PREPROCESSOR gsl_CPP11_140
#define gsl_HAVE_AUTO gsl_CPP11_100
#define gsl_HAVE_RVALUE_REFERENCE gsl_CPP11_100
#define gsl_HAVE_FUNCTION_REF_QUALIFIER (gsl_CPP11_140 && !gsl_BETWEEN(gsl_COMPILER_GNUC_VERSION, 1, 481))
#define gsl_HAVE_ENUM_CLASS gsl_CPP11_110
#define gsl_HAVE_ALIAS_TEMPLATE gsl_CPP11_120
#define gsl_HAVE_DEFAULT_FUNCTION_TEMPLATE_ARG gsl_CPP11_120
#define gsl_HAVE_EXPLICIT gsl_CPP11_120
#define gsl_HAVE_VARIADIC_TEMPLATE gsl_CPP11_120
#define gsl_HAVE_IS_DELETE gsl_CPP11_120
#define gsl_HAVE_CONSTEXPR_11 gsl_CPP11_140
#define gsl_HAVE_IS_DEFAULT gsl_CPP11_140
#define gsl_HAVE_NOEXCEPT gsl_CPP11_140
#define gsl_HAVE_NORETURN (gsl_CPP11_140 && !gsl_BETWEEN(gsl_COMPILER_GNUC_VERSION, 1, 480))
#define gsl_HAVE_EXPRESSION_SFINAE gsl_CPP11_140
#define gsl_HAVE_OVERRIDE_FINAL gsl_CPP11_110

#define gsl_HAVE_C99_PREPROCESSOR_() gsl_HAVE_C99_PREPROCESSOR
#define gsl_HAVE_AUTO_() gsl_HAVE_AUTO
#define gsl_HAVE_RVALUE_REFERENCE_() gsl_HAVE_RVALUE_REFERENCE
#define gsl_HAVE_FUNCTION_REF_QUALIFIER_() gsl_HAVE_FUNCTION_REF_QUALIFIER
#define gsl_HAVE_ENUM_CLASS_() gsl_HAVE_ENUM_CLASS
#define gsl_HAVE_ALIAS_TEMPLATE_() gsl_HAVE_ALIAS_TEMPLATE
#define gsl_HAVE_DEFAULT_FUNCTION_TEMPLATE_ARG_() gsl_HAVE_DEFAULT_FUNCTION_TEMPLATE_ARG
#define gsl_HAVE_EXPLICIT_() gsl_HAVE_EXPLICIT
#define gsl_HAVE_VARIADIC_TEMPLATE_() gsl_HAVE_VARIADIC_TEMPLATE
#define gsl_HAVE_IS_DELETE_() gsl_HAVE_IS_DELETE
#define gsl_HAVE_CONSTEXPR_11_() gsl_HAVE_CONSTEXPR_11
#define gsl_HAVE_IS_DEFAULT_() gsl_HAVE_IS_DEFAULT
#define gsl_HAVE_NOEXCEPT_() gsl_HAVE_NOEXCEPT
#define gsl_HAVE_NORETURN_() gsl_HAVE_NORETURN
#define gsl_HAVE_EXPRESSION_SFINAE_() gsl_HAVE_EXPRESSION_SFINAE
#define gsl_HAVE_OVERRIDE_FINAL_() gsl_HAVE_OVERRIDE_FINAL

// Presence of C++14 language features:

#define gsl_HAVE_CONSTEXPR_14 (gsl_CPP14_000 && !gsl_BETWEEN(gsl_COMPILER_GNUC_VERSION, 1, 600))
#define gsl_HAVE_DECLTYPE_AUTO gsl_CPP14_140
#define gsl_HAVE_DEPRECATED (gsl_CPP14_140 && !gsl_BETWEEN(gsl_COMPILER_MSVC_VERSION, 1, 142))

#define gsl_HAVE_CONSTEXPR_14_() gsl_HAVE_CONSTEXPR_14
#define gsl_HAVE_DECLTYPE_AUTO_() gsl_HAVE_DECLTYPE_AUTO
#define gsl_HAVE_DEPRECATED_() gsl_HAVE_DEPRECATED

// Presence of C++17 language features:
// MSVC: template parameter deduction guides since Visual Studio 2017 v15.7

#define gsl_HAVE_ENUM_CLASS_CONSTRUCTION_FROM_UNDERLYING_TYPE gsl_CPP17_000
#define gsl_HAVE_DEDUCTION_GUIDES (gsl_CPP17_000 && !gsl_BETWEEN(gsl_COMPILER_MSVC_VERSION_FULL, 1, 1414))
#define gsl_HAVE_NODISCARD gsl_CPP17_000
#define gsl_HAVE_CONSTEXPR_17 gsl_CPP17_OR_GREATER

#define gsl_HAVE_ENUM_CLASS_CONSTRUCTION_FROM_UNDERLYING_TYPE_() gsl_HAVE_ENUM_CLASS_CONSTRUCTION_FROM_UNDERLYING_TYPE
#define gsl_HAVE_DEDUCTION_GUIDES_() gsl_HAVE_DEDUCTION_GUIDES
#define gsl_HAVE_NODISCARD_() gsl_HAVE_NODISCARD
#define gsl_HAVE_CONSTEXPR_17_() gsl_HAVE_CONSTEXPR_17

// Presence of C++20 language features:

#define gsl_HAVE_CONSTEXPR_20 gsl_CPP20_OR_GREATER

#define gsl_HAVE_CONSTEXPR_20_() gsl_HAVE_CONSTEXPR_20

// Presence of C++ library features:

#if gsl_BETWEEN(gsl_COMPILER_ARMCC_VERSION, 1, 600)
// Some versions of the ARM compiler apparently ship without a C++11 standard library despite having some C++11 support.
#define gsl_STDLIB_CPP98_OR_GREATER gsl_CPP98_OR_GREATER
#define gsl_STDLIB_CPP11_OR_GREATER 0
#define gsl_STDLIB_CPP14_OR_GREATER 0
#define gsl_STDLIB_CPP17_OR_GREATER 0
#define gsl_STDLIB_CPP20_OR_GREATER 0
#else
#define gsl_STDLIB_CPP98_OR_GREATER gsl_CPP98_OR_GREATER
#define gsl_STDLIB_CPP11_OR_GREATER gsl_CPP11_OR_GREATER
#define gsl_STDLIB_CPP14_OR_GREATER gsl_CPP14_OR_GREATER
#define gsl_STDLIB_CPP17_OR_GREATER gsl_CPP17_OR_GREATER
#define gsl_STDLIB_CPP20_OR_GREATER gsl_CPP20_OR_GREATER
#endif

#define gsl_STDLIB_CPP11_100 (gsl_STDLIB_CPP11_OR_GREATER || gsl_COMPILER_MSVC_VER >= 1600)
#define gsl_STDLIB_CPP11_110 (gsl_STDLIB_CPP11_OR_GREATER || gsl_COMPILER_MSVC_VER >= 1700)
#define gsl_STDLIB_CPP11_120 (gsl_STDLIB_CPP11_OR_GREATER || gsl_COMPILER_MSVC_VER >= 1800)
#define gsl_STDLIB_CPP11_140 (gsl_STDLIB_CPP11_OR_GREATER || gsl_COMPILER_MSVC_VER >= 1900)

#define gsl_STDLIB_CPP14_000 (gsl_STDLIB_CPP14_OR_GREATER)
#define gsl_STDLIB_CPP14_120 (gsl_STDLIB_CPP14_OR_GREATER || gsl_COMPILER_MSVC_VER >= 1800)
#define gsl_STDLIB_CPP14_140 (gsl_STDLIB_CPP14_OR_GREATER || gsl_COMPILER_MSVC_VER >= 1900)

#define gsl_STDLIB_CPP17_000 (gsl_STDLIB_CPP17_OR_GREATER)
#define gsl_STDLIB_CPP17_140 (gsl_STDLIB_CPP17_OR_GREATER || gsl_COMPILER_MSVC_VER >= 1900)

#define gsl_STDLIB_CPP11_140_CPP0X_90 (gsl_STDLIB_CPP11_140 || (gsl_COMPILER_MSVC_VER >= 1500 && gsl_HAS_CPP0X))
#define gsl_STDLIB_CPP11_140_CPP0X_100 (gsl_STDLIB_CPP11_140 || (gsl_COMPILER_MSVC_VER >= 1600 && gsl_HAS_CPP0X))

#define gsl_HAVE_ADDRESSOF gsl_STDLIB_CPP17_000
#define gsl_HAVE_ARRAY gsl_STDLIB_CPP11_110
#define gsl_HAVE_TYPE_TRAITS gsl_STDLIB_CPP11_110
#define gsl_HAVE_TR1_TYPE_TRAITS gsl_STDLIB_CPP11_110
#define gsl_HAVE_CONTAINER_DATA_METHOD gsl_STDLIB_CPP11_140_CPP0X_90
#define gsl_HAVE_STD_DATA gsl_STDLIB_CPP17_000
#ifdef __cpp_lib_ssize
#define gsl_HAVE_STD_SSIZE 1
#else
#define gsl_HAVE_STD_SSIZE (gsl_COMPILER_GNUC_VERSION >= 1000 && __cplusplus > 201703L)
#endif
#define gsl_HAVE_HASH gsl_STDLIB_CPP11_120
#define gsl_HAVE_SIZED_TYPES gsl_STDLIB_CPP11_140
#define gsl_HAVE_MAKE_SHARED gsl_STDLIB_CPP11_140_CPP0X_100
#define gsl_HAVE_SHARED_PTR gsl_STDLIB_CPP11_140_CPP0X_100
#define gsl_HAVE_UNIQUE_PTR gsl_STDLIB_CPP11_140_CPP0X_100
#define gsl_HAVE_MAKE_UNIQUE gsl_STDLIB_CPP14_120
#define gsl_HAVE_MOVE_FORWARD gsl_STDLIB_CPP11_100
#define gsl_HAVE_NULLPTR gsl_STDLIB_CPP11_100
#define gsl_HAVE_UNCAUGHT_EXCEPTIONS gsl_STDLIB_CPP17_140
#define gsl_HAVE_ADD_CONST gsl_HAVE_TYPE_TRAITS
#define gsl_HAVE_INITIALIZER_LIST gsl_STDLIB_CPP11_120
#define gsl_HAVE_INTEGRAL_CONSTANT gsl_HAVE_TYPE_TRAITS
#define gsl_HAVE_REMOVE_CONST gsl_HAVE_TYPE_TRAITS
#define gsl_HAVE_REMOVE_REFERENCE gsl_HAVE_TYPE_TRAITS
#define gsl_HAVE_REMOVE_CVREF gsl_STDLIB_CPP20_OR_GREATER
#define gsl_HAVE_TR1_ADD_CONST gsl_HAVE_TR1_TYPE_TRAITS
#define gsl_HAVE_TR1_INTEGRAL_CONSTANT gsl_HAVE_TR1_TYPE_TRAITS
#define gsl_HAVE_TR1_REMOVE_CONST gsl_HAVE_TR1_TYPE_TRAITS
#define gsl_HAVE_TR1_REMOVE_REFERENCE gsl_HAVE_TR1_TYPE_TRAITS

#define gsl_HAVE_ADDRESSOF_() gsl_HAVE_ADDRESSOF
#define gsl_HAVE_ARRAY_() gsl_HAVE_ARRAY
#define gsl_HAVE_TYPE_TRAITS_() gsl_HAVE_TYPE_TRAITS
#define gsl_HAVE_TR1_TYPE_TRAITS_() gsl_HAVE_TR1_TYPE_TRAITS
#define gsl_HAVE_CONTAINER_DATA_METHOD_() gsl_HAVE_CONTAINER_DATA_METHOD
#define gsl_HAVE_HASH_() gsl_HAVE_HASH
#define gsl_HAVE_STD_DATA_() gsl_HAVE_STD_DATA
#define gsl_HAVE_STD_SSIZE_() gsl_HAVE_STD_SSIZE
#define gsl_HAVE_SIZED_TYPES_() gsl_HAVE_SIZED_TYPES
#define gsl_HAVE_MAKE_SHARED_() gsl_HAVE_MAKE_SHARED
#define gsl_HAVE_MOVE_FORWARD_() gsl_HAVE_MOVE_FORWARD
#define gsl_HAVE_NULLPTR_() gsl_HAVE_NULLPTR  // It's a language feature but needs library support, so we list it as a library feature.
#define gsl_HAVE_SHARED_PTR_() gsl_HAVE_SHARED_PTR
#define gsl_HAVE_UNIQUE_PTR_() gsl_HAVE_UNIQUE_PTR
#define gsl_HAVE_MAKE_UNIQUE_() gsl_HAVE_MAKE_UNIQUE
#define gsl_HAVE_UNCAUGHT_EXCEPTIONS_() gsl_HAVE_UNCAUGHT_EXCEPTIONS
#define gsl_HAVE_ADD_CONST_() gsl_HAVE_ADD_CONST
#define gsl_HAVE_INITIALIZER_LIST_() gsl_HAVE_INITIALIZER_LIST  // It's a language feature but needs library support, so we list it as a library feature.
#define gsl_HAVE_INTEGRAL_CONSTANT_() gsl_HAVE_INTEGRAL_CONSTANT
#define gsl_HAVE_REMOVE_CONST_() gsl_HAVE_REMOVE_CONST
#define gsl_HAVE_REMOVE_REFERENCE_() gsl_HAVE_REMOVE_REFERENCE
#define gsl_HAVE_REMOVE_CVREF_() gsl_HAVE_REMOVE_CVREF
#define gsl_HAVE_TR1_ADD_CONST_() gsl_HAVE_TR1_ADD_CONST
#define gsl_HAVE_TR1_INTEGRAL_CONSTANT_() gsl_HAVE_TR1_INTEGRAL_CONSTANT
#define gsl_HAVE_TR1_REMOVE_CONST_() gsl_HAVE_TR1_REMOVE_CONST
#define gsl_HAVE_TR1_REMOVE_REFERENCE_() gsl_HAVE_TR1_REMOVE_REFERENCE

// C++ feature usage:

#if gsl_HAVE(ADDRESSOF)
#define gsl_ADDRESSOF(x) std::addressof(x)
#else
#define gsl_ADDRESSOF(x) (&x)
#endif

#if gsl_HAVE(CONSTEXPR_11)
#define gsl_constexpr constexpr
#else
#define gsl_constexpr /*constexpr*/
#endif

#if gsl_HAVE(CONSTEXPR_14)
#define gsl_constexpr14 constexpr
#else
#define gsl_constexpr14 /*constexpr*/
#endif

#if gsl_HAVE(CONSTEXPR_17)
#define gsl_constexpr17 constexpr
#else
#define gsl_constexpr17 /*constexpr*/
#endif

#if gsl_HAVE(CONSTEXPR_20)
#define gsl_constexpr20 constexpr
#else
#define gsl_constexpr20 /*constexpr*/
#endif

#if gsl_HAVE(EXPLICIT)
#define gsl_explicit explicit
#else
#define gsl_explicit /*explicit*/
#endif

#if gsl_FEATURE(IMPLICIT_MACRO)
#define implicit /*implicit*/
#endif

#if gsl_HAVE(IS_DELETE)
#define gsl_is_delete = delete
#else
#define gsl_is_delete
#endif

#if gsl_HAVE(IS_DELETE)
#define gsl_is_delete_access public
#else
#define gsl_is_delete_access private
#endif

#if gsl_HAVE(NOEXCEPT)
#define gsl_noexcept noexcept
#define gsl_noexcept_if(expr) noexcept(expr)
#else
#define gsl_noexcept throw()
#define gsl_noexcept_if(expr) /*noexcept( expr )*/
#endif
#if defined(gsl_TESTING_)
#define gsl_noexcept_not_testing
#else
#define gsl_noexcept_not_testing gsl_noexcept
#endif

#if gsl_HAVE(NULLPTR)
#define gsl_nullptr nullptr
#else
#define gsl_nullptr NULL
#endif

#if gsl_HAVE(NODISCARD)
#define gsl_NODISCARD [[nodiscard]]
#else
#define gsl_NODISCARD
#endif

#if gsl_HAVE(NORETURN)
#define gsl_NORETURN [[noreturn]]
#elif defined(_MSC_VER)
#define gsl_NORETURN __declspec(noreturn)
#elif gsl_COMPILER_GNUC_VERSION || gsl_COMPILER_CLANG_VERSION || gsl_COMPILER_APPLECLANG_VERSION || gsl_COMPILER_ARMCC_VERSION
#define gsl_NORETURN __attribute__((noreturn))
#else
#define gsl_NORETURN
#endif

#if gsl_HAVE(DEPRECATED) && !defined(gsl_TESTING_)
#define gsl_DEPRECATED [[deprecated]]
#define gsl_DEPRECATED_MSG(msg) [[deprecated(msg)]]
#else
#define gsl_DEPRECATED
#define gsl_DEPRECATED_MSG(msg)
#endif

#if gsl_HAVE(C99_PREPROCESSOR)
#if gsl_CPP20_OR_GREATER
#define gsl_CONSTRAINT(...) __VA_ARGS__
#else
#define gsl_CONSTRAINT(...) typename
#endif
#endif

#if gsl_HAVE(TYPE_TRAITS)
#define gsl_STATIC_ASSERT_(cond, msg) static_assert(cond, msg)
#else
#define gsl_STATIC_ASSERT_(cond, msg) ((void)sizeof(char[1 - 2 * !!(cond)]))
#endif

#if gsl_HAVE(TYPE_TRAITS)

#define gsl_DEFINE_ENUM_BITMASK_OPERATORS_(ENUM)                      \
    gsl_NODISCARD gsl_api inline gsl_constexpr ENUM                   \
    operator~(ENUM val) gsl_noexcept                                  \
    {                                                                 \
        typedef typename ::gsl::std11::underlying_type<ENUM>::type U; \
        return ENUM(~U(val));                                         \
    }                                                                 \
    gsl_NODISCARD gsl_api inline gsl_constexpr ENUM                   \
    operator|(ENUM lhs, ENUM rhs) gsl_noexcept                        \
    {                                                                 \
        typedef typename ::gsl::std11::underlying_type<ENUM>::type U; \
        return ENUM(U(lhs) | U(rhs));                                 \
    }                                                                 \
    gsl_NODISCARD gsl_api inline gsl_constexpr ENUM                   \
    operator&(ENUM lhs, ENUM rhs) gsl_noexcept                        \
    {                                                                 \
        typedef typename ::gsl::std11::underlying_type<ENUM>::type U; \
        return ENUM(U(lhs) & U(rhs));                                 \
    }                                                                 \
    gsl_NODISCARD gsl_api inline gsl_constexpr ENUM                   \
    operator^(ENUM lhs, ENUM rhs) gsl_noexcept                        \
    {                                                                 \
        typedef typename ::gsl::std11::underlying_type<ENUM>::type U; \
        return ENUM(U(lhs) ^ U(rhs));                                 \
    }                                                                 \
    gsl_api inline gsl_constexpr14 ENUM &                             \
    operator|=(ENUM &lhs, ENUM rhs) gsl_noexcept                      \
    {                                                                 \
        return lhs = lhs | rhs;                                       \
    }                                                                 \
    gsl_api inline gsl_constexpr14 ENUM &                             \
    operator&=(ENUM &lhs, ENUM rhs) gsl_noexcept                      \
    {                                                                 \
        return lhs = lhs & rhs;                                       \
    }                                                                 \
    gsl_api inline gsl_constexpr14 ENUM &                             \
    operator^=(ENUM &lhs, ENUM rhs) gsl_noexcept                      \
    {                                                                 \
        return lhs = lhs ^ rhs;                                       \
    }

#define gsl_DEFINE_ENUM_RELATIONAL_OPERATORS_(ENUM)                   \
    gsl_NODISCARD gsl_api inline gsl_constexpr bool                   \
    operator<(ENUM lhs, ENUM rhs) gsl_noexcept                        \
    {                                                                 \
        typedef typename ::gsl::std11::underlying_type<ENUM>::type U; \
        return U(lhs) < U(rhs);                                       \
    }                                                                 \
    gsl_NODISCARD gsl_api inline gsl_constexpr bool                   \
    operator>(ENUM lhs, ENUM rhs) gsl_noexcept                        \
    {                                                                 \
        typedef typename ::gsl::std11::underlying_type<ENUM>::type U; \
        return U(lhs) > U(rhs);                                       \
    }                                                                 \
    gsl_NODISCARD gsl_api inline gsl_constexpr bool                   \
    operator<=(ENUM lhs, ENUM rhs) gsl_noexcept                       \
    {                                                                 \
        typedef typename ::gsl::std11::underlying_type<ENUM>::type U; \
        return U(lhs) <= U(rhs);                                      \
    }                                                                 \
    gsl_NODISCARD gsl_api inline gsl_constexpr bool                   \
    operator>=(ENUM lhs, ENUM rhs) gsl_noexcept                       \
    {                                                                 \
        typedef typename ::gsl::std11::underlying_type<ENUM>::type U; \
        return U(lhs) >= U(rhs);                                      \
    }

//
// Defines bitmask operators `|`, `&`, `^`, `~`, `|=`, `&=`, and `^=` for the given enum type.
//
//     enum class Vegetables { tomato = 0b001, onion = 0b010, eggplant = 0b100 };
//     gsl_DEFINE_ENUM_BITMASK_OPERATORS( Vegetables )
//
#define gsl_DEFINE_ENUM_BITMASK_OPERATORS(ENUM) gsl_DEFINE_ENUM_BITMASK_OPERATORS_(ENUM)

//
// Defines relational operators `<`, `>`, `<=`, `>=` for the given enum type.
//
//     enum class OperatorPrecedence { additive = 0, multiplicative = 1, power = 2 };
//     gsl_DEFINE_ENUM_RELATIONAL_OPERATORS( OperatorPrecedence )
//
#define gsl_DEFINE_ENUM_RELATIONAL_OPERATORS(ENUM) gsl_DEFINE_ENUM_RELATIONAL_OPERATORS_(ENUM)

#endif  // gsl_HAVE( TYPE_TRAITS )

#define gsl_DIMENSION_OF(a) (sizeof(a) / sizeof(0 [a]))


// Method enabling (C++98, VC120 (VS2013) cannot use __VA_ARGS__)

#if gsl_HAVE(EXPRESSION_SFINAE)
#define gsl_TRAILING_RETURN_TYPE_(T) auto
#define gsl_RETURN_DECLTYPE_(EXPR) ->decltype(EXPR)
#else
#define gsl_TRAILING_RETURN_TYPE_(T) T
#define gsl_RETURN_DECLTYPE_(EXPR)
#endif

// NOTE: When using SFINAE in gsl-lite, please note that overloads of function templates must always use SFINAE with non-type default arguments
//       as explained in https://en.cppreference.com/w/cpp/types/enable_if#Notes. `gsl_ENABLE_IF_()` implements graceful fallback to default
//       type arguments (for compilers that don't support non-type default arguments); please verify that this is appropriate in the given
//       situation, and add additional checks if necessary.
//
//       Also, please note that `gsl_ENABLE_IF_()` doesn't enforce the constraint at all if no compiler/library support is available (i.e. pre-C++11).

#if gsl_HAVE(TYPE_TRAITS) && gsl_HAVE(DEFAULT_FUNCTION_TEMPLATE_ARG)
#if !gsl_BETWEEN(gsl_COMPILER_MSVC_VERSION, 1, 140)  // VS 2013 seems to have trouble with SFINAE for default non-type arguments
#define gsl_ENABLE_IF_(VA) , typename std::enable_if<(VA), int>::type = 0
#else
#define gsl_ENABLE_IF_(VA) , typename = typename std::enable_if<(VA), ::gsl::detail::enabler>::type
#endif
#else
#define gsl_ENABLE_IF_(VA)
#endif


// Other features:

#define gsl_HAVE_CONSTRAINED_SPAN_CONTAINER_CTOR (gsl_HAVE_DEFAULT_FUNCTION_TEMPLATE_ARG && gsl_HAVE_CONTAINER_DATA_METHOD)
#define gsl_HAVE_CONSTRAINED_SPAN_CONTAINER_CTOR_() gsl_HAVE_CONSTRAINED_SPAN_CONTAINER_CTOR

#define gsl_HAVE_UNCONSTRAINED_SPAN_CONTAINER_CTOR (gsl_CONFIG_ALLOWS_UNCONSTRAINED_SPAN_CONTAINER_CTOR && gsl_COMPILER_NVCC_VERSION == 0)
#define gsl_HAVE_UNCONSTRAINED_SPAN_CONTAINER_CTOR_() gsl_HAVE_UNCONSTRAINED_SPAN_CONTAINER_CTOR

// GSL API (e.g. for CUDA platform):

// Guidelines for using `gsl_api`:
//
// NVCC imposes the restriction that a function annotated `__host__ __device__` cannot call host-only or device-only functions.
// This makes `gsl_api` inappropriate for generic functions that call unknown code, e.g. the template constructors of `span<>`
// or functions like `finally()` which accept an arbitrary  function object.
// It is often preferable to annotate functions only with `gsl_constexpr` or `gsl_constexpr14`. The "extended constexpr" mode
// of NVCC (currently an experimental feature) will implicitly consider constexpr functions `__host__ __device__` functions
// but tolerates calls to host-only or device-only functions.

#ifndef gsl_api
#ifdef __CUDACC__
#define gsl_api __host__ __device__
#else
#define gsl_api /*gsl_api*/
#endif
#endif

// Additional includes:

#if !gsl_CPP11_OR_GREATER
#include <algorithm>  // for swap() before C++11
#endif                // ! gsl_CPP11_OR_GREATER

#if gsl_HAVE(ARRAY)
#include <array>  // indirectly includes reverse_iterator<>
#endif

#if !gsl_HAVE(ARRAY)
#include <iterator>  // for reverse_iterator<>
#endif

#if !gsl_HAVE(CONSTRAINED_SPAN_CONTAINER_CTOR) || !gsl_HAVE(AUTO)
#include <vector>
#endif

#if gsl_HAVE(INITIALIZER_LIST)
#include <initializer_list>
#endif

#if defined(gsl_CONFIG_CONTRACT_VIOLATION_ASSERTS)
#include <cassert>
#endif

#if defined(gsl_CONFIG_CONTRACT_VIOLATION_TRAPS) && gsl_COMPILER_MSVC_VERSION >= 110  // __fastfail() supported by VS 2012 and later
#include <intrin.h>
#endif

#if gsl_HAVE(ENUM_CLASS) && gsl_COMPILER_ARMCC_VERSION
#include <endian.h>
#endif

#if gsl_HAVE(TYPE_TRAITS)
#include <type_traits>  // for enable_if<>,
                        // add_const<>, add_pointer<>, common_type<>, make_signed<>, remove_cv<>, remove_const<>, remove_volatile<>, remove_reference<>, remove_cvref<>, remove_pointer<>, underlying_type<>,
                        // is_assignable<>, is_constructible<>, is_const<>, is_convertible<>, is_integral<>, is_pointer<>, is_signed<>,
                        // integral_constant<>, declval()
#elif gsl_HAVE(TR1_TYPE_TRAITS)
#include <tr1/type_traits>  // for add_const<>, remove_cv<>, remove_const<>, remove_volatile<>, remove_reference<>, integral_constant<>
#endif

#if gsl_FEATURE(EXPERIMENTAL_RETURN_GUARD)

// Declare __cxa_get_globals() or equivalent in namespace gsl::detail for uncaught_exceptions():

#if !gsl_HAVE(UNCAUGHT_EXCEPTIONS)
#if defined(_MSC_VER)  // MS-STL with either MSVC or clang-cl
namespace gsl
{
namespace detail
{
extern "C" char *__cdecl _getptd();
}
}  // namespace gsl
#elif gsl_COMPILER_CLANG_VERSION || gsl_COMPILER_GNUC_VERSION || gsl_COMPILER_APPLECLANG_VERSION
#if defined(__GLIBCXX__) || defined(__GLIBCPP__)  // libstdc++: prototype from cxxabi.h
#include <cxxabi.h>
#elif !defined(BOOST_CORE_UNCAUGHT_EXCEPTIONS_HPP_INCLUDED_)  // libc++: prototype from Boost?
#if defined(__FreeBSD__) || defined(__OpenBSD__)
namespace __cxxabiv1
{
struct __cxa_eh_globals;
extern "C" __cxa_eh_globals *__cxa_get_globals();
}  // namespace __cxxabiv1
#else
namespace __cxxabiv1
{
struct __cxa_eh_globals;
extern "C" __cxa_eh_globals *__cxa_get_globals() gsl_noexcept;
}  // namespace __cxxabiv1
#endif
#endif
namespace gsl
{
namespace detail
{
using ::__cxxabiv1::__cxa_get_globals;
}
}  // namespace gsl
#endif
#endif  // ! gsl_HAVE( UNCAUGHT_EXCEPTIONS )
#endif  // gsl_FEATURE( EXPERIMENTAL_RETURN_GUARD )


// MSVC warning suppression macros:

#if gsl_COMPILER_MSVC_VERSION >= 140 && !gsl_COMPILER_NVCC_VERSION
#define gsl_SUPPRESS_MSGSL_WARNING(expr) [[gsl::suppress(expr)]]
#define gsl_SUPPRESS_MSVC_WARNING(code, descr) __pragma(warning(suppress \
                                                                : code))
#define gsl_DISABLE_MSVC_WARNINGS(codes) __pragma(warning(push)) __pragma(warning(disable \
                                                                                  : codes))
#define gsl_RESTORE_MSVC_WARNINGS() __pragma(warning(pop))
#else
// TODO: define for Clang
#define gsl_SUPPRESS_MSGSL_WARNING(expr)
#define gsl_SUPPRESS_MSVC_WARNING(code, descr)
#define gsl_DISABLE_MSVC_WARNINGS(codes)
#define gsl_RESTORE_MSVC_WARNINGS()
#endif

// Suppress the following MSVC GSL warnings:
// - C26432: gsl::c.21 : if you define or delete any default operation in the type '...', define or delete them all
// - C26410: gsl::r.32 : the parameter 'ptr' is a reference to const unique pointer, use const T* or const T& instead
// - C26415: gsl::r.30 : smart pointer parameter 'ptr' is used only to access contained pointer. Use T* or T& instead
// - C26418: gsl::r.36 : shared pointer parameter 'ptr' is not copied or moved. Use T* or T& instead
// - C26472: gsl::t.1  : don't use a static_cast for arithmetic conversions;
//                       use brace initialization, gsl::narrow_cast or gsl::narrow
// - C26439: gsl::f.6  : special function 'function' can be declared 'noexcept'
// - C26440: gsl::f.6  : function 'function' can be declared 'noexcept'
// - C26455: gsl::f.6  : default constructor may not throw. Declare it 'noexcept'
// - C26473: gsl::t.1  : don't cast between pointer types where the source type and the target type are the same
// - C26481: gsl::b.1  : don't use pointer arithmetic. Use span instead
// - C26482: gsl::b.2  : only index into arrays using constant expressions
// - C26446: gdl::b.4  : prefer to use gsl::at() instead of unchecked subscript operator
// - C26490: gsl::t.1  : don't use reinterpret_cast
// - C26487: gsl::l.4  : don't return a pointer '(<some number>'s result)' that may be invalid
// - C26457: es.48     : (void) should not be used to ignore return values, use 'std::ignore =' instead

gsl_DISABLE_MSVC_WARNINGS(26432 26410 26415 26418 26472 26439 26440 26455 26473 26481 26482 26446 26490 26487 26457)

    namespace gsl
{
    // forward declare span<>:

    template <class T>
    class span;

    // C++98 emulation:

    namespace std98
    {
    // We implement `equal()` and `lexicographical_compare()` here to avoid having to pull in the <algorithm> header.
    template <class InputIt1, class InputIt2>
    bool equal(InputIt1 first1, InputIt1 last1, InputIt2 first2)
    {
        // Implementation borrowed from https://en.cppreference.com/w/cpp/algorithm/equal.
        for (; first1 != last1; ++first1, ++first2)
            {
                if (!(*first1 == *first2)) return false;
            }
        return true;
    }
    template <class InputIt1, class InputIt2>
    bool lexicographical_compare(InputIt1 first1, InputIt1 last1, InputIt2 first2, InputIt2 last2)
    {
        // Implementation borrowed from https://en.cppreference.com/w/cpp/algorithm/lexicographical_compare.
        for (; first1 != last1 && first2 != last2; ++first1, (void)++first2)
            {
                if (*first1 < *first2) return true;
                if (*first2 < *first1) return false;
            }
        return first1 == last1 && first2 != last2;
    }

    }  // namespace std98

    // C++11 emulation:

    namespace std11
    {
#if gsl_HAVE(ADD_CONST)

    using std::add_const;

#elif gsl_HAVE(TR1_ADD_CONST)

    using std::tr1::add_const;

#else

    template <class T>
    struct add_const
    {
        typedef const T type;
    };

#endif  // gsl_HAVE( ADD_CONST )

#if gsl_HAVE(REMOVE_CONST)

    using std::remove_const;
    using std::remove_cv;
    using std::remove_volatile;

#elif gsl_HAVE(TR1_REMOVE_CONST)

    using std::tr1::remove_const;
    using std::tr1::remove_cv;
    using std::tr1::remove_volatile;

#else

    template <class T>
    struct remove_const
    {
        typedef T type;
    };
    template <class T>
    struct remove_const<T const>
    {
        typedef T type;
    };

    template <class T>
    struct remove_volatile
    {
        typedef T type;
    };
    template <class T>
    struct remove_volatile<T volatile>
    {
        typedef T type;
    };

    template <class T>
    struct remove_cv
    {
        typedef typename remove_volatile<typename remove_const<T>::type>::type type;
    };

#endif  // gsl_HAVE( REMOVE_CONST )

#if gsl_HAVE(REMOVE_REFERENCE)

    using std::remove_reference;

#elif gsl_HAVE(TR1_REMOVE_REFERENCE)

    using std::tr1::remove_reference;

#else

    template <class T>
    struct remove_reference
    {
        typedef T type;
    };
    template <class T>
    struct remove_reference<T &>
    {
        typedef T type;
    };
#if gsl_HAVE(RVALUE_REFERENCE)
    template <class T>
    struct remove_reference<T &&>
    {
        typedef T type;
    };
#endif

#endif  // gsl_HAVE( REMOVE_REFERENCE )


#if gsl_HAVE(INTEGRAL_CONSTANT)

    using std::false_type;
    using std::integral_constant;
    using std::true_type;

#elif gsl_HAVE(TR1_INTEGRAL_CONSTANT)

    using std::tr1::false_type;
    using std::tr1::integral_constant;
    using std::tr1::true_type;

#else

    template <class T, T v>
    struct integral_constant
    {
        enum
        {
            value = v
        };
    };
    typedef integral_constant<bool, true> true_type;
    typedef integral_constant<bool, false> false_type;

#endif

#if gsl_HAVE(TYPE_TRAITS)

    using std::underlying_type;

#elif gsl_HAVE(TR1_TYPE_TRAITS)

    using std::tr1::underlying_type;

#else

    // We could try to define `underlying_type<>` for pre-C++11 here, but let's not until someone actually needs it.

#endif

    }  // namespace std11

    // C++14 emulation:

    namespace std14
    {
#if gsl_HAVE(UNIQUE_PTR)
#if gsl_HAVE(MAKE_UNIQUE)

    using std::make_unique;

#elif gsl_HAVE(VARIADIC_TEMPLATE)

    template <class T, class... Args>
    gsl_NODISCARD std::unique_ptr<T>
    make_unique(Args &&...args)
    {
        return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
    }

#endif  // gsl_HAVE( MAKE_UNIQUE ), gsl_HAVE( VARIADIC_TEMPLATE )
#endif  // gsl_HAVE( UNIQUE_PTR )

    }  // namespace std14

    namespace detail
    {
#if gsl_HAVE(VARIADIC_TEMPLATE)

    template <bool V0, class T0, class... Ts>
    struct conjunction_
    {
        using type = T0;
    };
    template <class T0, class T1, class... Ts>
    struct conjunction_<true, T0, T1, Ts...> : conjunction_<T1::value, T1, Ts...>
    {
    };
    template <bool V0, class T0, class... Ts>
    struct disjunction_
    {
        using type = T0;
    };
    template <class T0, class T1, class... Ts>
    struct disjunction_<false, T0, T1, Ts...> : disjunction_<T1::value, T1, Ts...>
    {
    };

#endif

    template <typename>
    struct dependent_false : std11::integral_constant<bool, false>
    {
    };

    }  // namespace detail

    // C++17 emulation:

    namespace std17
    {
    template <bool v>
    struct bool_constant : std11::integral_constant<bool, v>
    {
    };

#if gsl_CPP11_120

    template <class... Ts>
    struct conjunction;
    template <>
    struct conjunction<> : std11::true_type
    {
    };
    template <class T0, class... Ts>
    struct conjunction<T0, Ts...> : detail::conjunction_<T0::value, T0, Ts...>::type
    {
    };
    template <class... Ts>
    struct disjunction;
    template <>
    struct disjunction<> : std11::false_type
    {
    };
    template <class T0, class... Ts>
    struct disjunction<T0, Ts...> : detail::disjunction_<T0::value, T0, Ts...>::type
    {
    };
    template <class T>
    struct negation : std11::integral_constant<bool, !T::value>
    {
    };

#if gsl_CPP14_OR_GREATER

    template <class... Ts>
    constexpr bool conjunction_v = conjunction<Ts...>::value;
    template <class... Ts>
    constexpr bool disjunction_v = disjunction<Ts...>::value;
    template <class T>
    constexpr bool negation_v = negation<T>::value;

#endif  // gsl_CPP14_OR_GREATER

    template <class... Ts>
    struct make_void
    {
        typedef void type;
    };

    template <class... Ts>
    using void_t = typename make_void<Ts...>::type;

#endif  // gsl_CPP11_120

#if gsl_HAVE(CONSTRAINED_SPAN_CONTAINER_CTOR)

    template <class T, size_t N>
    gsl_NODISCARD gsl_api inline gsl_constexpr auto
    size(T const (&)[N]) gsl_noexcept -> size_t
    {
        return N;
    }

    template <class C>
    gsl_NODISCARD inline gsl_constexpr auto
    size(C const &cont) -> decltype(cont.size())
    {
        return cont.size();
    }

    template <class T, size_t N>
    gsl_NODISCARD gsl_api inline gsl_constexpr auto
        data(T (&arr)[N]) gsl_noexcept -> T *
    {
        return &arr[0];
    }

    template <class C>
    gsl_NODISCARD inline gsl_constexpr auto
    data(C &cont) -> decltype(cont.data())
    {
        return cont.data();
    }

    template <class C>
    gsl_NODISCARD inline gsl_constexpr auto
    data(C const &cont) -> decltype(cont.data())
    {
        return cont.data();
    }

    template <class E>
    gsl_NODISCARD inline gsl_constexpr auto
    data(std::initializer_list<E> il) gsl_noexcept -> E const *
    {
        return il.begin();
    }

#endif  // gsl_HAVE( CONSTRAINED_SPAN_CONTAINER_CTOR )

    }  // namespace std17

    // C++20 emulation:

    namespace std20
    {
#if gsl_CPP11_100

    struct identity
    {
        template <class T>
        gsl_constexpr T &&operator()(T &&arg) const gsl_noexcept
        {
            return std::forward<T>(arg);
        }
    };

#if gsl_HAVE(ENUM_CLASS)
    enum class endian
    {
#if defined(_WIN32)
        little = 0,
        big = 1,
        native = little
#elif gsl_COMPILER_GNUC_VERSION || gsl_COMPILER_CLANG_VERSION || gsl_COMPILER_APPLECLANG_VERSION
        little = __ORDER_LITTLE_ENDIAN__,
        big = __ORDER_BIG_ENDIAN__,
        native = __BYTE_ORDER__
#elif gsl_COMPILER_ARMCC_VERSION
        // from <endian.h> header file
        little = __LITTLE_ENDIAN,
        big = __BIG_ENDIAN,
        native = __BYTE_ORDER
#else
// Do not define any endianness constants for unknown compilers.
#endif
    };
#endif  // gsl_HAVE( ENUM_CLASS )

#endif  // gsl_CPP11_100

    template <class T>
    struct type_identity
    {
        typedef T type;
    };
#if gsl_HAVE(ALIAS_TEMPLATE)
    template <class T>
    using type_identity_t = typename type_identity<T>::type;
#endif  // gsl_HAVE( ALIAS_TEMPLATE )

#if gsl_HAVE(STD_SSIZE)

    using std::ssize;

#elif gsl_HAVE(CONSTRAINED_SPAN_CONTAINER_CTOR)

    template <class C>
    gsl_NODISCARD gsl_constexpr auto
    ssize(C const &c)
        -> typename std::common_type<std::ptrdiff_t, typename std::make_signed<decltype(c.size())>::type>::type
    {
        using R = typename std::common_type<std::ptrdiff_t, typename std::make_signed<decltype(c.size())>::type>::type;
        return static_cast<R>(c.size());
    }

    template <class T, std::size_t N>
    gsl_NODISCARD gsl_constexpr auto
    ssize(T const (&)[N]) gsl_noexcept -> std::ptrdiff_t
    {
        return std::ptrdiff_t(N);
    }

#endif  // gsl_HAVE( STD_SSIZE )

#if gsl_HAVE(REMOVE_CVREF)

    using std::remove_cvref;

#else

    template <class T>
    struct remove_cvref
    {
        typedef typename std11::remove_cv<typename std11::remove_reference<T>::type>::type type;
    };

#endif  // gsl_HAVE( REMOVE_CVREF )

    }  // namespace std20

    namespace detail
    {
    /// for gsl_ENABLE_IF_()

    /*enum*/ class enabler
    {
    };

#if gsl_HAVE(TYPE_TRAITS)

    template <class Q>
    struct is_span_oracle : std::false_type
    {
    };

    template <class T>
    struct is_span_oracle<span<T>> : std::true_type
    {
    };

    template <class Q>
    struct is_span : is_span_oracle<typename std::remove_cv<Q>::type>
    {
    };

    template <class Q>
    struct is_std_array_oracle : std::false_type
    {
    };

#if gsl_HAVE(ARRAY)

    template <class T, std::size_t Extent>
    struct is_std_array_oracle<std::array<T, Extent>> : std::true_type
    {
    };

#endif

    template <class Q>
    struct is_std_array : is_std_array_oracle<typename std::remove_cv<Q>::type>
    {
    };

    template <class Q>
    struct is_array : std::false_type
    {
    };

    template <class T>
    struct is_array<T[]> : std::true_type
    {
    };

    template <class T, std::size_t N>
    struct is_array<T[N]> : std::true_type
    {
    };

#if gsl_CPP11_140 && !gsl_BETWEEN(gsl_COMPILER_GNUC_VERSION, 1, 500)

    template <class, class = void>
    struct has_size_and_data : std::false_type
    {
    };

    template <class C>
    struct has_size_and_data<
        C, std17::void_t<
               decltype(std17::size(std::declval<C>())),
               decltype(std17::data(std::declval<C>()))>> : std::true_type
    {
    };

    template <class, class, class = void>
    struct is_compatible_element : std::false_type
    {
    };

    template <class C, class E>
    struct is_compatible_element<
        C, E, std17::void_t<decltype(std17::data(std::declval<C>())), typename std::remove_pointer<decltype(std17::data(std::declval<C &>()))>::type (*)[]>> : std::is_convertible<typename std::remove_pointer<decltype(std17::data(std::declval<C &>()))>::type (*)[], E (*)[]>
    {
    };

    template <class C>
    struct is_container : std17::bool_constant<
                              !is_span<C>::value && !is_array<C>::value && !is_std_array<C>::value && has_size_and_data<C>::value>
    {
    };

    template <class C, class E>
    struct is_compatible_container : std17::bool_constant<
                                         is_container<C>::value && is_compatible_element<C, E>::value>
    {
    };

#else  // ^^^ gsl_CPP11_140 && ! gsl_BETWEEN( gsl_COMPILER_GNUC_VERSION, 1, 500 ) ^^^ / vvv ! gsl_CPP11_140 || gsl_BETWEEN( gsl_COMPILER_GNUC_VERSION, 1, 500 ) vvv

    template <
        class C, class E, typename = typename std::enable_if<!is_span<C>::value && !is_array<C>::value && !is_std_array<C>::value && (std::is_convertible<typename std::remove_pointer<decltype(std17::data(std::declval<C &>()))>::type (*)[], E (*)[]>::value)
                              //  &&   has_size_and_data< C >::value
                              ,
                              enabler>::type,
        class = decltype(std17::size(std::declval<C>())), class = decltype(std17::data(std::declval<C>()))>
#if gsl_BETWEEN(gsl_COMPILER_MSVC_VERSION, 1, 140)
    // VS2013 has insufficient support for expression SFINAE; we cannot make `is_compatible_container<>` a proper type trait here
    struct is_compatible_container : std::true_type
    {
    };
#else
    struct is_compatible_container_r
    {
        is_compatible_container_r(int);
    };
    template <class C, class E>
    std::true_type is_compatible_container_f(is_compatible_container_r<C, E>);
    template <class C, class E>
    std::false_type is_compatible_container_f(...);

    template <class C, class E>
    struct is_compatible_container : decltype(is_compatible_container_f<C, E>(0))
    {
    };
#endif  // gsl_BETWEEN( gsl_COMPILER_MSVC_VERSION, 1, 140 )

#endif  // gsl_CPP11_140 && ! gsl_BETWEEN( gsl_COMPILER_GNUC_VERSION, 1, 500 )

#endif  // gsl_HAVE( TYPE_TRAITS )

    }  // namespace detail

    //
    // GSL.util: utilities
    //

    // Integer type for indices (e.g. in a loop).
    typedef gsl_CONFIG_INDEX_TYPE index;

    // Integer type for dimensions.
    typedef gsl_CONFIG_INDEX_TYPE dim;

    // Integer type for array strides.
    typedef gsl_CONFIG_INDEX_TYPE stride;

    // Integer type for pointer, iterator, or index differences.
    typedef gsl_CONFIG_INDEX_TYPE diff;

//
// GSL.owner: ownership pointers
//
#if gsl_HAVE(SHARED_PTR)
    using std::make_shared;
    using std::shared_ptr;
    using std::unique_ptr;
#if gsl_HAVE(MAKE_UNIQUE) || gsl_HAVE(VARIADIC_TEMPLATE)
    using std14::make_unique;
#endif
#endif

#if gsl_HAVE(ALIAS_TEMPLATE)
    template <class T
#if gsl_HAVE(TYPE_TRAITS)
        ,
        typename = typename std::enable_if<std::is_pointer<T>::value>::type
#endif
        >
    using owner = T;
#elif gsl_CONFIG(DEFAULTS_VERSION) == 0
    // TODO vNext: remove
    template <class T>
    struct owner
    {
        typedef T type;
    };
#endif

#define gsl_HAVE_OWNER_TEMPLATE gsl_HAVE_ALIAS_TEMPLATE
#define gsl_HAVE_OWNER_TEMPLATE_() gsl_HAVE_OWNER_TEMPLATE

// TODO vNext: remove
#if gsl_FEATURE(OWNER_MACRO)
#if gsl_HAVE(OWNER_TEMPLATE)
#define Owner(t) ::gsl::owner<t>
#else
#define Owner(t) ::gsl::owner<t>::type
#endif
#endif

    //
    // GSL.assert: assertions
    //

#if gsl_HAVE(TYPE_TRAITS)
#define gsl_ELIDE_(x) static_assert(::std::is_constructible<bool, decltype(x)>::value, "argument of contract check must be convertible to bool")
#else
#define gsl_ELIDE_(x)
#endif
#define gsl_NO_OP_() (static_cast<void>(0))

#if defined(gsl_CONFIG_UNENFORCED_CONTRACTS_ASSUME)
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
#if gsl_COMPILER_NVCC_VERSION >= 113
#define gsl_ASSUME_(x) ((x) ? static_cast<void>(0) : __builtin_unreachable())
#define gsl_ASSUME_UNREACHABLE_() __builtin_unreachable()
#else
#define gsl_ASSUME_(x) gsl_ELIDE_(x)           /* there is no assume intrinsic in CUDA device code */
#define gsl_ASSUME_UNREACHABLE_() gsl_NO_OP_() /* there is no assume intrinsic in CUDA device code */
#endif
#elif gsl_COMPILER_MSVC_VERSION >= 140
#define gsl_ASSUME_(x) __assume(x)
#define gsl_ASSUME_UNREACHABLE_() __assume(0)
#elif gsl_COMPILER_GNUC_VERSION
#define gsl_ASSUME_(x) ((x) ? static_cast<void>(0) : __builtin_unreachable())
#define gsl_ASSUME_UNREACHABLE_() __builtin_unreachable()
#elif defined(__has_builtin)
#if __has_builtin(__builtin_unreachable)
#define gsl_ASSUME_(x) ((x) ? static_cast<void>(0) : __builtin_unreachable())
#define gsl_ASSUME_UNREACHABLE_() __builtin_unreachable()
#else
#error gsl_CONFIG_UNENFORCED_CONTRACTS_ASSUME: gsl-lite does not know how to generate UB optimization hints for this compiler; use gsl_CONFIG_UNENFORCED_CONTRACTS_ELIDE instead
#endif
#else
#error gsl_CONFIG_UNENFORCED_CONTRACTS_ASSUME: gsl-lite does not know how to generate UB optimization hints for this compiler; use gsl_CONFIG_UNENFORCED_CONTRACTS_ELIDE instead
#endif
#endif  // defined( gsl_CONFIG_UNENFORCED_CONTRACTS_ASSUME )

#if defined(gsl_CONFIG_UNENFORCED_CONTRACTS_ASSUME)
#define gsl_CONTRACT_UNENFORCED_(x) gsl_ASSUME_(x)
#else  // defined( gsl_CONFIG_UNENFORCED_CONTRACTS_ELIDE ) [default]
#define gsl_CONTRACT_UNENFORCED_(x) gsl_ELIDE_(x)
#endif

#if defined(gsl_CONFIG_CONTRACT_VIOLATION_TRAPS)
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
#define gsl_TRAP_() __trap()
#elif gsl_COMPILER_MSVC_VERSION >= 110  // __fastfail() supported by VS 2012 and later
#define gsl_TRAP_() __fastfail(0)       /* legacy failure code for buffer-overrun errors, cf. winnt.h, "Fast fail failure codes" */
#elif gsl_COMPILER_GNUC_VERSION
#define gsl_TRAP_() __builtin_trap()
#elif defined(__has_builtin)
#if __has_builtin(__builtin_trap)
#define gsl_TRAP_() __builtin_trap()
#else
#error gsl_CONFIG_CONTRACT_VIOLATION_TRAPS: gsl-lite does not know how to generate a trap instruction for this compiler; use gsl_CONFIG_CONTRACT_VIOLATION_TERMINATES instead
#endif
#else
#error gsl_CONFIG_CONTRACT_VIOLATION_TRAPS: gsl-lite does not know how to generate a trap instruction for this compiler; use gsl_CONFIG_CONTRACT_VIOLATION_TERMINATES instead
#endif
#endif  // defined( gsl_CONFIG_CONTRACT_VIOLATION_TRAPS )

#if defined(gsl_CONFIG_CONTRACT_VIOLATION_CALLS_HANDLER)
#define gsl_CONTRACT_CHECK_(str, x) ((x) ? static_cast<void>(0) : ::gsl::fail_fast_assert_handler(#x, str, __FILE__, __LINE__))
#if defined(__CUDACC__) && defined(__CUDA_ARCH__)
#define gsl_FAILFAST_() (::gsl::fail_fast_assert_handler("", "GSL: failure", __FILE__, __LINE__), gsl_TRAP_()) /* do not let the custom assertion handler continue execution */
#else
#define gsl_FAILFAST_() (::gsl::fail_fast_assert_handler("", "GSL: failure", __FILE__, __LINE__), ::gsl::detail::fail_fast_terminate()) /* do not let the custom assertion handler continue execution */
#endif
#elif defined(__CUDACC__) && defined(__CUDA_ARCH__)
#if defined(gsl_CONFIG_CONTRACT_VIOLATION_ASSERTS) || !defined(NDEBUG)
#define gsl_CONTRACT_CHECK_(str, x) assert(str && (x))
#else
#define gsl_CONTRACT_CHECK_(str, x) ((x) ? static_cast<void>(0) : __trap())
#endif
#define gsl_FAILFAST_() (__trap())
#elif defined(gsl_CONFIG_CONTRACT_VIOLATION_ASSERTS)
#define gsl_CONTRACT_CHECK_(str, x) assert(str && (x))
#if !defined(NDEBUG)
#define gsl_FAILFAST_() (assert(!"GSL: failure"), ::gsl::detail::fail_fast_terminate())
#else
#define gsl_FAILFAST_() (::gsl::detail::fail_fast_terminate())
#endif
#elif defined(gsl_CONFIG_CONTRACT_VIOLATION_TRAPS)
#define gsl_CONTRACT_CHECK_(str, x) ((x) ? static_cast<void>(0) : gsl_TRAP_())
#if gsl_COMPILER_MSVC_VERSION
#define gsl_FAILFAST_() (gsl_TRAP_(), ::gsl::detail::fail_fast_terminate())
#else
#define gsl_FAILFAST_() (gsl_TRAP_())
#endif
#elif defined(gsl_CONFIG_CONTRACT_VIOLATION_THROWS)
#define gsl_CONTRACT_CHECK_(str, x) ((x) ? static_cast<void>(0) : ::gsl::detail::fail_fast_throw(str ": '" #x "' at " __FILE__ ":" gsl_STRINGIFY(__LINE__)))
#define gsl_FAILFAST_() (::gsl::detail::fail_fast_throw("GSL: failure at " __FILE__ ":" gsl_STRINGIFY(__LINE__)))
#else  // defined( gsl_CONFIG_CONTRACT_VIOLATION_TERMINATES ) [default]
#define gsl_CONTRACT_CHECK_(str, x) ((x) ? static_cast<void>(0) : ::gsl::detail::fail_fast_terminate())
#define gsl_FAILFAST_() (::gsl::detail::fail_fast_terminate())
#endif

#if defined(gsl_CONFIG_CONTRACT_CHECKING_OFF) || defined(gsl_CONFIG_CONTRACT_CHECKING_EXPECTS_OFF)
#define gsl_Expects(x) gsl_CONTRACT_UNENFORCED_(x)
#else
#define gsl_Expects(x) gsl_CONTRACT_CHECK_("GSL: Precondition failure", x)
#endif
#define Expects(x) gsl_Expects(x)
#if !defined(gsl_CONFIG_CONTRACT_CHECKING_AUDIT) || defined(gsl_CONFIG_CONTRACT_CHECKING_EXPECTS_OFF)
#define gsl_ExpectsAudit(x) gsl_ELIDE_(x)
#else
#define gsl_ExpectsAudit(x) gsl_CONTRACT_CHECK_("GSL: Precondition failure (audit)", x)
#endif

#if defined(gsl_CONFIG_CONTRACT_CHECKING_OFF) || defined(gsl_CONFIG_CONTRACT_CHECKING_ENSURES_OFF)
#define gsl_Ensures(x) gsl_CONTRACT_UNENFORCED_(x)
#else
#define gsl_Ensures(x) gsl_CONTRACT_CHECK_("GSL: Postcondition failure", x)
#endif
#define Ensures(x) gsl_Ensures(x)
#if !defined(gsl_CONFIG_CONTRACT_CHECKING_AUDIT) || defined(gsl_CONFIG_CONTRACT_CHECKING_ENSURES_OFF)
#define gsl_EnsuresAudit(x) gsl_ELIDE_(x)
#else
#define gsl_EnsuresAudit(x) gsl_CONTRACT_CHECK_("GSL: Postcondition failure (audit)", x)
#endif

#if defined(gsl_CONFIG_CONTRACT_CHECKING_OFF) || defined(gsl_CONFIG_CONTRACT_CHECKING_ASSERT_OFF)
#define gsl_Assert(x) gsl_CONTRACT_UNENFORCED_(x)
#else
#define gsl_Assert(x) gsl_CONTRACT_CHECK_("GSL: Assertion failure", x)
#endif
#if !defined(gsl_CONFIG_CONTRACT_CHECKING_AUDIT) || defined(gsl_CONFIG_CONTRACT_CHECKING_ASSERT_OFF)
#define gsl_AssertAudit(x) gsl_ELIDE_(x)
#else
#define gsl_AssertAudit(x) gsl_CONTRACT_CHECK_("GSL: Assertion failure (audit)", x)
#endif

#define gsl_FailFast() gsl_FAILFAST_()


    struct fail_fast : public std::logic_error
    {
        explicit fail_fast(char const *message)
            : std::logic_error(message) {}
    };

    namespace detail
    {
#if gsl_HAVE(EXCEPTIONS)
    gsl_NORETURN inline void fail_fast_throw(char const *message)
    {
        throw fail_fast(message);
    }
#endif  // gsl_HAVE( EXCEPTIONS )
    gsl_NORETURN inline void fail_fast_terminate() gsl_noexcept
    {
        std::terminate();
    }

    }  // namespace detail

    // Should be defined by user
    gsl_api void fail_fast_assert_handler(char const *const expression, char const *const message, char const *const file, int line);

#if defined(gsl_CONFIG_CONTRACT_VIOLATION_THROWS)

#if gsl_HAVE(EXCEPTIONS)
    gsl_DEPRECATED_MSG("don't call gsl::fail_fast_assert() directly; use contract checking macros instead")
        gsl_constexpr14 inline void
        fail_fast_assert(bool cond, char const *const message)
    {
        if (!cond)
            throw fail_fast(message);
    }
#endif  // gsl_HAVE( EXCEPTIONS )

#elif defined(gsl_CONFIG_CONTRACT_VIOLATION_CALLS_HANDLER)

    gsl_DEPRECATED_MSG("don't call gsl::fail_fast_assert() directly; use contract checking macros instead")
        gsl_api gsl_constexpr14 inline void
        fail_fast_assert(bool cond, char const *const expression, char const *const message, char const *const file, int line)
    {
        if (!cond)
            ::gsl::fail_fast_assert_handler(expression, message, file, line);
    }

#else  // defined( gsl_CONFIG_CONTRACT_VIOLATION_TERMINATES ) [default]

    gsl_DEPRECATED_MSG("don't call gsl::fail_fast_assert() directly; use contract checking macros instead")
        gsl_constexpr14 inline void
        fail_fast_assert(bool cond) gsl_noexcept
    {
        if (!cond)
            std::terminate();
    }

#endif


    //
    // GSL.util: utilities
    //

#if gsl_FEATURE(EXPERIMENTAL_RETURN_GUARD)

    // Add uncaught_exceptions for pre-2017 MSVC, GCC and Clang
    // Return unsigned char to save stack space, uncaught_exceptions can only increase by 1 in a scope

    namespace std11
    {
#if gsl_HAVE(UNCAUGHT_EXCEPTIONS)

    inline unsigned char uncaught_exceptions() gsl_noexcept
    {
        return static_cast<unsigned char>(std::uncaught_exceptions());
    }

#else                  // ! gsl_HAVE( UNCAUGHT_EXCEPTIONS )
#if defined(_MSC_VER)  // MS-STL with either MSVC or clang-cl

    inline unsigned char uncaught_exceptions() gsl_noexcept
    {
        return static_cast<unsigned char>(*reinterpret_cast<unsigned const *>(detail::_getptd() + (sizeof(void *) == 8 ? 0x100 : 0x90)));
    }

#elif gsl_COMPILER_CLANG_VERSION || gsl_COMPILER_GNUC_VERSION || gsl_COMPILER_APPLECLANG_VERSION

    inline unsigned char uncaught_exceptions() gsl_noexcept
    {
        return static_cast<unsigned char>((*reinterpret_cast<unsigned const *>(reinterpret_cast<unsigned char const *>(detail::__cxa_get_globals()) + sizeof(void *))));
    }

#endif
#endif

    }  // namespace std11

#endif  // gsl_FEATURE( EXPERIMENTAL_RETURN_GUARD )

#if gsl_STDLIB_CPP11_110

    gsl_DISABLE_MSVC_WARNINGS(4702)  // unreachable code

        template <class F>
        class final_action
    {
    public:
        explicit final_action(F action) gsl_noexcept
            : action_(std::move(action)),
              invoke_(true)
        {
        }

        final_action(final_action &&other) gsl_noexcept
            : action_(std::move(other.action_)),
              invoke_(other.invoke_)
        {
            other.invoke_ = false;
        }

        gsl_SUPPRESS_MSGSL_WARNING(f .6) virtual ~final_action() gsl_noexcept
        {
            if (invoke_)
                action_();
        }

        gsl_is_delete_access : final_action(final_action const &) gsl_is_delete;
        final_action &operator=(final_action const &) gsl_is_delete;
        final_action &operator=(final_action &&) gsl_is_delete;

    protected:
        void dismiss() gsl_noexcept
        {
            invoke_ = false;
        }

    private:
        F action_;
        bool invoke_;
    };

    template <class F>
    gsl_NODISCARD inline final_action<F>
    finally(F const &action) gsl_noexcept
    {
        return final_action<F>(action);
    }

    template <class F>
    gsl_NODISCARD inline final_action<F>
        finally(F && action) gsl_noexcept
    {
        return final_action<F>(std::forward<F>(action));
    }

#if gsl_FEATURE(EXPERIMENTAL_RETURN_GUARD)

    template <class F>
    class final_action_return : public final_action<F>
    {
    public:
        explicit final_action_return(F &&action) gsl_noexcept
            : final_action<F>(std::move(action)),
              exception_count(std11::uncaught_exceptions())
        {
        }

        final_action_return(final_action_return &&other) gsl_noexcept
            : final_action<F>(std::move(other)),
              exception_count(std11::uncaught_exceptions())
        {
        }

        ~final_action_return() override
        {
            if (std11::uncaught_exceptions() != exception_count)
                this->dismiss();
        }

        gsl_is_delete_access : final_action_return(final_action_return const &) gsl_is_delete;
        final_action_return &operator=(final_action_return const &) gsl_is_delete;

    private:
        unsigned char exception_count;
    };

    template <class F>
    gsl_NODISCARD inline final_action_return<F>
    on_return(F const &action) gsl_noexcept
    {
        return final_action_return<F>(action);
    }

    template <class F>
    gsl_NODISCARD inline final_action_return<F>
        on_return(F && action) gsl_noexcept
    {
        return final_action_return<F>(std::forward<F>(action));
    }

    template <class F>
    class final_action_error : public final_action<F>
    {
    public:
        explicit final_action_error(F &&action) gsl_noexcept
            : final_action<F>(std::move(action)),
              exception_count(std11::uncaught_exceptions())
        {
        }

        final_action_error(final_action_error &&other) gsl_noexcept
            : final_action<F>(std::move(other)),
              exception_count(std11::uncaught_exceptions())
        {
        }

        ~final_action_error() override
        {
            if (std11::uncaught_exceptions() == exception_count)
                this->dismiss();
        }

        gsl_is_delete_access : final_action_error(final_action_error const &) gsl_is_delete;
        final_action_error &operator=(final_action_error const &) gsl_is_delete;

    private:
        unsigned char exception_count;
    };

    template <class F>
    gsl_NODISCARD inline final_action_error<F>
    on_error(F const &action) gsl_noexcept
    {
        return final_action_error<F>(action);
    }

    template <class F>
    gsl_NODISCARD inline final_action_error<F>
        on_error(F && action) gsl_noexcept
    {
        return final_action_error<F>(std::forward<F>(action));
    }

#endif  // gsl_FEATURE( EXPERIMENTAL_RETURN_GUARD )

    gsl_RESTORE_MSVC_WARNINGS()

#else  // ! gsl_STDLIB_CPP11_110

    class final_action
    {
    public:
        typedef void (*Action)();

        final_action(Action action)
            : action_(action), invoke_(true)
        {
        }

        final_action(final_action const &other)
            : action_(other.action_), invoke_(other.invoke_)
        {
            other.invoke_ = false;
        }

        virtual ~final_action()
        {
            if (invoke_)
                action_();
        }

    protected:
        void dismiss()
        {
            invoke_ = false;
        }

    private:
        final_action &operator=(final_action const &);

    private:
        Action action_;
        mutable bool invoke_;
    };

    template <class F>
    inline final_action finally(F const &f)
    {
        return final_action((f));
    }

#if gsl_FEATURE(EXPERIMENTAL_RETURN_GUARD)

    class final_action_return : public final_action
    {
    public:
        explicit final_action_return(Action action)
            : final_action(action), exception_count(std11::uncaught_exceptions())
        {
        }

        ~final_action_return()
        {
            if (std11::uncaught_exceptions() != exception_count)
                this->dismiss();
        }

    private:
        final_action_return &operator=(final_action_return const &);

    private:
        unsigned char exception_count;
    };

    template <class F>
    inline final_action_return on_return(F const &action)
    {
        return final_action_return(action);
    }

    class final_action_error : public final_action
    {
    public:
        explicit final_action_error(Action action)
            : final_action(action), exception_count(std11::uncaught_exceptions())
        {
        }

        ~final_action_error()
        {
            if (std11::uncaught_exceptions() == exception_count)
                this->dismiss();
        }

    private:
        final_action_error &operator=(final_action_error const &);

    private:
        unsigned char exception_count;
    };

    template <class F>
    inline final_action_error on_error(F const &action)
    {
        return final_action_error(action);
    }

#endif  // gsl_FEATURE( EXPERIMENTAL_RETURN_GUARD )

#endif  // gsl_STDLIB_CPP11_110

#if gsl_STDLIB_CPP11_120

        template <class T, class U>
        gsl_NODISCARD gsl_api inline gsl_constexpr T
            narrow_cast(U && u) gsl_noexcept
    {
        return static_cast<T>(std::forward<U>(u));
    }

#else  // ! gsl_STDLIB_CPP11_120

    template <class T, class U>
    gsl_api inline T
    narrow_cast(U u) gsl_noexcept
    {
        return static_cast<T>(u);
    }

#endif  // gsl_STDLIB_CPP11_120

    struct narrowing_error : public std::exception
    {
        char const *what() const gsl_noexcept
#if gsl_HAVE(OVERRIDE_FINAL)
            override
#endif
        {
            return "narrowing_error";
        }
    };

#if gsl_HAVE(TYPE_TRAITS)

    namespace detail
    {
    template <class T, class U>
    struct is_same_signedness : public std::integral_constant<bool, std::is_signed<T>::value == std::is_signed<U>::value>
    {
    };

#if gsl_COMPILER_NVCC_VERSION
    // We do this to circumvent NVCC warnings about pointless unsigned comparisons with 0.
    template <class T>
    gsl_constexpr gsl_api bool is_negative(T value, std::true_type /*isSigned*/) gsl_noexcept
    {
        return value < T();
    }
    template <class T>
    gsl_constexpr gsl_api bool is_negative(T /*value*/, std::false_type /*isUnsigned*/) gsl_noexcept
    {
        return false;
    }
    template <class T, class U>
    gsl_constexpr gsl_api bool have_same_sign(T, U, std::true_type /*isSameSignedness*/) gsl_noexcept
    {
        return true;
    }
    template <class T, class U>
    gsl_constexpr gsl_api bool have_same_sign(T t, U u, std::false_type /*isSameSignedness*/) gsl_noexcept
    {
        return detail::is_negative(t, std::is_signed<T>()) == detail::is_negative(u, std::is_signed<U>());
    }
#endif  // gsl_COMPILER_NVCC_VERSION

    }  // namespace detail

#endif

    template <class T, class U>
    gsl_NODISCARD
#if !gsl_CONFIG(NARROW_THROWS_ON_TRUNCATION) && !defined(gsl_CONFIG_CONTRACT_VIOLATION_THROWS)
        gsl_api
#endif
        inline T
        narrow(U u)
    {
#if gsl_CONFIG(NARROW_THROWS_ON_TRUNCATION) && !gsl_HAVE(EXCEPTIONS)
        gsl_STATIC_ASSERT_(detail::dependent_false<T>::value,
            "According to the GSL specification, narrow<>() throws an exception of type narrowing_error on truncation. Therefore "
            "it cannot be used if exceptions are disabled. Consider using narrow_failfast<>() instead which raises a precondition "
            "violation if the given value cannot be represented in the target type.");
#endif

        T t = static_cast<T>(u);

        if (static_cast<U>(t) != u)
            {
#if gsl_HAVE(EXCEPTIONS) && (gsl_CONFIG(NARROW_THROWS_ON_TRUNCATION) || defined(gsl_CONFIG_CONTRACT_VIOLATION_THROWS))
                throw narrowing_error();
#else
                std::terminate();
#endif
            }

#if gsl_HAVE(TYPE_TRAITS)
#if gsl_COMPILER_NVCC_VERSION
        if (!detail::have_same_sign(t, u, detail::is_same_signedness<T, U>()))
#else
        gsl_SUPPRESS_MSVC_WARNING(4127, "conditional expression is constant") if (!detail::is_same_signedness<T, U>::value && (t < T()) != (u < U()))
#endif
#else
        // Don't assume T() works:
        gsl_SUPPRESS_MSVC_WARNING(4127, "conditional expression is constant") if ((t < 0) != (u < 0))
#endif
            {
#if gsl_HAVE(EXCEPTIONS) && (gsl_CONFIG(NARROW_THROWS_ON_TRUNCATION) || defined(gsl_CONFIG_CONTRACT_VIOLATION_THROWS))
                throw narrowing_error();
#else
            std::terminate();
#endif
            }

        return t;
    }

    template <class T, class U>
    gsl_NODISCARD gsl_api inline T
    narrow_failfast(U u)
    {
        T t = static_cast<T>(u);

        gsl_Expects(static_cast<U>(t) == u);

#if gsl_HAVE(TYPE_TRAITS)
#if gsl_COMPILER_NVCC_VERSION
        gsl_Expects(::gsl::detail::have_same_sign(t, u, ::gsl::detail::is_same_signedness<T, U>()));
#else
        gsl_SUPPRESS_MSVC_WARNING(4127, "conditional expression is constant")
            gsl_Expects((::gsl::detail::is_same_signedness<T, U>::value || (t < T()) == (u < U())));
#endif
#else
        // Don't assume T() works:
        gsl_SUPPRESS_MSVC_WARNING(4127, "conditional expression is constant")
            gsl_Expects((t < 0) == (u < 0));
#endif

        return t;
    }


    //
    // at() - Bounds-checked way of accessing static arrays, std::array, std::vector.
    //

    template <class T, size_t N>
    gsl_NODISCARD gsl_api inline gsl_constexpr14 T &
    at(T(&arr)[N], size_t pos)
    {
        gsl_Expects(pos < N);
        return arr[pos];
    }

    template <class Container>
    gsl_NODISCARD gsl_api inline gsl_constexpr14 typename Container::value_type &
    at(Container & cont, size_t pos)
    {
        gsl_Expects(pos < cont.size());
        return cont[pos];
    }

    template <class Container>
    gsl_NODISCARD gsl_api inline gsl_constexpr14 typename Container::value_type const &
    at(Container const &cont, size_t pos)
    {
        gsl_Expects(pos < cont.size());
        return cont[pos];
    }

#if gsl_HAVE(INITIALIZER_LIST)

    template <class T>
    gsl_NODISCARD gsl_api inline const gsl_constexpr14 T
    at(std::initializer_list<T> cont, size_t pos)
    {
        gsl_Expects(pos < cont.size());
        return *(cont.begin() + pos);
    }
#endif

    template <class T>
    gsl_NODISCARD gsl_api inline gsl_constexpr14 T &
    at(span<T> s, size_t pos)
    {
        return s[pos];
    }

    //
    // GSL.views: views
    //

    //
    // not_null<> - Wrap any indirection and enforce non-null.
    //

    template <class T>
    class not_null;

    namespace detail
    {
// helper class to figure out the pointed-to type of a pointer
#if gsl_STDLIB_CPP11_OR_GREATER
    template <class T, class E = void>
    struct element_type_helper
    {
        // For types without a member element_type (this will handle raw pointers)
        typedef typename std::remove_reference<decltype(*std::declval<T>())>::type type;
    };

    template <class T>
    struct element_type_helper<T, std17::void_t<typename T::element_type>>
    {
        // For types with a member element_type
        typedef typename T::element_type type;
    };
#else   // ! gsl_STDLIB_CPP11_OR_GREATER
    // Pre-C++11, we cannot have decltype, so we cannot handle types without a member element_type
    template <class T, class E = void>
    struct element_type_helper
    {
        typedef typename T::element_type type;
    };

    template <class T>
    struct element_type_helper<T *>
    {
        typedef T type;
    };
#endif  // gsl_STDLIB_CPP11_OR_GREATER

    template <class T>
    struct is_not_null_or_bool_oracle : std11::false_type
    {
    };
    template <class T>
    struct is_not_null_or_bool_oracle<not_null<T>> : std11::true_type
    {
    };
    template <>
    struct is_not_null_or_bool_oracle<bool> : std11::true_type
    {
    };


    template <class T, bool IsCopyable = true>
    struct not_null_data;
#if gsl_HAVE(MOVE_FORWARD)
    template <class T>
    struct not_null_data<T, false>
    {
        T ptr_;

        gsl_api gsl_constexpr14 not_null_data(T &&_ptr) gsl_noexcept
            : ptr_(std::move(_ptr))
        {
        }

        gsl_api gsl_constexpr14 not_null_data(not_null_data &&other) gsl_noexcept
            : ptr_(std::move(other.ptr_))
        {
        }
        gsl_api gsl_constexpr14 not_null_data &operator=(not_null_data &&other) gsl_noexcept
        {
            ptr_ = std::move(other.ptr_);
            return *this;
        }

        gsl_is_delete_access : not_null_data(not_null_data const &) gsl_is_delete;
        not_null_data &operator=(not_null_data const &) gsl_is_delete;
    };
#if gsl_CONFIG_DEFAULTS_VERSION >= 1
#endif  // gsl_CONFIG_DEFAULTS_VERSION >= 1
#endif  // gsl_HAVE( MOVE_FORWARD )
    template <class T>
    struct not_null_data<T, true>
    {
        T ptr_;

        gsl_api gsl_constexpr14 not_null_data(T const &_ptr) gsl_noexcept
            : ptr_(_ptr)
        {
        }

#if gsl_HAVE(MOVE_FORWARD)
        gsl_api gsl_constexpr14 not_null_data(T &&_ptr) gsl_noexcept
            : ptr_(std::move(_ptr))
        {
        }

        gsl_api gsl_constexpr14 not_null_data(not_null_data &&other) gsl_noexcept
            : ptr_(std::move(other.ptr_))
        {
        }
        gsl_api gsl_constexpr14 not_null_data &operator=(not_null_data &&other) gsl_noexcept
        {
            ptr_ = std::move(other.ptr_);
            return *this;
        }
#endif  // gsl_HAVE( MOVE_FORWARD )

        gsl_api gsl_constexpr14 not_null_data(not_null_data const &other)
            : ptr_(other.ptr_)
        {
            gsl_Expects(ptr_ != gsl_nullptr);
        }
        gsl_api gsl_constexpr14 not_null_data &operator=(not_null_data const &other)
        {
            gsl_Expects(other.ptr_ != gsl_nullptr);
            ptr_ = other.ptr_;
            return *this;
        }
    };
#if gsl_CONFIG_DEFAULTS_VERSION >= 1
    template <class T>
    struct not_null_data<T *, true>
    {
        T *ptr_;

        gsl_api gsl_constexpr14 not_null_data(T *_ptr) gsl_noexcept
            : ptr_(_ptr)
        {
        }
    };
#endif  // gsl_CONFIG_DEFAULTS_VERSION >= 1
    template <class T>
    struct is_copyable
#if gsl_HAVE(TYPE_TRAITS)
        : std11::integral_constant<bool, std::is_copy_constructible<T>::value && std::is_copy_assignable<T>::value>
#else
        : std11::true_type
#endif
    {
    };
#if gsl_HAVE(TYPE_TRAITS) && gsl_HAVE(UNIQUE_PTR) && gsl_BETWEEN(gsl_COMPILER_MSVC_VERSION, 1, 140)
    // Type traits are buggy in VC++ 2013, so we explicitly declare `unique_ptr<>` non-copyable.
    template <class T, class Deleter>
    struct is_copyable<std::unique_ptr<T, Deleter>> : std11::false_type
    {
    };
#endif

    template <class T>
    struct not_null_accessor;

    }  // namespace detail

    template <class T>
    class not_null
    {
    private:
        detail::not_null_data<T, detail::is_copyable<T>::value> data_;

        // need to access `not_null<U>::data_`
        template <class U>
        friend class not_null;

        template <class U>
        friend struct detail::not_null_accessor;

    public:
        typedef typename detail::element_type_helper<T>::type element_type;

#if gsl_HAVE(TYPE_TRAITS)
        static_assert(std::is_assignable<typename std::remove_const<typename std::remove_reference<T>::type>::type &, std::nullptr_t>::value, "T cannot be assigned nullptr.");
#endif

#if gsl_CONFIG(NOT_NULL_EXPLICIT_CTOR)
#if gsl_HAVE(MOVE_FORWARD)
        template <class U
        // In Clang 3.x, `is_constructible<not_null<unique_ptr<X>>, unique_ptr<X>>` tries to instantiate the copy constructor of `unique_ptr<>`, triggering an error.
        // Note that Apple Clang's `__clang_major__` etc. are different from regular Clang.
#if gsl_HAVE(TYPE_TRAITS) && gsl_HAVE(DEFAULT_FUNCTION_TEMPLATE_ARG) && !gsl_BETWEEN(gsl_COMPILER_CLANG_VERSION, 1, 400) && !gsl_BETWEEN(gsl_COMPILER_APPLECLANG_VERSION, 1, 1001)
            // We *have* to use SFINAE with an NTTP arg here, otherwise the overload is ambiguous.
            ,
            typename std::enable_if<(std::is_constructible<T, U>::value), int>::type = 0
#endif
            >
        gsl_api gsl_constexpr14 explicit not_null(U other)
            : data_(T(std::move(other)))
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }
#else   // a.k.a. ! gsl_HAVE( MOVE_FORWARD )
        template <class U>
        gsl_api gsl_constexpr14 explicit not_null(U const &other)
            : data_(T(other))
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }
#endif  // gsl_HAVE( MOVE_FORWARD )
#else   // a.k.a. !gsl_CONFIG( NOT_NULL_EXPLICIT_CTOR )
#if gsl_HAVE(MOVE_FORWARD)
        // In Clang 3.x, `is_constructible<not_null<unique_ptr<X>>, unique_ptr<X>>` tries to instantiate the copy constructor of `unique_ptr<>`, triggering an error.
        // Note that Apple Clang's `__clang_major__` etc. are different from regular Clang.
#if gsl_HAVE(TYPE_TRAITS) && gsl_HAVE(DEFAULT_FUNCTION_TEMPLATE_ARG) && !gsl_BETWEEN(gsl_COMPILER_CLANG_VERSION, 1, 400) && !gsl_BETWEEN(gsl_COMPILER_APPLECLANG_VERSION, 1, 1001)
        template <class U
            // We *have* to use SFINAE with an NTTP arg here, otherwise the overload is ambiguous.
            ,
            typename std::enable_if<(std::is_constructible<T, U>::value && !std::is_convertible<U, T>::value), int>::type = 0>
        gsl_api gsl_constexpr14 explicit not_null(U other)
            : data_(T(std::move(other)))
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }

        template <class U
            // We *have* to use SFINAE with an NTTP arg here, otherwise the overload is ambiguous.
            ,
            typename std::enable_if<(std::is_convertible<U, T>::value), int>::type = 0>
        gsl_api gsl_constexpr14 not_null(U other)
            : data_(std::move(other))
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }
#else   // a.k.a. !( gsl_HAVE( TYPE_TRAITS ) && gsl_HAVE( DEFAULT_FUNCTION_TEMPLATE_ARG ) && ! gsl_BETWEEN( gsl_COMPILER_CLANG_VERSION, 1, 400 ) && ! gsl_BETWEEN( gsl_COMPILER_APPLECLANG_VERSION, 1, 1001 )
        // If type_traits are not available, then we can't distinguish `is_convertible<>` and `is_constructible<>`, so we unconditionally permit implicit construction.
        template <class U>
        gsl_api gsl_constexpr14 not_null(U other)
            : data_(T(std::move(other)))
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }
#endif  // gsl_HAVE( TYPE_TRAITS ) && gsl_HAVE( DEFAULT_FUNCTION_TEMPLATE_ARG ) && ! gsl_BETWEEN( gsl_COMPILER_CLANG_VERSION, 1, 400 ) && ! gsl_BETWEEN( gsl_COMPILER_APPLECLANG_VERSION, 1, 1001 )
#else   // a.k.a. ! gsl_HAVE( MOVE_FORWARD )
        template <class U>
        gsl_api gsl_constexpr14 not_null(U const &other)
            : data_(T(other))
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }
#endif  // gsl_HAVE( MOVE_FORWARD )
#endif  // gsl_CONFIG( NOT_NULL_EXPLICIT_CTOR )

    public:
#if gsl_HAVE(MOVE_FORWARD)
        // In Clang 3.x, `is_constructible<not_null<unique_ptr<X>>, unique_ptr<X>>` tries to instantiate the copy constructor of `unique_ptr<>`, triggering an error.
        // Note that Apple Clang's `__clang_major__` etc. are different from regular Clang.
#if gsl_HAVE(TYPE_TRAITS) && gsl_HAVE(DEFAULT_FUNCTION_TEMPLATE_ARG) && !gsl_BETWEEN(gsl_COMPILER_CLANG_VERSION, 1, 400) && !gsl_BETWEEN(gsl_COMPILER_APPLECLANG_VERSION, 1, 1001)
        template <class U
            // We *have* to use SFINAE with an NTTP arg here, otherwise the overload is ambiguous.
            ,
            typename std::enable_if<(std::is_constructible<T, U>::value && !std::is_convertible<U, T>::value), int>::type = 0>
        gsl_api gsl_constexpr14 explicit not_null(not_null<U> other)
            : data_(T(std::move(other.data_.ptr_)))
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }

        template <class U
            // We *have* to use SFINAE with an NTTP arg here, otherwise the overload is ambiguous.
            ,
            typename std::enable_if<(std::is_convertible<U, T>::value), int>::type = 0>
        gsl_api gsl_constexpr14 not_null(not_null<U> other)
            : data_(T(std::move(other.data_.ptr_)))
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }
#else   // a.k.a. ! ( gsl_HAVE( TYPE_TRAITS ) && gsl_HAVE( DEFAULT_FUNCTION_TEMPLATE_ARG ) && ! gsl_BETWEEN( gsl_COMPILER_CLANG_VERSION, 1, 400 ) && ! gsl_BETWEEN( gsl_COMPILER_APPLECLANG_VERSION, 1, 1001 )
        // If type_traits are not available, then we can't distinguish `is_convertible<>` and `is_constructible<>`, so we unconditionally permit implicit construction.
        template <class U>
        gsl_api gsl_constexpr14 not_null(not_null<U> other)
            : data_(T(std::move(other.data_.ptr_)))
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }
        template <class U>
        gsl_api gsl_constexpr14 not_null<T> &operator=(not_null<U> other)
        {
            gsl_Expects(other.data_.ptr_ != gsl_nullptr);
            data_.ptr_ = std::move(other.data_.ptr_);
            return *this;
        }
#endif  // gsl_HAVE( TYPE_TRAITS ) && gsl_HAVE( DEFAULT_FUNCTION_TEMPLATE_ARG ) && ! gsl_BETWEEN( gsl_COMPILER_CLANG_VERSION, 1, 400 ) && ! gsl_BETWEEN( gsl_COMPILER_APPLECLANG_VERSION, 1, 1001 )
#else   // a.k.a. ! gsl_HAVE( MOVE_FORWARD )
        template <class U>
        gsl_api gsl_constexpr14 not_null(not_null<U> const &other)
            : data_(T(other.data_.ptr_))
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }
        template <class U>
        gsl_api gsl_constexpr14 not_null<T> &operator=(not_null<U> const &other)
        {
            gsl_Expects(other.data_.ptr_ != gsl_nullptr);
            data_.ptr_ = other.data_.ptr_;
            return *this;
        }
#endif  // gsl_HAVE( MOVE_FORWARD )

#if gsl_CONFIG(TRANSPARENT_NOT_NULL)
        gsl_NODISCARD gsl_api gsl_constexpr14 element_type *
        get() const
        {
            element_type *result = data_.ptr_.get();
            gsl_Ensures(result != gsl_nullptr);
            return result;
        }
#else
#if gsl_CONFIG(NOT_NULL_GET_BY_CONST_REF)
        gsl_NODISCARD gsl_api gsl_constexpr14 T const &
        get() const
        {
            T const &result = data_.ptr_;
            gsl_Ensures(result != gsl_nullptr);
            return result;
        }
#else
        gsl_NODISCARD gsl_api gsl_constexpr14 T
        get() const
        {
            T result = data_.ptr_;
            gsl_Ensures(result != gsl_nullptr);
            return result;
        }
#endif
#endif

        // We want an implicit conversion operator that can be used to convert from both lvalues (by
        // const reference or by copy) and rvalues (by move). So it seems like we could define
        //
        //     template< class U >
        //     operator U const &() const & { ... }
        //     template< class U >
        //     operator U &&() && { ... }
        //
        // However, having two conversion operators with different return types renders the assignment
        // operator of the result type ambiguous:
        //
        //     not_null<std::unique_ptr<T>> p( ... );
        //     std::unique_ptr<U> q;
        //     q = std::move( p ); // ambiguous
        //
        // To avoid this ambiguity, we have both overloads of the conversion operator return `U`
        // rather than `U const &` or `U &&`. This implies that converting an lvalue always induces
        // a copy, which can cause unnecessary copies or even fail to compile in some situations:
        //
        //     not_null<std::shared_ptr<T>> sp( ... );
        //     std::shared_ptr<U> const & rs = sp; // unnecessary copy
        //     std::unique_ptr<U> const & ru = p; // error: cannot copy `unique_ptr<T>`
        //
        // However, these situations are rather unusual, and the following, more frequent situations
        // remain unimpaired:
        //
        //     std::shared_ptr<U> vs = sp; // no extra copy
        //     std::unique_ptr<U> vu = std::move( p );

#if gsl_HAVE(MOVE_FORWARD) && gsl_HAVE(TYPE_TRAITS) && gsl_HAVE(DEFAULT_FUNCTION_TEMPLATE_ARG) && gsl_HAVE(EXPLICIT)
        // explicit conversion operator

        template <class U
            // We *have* to use SFINAE with an NTTP arg here, otherwise the overload is ambiguous.
            ,
            typename std::enable_if<(std::is_constructible<U, T const &>::value && !std::is_convertible<T, U>::value && !detail::is_not_null_or_bool_oracle<U>::value), int>::type = 0>
        gsl_NODISCARD gsl_api gsl_constexpr14 explicit
        operator U() const
#if gsl_HAVE(FUNCTION_REF_QUALIFIER)
            &
#endif
        {
            U result(data_.ptr_);
            gsl_Ensures(result != gsl_nullptr);
            return result;
        }
#if gsl_HAVE(FUNCTION_REF_QUALIFIER)
        template <class U
            // We *have* to use SFINAE with an NTTP arg here, otherwise the overload is ambiguous.
            ,
            typename std::enable_if<(std::is_constructible<U, T>::value && !std::is_convertible<T, U>::value && !detail::is_not_null_or_bool_oracle<U>::value), int>::type = 0>
        gsl_NODISCARD gsl_api gsl_constexpr14 explicit
        operator U() &&
        {
            U result(std::move(data_.ptr_));
            gsl_Ensures(result != gsl_nullptr);
            return result;
        }
#endif

        // implicit conversion operator
        template <class U
            // We *have* to use SFINAE with an NTTP arg here, otherwise the overload is ambiguous.
            ,
            typename std::enable_if<(std::is_constructible<U, T const &>::value && std::is_convertible<T, U>::value && !detail::is_not_null_or_bool_oracle<U>::value), int>::type = 0>
        gsl_NODISCARD gsl_api gsl_constexpr14
        operator U() const
#if gsl_HAVE(FUNCTION_REF_QUALIFIER)
            &
#endif
        {
            U result(data_.ptr_);
            gsl_Ensures(result != gsl_nullptr);
            return result;
        }
#if gsl_HAVE(FUNCTION_REF_QUALIFIER)
        template <class U
            // We *have* to use SFINAE with an NTTP arg here, otherwise the overload is ambiguous.
            ,
            typename std::enable_if<(std::is_convertible<T, U>::value && !detail::is_not_null_or_bool_oracle<U>::value), int>::type = 0>
        gsl_NODISCARD gsl_api gsl_constexpr14
        operator U() &&
        {
            U result(std::move(data_.ptr_));
            gsl_Ensures(result != gsl_nullptr);
            return result;
        }
#endif
#else   // a.k.a. #if !( gsl_HAVE( MOVE_FORWARD ) && gsl_HAVE( TYPE_TRAITS ) && gsl_HAVE( DEFAULT_FUNCTION_TEMPLATE_ARG ) && gsl_HAVE( EXPLICIT ) )
        template <class U>
        gsl_NODISCARD gsl_api gsl_constexpr14
        operator U() const
        {
            U result(data_.ptr_);
            gsl_Ensures(result != gsl_nullptr);
            return result;
        }
#endif  // gsl_HAVE( MOVE_FORWARD ) && gsl_HAVE( TYPE_TRAITS ) && gsl_HAVE( DEFAULT_FUNCTION_TEMPLATE_ARG ) && gsl_HAVE( EXPLICIT )

        gsl_NODISCARD gsl_api gsl_constexpr14 T const &
        operator->() const
        {
            T const &result(data_.ptr_);
            gsl_Ensures(result != gsl_nullptr);
            return result;
        }

        gsl_NODISCARD gsl_api gsl_constexpr14 element_type &
        operator*() const
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
            return *data_.ptr_;
        }

#if gsl_HAVE(MOVE_FORWARD)
        // Visual C++ 2013 doesn't generate default move constructors, so we declare them explicitly.
        gsl_api gsl_constexpr14 not_null(not_null &&other)
            gsl_noexcept_not_testing  // we want to be nothrow-movable despite the precondition check
            : data_(std::move(other.data_))
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }
        gsl_api gsl_constexpr14 not_null &operator=(not_null &&other)
            gsl_noexcept_not_testing  // we want to be nothrow-movable despite the precondition check
        {
            gsl_Expects(other.data_.ptr_ != gsl_nullptr || &other == this);
            data_ = std::move(other.data_);
            return *this;
        }
#endif  // gsl_HAVE( MOVE_FORWARD )

#if gsl_HAVE(IS_DEFAULT)
        gsl_constexpr14 not_null(not_null const &) = default;
        gsl_constexpr14 not_null &operator=(not_null const &) = default;
#endif

        gsl_api gsl_constexpr20 friend void swap(not_null &lhs, not_null &rhs)
            gsl_noexcept_not_testing  // we want to be nothrow-swappable despite the precondition check
        {
            gsl_Expects(lhs.data_.ptr_ != gsl_nullptr && rhs.data_.ptr_ != gsl_nullptr);
            using std::swap;
            swap(lhs.data_.ptr_, rhs.data_.ptr_);
        }

        gsl_is_delete_access : not_null() gsl_is_delete;
        // prevent compilation when initialized with a nullptr or literal 0:
#if gsl_HAVE(NULLPTR)
        not_null(std::nullptr_t) gsl_is_delete;
        not_null &operator=(std::nullptr_t) gsl_is_delete;
#else
        not_null(int) gsl_is_delete;
        not_null &operator=(int) gsl_is_delete;
#endif

        // unwanted operators...pointers only point to single objects!
        not_null &operator++() gsl_is_delete;
        not_null &operator--() gsl_is_delete;
        not_null operator++(int) gsl_is_delete;
        not_null operator--(int) gsl_is_delete;
        not_null &operator+(size_t) gsl_is_delete;
        not_null &operator+=(size_t) gsl_is_delete;
        not_null &operator-(size_t) gsl_is_delete;
        not_null &operator-=(size_t) gsl_is_delete;
        not_null &operator+=(std::ptrdiff_t) gsl_is_delete;
        not_null &operator-=(std::ptrdiff_t) gsl_is_delete;
        void operator[](std::ptrdiff_t) const gsl_is_delete;
    };
#if gsl_CONFIG_DEFAULTS_VERSION >= 1
    template <class T>
    class not_null<T *>
    {
    private:
        detail::not_null_data<T *, true> data_;

        // need to access `not_null<U>::data_`
        template <class U>
        friend class not_null;

        template <class U>
        friend struct detail::not_null_accessor;

    public:
        typedef T element_type;

        gsl_api gsl_constexpr14
#if gsl_CONFIG(NOT_NULL_EXPLICIT_CTOR)
            explicit
#endif  // gsl_CONFIG( NOT_NULL_EXPLICIT_CTOR )
            not_null(T *other)
            : data_(other)
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }

#if gsl_HAVE(MOVE_FORWARD)
        // In Clang 3.x, `is_constructible<not_null<unique_ptr<X>>, unique_ptr<X>>` tries to instantiate the copy constructor of `unique_ptr<>`, triggering an error.
        // Note that Apple Clang's `__clang_major__` etc. are different from regular Clang.
#if gsl_HAVE(TYPE_TRAITS) && gsl_HAVE(DEFAULT_FUNCTION_TEMPLATE_ARG) && !gsl_BETWEEN(gsl_COMPILER_CLANG_VERSION, 1, 400) && !gsl_BETWEEN(gsl_COMPILER_APPLECLANG_VERSION, 1, 1001)
        template <class U
            // We *have* to use SFINAE with an NTTP arg here, otherwise the overload is ambiguous.
            ,
            typename std::enable_if<(std::is_constructible<T *, U>::value && !std::is_convertible<U, T *>::value), int>::type = 0>
        gsl_api gsl_constexpr14 explicit not_null(not_null<U> other)
            : data_(static_cast<T *>(std::move(other.data_.ptr_)))
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }

        template <class U
            // We *have* to use SFINAE with an NTTP arg here, otherwise the overload is ambiguous.
            ,
            typename std::enable_if<(std::is_convertible<U, T *>::value), int>::type = 0>
        gsl_api gsl_constexpr14 not_null(not_null<U> other)
            : data_(std::move(other.data_.ptr_))
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }
#else   // a.k.a. ! ( gsl_HAVE( TYPE_TRAITS ) && gsl_HAVE( DEFAULT_FUNCTION_TEMPLATE_ARG ) && ! gsl_BETWEEN( gsl_COMPILER_CLANG_VERSION, 1, 400 ) && ! gsl_BETWEEN( gsl_COMPILER_APPLECLANG_VERSION, 1, 1001 )
        // If type_traits are not available, then we can't distinguish `is_convertible<>` and `is_constructible<>`, so we unconditionally permit implicit construction.
        template <class U>
        gsl_api gsl_constexpr14 not_null(not_null<U> other)
            : data_(std::move(other.data_.ptr_))
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }
        template <class U>
        gsl_api gsl_constexpr14 not_null<T *> &operator=(not_null<U> other)
        {
            gsl_Expects(other.data_.ptr_ != gsl_nullptr);
            data_.ptr_ = std::move(other.data_.ptr_);
            return *this;
        }
#endif  // gsl_HAVE( TYPE_TRAITS ) && gsl_HAVE( DEFAULT_FUNCTION_TEMPLATE_ARG ) && ! gsl_BETWEEN( gsl_COMPILER_CLANG_VERSION, 1, 400 ) && ! gsl_BETWEEN( gsl_COMPILER_APPLECLANG_VERSION, 1, 1001 )
#else   // a.k.a. ! gsl_HAVE( MOVE_FORWARD )
        template <class U>
        gsl_api gsl_constexpr14 not_null(not_null<U> const &other)
            : data_(other.data_.ptr_)
        {
            gsl_Expects(data_.ptr_ != gsl_nullptr);
        }
        template <class U>
        gsl_api gsl_constexpr14 not_null<T *> &operator=(not_null<U> const &other)
        {
            gsl_Expects(other.data_.ptr_ != gsl_nullptr);
            data_.ptr_ = other.data_.ptr_;
            return *this;
        }
#endif  // gsl_HAVE( MOVE_FORWARD )

#if !gsl_CONFIG(TRANSPARENT_NOT_NULL)
        gsl_NODISCARD gsl_api gsl_constexpr14 T *
        get() const
        {
            return data_.ptr_;
        }
#endif

#if gsl_HAVE(TYPE_TRAITS) && gsl_HAVE(DEFAULT_FUNCTION_TEMPLATE_ARG) && gsl_HAVE(EXPLICIT)
        // explicit conversion operator
        template <class U
            // We *have* to use SFINAE with an NTTP arg here, otherwise the overload is ambiguous.
            ,
            typename std::enable_if<(std::is_constructible<U, T *>::value && !std::is_convertible<T *, U>::value && !detail::is_not_null_or_bool_oracle<U>::value), int>::type = 0>
        gsl_NODISCARD gsl_api gsl_constexpr14 explicit
        operator U() const
        {
            return U(data_.ptr_);
        }

        // implicit conversion operator
        template <class U
            // We *have* to use SFINAE with an NTTP arg here, otherwise the overload is ambiguous.
            ,
            typename std::enable_if<(std::is_constructible<U, T *>::value && std::is_convertible<T *, U>::value && !detail::is_not_null_or_bool_oracle<U>::value), int>::type = 0>
        gsl_NODISCARD gsl_api gsl_constexpr14
        operator U() const
        {
            return data_.ptr_;
        }
#else   // a.k.a. #if !( gsl_HAVE( MOVE_FORWARD ) && gsl_HAVE( TYPE_TRAITS ) && gsl_HAVE( DEFAULT_FUNCTION_TEMPLATE_ARG ) && gsl_HAVE( EXPLICIT ) )
        template <class U>
        gsl_NODISCARD gsl_api gsl_constexpr14
        operator U() const
        {
            return data_.ptr_;
        }
#endif  // gsl_HAVE( MOVE_FORWARD ) && gsl_HAVE( TYPE_TRAITS ) && gsl_HAVE( DEFAULT_FUNCTION_TEMPLATE_ARG ) && gsl_HAVE( EXPLICIT )

        gsl_NODISCARD gsl_api gsl_constexpr14 T *
        operator->() const
        {
            return data_.ptr_;
        }

        gsl_NODISCARD gsl_api gsl_constexpr14 element_type &
        operator*() const
        {
            return *data_.ptr_;
        }

#if gsl_HAVE(IS_DEFAULT)
        gsl_constexpr14 not_null(not_null const &) = default;
        gsl_constexpr14 not_null &operator=(not_null const &) = default;
#endif

        gsl_api gsl_constexpr20 friend void swap(not_null &lhs, not_null &rhs) gsl_noexcept
        {
            using std::swap;
            swap(lhs.data_.ptr_, rhs.data_.ptr_);
        }

        gsl_is_delete_access : not_null() gsl_is_delete;
        // prevent compilation when initialized with a nullptr or literal 0:
#if gsl_HAVE(NULLPTR)
        not_null(std::nullptr_t) gsl_is_delete;
        not_null &operator=(std::nullptr_t) gsl_is_delete;
#else
        not_null(int) gsl_is_delete;
        not_null &operator=(int) gsl_is_delete;
#endif

        // unwanted operators...pointers only point to single objects!
        not_null &operator++() gsl_is_delete;
        not_null &operator--() gsl_is_delete;
        not_null operator++(int) gsl_is_delete;
        not_null operator--(int) gsl_is_delete;
        not_null &operator+(size_t) gsl_is_delete;
        not_null &operator+=(size_t) gsl_is_delete;
        not_null &operator-(size_t) gsl_is_delete;
        not_null &operator-=(size_t) gsl_is_delete;
        not_null &operator+=(std::ptrdiff_t) gsl_is_delete;
        not_null &operator-=(std::ptrdiff_t) gsl_is_delete;
        void operator[](std::ptrdiff_t) const gsl_is_delete;
    };
#endif  // gsl_CONFIG_DEFAULTS_VERSION >= 1
#if gsl_HAVE(DEDUCTION_GUIDES)
    template <class U>
    not_null(U) -> not_null<U>;
    template <class U>
    not_null(not_null<U>) -> not_null<U>;
#endif

#if gsl_HAVE(NULLPTR)
    void make_not_null(std::nullptr_t) gsl_is_delete;
#endif  // gsl_HAVE( NULLPTR )
#if gsl_HAVE(MOVE_FORWARD)
    template <class U>
    gsl_NODISCARD gsl_api gsl_constexpr14 not_null<U>
    make_not_null(U u)
    {
        return not_null<U>(std::move(u));
    }
    template <class U>
    gsl_NODISCARD gsl_api gsl_constexpr14 not_null<U>
    make_not_null(not_null<U> u)
    {
        return std::move(u);
    }
#else   // a.k.a. ! gsl_HAVE( MOVE_FORWARD )
    template <class U>
    gsl_NODISCARD gsl_api not_null<U>
    make_not_null(U const &u)
    {
        return not_null<U>(u);
    }
    template <class U>
    gsl_NODISCARD gsl_api not_null<U>
    make_not_null(not_null<U> const &u)
    {
        return u;
    }
#endif  // gsl_HAVE( MOVE_FORWARD )

    namespace detail
    {
    template <class T>
    struct as_nullable_helper
    {
        typedef T type;
    };
    template <class T>
    struct as_nullable_helper<not_null<T>>
    {
    };

    template <class T>
    struct not_null_accessor
    {
#if gsl_HAVE(MOVE_FORWARD)
        static gsl_api T get(not_null<T> &&p) gsl_noexcept
        {
            return std::move(p.data_.ptr_);
        }
#endif
        static gsl_api T const &get(not_null<T> const &p) gsl_noexcept
        {
            return p.data_.ptr_;
        }
    };

    namespace no_adl
    {
#if gsl_HAVE(MOVE_FORWARD)
    template <class T>
    gsl_NODISCARD gsl_api gsl_constexpr auto as_nullable(T &&p)
        gsl_noexcept_if(std::is_nothrow_move_constructible<T>::value)
            -> typename detail::as_nullable_helper<typename std20::remove_cvref<T>::type>::type
    {
        return std::move(p);
    }
    template <class T>
    gsl_NODISCARD gsl_api gsl_constexpr14 T as_nullable(not_null<T> &&p)
    {
        T result = detail::not_null_accessor<T>::get(std::move(p));
        gsl_Expects(result != gsl_nullptr);
        return result;
    }
#else   // ! gsl_HAVE( MOVE_FORWARD )
    template <class T>
    gsl_NODISCARD gsl_api gsl_constexpr T const &as_nullable(T const &p) gsl_noexcept
    {
        return p;
    }
#endif  // gsl_HAVE( MOVE_FORWARD )
    template <class T>
    gsl_NODISCARD gsl_api gsl_constexpr14 T const &as_nullable(not_null<T> const &p)
    {
        T const &result = detail::not_null_accessor<T>::get(p);
        gsl_Expects(result != gsl_nullptr);
        return result;
    }
    template <class T>
    gsl_NODISCARD gsl_api gsl_constexpr T *as_nullable(not_null<T *> p) gsl_noexcept
    {
        return detail::not_null_accessor<T *>::get(p);
    }

    }  // namespace no_adl
    }  // namespace detail

    using namespace detail::no_adl;

    // not_null with implicit constructor, allowing copy-initialization:

    template <class T>
    class not_null_ic : public not_null<T>
    {
    public:
        template <class U
                gsl_ENABLE_IF_((std::is_constructible<T, U>::value))>
        gsl_api gsl_constexpr14
#if gsl_HAVE(MOVE_FORWARD)
        not_null_ic(U &&u)
            : not_null<T>(std::forward<U>(u))
#else   // ! gsl_HAVE( MOVE_FORWARD )
        not_null_ic(U const &u)
            : not_null<T>(u)
#endif  // gsl_HAVE( MOVE_FORWARD )
        {
        }
    };

    // more not_null unwanted operators

    template <class T, class U>
    std::ptrdiff_t operator-(not_null<T> const &, not_null<U> const &) gsl_is_delete;

    template <class T>
    not_null<T> operator-(not_null<T> const &, std::ptrdiff_t) gsl_is_delete;

    template <class T>
    not_null<T> operator+(not_null<T> const &, std::ptrdiff_t) gsl_is_delete;

    template <class T>
    not_null<T> operator+(std::ptrdiff_t, not_null<T> const &) gsl_is_delete;

    // not_null comparisons

#if gsl_HAVE(NULLPTR) && gsl_HAVE(IS_DELETE)
    template <class T>
    gsl_constexpr bool
    operator==(not_null<T> const &, std::nullptr_t) = delete;
    template <class T>
    gsl_constexpr bool
    operator==(std::nullptr_t, not_null<T> const &) = delete;
    template <class T>
    gsl_constexpr bool
    operator!=(not_null<T> const &, std::nullptr_t) = delete;
    template <class T>
    gsl_constexpr bool
    operator!=(std::nullptr_t, not_null<T> const &) = delete;
#endif  // gsl_HAVE( NULLPTR ) && gsl_HAVE( IS_DELETE )

    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator==(not_null<T> const &l, not_null<U> const &r)
        gsl_RETURN_DECLTYPE_(l.operator->() == r.operator->())
    {
        return l.operator->() == r.operator->();
    }
    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator==(not_null<T> const &l, U const &r)
        gsl_RETURN_DECLTYPE_(l.operator->() == r)
    {
        return l.operator->() == r;
    }
    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator==(T const &l, not_null<U> const &r)
        gsl_RETURN_DECLTYPE_(l == r.operator->())
    {
        return l == r.operator->();
    }

    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator<(not_null<T> const &l, not_null<U> const &r)
        gsl_RETURN_DECLTYPE_(l.operator->() < r.operator->())
    {
        return l.operator->() < r.operator->();
    }
    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator<(not_null<T> const &l, U const &r)
        gsl_RETURN_DECLTYPE_(l.operator->() < r)
    {
        return l.operator->() < r;
    }
    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator<(T const &l, not_null<U> const &r)
        gsl_RETURN_DECLTYPE_(l < r.operator->())
    {
        return l < r.operator->();
    }

    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator!=(not_null<T> const &l, not_null<U> const &r)
        gsl_RETURN_DECLTYPE_(!(l == r))
    {
        return !(l == r);
    }
    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator!=(not_null<T> const &l, U const &r)
        gsl_RETURN_DECLTYPE_(!(l == r))
    {
        return !(l == r);
    }
    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator!=(T const &l, not_null<U> const &r)
        gsl_RETURN_DECLTYPE_(!(l == r))
    {
        return !(l == r);
    }

    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator<=(not_null<T> const &l, not_null<U> const &r)
        gsl_RETURN_DECLTYPE_(!(r < l))
    {
        return !(r < l);
    }
    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator<=(not_null<T> const &l, U const &r)
        gsl_RETURN_DECLTYPE_(!(r < l))
    {
        return !(r < l);
    }
    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator<=(T const &l, not_null<U> const &r)
        gsl_RETURN_DECLTYPE_(!(r < l))
    {
        return !(r < l);
    }

    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator>(not_null<T> const &l, not_null<U> const &r)
        gsl_RETURN_DECLTYPE_(r < l)
    {
        return r < l;
    }
    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator>(not_null<T> const &l, U const &r)
        gsl_RETURN_DECLTYPE_(r < l)
    {
        return r < l;
    }
    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator>(T const &l, not_null<U> const &r)
        gsl_RETURN_DECLTYPE_(r < l)
    {
        return r < l;
    }

    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator>=(not_null<T> const &l, not_null<U> const &r)
        gsl_RETURN_DECLTYPE_(!(l < r))
    {
        return !(l < r);
    }
    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator>=(not_null<T> const &l, U const &r)
        gsl_RETURN_DECLTYPE_(!(l < r))
    {
        return !(l < r);
    }
    template <class T, class U>
    gsl_NODISCARD inline gsl_api gsl_constexpr gsl_TRAILING_RETURN_TYPE_(bool)
    operator>=(T const &l, not_null<U> const &r)
        gsl_RETURN_DECLTYPE_(!(l < r))
    {
        return !(l < r);
    }

    // print not_null

    template <class CharType, class Traits, class T>
    std::basic_ostream<CharType, Traits> &operator<<(std::basic_ostream<CharType, Traits> &os, not_null<T> const &p)
    {
        return os << p.operator->();
    }


//
// Byte-specific type.
//
#if gsl_HAVE(ENUM_CLASS_CONSTRUCTION_FROM_UNDERLYING_TYPE)
    enum class gsl_may_alias byte : unsigned char
    {
    };
#else
    struct gsl_may_alias byte
    {
        typedef unsigned char type;
        type v;
    };
#endif

    template <class T>
    gsl_NODISCARD gsl_api inline gsl_constexpr byte
    to_byte(T v) gsl_noexcept
    {
#if gsl_HAVE(ENUM_CLASS_CONSTRUCTION_FROM_UNDERLYING_TYPE)
        return static_cast<byte>(v);
#elif gsl_HAVE(CONSTEXPR_11)
        return {static_cast<typename byte::type>(v)};
#else
        byte b = {static_cast<typename byte::type>(v)};
        return b;
#endif
    }

    template <class IntegerType gsl_ENABLE_IF_((std::is_integral<IntegerType>::value))>
    gsl_NODISCARD gsl_api inline gsl_constexpr IntegerType
    to_integer(byte b) gsl_noexcept
    {
#if gsl_HAVE(ENUM_CLASS_CONSTRUCTION_FROM_UNDERLYING_TYPE)
        return static_cast<typename std::underlying_type<byte>::type>(b);
#else
        return b.v;
#endif
    }

    gsl_NODISCARD gsl_api inline gsl_constexpr unsigned char
    to_uchar(byte b) gsl_noexcept
    {
        return to_integer<unsigned char>(b);
    }

    gsl_NODISCARD gsl_api inline gsl_constexpr unsigned char
    to_uchar(int i) gsl_noexcept
    {
        return static_cast<unsigned char>(i);
    }

    template <class IntegerType gsl_ENABLE_IF_((std::is_integral<IntegerType>::value))>
    gsl_api inline gsl_constexpr14 byte &
    operator<<=(byte &b, IntegerType shift) gsl_noexcept
    {
#if gsl_HAVE(ENUM_CLASS_CONSTRUCTION_FROM_UNDERLYING_TYPE)
        return b = ::gsl::to_byte(::gsl::to_uchar(b) << shift);
#else
        b.v = ::gsl::to_uchar(b.v << shift);
        return b;
#endif
    }

    template <class IntegerType gsl_ENABLE_IF_((std::is_integral<IntegerType>::value))>
    gsl_NODISCARD gsl_api inline gsl_constexpr byte
    operator<<(byte b, IntegerType shift) gsl_noexcept
    {
        return ::gsl::to_byte(::gsl::to_uchar(b) << shift);
    }

    template <class IntegerType gsl_ENABLE_IF_((std::is_integral<IntegerType>::value))>
    gsl_NODISCARD gsl_api inline gsl_constexpr14 byte &
    operator>>=(byte &b, IntegerType shift) gsl_noexcept
    {
#if gsl_HAVE(ENUM_CLASS_CONSTRUCTION_FROM_UNDERLYING_TYPE)
        return b = ::gsl::to_byte(::gsl::to_uchar(b) >> shift);
#else
        b.v = ::gsl::to_uchar(b.v >> shift);
        return b;
#endif
    }

    template <class IntegerType gsl_ENABLE_IF_((std::is_integral<IntegerType>::value))>
    gsl_NODISCARD gsl_api inline gsl_constexpr byte
    operator>>(byte b, IntegerType shift) gsl_noexcept
    {
        return ::gsl::to_byte(::gsl::to_uchar(b) >> shift);
    }

#if gsl_HAVE(ENUM_CLASS_CONSTRUCTION_FROM_UNDERLYING_TYPE)
    gsl_DEFINE_ENUM_BITMASK_OPERATORS(byte)
        gsl_DEFINE_ENUM_RELATIONAL_OPERATORS(byte)
#else   // a.k.a. !gsl_HAVE( ENUM_CLASS_CONSTRUCTION_FROM_UNDERLYING_TYPE )
    gsl_api inline gsl_constexpr bool operator==(byte l, byte r) gsl_noexcept
    {
        return l.v == r.v;
    }

    gsl_api inline gsl_constexpr bool operator!=(byte l, byte r) gsl_noexcept
    {
        return !(l == r);
    }

    gsl_api inline gsl_constexpr bool operator<(byte l, byte r) gsl_noexcept
    {
        return l.v < r.v;
    }

    gsl_api inline gsl_constexpr bool operator<=(byte l, byte r) gsl_noexcept
    {
        return !(r < l);
    }

    gsl_api inline gsl_constexpr bool operator>(byte l, byte r) gsl_noexcept
    {
        return (r < l);
    }

    gsl_api inline gsl_constexpr bool operator>=(byte l, byte r) gsl_noexcept
    {
        return !(l < r);
    }

    gsl_api inline gsl_constexpr14 byte &operator|=(byte &l, byte r) gsl_noexcept
    {
        l.v |= r.v;
        return l;
    }

    gsl_api inline gsl_constexpr byte operator|(byte l, byte r) gsl_noexcept
    {
        return ::gsl::to_byte(l.v | r.v);
    }

    gsl_api inline gsl_constexpr14 byte &operator&=(byte &l, byte r) gsl_noexcept
    {
        l.v &= r.v;
        return l;
    }

    gsl_api inline gsl_constexpr byte operator&(byte l, byte r) gsl_noexcept
    {
        return ::gsl::to_byte(l.v & r.v);
    }

    gsl_api inline gsl_constexpr14 byte &operator^=(byte &l, byte r) gsl_noexcept
    {
        l.v ^= r.v;
        return l;
    }

    gsl_api inline gsl_constexpr byte operator^(byte l, byte r) gsl_noexcept
    {
        return ::gsl::to_byte(l.v ^ r.v);
    }

    gsl_api inline gsl_constexpr byte operator~(byte b) gsl_noexcept
    {
        return ::gsl::to_byte(~b.v);
    }
#endif  // gsl_HAVE( ENUM_CLASS_CONSTRUCTION_FROM_UNDERLYING_TYPE )

#if gsl_FEATURE_TO_STD(WITH_CONTAINER)

        // Tag to select span constructor taking a container:

        struct with_container_t
    {
        gsl_constexpr with_container_t() gsl_noexcept {}
    };
    const gsl_constexpr with_container_t with_container;  // TODO: this can lead to ODR violations because the symbol will be defined in multiple translation units

#endif

    namespace detail
    {
    template <class T>
    gsl_api gsl_constexpr14 T *endptr(T *data, gsl_CONFIG_SPAN_INDEX_TYPE size)
    {
        // Be sure to run the check before doing pointer arithmetics, which would be UB for `nullptr` and non-0 integers.
        gsl_Expects(size == 0 || data != gsl_nullptr);
        return data + size;
    }

    }  // namespace detail

    //
    // span<> - A 1D view of contiguous T's, replace (*,len).
    //
    template <class T>
    class span
    {
        template <class U>
        friend class span;

    public:
        typedef gsl_CONFIG_SPAN_INDEX_TYPE index_type;

        typedef T element_type;
        typedef typename std11::remove_cv<T>::type value_type;

        typedef T &reference;
        typedef T *pointer;
        typedef T const *const_pointer;
        typedef T const &const_reference;

        typedef pointer iterator;
        typedef const_pointer const_iterator;

        typedef std::reverse_iterator<iterator> reverse_iterator;
        typedef std::reverse_iterator<const_iterator> const_reverse_iterator;

        typedef gsl_CONFIG_SPAN_INDEX_TYPE size_type;
        typedef std::ptrdiff_t difference_type;

        // 26.7.3.2 Constructors, copy, and assignment [span.cons]

        gsl_api gsl_constexpr span() gsl_noexcept
            : first_(gsl_nullptr),
              last_(gsl_nullptr)
        {
        }

#if !gsl_DEPRECATE_TO_LEVEL(5)

#if gsl_HAVE(NULLPTR)
        gsl_api gsl_constexpr14 span(std::nullptr_t, index_type size_in)
            : first_(nullptr), last_(nullptr)
        {
            gsl_Expects(size_in == 0);
        }
#endif

#if gsl_HAVE(IS_DELETE)
        gsl_DEPRECATED
            gsl_api gsl_constexpr
            span(reference data_in)
            : span(&data_in, 1)
        {
        }

        gsl_api gsl_constexpr span(element_type &&) = delete;
#endif

#endif  // deprecate

        gsl_api gsl_constexpr14 span(pointer data_in, index_type size_in)
            : first_(data_in), last_(detail::endptr(data_in, size_in))
        {
        }

        gsl_api gsl_constexpr14 span(pointer first_in, pointer last_in)
            : first_(first_in), last_(last_in)
        {
            gsl_Expects(first_in <= last_in);
        }

#if !gsl_DEPRECATE_TO_LEVEL(5)

        template <class U>
        gsl_api gsl_constexpr14 span(U *data_in, index_type size_in)
            : first_(data_in), last_(detail::endptr(data_in, size_in))
        {
        }

#endif  // deprecate

#if !gsl_DEPRECATE_TO_LEVEL(5)
        template <class U, size_t N>
        gsl_api gsl_constexpr span(U (&arr)[N]) gsl_noexcept
            : first_(gsl_ADDRESSOF(arr[0])),
              last_(gsl_ADDRESSOF(arr[0]) + N)
        {
        }
#else
        template <size_t N
                gsl_ENABLE_IF_((std::is_convertible<value_type (*)[], element_type (*)[]>::value))>
        gsl_api gsl_constexpr span(element_type (&arr)[N]) gsl_noexcept
            : first_(gsl_ADDRESSOF(arr[0])),
              last_(gsl_ADDRESSOF(arr[0]) + N)
        {
        }
#endif  // deprecate

#if gsl_HAVE(ARRAY)
#if !gsl_DEPRECATE_TO_LEVEL(5)

        template <class U, size_t N>
        gsl_api gsl_constexpr span(std::array<U, N> &arr)
            : first_(arr.data()), last_(arr.data() + N)
        {
        }

        template <class U, size_t N>
        gsl_api gsl_constexpr span(std::array<U, N> const &arr)
            : first_(arr.data()), last_(arr.data() + N)
        {
        }

#else

        template <size_t N
                gsl_ENABLE_IF_((std::is_convertible<value_type (*)[], element_type (*)[]>::value))>
        gsl_constexpr span(std::array<value_type, N> &arr)
            : first_(arr.data()), last_(arr.data() + N)
        {
        }

        template <size_t N
                gsl_ENABLE_IF_((std::is_convertible<value_type (*)[], element_type (*)[]>::value))>
        gsl_constexpr span(std::array<value_type, N> const &arr)
            : first_(arr.data()), last_(arr.data() + N)
        {
        }

#endif  // deprecate
#endif  // gsl_HAVE( ARRAY )

#if gsl_HAVE(CONSTRAINED_SPAN_CONTAINER_CTOR)
        template <class Container
                gsl_ENABLE_IF_((detail::is_compatible_container<Container, element_type>::value))>
        gsl_api gsl_constexpr span(Container &cont) gsl_noexcept
            : first_(std17::data(cont)),
              last_(std17::data(cont) + std17::size(cont))
        {
        }

        template <class Container
                gsl_ENABLE_IF_((
                    std::is_const<element_type>::value && detail::is_compatible_container<Container, element_type>::value))>
        gsl_api gsl_constexpr span(Container const &cont) gsl_noexcept
            : first_(std17::data(cont)),
              last_(std17::data(cont) + std17::size(cont))
        {
        }

#elif gsl_HAVE(UNCONSTRAINED_SPAN_CONTAINER_CTOR)

        template <class Container>
        gsl_constexpr span(Container &cont)
            : first_(cont.size() == 0 ? gsl_nullptr : gsl_ADDRESSOF(cont[0])), last_(cont.size() == 0 ? gsl_nullptr : gsl_ADDRESSOF(cont[0]) + cont.size())
        {
        }

        template <class Container>
        gsl_constexpr span(Container const &cont)
            : first_(cont.size() == 0 ? gsl_nullptr : gsl_ADDRESSOF(cont[0])), last_(cont.size() == 0 ? gsl_nullptr : gsl_ADDRESSOF(cont[0]) + cont.size())
        {
        }

#endif

#if gsl_FEATURE_TO_STD(WITH_CONTAINER)

        template <class Container>
        gsl_constexpr span(with_container_t, Container &cont) gsl_noexcept
            : first_(cont.size() == 0 ? gsl_nullptr : gsl_ADDRESSOF(cont[0])),
              last_(cont.size() == 0 ? gsl_nullptr : gsl_ADDRESSOF(cont[0]) + cont.size())
        {
        }

        template <class Container>
        gsl_constexpr span(with_container_t, Container const &cont) gsl_noexcept
            : first_(cont.size() == 0 ? gsl_nullptr : gsl_ADDRESSOF(cont[0])),
              last_(cont.size() == 0 ? gsl_nullptr : gsl_ADDRESSOF(cont[0]) + cont.size())
        {
        }

#endif

#if !gsl_DEPRECATE_TO_LEVEL(4)
        // constructor taking shared_ptr deprecated since 0.29.0

#if gsl_HAVE(SHARED_PTR)
        gsl_DEPRECATED
            gsl_constexpr
            span(shared_ptr<element_type> const &ptr)
            : first_(ptr.get()), last_(ptr.get() ? ptr.get() + 1 : gsl_nullptr)
        {
        }
#endif

        // constructors taking unique_ptr deprecated since 0.29.0

#if gsl_HAVE(UNIQUE_PTR)
#if gsl_HAVE(DEFAULT_FUNCTION_TEMPLATE_ARG)
        template <class ArrayElementType = typename std::add_pointer<element_type>::type>
#else
        template <class ArrayElementType>
#endif
        gsl_DEPRECATED
            gsl_constexpr
            span(unique_ptr<ArrayElementType> const &ptr, index_type count)
            : first_(ptr.get()), last_(ptr.get() + count)
        {
        }

        gsl_DEPRECATED
            gsl_constexpr
            span(unique_ptr<element_type> const &ptr)
            : first_(ptr.get()), last_(ptr.get() ? ptr.get() + 1 : gsl_nullptr)
        {
        }
#endif

#endif  // deprecate shared_ptr, unique_ptr

#if gsl_HAVE(IS_DEFAULT) && !gsl_BETWEEN(gsl_COMPILER_GNUC_VERSION, 430, 600)
        gsl_constexpr span(span &&) gsl_noexcept = default;
        gsl_constexpr span(span const &) = default;
#else
        gsl_api gsl_constexpr span(span const &other)
            : first_(other.begin()), last_(other.end())
        {
        }
#endif

#if gsl_HAVE(IS_DEFAULT)
        gsl_constexpr14 span &operator=(span &&) gsl_noexcept = default;
        gsl_constexpr14 span &operator=(span const &) gsl_noexcept = default;
#else
        gsl_constexpr14 span &operator=(span other) gsl_noexcept
        {
            first_ = other.first_;
            last_ = other.last_;
            return *this;
        }
#endif

        template <class U
                gsl_ENABLE_IF_((std::is_convertible<U (*)[], element_type (*)[]>::value))>
        gsl_api gsl_constexpr span(span<U> const &other)
            : first_(other.begin()), last_(other.end())
        {
        }

#if 0
    // Converting from other span ?
    template< class U > operator=();
#endif

        // 26.7.3.3 Subviews [span.sub]

        gsl_NODISCARD gsl_api gsl_constexpr14 span
        first(index_type count) const
        {
            gsl_Expects(std::size_t(count) <= std::size_t(this->size()));
            return span(this->data(), count);
        }

        gsl_NODISCARD gsl_api gsl_constexpr14 span
        last(index_type count) const
        {
            gsl_Expects(std::size_t(count) <= std::size_t(this->size()));
            return span(this->data() + this->size() - count, count);
        }

        gsl_NODISCARD gsl_api gsl_constexpr14 span
        subspan(index_type offset) const
        {
            gsl_Expects(std::size_t(offset) <= std::size_t(this->size()));
            return span(this->data() + offset, this->size() - offset);
        }

        gsl_NODISCARD gsl_api gsl_constexpr14 span
        subspan(index_type offset, index_type count) const
        {
            gsl_Expects(
                std::size_t(offset) <= std::size_t(this->size()) &&
                std::size_t(count) <= std::size_t(this->size() - offset));
            return span(this->data() + offset, count);
        }

        // 26.7.3.4 Observers [span.obs]

        gsl_NODISCARD gsl_api gsl_constexpr index_type
        size() const gsl_noexcept
        {
            return narrow_cast<index_type>(last_ - first_);
        }

        gsl_NODISCARD gsl_api gsl_constexpr std::ptrdiff_t
        ssize() const gsl_noexcept
        {
            return narrow_cast<std::ptrdiff_t>(last_ - first_);
        }

        gsl_NODISCARD gsl_api gsl_constexpr index_type
        size_bytes() const gsl_noexcept
        {
            return size() * narrow_cast<index_type>(sizeof(element_type));
        }

        gsl_NODISCARD gsl_api gsl_constexpr bool
        empty() const gsl_noexcept
        {
            return size() == 0;
        }

        // 26.7.3.5 Element access [span.elem]

        gsl_NODISCARD gsl_api gsl_constexpr14 reference
        operator[](index_type pos) const
        {
            gsl_Expects(pos < size());
            return first_[pos];
        }

#if !gsl_DEPRECATE_TO_LEVEL(6)
        gsl_DEPRECATED_MSG("use subscript indexing instead")
            gsl_api gsl_constexpr14 reference
            operator()(index_type pos) const
        {
            return (*this)[pos];
        }

        gsl_DEPRECATED_MSG("use subscript indexing instead")
            gsl_api gsl_constexpr14 reference
            at(index_type pos) const
        {
            return (*this)[pos];
        }
#endif  // deprecate

        gsl_NODISCARD gsl_api gsl_constexpr14 reference
        front() const
        {
            gsl_Expects(first_ != last_);
            return *first_;
        }

        gsl_NODISCARD gsl_api gsl_constexpr14 reference
        back() const
        {
            gsl_Expects(first_ != last_);
            return *(last_ - 1);
        }

        gsl_NODISCARD gsl_api gsl_constexpr pointer
        data() const gsl_noexcept
        {
            return first_;
        }

        // 26.7.3.6 Iterator support [span.iterators]

        gsl_NODISCARD gsl_api gsl_constexpr iterator
        begin() const gsl_noexcept
        {
            return iterator(first_);
        }

        gsl_NODISCARD gsl_api gsl_constexpr iterator
        end() const gsl_noexcept
        {
            return iterator(last_);
        }

        gsl_NODISCARD gsl_api gsl_constexpr const_iterator
        cbegin() const gsl_noexcept
        {
#if gsl_CPP11_OR_GREATER
            return {begin()};
#else
            return const_iterator(begin());
#endif
        }

        gsl_NODISCARD gsl_api gsl_constexpr const_iterator
        cend() const gsl_noexcept
        {
#if gsl_CPP11_OR_GREATER
            return {end()};
#else
            return const_iterator(end());
#endif
        }

        gsl_NODISCARD gsl_constexpr17 reverse_iterator
        rbegin() const gsl_noexcept
        {
            return reverse_iterator(end());
        }

        gsl_NODISCARD gsl_constexpr17 reverse_iterator
        rend() const gsl_noexcept
        {
            return reverse_iterator(begin());
        }

        gsl_NODISCARD gsl_constexpr17 const_reverse_iterator
        crbegin() const gsl_noexcept
        {
            return const_reverse_iterator(cend());
        }

        gsl_NODISCARD gsl_constexpr17 const_reverse_iterator
        crend() const gsl_noexcept
        {
            return const_reverse_iterator(cbegin());
        }

        gsl_constexpr14 void swap(span &other) gsl_noexcept
        {
            std::swap(first_, other.first_);
            std::swap(last_, other.last_);
        }

#if !gsl_DEPRECATE_TO_LEVEL(3)
        // member length() deprecated since 0.29.0

        gsl_DEPRECATED_MSG("use size() instead")
            gsl_api gsl_constexpr index_type length() const gsl_noexcept
        {
            return size();
        }

        // member length_bytes() deprecated since 0.29.0

        gsl_DEPRECATED_MSG("use size_bytes() instead")
            gsl_api gsl_constexpr index_type length_bytes() const gsl_noexcept
        {
            return size_bytes();
        }
#endif

#if !gsl_DEPRECATE_TO_LEVEL(2)
        // member as_bytes(), as_writeable_bytes deprecated since 0.17.0

        gsl_DEPRECATED_MSG("use free function gsl::as_bytes() instead")
            gsl_api span<const byte> as_bytes() const gsl_noexcept
        {
            return span<const byte>(reinterpret_cast<const byte *>(data()), size_bytes());  // NOLINT
        }

        gsl_DEPRECATED_MSG("use free function gsl::as_writable_bytes() instead")
            gsl_api span<byte> as_writeable_bytes() const gsl_noexcept
        {
            return span<byte>(reinterpret_cast<byte *>(data()), size_bytes());  // NOLINT
        }

#endif

        template <class U>
        gsl_NODISCARD gsl_api span<U>
        as_span() const
        {
            gsl_Expects((this->size_bytes() % sizeof(U)) == 0);
            return span<U>(reinterpret_cast<U *>(this->data()), this->size_bytes() / sizeof(U));  // NOLINT
        }

    private:
        pointer first_;
        pointer last_;
    };

    // class template argument deduction guides:

#if gsl_HAVE(DEDUCTION_GUIDES)  // gsl_CPP17_OR_GREATER

    template <class T, size_t N>
    span(T(&)[N]) -> span<T /*, N*/>;

    template <class T, size_t N>
    span(std::array<T, N> &) -> span<T /*, N*/>;

    template <class T, size_t N>
    span(std::array<T, N> const &) -> span<const T /*, N*/>;

    template <class Container>
    span(Container &) -> span<typename Container::value_type>;

    template <class Container>
    span(Container const &) -> span<const typename Container::value_type>;

#endif  // gsl_HAVE( DEDUCTION_GUIDES )

    // 26.7.3.7 Comparison operators [span.comparison]

#if gsl_CONFIG(ALLOWS_SPAN_COMPARISON)
#if gsl_CONFIG(ALLOWS_NONSTRICT_SPAN_COMPARISON)

    template <class T, class U>
    gsl_SUPPRESS_MSGSL_WARNING(stl .1)
        gsl_NODISCARD inline gsl_constexpr bool
        operator==(span<T> const &l, span<U> const &r)
    {
        return l.size() == r.size() && (l.begin() == r.begin() || std98::equal(l.begin(), l.end(), r.begin()));
    }

    template <class T, class U>
    gsl_SUPPRESS_MSGSL_WARNING(stl .1)
        gsl_NODISCARD inline gsl_constexpr bool
        operator<(span<T> const &l, span<U> const &r)
    {
        return std98::lexicographical_compare(l.begin(), l.end(), r.begin(), r.end());
    }

#else   // a.k.a. !gsl_CONFIG( ALLOWS_NONSTRICT_SPAN_COMPARISON )

    template <class T>
    gsl_SUPPRESS_MSGSL_WARNING(stl .1)
        gsl_NODISCARD inline gsl_constexpr bool
        operator==(span<T> const &l, span<T> const &r)
    {
        return l.size() == r.size() && (l.begin() == r.begin() || std98::equal(l.begin(), l.end(), r.begin()));
    }

    template <class T>
    gsl_SUPPRESS_MSGSL_WARNING(stl .1)
        gsl_NODISCARD inline gsl_constexpr bool
        operator<(span<T> const &l, span<T> const &r)
    {
        return std98::lexicographical_compare(l.begin(), l.end(), r.begin(), r.end());
    }
#endif  // gsl_CONFIG( ALLOWS_NONSTRICT_SPAN_COMPARISON )

    template <class T, class U>
    gsl_NODISCARD inline gsl_constexpr bool operator!=(span<T> const &l, span<U> const &r)
    {
        return !(l == r);
    }

    template <class T, class U>
    gsl_NODISCARD inline gsl_constexpr bool operator<=(span<T> const &l, span<U> const &r)
    {
        return !(r < l);
    }

    template <class T, class U>
    gsl_NODISCARD inline gsl_constexpr bool operator>(span<T> const &l, span<U> const &r)
    {
        return (r < l);
    }

    template <class T, class U>
    gsl_NODISCARD inline gsl_constexpr bool operator>=(span<T> const &l, span<U> const &r)
    {
        return !(l < r);
    }
#endif  // gsl_CONFIG( ALLOWS_SPAN_COMPARISON )

    // span algorithms

    template <class T>
    gsl_NODISCARD gsl_api inline gsl_constexpr std::size_t
    size(span<T> const &spn)
    {
        return static_cast<std::size_t>(spn.size());
    }

    template <class T>
    gsl_NODISCARD gsl_api inline gsl_constexpr std::ptrdiff_t
    ssize(span<T> const &spn)
    {
        return spn.ssize();
    }

    namespace detail
    {
    template <class II, class N, class OI>
    gsl_api gsl_constexpr14 inline OI copy_n(II first, N count, OI result)
    {
        if (count > 0)
            {
                *result++ = *first;
                for (N i = 1; i < count; ++i)
                    {
                        *result++ = *++first;
                    }
            }
        return result;
    }
    }  // namespace detail

    template <class T, class U>
    gsl_api gsl_constexpr14 inline void copy(span<T> src, span<U> dest)
    {
#if gsl_CPP14_OR_GREATER  // gsl_HAVE( TYPE_TRAITS ) (circumvent Travis clang 3.4)
        static_assert(std::is_assignable<U &, T const &>::value, "Cannot assign elements of source span to elements of destination span");
#endif
        gsl_Expects(dest.size() >= src.size());
        detail::copy_n(src.data(), src.size(), dest.data());
    }

    // span creator functions (see ctors)

    template <class T>
    gsl_NODISCARD gsl_api inline span<const byte>
    as_bytes(span<T> spn) gsl_noexcept
    {
        return span<const byte>(reinterpret_cast<const byte *>(spn.data()), spn.size_bytes());  // NOLINT
    }

    template <class T>
    gsl_NODISCARD gsl_api inline span<byte>
    as_writable_bytes(span<T> spn) gsl_noexcept
    {
        return span<byte>(reinterpret_cast<byte *>(spn.data()), spn.size_bytes());  // NOLINT
    }

#if !gsl_DEPRECATE_TO_LEVEL(6)
    template <class T>
    gsl_DEPRECATED_MSG("use as_writable_bytes() (different spelling) instead")
        gsl_api inline span<byte>
        as_writeable_bytes(span<T> spn) gsl_noexcept
    {
        return span<byte>(reinterpret_cast<byte *>(spn.data()), spn.size_bytes());  // NOLINT
    }
#endif  // deprecate

#if gsl_FEATURE_TO_STD(MAKE_SPAN)

    template <class T>
    gsl_NODISCARD gsl_api inline gsl_constexpr span<T>
    make_span(T * ptr, typename span<T>::index_type count)
    {
        return span<T>(ptr, count);
    }

    template <class T>
    gsl_NODISCARD gsl_api inline gsl_constexpr span<T>
        make_span(T * first, T * last)
    {
        return span<T>(first, last);
    }

    template <class T, size_t N>
    gsl_NODISCARD inline gsl_constexpr span<T>
        make_span(T(&arr)[N])
    {
        return span<T>(gsl_ADDRESSOF(arr[0]), N);
    }

#if gsl_HAVE(ARRAY)

    template <class T, size_t N>
    gsl_NODISCARD inline gsl_constexpr span<T>
        make_span(std::array<T, N> & arr)
    {
        return span<T>(arr);
    }

    template <class T, size_t N>
    gsl_NODISCARD inline gsl_constexpr span<const T>
    make_span(std::array<T, N> const &arr)
    {
        return span<const T>(arr);
    }
#endif

#if gsl_HAVE(CONSTRAINED_SPAN_CONTAINER_CTOR) && gsl_HAVE(AUTO)

    template <class Container, class EP = decltype(std17::data(std::declval<Container &>()))>
    gsl_NODISCARD inline gsl_constexpr auto
        make_span(Container & cont)
            ->span<typename std::remove_pointer<EP>::type>
    {
        return span<typename std::remove_pointer<EP>::type>(cont);
    }

    template <class Container, class EP = decltype(std17::data(std::declval<Container &>()))>
    gsl_NODISCARD inline gsl_constexpr auto
    make_span(Container const &cont)
        ->span<const typename std::remove_pointer<EP>::type>
    {
        return span<const typename std::remove_pointer<EP>::type>(cont);
    }

#else

    template <class T>
    inline span<T>
        make_span(std::vector<T> & cont)
    {
        return span<T>(with_container, cont);
    }

    template <class T>
    inline span<const T>
    make_span(std::vector<T> const &cont)
    {
        return span<const T>(with_container, cont);
    }
#endif

#if gsl_FEATURE_TO_STD(WITH_CONTAINER)

    template <class Container>
    gsl_NODISCARD inline gsl_constexpr span<typename Container::value_type>
        make_span(with_container_t, Container & cont) gsl_noexcept
    {
        return span<typename Container::value_type>(with_container, cont);
    }

    template <class Container>
    gsl_NODISCARD inline gsl_constexpr span<const typename Container::value_type>
    make_span(with_container_t, Container const &cont) gsl_noexcept
    {
        return span<const typename Container::value_type>(with_container, cont);
    }

#endif  // gsl_FEATURE_TO_STD( WITH_CONTAINER )

#if !gsl_DEPRECATE_TO_LEVEL(4)
    template <class Ptr>
    gsl_DEPRECATED inline span<typename Ptr::element_type>
        make_span(Ptr & ptr)
    {
        return span<typename Ptr::element_type>(ptr);
    }
#endif  // !gsl_DEPRECATE_TO_LEVEL( 4 )

    template <class Ptr>
    gsl_DEPRECATED inline span<typename Ptr::element_type>
    make_span(Ptr & ptr, typename span<typename Ptr::element_type>::index_type count)
    {
        return span<typename Ptr::element_type>(ptr, count);
    }

#endif  // gsl_FEATURE_TO_STD( MAKE_SPAN )

#if gsl_FEATURE_TO_STD(BYTE_SPAN)

    template <class T>
    gsl_NODISCARD gsl_api inline gsl_constexpr span<byte>
        byte_span(T & t) gsl_noexcept
    {
        return span<byte>(reinterpret_cast<byte *>(&t), sizeof(T));
    }

    template <class T>
    gsl_NODISCARD gsl_api inline gsl_constexpr span<const byte>
    byte_span(T const &t) gsl_noexcept
    {
        return span<const byte>(reinterpret_cast<byte const *>(&t), sizeof(T));
    }

#endif  // gsl_FEATURE_TO_STD( BYTE_SPAN )

    //
    // basic_string_span:
    //

    template <class T>
    class basic_string_span;

    namespace detail
    {
    template <class T>
    struct is_basic_string_span_oracle : std11::false_type
    {
    };

    template <class T>
    struct is_basic_string_span_oracle<basic_string_span<T>> : std11::true_type
    {
    };

    template <class T>
    struct is_basic_string_span : is_basic_string_span_oracle<typename std11::remove_cv<T>::type>
    {
    };

    template <class T>
    gsl_api inline gsl_constexpr14 std::size_t string_length(T *ptr, std::size_t max)
    {
        if (ptr == gsl_nullptr || max <= 0)
            return 0;

        std::size_t len = 0;
        while (len < max && ptr[len])  // NOLINT
            ++len;

        return len;
    }

    }  // namespace detail

    //
    // basic_string_span<> - A view of contiguous characters, replace (*,len).
    //
    template <class T>
    class basic_string_span
    {
    public:
        typedef T element_type;
        typedef span<T> span_type;

        typedef typename span_type::size_type size_type;
        typedef typename span_type::index_type index_type;
        typedef typename span_type::difference_type difference_type;

        typedef typename span_type::pointer pointer;
        typedef typename span_type::reference reference;

        typedef typename span_type::iterator iterator;
        typedef typename span_type::const_iterator const_iterator;
        typedef typename span_type::reverse_iterator reverse_iterator;
        typedef typename span_type::const_reverse_iterator const_reverse_iterator;

        // construction:

#if gsl_HAVE(IS_DEFAULT)
        gsl_constexpr basic_string_span() gsl_noexcept = default;
#else
        gsl_api gsl_constexpr basic_string_span() gsl_noexcept
        {
        }
#endif

#if gsl_HAVE(NULLPTR)
        gsl_api gsl_constexpr basic_string_span(std::nullptr_t) gsl_noexcept
            : span_(nullptr, static_cast<index_type>(0))
        {
        }
#endif

#ifdef __CUDACC_RELAXED_CONSTEXPR__
        gsl_api
#endif  // __CUDACC_RELAXED_CONSTEXPR__
            gsl_constexpr
            basic_string_span(pointer ptr)
            : span_(remove_z(ptr, (std::numeric_limits<index_type>::max)()))
        {
        }

        gsl_api gsl_constexpr basic_string_span(pointer ptr, index_type count)
            : span_(ptr, count)
        {
        }

        gsl_api gsl_constexpr basic_string_span(pointer firstElem, pointer lastElem)
            : span_(firstElem, lastElem)
        {
        }

        template <std::size_t N>
        gsl_constexpr basic_string_span(element_type (&arr)[N])
            : span_(remove_z(gsl_ADDRESSOF(arr[0]), N))
        {
        }

#if gsl_HAVE(ARRAY)

        template <std::size_t N>
        gsl_constexpr basic_string_span(std::array<typename std11::remove_const<element_type>::type, N> &arr)
            : span_(remove_z(arr))
        {
        }

        template <std::size_t N>
        gsl_constexpr basic_string_span(std::array<typename std11::remove_const<element_type>::type, N> const &arr)
            : span_(remove_z(arr))
        {
        }

#endif

#if gsl_HAVE(CONSTRAINED_SPAN_CONTAINER_CTOR)

        // Exclude: array, [basic_string,] basic_string_span

        template <class Container
                gsl_ENABLE_IF_((
                    !detail::is_std_array<Container>::value && !detail::is_basic_string_span<Container>::value && std::is_convertible<typename Container::pointer, pointer>::value && std::is_convertible<typename Container::pointer, decltype(std::declval<Container>().data())>::value))>
        gsl_constexpr basic_string_span(Container &cont)
            : span_((cont))
        {
        }

        // Exclude: array, [basic_string,] basic_string_span

        template <class Container
                gsl_ENABLE_IF_((
                    !detail::is_std_array<Container>::value && !detail::is_basic_string_span<Container>::value && std::is_convertible<typename Container::pointer, pointer>::value && std::is_convertible<typename Container::pointer, decltype(std::declval<Container const &>().data())>::value))>
        gsl_constexpr basic_string_span(Container const &cont)
            : span_((cont))
        {
        }

#elif gsl_HAVE(UNCONSTRAINED_SPAN_CONTAINER_CTOR)

        template <class Container>
        gsl_constexpr basic_string_span(Container &cont)
            : span_(cont)
        {
        }

        template <class Container>
        gsl_constexpr basic_string_span(Container const &cont)
            : span_(cont)
        {
        }

#else

        template <class U>
        gsl_api gsl_constexpr basic_string_span(span<U> const &rhs)
            : span_(rhs)
        {
        }

#endif

#if gsl_FEATURE_TO_STD(WITH_CONTAINER)

        template <class Container>
        gsl_constexpr basic_string_span(with_container_t, Container &cont)
            : span_(with_container, cont)
        {
        }
#endif

#if gsl_HAVE(IS_DEFAULT)
#if gsl_BETWEEN(gsl_COMPILER_GNUC_VERSION, 440, 600)
        gsl_constexpr basic_string_span(basic_string_span const &) = default;

        gsl_constexpr basic_string_span(basic_string_span &&) = default;
#else
        gsl_constexpr basic_string_span(basic_string_span const &) gsl_noexcept = default;

        gsl_constexpr basic_string_span(basic_string_span &&) gsl_noexcept = default;
#endif
#endif

        template <class U
                gsl_ENABLE_IF_((std::is_convertible<typename basic_string_span<U>::pointer, pointer>::value))>
        gsl_api gsl_constexpr basic_string_span(basic_string_span<U> const &rhs)
            : span_(reinterpret_cast<pointer>(rhs.data()), rhs.length())  // NOLINT
        {
        }

#if gsl_STDLIB_CPP11_120
        template <class U
                gsl_ENABLE_IF_((std::is_convertible<typename basic_string_span<U>::pointer, pointer>::value))>
        gsl_api gsl_constexpr basic_string_span(basic_string_span<U> &&rhs)
            : span_(reinterpret_cast<pointer>(rhs.data()), rhs.length())  // NOLINT
        {
        }
#endif  // gsl_STDLIB_CPP11_120

        template <class CharTraits, class Allocator>
        gsl_constexpr basic_string_span(
            std::basic_string<typename std11::remove_const<element_type>::type, CharTraits, Allocator> &str)
            : span_(gsl_ADDRESSOF(str[0]), str.length())
        {
        }

        template <class CharTraits, class Allocator>
        gsl_constexpr basic_string_span(
            std::basic_string<typename std11::remove_const<element_type>::type, CharTraits, Allocator> const &str)
            : span_(gsl_ADDRESSOF(str[0]), str.length())
        {
        }

        // assignment:

#if gsl_HAVE(IS_DEFAULT)
        gsl_constexpr14 basic_string_span &operator=(basic_string_span const &) gsl_noexcept = default;

        gsl_constexpr14 basic_string_span &operator=(basic_string_span &&) gsl_noexcept = default;
#endif

        // sub span:

        /*gsl_api*/  // currently disabled due to an apparent NVCC bug
        gsl_NODISCARD gsl_constexpr14 basic_string_span
        first(index_type count) const
        {
            return span_.first(count);
        }

        /*gsl_api*/  // currently disabled due to an apparent NVCC bug
        gsl_NODISCARD gsl_constexpr14 basic_string_span
        last(index_type count) const
        {
            return span_.last(count);
        }

        /*gsl_api*/  // currently disabled due to an apparent NVCC bug
        gsl_NODISCARD gsl_constexpr14 basic_string_span
        subspan(index_type offset) const
        {
            return span_.subspan(offset);
        }

        /*gsl_api*/  // currently disabled due to an apparent NVCC bug
        gsl_NODISCARD gsl_constexpr14 basic_string_span
        subspan(index_type offset, index_type count) const
        {
            return span_.subspan(offset, count);
        }

        // observers:

        gsl_NODISCARD gsl_api gsl_constexpr index_type
        length() const gsl_noexcept
        {
            return span_.size();
        }

        gsl_NODISCARD gsl_api gsl_constexpr index_type
        size() const gsl_noexcept
        {
            return span_.size();
        }

        gsl_NODISCARD gsl_api gsl_constexpr index_type
        length_bytes() const gsl_noexcept
        {
            return span_.size_bytes();
        }

        gsl_NODISCARD gsl_api gsl_constexpr index_type
        size_bytes() const gsl_noexcept
        {
            return span_.size_bytes();
        }

        gsl_NODISCARD gsl_api gsl_constexpr bool
        empty() const gsl_noexcept
        {
            return size() == 0;
        }

        gsl_NODISCARD gsl_api gsl_constexpr14 reference
        operator[](index_type idx) const
        {
            return span_[idx];
        }

#if !gsl_DEPRECATE_TO_LEVEL(6)
        gsl_DEPRECATED_MSG("use subscript indexing instead")
            gsl_api gsl_constexpr14 reference
            operator()(index_type idx) const
        {
            return span_[idx];
        }
#endif  // deprecate

        gsl_NODISCARD gsl_api gsl_constexpr14 reference
        front() const
        {
            return span_.front();
        }

        gsl_NODISCARD gsl_api gsl_constexpr14 reference
        back() const
        {
            return span_.back();
        }

        gsl_NODISCARD gsl_api gsl_constexpr pointer
        data() const gsl_noexcept
        {
            return span_.data();
        }

        gsl_NODISCARD gsl_api gsl_constexpr iterator
        begin() const gsl_noexcept
        {
            return span_.begin();
        }

        gsl_NODISCARD gsl_api gsl_constexpr iterator
        end() const gsl_noexcept
        {
            return span_.end();
        }

        gsl_NODISCARD gsl_constexpr17 reverse_iterator
        rbegin() const gsl_noexcept
        {
            return span_.rbegin();
        }

        gsl_NODISCARD gsl_constexpr17 reverse_iterator
        rend() const gsl_noexcept
        {
            return span_.rend();
        }

        // const version not in p0123r2:

        gsl_NODISCARD gsl_api gsl_constexpr const_iterator
        cbegin() const gsl_noexcept
        {
            return span_.cbegin();
        }

        gsl_NODISCARD gsl_api gsl_constexpr const_iterator
        cend() const gsl_noexcept
        {
            return span_.cend();
        }

        gsl_NODISCARD gsl_constexpr17 const_reverse_iterator
        crbegin() const gsl_noexcept
        {
            return span_.crbegin();
        }

        gsl_NODISCARD gsl_constexpr17 const_reverse_iterator
        crend() const gsl_noexcept
        {
            return span_.crend();
        }

    private:
        gsl_api static gsl_constexpr14 span_type remove_z(pointer sz, std::size_t max)
        {
            return span_type(sz, detail::string_length(sz, max));
        }

#if gsl_HAVE(ARRAY)
        template <size_t N>
        gsl_NODISCARD static gsl_constexpr14 span_type
        remove_z(std::array<typename std11::remove_const<element_type>::type, N> &arr)
        {
            return remove_z(gsl_ADDRESSOF(arr[0]), narrow_cast<std::size_t>(N));
        }

        template <size_t N>
        gsl_NODISCARD static gsl_constexpr14 span_type
        remove_z(std::array<typename std11::remove_const<element_type>::type, N> const &arr)
        {
            return remove_z(gsl_ADDRESSOF(arr[0]), narrow_cast<std::size_t>(N));
        }
#endif

    private:
        span_type span_;
    };

    // basic_string_span comparison functions:

#if gsl_CONFIG(ALLOWS_NONSTRICT_SPAN_COMPARISON)

    template <class T, class U>
    gsl_SUPPRESS_MSGSL_WARNING(stl .1)
        gsl_NODISCARD inline gsl_constexpr14 bool
        operator==(basic_string_span<T> const &l, U const &u) gsl_noexcept
    {
        const basic_string_span<typename std11::add_const<T>::type> r(u);

        return l.size() == r.size() && std98::equal(l.begin(), l.end(), r.begin());
    }

    template <class T, class U>
    gsl_SUPPRESS_MSGSL_WARNING(stl .1)
        gsl_NODISCARD inline gsl_constexpr14 bool
        operator<(basic_string_span<T> const &l, U const &u) gsl_noexcept
    {
        const basic_string_span<typename std11::add_const<T>::type> r(u);

        return std98::lexicographical_compare(l.begin(), l.end(), r.begin(), r.end());
    }

#if gsl_HAVE(DEFAULT_FUNCTION_TEMPLATE_ARG)

    template <class T, class U
                           gsl_ENABLE_IF_((!detail::is_basic_string_span<U>::value))>
    gsl_SUPPRESS_MSGSL_WARNING(stl .1)
        gsl_NODISCARD inline gsl_constexpr14 bool
        operator==(U const &u, basic_string_span<T> const &r) gsl_noexcept
    {
        const basic_string_span<typename std11::add_const<T>::type> l(u);

        return l.size() == r.size() && std98::equal(l.begin(), l.end(), r.begin());
    }

    template <class T, class U
                           gsl_ENABLE_IF_((!detail::is_basic_string_span<U>::value))>
    gsl_SUPPRESS_MSGSL_WARNING(stl .1)
        gsl_NODISCARD inline gsl_constexpr14 bool
        operator<(U const &u, basic_string_span<T> const &r) gsl_noexcept
    {
        const basic_string_span<typename std11::add_const<T>::type> l(u);

        return std98::lexicographical_compare(l.begin(), l.end(), r.begin(), r.end());
    }
#endif

#else  //gsl_CONFIG( ALLOWS_NONSTRICT_SPAN_COMPARISON )

    template <class T>
    gsl_SUPPRESS_MSGSL_WARNING(stl .1)
        gsl_NODISCARD inline gsl_constexpr14 bool
        operator==(basic_string_span<T> const &l, basic_string_span<T> const &r) gsl_noexcept
    {
        return l.size() == r.size() && std98::equal(l.begin(), l.end(), r.begin());
    }

    template <class T>
    gsl_SUPPRESS_MSGSL_WARNING(stl .1)
        gsl_NODISCARD inline gsl_constexpr14 bool
        operator<(basic_string_span<T> const &l, basic_string_span<T> const &r) gsl_noexcept
    {
        return std98::lexicographical_compare(l.begin(), l.end(), r.begin(), r.end());
    }

#endif  // gsl_CONFIG( ALLOWS_NONSTRICT_SPAN_COMPARISON )

    template <class T, class U>
    gsl_NODISCARD inline gsl_constexpr14 bool
    operator!=(basic_string_span<T> const &l, U const &r) gsl_noexcept
    {
        return !(l == r);
    }

    template <class T, class U>
    gsl_NODISCARD inline gsl_constexpr14 bool
    operator<=(basic_string_span<T> const &l, U const &r) gsl_noexcept
    {
#if gsl_HAVE(DEFAULT_FUNCTION_TEMPLATE_ARG) || !gsl_CONFIG(ALLOWS_NONSTRICT_SPAN_COMPARISON)
        return !(r < l);
#else
        basic_string_span<typename std11::add_const<T>::type> rr(r);
        return !(rr < l);
#endif
    }

    template <class T, class U>
    gsl_NODISCARD inline gsl_constexpr14 bool
    operator>(basic_string_span<T> const &l, U const &r) gsl_noexcept
    {
#if gsl_HAVE(DEFAULT_FUNCTION_TEMPLATE_ARG) || !gsl_CONFIG(ALLOWS_NONSTRICT_SPAN_COMPARISON)
        return (r < l);
#else
        basic_string_span<typename std11::add_const<T>::type> rr(r);
        return (rr < l);
#endif
    }

    template <class T, class U>
    gsl_NODISCARD inline gsl_constexpr14 bool
    operator>=(basic_string_span<T> const &l, U const &r) gsl_noexcept
    {
        return !(l < r);
    }

#if gsl_HAVE(DEFAULT_FUNCTION_TEMPLATE_ARG)

    template <class T, class U
                           gsl_ENABLE_IF_((!detail::is_basic_string_span<U>::value))>
    gsl_NODISCARD inline gsl_constexpr14 bool
    operator!=(U const &l, basic_string_span<T> const &r) gsl_noexcept
    {
        return !(l == r);
    }

    template <class T, class U
                           gsl_ENABLE_IF_((!detail::is_basic_string_span<U>::value))>
    gsl_NODISCARD inline gsl_constexpr14 bool
    operator<=(U const &l, basic_string_span<T> const &r) gsl_noexcept
    {
        return !(r < l);
    }

    template <class T, class U
                           gsl_ENABLE_IF_((!detail::is_basic_string_span<U>::value))>
    gsl_NODISCARD inline gsl_constexpr14 bool
    operator>(U const &l, basic_string_span<T> const &r) gsl_noexcept
    {
        return (r < l);
    }

    template <class T, class U
                           gsl_ENABLE_IF_((!detail::is_basic_string_span<U>::value))>
    gsl_NODISCARD inline gsl_constexpr14 bool
    operator>=(U const &l, basic_string_span<T> const &r) gsl_noexcept
    {
        return !(l < r);
    }

#endif  // gsl_HAVE( DEFAULT_FUNCTION_TEMPLATE_ARG )

    // convert basic_string_span to byte span:

    template <class T>
    gsl_NODISCARD gsl_api inline span<const byte>
    as_bytes(basic_string_span<T> spn) gsl_noexcept
    {
        return span<const byte>(reinterpret_cast<const byte *>(spn.data()), spn.size_bytes());  // NOLINT
    }

    //
    // String types:
    //

    typedef char *zstring;
    typedef const char *czstring;

#if gsl_HAVE(WCHAR)
    typedef wchar_t *wzstring;
    typedef const wchar_t *cwzstring;
#endif

    typedef basic_string_span<char> string_span;
    typedef basic_string_span<char const> cstring_span;

#if gsl_HAVE(WCHAR)
    typedef basic_string_span<wchar_t> wstring_span;
    typedef basic_string_span<wchar_t const> cwstring_span;
#endif

    // to_string() allow (explicit) conversions from string_span to string

#if 0

template< class T >
inline std::basic_string< typename std::remove_const<T>::type > to_string( basic_string_span<T> spn )
{
     std::string( spn.data(), spn.length() );
}

#else

    gsl_NODISCARD inline std::string
    to_string(string_span const &spn)
    {
        return std::string(spn.data(), static_cast<std::size_t>(spn.length()));
    }

    gsl_NODISCARD inline std::string
    to_string(cstring_span const &spn)
    {
        return std::string(spn.data(), static_cast<std::size_t>(spn.length()));
    }

#if gsl_HAVE(WCHAR)

    gsl_NODISCARD inline std::wstring
    to_string(wstring_span const &spn)
    {
        return std::wstring(spn.data(), static_cast<std::size_t>(spn.length()));
    }

    gsl_NODISCARD inline std::wstring
    to_string(cwstring_span const &spn)
    {
        return std::wstring(spn.data(), static_cast<std::size_t>(spn.length()));
    }

#endif  // gsl_HAVE( WCHAR )
#endif  // to_string()

    //
    // Stream output for string_span types
    //

    namespace detail
    {
    template <class Stream>
    void write_padding(Stream &os, std::streamsize n)
    {
        for (std::streamsize i = 0; i < n; ++i)
            os.rdbuf()->sputc(os.fill());
    }

    template <class Stream, class Span>
    Stream &write_to_stream(Stream &os, Span const &spn)
    {
        typename Stream::sentry sentry(os);

        if (!os)
            return os;

        const std::streamsize length = gsl::narrow_failfast<std::streamsize>(spn.length());

        // Whether, and how, to pad
        const bool pad = (length < os.width());
        const bool left_pad = pad && (os.flags() & std::ios_base::adjustfield) == std::ios_base::right;

        if (left_pad)
            detail::write_padding(os, os.width() - length);

        // Write span characters
        os.rdbuf()->sputn(spn.begin(), length);

        if (pad && !left_pad)
            detail::write_padding(os, os.width() - length);

        // Reset output stream width
        os.width(0);

        return os;
    }

    }  // namespace detail

    template <typename Traits>
    std::basic_ostream<char, Traits> &operator<<(std::basic_ostream<char, Traits> &os, string_span const &spn)
    {
        return detail::write_to_stream(os, spn);
    }

    template <typename Traits>
    std::basic_ostream<char, Traits> &operator<<(std::basic_ostream<char, Traits> &os, cstring_span const &spn)
    {
        return detail::write_to_stream(os, spn);
    }

#if gsl_HAVE(WCHAR)

    template <typename Traits>
    std::basic_ostream<wchar_t, Traits> &operator<<(std::basic_ostream<wchar_t, Traits> &os, wstring_span const &spn)
    {
        return detail::write_to_stream(os, spn);
    }

    template <typename Traits>
    std::basic_ostream<wchar_t, Traits> &operator<<(std::basic_ostream<wchar_t, Traits> &os, cwstring_span const &spn)
    {
        return detail::write_to_stream(os, spn);
    }

#endif  // gsl_HAVE( WCHAR )

    //
    // ensure_sentinel()
    //
    // Provides a way to obtain a span from a contiguous sequence
    // that ends with a (non-inclusive) sentinel value.
    //
    // Will fail-fast if sentinel cannot be found before max elements are examined.
    //
    namespace detail
    {
    template <class T, class SizeType, const T Sentinel>
    gsl_constexpr14 static span<T> ensure_sentinel(T *seq, SizeType max = (std::numeric_limits<SizeType>::max)())
    {
        typedef T *pointer;

        gsl_SUPPRESS_MSVC_WARNING(26429, "f.23: symbol 'cur' is never tested for nullness, it can be marked as not_null")
            pointer cur = seq;

        while (static_cast<SizeType>(cur - seq) < max && *cur != Sentinel)
            ++cur;

        gsl_Expects(*cur == Sentinel);

        return span<T>(seq, gsl::narrow_cast<typename span<T>::index_type>(cur - seq));
    }
    }  // namespace detail

    //
    // ensure_z - creates a string_span for a czstring or cwzstring.
    // Will fail fast if a null-terminator cannot be found before
    // the limit of size_type.
    //

    template <class T>
    gsl_NODISCARD inline gsl_constexpr14 span<T>
    ensure_z(T *const &sz, size_t max = (std::numeric_limits<size_t>::max)())
    {
        return detail::ensure_sentinel<T, size_t, 0>(sz, max);
    }

    template <class T, size_t N>
    gsl_NODISCARD inline gsl_constexpr14 span<T>
        ensure_z(T(&sz)[N])
    {
        return ::gsl::ensure_z(gsl_ADDRESSOF(sz[0]), N);
    }

#if gsl_HAVE(TYPE_TRAITS)

    template <class Container>
    gsl_NODISCARD inline gsl_constexpr14 span<typename std::remove_pointer<typename Container::pointer>::type>
        ensure_z(Container & cont)
    {
        return ::gsl::ensure_z(cont.data(), cont.length());
    }
#endif

    //
    // basic_zstring_span<> - A view of contiguous null-terminated characters, replace (*,len).
    //

    template <typename T>
    class basic_zstring_span
    {
    public:
        typedef T element_type;
        typedef span<T> span_type;

        typedef typename span_type::index_type index_type;
        typedef typename span_type::difference_type difference_type;

        typedef element_type *czstring_type;
        typedef basic_string_span<element_type> string_span_type;

        gsl_api gsl_constexpr14 basic_zstring_span(span_type s)
            : span_(s)
        {
            // expects a zero-terminated span
            gsl_Expects(s.back() == '\0');
        }

#if gsl_HAVE(IS_DEFAULT)
        gsl_constexpr basic_zstring_span(basic_zstring_span const &) = default;
        gsl_constexpr basic_zstring_span(basic_zstring_span &&) = default;
        gsl_constexpr14 basic_zstring_span &operator=(basic_zstring_span const &) = default;
        gsl_constexpr14 basic_zstring_span &operator=(basic_zstring_span &&) = default;
#else
        gsl_api gsl_constexpr basic_zstring_span(basic_zstring_span const &other) : span_(other.span_)
        {
        }
        gsl_api gsl_constexpr basic_zstring_span &operator=(basic_zstring_span const &other)
        {
            span_ = other.span_;
            return *this;
        }
#endif

        gsl_NODISCARD gsl_api gsl_constexpr bool
        empty() const gsl_noexcept
        {
            return false;
        }

        gsl_NODISCARD gsl_api gsl_constexpr string_span_type
        as_string_span() const gsl_noexcept
        {
            return string_span_type(span_.data(), span_.size() - 1);
        }

        /*gsl_api*/  // currently disabled due to an apparent NVCC bug
        gsl_NODISCARD gsl_constexpr string_span_type
        ensure_z() const
        {
            return ::gsl::ensure_z(span_.data(), span_.size());
        }

        gsl_NODISCARD gsl_api gsl_constexpr czstring_type
        assume_z() const gsl_noexcept
        {
            return span_.data();
        }

    private:
        span_type span_;
    };

    //
    // zString types:
    //

    typedef basic_zstring_span<char> zstring_span;
    typedef basic_zstring_span<char const> czstring_span;

#if gsl_HAVE(WCHAR)
    typedef basic_zstring_span<wchar_t> wzstring_span;
    typedef basic_zstring_span<wchar_t const> cwzstring_span;
#endif

}  // namespace gsl

#if gsl_HAVE(HASH)

//
// std::hash specializations for GSL types
//

namespace gsl
{
namespace detail
{
//
// Helper struct for std::hash specializations
//

template <bool Condition>
struct conditionally_enabled_hash
{
};

// disabled as described in [unord.hash]
template <>
struct conditionally_enabled_hash<false>
{
    gsl_is_delete_access : conditionally_enabled_hash() gsl_is_delete;
    conditionally_enabled_hash(conditionally_enabled_hash const &) gsl_is_delete;
    conditionally_enabled_hash(conditionally_enabled_hash &&) gsl_is_delete;
    conditionally_enabled_hash &operator=(conditionally_enabled_hash const &) gsl_is_delete;
    conditionally_enabled_hash &operator=(conditionally_enabled_hash &&) gsl_is_delete;
};

}  // namespace detail

}  // namespace gsl

namespace std
{
template <class T>
struct hash<::gsl::not_null<T>> : public ::gsl::detail::conditionally_enabled_hash<is_default_constructible<hash<T>>::value>
{
public:
    gsl_NODISCARD gsl_constexpr std::size_t
    operator()(::gsl::not_null<T> const &v) const
    // hash function is not `noexcept` because `as_nullable()` has preconditions
    {
        return hash<T>()(::gsl::as_nullable(v));
    }
};
template <class T>
struct hash<::gsl::not_null<T *>>
{
public:
    gsl_NODISCARD gsl_constexpr std::size_t
    operator()(::gsl::not_null<T *> const &v) const gsl_noexcept
    {
        return hash<T *>()(::gsl::as_nullable(v));
    }
};

template <>
struct hash<::gsl::byte>
{
public:
    gsl_NODISCARD gsl_constexpr std::size_t operator()(::gsl::byte v) const gsl_noexcept
    {
        return ::gsl::to_integer<std::size_t>(v);
    }
};

}  // namespace std

#endif  // gsl_HAVE( HASH )

#if gsl_FEATURE(GSL_LITE_NAMESPACE)

// gsl_lite namespace:

// gsl-lite currently keeps all symbols in the namespace `gsl`. The `gsl_lite` namespace contains all the symbols in the
// `gsl` namespace, plus some extensions that are not specified in the Core Guidelines.
//
// Going forward, we want to support coexistence of gsl-lite with M-GSL, so we want to encourage using the `gsl_lite`
// namespace when consuming gsl-lite. Typical use in library code would be:
//
//     #include <gsl-lite/gsl-lite.hpp>  // instead of <gsl/gsl-lite.hpp>
//
//     namespace foo {
//         namespace gsl = ::gsl_lite;  // convenience alias
//         double mean( gsl::span<double const> elements )
//         {
//             gsl_Expects( ! elements.empty() );  // instead of Expects()
//             ...
//         }
//     } // namespace foo
//
// In a future version, the new <gsl-lite/gsl-lite.hpp> header will only define the `gsl_lite` namespace and no
// unprefixed `Expects()` and `Ensures()` macros to avoid collision with M-GSL. To ensure backward compatibility, the
// old header <gsl/gsl-lite.hpp> will keep defining the `gsl` namespace and the `Expects()` and `Ensures()` macros.

namespace gsl_lite
{
namespace std11 = ::gsl::std11;
namespace std14 = ::gsl::std14;
namespace std17 = ::gsl::std17;
namespace std20 = ::gsl::std20;

using namespace std11;
using namespace std14;
using namespace std17;
using namespace std20;

using namespace ::gsl::detail::no_adl;

#if gsl_HAVE(SHARED_PTR)
using std::make_shared;
using std::shared_ptr;
using std::unique_ptr;
#endif

using ::gsl::diff;
using ::gsl::dim;
using ::gsl::index;
using ::gsl::stride;

#if gsl_HAVE(ALIAS_TEMPLATE)
#if gsl_BETWEEN(gsl_COMPILER_MSVC_VERSION, 1, 141)  // VS 2015 and earlier have trouble with `using` for alias templates
template <class T
#if gsl_HAVE(TYPE_TRAITS)
    ,
    typename = typename std::enable_if<std::is_pointer<T>::value>::type
#endif
    >
using owner = T;
#else
using ::gsl::owner;
#endif
#endif

using ::gsl::fail_fast;

using ::gsl::finally;
#if gsl_FEATURE(EXPERIMENTAL_RETURN_GUARD)
using ::gsl::on_error;
using ::gsl::on_return;
#endif  // gsl_FEATURE( EXPERIMENTAL_RETURN_GUARD )

using ::gsl::narrow;
using ::gsl::narrow_cast;
using ::gsl::narrow_failfast;
using ::gsl::narrowing_error;

using ::gsl::at;

using ::gsl::make_not_null;
using ::gsl::not_null;
using ::gsl::not_null_ic;

using ::gsl::byte;

using ::gsl::to_byte;
using ::gsl::to_integer;
using ::gsl::to_string;
using ::gsl::to_uchar;

using ::gsl::with_container;
using ::gsl::with_container_t;

using ::gsl::as_bytes;
using ::gsl::as_writable_bytes;
using ::gsl::byte_span;
using ::gsl::copy;
using ::gsl::make_span;
using ::gsl::span;
#if !gsl_DEPRECATE_TO_LEVEL(6)
using ::gsl::as_writeable_bytes;
#endif

using ::gsl::basic_string_span;
using ::gsl::cstring_span;
using ::gsl::string_span;

using ::gsl::basic_zstring_span;
using ::gsl::czstring_span;
using ::gsl::zstring_span;

using ::gsl::czstring;
using ::gsl::zstring;

#if gsl_HAVE(WCHAR)
using ::gsl::cwzstring;
using ::gsl::wzstring;

using ::gsl::cwzstring_span;
using ::gsl::wzstring_span;
#endif  // gsl_HAVE( WCHAR )

using ::gsl::ensure_z;

}  // namespace gsl_lite

#endif  // gsl_FEATURE( GSL_LITE_NAMESPACE )

gsl_RESTORE_MSVC_WARNINGS()

// #undef internal macros
#undef gsl_STATIC_ASSERT_
#undef gsl_ENABLE_IF_
#undef gsl_TRAILING_RETURN_TYPE_
#undef gsl_RETURN_DECLTYPE_

#endif  // GSL_GSL_LITE_HPP_INCLUDED

    // end of file
