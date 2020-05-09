/*!
 * \file string_converter_test.cc
 * \brief  This file implements unit tests for the StringConverter class.
 * \author Carlos Aviles, 2010. carlos.avilesr(at)googlemail.com
 *         Carles Fernandez-Prades, 2012. cfernandez(at)cttc.es
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * -------------------------------------------------------------------------
 */

#include "string_converter.h"


TEST(StringConverterTest, StringToBool)
{
    std::unique_ptr<StringConverter> converter(new StringConverter());
    bool conversion_result = converter->convert("false", true);
    bool expected_false = false;
    EXPECT_EQ(expected_false, conversion_result);
}


TEST(StringConverterTest, StringToSizeT)
{
    // Example using a raw pointer
    StringConverter* converter;
    converter = new StringConverter();
    size_t conversion_result = converter->convert("8", 1);
    unsigned int expected8 = 8;
    EXPECT_EQ(expected8, conversion_result);
    delete converter;
}


TEST(StringConverterTest, StringToBoolFail)
{
    std::unique_ptr<StringConverter> converter(new StringConverter());
    bool conversion_result = converter->convert("lse", true);
    bool expected_true = true;
    EXPECT_EQ(expected_true, conversion_result);
}


TEST(StringConverterTest, StringToSizeTFail)
{
    std::unique_ptr<StringConverter> converter(new StringConverter());
    size_t conversion_result = converter->convert("false", 1);
    unsigned int expected1 = 1;
    EXPECT_EQ(expected1, conversion_result);
}
