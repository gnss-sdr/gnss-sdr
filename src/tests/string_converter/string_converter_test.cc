
/**
 * Copyright notice
 */

/**
 * Author: Carlos Avilés, 2010. carlos.avilesr(at)googlemail.com
 */

/**
 * This class implements a Unit Tests for the class StringConverter.
 *
 */

#include <gtest/gtest.h>

#include "string_converter.h"

TEST(StringConverter, StringToBool) {

	StringConverter *converter = new StringConverter();

	bool conversion_result = converter->convert("false", true);

	EXPECT_EQ(false, conversion_result);

	delete converter;
}

TEST(StringConverter, StringToSizeT) {

	StringConverter *converter = new StringConverter();

	size_t conversion_result = converter->convert("8", 1);

	EXPECT_EQ(8, conversion_result);

	delete converter;
}

TEST(StringConverter, StringToBoolFail) {

	StringConverter *converter = new StringConverter();

	bool conversion_result = converter->convert("lse", true);

	EXPECT_EQ(true, conversion_result);

	delete converter;
}

TEST(StringConverter, StringToSizeTFail) {

	StringConverter *converter = new StringConverter();

	size_t conversion_result = converter->convert("false", 1);

	EXPECT_EQ(1, conversion_result);

	delete converter;
}