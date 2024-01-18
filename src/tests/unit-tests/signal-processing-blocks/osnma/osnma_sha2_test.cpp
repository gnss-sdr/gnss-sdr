//
// Created by cghio on 17.01.24.
//
#include "gtest/gtest.h"
#include "gnss_crypto.h"
//#include "std"
#ifndef GNSS_SDR_GNSS_CRYPTO_SHA2_TEST_H
#define GNSS_SDR_GNSS_CRYPTO_SHA2_TEST_H

class OsnmaCryptoTest : public :: testing ::Test{
};

TEST_F(OsnmaCryptoTest, basicTest)
{
    std::unique_ptr<Gnss_Crypto> d_crypto;

    auto str = "Hello World!";
    std::vector<uint8_t> input (str, str + strlen(str));

    auto expectedOutputStr = "86933b0b147ac4c010266b99004158fa17937db89a03dd7bb2ca5ef7f43c325a";
    std::vector<uint8_t> expectedOutput(expectedOutputStr, expectedOutputStr + strlen(expectedOutputStr));

    std::vector<uint8_t> computedOutput = d_crypto->computeSHA256(input);

    ASSERT_TRUE(computedOutput == expectedOutput);

}

#endif  // GNSS_SDR_GNSS_CRYPTO_SHA2_TEST_H
