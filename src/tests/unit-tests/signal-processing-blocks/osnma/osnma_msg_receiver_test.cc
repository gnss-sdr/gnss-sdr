#include <gtest/gtest.h>
#include <bitset>
#include <filesystem>
#include <fstream>
#include <logging.h>
#include <osnma_msg_receiver.h>
#include <vector>


struct TestVector
{
    int svId;
    int numNavBits;
    std::vector<uint8_t> navBits;
};

// TODO - parametrize class for different configurations (config_1, config_2, etc.. potentially 5 or 6 more) an make sure wont affect current TEST_F
class OsnmaMsgReceiverTest : public ::testing::Test
{
protected:
    osnma_msg_receiver_sptr osnma;
    Galileo_Inav_Message galileo_message;
    uint8_t page_position_in_inav_subframe;
    bool flag_CRC_test;
    std::string page_even;
    OSNMA_msg osnma_msg{};
    std::array<int8_t, 15> nma_position_filled;
    uint32_t d_GST_SIS{}; // 16 AUG 2023 05 00 01
    uint32_t TOW{};
    uint32_t WN{};
    std::tm GST_START_EPOCH = {0, 0, 0, 22, 8 - 1, 1999 - 1900, 0}; // months start with 0 and years since 1900 in std::tm
    const uint32_t LEAP_SECONDS = 0;//13 + 5;
    void set_time(std::tm& input);
//    std::string log_name {"CONFIG1-2023-08-23-PKID1-OSNMA"};
    std::string log_name {"CONFIG2-2023-07-27-PKID2-MT2-OSNMA"};
    void initializeGoogleLog();

    void SetUp() override
    {
        flag_CRC_test = false; // TODO what for?
        page_even = "";

//        std::tm input_time = {0, 0, 5, 16, 8 - 1, 2023 - 1900, 0};
        std::tm input_time = {0, 0, 0, 27, 7 - 1, 2023 - 1900, 0};
        set_time(input_time);
//        std::string pemFilePath = "/home/cgm/CLionProjects/osnma/data/OSNMA_PublicKey_20230803105952_newPKID_1.pem";
//        std::string merkleFilePath = "/home/cgm/CLionProjects/osnma/data/OSNMA_MerkleTree_20230803105953_newPKID_1.xml";
        std::string pemFilePath = "/home/cgm/CLionProjects/osnma/data/OSNMA_PublicKey_20230720113300_newPKID_2.pem";
        std::string merkleFilePath = "/home/cgm/CLionProjects/osnma/data/OSNMA_MerkleTree_20230720113300_newPKID_2.xml";
        osnma = osnma_msg_receiver_make(pemFilePath, merkleFilePath);
    }

public:
    static std::vector<uint8_t> parseNavBits(const std::string& hex);
    static std::vector<TestVector> readTestVectorsFromFile(const std::string& filename);
    std::string bytes_to_str(const std::vector<uint8_t>& bytes);
    std::vector<uint8_t> extract_page_bytes(const TestVector& tv, const int byte_index, const int num_bytes);
};


TEST_F(OsnmaMsgReceiverTest, TeslaKeyVerification) {
    // Arrange
    // ----------
    osnma->d_tesla_key_verified = false;
    osnma->d_osnma_data.d_dsm_kroot_message.kroot = {0x5B, 0xF8, 0xC9, 0xCB, 0xFC, 0xF7, 0x04, 0x22, 0x08, 0x14, 0x75, 0xFD, 0x44, 0x5D, 0xF0, 0xFF}; // Kroot, TOW 345570 GST_0 - 30
    osnma->d_osnma_data.d_dsm_kroot_message.ks = 4; // TABLE 10 --> 128 bits
    osnma->d_osnma_data.d_dsm_kroot_message.alpha = 0x610BDF26D77B;
    // local_time_verification would do this operation. TODO - eliminate duplication.
    osnma->d_GST_SIS = (1248 & 0x00000FFF) << 20 | (345630 & 0x000FFFFF);
    osnma->d_GST_0 = ((1248  & 0x00000FFF) << 20 | (345600 & 0x000FFFFF)); // applicable time (GST_Kroot + 30)
    osnma->d_receiver_time =  osnma->d_GST_0 + 30 * std::floor((osnma->d_GST_SIS - osnma->d_GST_0) / 30); // Eq. 3 R.G.//345630;

    osnma->d_tesla_keys.insert((std::pair<uint32_t, std::vector<uint8_t>>(345600,{0xEF, 0xF9, 0x99, 0x04, 0x0E, 0x19, 0xB5, 0x70, 0x83, 0x50, 0x60, 0xBE, 0xBD, 0x23, 0xED, 0x92}))); // K1, not needed, just for reference.
    std::vector<uint8_t> key = {0x2D, 0xC3, 0xA3, 0xCD, 0xB1, 0x17, 0xFA, 0xAD, 0xB8, 0x3B, 0x5F, 0x0B, 0x6F, 0xEA, 0x88, 0xEB}; // K2
    uint32_t TOW = 345630;




    // Act
    // ----------
    bool result = osnma->verify_tesla_key(key, TOW);





    // Assert
    // ----------
    ASSERT_TRUE(result);

}

TEST_F(OsnmaMsgReceiverTest, OsnmaTestVectorsSimulation)
{
    initializeGoogleLog();
    // Arrange
    // ----------
//    std::vector<TestVector> testVectors = readTestVectorsFromFile("/home/cgm/CLionProjects/osnma/data/16_AUG_2023_GST_05_00_01.csv");
    std::vector<TestVector> testVectors = readTestVectorsFromFile("/home/cgm/CLionProjects/osnma/data/27_JUL_2023_GST_00_00_01.csv");
    if (testVectors.empty()){
            ASSERT_TRUE(false);
        }

    bool end_of_hex_stream{false};
    int offset_byte{0};
    int byte_index{0}; // index containing the last byte position of the hex stream that was retrieved. Takes advantage that all TVs have same size
    const int SIZE_PAGE_BYTES{240/8}; // total bytes of a page
    const int SIZE_SUBFRAME_PAGES{15}; // number of pages of a subframe
    const int SIZE_SUBFRAME_BYTES{SIZE_PAGE_BYTES*SIZE_SUBFRAME_PAGES}; // total bytes of a subframe
    const int DURATION_SUBFRAME{30}; // duration of a subframe, in seconds

    const int DUMMY_PAGE{63};
    bool flag_dummy_page{false};
    std::cout << "OsnmaTestVectorsSimulation:" << " d_GST_SIS= " << d_GST_SIS
    << ", TOW=" << TOW
    << ", WN=" << WN << std::endl;





    // Act
    // ----------

    // loop over all bytes of data. Note: all TestVectors have same amount of data.
    while (end_of_hex_stream == false){
            // loop over all SVs, extract a subframe
            for(const TestVector& tv : testVectors) { // loop over all SVs, extract a subframe
                    std::cout << "OsnmaTestVectorsSimulation: SVID (PRN_a) "<< tv.svId << std::endl;
                    auto osnmaMsg_sptr = std::make_shared<OSNMA_msg>();
                    std::array<uint8_t, 15> hkroot{};
                    std::array<uint32_t, 15> mack{};
                    byte_index = offset_byte; // reset byte_index to the offset position for the next test vector. Offset is updated at the end of each Subframe (every 30 s or 450 Bytes)
                    std::map<uint8_t, std::bitset<128>> words; // structure containing <WORD_NUMBER> and <EXTRACTED_BITS>

                    for (int idx = 0; idx < SIZE_SUBFRAME_PAGES; ++idx)    // extract all pages of a subframe
                        {
                            // extract bytes of complete page (odd+even) -- extract SIZE_PAGE from tv.navBits, starting from byte_index
                            std::vector<uint8_t> page_bytes = extract_page_bytes(tv,byte_index,SIZE_PAGE_BYTES);
                            if(page_bytes.empty()){
                                    std::cout<< "OsnmaTestVectorsSimulation: end of TestVectors \n" << "byte_index="<<byte_index<< " expected= " << 432000/8 << std::endl;
                                    end_of_hex_stream = true;
                                    break;
                                }
                            // convert them to bitset representation using bytes_to_string
                            std::string page_bits = bytes_to_str(page_bytes);
                            // Extract the 40 OSNMA bits starting from the 18th bit
                            std::string even_page = page_bits.substr(0, page_bits.size() / 2);;
                            std::string odd_page = page_bits.substr( page_bits.size() / 2);
                            if(even_page.size() < 120 || odd_page.size() < 120 ){
                                    std::cout<< "OsnmaTestVectorsSimulation: error parsing pages" << std::endl;
                                }
                            bool even_odd_OK = even_page[0] == '0' && odd_page[0] == '1';
                            bool page_type_OK = even_page[1] == '0' && odd_page[1] == '0';
                            bool tail_bits_OK = even_page.substr(even_page.size() - 6) == "000000" && odd_page.substr(odd_page.size() - 6) == "000000";
                            if(!even_odd_OK || !page_type_OK || !tail_bits_OK)
                                std::cerr<< "OsnmaTestVectorsSimulation: error parsing pages." << std::endl;

                            std::bitset<112> data_k(even_page.substr(2,112));
                            std::bitset<16> data_j(odd_page.substr(2,16));
                            std::bitset<112> shifted_data_k = data_k;
//                            uint8_t word_type = 0;
//                            for(int i = 0; i < 6; ++i) {
//                                    word_type |= (data_k[104 + i] << i);
//                                }
                            uint8_t word_type = static_cast<uint8_t>((shifted_data_k >>= 106).to_ulong()); // word type is the first 6 bits of the word
                            std::cout<< "OsnmaTestVectorsSimulation: received Word "<< static_cast<int>(word_type) << std::endl;
                            if( (word_type >= 1 && word_type <=5) || word_type == 6 || word_type == 10)
                                {
                                    // store raw word
                                    std::bitset<128> data_combined(data_k.to_string() + data_j.to_string());
                                    words[word_type] = data_combined;
                                }
                            if(word_type == DUMMY_PAGE)
                                flag_dummy_page = true;
                            // place it into osnma object.
                            std::bitset<40> osnmaBits(odd_page.substr(18, 40));
                            // Extract bits for hkroot and mack
                            std::bitset<8> hkrootBits(osnmaBits.to_string().substr(0, 8));
                            std::bitset<32> mackBits(osnmaBits.to_string().substr(8, 32));
                            hkroot[idx] = static_cast<uint8_t>(hkrootBits.to_ulong());
                            mack[idx] = static_cast<uint32_t>(mackBits.to_ulong());

                            byte_index += SIZE_PAGE_BYTES;
                        }

                    std::cout<< "----------" << std::endl;
                    if(end_of_hex_stream)
                        break;
                    if(flag_dummy_page){
                            flag_dummy_page = false;
                            continue; // skip this SV
                        }

                    // Fill osnma object
                    osnmaMsg_sptr->hkroot = hkroot;
                    osnmaMsg_sptr->mack = mack;

                    osnmaMsg_sptr->TOW_sf0 = d_GST_SIS & 0x000FFFFF;
                    osnmaMsg_sptr->WN_sf0 = (d_GST_SIS & 0xFFF00000) >> 20 ;
                    osnmaMsg_sptr->PRN = tv.svId; // PRNa

                    // TODO - refactor this logic, currently it is split

                    // check if words 1--> 5 words are received => fill EphClockStatus data vector
                    bool ephClockStatusWordsReceived = true;
                    for (int i = 1; i <= 5; ++i)
                        {
                            if (words.find(i) == words.end())
                                {
                                    ephClockStatusWordsReceived = false;
                                    std::cerr<< "OsnmaTestVectorsSimulation: error parsing words 1->5. "
                                                 "Word "<< i << " should be received for each subframe but was not." << std::endl;
                                }
                        }
                    // extract bits as needed by osnma block
                    if(ephClockStatusWordsReceived)
                        {

                            // Define the starting position and length of bits to extract for each word
                            std::map<uint8_t, std::pair<uint8_t, uint8_t>> extractionParams = {
                                {1, {6, 120}},
                                {2, {6, 120}},
                                {3, {6, 122}},
                                {4, {6, 120}},
                                {5, {6, 67}},
                                // TODO words 6 and 10 for TimingData
                            };

                            // Fill NavData bits -- Iterate over the extraction parameters
                            for (const auto& param : extractionParams) {
                                    uint8_t wordKey = param.first;
                                    uint8_t start = param.second.first;
                                    uint8_t length = param.second.second;

                                    // Extract the required bits and fill osnma block
                                    osnmaMsg_sptr->EphemerisClockAndStatusData_2 += words[wordKey].
                                                                                    to_string().substr(
                                                                                            start, length);
                                }
                        }

                    // check w6 && w10 is received => fill TimingData data vector
                    bool timingWordsReceived = words.find(6) != words.end() &&
                                               words.find(10) != words.end();
                    // extract bits as needed by osnma block
                    if(timingWordsReceived){
                            // Define the starting position and length of bits to extract for each word
                            std::map<uint8_t, std::pair<uint8_t, uint8_t>> extractionParams = {
                                {6, {6, 99}},
                                {10, {86, 42}}
                            };

                            // Fill NavData bits -- Iterate over the extraction parameters
                            for (const auto& param : extractionParams)
                                {
                                    uint8_t wordKey = param.first;
                                    uint8_t start = param.second.first;
                                    uint8_t length = param.second.second;

                                    // Extract the required bits and fill osnma block
                                    osnmaMsg_sptr->TimingData_2 += words[wordKey].to_string().substr(
                                        start, length);
                                }

                        }

                    // Call the handler, as if it came from telemetry decoder block
                    auto temp_obj = pmt::make_any(osnmaMsg_sptr);

                    osnma->msg_handler_osnma(temp_obj); // osnma entry point
                }


            if(!end_of_hex_stream){
                    offset_byte = byte_index; // update offset for the next subframe
                    d_GST_SIS += DURATION_SUBFRAME;
                    TOW = d_GST_SIS & 0x000FFFFF;
                    WN = (d_GST_SIS & 0xFFF00000) >> 20 ;
                    std::cout << "OsnmaTestVectorsSimulation:" << " d_GST_SIS= " << d_GST_SIS
                              << ", TOW=" << TOW
                              << ", WN=" << WN << std::endl;
                }


        }


    // Assert
    // ----------

    // TODO - create global vars with failed tags and compare to total tags (Tag Id for example)
}

std::vector<TestVector> OsnmaMsgReceiverTest::readTestVectorsFromFile(const std::string& filename)
{
    std::ifstream file(filename);
    std::vector<TestVector> testVectors;
    if (!file.is_open()) {
            std::cerr<<"Error reading the file \"" << filename <<"\" \n";
            return testVectors;
        }

    std::string line;
    std::getline(file, line);
        if (line != "SVID,NumNavBits,NavBitsHEX\r" ){
                std::cerr<<"Error parsing first line" <<"\n";
            }

    while (std::getline(file, line))
        {
            std::stringstream ss(line);
            TestVector tv;

            std::string val;

            std::getline(ss, val, ',');
            tv.svId = std::stoi(val);

            std::getline(ss, val, ',');
            tv.numNavBits = std::stoi(val);

            std::getline(ss, val, ',');
            tv.navBits = OsnmaMsgReceiverTest::parseNavBits(val);

            testVectors.push_back(tv);
        }

    return testVectors;
}
std::vector<uint8_t> OsnmaMsgReceiverTest::parseNavBits(const std::string& hex)
{
    std::vector<uint8_t> bytes;

    for (unsigned int i = 0; i < hex.length()-1; i += 2)
        {
            std::string byteString = hex.substr(i, 2);
            uint8_t byte = (uint8_t) strtol(byteString.c_str(), NULL, 16);
            bytes.push_back(byte);
        }
    return bytes;
}
std::string OsnmaMsgReceiverTest::bytes_to_str(const std::vector<uint8_t>& bytes)
{
    std::string bit_string;
    bit_string.reserve(bytes.size() * 8);
    for(const auto& byte : bytes)
        {
            std::bitset<8> bits(byte);
            bit_string += bits.to_string();
        }
    return bit_string;
}

/**
 * @brief Extracts a range of bytes from a TestVector's navBits vector.
 *
 * This function extracts a extracts the bytes of complete page (odd+even)
 * from the navBits vector of a TestVector object.
 *
 *
 * @param tv The TestVector object from which to extract bytes.
 * @param byte_index The index of the first byte to extract.
 * @param num_bytes The number of bytes to extract.
 * @return A vector containing the extracted bytes, or an empty vector if extraction is not possible.
 */
std::vector<uint8_t> OsnmaMsgReceiverTest::extract_page_bytes(const TestVector& tv, const int byte_index, const int num_bytes)
{
    // Ensure we don't go beyond the end of tv.navBits
    int num_bytes_to_extract = std::min(num_bytes, static_cast<int>(tv.navBits.size() - byte_index));

    // If byte_index is beyond the end of tv.navBits, return an empty vector
    if (num_bytes_to_extract <= 0)
        {
            return std::vector<uint8_t>();
        }

    // Use std::next to get an iterator to the range to extract
    std::vector<uint8_t> extracted_bytes(tv.navBits.begin() + byte_index, tv.navBits.begin() + byte_index + num_bytes_to_extract);

    return extracted_bytes;
}
/**
 * @brief Sets the time based on the given input.
 *
 * This function calculates the week number (WN) and time of week (TOW)
 * based on the input time and the GST_START_EPOCH. It then stores the
 * calculated values in the WN and TOW member variables. Finally, it
 * combines the WN and TOW into a single 32-bit value and stores it in
 * the d_GST_SIS member variable.
 *
 * @param input The input time as a tm struct.
 */
void OsnmaMsgReceiverTest::set_time(std::tm& input)
{
    auto epoch_time_point = std::chrono::system_clock::from_time_t(mktime(&GST_START_EPOCH));
    auto input_time_point = std::chrono::system_clock::from_time_t(mktime(&input));

    // Get the duration from epoch in seconds
    auto duration_sec = std::chrono::duration_cast<std::chrono::seconds>(input_time_point - epoch_time_point);

    // Calculate the week number (WN) and time of week (TOW)
    uint32_t sec_in_week = 7 * 24 * 60 * 60;
    uint32_t week_number = duration_sec.count() / sec_in_week;
    uint32_t time_of_week = duration_sec.count() % sec_in_week;
    this->WN =  week_number;
    this->TOW =  time_of_week + LEAP_SECONDS;
    // Return the week number and time of week as a pair

    // TODO: d_GST_SIS or d_receiver_time? doubt
    // I am assuming that local realisation of receiver is identical to SIS GST time coming from W5 or W0
    this->d_GST_SIS =  (this->WN  & 0x00000FFF) << 20 | (this->TOW & 0x000FFFFF);


}
void OsnmaMsgReceiverTest::initializeGoogleLog()
{
    google::InitGoogleLogging(log_name.c_str());
    FLAGS_minloglevel = 0; // INFO
    FLAGS_logtostderr = 0;  // add this line
    FLAGS_log_dir = "/home/cgm/CLionProjects/osnma/build/src/tests/logs";
    if (FLAGS_log_dir.empty())
        {
            std::cout << "Logging will be written at "
                      << std::filesystem::temp_directory_path()
                      << '\n'
                      << "Use gnss-sdr --log_dir=/path/to/log to change that.\n";
        }
    else
        {
            try
                {
                    const std::filesystem::path p(FLAGS_log_dir);
                    if (!std::filesystem::exists(p))
                        {
                            std::cout << "The path "
                                      << FLAGS_log_dir
                                      << " does not exist, attempting to create it.\n";
                            std::error_code ec;
                            if (!std::filesystem::create_directory(p, ec))
                                {
                                    std::cout << "Could not create the " << FLAGS_log_dir << " folder.\n";
                                    gflags::ShutDownCommandLineFlags();
                                    throw std::runtime_error("Could not create folder for logs");
                                }
                        }
                    std::cout << "Logging will be written at " << FLAGS_log_dir << '\n';
                }
            catch (const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                    std::cerr << "Could not create the " << FLAGS_log_dir << " folder.\n";
                    gflags::ShutDownCommandLineFlags();
                    throw;
                }
        }
}


TEST_F(OsnmaMsgReceiverTest, BuildTagMessageM0)
{
   // Arrange
   // ----------
   // m0
   std::vector<uint8_t> expected_message =  {
       0x02, 0x4E, 0x05, 0x46, 0x3C, 0x01, 0x83, 0xA5, 0x91, 0x05, 0x1D, 0x69, 0x25, 0x80, 0x07, 0x6B,
       0x3E, 0xEA, 0x81, 0x41, 0xBF, 0x03, 0xAD, 0xCB, 0x5A, 0xAD, 0xB2, 0x77, 0xAF, 0x6F, 0xCF, 0x21,
       0xFB, 0x98, 0xFF, 0x7E, 0x83, 0xAF, 0xFC, 0x37, 0x02, 0x03, 0xB0, 0xD8, 0xE1, 0x0E, 0xB1, 0x4D,
       0x11, 0x18, 0xE6, 0xB0, 0xE8, 0x20, 0x01, 0xA0, 0x00, 0xE5, 0x91, 0x00, 0x06, 0xD3, 0x1F, 0x00,
       0x02, 0x68, 0x05, 0x4A, 0x02, 0xC2, 0x26, 0x07, 0xF7, 0xFC, 0x00
   };

   uint32_t TOW_Tag0 = 345660;
   uint32_t TOW_NavData = TOW_Tag0 - 30;
   uint32_t TOW_Key_Tag0 = TOW_Tag0 + 30 ;
   uint32_t WN = 1248;
   uint32_t PRNa = 2;
   uint8_t CTR = 1;

   osnma->d_osnma_data.d_dsm_kroot_message.ts = 9; // 40 bit
   osnma->d_tesla_keys[TOW_Key_Tag0] = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6, 0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDB, 0xBC, 0x73}; // K4
   osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
   osnma->d_satellite_nav_data[PRNa][TOW_NavData].ephemeris_iono_vector_2 = "000011101001011001000100000101000111010110100100100101100000000000011101101011001111101110101010000001010000011011111100000011101011011100101101011010101011011011001001110111101011110110111111001111001000011111101110011000111111110111111010000011101011111111110000110111000000100000001110110000110110001110000100001110101100010100110100010001000110001110011010110000111010000010000000000001101000000000000011100101100100010000000000000110110100110001111100000000000000100110100000000101010010100000001011000010001001100000011111110111111111000000000";
   osnma->d_osnma_data.d_nma_header.nmas = 0b10;

   MACK_tag_and_info MTI;
   MTI.tag = static_cast<uint64_t>(0xE37BC4F858);
   MTI.tag_info.PRN_d = 0x02;
   MTI.tag_info.ADKD = 0x00;
   MTI.tag_info.cop = 0x0F;
   Tag t0(MTI, TOW_Tag0, WN, PRNa, CTR);



   // Act
   // ----------
   auto computed_message = osnma->build_message(t0);


   // Assert
   // ----------
   ASSERT_TRUE(computed_message == expected_message);

}
TEST_F(OsnmaMsgReceiverTest, TagVerification) {
    // Arrange
    // ----------
    // Tag0
    uint32_t TOW_Tag0 = 345660;
    uint32_t TOW_NavData = TOW_Tag0 - 30;
    uint32_t TOW_Key_Tag0 = TOW_Tag0 + 30 ;
    uint32_t WN = 1248;
    uint32_t PRNa = 2;
    uint8_t CTR = 1;

    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9; // 40 bit
    osnma->d_tesla_keys[TOW_Key_Tag0] = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6, 0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDD, 0xBC, 0x73}; // K4
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_satellite_nav_data[PRNa][TOW_NavData].ephemeris_iono_vector_2 = "000011101001011001000100000101000111010110100100100101100000000000011101101011001111101110101010000001010000011011111100000011101011011100101101011010101011011011001001110111101011110110111111001111001000011111101110011000111111110111111010000011101011111111110000110111000000100000001110110000110110001110000100001110101100010100110100010001000110001110011010110000111010000010000000000001101000000000000011100101100100010000000000000110110100110001111100000000000000100110100000000101010010100000001011000010001001100000011111110111111111000000000";
    osnma->d_osnma_data.d_nma_header.nmas = 0b10;

    MACK_tag_and_info MTI;
    MTI.tag = static_cast<uint64_t>(0xE37BC4F858);
    MTI.tag_info.PRN_d = 0x02;
    MTI.tag_info.ADKD = 0x00;
    MTI.tag_info.cop = 0x0F;
    Tag t0(MTI, TOW_Tag0, WN, PRNa, CTR);



    // Act
    // ----------
    bool result_tag0 = osnma->verify_tag(t0);





    // Assert
    // ----------
    //ASSERT_TRUE(result_tag0);

    // Tag3
    uint32_t TOW_Tag3 = 345660;
    uint32_t TOW_NavData_Tag3 = TOW_Tag3 - 30;
    uint32_t TOW_Key_Tag3 = TOW_Tag0 + 30 ;
    WN = 1248;
    PRNa = 2;
    CTR = 3;

    osnma->d_osnma_data.d_dsm_kroot_message.ts = 9; // 40 bit
    osnma->d_tesla_keys[TOW_Key_Tag3] = {0x69, 0xC0, 0x0A, 0xA7, 0x36, 0x42, 0x37, 0xA6, 0x5E, 0xBF, 0x00, 0x6A, 0xD8, 0xDD, 0xBC, 0x73}; // K4
    osnma->d_osnma_data.d_dsm_kroot_message.mf = 0;
    osnma->d_satellite_nav_data[PRNa][TOW_NavData].utc_vector_2 =
        "111111111111111111111111111111110000000000000000000000010001001001001000"
        "111000001000100111100010010111111111011110111111111001001100000100000000";
    osnma->d_osnma_data.d_nma_header.nmas = 0b10;

    MTI.tag = static_cast<uint64_t>(0x7BB238C883);
    MTI.tag_info.PRN_d = 0x02;
    MTI.tag_info.ADKD = 0x04;
    MTI.tag_info.cop = 0x0F;
    Tag t3(MTI, TOW_Tag0, WN, PRNa, CTR);

    bool result_tag3 = osnma->verify_tag(t3);

    ASSERT_TRUE(result_tag0 && result_tag3);

}
