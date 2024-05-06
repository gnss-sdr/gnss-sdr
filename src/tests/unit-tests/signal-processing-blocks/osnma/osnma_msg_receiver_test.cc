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
    std::string log_name {"CONFIG1-2023-08-23-PKID1-OSNMA"};
    void initializeGoogleLog();

    void SetUp() override
    {
        flag_CRC_test = false;
        page_even = "";

        std::tm input_time = {0, 0, 5, 16, 8 - 1, 2023 - 1900, 0};
        set_time(input_time);
        std::string pemFilePath = "OSNMA_PublicKey_20230803105952_newPKID_1.pem";
        std::string merkleFilePath = "OSNMA_MerkleTree_20230803105953_newPKID_1.xml";
        osnma = osnma_msg_receiver_make(pemFilePath, merkleFilePath);
    }
    void TearDown() override{
        google::ShutdownGoogleLogging();
    }

public:
    static std::vector<uint8_t> parseNavBits(const std::string& hex);
    static std::vector<TestVector> readTestVectorsFromFile(const std::string& filename);
    std::string bytes_to_str(const std::vector<uint8_t>& bytes);
    std::vector<uint8_t> extract_page_bytes(const TestVector& tv, const int byte_index, const int num_bytes);
};


TEST_F(OsnmaMsgReceiverTest, TeslaKeyVerification) {
    // Arrange
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
    bool result = osnma->verify_tesla_key(key, TOW);





    // Assert
    ASSERT_TRUE(result); // Adjust this according to what you expect

}

TEST_F(OsnmaMsgReceiverTest, OsnmaTestVectorsSimulation)
{
    initializeGoogleLog();
    // Arrange
    std::vector<TestVector> testVectors = readTestVectorsFromFile(/*"/home/cgm/CLionProjects/osnma/src/tests/data/*/"16_AUG_2023_GST_05_00_01.csv");
    bool end_of_hex_stream{false};
    int offset_byte{0};
    int byte_index{0}; // index containing the last byte position of the hex stream that was retrieved. Takes advantage that all TVs have same size
    const int SIZE_PAGE_BYTES{240/8};
    const int SIZE_SUBFRAME_PAGES{15};
    const int SIZE_SUBFRAME_BYTES{SIZE_PAGE_BYTES*SIZE_SUBFRAME_PAGES};
    const int DURATION_SUBFRAME{30};

    std::cout << "OsnmaTestVectorsSimulation:" << " d_GST_SIS= " << d_GST_SIS
    << ", TOW=" << TOW
    << ", WN=" << WN << std::endl;


    // Act
    while (end_of_hex_stream == false){ // loop over all bytes of data. Note all TestVectors have same amount of data.
            for(const TestVector& tv : testVectors) { // loop over all SVs, extract a subframe
                    auto osnmaMsg_sptr = std::make_shared<OSNMA_msg>();
                    std::array<uint8_t, 15> hkroot{};
                    std::array<uint32_t, 15> mack{};
                    byte_index = offset_byte; // reset byte_index to the offset position for the next test vector. Offset is updated at the end of each Subframe (every 30 s or 450 Bytes)
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
                            std::bitset<40> osnmaBits(odd_page.substr(18, 40));
                            // Extract bits for hkroot and mack
                            std::bitset<8> hkrootBits(osnmaBits.to_string().substr(0, 8));
                            std::bitset<32> mackBits(osnmaBits.to_string().substr(8, 32));
                            hkroot[idx] = static_cast<uint8_t>(hkrootBits.to_ulong());
                            mack[idx] = static_cast<uint32_t>(mackBits.to_ulong());

                            byte_index += SIZE_PAGE_BYTES;
                        }
                    if(end_of_hex_stream)
                        break;
                    osnmaMsg_sptr->hkroot = hkroot;
                    osnmaMsg_sptr->mack = mack;

                    osnmaMsg_sptr->TOW_sf0 = d_GST_SIS & 0x000FFFFF;
                    osnmaMsg_sptr->WN_sf0 = (d_GST_SIS & 0xFFF00000) >> 20 ;
                    osnmaMsg_sptr->PRN = tv.svId;

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
    // TODO
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

    this->d_GST_SIS =  (this->WN  & 0x00000FFF) << 20 | (this->TOW & 0x000FFFFF);


}

void OsnmaMsgReceiverTest::initializeGoogleLog()
{
    google::InitGoogleLogging(log_name.c_str());
    FLAGS_minloglevel = 1;
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