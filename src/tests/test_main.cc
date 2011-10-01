#include <iostream>
#include <gtest/gtest.h>
#include <glog/log_severity.h>
#include <glog/logging.h>

int main(int argc, char **argv) {
  std::cout << "Running main() from test_main.cc\n";

  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();
}