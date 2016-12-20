
#include <iostream>
#include <unistd.h>
#include <sys/wait.h>
#include <exception>
//#include <gflags/gflags.h>
//#include <glog/logging.h>
//include <gtest/gtest.h>



int main()
{
    pid_t wait_result;
    int child_status;

    // Configure signal
    std::string str = std::string(SW_GENERATOR_BIN);
    std::string p1 = std::string("-rinex_nav_file=") + std::string(DEFAULT_RINEX_NAV);
    std::string p2 = std::string("-static_position=30.286502,120.032669,100,40");

    char *const parmList[] = { &str[0], &str[0], &p1[0], &p2[0], NULL };
    int pid;
    if ((pid = fork()) == -1)
        perror("fork error");
    else if (pid == 0)
    {
        execv(&str[0], parmList);
        std::cout << "Return not expected. Must be an execv error." << std::endl;
        std::terminate();
    }

    wait_result = waitpid(pid, &child_status, 0);
    std::cout << "Signal and Observables RINEX files created."  << std::endl;

    return 0;
}
