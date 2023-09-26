#include <iostream>
#include <Eigen/Dense>
#include "common.hpp"

int main(int argc, char const *argv[])
{
    (void)argc;
    (void)argv;
    Eigen::Quaterniond enu_q = Eigen::Quaterniond::UnitRandom();
    Eigen::Quaterniond ned_q = personal::common::enu_2_ned(enu_q);
    Eigen::Quaterniond enu_q2 = personal::common::ned_2_enu(ned_q);

    std::cout << "ENU 1 " << enu_q << std::endl;
    std::cout << "NED 1 " << ned_q << std::endl;
    std::cout << "ENU 2 " << enu_q2 << std::endl;
    
    /* code */
    return 0;
}
