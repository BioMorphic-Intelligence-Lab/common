#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>
#include "constants.hpp"

namespace personal
{
        
    namespace common
    {
        float yaw_from_quaternion(const Eigen::Quaterniond &q);
        Eigen::Quaterniond quaternion_from_euler(const double roll, 
                                                const double pitch,
                                                const double yaw);
        Eigen::Quaterniond quaternion_from_euler(const Eigen::Vector3d &euler);
        Eigen::Vector3d enu_2_ned(const Eigen::Vector3d &enu);
        Eigen::Vector3d ned_2_enu(const Eigen::Vector3d &ned);
        Eigen::Quaterniond enu_2_ned(const Eigen::Quaterniond &enu);
        Eigen::Quaterniond ned_2_enu(const Eigen::Quaterniond &ned);
        Eigen::Matrix3d rot_x(double theta);
        Eigen::Matrix3d rot_y(double theta);
        Eigen::Matrix3d rot_z(double theta);

        double normalize_angle(double theta);
    }

}
#endif // COMMON_H