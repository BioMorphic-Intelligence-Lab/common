#include "common.hpp"

namespace personal
{
namespace common
{
    const Eigen::Quaterniond NED_ENU_Q = 
        quaternion_from_euler(M_PI, 0.0, M_PI_2);

    Eigen::Quaterniond quaternion_from_euler(const Eigen::Vector3d &euler)
    {
        // YPR is ZYX axes
        Eigen::Quaterniond q(Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()));

        double norm_factor = copysign(1.0, q.w());
        // Normalize to w being always positive
        Eigen::Quaterniond q_norm(q.w() * norm_factor, q.x() * norm_factor,
                               q.y() * norm_factor, q.z() * norm_factor);

        return q_norm;
    }

    float yaw_from_quaternion(const Eigen::Quaterniond &q)
    {
        double norm_factor = copysign(1.0, q.w());
        // Normalize to w being always positive
        Eigen::Quaterniond q_norm(q.w() * norm_factor, q.x() * norm_factor,
                               q.y() * norm_factor, q.z() * norm_factor);

        auto euler = q_norm.toRotationMatrix().eulerAngles(0,1,2);

        return euler.z();
    }

    Eigen::Quaterniond quaternion_from_euler(const double roll, 
                                           const double pitch,
                                           const double yaw)
    {
        return Eigen::AngleAxis<double>(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxis<double>(pitch,  Eigen::Vector3d::UnitY()) 
            * Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ());
    }

    Eigen::Vector3d enu_2_ned(const Eigen::Vector3d &enu)
    {
        Eigen::Vector3d ned(enu.y(), enu.x(), -enu.z());
        return ned;
    }
    Eigen::Vector3d ned_2_enu(const Eigen::Vector3d &ned)
    {
        Eigen::Vector3d enu(ned.y(), ned.x(), -ned.z());
        return enu;

    }
    Eigen::Quaterniond enu_2_ned(const Eigen::Quaterniond &enu)
    {
        Eigen::Quaternion ned = NED_ENU_Q * enu;
        return ned;
    }
    Eigen::Quaterniond ned_2_enu(const Eigen::Quaterniond &ned)
    {
        Eigen::Quaternion enu = NED_ENU_Q * ned;
        return enu;
    }

    Eigen::Matrix3d rot_x(double theta)
    {
    double sT = sin(theta), cT = cos(theta);
    Eigen::Matrix3d rot;
    rot << 1, 0, 0,
            0, cT, -sT,
            0, sT, cT;
    return rot;
    }

    Eigen::Matrix3d rot_y(double theta)
    {

    double sT = sin(theta), cT = cos(theta);
    Eigen::Matrix3d rot;
    rot << cT, 0, sT,
            0, 1, 0,
            -sT, 0, cT;
    return rot;
    }
    Eigen::Matrix3d rot_z(double theta)
    {
    double sT = sin(theta), cT = cos(theta);
    Eigen::Matrix3d rot;
    rot << cT, -sT, 0,
            sT, cT, 0,
            0, 0, 1;
    return rot;
    }
}
}