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
            Eigen::Quaterniond q(Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()) *
                                 Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                                 Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitZ()));

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

            auto euler = q_norm.toRotationMatrix().eulerAngles(2, 1, 0);

            return euler[0];
        }

        float yaw_from_quaternion(double w, double x, double y, double z)
        {
            Eigen::Quaterniond q(w, x, y, z);
            return yaw_from_quaternion(q);
        }

        /*get yaw that makes y-axis lay in y-z-plane of unit-orientation after applying rot_z(yaw)*q.toRotationMatrix */
        float yaw_from_quaternion_y_align(const Eigen::Quaterniond &q)
        {
            double norm_factor = copysign(1.0, q.w());
            Eigen::Quaterniond q_norm(q.w() * norm_factor, q.x() * norm_factor,
                                      q.y() * norm_factor, q.z() * norm_factor);
            const Eigen::Matrix3d R = q_norm.toRotationMatrix();
            double yaw = -atan2(R(0, 1), R(1, 1));

            return yaw;
        }

        float roll_from_quaternion(const Eigen::Quaterniond &q)
        {
            double norm_factor = copysign(1.0, q.w());
            // Normalize to w being always positive
            Eigen::Quaterniond q_norm(q.w() * norm_factor, q.x() * norm_factor,
                                      q.y() * norm_factor, q.z() * norm_factor);

            auto euler = q_norm.toRotationMatrix().eulerAngles(2, 1, 0);

            return euler[1];
        }

        Eigen::Quaterniond quaternion_from_euler(const double roll,
                                                 const double pitch,
                                                 const double yaw)
        {
            Eigen::Quaterniond q(Eigen::AngleAxis<double>(roll, Eigen::Vector3d::UnitX()) * Eigen::AngleAxis<double>(pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxis<double>(yaw, Eigen::Vector3d::UnitZ()));

            double norm_factor = copysign(1.0, q.w());
            // Normalize to w being always positive
            Eigen::Quaterniond q_norm(q.w() * norm_factor, q.x() * norm_factor,
                                      q.y() * norm_factor, q.z() * norm_factor);

            return q_norm;
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
            double norm_factor = copysign(1.0, ned.w());
            Eigen::Quaternion ned_norm(ned.w() / norm_factor,
                                       ned.x() / norm_factor,
                                       ned.y() / norm_factor,
                                       ned.z() / norm_factor);
            return ned_norm;
        }
        Eigen::Quaterniond ned_2_enu(const Eigen::Quaterniond &ned)
        {
            Eigen::Quaternion enu = NED_ENU_Q * ned;

            double norm_factor = copysign(1.0, enu.w());
            Eigen::Quaternion enu_norm(enu.w() / norm_factor,
                                       enu.x() / norm_factor,
                                       enu.y() / norm_factor,
                                       enu.z() / norm_factor);
            return enu_norm;
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

        double normalize_angle(double theta)
        {
            theta = fmod(theta + M_PI, 2 * M_PI);
            if (theta < 0)
                theta += 2 * M_PI;
            return theta - M_PI;
        }
    }
}