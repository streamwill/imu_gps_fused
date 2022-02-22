#pragma once
#include <iostream>
#include <Eigen/Core>
#include <GeographicLib/LocalCartesian.hpp>

namespace ImuGpsLocalization {

constexpr double kDegreeToRadian = M_PI / 180.;
constexpr double kRadianToDegree = 180. / M_PI;

inline void ConvertLLAToENU(const Eigen::Vector3d& init_lla, 
                            const Eigen::Vector3d& point_lla, 
                            Eigen::Vector3d& point_enu) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2), 
                            point_enu(0), point_enu(1), point_enu(2));
}

inline void ConvertENUToLLA(const Eigen::Vector3d& init_lla, 
                            const Eigen::Vector3d& point_enu,
                            Eigen::Vector3d& point_lla) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2), 
                            point_lla(0), point_lla(1), point_lla(2));                            
}

inline void ConvertENU2BODY(const Eigen::Vector3d& init_rpy,
                            const Eigen::Vector3d& body_rpy,
                            Eigen::Vector3d& body_xyz, 
                            Eigen::Quaterniond& body_qua){
    // rpy init angle
    Eigen::AngleAxisd ria(0 * kDegreeToRadian, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pia(0 * kDegreeToRadian, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yia(init_rpy(2) * kDegreeToRadian, Eigen::Vector3d::UnitZ());
    // Eigen::Matrix3d mat_init;    mat_init = yia * pia * ria;
    Eigen::Quaterniond qua_init = yia * pia * ria;

    // rpy body angle
    Eigen::AngleAxisd rba(body_rpy(0) * kDegreeToRadian, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pba(body_rpy(1) * kDegreeToRadian, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yba(body_rpy(2) * kDegreeToRadian, Eigen::Vector3d::UnitZ());
    // Eigen::Matrix3d mat_body;   mat_body = yba * pba * rba;
    Eigen::Quaterniond qua_body = yba * pba * rba;

    // rpy flu to body
    Eigen::AngleAxisd rf2ba(M_PI, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pf2ba(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yf2ba(0, Eigen::Vector3d::UnitZ());
    // Eigen::Matrix3d mat_f2b;    mat_f2b = yf2ba * pf2ba * rf2ba;
    Eigen::Quaterniond qua_flu2body = yf2ba * pf2ba * rf2ba;

    // rpy ned to enu
    Eigen::AngleAxisd rn2ea(M_PI, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pn2ea(0, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yn2ea(M_PI / 2, Eigen::Vector3d::UnitZ());
    // Eigen::Matrix3d mat_n2e;    mat_n2e = yn2ea * pn2ea * rn2ea;
    Eigen::Quaterniond qua_ned2enu = yn2ea * pn2ea * rn2ea;

    // convert position
    // Eigen::Vector3d vec_now = body_xyz;
    // body_xyz = mat_f2b * mat_init.inverse() * mat_n2e * vec_now;
    body_xyz = qua_flu2body * qua_init.inverse() * qua_ned2enu * body_xyz;

    // convert atitude
    // Eigen::Matrix3d mat_now;
    // mat_now = mat_f2b * mat_init.inverse() * mat_body * mat_f2b.inverse();
    // body_rpy = mat_now.eulerAngles(2, 1, 0).reverse(); 
    body_qua = qua_flu2body * qua_init.inverse() * qua_body * qua_flu2body.inverse();
}

inline Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d w;
    w <<  0.,   -v(2),  v(1),
          v(2),  0.,   -v(0),
         -v(1),  v(0),  0.;

    return w;
}

}  // namespace ImuGpsLocalization