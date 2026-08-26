#include "rtctrl/model/ptp_csv.hpp"

#include <zeo/zeo_ep.h>

#include <array>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <stdexcept>

namespace rtctrl::model {

namespace {

struct Quaternion {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Quaternion quaternion(const zMat3D& attitude,
                      const std::optional<Quaternion>& previous) {
  zEP value;
  zMat3DToEP(&attitude, &value);
  Quaternion result{value.ex.w, value.ex.v.c.x, value.ex.v.c.y,
                    value.ex.v.c.z};
  if (previous &&
      result.w * previous->w + result.x * previous->x +
              result.y * previous->y + result.z * previous->z <
          0.0) {
    result.w = -result.w;
    result.x = -result.x;
    result.y = -result.y;
    result.z = -result.z;
  }
  return result;
}

void writeVec3(std::ostream& output, const zVec3D& vector) {
  output << ',' << vector.c.x << ',' << vector.c.y << ',' << vector.c.z;
}

void writeQuaternion(std::ostream& output, const Quaternion& quaternion) {
  output << ',' << quaternion.w << ',' << quaternion.x << ',' << quaternion.y
         << ',' << quaternion.z;
}

void writeHeader(std::ostream& output) {
  output << "schema_version,sample,time_s,progress,progress_rate_1_s,"
            "progress_acceleration_1_s2,profile_derivative_discontinuity"
         << ",target_pos_x_m,target_pos_y_m,target_pos_z_m"
         << ",target_quat_w,target_quat_x,target_quat_y,target_quat_z"
         << ",target_vel_x_m_s,target_vel_y_m_s,target_vel_z_m_s"
         << ",target_acc_x_m_s2,target_acc_y_m_s2,target_acc_z_m_s2"
         << ",target_omega_x_rad_s,target_omega_y_rad_s,"
            "target_omega_z_rad_s"
         << ",target_alpha_x_rad_s2,target_alpha_y_rad_s2,"
            "target_alpha_z_rad_s2"
         << ",fk_pos_x_m,fk_pos_y_m,fk_pos_z_m"
         << ",fk_quat_w,fk_quat_x,fk_quat_y,fk_quat_z"
         << ",fk_vel_x_m_s,fk_vel_y_m_s,fk_vel_z_m_s"
         << ",fk_acc_x_m_s2,fk_acc_y_m_s2,fk_acc_z_m_s2"
         << ",fk_omega_x_rad_s,fk_omega_y_rad_s,fk_omega_z_rad_s"
         << ",fk_alpha_x_rad_s2,fk_alpha_y_rad_s2,fk_alpha_z_rad_s2"
         << ",position_error_x_m,position_error_y_m,position_error_z_m,"
            "position_error_norm_m"
         << ",attitude_error_x_rad,attitude_error_y_rad,"
            "attitude_error_z_rad,attitude_error_norm_rad"
         << ",ik_converged,ik_iterations,ik_position_residual_m,"
            "ik_attitude_residual_rad,ik_within_limits,ik_finite";
  for (int joint = 0; joint < kCanonicalDof; ++joint) {
    output << ",q" << joint << "_rad";
  }
  for (int joint = 0; joint < kCanonicalDof; ++joint) {
    output << ",dq" << joint << "_rad_s";
  }
  for (int joint = 0; joint < kCanonicalDof; ++joint) {
    output << ",ddq" << joint << "_rad_s2";
  }
  for (int joint = 0; joint < kCanonicalDof; ++joint) {
    output << ",joint_limit_margin" << joint << "_rad";
  }
  output << '\n';
}

}  // namespace

void writePtpCsv(const std::string& path, const PtpPlan& plan) {
  std::ofstream output(path, std::ios::out | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot open PTP diagnostics '" + path + "'");
  }
  output << std::setprecision(std::numeric_limits<double>::max_digits10);
  writeHeader(output);
  std::optional<Quaternion> previous_target;
  std::optional<Quaternion> previous_achieved;
  for (std::size_t i = 0; i < plan.samples.size(); ++i) {
    const auto& sample = plan.samples[i];
    const auto target_quaternion =
        quaternion(sample.target.attitude, previous_target);
    const auto achieved_quaternion =
        quaternion(sample.achieved.attitude, previous_achieved);
    previous_target = target_quaternion;
    previous_achieved = achieved_quaternion;
    const double progress_rate =
        plan.duration > 0.0 ? sample.progress.velocity / plan.duration : 0.0;
    const double progress_acceleration =
        plan.duration > 0.0
            ? sample.progress.acceleration / (plan.duration * plan.duration)
            : 0.0;

    output << 1 << ',' << i << ',' << sample.time << ','
           << sample.progress.position << ',' << progress_rate << ','
           << progress_acceleration << ','
           << (sample.progress.derivative_discontinuity ? 1 : 0);
    writeVec3(output, sample.target.position);
    writeQuaternion(output, target_quaternion);
    writeVec3(output, sample.target_linear_velocity);
    writeVec3(output, sample.target_linear_acceleration);
    writeVec3(output, sample.target_angular_velocity);
    writeVec3(output, sample.target_angular_acceleration);
    writeVec3(output, sample.achieved.position);
    writeQuaternion(output, achieved_quaternion);
    writeVec3(output, sample.achieved_linear_velocity);
    writeVec3(output, sample.achieved_linear_acceleration);
    writeVec3(output, sample.achieved_angular_velocity);
    writeVec3(output, sample.achieved_angular_acceleration);
    writeVec3(output, sample.position_error);
    output << ',' << zVec3DNorm(&sample.position_error);
    writeVec3(output, sample.attitude_error);
    output << ',' << zVec3DNorm(&sample.attitude_error) << ','
           << (sample.ik.converged ? 1 : 0) << ',' << sample.ik.iterations
           << ',' << sample.ik.pos_residual << ',' << sample.ik.att_residual
           << ',' << (sample.ik.within_limits ? 1 : 0) << ','
           << (sample.ik.finite ? 1 : 0);
    for (const double value : sample.joint_position) output << ',' << value;
    for (const double value : sample.joint_velocity) output << ',' << value;
    for (const double value : sample.joint_acceleration) output << ',' << value;
    for (const double value : sample.joint_limit_margin) output << ',' << value;
    output << '\n';
  }
  output.close();
  if (!output) {
    throw std::runtime_error("cannot finish PTP diagnostics '" + path + "'");
  }
}

}  // namespace rtctrl::model
