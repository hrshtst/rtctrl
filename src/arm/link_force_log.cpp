#include "rtctrl/arm/link_force_log.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace rtctrl::arm {

LinkForceCsvLog::LinkForceCsvLog(const std::string& path)
    : file_(std::fopen(path.c_str(), "wx")) {
  if (file_ == nullptr) {
    throw std::runtime_error(
        "link force log: cannot create '" + path +
        "' (it may already exist): " + std::strerror(errno));
  }
  writeHeader();
}

LinkForceCsvLog::LinkForceCsvLog(std::FILE* stream, AdoptStream)
    : file_(stream) {
  if (file_ == nullptr) {
    throw std::invalid_argument("link force log: null stream");
  }
  writeHeader();
}

LinkForceCsvLog::~LinkForceCsvLog() { close(); }

void LinkForceCsvLog::writeHeader() {
  int status = std::fprintf(
      file_,
      "schema_version,attempt,start_seq,end_seq,start_s,end_s,"
      "substeps_completed,complete,status,rejected,rejected_eval_s,"
      "prepared,started,active,last_eval_s,last_substep_dt_s,last_scale,"
      "last_force_x_n,last_force_y_n,last_force_z_n,"
      "last_point_x_m,last_point_y_m,last_point_z_m");
  for (int i = 0; i < model::kCanonicalDof && status >= 0; ++i) {
    status = std::fprintf(file_, ",last_tau_ext%d_nm", i);
  }
  if (status >= 0) {
    status = std::fprintf(file_, ",impulse_x_ns,impulse_y_ns,impulse_z_ns");
  }
  for (int i = 0; i < model::kCanonicalDof && status >= 0; ++i) {
    status = std::fprintf(file_, ",joint_impulse%d_nms", i);
  }
  if (status >= 0) status = std::fprintf(file_, "\n");
  if (status < 0) failed_ = true;
}

bool LinkForceCsvLog::write(const LinkForceInterval& iv) {
  if (failed_ || closed_) return false;
  int status = std::fprintf(
      file_,
      "1,%llu,%llu,%llu,%.9f,%.9f,%d,%d,%s,%d,%.9f,%d,%d,%d,%.9f,%.9g,%.9g,"
      "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
      static_cast<unsigned long long>(iv.attempt),
      static_cast<unsigned long long>(iv.start_seq),
      static_cast<unsigned long long>(iv.end_seq), iv.start_s, iv.end_s,
      iv.substeps_completed, iv.complete ? 1 : 0,
      linkForceStatusName(iv.status), iv.rejected ? 1 : 0, iv.rejected_eval_s,
      iv.prepared ? 1 : 0, iv.started ? 1 : 0, iv.active ? 1 : 0,
      iv.last_eval_s, iv.last_substep_dt, iv.last_scale, iv.last_force_world[0],
      iv.last_force_world[1], iv.last_force_world[2], iv.last_point_world[0],
      iv.last_point_world[1], iv.last_point_world[2]);
  for (int i = 0; i < model::kCanonicalDof && status >= 0; ++i) {
    status = std::fprintf(file_, ",%.9g", iv.last_joint_torque_nm[i]);
  }
  for (int k = 0; k < 3 && status >= 0; ++k) {
    status = std::fprintf(file_, ",%.9g", iv.force_impulse_ns[k]);
  }
  for (int i = 0; i < model::kCanonicalDof && status >= 0; ++i) {
    status = std::fprintf(file_, ",%.9g", iv.joint_impulse_nms[i]);
  }
  if (status >= 0) status = std::fprintf(file_, "\n");
  if (status < 0 || std::fflush(file_) != 0) failed_ = true;
  return !failed_;
}

bool LinkForceCsvLog::close() {
  if (!closed_) {
    closed_ = true;
    close_ok_ = std::fclose(file_) == 0;
    file_ = nullptr;
  }
  return close_ok_ && !failed_;
}

}  // namespace rtctrl::arm
