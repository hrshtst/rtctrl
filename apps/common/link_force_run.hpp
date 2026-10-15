// Simulation-frontend lifecycle of a configured link-force disturbance,
// shared by x7_follow_sim and x7_track_sim: preflight, arming on the
// simulator with its interval log, starting the profile clock from the
// run's tracking-start hook, and the end-of-run report.
#pragma once

#include <cstdio>
#include <stdexcept>
#include <string>

#include "common/link_force_bundle.hpp"
#include "rtctrl/arm/link_force_interval.hpp"
#include "rtctrl/arm/link_force_log.hpp"
#include "rtctrl/arm/sim_arm.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/link_force.hpp"

namespace x7 {

// The link must exist; a profile whose onset is at or after the end of
// the run (tracking plus final hold, run_s seconds of profile clock)
// would never act and is rejected; one that outlasts the run is cut off,
// which is only warned about.
inline void preflightLinkForce(const rtctrl::model::LinkForceSpec& spec,
                               const rtctrl::model::ChainModel& chain,
                               double run_s) {
  if (chain.linkIndex(spec.link) < 0) {
    throw std::runtime_error("link force: unknown link '" + spec.link + "'");
  }
  if (spec.profile.onset_s >= run_s) {
    throw std::runtime_error(
        "link force: onset_s " + std::to_string(spec.profile.onset_s) +
        " s is at or after the end of the run (" + std::to_string(run_s) +
        " s of tracking and final hold); the force would never act");
  }
  if (spec.profile.end_s() > run_s) {
    std::fprintf(stderr,
                 "warning: link force profile ends at %.3f s, after the run "
                 "(%.3f s); it will be cut off\n",
                 spec.profile.end_s(), run_s);
  }
}

struct LinkForceRunReport {
  LinkForceOutcome outcome;
  bool faulted = false;
  rtctrl::arm::LinkForceInterval last_interval;
  bool log_ok = true;
};

// Sets the spec and log on the simulator (strict accounting from now on)
// and starts the profile clock at the feedback time of the run's first
// tracking cycle. Run is follow::FollowRun or track::TrackingRun.
template <class Run>
void armLinkForce(rtctrl::arm::SimArm& robot,
                  const rtctrl::model::LinkForceSpec& spec,
                  rtctrl::arm::LinkForceCsvLog* log, Run& run,
                  LinkForceRunReport* report) {
  robot.setLinkForce(spec);
  robot.linkForceLogTo(log);
  run.setTrackingStartHook(
      [&robot, report](const rtctrl::arm::JointState& state) {
        robot.startLinkForce(state.t);
        report->outcome.started = true;
      });
}

// Collects what the simulator actually did and closes the interval log.
inline void finishLinkForce(const rtctrl::arm::SimArm& robot,
                            rtctrl::arm::LinkForceCsvLog& log,
                            LinkForceRunReport* report) {
  report->outcome.applied = robot.linkForceApplied();
  report->outcome.first_applied_s = robot.firstLinkForceTime();
  report->outcome.last_applied_s = robot.lastLinkForceTime();
  report->faulted = robot.linkForceFaulted();
  report->last_interval = robot.lastLinkForceInterval();
  const bool closed = log.close();
  report->log_ok = !robot.linkForceLogFailed() && closed;
}

// Prints the outcome; returns false when the run must fail because of the
// link force (a rejected substep, or an incomplete disturbance log).
inline bool printLinkForceReport(const LinkForceRunReport& report,
                                 const std::string& log_path) {
  const auto& outcome = report.outcome;
  if (outcome.applied) {
    std::printf(
        "link force: started, applied from %.4f s to %.4f s (sim time)\n",
        outcome.first_applied_s, outcome.last_applied_s);
  } else {
    std::printf("link force: %s, not applied\n",
                outcome.started ? "started" : "not started");
  }
  std::printf("disturbance log: %s\n", log_path.c_str());
  if (report.faulted) {
    std::fprintf(stderr,
                 "link force: %s at sim time %.6f s (attempt %llu); the "
                 "simulator latched and the run stopped\n",
                 rtctrl::arm::linkForceStatusName(report.last_interval.status),
                 report.last_interval.end_s,
                 static_cast<unsigned long long>(report.last_interval.attempt));
  }
  if (!report.log_ok) {
    std::fprintf(stderr,
                 "link force: writing the disturbance log failed; it is "
                 "incomplete\n");
  }
  return !report.faulted && report.log_ok;
}

}  // namespace x7
