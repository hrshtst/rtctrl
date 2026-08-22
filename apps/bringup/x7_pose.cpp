// Positioning aid: moves the arm to a canonical posture in POSITION
// mode so the operator sees the real target on hardware, holds it,
// then goes limp on request while the operator supports the arm.
// Built as the manual-fallback handover for the (since removed)
// identification campaign; it remains a general positioning aid for
// visually confirming a posture on hardware.
//
// SAFETY: the arm is servo-stiff during the move and DROPS under
// gravity the moment it goes limp — support the shoulder and elbow
// BEFORE pressing Enter. Power cutoff within reach.
//
// Usage: x7_pose [--config path] [--port dev]
//                (--posture <file> | --tcp X Y Z ROLL PITCH YAW)
//                [--vel v] [--preview <basename>]
//   --posture  target vector: a checked-in config/postures/*.json, a
//              .dwells.json sidecar, or any file with 8 numbers
//   --tcp      world-frame tool-center pose: position [m] and
//              roll/pitch/yaw [rad] (R = Rz(yaw) Ry(pitch) Rx(roll))
//              of crane_x7_tcp_link, the fingertip-midpoint frame
//              whose axes align with the world axes when the gripper
//              points forward (+x). Resolved to joint targets by IK
//              (arm joints only; the gripper holds its value): a
//              non-converged solve is refused, on hardware BEFORE any
//              motion. The hardware solve seeds from the measured
//              posture, so of multiple IK branches the nearest wins.
//   --vel      joint speed limit [rad/s], default 0.25, max 0.5
//   --preview  solve/load the target, write <basename>.init.ztk, and
//              exit WITHOUT touching the bus; view with
//              rk_pen -model models/crane_x7/crane_x7.ztk -init <file>

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "ident/ident_common.hpp"
#include "bringup/pose_common.hpp"
#include "rtctrl/model/chain_model.hpp"
#include "rtctrl/model/ik_solver.hpp"
#include "rtctrl/model/joint_map.hpp"
#include "rtctrl/model/trajectory.hpp"
#include "rtctrl/model/zvector.hpp"
#include "common/x7_common.hpp"

namespace model = rtctrl::model;

namespace {

bool enterPressed() {
  struct pollfd pfd = {0 /*stdin*/, POLLIN, 0};
  if (poll(&pfd, 1, 0) <= 0) return false;
  char buf[64];
  return read(0, buf, sizeof buf) > 0;
}

// World-frame TCP pose -> canonical joint target via IK on
// crane_x7_tcp_link. Arm joints only; the gripper keeps the seed
// value. seed8 == nullptr seeds from the zero posture (preview).
// Returns false (with diagnostics) unless the solve CONVERGED — a
// finite-but-wrong answer is never handed to the motion.
bool solveTcpTarget(model::ChainModel& chain, const model::JointMap& map,
                    const double* tcp, const double* seed8,
                    double* out8) {
  model::IkSolver ik(chain, map, "crane_x7_tcp_link");
  model::ZVector q8(model::kCanonicalDof);
  model::ZVector q9(model::kModelDof), sol(model::kModelDof);
  if (seed8 != nullptr) {
    for (int i = 0; i < model::kCanonicalDof; ++i) q8[i] = seed8[i];
  }
  map.expand(q8.get(), q9.get());

  zVec3D pos;
  zVec3DCreate(&pos, tcp[0], tcp[1], tcp[2]);
  zMat3D att;
  zMat3DFromZYX(&att, tcp[5], tcp[4], tcp[3]);  // yaw, pitch, roll
  const auto result = ik.solve(pos, att, q9.get(), sol.get());
  if (!result.converged) {
    std::fprintf(stderr,
                 "IK did not converge on the TCP target: residuals "
                 "%.4f m / %.4f rad after %d iterations%s%s\n",
                 result.pos_residual, result.att_residual,
                 result.iterations,
                 result.within_limits ? "" : " (joint limits violated)",
                 result.finite ? "" : " (non-finite solution)");
    return false;
  }
  model::ZVector out(model::kCanonicalDof);
  map.reduce(sol.get(), out.get());
  for (int i = 0; i < model::kCanonicalDof; ++i) out8[i] = out[i];
  return true;
}

// rk_pen initial-state file for the resolved target (the same format
// examples/make_pose emits): [roki::chain::init] with one
// "joint: <link> <dis>" line per revolute joint.
bool writeInitZtk(const std::string& base, model::ChainModel& chain,
                  const model::JointMap& map, const double* q8) {
  const std::string path = base + ".init.ztk";
  std::FILE* f = std::fopen(path.c_str(), "w");
  if (f == nullptr) {
    std::fprintf(stderr, "cannot open %s\n", path.c_str());
    return false;
  }
  std::fprintf(f, "[roki::chain::init]\n");
  for (int i = 0; i < model::kCanonicalDof; ++i) {
    std::fprintf(f, "joint: %s %.6f\n",
                 zName(rkChainLink(chain.chain(), map.linkId(i))), q8[i]);
  }
  std::fprintf(f, "joint: %s %.6f\n",
               zName(rkChainLink(chain.chain(), map.linkIdFingerB())),
               q8[model::kCanonicalDof - 1]);
  std::fclose(f);
  std::printf("wrote %s\nview: rk_pen -model models/crane_x7/"
              "crane_x7.ztk -init %s\n",
              path.c_str(), path.c_str());
  return true;
}

}  // namespace

int main(int argc, char* argv[]) {
  const auto cli = x7::parseCli(argc, argv);
  if (!cli.ok) return 1;
  std::string posture_path;
  std::string preview_base;
  double tcp[6] = {};
  bool have_tcp = false;
  double vel = 0.25;
  const auto& rest = cli.rest;
  for (std::size_t i = 0; i < rest.size(); ++i) {
    if (std::strcmp(rest[i], "--posture") == 0 && i + 1 < rest.size()) {
      posture_path = rest[++i];
    } else if (std::strcmp(rest[i], "--tcp") == 0) {
      // six world-frame values; negatives are legitimate, so no
      // flag-lookalike guard — strict parsing rejects a flag anyway
      if (i + 6 >= rest.size()) {
        std::fprintf(stderr,
                     "--tcp requires 6 values: X Y Z ROLL PITCH YAW\n");
        return 1;
      }
      for (int k = 0; k < 6; ++k) {
        if (!x7::parseStrictDouble(rest[++i], &tcp[k])) {
          std::fprintf(stderr, "--tcp: invalid value %s\n", rest[i]);
          return 1;
        }
      }
      have_tcp = true;
    } else if (std::strcmp(rest[i], "--preview") == 0) {
      if (i + 1 >= rest.size() ||
          std::strncmp(rest[i + 1], "--", 2) == 0) {
        std::fprintf(stderr, "--preview requires an output basename\n");
        return 1;
      }
      preview_base = rest[++i];
    } else if (std::strcmp(rest[i], "--vel") == 0 &&
               i + 1 < rest.size()) {
      if (!x7::parseStrictDouble(rest[++i], &vel)) {
        std::fprintf(stderr, "--vel: invalid value %s\n", rest[i]);
        return 1;
      }
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", rest[i]);
      return 1;
    }
  }
  if (have_tcp && !posture_path.empty()) {
    std::fprintf(stderr,
                 "--posture and --tcp are exclusive; give one target, "
                 "not both\n");
    return 1;
  }
  if (!have_tcp && posture_path.empty()) {
    std::fprintf(stderr,
                 "a target is required: --posture <file> or "
                 "--tcp X Y Z ROLL PITCH YAW\n");
    return 1;
  }
  vel = std::clamp(vel, 0.05, 0.5);
  double posture[model::kCanonicalDof];
  if (!posture_path.empty() && !x7::loadAnchorRef(posture_path, posture)) {
    std::fprintf(stderr, "cannot read %d joint values from %s\n",
                 model::kCanonicalDof, posture_path.c_str());
    return 1;
  }

  // Preview: resolve the target against the model only and emit the
  // rk_pen initial state — the bus is never touched (a TCP solve
  // seeds from the zero posture here; the hardware path re-solves
  // from the measured posture).
  if (!preview_base.empty()) {
    try {
      model::ChainModel chain("models/crane_x7/crane_x7.ztk");
      model::JointMap map(chain);
      if (have_tcp &&
          !solveTcpTarget(chain, map, tcp, nullptr, posture)) {
        return 1;
      }
      return writeInitZtk(preview_base, chain, map, posture) ? 0 : 1;
    } catch (const std::exception& e) {
      std::fprintf(stderr, "error: %s\n", e.what());
      return 1;
    }
  }

  try {
    auto session = x7::openSession(cli, /*operating_mode_override=*/3);
    auto& arm = *session.arm;
    if (!arm.activate()) {
      std::fprintf(stderr, "activation failed: %s\n",
                   arm.lastError().c_str());
      return 1;
    }
    x7::ShutdownGuard shutdown{arm};

    if (have_tcp) {
      // Resolve the TCP pose against the model, seeded from the
      // MEASURED posture so the nearest IK branch wins; a refusal
      // deactivates before any motion. The gripper joint is not part
      // of the solve and holds its measured value.
      const auto fb = arm.lastFeedback();
      if (static_cast<int>(fb.size()) != model::kCanonicalDof) {
        std::fprintf(stderr, "posture read failed — aborting\n");
        shutdown.run();
        return 1;
      }
      double seed[model::kCanonicalDof];
      for (int i = 0; i < model::kCanonicalDof; ++i) {
        seed[i] = fb[i].position;
      }
      model::ChainModel chain("models/crane_x7/crane_x7.ztk");
      model::JointMap map(chain);
      if (!solveTcpTarget(chain, map, tcp, seed, posture)) {
        std::fprintf(stderr, "TCP target refused — deactivating\n");
        shutdown.run();
        return 1;
      }
      posture[model::kCanonicalDof - 1] =
          seed[model::kCanonicalDof - 1];
    }

    // converging placement: goal-offset iterations close the friction
    // sag so "reached" means the MEASURED posture
    const auto placed = x7::movePose(arm, posture, vel, 0.01, 5);
    if (!placed.ok) {
      std::fprintf(stderr, "placement failed — deactivating\n");
      shutdown.run();
      return 1;
    }
    if (!placed.converged) {
      std::printf("NOTE: joint %d still %.4f rad from target after "
                  "convergence iterations\n",
                  placed.worst_joint, placed.worst_dev);
    }
    const int n = static_cast<int>(placed.hold_goal.size());
    constexpr int kCycleUs = 10000;  // 100 Hz

    std::printf(
        "\nHOLDING the posture in position mode.\n"
        ">>> The arm goes LIMP and drops when you continue. CATCH,\n"
        ">>> don't hold: support from below (forearm/under-elbow)\n"
        ">>> only against the drop — do NOT grip or lift, or the\n"
        ">>> posture is lost. Press Enter to torque off.\n");
    // keep the command stream (and both watchdog layers) alive while
    // waiting — a silent bus would trip the servo Bus Watchdogs
    (void)n;
    while (!enterPressed()) {
      if (!arm.writePositions(placed.hold_goal) && arm.escalated()) {
        shutdown.run();
        return 1;
      }
      if (!arm.checkDeadman()) {
        shutdown.run();
        return 1;
      }
      usleep(kCycleUs);
    }
    const bool clean = shutdown.run();
    std::printf("%s — the arm is limp. Catch the drop and lower "
                "the arm gently to a resting posture.\n",
                clean ? "torque off" : "torque off INCOMPLETE — check "
                                       "the arm before proceeding");
    return clean ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
