// End-to-end identification regression: a full dwell sequence against
// the TwoMassArm fixture must recover the PLANTED transmission modes
// through the direct-ratio FRF — the same estimator the offline
// analysis applies — before any hardware run. The anchor is the ZERO
// pose: the real model's gravity vanishes there, so the gravity-free
// fixture is never fed a fictitious gravity feedforward (C6b).
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

#include "ident_common.hpp"
#include "two_mass_arm.hpp"

using Catch::Approx;
namespace arm = rtctrl::arm;
namespace model = rtctrl::model;
using model::kCanonicalDof;

namespace {

constexpr const char* kModelPath = "models/crane_x7/crane_x7.ztk";

struct FrfPoint {
  double f = 0.0;
  std::complex<double> h;  // q_link / tau_meas [rad/Nm]
};

// Complex SDOF fit on a fine local grid: H(f) ~ c*G(f) + d with
// G = 1/(f_n^2 - f^2 + 2 j zeta f_n f); c, d solved by linear least
// squares per (f_n, zeta) candidate, best residual wins.
struct SdofFit {
  double f_n = 0.0;
  double zeta = 0.0;
};

SdofFit fitSdof(const std::vector<FrfPoint>& pts, double f_lo,
                double f_hi) {
  SdofFit best;
  double best_res = 1e300;
  for (double f_n = f_lo; f_n <= f_hi; f_n += 0.02) {
    for (double zeta = 0.008; zeta <= 0.2; zeta *= 1.06) {
      std::complex<double> sgg = 0.0, sg = 0.0, shg = 0.0, sh = 0.0;
      const double n = static_cast<double>(pts.size());
      std::vector<std::complex<double>> g(pts.size());
      for (std::size_t k = 0; k < pts.size(); ++k) {
        const double f = pts[k].f;
        g[k] = 1.0 / std::complex<double>(f_n * f_n - f * f,
                                          2.0 * zeta * f_n * f);
        sgg += g[k] * std::conj(g[k]);
        sg += g[k];
        shg += pts[k].h * std::conj(g[k]);
        sh += pts[k].h;
      }
      // normal equations for min sum |h - cG - d|^2
      const std::complex<double> det = sgg * n - sg * std::conj(sg);
      if (std::abs(det) < 1e-18) continue;
      const std::complex<double> c = (shg * n - sh * std::conj(sg)) / det;
      const std::complex<double> d = (sgg * sh - sg * shg) / det;
      double res = 0.0;
      for (std::size_t k = 0; k < pts.size(); ++k) {
        res += std::norm(pts[k].h - c * g[k] - d);
      }
      if (res < best_res) {
        best_res = res;
        best = {f_n, zeta};
      }
    }
  }
  return best;
}

// Run a refinement-style grid on one joint of the fixture and return
// the direct-ratio FRF points from the completed dwells.
std::vector<FrfPoint> probeFixture(int joint,
                                   const std::vector<double>& freqs,
                                   x7::IdentRun::Outcome* outcome,
                                   std::FILE* log = nullptr) {
  model::ChainModel chain(kModelPath);
  model::JointMap map(chain);
  x7::TwoMassArm robot;  // planted: joint 1 4.5 Hz z0.03, joint 5 13 Hz z0.05

  x7::IdentRun::Options opt;
  opt.probe_joint = joint;
  for (const double f : freqs) opt.dwells.push_back({f, 0.15, 1});
  opt.anchor.assign(kCanonicalDof, 0.0);
  opt.tau_max.assign(kCanonicalDof, 4.0);
  opt.gravity_free_plant = true;  // C6b: TwoMassArm carries no gravity
  x7::IdentRun run(chain, map, opt, log);

  REQUIRE(robot.setMode(arm::ControlMode::Current));
  REQUIRE(robot.activate());
  CHECK_FALSE(arm::run(robot, run, 150.0, &run));  // ends by Done veto
  *outcome = run.outcome();
  if (*outcome != x7::IdentRun::Outcome::Completed) {
    std::printf("ident_sim: outcome %d fault '%s'\n",
                static_cast<int>(*outcome), run.faultReason().c_str());
    for (const auto& r : run.results()) {
      std::printf("  dwell %.2f Hz: started %d completed %d note '%s'\n",
                  r.spec.freq_hz, r.started, r.completed, r.note.c_str());
    }
  }

  std::vector<FrfPoint> pts;
  for (const auto& r : run.results()) {
    if (!r.completed) continue;
    const std::complex<double> q(r.resp[joint].re, r.resp[joint].im);
    const std::complex<double> tau(r.tau_meas.re, r.tau_meas.im);
    REQUIRE(std::abs(tau) > 1e-6);
    pts.push_back({r.spec.freq_hz, q / tau});
  }
  return pts;
}

}  // namespace

TEST_CASE("ident pipeline recovers the 4.5 Hz zeta 0.03 mode on joint 1",
          "[ident_sim]") {
  x7::IdentRun::Outcome outcome;
  const std::vector<double> grid = {3.6,  3.9, 4.2, 4.35, 4.5,
                                    4.65, 4.8, 5.1, 5.4};
  const auto pts = probeFixture(1, grid, &outcome);
  CHECK(outcome == x7::IdentRun::Outcome::Completed);
  REQUIRE(pts.size() == grid.size());  // no dwell lost to a monitor
  const auto fit = fitSdof(pts, 3.5, 5.5);
  INFO("fitted " << fit.f_n << " Hz, zeta " << fit.zeta);
  CHECK(fit.f_n == Approx(4.5).margin(0.3));
  CHECK(fit.zeta > 0.015);
  CHECK(fit.zeta < 0.06);
}

TEST_CASE("ident pipeline recovers the 13 Hz zeta 0.05 mode on joint 5",
          "[ident_sim]") {
  x7::IdentRun::Outcome outcome;
  const std::vector<double> grid = {11.5, 12.1, 12.7, 13.0,
                                    13.3, 13.9, 14.5};
  const auto pts = probeFixture(5, grid, &outcome);
  CHECK(outcome == x7::IdentRun::Outcome::Completed);
  REQUIRE(pts.size() == grid.size());
  const auto fit = fitSdof(pts, 11.0, 15.0);
  INFO("fitted " << fit.f_n << " Hz, zeta " << fit.zeta);
  CHECK(fit.f_n == Approx(13.0).margin(0.3));
  CHECK(fit.zeta > 0.025);
  CHECK(fit.zeta < 0.1);
}

TEST_CASE("ident CSV and JSON sidecar are written end to end",
          "[ident_sim]") {
  std::FILE* log =
      x7::openIdentCsvLog("build/ident_sim_test.csv", "label=test");
  REQUIRE(log != nullptr);

  model::ChainModel chain(kModelPath);
  model::JointMap map(chain);
  x7::TwoMassArm robot;
  x7::IdentRun::Options opt;
  opt.probe_joint = 1;
  opt.dwells = {{4.5, 0.15, 1}};
  opt.anchor.assign(kCanonicalDof, 0.0);
  opt.tau_max.assign(kCanonicalDof, 4.0);
  opt.gravity_free_plant = true;
  x7::IdentRun run(chain, map, opt, log);
  REQUIRE(robot.setMode(arm::ControlMode::Current));
  REQUIRE(robot.activate());
  CHECK_FALSE(arm::run(robot, run, 60.0, &run));
  std::fclose(log);
  REQUIRE(run.finishedCleanly());
  REQUIRE(x7::writeDwellJson("build/ident_sim_test.json", opt, run));

  // the CSV has the '#' semantics line, the header, and one row per
  // controlled cycle
  std::FILE* f = std::fopen("build/ident_sim_test.csv", "r");
  REQUIRE(f != nullptr);
  long lines = 0;
  int c;
  while ((c = std::fgetc(f)) != EOF) {
    if (c == '\n') ++lines;
  }
  std::fclose(f);
  CHECK(lines > 500);  // ~10 s of 100 Hz telemetry plus two header lines
}
