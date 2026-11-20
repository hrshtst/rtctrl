// Offline replay of a probe run's reply stamps through the adapter phase
// estimator (rtctrl/hw/adapter_phase.hpp). Read-only; no bus, no hardware.
//
//   adapter_phase_replay RUN.csv [--acquisition-ns N] [--offline-grid]
//
// RUN.csv is x7_bus_timing's per-cycle log. Its first_reply_ns stamps before
// N (default 2e9 ns) are the acquisition; the estimator locks on their fit at
// N and then tracks every later stamp as the offline analysis's replay did:
// the nearest lattice point, no lock supervision (adapter_timing_replay.py,
// review-notes/2026-11-20-adapter-referenced-timing-proposal). The host-grid
// runs have no intended completion, so the referenced tracking that the
// planner would use cannot be replayed on them; it is exercised in
// tests/unit/adapter_phase_test.cpp against synthetic ground truth.
// --offline-grid fits on that analysis's slope grid (-80 to +20 ppm) instead
// of the default +-200 ppm, so that the fit is the analysis's own.
//
// Output: a comment line with the fit, then one row per tracked stamp: the
// stamp, the anchor and period as they stood before it, the predicted
// completion it belongs to (anchor + round((stamp - anchor - D) / period) *
// period), and the outcome. Doubles print with 17 significant digits.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "rtctrl/hw/adapter_phase.hpp"

namespace ad = rtctrl::hw::adapter;

namespace {

const char* outcomeName(ad::Observation o) {
  switch (o) {
    case ad::Observation::kNone: return "none";
    case ad::Observation::kGated: return "gated";
    case ad::Observation::kLate: return "late";
    case ad::Observation::kEarly: return "early";
    case ad::Observation::kInvalid: return "invalid";
    case ad::Observation::kStored: return "stored";
    case ad::Observation::kDropped: return "dropped";
  }
  return "?";
}

bool readStamps(const char* path, std::vector<std::int64_t>* out) {
  std::ifstream in(path);
  if (!in) return false;
  std::string line;
  if (!std::getline(in, line)) return false;
  int col = -1;
  {
    std::stringstream ss(line);
    std::string name;
    for (int i = 0; std::getline(ss, name, ','); ++i) {
      if (name == "first_reply_ns") col = i;
    }
  }
  if (col < 0) return false;
  while (std::getline(in, line)) {
    const char* p = line.c_str();
    for (int i = 0; i < col && p != nullptr; ++i) {
      p = std::strchr(p, ',');
      if (p != nullptr) ++p;
    }
    if (p == nullptr) return false;
    out->push_back(std::strtoll(p, nullptr, 10));
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: adapter_phase_replay RUN.csv [--acquisition-ns N] [--offline-grid]\n");
    return 2;
  }
  std::int64_t acquisition_ns = 2000000000;
  bool offline_grid = false;
  for (int i = 2; i < argc; ++i) {
    if (std::strcmp(argv[i], "--acquisition-ns") == 0 && i + 1 < argc) {
      acquisition_ns = std::strtoll(argv[++i], nullptr, 10);
    } else if (std::strcmp(argv[i], "--offline-grid") == 0) {
      offline_grid = true;
    } else {
      std::fprintf(stderr, "adapter_phase_replay: unknown argument %s\n", argv[i]);
      return 2;
    }
  }
  std::vector<std::int64_t> all;
  if (!readStamps(argv[1], &all)) {
    std::fprintf(stderr, "adapter_phase_replay: cannot read first_reply_ns from %s\n", argv[1]);
    return 2;
  }
  ad::EstimatorConfig cfg;
  cfg.supervise = false;  // the analysis's tracker had no lock supervision
  if (offline_grid) {
    cfg.fit.slope_min_ppm = -80.0;
    cfg.fit.slope_max_ppm = 20.0;
  }
  std::vector<std::int64_t> acq, later;
  for (std::int64_t s : all) {
    if (s <= 0) continue;  // no reply
    (s < acquisition_ns ? acq : later).push_back(s);
  }
  const ad::LatticeFit fit = ad::fitLattice(acq.data(), acq.size(), cfg.fit);
  ad::AdapterPhaseEstimator est(cfg);
  est.lockFrom(fit.slope_ppm, fit.phase0_us, acquisition_ns);
  std::printf("# fit over %zu stamps before %lld ns: slope_ppm %.17g phase0_us %.17g share %.17g\n",
              acq.size(), static_cast<long long>(acquisition_ns), fit.slope_ppm, fit.phase0_us,
              fit.share);
  std::printf("stamp_ns,anchor_ns,period_ns,predicted_ns,outcome\n");
  for (std::int64_t s : later) {
    const double a = est.anchorNs(), p = est.periodNs();
    const double n = std::nearbyint((static_cast<double>(s) - a - cfg.delay_ns) / p);
    const double pred = a + n * p;
    const ad::Observation o = est.observeNearest(s);
    std::printf("%lld,%.17g,%.17g,%.17g,%s\n", static_cast<long long>(s), a, p, pred, outcomeName(o));
  }
  const auto& c = est.counts();
  std::fprintf(stderr, "adapter_phase_replay: %zu stamps tracked: gated %llu, late %llu, early %llu, invalid %llu\n",
               later.size(), static_cast<unsigned long long>(c.gated), static_cast<unsigned long long>(c.late),
               static_cast<unsigned long long>(c.early), static_cast<unsigned long long>(c.invalid));
  return 0;
}
