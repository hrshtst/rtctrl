// Pins the frozen scoring formulas of the offline EFL study
// (docs/HISTORY.md (EFL frozen specification)): the least-squares amplitude
// fit, the modal censoring calculus with its comparator ordering, the
// settling metrics, and the four decision-rule booleans.
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <vector>

#include "study_metrics.hpp"

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
namespace study = x7::study;

namespace {

// Sample y(t) over [t0, t1) at 100 Hz.
template <typename F>
void sample(F f, double t0, double t1, std::vector<double>* t,
            std::vector<double>* y) {
  for (double tt = t0; tt < t1 - 1e-12; tt += 0.01) {
    t->push_back(tt);
    y->push_back(f(tt));
  }
}

}  // namespace

TEST_CASE("least-squares fit recovers amplitude at noninteger cycles",
          "[study_metrics]") {
  // 4.5 Hz over a 0.5 s window = 2.25 cycles, arbitrary phase, offset
  std::vector<double> t, y;
  sample([](double tt) {
    return 0.003 * std::cos(2.0 * M_PI * 4.5 * tt + 0.7) + 0.001;
  }, 0.0, 0.5, &t, &y);
  REQUIRE_THAT(study::sineFitAmplitude(t, y, 4.5),
               WithinRel(0.003, 1e-9));
}

TEST_CASE("rate fit recovers a planted exponential envelope",
          "[study_metrics]") {
  const double lambda = 1.2;
  auto ring = [&](double tt) {
    return 0.01 * std::exp(-lambda * tt) *
           std::sin(2.0 * M_PI * 13.0 * tt + 0.3);
  };
  std::vector<double> t1, y1, t2, y2;
  sample(ring, 0.5, 1.5, &t1, &y1);
  sample(ring, 1.5, 2.5, &t2, &y2);
  const double a1 = study::sineFitAmplitude(t1, y1, 13.0);
  const double a2 = study::sineFitAmplitude(t2, y2, 13.0);
  const auto r = study::classifyModal(0.01, a1, a2, 1e-9, 1.0);
  REQUIRE(r.cls == study::ModalClass::Uncensored);
  REQUIRE_THAT(r.rate, WithinRel(-lambda, 0.05));

  // growth flips the sign
  const auto g = study::classifyModal(0.01, a2, a1, 1e-9, 1.0);
  REQUIRE_THAT(g.rate, WithinRel(lambda, 0.05));
}

TEST_CASE("modal censoring classification is the frozen five-branch table",
          "[study_metrics]") {
  const double eps = 1e-9;
  using MC = study::ModalClass;
  REQUIRE(study::classifyModal(1e-5, 1e-5, 1e-6, eps, 1.0).cls ==
          MC::Uncensored);
  REQUIRE(study::classifyModal(1e-5, 1e-5, 1e-10, eps, 1.0).cls ==
          MC::CensoredDecayed);
  // the review round-3 counterexample: decayed then GREW again
  REQUIRE(study::classifyModal(1e-5, 1e-10, 1e-5, eps, 1.0).cls ==
          MC::CensoredGrown);
  REQUIRE(study::classifyModal(1e-5, 1e-10, 1e-10, eps, 1.0).cls ==
          MC::CensoredDecayed);
  REQUIRE(study::classifyModal(1e-10, 1e-10, 1e-10, eps, 1.0).cls ==
          MC::Inconclusive);
}

TEST_CASE("modal comparator ordering is complete", "[study_metrics]") {
  using MC = study::ModalClass;
  auto mk = [](MC cls, double rate = 0.0) {
    study::ModalResult r;
    r.cls = cls;
    r.rate = rate;
    return r;
  };
  const auto unc_neg = mk(MC::Uncensored, -0.8);
  const auto unc_worse = mk(MC::Uncensored, -0.2);
  const auto unc_pos = mk(MC::Uncensored, 0.4);
  const auto dec = mk(MC::CensoredDecayed);
  const auto grown = mk(MC::CensoredGrown);
  const auto inc = mk(MC::Inconclusive);

  // comparator censored-decayed: only censored-decayed EFL is no worse
  REQUIRE(study::modalNoWorse(dec, dec));
  REQUIRE_FALSE(study::modalNoWorse(unc_neg, dec));
  // comparator uncensored: decayed passes; else numeric comparison
  REQUIRE(study::modalNoWorse(dec, unc_worse));
  REQUIRE(study::modalNoWorse(unc_neg, unc_worse));
  REQUIRE_FALSE(study::modalNoWorse(unc_worse, unc_neg));
  // comparator censored-grown: any valid negative/decayed EFL is better
  REQUIRE(study::modalNoWorse(unc_neg, grown));
  REQUIRE(study::modalNoWorse(dec, grown));
  // comparator inconclusive: C4 fails
  REQUIRE_FALSE(study::modalNoWorse(unc_neg, inc));
  // EFL itself must be valid-negative regardless of the comparator
  REQUIRE_FALSE(study::modalNoWorse(unc_pos, grown));
  REQUIRE_FALSE(study::modalNoWorse(grown, grown));
  REQUIRE_FALSE(study::modalNoWorse(inc, grown));
}

TEST_CASE("settling metrics follow the frozen definitions",
          "[study_metrics]") {
  // error decays through the 0.01 band at t = 5.3 s (t_end = 5.0)
  std::vector<double> t, e;
  sample([](double tt) { return 0.05 * std::exp(-(tt - 5.0) * 5.4); },
         5.0, 9.0, &t, &e);
  const auto s = study::settlingMetrics(t, e, 5.0, 0.01, 0.5, 1.0);
  REQUIRE(s.settled);
  REQUIRE_THAT(s.settle_time, WithinAbs(0.3, 0.02));
  REQUIRE(s.steady_state_err < 1e-6);

  // never inside the band: not settled, honest steady-state error
  std::vector<double> t2, e2;
  sample([](double) { return 0.02; }, 5.0, 9.0, &t2, &e2);
  const auto s2 = study::settlingMetrics(t2, e2, 5.0, 0.01, 0.5, 1.0);
  REQUIRE_FALSE(s2.settled);
  REQUIRE_THAT(s2.steady_state_err, WithinAbs(0.02, 1e-12));
}

TEST_CASE("the first qualifying dwell latches through later excursions",
          "[study_metrics]") {
  // in band [4.53, 5.20) — a full 0.5 s dwell qualifies at 5.03 —
  // then an excursion [5.20, 5.60), then reacquisition. The frozen
  // definition keeps the FIRST dwell (review finding: the old code
  // reported the final reacquisition instead).
  std::vector<double> t, e;
  sample([](double tt) {
    if (tt < 4.53) return 0.05;
    if (tt < 5.20) return 0.005;
    if (tt < 5.60) return 0.05;
    return 0.002;
  }, 4.0, 8.0, &t, &e);
  const auto s = study::settlingMetrics(t, e, 4.0, 0.01, 0.5, 1.0);
  REQUIRE(s.settled);
  REQUIRE_THAT(s.settle_time, WithinAbs(0.53, 0.02));
}

TEST_CASE("decision-rule booleans flip on each criterion independently",
          "[study_metrics]") {
  auto pass_case = [](const char* id, bool in_s) {
    study::CasePair c;
    c.id = id;
    c.in_s = in_s;
    c.rms_efl = 0.5;
    c.rms_cmp = 1.0;  // 50% better: counts toward C1
    c.peak_err_efl = c.peak_err_cmp = 0.01;
    c.peak_tau_efl = c.peak_tau_cmp = 1.0;
    c.sat_efl = c.sat_cmp = 0;
    c.efl_complete_finite = true;
    return c;
  };
  std::vector<study::CasePair> cases;
  for (const char* id : {"R1", "R2", "L1", "L2", "D+", "D-"}) {
    cases.push_back(pass_case(id, true));
  }
  cases.push_back(pass_case("F4", false));
  cases.push_back(pass_case("F13", false));

  REQUIRE(study::criterionC1(cases));
  REQUIRE(study::criterionC2(cases));
  REQUIRE(study::criterionC3(cases));

  // C1: only 3 of 6 improved -> fail (majority means >= 4)
  auto c1_fail = cases;
  for (int k = 0; k < 3; ++k) c1_fail[k].rms_efl = 0.9;
  REQUIRE_FALSE(study::criterionC1(c1_fail));
  auto c1_edge = cases;
  for (int k = 0; k < 2; ++k) c1_edge[k].rms_efl = 0.9;
  REQUIRE(study::criterionC1(c1_edge));  // 4 of 6 still passes

  // C2: a flexible-case saturation regression fails (review round 3)
  auto c2_fail = cases;
  c2_fail[7].sat_efl = 1;  // F13
  REQUIRE_FALSE(study::criterionC2(c2_fail));
  auto c2_peak = cases;
  c2_peak[0].peak_tau_efl = 1.2;  // > 1.1x comparator
  REQUIRE_FALSE(study::criterionC2(c2_peak));

  // C3: a flexible case failing to complete fails (review round 3)
  auto c3_fail = cases;
  c3_fail[6].efl_complete_finite = false;  // F4
  std::vector<study::CasePair> delayed = {c3_fail[2], c3_fail[3],
                                          c3_fail[4], c3_fail[5],
                                          c3_fail[6], c3_fail[7]};
  REQUIRE_FALSE(study::criterionC3(delayed));

  // C4 composes the modal ordering
  study::ModalResult neg;
  neg.cls = study::ModalClass::Uncensored;
  neg.rate = -1.0;
  study::ModalResult pos = neg;
  pos.rate = 0.5;
  REQUIRE(study::criterionC4(neg, neg, neg, neg));
  REQUIRE_FALSE(study::criterionC4(neg, neg, pos, neg));
}
