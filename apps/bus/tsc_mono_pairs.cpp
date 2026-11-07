// Bracketed TSC and CLOCK_MONOTONIC pairs for the timer service diagnostics
// (tools/timer_service.py, docs/hardware/control-rate.md): an independent
// check of the conversion of the TSC deadline MSR's values to the trace's
// monotonic time.
//
// usage: tsc_mono_pairs --output FILE [--cpu N] [--interval-ms I]
//                       [--duration-ms D] [--tries K]
//
// Every interval it takes K reads of (CLOCK_MONOTONIC, the TSC by rdtscp,
// CLOCK_MONOTONIC) and keeps the one with the tightest bracket, appending
// "mono_before_ns,tsc,mono_after_ns,cpu" to FILE, where cpu is the CPU that
// read the TSC (rdtscp's TSC_AUX). With --cpu it runs pinned to that CPU and
// drops a read taken elsewhere. It runs for --duration-ms (0: until SIGINT
// or SIGTERM), then exits 0; 2 for a bad argument, 1 for an I/O or
// affinity error. Each run first appends a "# tsc_mono_pairs ..." line. It
// never touches the bus or a servo. x86-64 only.

#include <x86intrin.h>

#include <sched.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

volatile sig_atomic_t g_stop = 0;

void onSignal(int) { g_stop = 1; }

std::int64_t monoNs() {
  timespec t{};
  clock_gettime(CLOCK_MONOTONIC, &t);
  return static_cast<std::int64_t>(t.tv_sec) * 1'000'000'000 + t.tv_nsec;
}

bool parseInt(const char* s, long lo, long hi, long& out) {
  char* end = nullptr;
  errno = 0;
  const long v = std::strtol(s, &end, 10);
  if (errno != 0 || end == s || *end != '\0' || v < lo || v > hi) return false;
  out = v;
  return true;
}

int usage() {
  std::fprintf(stderr,
               "usage: tsc_mono_pairs --output FILE [--cpu N] [--interval-ms I] "
               "[--duration-ms D] [--tries K]\n");
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  std::string output;
  long cpu = -1, interval_ms = 10, duration_ms = 0, tries = 16;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (i + 1 >= argc) return usage();
    const char* v = argv[++i];
    bool ok = true;
    if (a == "--output") {
      output = v;
    } else if (a == "--cpu") {
      ok = parseInt(v, 0, CPU_SETSIZE - 1, cpu);
    } else if (a == "--interval-ms") {
      ok = parseInt(v, 1, 60'000, interval_ms);
    } else if (a == "--duration-ms") {
      ok = parseInt(v, 0, 86'400'000, duration_ms);
    } else if (a == "--tries") {
      ok = parseInt(v, 1, 1'000, tries);
    } else {
      ok = false;
    }
    if (!ok) return usage();
  }
  if (output.empty()) return usage();
  if (cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(static_cast<int>(cpu), &set);
    if (sched_setaffinity(0, sizeof set, &set) != 0) {
      std::fprintf(stderr, "tsc_mono_pairs: cannot run on CPU %ld: %s\n", cpu, std::strerror(errno));
      return 1;
    }
  }
  std::FILE* f = std::fopen(output.c_str(), "a");
  if (!f) {
    std::fprintf(stderr, "tsc_mono_pairs: cannot open %s: %s\n", output.c_str(), std::strerror(errno));
    return 1;
  }
  struct sigaction sa {};
  sa.sa_handler = onSignal;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
  std::fprintf(f, "# tsc_mono_pairs cpu=%ld interval_ms=%ld tries=%ld pid=%d\n", cpu, interval_ms, tries,
               static_cast<int>(getpid()));
  std::fflush(f);
  const std::int64_t start = monoNs();
  timespec next{};
  clock_gettime(CLOCK_MONOTONIC, &next);
  while (!g_stop) {
    std::int64_t best_a = 0, best_b = 0;
    std::uint64_t best_t = 0;
    unsigned best_cpu = 0;
    bool have = false;
    for (long k = 0; k < tries; ++k) {
      unsigned aux = 0;
      const std::int64_t a = monoNs();
      const std::uint64_t t = __rdtscp(&aux);
      const std::int64_t b = monoNs();
      const unsigned on = aux & 0xfffu;
      if (cpu >= 0 && static_cast<long>(on) != cpu) continue;
      if (!have || b - a < best_b - best_a) {
        best_a = a, best_b = b, best_t = t, best_cpu = on, have = true;
      }
    }
    if (have &&
        std::fprintf(f, "%lld,%llu,%lld,%u\n", static_cast<long long>(best_a),
                     static_cast<unsigned long long>(best_t), static_cast<long long>(best_b), best_cpu) < 0) {
      std::fprintf(stderr, "tsc_mono_pairs: write failed\n");
      std::fclose(f);
      return 1;
    }
    std::fflush(f);
    if (duration_ms > 0 && monoNs() - start >= duration_ms * 1'000'000) break;
    next.tv_nsec += interval_ms * 1'000'000;
    while (next.tv_nsec >= 1'000'000'000) {
      next.tv_nsec -= 1'000'000'000;
      ++next.tv_sec;
    }
    while (!g_stop && clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr) == EINTR) {
    }
  }
  return std::fclose(f) == 0 ? 0 : 1;
}
