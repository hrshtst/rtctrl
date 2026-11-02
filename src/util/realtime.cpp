#include "rtctrl/util/realtime.hpp"

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/prctl.h>

#include <cerrno>
#include <cstring>
#include <string>
#include <utility>

namespace rtctrl::util {
namespace {

constexpr int kMaxFifoPriority = 99;

std::string errnoText(const char* what, int err) {
  return std::string(what) + ": " + std::strerror(err);
}

std::string cpuList(const cpu_set_t& set) {
  std::string out;
  int run_start = -1;
  for (int cpu = 0; cpu <= CPU_SETSIZE; ++cpu) {
    const bool in = cpu < CPU_SETSIZE && CPU_ISSET(cpu, &set);
    if (in && run_start < 0) run_start = cpu;
    if (!in && run_start >= 0) {
      if (!out.empty()) out += ",";
      out += std::to_string(run_start);
      if (cpu - 1 > run_start) out += "-" + std::to_string(cpu - 1);
      run_start = -1;
    }
  }
  return out;
}

// Everything applyRealtimePolicy may change on the calling thread.
struct Saved {
  cpu_set_t affinity;
  int timer_slack_ns = 0;
  int policy = SCHED_OTHER;
  sched_param param{};
};

bool save(Saved* s, std::string* error) {
  CPU_ZERO(&s->affinity);
  int err = pthread_getaffinity_np(pthread_self(), sizeof(s->affinity),
                                   &s->affinity);
  if (err != 0) {
    *error = errnoText("reading CPU affinity", err);
    return false;
  }
  const int slack = prctl(PR_GET_TIMERSLACK, 0, 0, 0, 0);
  if (slack < 0) {
    *error = errnoText("reading timer slack", errno);
    return false;
  }
  s->timer_slack_ns = slack;
  err = pthread_getschedparam(pthread_self(), &s->policy, &s->param);
  if (err != 0) {
    *error = errnoText("reading scheduling policy", err);
    return false;
  }
  return true;
}

void restore(const Saved& s) {
  pthread_setschedparam(pthread_self(), s.policy, &s.param);
  prctl(PR_SET_TIMERSLACK, static_cast<unsigned long>(s.timer_slack_ns), 0,
        0, 0);
  pthread_setaffinity_np(pthread_self(), sizeof(s.affinity), &s.affinity);
}

}  // namespace

std::string validateRealtimePolicy(const RealtimePolicy& policy) {
  if (policy.priority < 0 || policy.priority > kMaxFifoPriority) {
    return "real-time priority must be 0 (off) or 1..99, got " +
           std::to_string(policy.priority);
  }
  if (policy.cpu < -1 || policy.cpu >= CPU_SETSIZE) {
    return "real-time cpu must be -1 (none) or 0.." +
           std::to_string(CPU_SETSIZE - 1) + ", got " +
           std::to_string(policy.cpu);
  }
  if (policy.priority > 0 && policy.cpu < 0) {
    return "a real-time priority requires a dedicated cpu (the bus "
           "receive path busy-polls)";
  }
  return {};
}

bool applyRealtimePolicy(const RealtimePolicy& policy, std::string* error) {
  std::string local;
  std::string& why = error != nullptr ? *error : local;
  why = validateRealtimePolicy(policy);
  if (!why.empty()) return false;
  if (policy.isDefault()) return true;

  Saved saved;
  if (!save(&saved, &why)) return false;
  const auto refuse = [&](std::string reason) {
    restore(saved);
    why = std::move(reason);
    return false;
  };

  if (policy.cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(policy.cpu, &set);
    const int err = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
    if (err != 0) {
      return refuse(errnoText(
          ("pinning to cpu " + std::to_string(policy.cpu)).c_str(), err));
    }
  }
  if (policy.fine_timer_slack &&
      prctl(PR_SET_TIMERSLACK, 1UL, 0, 0, 0) != 0) {
    return refuse(errnoText("setting 1 ns timer slack", errno));
  }
  if (policy.priority > 0) {
    sched_param param{};
    param.sched_priority = policy.priority;
    const int err = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (err != 0) {
      return refuse(errnoText(
          ("SCHED_FIFO priority " + std::to_string(policy.priority)).c_str(),
          err));
    }
  }
  if (policy.lock_memory && mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    return refuse(errnoText("locking memory", errno));
  }
  return true;
}

RealtimeState currentRealtimeState() {
  RealtimeState state;
  sched_param param{};
  if (pthread_getschedparam(pthread_self(), &state.policy, &param) == 0) {
    state.priority = param.sched_priority;
  }
  const int slack = prctl(PR_GET_TIMERSLACK, 0, 0, 0, 0);
  state.timer_slack_ns = slack < 0 ? -1 : slack;
  cpu_set_t set;
  CPU_ZERO(&set);
  if (pthread_getaffinity_np(pthread_self(), sizeof(set), &set) == 0) {
    state.cpus = cpuList(set);
  }
  return state;
}

std::string describeRealtimeState(const RealtimeState& state) {
  std::string policy;
  switch (state.policy) {
    case SCHED_OTHER: policy = "SCHED_OTHER"; break;
    case SCHED_FIFO: policy = "SCHED_FIFO"; break;
    case SCHED_RR: policy = "SCHED_RR"; break;
    default: policy = "policy " + std::to_string(state.policy); break;
  }
  if (state.policy == SCHED_FIFO || state.policy == SCHED_RR) {
    policy += " " + std::to_string(state.priority);
  }
  return policy + ", cpus " + state.cpus + ", timer slack " +
         std::to_string(state.timer_slack_ns) + " ns";
}

}  // namespace rtctrl::util
