// The offline learning step between follow trials: from a trial's follow
// log and the offset it ran with, the next command offset (iterative
// learning control or feedback-error learning), with its lineage. Never
// opens the bus. See learn/follow_learn.hpp; usage: x7_follow_learn --help.

#include <cstdio>
#include <exception>

#include "learn/follow_learn.hpp"

namespace learn = x7::learn;

int main(int argc, char* argv[]) {
  try {
    const learn::LearnCli cli = learn::parseLearnCli(argc, argv);
    if (cli.help) {
      std::fputs(learn::learnUsage(cli.command).c_str(), stdout);
      return 0;
    }
    const learn::LearnResult result = learn::runLearnCommand(cli);
    std::fputs(learn::formatSummary(result).c_str(), stdout);
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "x7_follow_learn: %s\n", error.what());
    return 1;
  }
}
