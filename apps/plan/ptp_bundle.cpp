#include "plan/ptp_bundle.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <vector>

#if defined(__linux__)
#include <fcntl.h>
#include <linux/fs.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace x7::ptp {

namespace fs = std::filesystem;

namespace {

constexpr const char* kBundleFormat = "rtctrl-ptp-bundle";
constexpr int kBundleFormatVersion = 1;

bool pathExists(const fs::path& path) {
  std::error_code error;
  const auto status = fs::symlink_status(path, error);
  if (error == std::errc::no_such_file_or_directory) return false;
  if (error) {
    throw std::runtime_error("cannot inspect bundle path '" + path.string() +
                             "': " + error.message());
  }
  return status.type() != fs::file_type::not_found;
}

void requireNewBundlePath(const fs::path& path) {
  if (pathExists(path)) {
    throw std::runtime_error("bundle directory already exists: " +
                             path.string() + "; choose a new path");
  }
}

bool safeRelativePath(const fs::path& path) {
  if (path.empty() || path.is_absolute() || path.has_root_path()) return false;
  for (const auto& part : path.lexically_normal()) {
    if (part == "..") return false;
  }
  return true;
}

void copyRegularFile(const fs::path& source, const fs::path& destination) {
  std::error_code error;
  if (!fs::is_regular_file(source, error) || error) {
    throw std::runtime_error("bundle dependency is not a regular file: " +
                             source.string());
  }
  fs::create_directories(destination.parent_path());
  if (!fs::copy_file(source, destination, fs::copy_options::none, error)) {
    throw std::runtime_error("cannot copy '" + source.string() + "' to '" +
                             destination.string() + "': " + error.message());
  }
}

void copyModelDependencies(const fs::path& source_model,
                           const fs::path& destination_root) {
  const fs::path source_root = source_model.parent_path();
  const fs::path model_name = source_model.filename();
  if (model_name.empty()) {
    throw std::runtime_error("model path has no filename: " +
                             source_model.string());
  }

  std::set<fs::path> pending{model_name};
  std::set<fs::path> copied;
  while (!pending.empty()) {
    const fs::path relative = *pending.begin();
    pending.erase(pending.begin());
    if (!safeRelativePath(relative)) {
      throw std::runtime_error("model dependency escapes its directory: " +
                               relative.string());
    }
    if (!copied.insert(relative).second) continue;

    const fs::path source = source_root / relative;
    copyRegularFile(source, destination_root / relative);
    if (source.extension() != ".ztk") continue;

    std::ifstream input(source);
    if (!input) {
      throw std::runtime_error("cannot inspect model dependencies in " +
                               source.string());
    }
    std::string line;
    while (std::getline(input, line)) {
      const auto first = line.find_first_not_of(" \t");
      if (first == std::string::npos ||
          line.compare(first, 7, "import:") != 0) {
        continue;
      }
      std::istringstream fields(line.substr(first + 7));
      std::string imported;
      fields >> std::quoted(imported);
      if (imported.empty()) {
        throw std::runtime_error("empty import in model file " +
                                 source.string());
      }
      const fs::path dependency = fs::path(imported).lexically_normal();
      if (!safeRelativePath(dependency)) {
        throw std::runtime_error("non-portable model import '" + imported +
                                 "' in " + source.string());
      }
      pending.insert(dependency);
    }
  }
}

class Sha256 {
 public:
  Sha256() = default;

  void update(const unsigned char* data, std::size_t size) {
    total_bytes_ += size;
    while (size > 0) {
      const std::size_t count =
          std::min(size, block_.size() - block_size_);
      std::memcpy(block_.data() + block_size_, data, count);
      block_size_ += count;
      data += count;
      size -= count;
      if (block_size_ == block_.size()) {
        transform(block_.data());
        block_size_ = 0;
      }
    }
  }

  std::string finish() {
    const std::uint64_t bit_count = total_bytes_ * 8;
    block_[block_size_++] = 0x80;
    if (block_size_ > 56) {
      while (block_size_ < block_.size()) block_[block_size_++] = 0;
      transform(block_.data());
      block_size_ = 0;
    }
    while (block_size_ < 56) block_[block_size_++] = 0;
    for (int i = 7; i >= 0; --i) {
      block_[block_size_++] =
          static_cast<unsigned char>(bit_count >> (8 * i));
    }
    transform(block_.data());

    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (const auto word : state_) output << std::setw(8) << word;
    return output.str();
  }

 private:
  static std::uint32_t rotateRight(std::uint32_t value, int count) {
    return (value >> count) | (value << (32 - count));
  }

  void transform(const unsigned char* data) {
    static constexpr std::array<std::uint32_t, 64> constants = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
        0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
        0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
        0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
        0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
        0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
        0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
        0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    std::array<std::uint32_t, 64> words{};
    for (int i = 0; i < 16; ++i) {
      words[i] = (static_cast<std::uint32_t>(data[4 * i]) << 24) |
                 (static_cast<std::uint32_t>(data[4 * i + 1]) << 16) |
                 (static_cast<std::uint32_t>(data[4 * i + 2]) << 8) |
                 static_cast<std::uint32_t>(data[4 * i + 3]);
    }
    for (int i = 16; i < 64; ++i) {
      const std::uint32_t s0 = rotateRight(words[i - 15], 7) ^
                               rotateRight(words[i - 15], 18) ^
                               (words[i - 15] >> 3);
      const std::uint32_t s1 = rotateRight(words[i - 2], 17) ^
                               rotateRight(words[i - 2], 19) ^
                               (words[i - 2] >> 10);
      words[i] = words[i - 16] + s0 + words[i - 7] + s1;
    }

    std::uint32_t a = state_[0];
    std::uint32_t b = state_[1];
    std::uint32_t c = state_[2];
    std::uint32_t d = state_[3];
    std::uint32_t e = state_[4];
    std::uint32_t f = state_[5];
    std::uint32_t g = state_[6];
    std::uint32_t h = state_[7];
    for (int i = 0; i < 64; ++i) {
      const std::uint32_t sum1 = rotateRight(e, 6) ^ rotateRight(e, 11) ^
                                 rotateRight(e, 25);
      const std::uint32_t choice = (e & f) ^ (~e & g);
      const std::uint32_t temp1 =
          h + sum1 + choice + constants[i] + words[i];
      const std::uint32_t sum0 = rotateRight(a, 2) ^ rotateRight(a, 13) ^
                                 rotateRight(a, 22);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temp2 = sum0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
    }
    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
  }

  std::array<std::uint32_t, 8> state_ = {
      0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
      0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  std::array<unsigned char, 64> block_{};
  std::size_t block_size_ = 0;
  std::uint64_t total_bytes_ = 0;
};

std::string sha256File(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot hash " + path.string());
  Sha256 hash;
  std::array<unsigned char, 65536> buffer{};
  while (input) {
    input.read(reinterpret_cast<char*>(buffer.data()), buffer.size());
    const auto count = input.gcount();
    if (count > 0) hash.update(buffer.data(), static_cast<std::size_t>(count));
  }
  if (!input.eof()) throw std::runtime_error("cannot read " + path.string());
  return hash.finish();
}

}  // namespace

BundleWorkspace::BundleWorkspace(const fs::path& target)
    : target_(fs::absolute(target).lexically_normal()) {
  if (target.empty() || target_.filename().empty()) {
    throw std::runtime_error("--bundle requires a directory path");
  }
  requireNewBundlePath(target_);
  fs::create_directories(target_.parent_path());
  for (int attempt = 0; attempt < 1000; ++attempt) {
    staging_ = target_.parent_path() /
               ("." + target_.filename().string() + ".tmp." +
#if defined(__linux__)
                std::to_string(static_cast<long long>(getpid())) + "." +
#else
                std::string("process.") +
#endif
                std::to_string(attempt));
    std::error_code error;
    if (fs::create_directory(staging_, error)) return;
    if (error && error != std::errc::file_exists) {
      throw std::runtime_error("cannot create bundle staging directory '" +
                               staging_.string() + "': " + error.message());
    }
  }
  throw std::runtime_error("cannot allocate a bundle staging directory");
}

BundleWorkspace::~BundleWorkspace() {
  if (published_ || staging_.empty()) return;
  std::error_code ignored;
  fs::remove_all(staging_, ignored);
}

void BundleWorkspace::publish() {
  requireNewBundlePath(target_);
#if defined(__linux__) && defined(SYS_renameat2)
  if (syscall(SYS_renameat2, AT_FDCWD, staging_.c_str(), AT_FDCWD,
              target_.c_str(), RENAME_NOREPLACE) != 0) {
    const int error = errno;
    if (error == EEXIST) {
      throw std::runtime_error("bundle directory already exists: " +
                               target_.string() + "; choose a new path");
    }
    throw std::system_error(error, std::generic_category(),
                            "cannot publish bundle " + target_.string());
  }
#else
  fs::rename(staging_, target_);
#endif
  published_ = true;
}

PreparedBundle prepareBundle(const fs::path& staging,
                             const fs::path& source_config,
                             const Config& effective_config) {
  copyRegularFile(source_config, staging / "source.toml");
  const fs::path bundled_model =
      fs::path("model") / effective_config.model_path.filename();
  copyModelDependencies(effective_config.model_path, staging / "model");

  const fs::path bundled_config = staging / "plan.toml";
  std::ofstream output(bundled_config, std::ios::out | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot write " + bundled_config.string());
  }
  output << serializeEffectiveConfig(effective_config, bundled_model,
                                     "trajectory.zvs");
  output.close();
  if (!output) {
    throw std::runtime_error("cannot finish " + bundled_config.string());
  }
  return {bundled_config, staging / "trajectory.zvs"};
}

void writeBundleManifest(const fs::path& root,
                         const std::string& rtctrl_version,
                         const std::string& git_commit, bool git_dirty) {
  std::vector<fs::path> files;
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    if (entry.is_regular_file() && entry.path().filename() != "manifest.toml") {
      files.push_back(fs::relative(entry.path(), root));
    }
  }
  std::sort(files.begin(), files.end());

  const fs::path manifest_path = root / "manifest.toml";
  std::ofstream output(manifest_path, std::ios::out | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot write " + manifest_path.string());
  }
  output << "format = " << quoteTomlString(kBundleFormat)
         << "\nformat_version = " << kBundleFormatVersion
         << "\nrtctrl_version = " << quoteTomlString(rtctrl_version)
         << "\ngit_commit = " << quoteTomlString(git_commit)
         << "\ngit_dirty = " << (git_dirty ? "true" : "false") << '\n';
  for (const auto& relative : files) {
    const fs::path path = root / relative;
    output << "\n[[files]]\npath = "
           << quoteTomlString(relative.generic_string())
           << "\nsize = " << fs::file_size(path)
           << "\nsha256 = " << quoteTomlString(sha256File(path)) << '\n';
  }
  output.close();
  if (!output) {
    throw std::runtime_error("cannot finish " + manifest_path.string());
  }
}

}  // namespace x7::ptp
