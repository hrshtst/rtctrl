#pragma once

#include <array>
#include <string>

namespace rtctrl::model {

// Amplitude profile of a prescribed disturbance, as a fraction of its
// magnitude: zero before onset_s, a raised-cosine rise to 1 over rise_s,
// a hold of hold_s, and a raised-cosine fall back to 0 over fall_s. Both
// ramps have zero slope at their ends, so the force has no steps. Times
// are seconds since the profile clock started (SimArm::startLinkForce).
struct ForceProfile {
  double onset_s = 0.0;
  double rise_s = 0.0;  // > 0
  double hold_s = 0.0;
  double fall_s = 0.0;  // > 0

  double end_s() const;  // onset + rise + hold + fall
  // Fraction of the magnitude in [0, 1]; 0 for a non-finite t.
  double scale(double t) const;
  // Integral of scale over all t: rise/2 + hold + fall/2 [s].
  double impulseScale() const;
};

// Where the force points. World: a fixed world direction. Anchor: from
// the application point toward a fixed world point, e.g. a cable over a
// pulley, so the realized direction follows the attachment point.
enum class ForceDirection { World, Anchor };

// A prescribed external force on one link of the model: magnitude_n
// times the profile, applied at point_link_m (in the link's own frame)
// along the chosen direction. Only the vector of the selected direction
// mode is read; the other stays zero.
struct LinkForceSpec {
  std::string link;
  std::array<double, 3> point_link_m{};
  ForceDirection direction = ForceDirection::World;
  std::array<double, 3> direction_world{};  // World mode; normalized on use
  std::array<double, 3> anchor_world_m{};   // Anchor mode
  double magnitude_n = 0.0;
  ForceProfile profile;
};

inline constexpr const char* kLinkForceFormat = "rtctrl-x7-link-force";
inline constexpr int kLinkForceFormatVersion = 1;
// Anchor mode is undefined closer than this to the application point.
inline constexpr double kAnchorMinDistanceM = 1e-3;
// World mode requires at least this direction norm before normalization.
inline constexpr double kMinDirectionNorm = 1e-9;

// Programmatic validation, shared by the loader, SimArm, and the force
// budget. Throws std::invalid_argument naming the first violation. Model
// membership of the link name is checked where a model is at hand.
void validateLinkForce(const LinkForceSpec& spec);

// Unit vector of the force at the world application point. Returns false
// (and leaves *unit untouched) when the direction is undefined: a World
// direction shorter than kMinDirectionNorm, an Anchor closer than
// kAnchorMinDistanceM, or any non-finite input.
bool linkForceDirection(const LinkForceSpec& spec,
                        const std::array<double, 3>& point_world,
                        std::array<double, 3>* unit);

// Strict TOML loader (format = "rtctrl-x7-link-force", version = 1):
// unknown keys, missing profile keys, non-3-element vectors, and anything
// other than exactly one of direction_world / anchor_world_m are
// rejected, then validateLinkForce runs. Throws std::runtime_error.
LinkForceSpec loadLinkForce(const std::string& path);

}  // namespace rtctrl::model
