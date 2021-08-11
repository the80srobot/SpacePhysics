#include <cmath>

namespace vstr {

inline bool FloatEq(const float x, const float y,
                    const float epsilon = 0.005f) {
  return std::fabs(x - y) < epsilon;
}

}  // namespace vstr
