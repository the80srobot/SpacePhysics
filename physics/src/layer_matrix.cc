#include "layer_matrix.h"

#include <vector>

namespace vstr {

LayerMatrix::LayerMatrix(
    const std::vector<std::pair<uint32_t, uint32_t>> &layer_pairs) {
  for (const auto lp : layer_pairs) {
    Set(lp.first, lp.second, true);
  }
}

void LayerMatrix::Set(uint32_t x, uint32_t y, bool enabled) {
  assert(x < 32 && x >= 0);
  assert(y < 32 && y >= 0);
  if (enabled) {
    layers_[x] |= (1 << y);
    layers_[y] |= (1 << x);
  } else {
    layers_[x] &= ~(1 << y);
    layers_[y] &= ~(1 << x);
  }
}

bool LayerMatrix::Check(uint32_t x, uint32_t y) const {
  assert(x < 32 && x >= 0);
  assert(y < 32 && y >= 0);
  return (layers_[x] & 1 << y) != 0;
}

}  // namespace vstr