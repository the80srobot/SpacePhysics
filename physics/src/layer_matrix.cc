// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "layer_matrix.h"

#include <iomanip>
#include <vector>

namespace vstr {

LayerMatrix::LayerMatrix(
    const std::vector<std::pair<uint32_t, uint32_t>> &layer_pairs) {
  for (const auto lp : layer_pairs) {
    Set(lp.first, lp.second, true);
  }
}

void LayerMatrix::Set(const uint32_t x, const uint32_t y, const bool enabled) {
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

bool LayerMatrix::Check(const uint32_t x, const uint32_t y) const {
  assert(x < 32 && x >= 0);
  assert(y < 32 && y >= 0);
  return (layers_[x] & 1 << y) != 0;
}

std::ostream &operator<<(std::ostream &os, const LayerMatrix &lm) {
  os << "LayerMatrix(\n   ";
  os << std::fixed << std::setfill(' ');
  // Header:
  for (uint32_t u = 0; u < 32; ++u) {
    os << " " << std::setw(2) << u;
  }
  os << "\n   ";
  for (uint32_t u = 0; u < 32; ++u) {
    os << " --";
  }
  os << "\n";

  // Matrix:
  for (uint32_t row = 0; row < 32; ++row) {
    os << std::setw(2) << row << "|";
    for (uint32_t col = 0; col < 32; ++col) {
      os << "  " << lm.Check(row, col);
    }
    os << "\n";
  }

  os << ")";
  return os;
}

}  // namespace vstr