// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_LAYER_MATRIX
#define VSTR_LAYER_MATRIX

#include <assert.h>

#include <cstdint>
#include <ostream>
#include <utility>

namespace vstr {

class LayerMatrix {
 public:
  explicit LayerMatrix(
      const std::vector<std::pair<uint32_t, uint32_t>> &layer_pairs);

  void Set(uint32_t x, uint32_t y, bool enabled);
  bool Check(uint32_t x, uint32_t y) const;

 private:
  uint32_t layers_[32] = {};
};

std::ostream &operator<<(std::ostream &os, const LayerMatrix &lm);

}  // namespace vstr
#endif