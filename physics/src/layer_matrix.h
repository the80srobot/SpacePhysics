#ifndef VSTR_LAYER_MATRIX
#define VSTR_LAYER_MATRIX

#include <assert.h>

#include <cstdint>
#include <utility>

namespace vstr {

class LayerMatrix {
 public:
  LayerMatrix(const std::vector<std::pair<uint32_t, uint32_t>>& layer_pairs);

  void Set(uint32_t x, uint32_t y, bool enabled);
  bool Check(uint32_t x, uint32_t y) const;

 private:
  uint32_t layers_[32];
};

}  // namespace vstr
#endif