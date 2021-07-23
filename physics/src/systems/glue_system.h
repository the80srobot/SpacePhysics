// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_GLUE_SYSTEM
#define VSTR_GLUE_SYSTEM

#include "component_data.h"

namespace vstr {

class GlueSystem {
 public:
  void UpdateGluedMotion(const std::vector<Position> &positions,
                         const std::vector<Glue> &glue,
                         const std::vector<Flags> &flags,
                         std::vector<Motion> &motion);

 private:
};
}  // namespace vstr

#endif
