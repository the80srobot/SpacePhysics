// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "interval_tree.h"

namespace vstr {

bool operator<(const Interval& x, const Interval& y) {
  if (x.low == y.low) {
    return x.high < y.high;
  }
  return x.low < y.low;
}

bool operator>(const Interval& x, const Interval& y) {
  if (x.low == y.low) {
    return x.high > y.high;
  }
  return x.low > y.low;
}

bool operator==(const Interval& x, const Interval& y) {
  return x.low == y.low && x.high == y.high;
}

std::ostream& operator<<(std::ostream& os, const Interval& interval) {
  return os << "[" << interval.low << ", " << interval.high << ")";
}

}  // namespace vstr