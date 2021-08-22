// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_ROCKET
#define VSTR_ROCKET

#include <absl/status/status.h>
#include <absl/status/statusor.h>
#include <absl/types/span.h>

#include "types/required_components.h"

namespace vstr {

absl::Status ConvertRocketBurnToAcceleration(const float dt,
                                             absl::Span<Event> input,
                                             std::vector<Mass> &mass,
                                             std::vector<Rocket> &rockets);

absl::Status ApplyRocketRefuel(const Event &event, std::vector<Mass> &mass,
                               std::vector<Rocket> &rockets);

}  // namespace vstr

#endif