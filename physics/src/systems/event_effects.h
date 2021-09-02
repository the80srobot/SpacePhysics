// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef VSTR_SYSTEMS_EVENT_EFFECTS
#define VSTR_SYSTEMS_EVENT_EFFECTS

#include "absl/types/span.h"
#include "types/events.h"
#include "types/frame.h"

namespace vstr {

void ApplyEventEffects(absl::Span<Event> events, Frame &frame);

}

#endif
