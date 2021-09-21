// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "frame.h"

namespace vstr {

Entity Frame::Push() {
  size_t id = transforms.size();
  assert(id < kMaxObjects);

  transforms.resize(id + 1);
  mass.resize(id + 1);
  motion.resize(id + 1);
  colliders.resize(id + 1);
  glue.resize(id + 1);
  flags.resize(id + 1);

  // Static cast checked by assert.
  return Entity{static_cast<int32_t>(id)};
}

Entity Frame::Push(Transform &&transform, Mass &&mass, Motion &&motion,
                   Collider &&collider, Glue &&glue, Flags &&flags) {
  assert(this->transforms.size() < kMaxObjects);

  this->transforms.push_back(std::move(transform));
  this->mass.push_back(std::move(mass));
  this->motion.push_back(std::move(motion));
  this->colliders.push_back(std::move(collider));
  this->glue.push_back(std::move(glue));
  this->flags.push_back(std::move(flags));

  // Static cast checked by assert.
  return Entity{static_cast<int32_t>(transforms.size() - 1)};
}

}  // namespace vstr
