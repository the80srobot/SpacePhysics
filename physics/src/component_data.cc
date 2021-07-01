#include "component_data.h"

namespace vstr {

std::ostream &operator<<(std::ostream &os, const CollisionEvent &event) {
  return os << "CollisionEvent{/*first_body_id=*/" << event.first_body_id
            << ", /*second_body_id=*/" << event.second_body_id
            << ", /*first_frame_offset_seconds=*/"
            << event.first_frame_offset_seconds << "}";
}

}  // namespace vstr