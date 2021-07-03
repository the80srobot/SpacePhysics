#include "component_data.h"

namespace vstr {

std::ostream &operator<<(std::ostream &os, const Collider &collider) {
  return os << "Collider{/*layer=*/" << collider.layer << ", /*radius=*/"
            << collider.radius << "}";
}

std::ostream &operator<<(std::ostream &os, const Position &position) {
  return os << "Position{" << position.value << "}";
}

std::ostream &operator<<(std::ostream &os, const Acceleration &acceleration) {
  return os << "Acceleration{" << acceleration.value << "}";
}

std::ostream &operator<<(std::ostream &os, const Mass &mass) {
  return os << "Mass{/*rest=*/" << mass.rest << ", /*effective=*/"
            << mass.effective << "}";
}

std::ostream &operator<<(std::ostream &os, const Motion &motion) {
  return os << "Motion{/*velocity=*/" << motion.velocity
            << ", /*new_position=*/" << motion.new_position << "}";
}

std::ostream &operator<<(std::ostream &os, const Glue &glue) {
  return os << "Glue{/*parent_id=*/" << glue.parent_id << "}";
}

std::ostream &operator<<(std::ostream &os, const Destroyed &destroyed) {
  return os << "Destroyed{" << destroyed.value << "}";
}

std::ostream &operator<<(std::ostream &os, const CollisionEvent &event) {
  return os << "CollisionEvent{/*first_body_id=*/" << event.first_body_id
            << ", /*second_body_id=*/" << event.second_body_id
            << ", /*first_frame_offset_seconds=*/"
            << event.first_frame_offset_seconds << "}";
}

}  // namespace vstr