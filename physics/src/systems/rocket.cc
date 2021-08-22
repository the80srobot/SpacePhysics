// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#include "rocket.h"

#include <absl/status/status.h>
#include <absl/status/statusor.h>

#include "types/required_components.h"

namespace vstr {

namespace {

absl::StatusOr<Event> ApplyRocketBurn(const float dt, const Event &event,
                                      std::vector<Mass> &mass,
                                      std::vector<Rocket> &rockets) {
  auto it = std::lower_bound(
      rockets.begin(), rockets.end(), Rocket{.id = event.id},
      [](const Rocket &a, const Rocket &b) { return a.id < b.id; });
  if (it == rockets.end() || it->id != event.id) {
    // Invalid state - the burn event targets an object with no rocket
    // component.
    return absl::NotFoundError("object has no Rocket component");
  }

  if (event.rocket_burn.fuel_tank >= Rocket::kMaxFuelTanks) {
    return absl::OutOfRangeError("no such fuel tank");
  }
  if (it->fuel_tanks[event.rocket_burn.fuel_tank].fuel <= 0) {
    return absl::ResourceExhaustedError("fuel tank empty");
  }
  const float throttle = Vector3::Magnitude(event.rocket_burn.thrust);
  const Vector3 thrust = event.rocket_burn.thrust *
                         it->fuel_tanks[event.rocket_burn.fuel_tank].thrust;
  const float fuel_used = throttle * dt;
  const float fuel_mass_used =
      it->fuel_tanks[event.rocket_burn.fuel_tank].mass_flow_rate * fuel_used;

  it->fuel_tanks[event.rocket_burn.fuel_tank].fuel -= fuel_used;
  mass[event.id].inertial -= fuel_mass_used;

  return Event(event.id, event.position,
               Acceleration{
                   .flags = Acceleration::kForce,
                   .linear = thrust,
               });
}

}  // namespace

absl::Status ApplyRocketRefuel(const Event &event, std::vector<Mass> &mass,
                               std::vector<Rocket> &rockets) {
  assert(event.type == Event::kRocketRefuel);

  auto it = std::lower_bound(
      rockets.begin(), rockets.end(), Rocket{.id = event.id},
      [](const Rocket &a, const Rocket &b) { return a.id < b.id; });
  if (it == rockets.end() || it->id != event.id) {
    // Invalid state - the burn event targets an object with no rocket
    // component.
    return absl::NotFoundError("object has no Rocket component");
  }

  int fuel_tank = event.rocket_refuel.fuel_tank_no;
  if (fuel_tank < 0) {
    // Find the first empty tank or abort.
    for (int i = 0; i < it->fuel_tank_count; ++i) {
      if (it->fuel_tanks[i].fuel <= 0) {
        fuel_tank = i;
        break;
      }
    }
    if (fuel_tank < 0) return absl::OutOfRangeError("no empty fuel tank");
  }

  if (fuel_tank >= Rocket::kMaxFuelTanks) {
    return absl::OutOfRangeError("fuel tank out of allowed range");
  }

  mass[event.id].inertial -=
      it->fuel_tanks[fuel_tank].mass_flow_rate * it->fuel_tanks[fuel_tank].fuel;
  it->fuel_tanks[fuel_tank] = event.rocket_refuel.fuel_tank;
  mass[event.id].inertial += event.rocket_refuel.fuel_tank.fuel *
                             event.rocket_refuel.fuel_tank.mass_flow_rate;
  return absl::OkStatus();
}

absl::Status ConvertRocketBurnToAcceleration(const float dt,
                                             absl::Span<Event> input,
                                             std::vector<Mass> &mass,
                                             std::vector<Rocket> &rockets) {
  for (Event &event : input) {
    if (event.type != Event::kRocketBurn) continue;
    auto converted_event = ApplyRocketBurn(dt, event, mass, rockets);
    if (!converted_event.ok()) {
      return converted_event.status();
    }
    event = converted_event.value();
  }
  return absl::OkStatus();
}

}  // namespace vstr
