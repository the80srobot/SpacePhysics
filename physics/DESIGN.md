# Rigid body physics with efficient rewinding

Adam Sindelar
June 27 2021

## Abstract

This a technical design for a specialized rigid body physics system that
supports efficient and deterministic queries about past and future states of
objects in the simulation.

Most modern rigid body systems support determinism (for example, Unity Physics
does this by running in fixed time steps), but not scrubbing to a previous
state, or efficient queries about the future.

The system proposed here is a time database (journal) of objects states and
physics events, such as collisions. User input is introduced as one type of
physics event, and causes recomputation of subsequent states and events. With
careful use, it should be possible to precompute state far ahead of the
displayed frame, making it trivial for application code to visualize
trajectories, detect whether a specific set of conditions (such as winning a
video game level) will occur from the present position and support user input
undo actions.

## Requirements

The requirements here are driven by the needs of a space physics game project.
Extensions, e.g., to other collision modes should be possible.

### Features

* Reliable collision detection between spheres traveling at very high speeds
* Support object destruction, but undo it when rewound past the time of
  destruction
* Support object breaking apart into multiple objects
* Support object being glued to another object

### Performance

* Collision detection should avoid quadratic complexity (checking every pair of
    objects)
* Queries for past and future object state must be free of side effects and
  complete within 1 ms on consumer hardware
* Fit all state into 100-200 MB of RAM
* Precalculate at least 10 seconds of state per second, running alongside game
  code

### Other

* Possible to integrate with major game engines

## Overview

TBD

## Design

### Life of a physics frame

* `frame_number` is the frame number being processed
* `frame` is the frame state, which includes all component data
* `head` is the highest frame number processed so far
* `log` is a database of events, indexed by frame number and object id

#### Frame at HEAD

If `frame_number == head + 1`, then a frame is being computed for the first
time.

1. Load the final state at `frame_number - 1` into `frame`
2. For each object, compute its `velocity` and `next_position`
3. Compute and resolve collisions
4. For each object, set the current `position` to its `next_position`
5. Record the frame in the `log`
  * Collisions, glued state and destroyed state are recorded as intervals
  * Acceleration from user input or scripted events is recorded as intervals
  * Object position is recorded at key frames only (by default 1 in every 60
    frames)

### Seeking

### Object destruction

### Collisions

Collisions can occur between any two eligible objects. To be eligible, both
objects in the pair must:

1. Be active (not destroyed)
2. Be assigned to a combination of layers for which collisions are enabled

Currently, all objects are spherical, and so collision between any two eligible
objects occurs if their centers come within the sum of both their radii at any
point during the frame. Because all motion in a single frame is linearized, for
every two objects the distance between their centers is a linear function of time:

`d(t) = |(posA + vA * t) - (posB + vB * t)|`

And the distance to collision is also a linear function of time:

`dc(t) = d(t) - rA - rB`

Finding the time of collision between two objects is simply a question of
solving the above equation for `t`, which is handled numerically.

Checking every pair of objects in the scene has O(N^2) complexity. We use a
bounding-volume hierarchy to quickly filter out any objects that are too distant
for a collision to be possible.

### Testing

### Integration

#### Unity

#### UnrealEngine

### Choice of implementation language

The implementation language is C++.

* (+) Predictable performance
* (+) Mature technologies exist to integrate with other languages and engines (e.g.
  SWIG)
* (+) Well known in the industry: resolving issues and finding collaborators is
  easier
* (+) Mature debuggers, profilers, static analysers and test frameworks exist
* (+) Comparatively easy to ensure state is local and compact
* (-) Limited compiler guarantees, safety and checks
* (-) Extreme amount of technical debt in the language and ecosystem
* (-) CMake is absolute crap, but no real alternatives exist

#### Alternative considered: C#

* (+) Well known in the industry
* (+) Good enough performance (or excellent performance when used correctly)
* (+) Memory-safe, compiler provides useful checks
* (+) Mature debuggers and test frameworks
* (-) Difficult to write well-performing code
* (-) C# requires use of `unsafe` code for basic language features (like working
  with arrays), and much earlier than comparable languages
* (-) Poor quality of libraries, including the standard library
* (-) Existing libraries and know-how are centered around Windows UI development
* (-) Requires special care to avoid latency issues related to garbage
  collection

#### Alternative considered: Go

* (+) Good enough performance
* (+) Memory-safe, compiler provides useful checks
* (+) Mature debuggers, test frameworks and analysers
* (+) Easy to use, limited surprises and good productivity
* (-) Hard to integrate with other languages and game engines
* (-) The FFI (CGO) is absurdly slow and hard to use correctly
* (-) Existing libraries and know-how are centered around web development, Cloud
  and security
* (-) Requires special care to avoid latency issues related to garbage
  collection

## Performance

### Complexity analysis

### Benchmarks

## Future work
