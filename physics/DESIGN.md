# Rigid body physics with efficient rewinding

Adam Sindelar
June 27 2021

## Abstract

This a technical design for a specialized rigid body physics system that
supports efficient and deterministic queries about past and future states of
objects in the simulation.

Most modern rigid body systems support determinism (for example, Unity Physics
does this by running in fixed time steps), but not rewinding to a previous
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
* Fit all state into 100 of RAM
* Precalculate at least 10 seconds of state per second, running alongside game
  code

### Other

* Possible to integrate with major game engines

## Overview

TBD

## Design

### Life of a physics frame

### Seeking

### Object destruction

### Collisions

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
* (-) Limited compiler guarantees, safety and checks
* (-) Extreme amount of technical debt in the language and ecosystem

#### Alternative considered: C#

* (+) Well known in the industry
* (+) Good enough performance (or excellent performance when used correctly)
* (+) Memory-safe, compiler provides useful checks
* (+) Mature debuggers and test frameworks
* (-) Difficult to write well-performing code
* (-) Poor quality of libraries, including the standard library
* (-) Existing libraries and know-how are centered around Windows UI development

#### Alternative considered: Go

* (+) Good enough performance
* (+) Memory-safe, compiler provides useful checks
* (+) Mature debuggers, test frameworks and analysers
* (+) Easy to use, limited surprises and good productivity
* (-) Hard to integrate with other languages and game engines
* (-) Existing libraries and know-how are centered web development, Cloud and
  security
* (-) Requires special care to avoid garbage collector inducing latency

## Performance

### Complexity analysis

### Benchmarks

## Future work
