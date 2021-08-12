// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>

#ifndef NDEBUG
#include <fenv.h>

#ifndef VSTR_DEBUG
#define VSTR_DEBUG

#if defined(__APPLE__) && defined(__MACH__)

// Public domain polyfill for feenableexcept on OS X
// http://www-personal.umich.edu/~williams/archive/computation/fe-handling-example.c

inline int feenableexcept(unsigned int excepts) {
  static fenv_t fenv;
  unsigned int new_excepts = excepts & FE_ALL_EXCEPT;
  // previous masks
  unsigned int old_excepts;

  if (fegetenv(&fenv)) {
    return -1;
  }
  old_excepts = fenv.__control & FE_ALL_EXCEPT;

  // unmask
  fenv.__control &= ~new_excepts;
  fenv.__mxcsr &= ~(new_excepts << 7);

  return fesetenv(&fenv) ? -1 : old_excepts;
}

inline int fedisableexcept(unsigned int excepts) {
  static fenv_t fenv;
  unsigned int new_excepts = excepts & FE_ALL_EXCEPT;
  // all previous masks
  unsigned int old_excepts;

  if (fegetenv(&fenv)) {
    return -1;
  }
  old_excepts = fenv.__control & FE_ALL_EXCEPT;

  // mask
  fenv.__control |= new_excepts;
  fenv.__mxcsr |= new_excepts << 7;

  return fesetenv(&fenv) ? -1 : old_excepts;
}

#endif

class DebugHelper {
 public:
  static DebugHelper* Singleton() {
    static DebugHelper* instance = new DebugHelper();
    return instance;
  }

  void EnableFloatExceptions() {
    // This is racy, but as worst case, we call feenableexcept twice, which is
    // fine.
    if (!float_exceptions_enabled) {
      feenableexcept(FE_INVALID | FE_OVERFLOW);
      float_exceptions_enabled = true;
    }
  }

 private:
  DebugHelper() = default;
  bool float_exceptions_enabled = false;
};

#endif
#endif