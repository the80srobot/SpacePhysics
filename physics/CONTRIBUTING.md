# Coding practices

## C++ Style

All code must be formatted with
[clang-format](https://clang.llvm.org/docs/ClangFormatStyleOptions.html) using
built-in [Google C++ style
guide](https://google.github.io/styleguide/cppguide.html) setting.

```bash
# Just run this
clang-format -i -style=google *.cc *.h
```

## License boilerplate

All source code files must include the following header:

```
// This file is part of VSTR Space Physics.
//
// Copyright 2021 Adam Sindelar
// License: http://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
//
// Author(s): Adam Sindelar <adam@wowsignal.io>
```

Obviously the Author(s) line should include names of the actual authors.

## Economy of Mechanism

Code is read a lot more than it's written. It should be obvious rather than
clever. In addition to the Google C++ style, observe the following rules:

1. Don't invent macros to avoid repeating similar-looking code a few times.
2. Comments should add value. Only point out things that are not obvious. Prefer
   to change non-obvious code to be obvious, rather than explain it.
3. Be consistent with surrounding code.
