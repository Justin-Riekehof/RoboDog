// Upstream writes `#include <arduino.h>` (app_httpd.cpp, line 1) and the
// Arduino core ships the header as `Arduino.h`. On Windows and macOS the
// filesystem does not care; on Linux the build fails at line 1, which is why
// this sketch had never been compiled outside its author's machine and why the
// baseline could not be built for comparison either (found 2026-08-24).
//
// A shim rather than a corrected include, because both invariants this fork
// lives by would otherwise break: the upstream copy under wavego-upstream/ has
// to stay byte-identical to vendor/ to be worth calling a baseline, and the
// fork is only ever allowed to ADD lines so a reviewer can check it by reading
// the additions. Tests enforce both. Putting the fix on the include path costs
// one file and touches neither.
#include <Arduino.h>
