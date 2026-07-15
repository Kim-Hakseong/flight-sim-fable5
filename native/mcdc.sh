#!/bin/sh
# MC/DC (Modified Condition/Decision Coverage) of the DEPLOYABLE C core (fdm.c),
# using gcc-14+'s -fcondition-coverage + `gcov --conditions` (real MC/DC tooling,
# toolchain-native, no commercial tool). golden-check + cov-driver together must
# cover every REACHABLE condition outcome; a documented allowlist carries the
# justified unreachable defensive guards.
#
# Requires gcc >= 14 (Apple clang has no -fcondition-coverage). If no capable gcc
# is found this SKIPS (exit 0) so local macOS dev isn't blocked; CI installs
# gcc-14 and sets CC=gcc-14 so the gate runs for real.
#
#   sh mcdc.sh
set -e
cd "$(dirname "$0")"

# Justified, unreachable defensive conditions, matched by SOURCE PATTERN (line
# numbers shift with refactors). Rationale in COMPLIANCE-DO331.md §5.
#   "n == 0.0" — quat_normalize zero-norm guard: a valid attitude quaternion never
#   has a zero norm; this protects the division. Provably unreachable.
ALLOW_PATTERNS="n == 0.0"

# --- find a gcc with -fcondition-coverage ------------------------------------
GCC=""
for cand in "$CC" gcc-16 gcc-15 gcc-14 gcc; do
  [ -z "$cand" ] && continue
  command -v "$cand" >/dev/null 2>&1 || continue
  if printf 'int main(){int a=1,b=0;return a&&b;}\n' | "$cand" -fcondition-coverage -x c - -o /dev/null 2>/dev/null; then
    GCC="$cand"; break
  fi
done
if [ -z "$GCC" ]; then
  echo "MCDC: SKIP — no gcc with -fcondition-coverage (need gcc>=14; Apple clang unsupported)"
  exit 0
fi
# matching gcov (gcc-16 → gcov-16, else plain gcov)
GCOV="gcov"
suffix=$(echo "$GCC" | sed -n 's/.*gcc-\([0-9]*\)$/\1/p')
[ -n "$suffix" ] && command -v "gcov-$suffix" >/dev/null 2>&1 && GCOV="gcov-$suffix"

echo "MCDC: using $GCC / $GCOV"
rm -f ./*.gcda ./*.gcno ./*.gcov cov-golden cov-driver fdm-cov.o
"$GCC" -fcondition-coverage --coverage -std=c99 -O0 -c fdm.c -o fdm-cov.o
"$GCC" --coverage -std=c99 -O0 -o cov-golden golden-check.c fdm-cov.o -lm
"$GCC" --coverage -std=c99 -O0 -o cov-driver cov-driver.c fdm-cov.o -lm
./cov-golden >/dev/null
./cov-driver
$GCOV --conditions fdm-cov.gcda >/dev/null 2>&1 || $GCOV --conditions fdm.gcda >/dev/null 2>&1 || true

ALLOW_PATTERNS="$ALLOW_PATTERNS" python3 - <<'PY'
import re, os, sys, glob
patterns = [p.strip() for p in os.environ.get("ALLOW_PATTERNS", "").split("|") if p.strip()]
gc = 'fdm.c.gcov'
if not glob.glob(gc):
    print("MCDC ERROR: no fdm.c.gcov"); sys.exit(2)
lines = open(gc).read().splitlines()
cur = None; cursrc = ""
total = covered = 0
unjustified = []; justified = []
for ln in lines:
    m = re.match(r'\s*[#\-0-9]+\*?:\s*(\d+):(.*)', ln)
    if m: cur, cursrc = m.group(1), m.group(2)
    mm = re.search(r'condition outcomes covered (\d+)/(\d+)', ln)
    if mm and cur:
        c, t = int(mm.group(1)), int(mm.group(2))
        total += t; covered += c
        if c < t:
            ok = any(p in cursrc for p in patterns)
            (justified if ok else unjustified).append((cur, c, t))
pct = 100.0 * covered / total if total else 100.0
print(f"fdm.c MC/DC condition coverage: {covered}/{total} = {pct:.1f}%")
for l, c, t in justified:
    print(f"  justified (allowlisted, §5): L{l} {c}/{t}")
for l, c, t in unjustified:
    print(f"  ✗ UNJUSTIFIED uncovered condition: L{l} {c}/{t}")
print("MCDC: PASS" if not unjustified else f"MCDC: {len(unjustified)} unjustified uncovered")
sys.exit(0 if not unjustified else 1)
PY
rc=$?
rm -f ./*.gcda ./*.gcno ./*.gcov cov-golden cov-driver fdm-cov.o
exit $rc
