#!/bin/sh
# Structural line coverage of the DEPLOYABLE C core (fdm.c), combining the
# golden cross-validation and the API cov-driver under gcov. Portable across
# gcc (native gcov) and clang (llvm-cov gcov). Fails below THRESHOLD.
#   sh coverage.sh [threshold_percent]   (default 100)
set -e
cd "$(dirname "$0")"
THRESHOLD="${1:-100}"
CC="${CC:-cc}"

# pick a gcov: prefer plain gcov (gcc), else llvm-cov gcov (clang/mac)
if command -v gcov >/dev/null 2>&1 && "$CC" --version 2>/dev/null | grep -qiv clang; then
  GCOV="gcov"
elif command -v llvm-cov >/dev/null 2>&1; then
  GCOV="llvm-cov gcov"
elif xcrun --find llvm-cov >/dev/null 2>&1; then
  GCOV="xcrun llvm-cov gcov"
else
  GCOV="gcov"
fi

rm -f ./*.gcda ./*.gcno ./*.gcov cov-golden cov-driver fdm-cov.o
"$CC" --coverage -std=c99 -O0 -c fdm.c -o fdm-cov.o
"$CC" --coverage -std=c99 -O0 -o cov-golden golden-check.c fdm-cov.o -lm
"$CC" --coverage -std=c99 -O0 -o cov-driver cov-driver.c fdm-cov.o -lm
./cov-golden >/dev/null
./cov-driver

# gcov the fdm object's data (gcda accumulates across both runs)
$GCOV fdm-cov.gcda >/dev/null 2>&1 || $GCOV fdm.gcda >/dev/null 2>&1 || true

python3 - "$THRESHOLD" <<'PY'
import re, sys, glob
gcov = 'fdm.c.gcov'
if not glob.glob(gcov):
    print("ERROR: no fdm.c.gcov produced"); sys.exit(2)
exe = cov = 0; miss = []
for ln in open(gcov):
    m = re.match(r'\s*([#\-0-9]+):\s*(\d+):', ln)
    if not m: continue
    tok = m.group(1)
    if tok == '-': continue
    exe += 1
    if tok == '#####': miss.append(m.group(2))
    else: cov += 1
pct = 100.0 * cov / exe
thr = float(sys.argv[1])
print(f"fdm.c line coverage: {cov}/{exe} = {pct:.1f}%  (threshold {thr:.0f}%)")
if miss: print("  uncovered lines:", ", ".join(miss))
sys.exit(0 if pct >= thr else 1)
PY
rc=$?
rm -f ./*.gcda ./*.gcno ./*.gcov cov-golden cov-driver fdm-cov.o
exit $rc
