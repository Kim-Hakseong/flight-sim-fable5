# Portable FDM core: golden cross-validation + deployable shared library.
#   make golden   — regenerate JS golden vectors, build + run the C check
#   make so       — build libfdm.so (position-independent, for VeriStand Linux RT
#                   cross-compile with CC=<toolchain-gcc>)
CC ?= cc
CFLAGS ?= -std=c99 -O2 -Wall -Wextra -Werror
LDLIBS = -lm

golden: golden-check
	./golden-check

golden.h: gen-golden.mjs ../src/physics.js ../src/wind.js ../src/params.js
	node gen-golden.mjs

golden-check: golden-check.c fdm.c fdm.h golden.h
	$(CC) $(CFLAGS) -o $@ golden-check.c fdm.c $(LDLIBS)

so: libfdm.so
libfdm.so: fdm.c fdm.h
	$(CC) $(CFLAGS) -fPIC -shared -o $@ fdm.c $(LDLIBS)

clean:
	rm -f golden-check libfdm.so

.PHONY: golden so clean
