
## 2026-07-14 — M23: portable C99 FDM core (the "yolk") + market research kickoff

**Status**: GREEN (research workflow running in background)
**Files changed**: PRD.md, native/{fdm.h,fdm.c,gen-golden.mjs,golden-check.c,Makefile,golden.h}, .github/workflows/ci.yml, .gitignore
**Tests**: golden cross-validation ALL PASS — point forces/moments worst 2.8e-14 rel; 30 s trim + 20 s doublets-in-turbulence + 8 s ground roll trajectories worst 2.3e-13 m · JS suite unaffected
**Decisions**:
- The deployable asset is a single dependency-free C99 file (native/fdm.c) mirroring src/physics.js + src/wind.js EXPRESSION-FOR-EXPRESSION (comment forbids "optimizing" the math) — 6-DOF, actuators incl. servo faults, ISA, mulberry32 Dryden, ground model.
- Validation architecture: the JS sim is the reference; gen-golden.mjs emits golden.h (point cases + 3 sampled trajectories incl. turbulence); golden-check.c must reproduce them. Tolerances: 1e-12 point-wise, 1e-3 m calm / 0.5 m turbulent trajectories (libm-vs-V8 ulp drift allowance) — measured 10+ orders better on this host.
- `make -C native golden so` gated in CI; libfdm.so builds -fPIC for the VeriStand Linux RT path (cross-compile via CC=<toolchain-gcc>).
- Market research (defense HILS model demand, per user's product focus) launched as a background deep-research workflow.
**Next**:
- M24: NI VeriStand Model Framework wrapper (Inports/Outports/Parameters from the shared table) + interface-spec autogen (참고자료 PDF 양식 준거). Research report → strategy doc.
