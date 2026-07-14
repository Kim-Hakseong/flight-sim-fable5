/* MINIMAL STUB of NI VeriStand's ni_modelframework.h — CI compile-check only.
 * On the customer toolchain, build nivs_model.c against NI's REAL Model
 * Framework sources (ni_modelframework.h/.c from the VeriStand Model Framework
 * template) and this stub must NOT be on the include path. Types below mirror
 * the documented framework shapes closely enough to keep the wrapper honest. */
#ifndef NI_MODELFRAMEWORK_STUB_H
#define NI_MODELFRAMEWORK_STUB_H

#include <stdint.h>

#define rtDBL 0

typedef struct {
  int32_t idx;
  const char *paramname;
  uintptr_t addr;
  int32_t datatype;
  int32_t width;
  int32_t numdimensions;
  int32_t dimListOffset;
} NI_Parameter;

typedef struct {
  int32_t idx;
  const char *blockname;
  int32_t portno;
  const char *signalname;
  uintptr_t addr;
  int32_t datatype;
  int32_t width;
  int32_t numdimensions;
  int32_t dimListOffset;
} NI_Signal;

typedef struct {
  int32_t tid;
  double tstep;
  double offset;
} NI_Task;

#define NI_OK 0
#define NI_ERROR (-1)

#endif
