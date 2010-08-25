#ifndef _DIAGNOSIS_H_
#define _DIAGNOSIS_H_

#include <pkgconf/hal.h>
#include <cyg/hal/hal_arch.h>
#include <cyg/hal/hal_cache.h>
#include <redboot.h>
#include <stdlib.h>

#include <pkgconf/diagnosis.h>

#ifdef CYGPKG_MEMORY_DIAGNOSIS
#include <cyg/diagnosis/memory.h>
#endif

extern struct cmd __DIAGNOSIS_cmds_TAB__[], __DIAGNOSIS_cmds_TAB_END__;
extern void diagnosis_usage(char *why);

#endif 		/* _DIAGNOSIS_H_ */
