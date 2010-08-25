#ifndef __DIAGNOSIS_MEMORY_H_
#define __DIAGNOSIS_MEMORY_H_

#ifdef CYGSEM_RAM_RW_DIAGNOSIS
extern void ram_rw_test(int argc, char * argv[]);
#endif

enum {
DIAGNOSIS_MEM_RAM_RD = 0,
};

#endif 		/* __DIAGNOSIS_MEMORY_H_ */
