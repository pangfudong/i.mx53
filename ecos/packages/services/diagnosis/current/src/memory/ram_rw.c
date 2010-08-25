#include <redboot.h>
#include <stdlib.h>
#include <cyg/diagnosis/diagnosis.h>
#include <cyg/hal/plf_io.h>

#include CYGHWR_MEMORY_LAYOUT_H


static int loops;
static unsigned int pattern1, pattern2;
static unsigned int start;
static int length;
static int burst = 0;

static void raw_rw_case1(void)
{
	unsigned int * current_write;
	unsigned int * current_read;
	int round = 0;
	diag_printf("RAM diagnostical pattern from David.Young of freescale\n");
	diag_printf("burst is %s\n", burst?"enabled":"disabled");
	while( (round++) < loops) {
	    if(burst) {
		current_write =(unsigned int *)start;
		memset(current_write, (pattern1&0xFF000000)>>24, length);
	    } else {
		for(current_write=(unsigned int *)start; current_write<(unsigned int *)(start + length); current_write += 2) {
			*current_write = ((unsigned int)current_write & 0x0000FFFF)|(0xFFFF0000 & pattern1);
		}
		for(current_write=(unsigned int *)start + 1; current_write<(unsigned int *)(start + length); current_write += 2) {
			*current_write = ((unsigned int)current_write & 0x0000FFFF)|(0xFFFF0000 & pattern2);
		}
	    }	
		for(current_read=(unsigned int *)start; current_read<(unsigned int *)(start + length); current_read ++) {
		    if(burst) {
			if((*current_read) != pattern2) {
					diag_printf("\tround %d::[0x%08x]=0x%08x:0x%08x\n", round, current_read, pattern2, *current_read);
					goto fail;

			}
		    } else {
			if((current_read - (unsigned int *)start)&1) {
				if(((*current_read)&0xFFFF0000) != (pattern2&0xFFFF0000)) {
					diag_printf("\tround %d::[0x%08x]=0x%08x:0x%08x\n", round, current_read, (pattern2&0xFFFF0000)|((unsigned int)current_read)&0xFFFF, *current_read);
					goto fail;
				}	
			} else {
				if(((*current_read)&0xFFFF0000) != (pattern1&0xFFFF0000)) {
					diag_printf("\tround %d::[0x%08x]=0x%08x:0x%08x\n", round, current_read, (pattern1&0xFFFF0000)|((unsigned int)current_read)&0xFFFF, *current_read);
					goto fail;
				}	

			}
		    }
		}	
	}
	diag_printf("Diagnosis is successful!\n");
	return;
fail:
	diag_printf("Diagnosis is failure !\n");	
}

void ram_rw_test(int argc, char * argv[])
{
	int opts_map[6];
	struct option_info opts[6];
	int mode;

	memset(opts_map, 0, sizeof(int)*5);

	init_opts(&opts[0], 'c', true, OPTION_ARG_TYPE_NUM,
          (void *)&loops, (bool *)&opts_map[0], "the rounds of test");
    	init_opts(&opts[1], 'b', true, OPTION_ARG_TYPE_NUM,
          (void *)&start, (bool *)&opts_map[1], "accessing start address");
    	init_opts(&opts[2], 'l', true, OPTION_ARG_TYPE_NUM,
          (void *)&length, (bool *)&opts_map[2], "accessing size(bytes)");
    	init_opts(&opts[3], 'p', true, OPTION_ARG_TYPE_NUM,
          (void *)&pattern1, (bool *)&opts_map[3], "High 16bit is valid");
    	init_opts(&opts[4], 'm', true, OPTION_ARG_TYPE_NUM,
          (void *)&mode, (bool *)&opts_map[4], "Test case number");
    	init_opts(&opts[5], 's', false, OPTION_ARG_TYPE_FLG,
          (void *)&burst, (bool *)0, "enable bust mode(based on memset)");

	if (!scan_opts(argc, argv, 2, opts, 6, 0, 0, 0)) {
        	diagnosis_usage("invalid arguments");
        	return;
    	}

	if(!opts_map[0]) {
		loops = 32;
	} 

	if(!opts_map[1]) {
		start = 0x80000;
	}

	if(!opts_map[2]) {
		length = 8192;
	}
	
	if(!opts_map[3]) {
		pattern1 = 0x55550000;
	}

	if(!opts_map[4]) {
		mode = DIAGNOSIS_MEM_RAM_RD;
	}

	if(burst) {
		pattern2 = (pattern1&0xFF000000);
		pattern2 |= pattern2>>8;
		pattern2 |= pattern2>>16;
	} else {
		pattern2 = (~pattern1)&0xFFFF0000;
	}

	if(!valid_address((unsigned char *)start)) {
		if (!verify_action("Specified address (%p) is not believed to be in RAM", (void*)start))
            return;
	}

	switch(mode) {
	case DIAGNOSIS_MEM_RAM_RD:
		raw_rw_case1();
		break;
	default:
		diag_printf("Invalid memory diagnosis case!\n");	
	}
}
