#include <redboot.h>
#include <stdlib.h>
#include <cyg/diagnosis/diagnosis.h>

#include CYGHWR_MEMORY_LAYOUT_H

// Define table boundaries
CYG_HAL_TABLE_BEGIN( __DIAGNOSIS_cmds_TAB__, DIAGNOSIS_cmds);
CYG_HAL_TABLE_END( __DIAGNOSIS_cmds_TAB_END__, DIAGNOSIS_cmds);

// CLI function
static cmd_fun do_diagnosis_cmds;
RedBoot_nested_cmd("diagnosis",
           "Tools for diagnosis system",
           "{cmds}",
           do_diagnosis_cmds,
           __DIAGNOSIS_cmds_TAB__, &__DIAGNOSIS_cmds_TAB_END__
          );



void diagnosis_usage(char *why)
{
    diag_printf("*** invalid 'diagnosis' command: %s\n", why);
    cmd_usage(__DIAGNOSIS_cmds_TAB__, &__DIAGNOSIS_cmds_TAB_END__, "diagnosis ");
}

static void do_diagnosis_cmds(int argc, char *argv[])
{
	struct cmd * cmd;
	if (argc < 2) {
        	diagnosis_usage("too few arguments");
        	return;
    	}
	if ((cmd = cmd_search(__DIAGNOSIS_cmds_TAB__, 
			  &__DIAGNOSIS_cmds_TAB_END__,
                          argv[1])) != (struct cmd *)0) {
        	(cmd->fun)(argc, argv);
        	return;
    	}
    	diagnosis_usage("unrecognized command");
}
