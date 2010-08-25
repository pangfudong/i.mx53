#include <cyg/diagnosis/diagnosis.h>

#ifdef CYGSEM_RAM_RW_DIAGNOSIS
local_cmd_entry("ram_rw",
        "ram read/write accessing",
        "-c iterators -b <base address> -l <length> "\
        "-p pattern -m case [-s]\n",
        ram_rw_test,
        DIAGNOSIS_cmds
           );
#endif
