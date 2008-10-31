#ifndef EZX_BP_H
#define EZX_BP_H

typedef void (*phandshake)();
typedef struct _bp_single{
	int bp_rdy;
	int ap_rdy;
	int bp_wdi;
	int bp_reset;
	int bp_flash;
	int bp_mcu_int_sw;
	int last_step;
	int cur_step;
	phandshake handshake;
}bp_single_t;
#endif
