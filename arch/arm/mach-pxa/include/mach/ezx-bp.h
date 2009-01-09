struct ezxbp_config {
	/* gpios for handshake */
	int bp_reset;
	int bp_wdi;
	int bp_wdi2;
	int bp_rdy;
	int ap_rdy;
	int first_step;
	void (*init)(void);
};

int ezx_wake_bp(void);
