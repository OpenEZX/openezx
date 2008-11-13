struct pcap_led {
	u8 type;
	char *name;
	u8 curr;
	u8 timing;
	u32 gpio;
	int brightness;
	struct led_classdev ldev;
	struct work_struct work;
};

struct pcap_leds_platform_data {
	int num_leds;
	struct pcap_led leds[];
};
