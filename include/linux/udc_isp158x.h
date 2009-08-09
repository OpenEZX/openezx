/*
 */

struct isp158x_udc_mach_info {
        void (*udc_command)(int cmd);
#define	ISP158X_UDC_CMD_CONNECT		0	/* let host see us */
#define	ISP158X_UDC_CMD_DISCONNECT	1	/* so host won't see us */
};

