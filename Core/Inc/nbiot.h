#ifndef __NB_IOT_H_
#define __NB_IOT_H_

typedef enum{
	NB_OK = 0,
	NB_TIMEOUT = 1,
	NB_ERROR = 2,
	NB_DATA_IN = 3,
	NB_NET_CLOSED = 4,
	NB_REG_CHANGED = 5
}NB_STA;

typedef enum{
	NB_STEP_BUFF_CLEAR = 0,
	NB_STEP_CHECK_AT,
	NB_STEP_CHECK_REG,
	NB_STEP_STOP_MODULE,
	NB_STEP_SET_COAP,
	NB_STEP_START_MODULE,
	NB_STEP_SET_PDP,
	NB_STEP_SIM_CHECK,
	NB_STEP_START_REG,
	NB_STEP_SET_AUTO_REG,
	NB_STEP_WAITING_REG_OK,
	NB_STEP_UP_REG_INFO,
	NB_STEP_UP_DEV_INFO
}NB_WORK_STEP;

NB_STA nbiot_reboot(int iTimeout);
NB_STA nbiot_check_reg(int iTimeout);
NB_STA nbiot_get_signl_val(char *pcSigVal);
NB_STA nbiot_get_nuestats(int *piRSRP, int *piSNR, int *piCellID, unsigned char *pucECL);

#endif
