#ifndef _MAGCAL_H_
#define _MAGCAL_H_

#include <stdio.h>

extern double cpk[3][3];
extern double cpz[3];
extern bool   mag_cal_data;
#ifdef __cplusplus
extern "C"
{
#endif

bool mag_cali_init(void);
//void mag_cali(double);
#ifdef __cplusplus
}
#endif

#endif
