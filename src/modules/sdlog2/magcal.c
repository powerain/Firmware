#include <stdlib.h>
#include "magcal.h"
#include "iniparser.h"
#include "dictionary.h"

//static double pk[3][3];
//static double pz[3];
#define MAG_CAL_INI "/fs/microsd/magcal.ini"
// FIXME For TEMP
#define K11 "PARAMK:k11"
#define K12 "PARAMK:k12"
#define K13 "PARAMK:k13"
#define K21 "PARAMK:k21"
#define K22 "PARAMK:k22"
#define K23 "PARAMK:k23"
#define K31 "PARAMK:k31"
#define K32 "PARAMK:k32"
#define K33 "PARAMK:k33"

#define Z1  "PARAMZ:z1"
#define Z2  "PARAMZ:z2"
#define Z3  "PARAMZ:z3"

double cpk[3][3] = {
	{1.0, 0.0, 0.0},
	{0.0, 1.0, 0.0},
	{0.0, 0.0, 1.0},
};
double cpz[3] = {
   0.0, 0.0, 0.0,
};
bool mag_cal_data = false;

bool mag_cali_init()
{
	dictionary *ini;
	bool ret = false;

	ini = iniparser_load(MAG_CAL_INI);

	if (iniparser_find_entry(ini, K11) && iniparser_find_entry(ini, K12) && iniparser_find_entry(ini, K13) && 
		iniparser_find_entry(ini, K21) && iniparser_find_entry(ini, K22) && iniparser_find_entry(ini, K23) &&
		iniparser_find_entry(ini, K31) && iniparser_find_entry(ini, K32) && iniparser_find_entry(ini, K33) && 
		iniparser_find_entry(ini, Z1) && iniparser_find_entry(ini, Z2) && iniparser_find_entry(ini, Z3) )
	{
		cpk[0][0] = iniparser_getdouble(ini, K11, 0xFF);
		cpk[0][1] = iniparser_getdouble(ini, K12, 0xFF);
		cpk[0][2] = iniparser_getdouble(ini, K13, 0xFF);
		
		cpk[1][0] = iniparser_getdouble(ini, K21, 0xFF);
		cpk[1][1] = iniparser_getdouble(ini, K22, 0xFF);
		cpk[1][2] = iniparser_getdouble(ini, K23, 0xFF);
		
		cpk[2][0] = iniparser_getdouble(ini, K31, 0xFF);
		cpk[2][1] = iniparser_getdouble(ini, K32, 0xFF);
		cpk[2][2] = iniparser_getdouble(ini, K33, 0xFF);
		
		cpz[0] = iniparser_getdouble(ini, Z1, 0xFF);
		cpz[1] = iniparser_getdouble(ini, Z2, 0xFF);
		cpz[2] = iniparser_getdouble(ini, Z3, 0xFF);

		ret = true;
		mag_cal_data = true;
	}

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
			printf("%.8f\n", cpk[i][j]);
	}
	for (int i = 0; i < 3; i++)
		printf("%.8f\n", cpz[i]);

	if (ini != NULL)
		iniparser_freedict(ini);

	return ret;
}
