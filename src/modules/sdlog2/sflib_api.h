// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the SFLIB_DLL_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// SFLIB_DLL_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.

#if (defined SFLIB_DLL_EXPORTS)&&(defined WIN32)
#define SFLIB_DLL_API __declspec(dllexport)
#elif (defined WIN32)
#define SFLIB_DLL_API __declspec(dllimport)
#else
#define SFLIB_DLL_API
#endif

#include <stdint.h>
/*
#ifdef WIN32
#include <stdint.h>
#else
#include "sflib_types.h"
#endif
*/

/************************************************************************/
#pragma pack(push ,1)
enum SensorFilterType
{
	SF_AHRS = 0,
	SF_IG_LC
};
enum ErrorCode
{
	EC_NONE = 0,	//success
	EC_GYRO_ERR,				//no coordinate calibration parameters.
	EC_ACC_ERR,			//rigid body is not supported.
	EC_MAG_ERR
};
enum NavMode
{
	NM_IDLE = 0,
	NM_ALIGN,
	NM_NAV
};
typedef struct
{
	enum SensorFilterType sf_type;
	uint8_t index;
}SFLibInstance;
/*
* library version.
* library version equal 1.2.3.4, e.g.,  main version equals 1, sub version equals 2,
*  control version equals 3,test version equals 4.
*/
struct LibVersion
{
	uint8_t lib_main_ver;	// main version number.
	uint8_t lib_sub_ver;	// sub. version number.
	uint8_t lib_ctrl_ver;	// control version number.
	uint8_t lib_test_ver;	// test version number.
};
struct InputImuData
{
	uint64_t	timestamp_us;
	float		IncrementAngle_Measure[3];
	float		IncrementVel_Measure[3];
	float		Mag_measure[3];
	bool		magdata_new;
};
struct InputGpsData
{
	uint64_t	timestamp_us;
	double		longitude;
	double		latitude;
	float		height;
	float		ve;
	float		vn;
	float		vu;
	uint8_t		sat_num;
	bool		valid;
};
struct OutputData
{
	uint64_t	timestamp_us;
	float		Quat[4];
	float		yaw;
	float		pitch;
	float		roll;
	double		longitude;
	double		latitude;
	float		height;
	float		ve;
	float		vn;
	float		vu;
};
#pragma pack(pop)
/************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

	SFLIB_DLL_API bool CreatSFLib(SFLibInstance *ptr_sflib_instance);

	SFLIB_DLL_API bool DestroySFLib(SFLibInstance *ptr_sflib_instance);

	SFLIB_DLL_API bool InitSFLib_Geo(SFLibInstance *ptr_sflib_instance, double longitude, double latitude, float height, uint64_t timestamp_us);

	SFLIB_DLL_API enum ErrorCode CallSFImuProc(SFLibInstance *ptr_sflib_instance, struct InputImuData* pIn);

	SFLIB_DLL_API enum ErrorCode CallSFGpsProc(SFLibInstance *ptr_sflib_instance, struct InputGpsData* pIn);

	SFLIB_DLL_API bool GetSFResult(SFLibInstance *ptr_sflib_instance, struct OutputData* pOut);

	//SFLIB_DLL_API bool set_ParamBlock(void *ptr_block, uint16_t num_byte);

	//SFLIB_DLL_API void get_ParamBlock(void *ptr_block, uint16_t &num_byte);

#ifdef __cplusplus
}
#endif

