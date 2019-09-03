#ifndef ANALYSIS_DATA_H
#define ANALYSIS_DATA_H

#ifdef __cplusplus
extern "C"
{
#endif

/*------------------------------------------------MARCOS define------------------------------------------------*/


/*------------------------------------------------Type define--------------------------------------------------*/
typedef enum
{
	crc_err = -3,
	data_len_err = -2,
	para_err = -1,
	analysis_ok = 0,
	analysis_done = 1
}analysis_res_t;

#pragma pack(1)

typedef struct
{
	unsigned char header1;	/*0x59*/
	unsigned char header2;	/*0x53*/
	unsigned short tid;		/*1 -- 60000*/
	unsigned char len;		/*length of payload, 0 -- 255*/
}output_data_header_t;

typedef struct
{
	unsigned char data_id;
	unsigned char data_len;
}payload_data_t;

typedef struct
{
	float accel_x;			/*uinit: m/s2*/
	float accel_y;
	float accel_z;

	float angle_x;			/*uinit: ° (deg)/s*/
	float angle_y;
	float angle_z;

	float mag_x;			/*uinit: 归一化值*/
	float mag_y;
	float mag_z;

	float raw_mag_x;		/*uinit: mGauss*/
	float raw_mag_y;
	float raw_mag_z;
	
	float pitch;			/*uinit: ° (deg)*/
	float roll;
	float yaw;
	
	float quaternion_data0;
	float quaternion_data1;	
	float quaternion_data2;
	float quaternion_data3;
}protocol_info_t;

#pragma pack()

/*------------------------------------------------------------------------------------------------------------*/
extern protocol_info_t g_output_info;

/*------------------------------------------------Functions declare--------------------------------------------*/
int analysis_data(unsigned char *data, short len);

#ifdef __cplusplus
}
#endif

#endif
