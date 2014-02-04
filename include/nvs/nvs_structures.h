/*
    NVS NV08C-CSM Data Structures

    Author: Robert Cofield, for GAVLab
*/
#ifndef NVSSTRUCTURES_H
#define NVSSTRUCTURES_H

#include "stdint.h"
namespace nvs{
/*
    limits to impose
*/
#define MAX_NOUT_SIZE 3000  // Maximum size of a NovAtel log buffer (ALMANAC logs are big!)
#define MAXCHAN     50  // Maximum number of signal channels
#define MAX_SAT     33  // maximum number of prns - max prn is 32 plus prn 0 is 33
#define MAX_MSG_SIZE (50) // max for the single message buffer    

#define NVS_DLE_BYTE 0x10
#define NVS_ETX_BYTE 0x03
#define NVS_CRC_BYTE 0xFF
    #define NVS_CNT_RBT  0x01
	#define NVS_CNT_SET  0x0b 
	#define NVS_RSP_SET  0x50
	#define NVS_CNT_OPPARAM  0x0d 
	#define NVS_RSP_OPPARAM  0x51
	#define NVS_CNT_CANCEL  0x0e 
	#define NVS_CNT_COOR  0x0f 
	#define NVS_RSP_COOR  0x55 
	#define NVS_CNT_TEST  0x11 
	#define NVS_RSP_TEST  0x43 
	#define NVS_CNT_SATUSED  0x12 
	#define NVS_RSP_SATUSED  0x47 
	#define NVS_CNT_CAAS  0x13 
	#define NVS_RSP_CAAS  0x41 
	#define NVS_CNT_RECCHAN  0x17 
	#define NVS_RSP_RECCHAN  0x42 
	#define NVS_CNT_SVEPHEM  0x19 
	#define NVS_RSP_SVEPHEM  0x49 
	#define NVS_CNT_SOFTW 0x1b 
	#define NVS_RSP_SOFTW  0x70 
	#define NVS_CNT_OPMOANT  0x1d 
	#define NVS_RSP_OPMOANT  0x73 
	#define NVS_CNT_GPSTM  0x1e 
	#define NVS_RSP_GPSTM  0x74 
	#define NVS_CNT_TMFREQ  0x1f 
	#define NVS_RSP_TMFREQ 0x72 
	#define NVS_CNT_ALMAN  0x20
	#define NVS_RSP_ALMAN  0x40
	#define NVS_CNT_NUMSATS  0x21 
	#define NVS_RSP_NUMSATS  0x60 
	#define NVS_CNT_TMZONE  0x23 
	#define NVS_RSP_TMZONE  0x46 
	#define NVS_CNT_VISSAT  0x24 
	#define NVS_RSP_VISSAT  0x52 
	#define NVS_CNT_COMCHK  0x26 
	#define NVS_RSP_COMCHK   0x54 
	#define NVS_CNT_PVT  0x27 
	#define NVS_RSP_PVT  0x88 
	#define NVS_CNT_IONO  0x2a 
	#define NVS_RSP_IONO  0x4a
	#define NVS_CNT_TMSCAL  0x2b 
	#define NVS_RSP_TMSCAL  0x4b 
	#define NVS_CNT_DOPRMS  0x31 
	#define NVS_RSP_DOPRMS  0x61 
	#define NVS_CNT_USEBLK  0x35 
	#define NVS_RSP_USEBLK  0x93 
	#define NVS_CNT_INFOCHN  0x39 
	#define NVS_RSP_INFOCHN  0x87 
	#define NVS_CNT_ATMOCOR  0x5c 
	#define NVS_RSP_ATMOCOR  0x5d 
	#define NVS_CNT_GDC  0xa0 
	#define NVS_RSP_GDC  0xa1
	#define NVS_CNT_INSTCOOR 0xa2 
	#define NVS_RSP_INSTCOOR  0xa3 
	#define NVS_CNT_BINR  0xb2 
	#define NVS_RSP_BINR   0xc2
	#define NVS_CNT_ADDPARAM  0xd7 
	#define NVS_RSP_ADDPARAM  0xe7 
	#define NVS_CNT_GLOTM  0xdb 
	#define NVS_RSP_GLOTM  0xeb 
	#define NVS_RAW_CNT  0xf4 
	#define NVS_RAW_RSP  0xf5 
	#define NVS_RAW_COOR  0xf6 
	#define NVS_RAW_EPHEM  0xf7
	#define NVS_AID_CNT  0x6f
	#define NVS_AID_EPHEM  0x62 
	#define NVS_AID_ALMAN  0x63 
	#define NVS_AID_IONO  0x64 
	#define NVS_AID_TIME  0x65 
	#define NVS_AID_DIFF  0x69 
	#define NVS_AID_REF  0x6a 
	#define NVS_AID_REFTM  0x6b 
    
    // #define RSP_SET 0x50
    // #define RSP_SOFTW 0x70


// define macro to pack structures correctly with both GCC and MSVC compilers
#ifdef _MSC_VER // using MSVC
    #define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#else
    #define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif  

/*
 *  Messages to send to receiver
 */

/*
INT8U 		uint8_t
INT8S 		int8_t
INT16U 		uin16_t
INT16S 		int16_t
INT32U 		uint32_t
INT32S 		int32_t
FP32 		float
FP64 		double
FP80 		triple 
*/



PACK(
	struct NVSHeader{
		uint8_t dle; //start of every message (0x10)
		uint8_t message_id;
	});

PACK(
	struct NVSFooter{
		uint8_t dle; 
		uint8_t etx; 
	});

//Control Messages Request Messages

/* Requests a forced reboot of the device and with or without erasing 
Message ID= 0x01
Payload Length: */

PACK(
	struct CntReboot{ //Reboot
		NVSHeader header; 
		uint8_t constant;  //0x00
		uint8_t constant2; //0x01
		uint8_t constant3; //0x21
		uint8_t constant4; //0x01
		uint8_t constant5; //0x00
		uint8_t reboot_type; //With 0x00 or without erasing 0x01
		NVSFooter footer; 

	});

/* Requests/Sets current port status
Message ID= 0x0b
Payload Length: */
PACK(
	struct CntPort_Status{
		NVSHeader header;
		uint8_t port_no; // Port number
		uint32_t ex_speed; //Baud Rate 115,200: 00,c2,01,00
		uint8_t protocol; //Protocol type Binary 0x04
		NVSFooter footer; 
	});

PACK(
	struct Current_Status{
		NVSHeader header; //Protocol type Binary 0x04
		NVSFooter footer; 
	});

/* Sets the coordinate system used, sat system used and PVT settings
Message ID= 0x0d
Payload Length: */
PACK(
	struct CntOperating_Param{
		NVSHeader header;
		uint8_t data_type;
		uint8_t coordinate_sys;
		uint8_t op_nav_sys; //GPS GLONASS GALILEO SBAS
		uint8_t min_mask_ele; // Minimum mask elevation
		uint8_t min_sig_noise; // Minimum signal to noise ratio
		uint16_t max_rms; // Maximum RMS value
		float coor_filt_factor; //Coordinates filtration factor
		NVSFooter footer; 
	});

/* Setting of reference coordinates for Fixed-Coordinates Operating Mode
Message ID= 0x0f
Payload Length: */
PACK(
	struct CntCoordinates_For_Fixed{ //
		NVSHeader header;
		uint8_t data_type;
		double latitude; //radians
		double longitude; //radians
		double height; //meters
		NVSFooter footer; 
	});

/* Requests internal test results
Message ID= 0x11
Payload Length: */
PACK(
	struct CntTest_Results{
		NVSHeader header;
		uint8_t internal_test; // 0 request/response a test results
		NVSFooter footer; 
	});

/* Enables or disables the use of specific satellite for navegation
Message ID= 0x12
Payload Length: */
PACK(
	struct CntUsed_Satellites{
		NVSHeader header;
		uint8_t sat_system; //GPS GLONASS
		uint8_t sat_number; //1/32 for GPS 1/24 GlONASS
		uint8_t use; //enable-1 disable-2
		NVSFooter footer; 
	});

/* Requests or disables the output of a message with course angle or actual speed values
Message ID= 0x13
Payload Length: */
PACK(
	struct CntCourse_Ang_Speed{
		NVSHeader header;
		uint8_t output_rate; //1/255
		NVSFooter footer; 
	});

/* Requests or sets the information on all or specific receiver channels
Message ID= 0x17
Payload Length: */
PACK(
	struct CntCurrent_Rcv_Sts{
		NVSHeader header;
		uint8_t channel_number; //0/31
		uint8_t sat_system; //1-GPS 2-GLONASS 4-SBAS
		uint8_t sv_number;  //SV number for GPS AND SBAS
		int8_t frequency_number; // for Glonass
		uint8_t channel_sts; //0-automatic 1-manual
		float doppler; //debug parameter
		NVSFooter footer; 
	});

/* Requests or loads ephemerides of the specified satellites state
Message ID= 0x19
Payload Length: */
PACK(
	struct CntSV_Ephemeris{
		NVSHeader header;
		uint8_t request; //255
		uint8_t output_rate; //disable of message output-0 output rate-1
		uint8_t sat_system; //1-GPS 2-GLONASS
		uint8_t sat_number; // 1/32 GPS 1/24 GlONASS
		int8_t frequency_number; //Carrier frequenct for GLONASS -7 to 6
		NVSFooter footer; 
	});

PACK(
	struct CntSoftware_Ver{
		NVSHeader header;
		NVSFooter footer; 

	});

PACK(
	struct CntModes{
		NVSHeader header;
		uint8_t data_type; //0-Operating mode setting, 1-Antenna Cable delay entry, 6- Coordinate Averaging time setting, 7-Entry of Coordinates
		uint8_t op_mode;//0-Standalone Mode, 1-Mode with Fixed Coordinates, 2-Averaging of Coordinates
		double delay; //miliseconds
		uint16_t average_time; //minutes
		double latitude; //radians
		double longitude; // radians
		double height; //meters
		NVSFooter footer; 
	});

PACK(
	struct CntRequest_TimeScale{
		NVSHeader header;
		NVSFooter footer; 

	});

PACK(
	struct CntTime_Fre_Param{
		NVSHeader header;
		uint8_t output_rate; //0-Disable, 1-Set output rate is 1 message per second
		NVSFooter footer; 
	});

PACK(
	struct CntAlmanac{
		NVSHeader header;
		uint8_t sat_system; //1-GPS, 2-GLONASS
		uint8_t sat_number; //1/32 GPS, 1/24 GLONASS
		NVSFooter footer; 
	});

PACK(
	struct CntNumber_Sat_Used{
		NVSHeader header;
		uint8_t output_rate; //0-Disable of Message output, 1/255 output rate seconds
		NVSFooter footer; 
	});

PACK(
	struct CntTime_Zone{
		NVSHeader header;
		int8_t hours; // +-13
		int8_t minutes; //+- 59
		NVSFooter footer; 
	});

PACK(
	struct CntVisible_Sat{
		NVSHeader header;
		uint8_t output_rate; //0-Disable of Message output, 1/255 output rate seconds
		NVSFooter footer; 
	});

PACK(
	struct CntCom_Check{
		NVSHeader header;
		NVSFooter footer; 
	});

PACK(
	struct CntPVT{
		NVSHeader header;
		uint8_t output_rate; //0-Disable of Message output, 1/255 output rate seconds
		NVSFooter footer; 
	});

PACK(
	struct CntIonosphere{
		NVSHeader header;
		uint8_t output_rate; //0-Disable of Message output, 1/255 output rate seconds
		NVSFooter footer; 
	});

PACK(
	struct CntTime_Scale{
		NVSHeader header;
		uint8_t output_rate; //0-Disable of Message output, 1/255 output rate seconds
		NVSFooter footer; 
	});

PACK(
	struct CntDOPandRMS{
		NVSHeader header;
		uint8_t output_rate; //0-Disable of Message output, 1/255 output rate seconds
		NVSFooter footer; 
	});

PACK(
	struct CntSat_UsedandBlocked{
		NVSHeader header;
		uint8_t output_rate; //0-Disable of Message output, 1/255 output rate seconds
		NVSFooter footer; 
	});

PACK(
	struct CntReciever_Info{
		NVSHeader header;
		uint8_t output_rate; //0-Disable of Message output, 1/255 output rate seconds
		NVSFooter footer; 
	});

PACK(
	struct CntAtmosphere{
		NVSHeader header;
		uint8_t output_rate; //0-Disable of Message output, 1/255 output rate seconds
		NVSFooter footer; 
	});

PACK(//NEEDS to be FIXED
	struct CntGroup_Delay{
		NVSHeader header;
		uint8_t request_type; 
		uint8_t table_no;
		float gd;
		float reserved;
		NVSFooter footer; 
	});

PACK(
	struct CntCoor_Sys_Param{
		NVSHeader header; 
		uint8_t coordinate_sys; //0/255
		float delta_a; //Correction to the major semi-axis of ellipsoid WGS84, meters
		float delta_f; //Correction of compression ratio
		float delta_x;// Mass centre offset, meters
		float delta_y;// Mass centre offset, meters
		float delta_z;// Mass centre offset, meters
		float omega_x; // Axis turn angles, angle. minutes?
		float omega_y;// Axis turn angles, angle. minutes?
		float omega_z;// Axis turn angles, angle. minutes?
		float m; //difference among linear scales*10^6
		uint8_t system_name[6]; 
		NVSFooter footer; 
	});

PACK(
	struct CntBINR_Proto{
		NVSHeader header;
		uint16_t status_word; // 0-not used, 1-Checksum(0-disable, 1-enable), 2-(0- altitude above geoid, 1-above ellipsoid), 3-(0-Geodectic Coordinates, 1-Rectangular Coordinates) 
		NVSFooter footer; 
	});

PACK(
	struct CntAddition_Param{
		NVSHeader header;
		uint8_t data_type; //1/11
		float max_accel; // .1/100 m/s^2
		uint8_t nav_rate; // 1, 2, 5, 10 Hz
		uint16_t pseudo_smoothing; //seconds
		uint8_t bi; //bit FIX!!!!!!!!!!!!!!!!!!!!!1
		uint8_t time_scale;  //0-on, 1-off
		uint32_t pps_length; //nsec
		double delay; //miliseconds
		uint16_t bit; //FIX!!!!!!!!!!
		uint8_t default_num_GLONASS; // Number of Channels used for cold start
		uint8_t default_num_GPS; // Number of Channels used for cold start
		uint8_t default_num_SBAS; // Number of Channels 
		uint8_t SBAS_test_mode; // Use of SBAS in test mode
		uint8_t assisted_data; //0-Assisted Request Turn-off, 1-Assisted Request Turn-On
		uint16_t RCTM_corrections; //Lifetime duration, seconds
		NVSFooter footer; 
	});

PACK(
	struct CntGlonass_timescale{
		NVSHeader header;
		uint8_t output_rate; ////0-Disable of Message output, 1/255 output rate seconds
		NVSFooter footer; 
	});

PACK(
	struct CntRaw_Data {
		NVSHeader header; 
		uint8_t meas_interval; 
		NVSFooter footer; 
	}); 


//RESPONSE MESSAGES

PACK(
	struct RspAlman_Spec_Sat{
		NVSHeader header;
		uint8_t sat_system; //1-GPS, 2-GLONASS
		//GPS System
		uint8_t prn; //Satellite on board number 1/32
		uint8_t health; //0-healthy
		uint8_t prn2; //On-baord Number 1/32
		float orbit_e; // Orbit Eccentricty 
		float orbit_i; // Orbit Inclination radians
		float omega_dot; // Ascending Mode Speed rad/ms
		double root_a;  // Square root of the major semi-axis of the orbit meters^1/2
		float omega_not; // Ascending Node Longitutde radians
		float omega; // Ascending Node perigee angle radians
		float m_not; // mean anomaly per average time 
		float af_not; // Time Correction (Polynomial Factor) milliseconds
		float af_one; //Time Correction milliseconds/milliseconds
		float af_not_upper; // The upper part of af_not milliseconds
		//super t_out; //Almanac reference Time !!!!!!!!!!!FIX!!!!!!!!!!!
		int16_t wn; //GPS WEEK Number starting from last turnover
		//GLONASS System
		uint8_t n_a; // Onboard satellite number 1/24
		uint8_t c_a; // /Satelite Status 0-healthy
		uint8_t h_a; // Satellite carrier frequency number (negative numbers are 25 to 31)
		float tau_a; // The satellite time scale offseet value in relation to the GLONASS Scale
		float lambda_a; // Orbit Ascending Node Longitude radians
		float I; // Orbit inclintation radians
		float epsilon_a; // Eccentricity 
		float omega_a; // Orbit Ascending Nodeperigee radians
		float t_a; // Orbit Asceding Node Passing Time milliseconds
		double T_a; // Draconic Period milliseconds/revolution
		float T_a_dot;// Draconic Period Change Speed milliseconds/revolution/revolution
		uint16_t N_a; // Day Number in a fuor year period
		NVSFooter footer; 
	});
PACK(
	struct RspCourse_Ang_Spd{
		NVSHeader header;
		float course_ang; //speed
		float speed; //km/h
		uint32_t time; //from beginning of week
		NVSFooter footer; 
	});

PACK(
	struct RspRec_Status{
		NVSHeader header;
		uint32_t sat_system; //1-GPS, 2-GLONASS, 4_SBAS, 8-Galileo Elb
		uint8_t sat_number; // Satellite number for GPS and SBAS
		int8_t carrier_freq; // Carrier Frequency for GLONASS SV
		uint8_t sig_noise_ratio; 
		uint8_t channel_sts; //0-Automatic Contron, 1-Manual Control, 2-Being Tested, 3-error
		uint8_t pseudo_sign; //0- There is a digitization and measurement of pseudorange, 1-failure, 2-Pseudorange measurement is available
		NVSFooter footer; 
	});

PACK(
	struct RspTest_Results{
		NVSHeader header;
		uint8_t response_type; //2-Test Results
		uint8_t test_results; //0-3 Reserved, 4-GPS Test, 5-GLONASS Test, 7-6 Antenna test (0-antenna not connected, 1-antenna connected, 3-antenna-short-circuited)
		uint32_t reserved;
		NVSFooter footer;  
	});

PACK(
	struct RspTime_Day_Zone{
		NVSHeader header;
		uint32_t week_time; // Time From beginning of week 
		uint8_t day; //1/31 
		uint8_t month; //1/12
		uint16_t year; //>1999
		int8_t tzhours; // Time Zone Correction (hours)
		int8_t tzminutes; // Time zone Correction (minutes)
		NVSFooter footer; 
	});

PACK(
	struct RspSat_Used{
		NVSHeader header;
		uint8_t sat_system; //1-GPS, 2-GLONASS
		uint8_t sat_number; // 1/32 for GPS, 1/24 for Glonass
		uint8_t use; //1 Enabled, 2-disabled
		NVSFooter footer; 
	});

PACK(
	struct RspSat_Ephem{
		NVSHeader header;
		uint8_t sat_system; //1-GPS, 2-GLONASS
		//GPS Satellite
		uint8_t prn; //On board number 1/32
		float c_rs; // Orbit Radius SIne Correction (meters)
		float delta_n; // Difference between principal motion and defined motion rad/millisecond
		double M_not; // Mean Anomaly radians
		float c_uc; //Longitude argument cosine correction radians
		double e; // eccentricity 
		float c_us; // Latitude sine correction radians
		double root_a; // Square root of major Semi-axis m^1/2
		double t_oe; //Ephemerides Reference time (ms)
		float c_ic;  // cosine correction to the inclination angle  radians
		double omega_not; // Orbital Plane ascending Node Longitude radians
		float c_is; // Sine Correction to ehe Inclination angle radians
		double i_not; // Inclination angle radians
		float c_rc; // Orbit Radius Cosine Correction meters
		double omega; //Ascending Node-Perigeee angle radians
		double i_dot; //inclination angle change speed radians/millisecond
		float T_gd; // Group Differential Delay estimation milliseconds
		double t_oc; // time correction (polynomial fact) ms
		float a_f2; //Time correction ms/ms^2
		float a_f1; // Time Correction ms/ms
		float a_f0; // Time Correction ms
		uint16_t ura; // User Range Measurement Accuracy 
		uint16_t iode; // Identifier of a set of ephemerides
		//GLONASSS Satellite
		uint8_t n_a; // On board number  1/24
		int8_t h_a; // Carrier Frequency Number 
		double x_n; //coordinates meters
		double y_n; 
		double z_n; 
		double v_x; // Speeds n/msec
		double v_y; 
		double v_z; 
		double a_x; //Accelerations m/msec^2
		double a_y; 
		double a_z; 
		double t_b; // Time interval inside current day ms
		float gamma_n; // Signal carrier frequency value relative deviation 
		float tau_n; // The satellite time scale offset value in relation to the GLONASS Scale ms
		uint16_t E_n; // Age of Ephemerides, days
		NVSFooter footer; 
	});

PACK(
	struct RspIono_Param{
		NVSHeader header;
		float alpha_not; //sec
		float alpha_1; //sec/semicycle
		float alpha_2; //sec/semicycle/semicycle
		float alpha_3; //sec/semicycle/semicycle/semicycle
		float beta_not; //sec
		float beta_1; //sec/semicycle
		float beta_2; //sec/semicycle/semicycle
		float beta_3; //sec/semicycle/semicycle/semicycle
		uint8_t reliability; // Reliability Sign
		NVSFooter footer; 
	});

PACK(
	struct RspTimeScale_Param{
		NVSHeader header;
		double A_1; //sec/sec
		double A_not; //sec
		uint32_t t_ot; //sec
		uint16_t WN_t; // weeks
		int16_t delta_t_ls; //sec
		uint16_t WN_lsf; //weeks
		uint16_t DN; //days
		int16_t delta_t_lsf; //sec
		uint8_t reliability; //For GPS
		uint16_t N_a; //number of days to the time stamp
		float tau_c; //correction to GLONASS system times scale in relation to UTC
		uint8_t reliability2; //For GLonass
		NVSFooter footer; 
	});

PACK(
	struct RspPort_Sts{
		NVSHeader header;
		uint8_t port_no; //1-Port1,2-Port2
		uint32_t baud_rate; //4800/230400
		uint8_t protocol; //1-No Protocol used, 2-NMEA protocol, 3-DIFF Protocol, 4-BINR Protocol, 5-BINR2 Protocol
		NVSFooter footer; 
	});

PACK(
	struct RspRec_Op_Param{
		NVSHeader header;
		uint8_t op_sat_nav_sys; //
		uint8_t reserved; 
		uint8_t coordinate_sys; 
		int8_t min_mask_ele; 
		uint8_t min_snr_level; 
		uint16_t max_rms; 
		float sol_fil_degree; // Solution filtration degree
		NVSFooter footer; 
	});

PACK(
	struct RspVis_Sats{
		NVSHeader header;
		uint8_t sat_system; 
		uint8_t sat_number; 
		int8_t carrier_freq; 
		uint8_t sat_height_mask; 
		uint16_t azimuth; 
		uint8_t sig_noise_ratio; 
		NVSFooter footer; 
	});

PACK(
	struct RspMode_Status{
		NVSHeader header;
		uint8_t op_mode; 
		double delay; 
		uint16_t averaging_time; 
		NVSFooter footer; 
	});

PACK(
	struct RspAtm_Corrections{
		NVSHeader header;
		uint8_t num_of_sat; 
		uint8_t sat_system; 
		uint8_t sat_number; 
		float tropo_delay; 
		float iono_delay; 
		float reserved; //dual frequency receivers
		NVSFooter footer; 
	});

PACK(
	struct RspNum_Sat_DOP{
		NVSHeader header;
		uint8_t num_gps_sat; 
		uint8_t num_glonall_sat; 
		float hdop; 
		float vdop; 
		NVSFooter footer; 
	});

PACK(
	struct RspDOP_RMS_PVT{
		NVSHeader header;
		float hdop; 
		float vdop; 
		float tdop; 
		float lat_error; 
		float long_error; 
		float alt_error; 
		float lat_speed_error; 
		float long_speed_error; 
		float alt_speed_error;
		NVSFooter footer;  
	});

PACK(
	struct RspSoftware{
		NVSHeader header;
		uint8_t num_channels; 
		uint8_t version_identifier[21];
		uint32_t serial_num; 
		uint8_t reserved[21];
		uint32_t reserved2; 
		uint8_t reserved3[21];
		uint32_t reserved4; 
		NVSFooter footer; 
	});

PACK(
	struct RspTime_Freq_Param{
		NVSHeader header;
		//super current_time; 
		int16_t gps_week_num; 
		uint8_t time_scale_type; 
		double period_deviation; 
		double time_stamp_deviation; 
		uint16_t gps_scale_deviation; 
		uint8_t notused; 
		uint16_t notused2; 
		NVSFooter footer; 
	});

PACK(
	struct RspTime_Sync_Opmode{
		NVSHeader header;
		int8_t mode_operation; 
		double ts_form_delay; 
		int16_t avging_time_coor; 
		NVSFooter footer; 
	});

PACK(
	struct RspTime_Scale_Param{
		NVSHeader header;
		//super gps_int_scale_shift; 
		//super glonass_int_scale_shift;
		//super gps_utc_scale_shift; 
		//super glonass_utc_scale_shift; 
		//super gps_glonass_shift; 
		uint8_t data_validitiy; 
		NVSFooter footer; 
	});

PACK(
	struct RspRec_Chan_Info{
		NVSHeader header;
		uint8_t sat_system;
		int8_t sat_number; 
		uint8_t sig_noise_ratio; 
		uint8_t channel_sts; 
		uint16_t channel_sts2; 
		double range_epoch; 
		float dopple; 
		int16_t pseudo_sign; 
		NVSFooter footer; 
	});

PACK(
	struct RspPVT{
		NVSHeader header;
		double latitude; 
		double longitude; 
		double height; 
		float rms; 
		//super week_time; 
		int16_t gps_week_num; 
		double lat_speed; 
		double long_speed; 
		double alt_speed; 
		float rgp_deviation; 
		uint8_t solution_sts; 
		NVSFooter footer; 
	});

PACK(
	struct RspUsed_B_Sats{
		NVSHeader header;
		uint32_t solution_sts; 
		uint32_t used_gps_sats; 
		uint32_t used_glonass_sats; 
		uint32_t used_sbas_sats; 
		uint8_t sat_number; 
		float range_error; 
		NVSFooter footer; 
	});

PACK(
	struct RspGrp_Delay_Tab{
		NVSHeader header;
		uint8_t response_type; 
		uint8_t table_no; 
		float gd1; 
		float gd2;
		float gd3;
		float gd4;
		float gd5;
		float gd6;
		float gd7;
		float gd8;
		float gd9;
		float gd10;
		float gd11;
		float gd12;
		float gd13;
		float reserved1;
		float reserved2;
		float reserved3;
		float reserved4;
		float reserved5;
		float reserved6;
		float reserved7;
		float gd_gps; 
		NVSFooter footer; 
	});

PACK(
	struct RspUser_Coor{
		NVSHeader header;
		NVSFooter footer; 

	});

PACK(
	struct RspBINR_Word{
		NVSHeader header;
		NVSFooter footer; 

	});

PACK(
	struct RspAdd_Op_Param{
		NVSHeader header;
		NVSFooter footer; 

	});

PACK(
	struct RspGLONASS_TS{
		NVSHeader header;
		double tau_c; 
		float tau_gps; 
		uint16_t N_a; 
		uint8_t N_4; 
		uint8_t reliability; 
		uint8_t KP; 
		float B1; 
		float B2; 
		uint8_t reliability2; 
		NVSFooter footer; 
	});

//Raw Data Messages

PACK(
	struct RawData{
		NVSHeader header;
		double time;
		uint16_t week_number; 
		double gps_utc_time_shift;
		double glonass_utc_time_shift;
		int8_t rec_time_correction; 
		uint8_t signal_type; 
		uint8_t sat_number;
		uint8_t carrier_num; 
		uint8_t sig_noise_ratio;
		double carrier_phase; 
		double pseudo_range; 
		double doppler_freq; 
		uint8_t raw_data_flags; 
		uint8_t reserved; 
		NVSFooter footer; 
	});

PACK(
	struct RawECEF{
		NVSHeader header;
		double x; 
		double y; 
		double z; 
		double rms_x; 
		double rms_y; 
		double rms_z; 
		uint8_t flag_of_user;
		NVSFooter footer;  
	});

PACK(
	struct RawEmphemeris{
		NVSHeader header;
		uint8_t sat_system; 
		uint8_t sat_number; 
		//GPS
		float c_rs; 
		float delta_n; 
		double m_not; 
		float c_uc; 
		double e; 
		float c_us; 
		double root_a; 
		double t_oe; 
		float c_ic; 
		double omega_not; 
		float c_is; 
		double i_not; 
		float c_rc; 
		double omega_dot; 
		double i_dot; 
		float T_gd; 
		double t_oc; 
		float a_f2; 
		float a_f1; 
		float a_f0; 
		uint16_t ura; 
		uint16_t iode; 
		uint16_t idoc; 
		uint16_t l2code; 
		uint16_t l2_pdata_flag; 
		uint16_t week_number; 
		//GLONASS
		int8_t carrier_num; 
		double x_n; 
		double y_n; 
		double z_n; 
		double v_x; 
		double v_y; 
		double v_z; 
		double a_x; 
		double a_y; 
		double a_z; 
		double t_not; 
		float gamma_n; 
		float tau_n; 
		uint16_t E; 
		NVSFooter footer; 
	});

/*PACK(
	struct RawRequest_Bi{
		NVSHeader header;
		uint8_t action;
		NVSFooter footer;  
	});

PACK(
	struct RawBi_Trans{
		NVSHeader header;
		uint8_t num_blocks; 
		//GLONSASS
		uint8_t num_channel; 
		uint8_t sns_type; 
		uint8_t carrier_num; 
		uint32_t time_reception_of_data; 
		uint8_t 85bit[3]; 
		//GPS
		uint8_t num_channel; 
		uint8_t sns_type; 
		uint8_t gps_sat_number; 
		uint32_t time_reception_of_data; 
		//uint8_t 30bit[10];
		//SBAS
		uint8_t num_channel; 
		uint8_t sns_type;
		uint8_t sat_number; 
		uint32_t time_reception_of_data; 
		//uint8_t 250bit[8];
		NVSFooter footer;  
	});*/
//ASSISTED MESSAGES NEED TO BE FINISHED 
PACK(
	struct AstRequest{
		NVSHeader header;
		uint8_t request_flag; 
		uint16_t requested_data; 
		uint32_t reserved; 
		NVSFooter footer; 
	});

/*PACK(
	struct AstEphemeris{
		NVSHeader header;
		uint8_t type; 
		uint8_t prn; 
		uint8_t status;
		int wn : 10;  
		int l2code : 2; 
		int ura : 4; 
		int health : 4; 
		int iodc : 2; 
		int l2_pdata_flag: 1; 
		int reserved: 87; 
		int t_gd : 8; 
		int iodc2 : 8; 
		int t_oc : 16; 
		int a_f2 : 8; 
		int a_f1 : 16; 
		int a_f0 : 22; 
		int t : 2; 
		int iode : 8; 
		int c_rs : 16; 
		int delta_n : 16; 
		int m_not : 32; 
		int c_uc : 16; 
		int e : 32; 
		int c_us : 16; 
		int root_a: 32; 
		int t_oe: 16; 
		int fit_interval_flag : 1; 
		int aodo : 5; 
		int t : 2; 
		int c_ic : 16; 
		int omega_not : 32; 
		int c_is : 16; 
		int i_not : 32; 
		int c_rc : 16; 
		int omega : 32; 
		int omega_dot : 24; 
		int iode : 8; 
		int idot : 14; 
		int t : 2; 
		uint8_t prn; 
		int b_n : 3; 
		int t_b : 7; 
		int tau_n : 22; 
		int gamma_n : 11; 
		int delta_tau : 5; 
		int E_n : 2; 
		int p1 : 2; 
		int p2 : 1; 
		int m : 2; 
		int px : 27; 
		int vx : 24; 
		int ax : 5; 
		int py : 27; 
		int vy : 24; 
		int ay : 5; 
		int pz : 27; 
		int vz : 24; 
		int az : 5; 
		int reserved : 14; 
	});*/

enum Message_ID{
	
	CNT_RBT= 0x01,
	CNT_SET= 0x0b, 
	RSP_SET= 0x50,
	CNT_OPPARAM= 0x0d, 
	RSP_OPPARAM= 0x51, 
	CNT_CANCEL= 0x0e, 
	CNT_COOR= 0x0f, 
	RSP_COOR= 0x55, 
	CNT_TEST= 0x11, 
	RSP_TEST= 0x43, 
	CNT_SATUSED= 0x12, 
	RSP_SATUSED= 0x47, 
	CNT_CAAS= 0x13, 
	RSP_CAAS= 0x41, 
	CNT_RECCHAN= 0x17, 
	RSP_RECCHAN= 0x42, 
	CNT_SVEPHEM= 0x19, 
	RSP_SVEPHEM= 0x49, 
	CNT_SOFTW= 0x1b, 
	RSP_SOFTW= 0x70, 
	CNT_OPMOANT= 0x1d, 
	RSP_OPMOANT= 0x73, 
	CNT_GPSTM=0x1e, 
	RSP_GPSTM= 0x74, 
	CNT_TMFREQ= 0x1f, 
	RSP_TMFREQ=0x72, 
	CNT_ALMAN= 0x20,
	RSP_ALMAN= 0x40, 
	CNT_NUMSATS= 0x21, 
	RSP_NUMSATS= 0x60, 
	CNT_TMZONE= 0x23, 
	RSP_TMZONE= 0x46, 
	CNT_VISSAT= 0x24, 
	RSP_VISSAT= 0x52, 
	CNT_COMCHK=0x26, 
	RSP_COMCHK= 0x54, 
	CNT_PVT= 0x27, 
	RSP_PVT= 0x88, 
	CNT_IONO= 0x2a, 
	RSP_IONO= 0x4a, 
	CNT_TMSCAL= 0x2b, 
	RSP_TMSCAL= 0x4b, 
	CNT_DOPRMS= 0x31, 
	RSP_DOPRMS= 0x61, 
	CNT_USEBLK= 0x35, 
	RSP_USEBLK= 0x93, 
	CNT_INFOCHN= 0x39, 
	RSP_INFOCHN= 0x87, 
	CNT_ATMOCOR= 0x5c, 
	RSP_ATMOCOR= 0x5d, 
	CNT_GDC= 0xa0, 
	RSP_GDC= 0xa1, 
	CNT_INSTCOOR= 0xa2, 
	RSP_INSTCOOR= 0xa3, 
	CNT_BINR= 0xb2, 
	RSP_BINR= 0xc2, 
	CNT_ADDPARAM= 0xd7, 
	RSP_ADDPARAM= 0xe7, 
	CNT_GLOTM= 0xdb, 
	RSP_GLOTM= 0xeb, 
	RAW_CNT= 0xf4, 
	RAW_RSP= 0xf5, 
	RAW_COOR= 0xf6, 
	RAW_EPHEM= 0xf7,
	AID_CNT= 0x6f, 
	AID_EPHEM= 0x62, 
	AID_ALMAN= 0x63, 
	AID_IONO= 0x64, 
	AID_TIME=0x65, 
	AID_DIFF= 0x69, 
	AID_REF= 0x6a, 
	AID_REFTM= 0x6b 
};
} //endnamespace
#endif
