#include <string>
using namespace std;

//! Heading – Deviation & Variation
/*!
		1   2  3  4  5 6
        |   |  |  |  | |
$--HDG,x.x,x.x,a,x.x,a*hh
*/
struct HDG {
	double magHeading; //!< 1) Magnetic Sensor heading in degrees	
	double magDeviation; //!< 2) Magnetic Deviation, degrees
	char magDeviationDirection; //!< 3) Magnetic Deviation direction, E = Easterly, W = Westerly
	double magVariation; //!< 4) Magnetic Variation degrees
	char magVariationDirection; //!< 5) Magnetic Variation direction, E = Easterly, W = Westerly
	char checksum; //!< 6) Checksum
};

//! Heading - true
/*!
        1  2 3
        |  | |
$--HDT,x.x,T*hh
*/
struct HDT {
	double heading; //!< 1) Heading Degrees, true
	char tru; //!< 2) T = True
	char checksum; //!< 3) checksum
};

//! Cross Track Error – Dead Reckoning
/*!
       1  2  3   4          n
       |  |  |   |          |
$--XDR,a,x.x,a,c--c, ..... *hh
*/
struct XDR {
	char transducerType; //!< 1) Transducer type
	double measData; //!< 2) Measurement data
	char units;  //!< 3) Units of measurement
	string transducerName; //!< 4) Name of transducer
				// x) more of the same
	char checksum; //!< n) checksum
};

//! Global Positioning System Fix Data. Time, Position and fix related data for a GPS receiver
/*!
             1       2   3      4   5 6  7  8   9 10 11 12 13  14  15
             |       |   |      |   | |  |  |   |  |  |  |  |   |   |
$--GGA,hhmmss.ss,DDMM.MM,a,DDDMM.MM,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
*/
struct GGA {
	double time;	//!< 1) Time (UTC) in seconds
	double latitude;	//!< 2) Latitude
	char NS;		//!< 3) N or S (North or South)
	double longitude;	//4) Longitude
	char EW;		//!< 5) E or W (East or West)
	int gpsQual;	//!< 6) GPS Quality Indicator,
					//   0 - fix not available,
					//	 1 - GPS fix,
					//	 2 - Differential GPS fix
	int numSat;		//!<  7) Number of satellites in view, 00 - 12
	double HDOP;	//!<  8) Horizontal Dilution of precision
	double altitude; //!< 9) Antenna Altitude above/below mean-sea-level (geoid)
	char altUnits;	//!< 10) Units of antenna altitude, meters
	double geoidSep; //!< 11) Geoidal separation, the difference between the WGS-84 earth
					 //    ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid
	char geoidSepUnits; //!<  12) Units of geoidal separation,
	double difAge;	//!<  13) Age of differential GPS data, time in seconds since last SC104
					//     type 1 or 9 update, null field when DGPS is not used
	int difID;	//!< 14) Differential reference station ID, 0000-1023
	char checksum; //!< 15) Checksum
};

//! RMC Recommended Minimum Navigation Information
/*!
														  12
             1   2     3   4      5   6  7   8   9    10 11|
             |   |     |   |      |   |  |   |   |    |  | |
$--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a*hh
*/
struct RMC {
	double time; //!< 1) Time (UTC)
	char status; //!< 2) Status, V = Navigation receiver warning
	double latitude; //!< 3) Latitude
	char NS; //!< 4) N or S
	double longitude; //!< 5) Longitude
	char EW;	//!< 6) E or W
	double speed; //!< 7) Speed over ground, knots
	double course; //!< 8) Track made good, degrees true
	string date; //!< 9) Date, ddmmyy
	double magVar; //!< 10) Magnetic Variation, degrees
	char varEW;	//!< 11) E or W
	char checksum; //!< 12) checksum

};

//! VTG Track Made Good and Ground Speed
/*!
        1  2  3  4  5  6  7  8 9
        |  |  |  |  |  |  |  | |
$--VTG,x.x,T,x.x,M,x.x,N,x.x,K*hh
*/
struct VTG{
	double trueTrack; //!< 1) Track Degrees
	char t; //!< 2) T = True
	double magTrack; //!< 3) Track Degrees
	char m; //!< 4) M = Magnetic
	double speedKnots; //!< 5) Speed Knots
	char n; //!< 6) N = Knots
	double speedKPH; //!< 7) Speed Kilometers Per Hour
	char k; //!< 8) K = Kilometres Per Hour
	char checksum; //!< 9) Checksum
};

//////////////////////////////////////////////////////////////////
// Proprietary Messages
//////////////////////////////////////////////////////////////////

//! HPR - Heading, pitch, and roll - Honeywell compass
/*!
          1  2  3  4  5  6  7
          |  |  |  |  |  |  |
$PTNTHPR,x.x,a,x.x,a,x.x,a*hh
*/
struct HPR {
	double heading; //!< 1) Heading, degrees
	char headingStatus; //!< 2) heading status
	double pitch; //!< 3) Pitch, degrees
	char pitchStatus; //!< 4) pitch status
	double roll; //!< 5) roll, degrees
	char rollStatus; //!< 6) roll status
	char checksum; //!< 7) checksum
};