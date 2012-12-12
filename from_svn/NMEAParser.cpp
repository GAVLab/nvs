#include "NMEAParser.h"



void NMEAParser::addToBuffer(string msg)
{
	buffer.append(msg);
}

void NMEAParser::addToBuffer(const char* msg, int length)
{
	buffer.append(msg,length);
}

bool NMEAParser::parse(NMEASentence &snt)
{
	int pos1, pos2; // used to search string
	// search through buffer looking for start of message
	pos1=buffer.find_first_of('$');
	pos2=buffer.find_first_of('\n');

	// check to see if a full message exists in the buffer
	if ((pos1==-1)||(pos2==-1))
		return false;

	// get single message
	string msg=buffer.substr(pos1, pos2-pos1+1);
	// remove message from buffer
	buffer.erase(0,pos2+1);

	// tokenize message
	snt.tokens.clear();
	Tokenize(msg, snt.tokens, ",");

	// parse sentence
	// check for proprietary messages - start with 'P'
	if (snt.tokens[0].substr(1,1)=="P") {
		snt.msgID=snt.tokens[0].substr(snt.tokens[0].length()-3,3);
		snt.tokens[0].erase(snt.tokens[0].length()-3,3);
		snt.talkerID=snt.tokens[0].substr(2,snt.tokens[0].length()-2);
	}
	else { // standard message
		// 1st 2 letters after $ are talker id
		snt.talkerID=snt.tokens[0].substr(1,2);
		// next 3 letters are message id
		snt.msgID=snt.tokens[0].substr(3,3);
	}
	snt.tokens.erase(snt.tokens.begin());

	// look for checksum
	pos1=snt.tokens.back().find_first_of('*');
	if (pos1!=-1){
		string chksum=snt.tokens.back().substr(pos1+1,2);
		// TODO: convert from hex to decimal and put in structure
		// remove checksum and carriage returns
		snt.tokens.back().erase(pos1,snt.tokens.back().length()-pos1);
	}
	else {
		// no checksum
		snt.checksum=0;
	}

	return true;
}

HDG NMEAParser::parseHDG(NMEASentence snt){
	HDG retData;

	const char* temp;

	retData.magHeading=convertToDouble(snt.tokens[0]);
	retData.magDeviation=convertToDouble(snt.tokens[1]);
	temp=snt.tokens[2].c_str();
	retData.magDeviationDirection=temp[0];
	retData.magVariation=convertToDouble(snt.tokens[3]);
	temp=snt.tokens[4].c_str();
	retData.magVariationDirection=temp[0];

	return retData;
}

HDT NMEAParser::parseHDT(NMEASentence snt){
	HDT retData;

	const char* temp;
	retData.heading=convertToDouble(snt.tokens[0]);
	temp=snt.tokens[1].c_str();
	retData.tru=temp[0];
	retData.checksum=snt.checksum;

	return retData;
}

GGA NMEAParser::parseGGA(NMEASentence snt){
	GGA retData;

	const char* temp;

	// convert time
	int hours=convertToInteger(snt.tokens[0].substr(0,2));
	int mins=convertToInteger(snt.tokens[0].substr(2,2));
	double seconds=convertToInteger(snt.tokens[0].substr(4,4));
	seconds=seconds+60*mins+3600*hours;
	retData.time=seconds;

	double deg=(double)convertToInteger(snt.tokens[1].substr(0,2));
	snt.tokens[1].erase(0,2);
	double min=convertToDouble(snt.tokens[1]);
	retData.latitude=deg+min/60;
	
	temp=snt.tokens[2].c_str();
	retData.NS=temp[0];

	deg=(double)convertToInteger(snt.tokens[3].substr(0,3));
	snt.tokens[3].erase(0,3);
	min=convertToDouble(snt.tokens[3]);
	retData.longitude=deg+min/60;

	temp=snt.tokens[4].c_str();
	retData.EW=temp[0];

	retData.gpsQual=convertToInteger(snt.tokens[5]);
	retData.numSat=convertToInteger(snt.tokens[6]);
	retData.HDOP=convertToDouble(snt.tokens[7]);
	retData.altitude=convertToDouble(snt.tokens[8]);

	temp=snt.tokens[9].c_str();
	retData.altUnits=temp[0];

	retData.geoidSep=convertToDouble(snt.tokens[10]);

	temp=snt.tokens[11].c_str();
	retData.geoidSepUnits=temp[0];

	retData.difAge=convertToDouble(snt.tokens[12]);
	retData.difID=convertToInteger(snt.tokens[13]);

	retData.checksum=snt.checksum;

	return retData;
}

RMC NMEAParser::parseRMC(NMEASentence snt){
	RMC retData;

	const char* temp;

	// convert time
	int hours=convertToInteger(snt.tokens[0].substr(0,2));
	int mins=convertToInteger(snt.tokens[0].substr(2,2));
	double seconds=convertToInteger(snt.tokens[0].substr(4,4));
	seconds=seconds+60*mins+3600*hours;
	retData.time=seconds;

	
	temp=snt.tokens[1].c_str();
	retData.status=temp[0];

	double deg=(double)convertToInteger(snt.tokens[2].substr(0,2));
	snt.tokens[2].erase(0,2);
	double min=convertToDouble(snt.tokens[2]);
	retData.latitude=deg+min/60;

	temp=snt.tokens[3].c_str();
	retData.NS=temp[0];

	deg=(double)convertToInteger(snt.tokens[4].substr(0,3));
	snt.tokens[4].erase(0,3);
	min=convertToDouble(snt.tokens[4]);
	retData.longitude=deg+min/60;

	temp=snt.tokens[5].c_str();
	retData.EW=temp[0];

	retData.speed=convertToDouble(snt.tokens[6]);
	retData.course=convertToDouble(snt.tokens[7]);
	retData.date=snt.tokens[8];
	retData.magVar=convertToDouble(snt.tokens[9]);

	temp=snt.tokens[10].c_str();
	retData.varEW=temp[0];

	retData.checksum=snt.checksum;

	return retData;
}

VTG NMEAParser::parseVTG(NMEASentence snt){
	VTG retData;

	const char* temp;

	retData.trueTrack=convertToDouble(snt.tokens[0]);
	temp=snt.tokens[1].c_str();
	retData.t=temp[0];
	retData.magTrack=convertToDouble(snt.tokens[2]);
	temp=snt.tokens[3].c_str();
	retData.m=temp[0];
	retData.speedKnots=convertToDouble(snt.tokens[4]);
	temp=snt.tokens[5].c_str();
	retData.n=temp[0];
	retData.speedKPH=convertToDouble(snt.tokens[6]);
	temp=snt.tokens[7].c_str();
	retData.k=temp[0];
	return retData;
}

HPR NMEAParser::parseHPR(NMEASentence snt) {
	HPR retData;

	const char* temp;

	retData.heading=convertToDouble(snt.tokens[0]);
	temp=snt.tokens[1].c_str();
	retData.headingStatus=temp[0];
	retData.pitch=convertToDouble(snt.tokens[2]);
	temp=snt.tokens[3].c_str();
	retData.pitchStatus=temp[0];
	retData.roll=convertToDouble(snt.tokens[4]);
	temp=snt.tokens[5].c_str();
	retData.rollStatus=temp[0];
	
	return retData;
}

// stolen from: http://oopweb.com/CPP/Documents/CPPHOWTO/Volume/C++Programming-HOWTO-7.html
void NMEAParser::Tokenize(const string& str,
						  vector<string>& tokens,
						  const string& delimiters = " ") {
	// Skip delimiters at beginning.
	string::size_type lastPos = str.find_first_not_of(delimiters, 0);
	// Find first "non-delimiter".
	string::size_type pos     = str.find_first_of(delimiters, lastPos);

	while (string::npos != pos || string::npos != lastPos)
	{
		// Found a token, add it to the vector.
		tokens.push_back(str.substr(lastPos, pos - lastPos));
		// Skip delimiters.  Note the "not_of"
		lastPos = str.find_first_not_of(delimiters, pos);
		// Find next "non-delimiter"
		pos = str.find_first_of(delimiters, lastPos);
	}
}

// stolen from: http://www.parashift.com/c++-faq-lite/misc-technical-issues.html#faq-39.2
inline double NMEAParser::convertToDouble(const std::string& s) {
	std::istringstream i(s);
	double x;
	if (!(i >> x))
		return 0;
	else
		return x;
}

inline int NMEAParser::convertToInteger(const std::string& s) {
	std::istringstream i(s);
	int x;
	if (!(i >> x))
		return 0;
	else
		return x;
}