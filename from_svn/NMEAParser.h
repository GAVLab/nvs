#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include "NMEAStructures.h"

#include <vector>
#include <string>
#include <sstream>
using namespace std;


struct NMEASentence {
	string talkerID; //!< two letter identifier for talker
	string msgID;	//!< three letter identifier for message type
	vector<string> tokens;	//!< vector of tokenized elements in message
	char checksum;	//!< checksum
};

class NMEAParser {
public:
	NMEAParser(){buffer="";};
	~NMEAParser(){};

	//! Add new received data to buffer for parsing
	void addToBuffer(string msg);
	//! Add new received data to buffer for parsing
	void addToBuffer(const char* msg, int length);

	//! Parse one NMEA message from the buffer and return it
	// snt - data parsed from buffer
	// returns true if valid message was found, false otherwise
	bool parse(NMEASentence &snt);

	//! Indicates whether any data is in the buffer or not
	bool bufferEmpty(){return (buffer.length()==0);};
	
	
	HDG parseHDG(NMEASentence snt);
	HDT parseHDT(NMEASentence snt);
	GGA parseGGA(NMEASentence snt);
	RMC parseRMC(NMEASentence snt);
	VTG parseVTG(NMEASentence snt);
	HPR parseHPR(NMEASentence snt);

private:
	string buffer; //!< buffer to hold incoming 

	//! method to split incoming messages into tokens
	void Tokenize(const string& str,vector<string>& tokens,const string& delimiters);

	inline double convertToDouble(const std::string& s);
	inline int convertToInteger(const std::string& s);
};

#endif