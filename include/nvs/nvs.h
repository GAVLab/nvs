/*
    Author: Robert Cofield, for GAVLab
*/
#ifndef NVS_H
#define NVS_H

#include <fstream>
#include <iostream>
#include <queue>

#include "nvs_structures.h"
#include <serial/serial.h> 

#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/pthread/condition_variable_fwd.hpp>
#include <boost/algorithm/string.hpp>

// #include <boost/thread/thread.hpp>
// #include <boost/thread/locks.hpp>

#define PI 3.14159265

/* 
    Utility Functions
*/
// Default callback method for time stamping data.  Used if a
// user callback is not set.  Returns the current time from the
// CPU clock as the number of seconds from Jan 1, 1970 
double DefaultGetTime();

// Sleep for specified milliseconds
void sleep_msecs(unsigned int msecs);


class NVS {
public:
    NVS();
    ~NVS();

    bool Connect(std::string port, int baudrate=115200);
    void Disconnect();

    bool IsConnected() {return is_connected_;}
    bool Ping(int num_attempts=5);
    std::string GetVersion();

/*
    Private Functions
*/
private:
    void StartReading();
    void StopReading();    
    void ReadSerialPort();
    void BufferIncomingData(std::vector<std::string> msgs, size_t len);
    void DelegateParsing();

    bool SendMessage(std::string msg, size_t len);


    /*
        Parse Specific Messages
    */
    /* NMEA Standard Messages */
    void ParseGGA(std::string talker_id, std::string payload);
    void ParseGSV(std::string talker_id, std::string payload);
    void ParseGSA(std::string talker_id, std::string payload);
    /* Proprietary Messages */
    void ParsePORZD(std::string payload);

/*
    Private Attributes
*/
private:
    boost::function<double()> time_handler_;

    /*
        Serial Port
    */
    // Number of milliseconds between bytes received to timeout on.
    const static uint32_t serial_inter_byte_timeout_ = 100;
    // A constant number of milliseconds to wait after calling read.
    const static uint32_t serial_read_timeout_constant_ = 1500;
    // A multiplier against the number of requested bytes to wait after calling read.
    const static uint32_t serial_read_timeout_multiplier_ = 0;
    // A constant number of milliseconds to wait after calling write.
    const static uint32_t serial_write_timeout_constant_ = 1000;
    // A multiplier against the number of requested bytes to wait after calling write.
    const static uint32_t serial_write_timeout_multiplier_ = 0;
    serial::Serial* serial_port_;
    bool is_connected_;

    const static uint max_buffer_size_ = 5000;  

    /* 
        Incoming data buffers
    */
    bool reading_status_;
    // Maximum size of an NVS log buffer
    std::queue<std::string> data_buffer_;
    double read_timestamp_;         //!< time stamp when last serial port read completed
    // double parse_timestamp_;        //!< time stamp when last parse began
    // unsigned short msgID_;
  
/*
    Public Data from reciever in usable format
*/
public:
    float rmsError; // PORZD - planar
    bool dataIsValid; // PORZD

};

#endif 