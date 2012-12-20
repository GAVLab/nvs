/*
    Author: Robert Cofield, for GAVLab
*/
#ifndef NVS_H
#define NVS_H

#include <fstream>
#include <iostream>
#include <queue>
#include <algorithm>

#include "nvs_structures.h"
#include <serial/serial.h> 

#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/pthread/condition_variable_fwd.hpp>
#include <boost/algorithm/string.hpp>

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>

#define PI 3.14159265

/* 
    Utility Functions
*/
double default_get_time();  // TODO reevaluate
void sleep_msecs(unsigned int);  // Sleep for specified milliseconds
void print_hex(uint8_t);
std::deque<int> get_indices(uint8_t*, const uint8_t);
bool stob(const std::string&);



/*
    Primary Class For interfacing with the receiver
*/
class NVS {

/*
    Primary Stuff
*/
public:
    NVS();
    ~NVS();

    bool Connect(std::string, int baudrate=115200);
    void Disconnect();

    bool IsConnected() {return is_connected_;}
    bool Ping(int num_attempts=5);

    void WaitForCommand();

/*
    Data-related stuff
*/
public:
    int updRate;                    // (Hz) rate at which receiver updates
    double read_timestamp_;         // time stamp when last serial port read completed
    int msg_rate_;                  // rate (Hz) at which data is sent out

    /*
        user attributes & output messaging
    */
    bool display_log_data_;  // Whether to print data to terminal as it comes in

    bool is_binr_;


/*
    Private Functions
*/
private:
    void StartReading();
    void StopReading();    
    void ReadSerialPort();
    void BufferIncomingData(uint8_t*, size_t);
    void DelegateParsing();

    bool wait_for_command_;             // Whether to continuously wait for user terminal input
    void ParseCommand(std::string);     // decide how to act on user terminal input

    /*
        send data to the receiver
    */
    bool SendMessage(std::string); // used for NMEA messages
    bool SendMessage(const uint8_t*);
    bool SendMessage(uint8_t*); // this is the version chris had
    bool SendMessage(const std::vector< uint8_t > &);


/*
 * Private Attributes
 */
private:
    boost::function<double()> time_handler_;

    /*
     *  Serial Port
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

    boost::shared_ptr<boost::thread> read_thread_;

    const static uint max_buffer_size_ = 5000;  

    /* 
     *  Incoming data buffering
     */
    bool reading_status_;                       // Set whether to iteratively read data from the receiver
    uint8_t msg_buffer_[MAX_MSG_SIZE];         // Holds bytes while searching for a complete message
    // bool found_header_;                      // found the beginning of a complete message
    std::deque<int> header_indices_;            // Location of the current message's header in incoming buffer
    const static uint8_t header_byte_ = 0x10;   // starts message and preceeds the footer
    std::deque<int> footer_indices_;            // Location of the current message's footer in incoming buffer
    const static uint8_t footer_byte_ = 0x03;   // ends message
    size_t buffer_len_;                         // size of the data which hasn't been processed
    bool buffering_;                      // buffer not yet fully parsed
    int payload_indices_[2];                    // where message currently being parsed lies (begin/end) within income buffer


};

#endif 
