/*
    Author: Robert Cofield, for GAVLab
*/
#ifndef NVS_H
#define NVS_H

#include "nvs_structures.h"
#include <fstream>
#include <serial/serial.h> 

#include <boost/function.hpp>
#include <boost/thread.hpp>

#define PI 3.14159265

typedef boost::function<double()> GetTimeCallback;

class NVS {
public:
    NVS();
    ~NVS();

    bool Connect(std::string port, int baudrate=115200);
    void Disconnect();

    bool IsConnected() {return is_connected_;}
    bool Ping(int num_attempts=5);


private:
    /* Reading */
    void StartReading();
    void StopReading();
    void ReadSerialPort();
    void BufferIncomingData(uint8_t* msg, size_t len);

    bool SendMessage(uint8_t *msg, size_t len);
    

    GetTimeCallback time_handler_;

    /*Serial Port Parameters*/
    serial::Serial *serial_port_;
    boost::shared_ptr<boost::thread> read_thread_ptr_;
    bool reading_status_;
    bool is_connected_;

    /* Incoming data buffers */    
    unsigned char data_buffer_[MAX_NOUT_SIZE];  //!< data currently being buffered to read
    unsigned char* data_read_;      //!< used only in BufferIncomingData - declared here for speed
    size_t bytes_remaining_;    //!< bytes remaining to be read in the current message
    size_t buffer_index_;       //!< index into data_buffer_
    size_t header_length_;  //!< length of the current header being read
    bool reading_acknowledgment_;  //!< true if an acknowledgement is being received
    double read_timestamp_;         //!< time stamp when last serial port read completed
    double parse_timestamp_;        //!< time stamp when last parse began
    unsigned short msgID_;
      
};

#endif 