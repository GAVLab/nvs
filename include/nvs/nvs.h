/*
    Author: Robert Cofield, for GAVLab
*/
#ifndef NVS_H
#define NVS_H

namespace nvs {

// #include "nvs_structures.h"
#include <fstream>
// #include <serial/serial.h> 

class NVS {
public:
    NVS();
    ~NVS();

    // bool Connect(std::string port, int baudrate=115200);
    void Disconnect();

    bool IsConnected() {return is_connected_;}
    bool Ping(int num_attempts=5);


private:
    void StartReading();
    void StopReading();
    void ReadSerialPort();

    bool is_connected_;
};

} // end namespace

#endif 