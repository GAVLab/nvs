/*
    Author: Robert Cofield, for GAVLab
*/
#include <nvs/nvs.h>
#include <iostream>

using namespace std;

/*
    Default callback method for time stamping data.  Used if a
    user callback is not set.  Returns the current time from the
    CPU clock as the number of seconds from Jan 1, 1970
 */
double DefaultGetTime() {
    boost::posix_time::ptime present_time(
            boost::posix_time::microsec_clock::universal_time());
    boost::posix_time::time_duration duration(present_time.time_of_day());
    return duration.total_seconds();
}


NVS::NVS() {
    time_handler_ = DefaultGetTime;

    /* serial port */
    serial_port_ = NULL;
    reading_status_ = false;
    is_connected_ = false;

    /* Reading */
    buffer_index_ = 0;
    reading_acknowledgment_ = false;
    bytes_remaining_ = false;
    header_length_ = 0;
    msgID_ = 0;
    data_read_ = NULL;
    read_timestamp_ = 0;
    parse_timestamp_ = 0;

    /* Data Callbacks */

}

NVS::~NVS() {
    Disconnect();
}

bool NVS::Connect(std::string port, int baudrate) {
    serial::Timeout timeout(100, 1000, 0, 1000, 0);
    try {
        serial_port_ = new serial::Serial(port, baudrate, timeout);
    }
    catch (exception e) {
        std::stringstream output;
        cout << "\nFailed to create connection on port: " << port 
            << "\nErr: " << e.what() << "\n";
        serial_port_ = NULL;
        is_connected_ = false;
        return false;
    }

    if (!serial_port_->isOpen()) {
        stringstream output;
        cout << "\nFailed create open port: " << port << "\n";
        delete serial_port_;
        serial_port_ = NULL;
        is_connected_ = false;
        return false;
    } else {
        stringstream output;
        cout << "\nSuccessfully opened port: " << port;
    }

        serial_port_->flush();

    if (!Ping()) {
        stringstream output;
        cout << "\nNVS not found on port: " << port << "\n";
        delete serial_port_;
        serial_port_ = NULL;
        is_connected_ = false;
        return false;
    } else {
        std::stringstream output;
        cout << "\nSuccessfully found NVS on port: " << port << "\n";
    }

    is_connected_ = true;
    StartReading();
    return true;
}

void NVS::Disconnect() {
    if (reading_status_)
        StopReading();
    if (serial_port_ != NULL) {
        if (serial_port_->isOpen())
            serial_port_->close();
        delete serial_port_;
        serial_port_ = NULL;
    }
}

bool NVS::Ping(int num_attempts) {
    try {
        while ((num_attempts--) > 0) {
            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

            unsigned char result[MAX_NOUT_SIZE];
            size_t bytes_read;

            // version message - ALVER
            uint8_t message[18] = {0x24, 
                                   0x47, 0x50, 
                                   0x47, 0x50, 
                                   0x51, 0x2c, 
                                   0x41, 0x4c, 0x56, 0x45, 0x52, 
                                   0x2A, 0x33, 0x31, 
                                   0x0D, 0x0A};

            bool version_sent = SendMessage(message, sizeof(message));

            bytes_read = serial_port_->read(result, MAX_NOUT_SIZE+5000);

            if (bytes_read < 8) {
                cout << "\nOnly read " << bytes_read << " bytes in response to ping";
                continue;
            }

            string result_;
            result_.append((char*) result);
            cout << "\nPing result: " << result_;

            return true;


            /*From ubx code directly*/
/*            uint16_t length;
            // search through result for version message
            for (int ii = 0; ii < (bytes_read - 8); ii++) {
                //std::cout << hex << (unsigned int)result[ii] << std::endl;
                if (result[ii] == 0xB5) {
                    if (result[ii + 1] != 0x62)
                        continue;
                    if (result[ii + 2] != 0x0A)
                        continue;
                    if (result[ii + 3] != 0x04)
                        continue;
                    //std::cout << "length1:" << hex << (unsigned int)result[ii+4] << std::endl;
                    //std::cout << "length2:" << hex << (unsigned int)result[ii+5] << std::endl;
                    length = (result[ii + 4]) + (result[ii + 5] << 8);
                    if (length < 40) {
                        cout << "Incomplete version message received";
                        //    //return false;
                        continue;
                    }

                    string sw_version;
                    string hw_version;
                    string rom_version;
                    sw_version.append((char*) (result + 6));
                    hw_version.append((char*) (result + 36));
                    //rom_version.append((char*)(result+46));
                    cout << "Ublox receiver found.";
                    cout << "Software Version: " << sw_version;
                    cout << "Hardware Version: " << hw_version;
                    //log_info_("ROM Version: " + rom_version);
                    return true;
                }
            }
            stringstream output;
            output << "Read " << bytes_read
                    << " bytes, but version message not found.";
            cout << output.str();
*/




        }
    } catch (exception &e) {
        stringstream output;
        cout << "\nError pinging receiver: " << e.what();
        return false;
    }
    return false;
}

void NVS::StartReading() {
    reading_status_ = true;
    read_thread_ptr_ = boost::shared_ptr<boost::thread>(
        new boost::thread(boost::bind(&NVS::ReadSerialPort, this)));
    cout << "\nBegun reading from receiver ";
}

void NVS::StopReading() {
    reading_status_ = false;
}

void NVS::ReadSerialPort() {
    uint8_t buffer[MAX_NOUT_SIZE];
    size_t len;

    // continuously read data from serial port
    while (reading_status_) {
        try{
            len = serial_port_->read(buffer, MAX_NOUT_SIZE);
        } catch (exception &e) {
            stringstream output;
            cout << "\nError reading serial port: " << e.what();
            Disconnect();
            return;
        }
        // Timestamp the read
        read_timestamp_ = time_handler_();
        // add data to the buffer to be parsed
        BufferIncomingData(buffer, len);
    }
}

void NVS::BufferIncomingData(uint8_t *msg, size_t len) {
    cout << "\nBuffering data";
    for (uint i = 0; i < len; i++) {
        if (buffer_index_ > MAX_NOUT_SIZE) {
            buffer_index_ = 0;
            cout << "\nReciever buffer overflow!";
        }

    }
}

bool NVS::SendMessage(uint8_t *msg, size_t len) {
    size_t bytes_written = serial_port_->write(msg, len);
    if (bytes_written == len)
        return true;
    else {
        cout << "Full message was not sent over serial port";
        return false;
    }
}