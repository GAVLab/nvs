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


    /* Callbacks */

}

NVS::~NVS() {
    Disconnect();
}

bool NVS::Connect(std::string port, int baudrate) {
    serial::Timeout timeout(100, 1000, 0, 1000, 0);
    try {
        serial_port_ = new serial::Serial(port, baudrate, timeout);
    }
    catch (std::exception e) {
        std::stringstream output;
        output << "Failed create connection on port: " << port << "\nErr: " << e.what();
        log_error_(output.str());
        serial_port_ = NULL;
        is_connected_ = false;
        return false;
    }

    if (!serial_port_->isOpen()) {
        std::stringstream output;
        output << "Failed create open port: " << port;
        log_error_(output.str());
        delete serial_port_;
        serial_port_ = NULL;
        is_connected_ = false;
        return false;
    } else {
        std::stringstream output;
        output << "Successfully opened port: " << port;
        log_info_(output.str());
    }

    serial_port_->flush();

    if (!Ping()) {
        std::stringstream output;
        output << "NVS not found on port: " << port;
        log_error_(output.str());
        delete serial_port_;
        serial_port_ = NULL;
        is_connected_ = false;
        return false;
    } else {
        std::stringstream output;
        output << "Successfully found NVS on port: " << port;
        log_info_(output.str());
    }

    is_connected_ = true;
    // StartReading();
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
            log_info_("Searching for NVS receiver...");
            // boost::this_thread::sleep(boos::posix_time::milliseconds(1000));

            unsigned char result[MAX_NOUT_SIZE];
            size_t bytes_read;
            bytes_read = serial_port_->read(result, MAX_NOUT_SIZE);

            if (bytes_read < 8) {
                stringstream output;
                output << "Only read " << bytes_read
                        << " bytes in response to ping.";
                log_warning_(output.str());
                continue;
            }

            // version message - ALVER

            return true;
        }
    } catch (exception &e) {
        stringstream output;
        output << "Error pinging receiver: " << e.what();
        log_error_(output.str());
        return false;
    }
    return false;
}

void NVS::StartReading() {
    reading_status_ = true;
    read_thread_ptr_ = boost::shared_ptr<boost::thread>(
        new boost::thread(boost::bind(&NVS::ReadSerialPort, this)));
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
            output << "Error reading serial port: " << e.what();
            log_error_(output.str());
            Disconnect();
            return;
        }
        // Timestamp the read
        read_timestamp_ = time_handler_();
        // add data to the buffer to be parsed
        // BufferIncomingData(buffer, len);
    }
}