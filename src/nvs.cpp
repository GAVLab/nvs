/*
    Author: Robert Cofield, for GAVLab
*/
#include <nvs/nvs.h>

using namespace std;

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
    // buffer_index_ = 0;
    // reading_acknowledgment_ = false;
    // bytes_remaining_ = false;
    // header_length_ = 0;
    // msgID_ = 0;
    // data_read_ = NULL;
    // read_timestamp_ = 0;
    // parse_timestamp_ = 0;

    /* Data Callbacks */

}

NVS::~NVS() {
    Disconnect();
}

bool NVS::Connect(string port, int baudrate) {
    serial::Timeout timeout_ = serial::Timeout(serial_inter_byte_timeout_,
                                               serial_read_timeout_constant_,
                                               serial_read_timeout_multiplier_,
                                               serial_write_timeout_constant_,
                                               serial_write_timeout_multiplier_);
    try {
        serial_port_ = new serial::Serial(port, baudrate, timeout_);
    }
    catch (exception e) {
        cout << "Failed to create connection on port: " << port 
            << "\nErr: " << e.what() << "\n";
        serial_port_ = NULL;
        is_connected_ = false;
        return false;
    }

    if (!serial_port_->isOpen()) {
        cout << "Failed create open port: " << port << "\n";
        delete serial_port_;
        serial_port_ = NULL;
        is_connected_ = false;
        return false;
    } else {
        cout << "Successfully opened port: " << port << "\n";
    }

    serial_port_->flush();

    if (!Ping()) {
        cout << "NVS not found on port: " << port << "\n";
        delete serial_port_;
        serial_port_ = NULL;
        is_connected_ = false;
        return false;
    } else {
        cout << "Successfully found NVS\n";
        is_connected_ = true;
    }

    StartReading();
    StartParsing();
    cout << GetVersion();

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

            unsigned char result[5000];
            size_t bytes_read;
            bytes_read = serial_port_->read(result, 5000);

            if (bytes_read < 8) {
                cout << "Only read " << bytes_read << " bytes in response to ping\n";
                continue;
            }

            string result_;
            result_.append((char*) result);
            // cout << "Ping result: " << result_ << "\n";

            return true;
        }
    } catch (exception &e) {
        cout << "\nError pinging receiver: " << e.what() << "\n";
        return false;
    }
}

void NVS::StartReading() {
    reading_status_ = true;
    read_thread_ = boost::shared_ptr<boost::thread>(
        new boost::thread(boost::bind(&NVS::ReadSerialPort, this)) );
    // read_thread_->join();
    cout << "Started Reading thread\n";
}

void NVS::StopReading() {
    reading_status_ = false;
}

void NVS::StartParsing() {

}

void NVS::StopParsing() {

}

void NVS::ReadSerialPort() {
    vector<string> new_data;
    size_t len;
    cout << "\tReading Serial Port\n";
    // continuously read data from serial port
    while (reading_status_) {
        try{
            // len = serial_port_->read(buffer, max_buffer_size_);
            new_data = serial_port_->readlines();
        } catch (exception &e) {
            cout << "Error reading serial port: " << e.what() << "\n";
            Disconnect();
            return;
        }
        // TODO pass if empty
        // Timestamp the read
        read_timestamp_ = time_handler_();
        // add data to the buffer to be parsed
        len = new_data.size();
        BufferIncomingData(new_data, len);
    }
}

void NVS::BufferIncomingData(vector<string> msgs, size_t len) {
    cout << "\tBufferIncomingData\n";
    // Check for overflow
    bool too_many = false;
    if ((data_buffer_.size() + len) > max_buffer_size_) {
        too_many = true;
        cout << "buffer overflow! skipping messages\n";
    }

    for (vector<string>::const_iterator it = msgs.begin();
                                        it != msgs.end(); it++) {
        cout << "\t" << *it;

        if ( (it->at( 0 ) != '$') | (it->at(it->length() - 5) != '*') ) {
            cout << "Received bad sentence\n";
            continue;
        }

    }
}

bool NVS::SendMessage(string msg, size_t len) {
    size_t bytes_written = serial_port_->write(msg);
    if (bytes_written == len)
        return true;
    else {
        cout << "Full message was not sent over serial port\n";
        return false;
    }
}

string NVS::GetVersion() {
    string message = "$GPGPQ,ALVER*31<CR><LF>";
    bool sent = SendMessage(message, message.size());

/*            // Find response
            bool found_version = false;
            string line;
            uint search = 0;
            while (!found_version) {
                bool version_sent = SendMessage(message, message.size());
                line = serial_port_->readline();
                cout << "\n" << line;
                if (string::npos != line.find("ALVER")) {
                    found_version = true;
                }
                search++;
                if (search > 50) {
                    cout << "\ncould not find version information";
                    break;
                }
            }
            cout << "\n" << line;*/

    return "version:\n";
}

void NVS::ParseData() {

}