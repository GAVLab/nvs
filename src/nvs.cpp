/*
    Author: Robert Cofield, for GAVLab
*/
#include <nvs/nvs.h>

using namespace std;

/*
    Utility Methods
*/
double DefaultGetTime() {
    boost::posix_time::ptime present_time(
            boost::posix_time::microsec_clock::universal_time());
    boost::posix_time::time_duration duration(present_time.time_of_day());
    return duration.total_seconds();
}

void sleep_msecs(unsigned int msecs) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(msecs));
}

////////////////////////////////////////////////////////////////////////////////

NVS::NVS() {
    time_handler_ = DefaultGetTime;

    /* serial port */
    serial_port_ = NULL;
    reading_status_ = false;
    is_connected_ = false;

    /* Reading */
    data_buffer_ = queue<string>();
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
    // TODO correlate serial rate settings to receiver settings
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
            sleep_msecs(1000);

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
    cout << "Start Reading\n";

    read_thread_ = boost::shared_ptr<boost::thread>
        (new boost::thread( boost::bind(&NVS::ReadSerialPort, this)));

    // put settings/requests here

    // ReadSerialPort();    
}

void NVS::StopReading() {
    reading_status_ = false;
}

void NVS::WaitForCommand() {
    wait_for_command_ = true;
    string command;
    while (wait_for_command_) {
        cin >> command;
        if (!command.empty())
            ParseCommand(command);
        sleep_msecs(10);
    }
}

void NVS::ReadSerialPort() {
    vector<string> new_data;
    size_t len;
    cout << "Reading Serial Port\n";
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
        // cout << "Read Timestamp: " << read_timestamp_;
        // add data to the buffer to be parsed
        len = new_data.size();
        BufferIncomingData(new_data, len);
    }
}

void NVS::BufferIncomingData(vector<string> msgs, size_t len) {
    //  TODO Check for overflow
    // bool too_many = false;
    // if ((data_buffer_.size() + len) > max_buffer_size_) {
    //     too_many = true;
    //     cout << "buffer overflow! skipping messages\n";
    // }

    for (vector<string>::const_iterator it = msgs.begin();
                                        it != msgs.end(); it++) {
        // cout << "\t" << *it;

        // TODO stop if buffer full
        // TODO compare checksums

        uint end_pos = it->size() - 5;
        if ( (it->at( 0 ) != '$') | (it->at(end_pos) != '*') ) {
            cout << "Received bad sentence\n";
            continue;
        }

        data_buffer_.push( it->substr(1, end_pos - 1) );
        // cout << "Payload: " << data_buffer_.back() << "\n";
    }

    DelegateParsing();
}

void NVS::DelegateParsing() {
    // cout << "Delegate Parsing Data\n";
    string msg;
    string talker_id;
    string message_id;
    string payload;

    if (data_buffer_.empty())
        return;

    cout << "\nRead Timestamp: " << read_timestamp_ << "\n";

    while (!data_buffer_.empty()) {
        msg.assign( data_buffer_.front() );
        data_buffer_.pop();

        // Decide how to break up: Proprietary/Standard?
        if ((msg.substr(0,1) == "P") || (msg.substr(0,1) == "A")) {
            message_id.assign( msg.substr(0,5) );
            cout << "\tMessage ID: " << message_id << "\n";
            payload.assign( msg.substr(6, (msg.size()-6))); //comma after msg id
        } else {
            talker_id.assign( msg.substr(0,2) );
            message_id.assign( msg.substr(2,3) );
            cout << "\tMessage ID: " << message_id << "\n";
            payload.assign( msg.substr(6, (msg.size()-6))); //comma after msg id
        }

        // Send to correct parser function
        // TODO enumerate types
        /* NMEA standard messages */
        if (message_id == "GGA") {
            ParseGGA(talker_id, payload);
            continue;
        }
        if (message_id == "GSV") {
            ParseGSV(talker_id, payload);
            continue;
        }
        if (message_id == "GSA") {
            ParseGSA(talker_id, payload);
            continue;
        }
        /* Proprietary Messages */
        if (message_id == "PORZD") {
            ParsePORZD(payload);
            continue;
        }
    }
}


/*
    Parsing Functions for Specific Messages
*/
/* NMEA standard messages */
void NVS::ParseGGA(string talker_id, string payload) {
    // cout << "Parsing GGA\n";
    vector<string> fields;
    boost::split(fields, payload, boost::is_any_of(","));
    cout << "\t\tTime of position fix: " << fields[0] << "\n";
    cout << "\t\t# of Satellites Using: " << fields[5] << "\n";
    cout << "\t\tLatitude:  " << fields[1] << " " << fields[2] << "\n";
    cout << "\t\tLongitude: " << fields[3] << " " << fields[4] << "\n";
    cout << "\t\tAltitude:  " << fields[7] << "\n";
}

void NVS::ParseGSV(string talker_id, string payload) {
    cout << "\t\tParsing GSV\n";
}

void NVS::ParseGSA(string talker_id, string payload) {
    // cout << "Parsing GGA\n";
    // Mode: Auto 3D/2D or Manual 2D
    if (payload.substr(0,1) == "A") {
        cout << "\t\tAutomatic 2D/3D mode\n";
    } else if (payload.substr(0,1) == "M") {
        cout << "\t\tManual 2D mode\n";
    } else {
        cout << "\t\tunknown 2D/3D selection mode\n";
        return;
    }

    // Fix mode 2D or 3D
    if (payload.substr(2,1) == "1") {
        cout << "\t\tFix Not Available\n";
    } else if (payload.substr(2,1) == "2") {
        cout << "\t\tFix Mode 2D\n";
    } else if (payload.substr(2,1) == "3") {
        cout << "\t\tFix Mode 3D\n";
    } else {
        cout << "\t\tunknown fix mode\n";
        return;
    }
}

/* Proprietary Messages */
void NVS::ParsePORZD(string payload) {
    // cout << "Parsing PORZD\n";
    // Validity
    if (payload.substr(0,1) == "A") {
        cout << "\t\tData Valid\n";
    } else if (payload.substr(0,1) == "V") {
        cout << "\t\tData Not Valid\n";
    } else {
        cout << "\t\tData Validity is Unknown\n";
        return;
    }

    // Value of RMS error
    rmsError = atof(payload.substr(2,5).c_str());
    cout << "\t\tRMS Error: " << rmsError << "\n";
}


/*
    User input
*/

void NVS::ParseCommand(string cmd) {
    if (cmd == "v") {
        if (GetVersion())
            cout << "Query Sent: GetVersion\n";
    }
}



/*
    Send Functions
*/

bool NVS::SendMessage(string msg, size_t len) {
    size_t bytes_written = serial_port_->write(msg);
    if (bytes_written == len)
        return true;
    else {
        cout << "Full message was not sent over serial port\n";
        return false;
    }
}

bool NVS::SendMessage(uint8_t msg, size_t len) {
    return true;
}

bool NVS::GetVersion() {
    bool sent = SendMessage(queryVersionMsg, string(queryVersionMsg).size());
    return sent;
}