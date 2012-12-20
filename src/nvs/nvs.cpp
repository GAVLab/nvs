/*
 *  Author: Robert Cofield, for GAVLab
 */
#include <nvs/nvs.h>

using namespace std;

/*
 *  Utility Methods
 */
double default_get_time() {
    boost::posix_time::ptime present_time(
            boost::posix_time::microsec_clock::universal_time());
    boost::posix_time::time_duration duration(present_time.time_of_day());
    return duration.total_seconds();
}

void sleep_msecs(unsigned int msecs) {
    boost::this_thread::sleep(boost::posix_time::milliseconds(msecs));
}

void print_hex(uint8_t byte) {
    printf("%.2x:", byte);
}

bool stob(const string& s) {
    return s != "0";
}

// get the index of the first occurence of a certain value in an array
deque<int> get_indices(uint8_t* array, const uint8_t des) {
    cout << "get_indices\n";
    deque<int> indices;
    size_t len = sizeof(array);
    for (int i = 0; i != len; i++) {
        if (array[i] == des)
            indices.push_back(i);
        print_hex(array[i]);
    }
    cout << "done getting indices\n";
    return indices;
}



/*
 *   Primary Receiver Class
 */
NVS::NVS() {
    time_handler_ = default_get_time;

    /* serial port */
    serial_port_ = NULL;
    reading_status_ = false;
    is_connected_ = false;

    /* Reading */
    updRate = 1;
    display_log_data_ = false;

    is_binr_ = false;
}

NVS::~NVS() {
    Disconnect();
}

bool NVS::Connect(string port, int baudrate) {
    // TODO correlate serial rate settings to receiver msg rate settings
    serial::Timeout timeout_ = serial::Timeout(serial_inter_byte_timeout_,
                                               serial_read_timeout_constant_,
                                               serial_read_timeout_multiplier_,
                                               serial_write_timeout_constant_,
                                               serial_write_timeout_multiplier_);
    try {
        serial_port_ = new serial::Serial(port, baudrate, timeout_);
    }
    catch (const exception & e) {
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
        cout << "\tBaudrate: " << baudrate << "\n";
    }

    serial_port_->flush();

    if (!Ping()) {
        cout << "NVS not found on port: " << port << "\n";
        delete serial_port_;
        serial_port_ = NULL;
        is_connected_ = false;
        Disconnect();
    } else {
        cout << "Successfully found NVS\n";
        is_connected_ = true;
    }

    StartReading();

    // TODO - have some loop be entered here for MOOS/ROS/etc -- for standalone usage, this api provides the WaitForCommand loop

    return true;
}

void NVS::Disconnect() {
    wait_for_command_ = false;
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

            return true;
        }
    } catch (exception &e) {
        cout << "\nError pinging receiver: " << e.what() << "\n";
        return false;
    }
    return false;
}

void NVS::StartReading() {
    reading_status_ = true;
    
    read_thread_ = boost::shared_ptr<boost::thread>
        (new boost::thread( boost::bind(&NVS::ReadSerialPort, this)));

    sleep_msecs(1000);

    //! FIXME Not switching to binary protocol?
    if (!is_binr_) { // instance uses NMEA
        SendMessage(setBINRMsgNMEA);
        sleep_msecs(1000);
        SendMessage(setBINRMsgBINR_);
        SendMessage(setBINRMsgBINR__);
    } else { // instance uses BINR
        SendMessage(setBINRMsgBINR__);
        sleep_msecs(100);
        SendMessage(reqParamMsg);
        SendMessage(reqVersionMsg);
        SendMessage(reqTestMsg);
    }
    
    // SendMessage(setBINRMsgNMEA);
    // SendMessage(reqSilenceMsg);
    // SendMessage(setBINRMsgBINR);
    // SendMessage(setBINRMsgBINR_);
    // SendMessage(setBINRMsgBINR__);
    // SendMessage("$GPGPQ,ALVER*31\r\n");

    // sleep_msecs(1000);
    // SendMessage(reqParamMsg);
    // SendMessage(reqVersionMsg);
    // SendMessage(reqTestMsg);
}

void NVS::StopReading() {
    reading_status_ = false;
    sleep_msecs(100);
}

void NVS::ReadSerialPort() {
    uint8_t new_data_buffer[MAX_NOUT_SIZE];
    size_t len;

    // continuously read data from serial port
    while (reading_status_) {
        try{
            len = serial_port_->read(new_data_buffer, MAX_NOUT_SIZE);
        } catch (exception &e) { // most likely unplugged
            cout << "Error reading serial port:\n\t" << e.what() << "\n";
            Disconnect();
            return;
        }
        // send if content
        if (len != 0) {
            // Timestamp the read
            read_timestamp_ = time_handler_();
            // cout << "Read Timestamp: " << read_timestamp_;
            // add data to the buffer to be parsed
            BufferIncomingData(new_data_buffer, len);
        } else {
            cout << "No content received\n";
            sleep_msecs(500);
        }
    }
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


void NVS::BufferIncomingData(uint8_t* new_data, size_t len) {
    cout << "BufferIncomingData\n";
    // ! Assume that each new message begins right after the old one ends
    //      no stray bytes
    //  TODO Check for overflow better
    if (len > MAX_NOUT_SIZE) {
        cout << "NVS Buffer Overflow. Dumping new data.\n";
        return;
    }

    if (len == 0)
        cout << "no data received\n";
    cout << "new_data: " << new_data << "\n";

    // TODO stop if buffer full
    // TODO pass if new_data empty
    // TODO compare checksums?

    header_indices_ = get_indices(new_data, header_byte_);
    footer_indices_ = get_indices(new_data, footer_byte_);

    printf("# header indices: %i\n", int(header_indices_.size()));
    printf("# footer indices: %i\n", int(footer_indices_.size()));

    if (header_indices_.size() < 2 || footer_indices_.size() < 1) {
        cout << "not given enough indices\n";
        return;
    }

    // deal with the front bytes
    if (header_indices_.front() == (footer_indices_.front()+1)) { // got the tail end of a message - new_data begins with 0x10 0x03
        header_indices_.pop_front(); // get rid of the first from each
        footer_indices_.pop_front();
    } else if (footer_indices_.front() < header_indices_.front())  // got the tail end of a message - new_data begins with 0x03
        footer_indices_.pop_front(); // get rid of the first footer

    if (header_indices_.size() < 2 || footer_indices_.size() < 1) {
        cout << "not enough there after trimming front\n";
        return;
    }

    // deal with the back bytes
    if (header_indices_.back() == (footer_indices_.back()+1)) { // new_data cuts off in the middle of a payload
        header_indices_.pop_back(); // get rid of last header
    } else if (header_indices_.back() == len) { // newd_data cuts of in the middle of a footer -- last byte is 0x10
        header_indices_.pop_back(); // get rid of last footer 0x10 and the last header 0x10
        header_indices_.pop_back();
    }

    // TODO check that we have a sane number of both
    cout << "Done checking back\n";

    while(header_indices_.size() > 1) {
        // get the next payload bounds
        payload_indices_[0] = header_indices_[0];
        payload_indices_[1] = header_indices_[1];
        
        // put the payload in the buffer
        for (int i = payload_indices_[0]; i != payload_indices_[1]; i++)
            msg_buffer_[i-payload_indices_[0]] = new_data[i];

        // have it parsed
        DelegateParsing();

        // clean up
        header_indices_.pop_front();
        header_indices_.pop_front();
        footer_indices_.pop_front();
        delete msg_buffer_;
    }
    cout << "Done sending to DelegateParsing\n";
}

void NVS::DelegateParsing() {
    cout << "Delegate Parsing\n";
    cout << "msg_buffer_: " << msg_buffer_ << "\n";
}


/*
 *  Parsing Functions for Specific Messages
 */

/*
 *  User input
 */

void NVS::ParseCommand(string cmd) {
    // Toggle log data display
    if (cmd == "d") {
        if (!display_log_data_) {
            display_log_data_ = true;
            cout << "Printing Log Data to screen\n";
        } else {
            display_log_data_ = false;
            cout << "Stopping Log Data print\n";
        }
        return;
    }
    // kill any messages set to come in
    if (cmd == "k") {
        // SendMessage(stopTransmissionMsg);
        // cout << "Killing all incoming messages.\n";
    }
    // request proper messages
    if (cmd == "r") {
    }
    // Disconnect
    if (cmd == "X") {
        Disconnect();
        cout << "Disconnecting\n";
    }
}


/*
 *  Send Functions
 */
bool NVS::SendMessage(string msg) {
    size_t len = msg.size();
    size_t bytes_written = serial_port_->write(msg);
    if (bytes_written == len)
        return true;
    else {
        cout << "Full message was not sent over serial port\n";
        return false;
    }
}

bool NVS::SendMessage(const uint8_t* msg) {
    size_t len = sizeof(msg);
    size_t bytes_written = serial_port_->write(msg, len);
    if (bytes_written == len)
        return true;
    else {
        cout << "Full message was not sent over serial port\n";
        return false;
    }
}

bool NVS::SendMessage(uint8_t* msg) {
    cout << "Fill me in\n";
    return false;
}


bool NVS::SendMessage(const vector< uint8_t > & msg) {
    cout << "Fill me in\n";
    return true;
}

