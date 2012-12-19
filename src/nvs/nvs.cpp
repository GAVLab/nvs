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


/*
 *  Messages to send to receiver
 */
// Request setting binary protocols
const static string setBINRMsg = "$PORZA,0,115200,3*7E\r\n";
const static string factoryResetMsg = "$PORST,F*20\r\n";

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
    data_buffer_ = queue<string>();

    display_log_data_ = false;

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
        cout << "\tBaudrate: " << baudrate << "\n";
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
}

void NVS::StartReading() {
    reading_status_ = true;
    
    // switch to BINR protocol
    SendMessage(setBINRMsg);

    read_thread_ = boost::shared_ptr<boost::thread>
        (new boost::thread( boost::bind(&NVS::ReadSerialPort, this)));

    // SendMessage(queryVersionMsg);
}

void NVS::StopReading() {
    reading_status_ = false;
    sleep_msecs(100);
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

// void log_data(string info) {
//     if (display_log_data_)
//         cout << info;
// }

void NVS::ReadSerialPort() {
    vector<string> new_data;

    size_t len;
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
        BufferIncomingData(new_data);
    }
}

void NVS::BufferIncomingData(vector<string> msgs) {
    size_t len = msgs.size();
    //  TODO Check for overflow
    // bool too_many = false;
    // if ((data_buffer_.size() + len) > max_buffer_size_) {
    //     too_many = true;
    //     cout << "buffer overflow! skipping messages\n";
    // }

    for (vector<string>::const_iterator it = msgs.begin();
                                        it != msgs.end(); it++) {

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

    if (display_log_data_)
        cout << "\nRead Timestamp: " << read_timestamp_ << "\n";

    // iterate over the buffer and send messages to the proper place until empty
    while (!data_buffer_.empty()) {
        msg.assign( data_buffer_.front() );
        data_buffer_.pop();

        // Send to correct parser function
        // TODO enumerate types

    }
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
// used to send NMEA message(s)
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


bool NVS::SendMessage(const vector< uint8_t > & msg) {
    return true;
}

