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
const static string queryVersionMsg = "$GPGPQ,ALVER*31\r\n";
const static string setBINRMsg = "$PORZA,0,115200,3*7E\r\n";
const static string setDecPlaceMsg = "$PONME,6,6*59\r\n";
const static string stopTransmissionMsg = "$PORZB*55\r\n";
const static string factoryResetMsg = "$PORST,F*20\r\n";
const static string requestPrimaryMsg = "$PORZB,GBS,2,GGA,2,RMC,2,PORZD,2*4D\r\n";

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
    
    // Set Decimals
    SendMessage(setDecPlaceMsg);

    read_thread_ = boost::shared_ptr<boost::thread>
        (new boost::thread( boost::bind(&NVS::ReadSerialPort, this)));

    SendMessage(queryVersionMsg);
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

    while (!data_buffer_.empty()) {
        msg.assign( data_buffer_.front() );
        data_buffer_.pop();

        // Decide how to break up: Proprietary/Standard?
        if ((msg.substr(0,1) == "P") || (msg.substr(0,1) == "A")) {
            message_id.assign( msg.substr(0,5) );
            payload.assign( msg.substr(6, (msg.size()-6))); //comma after msg id
        } else {
            talker_id.assign( msg.substr(0,2) );
            message_id.assign( msg.substr(2,3) );          
            payload.assign( msg.substr(6, (msg.size()-6))); //comma after msg id
        }

        if (display_log_data_)
            cout << "\tMessage ID: " << message_id << "\n";

        // Send to correct parser function
        // TODO enumerate types
        /* NMEA standard messages */
        if (message_id == "GGA")
            ParseGGA(talker_id, payload);
        else if (message_id == "GSV")
            ParseGSV(talker_id, payload);
        else if (message_id == "GSA")
            ParseGSA(talker_id, payload);
        else if (message_id == "GBS")
            ParseGBS(talker_id, payload);
        else if (message_id == "RMC")
            ParseRMC(talker_id, payload);
        /* Proprietary Messages */
        else if (message_id == "PORZD")
            ParsePORZD(payload);
        else if (message_id == "ALVER")
            ParseALVER(payload);
        else {
            cout << "\tReceived Unknown message id: " << message_id << "\n";
            cout << "\tMessage: " << msg << "\n";
        }
    }
}


/*
 *  Parsing Functions for Specific Messages
 */
/* NMEA standard messages */
void NVS::ParseGBS(string talker_id, string payload) {
    vector<string> fields;
    boost::split(fields, payload, boost::is_any_of(","));

    if (display_log_data_) {
        cout << "\t\tMessage Fix Time:   " << fields[0] << "\n";
        cout << "\t\tLatitude Error:  " << fields[1] << "\n";
        cout << "\t\tLongitude Error: " << fields[2] << "\n";
        cout << "\t\tAltitude Error:  " << fields[3] << "\n";
        cout << "\t\tStd Dev of Position: " << fields[7] << "\n";
    }
}

void NVS::ParseGGA(string talker_id, string payload) {
    // cout << "Parsing GGA\n";
    vector<string> fields;
    boost::split(fields, payload, boost::is_any_of(","));

    satCountUsing = atoi(fields[4].c_str());

    lat = atof(fields[1].c_str());
    lon = atof(fields[3].c_str());
    if (fields[2] == "S")
        lat = (-1.0f)*lat;
    if (fields[4] == "W")
        lon = (-1.0f)*lon;
    
    alt = atof(fields[7].c_str());

    if (display_log_data_) {
        cout << "\t\tTime of position fix: " << fields[0] << "\n";
        cout << "\t\t# of Satellites Using: " << satCountUsing << "\n";
        cout << "\t\tLatitude:  " << lat << "\n";
        cout << "\t\tLongitude: " << lon << "\n";
        cout << "\t\tAltitude:  " << alt << "\n";
    }
}

void NVS::ParseGSA(string talker_id, string payload) {
    vector<string> fields;
    boost::split(fields, payload, boost::is_any_of(","));

    pdop = atof(fields[fields.size()-3].c_str());
    hdop = atof(fields[fields.size()-2].c_str());
    vdop = atof(fields[fields.size()-1].c_str());

    if (display_log_data_) {
        // Mode: Auto 3D/2D or Manual 2D
        if (payload.substr(0,1) == "A")
            cout << "\t\tAutomatic 2D/3D mode\n";
        else if (payload.substr(0,1) == "M")
            cout << "\t\tManual 2D mode\n";
        else {
            cout << "\t\tunknown 2D/3D selection mode\n";
            return;
        }
        // Fix mode 2D or 3D
        if (payload.substr(2,1) == "1")
            cout << "\t\tFix Not Available\n";
        else if (payload.substr(2,1) == "2")
            cout << "\t\tFix Mode 2D\n";
        else if (payload.substr(2,1) == "3")
            cout << "\t\tFix Mode 3D\n";
        else {
            cout << "\t\tunknown fix mode\n";
            return;
        }
    }
}

void NVS::ParseGSV(string talker_id, string payload) {
    vector<string> fields;
    boost::split(fields, payload, boost::is_any_of(","));

    satCountView = atoi(fields[2].c_str());

    if (display_log_data_)
        cout << "\t\t# Satellites in View: " << satCountView << "\n";
}


void NVS::ParseRMC(string talker_id, string payload) {
    vector<string> fields;
    boost::split(fields, payload, boost::is_any_of(","));

    gpsTime = atof(fields[0].c_str());
    gpsDate = atof(fields[6].c_str());

    if (fields[1] == "A")
        dataIsValid = true;
    else
        dataIsValid = false;

    lat = atof(fields[2].c_str());
    lon = atof(fields[3].c_str());
    sog = atof(fields[4].c_str());
    cog = atof(fields[5].c_str());

    if (display_log_data_) {
        cout << "\t\tDate:  " << gpsDate << "\n";
        cout << "\t\tMessage Fix Time:   " << gpsTime << "\n";
        cout << "\t\tData Valid:  " << dataIsValid << "\n";
        cout << "\t\tLatitude:  " << lat << "\n";
        cout << "\t\tLongitude: " << lon << "\n";
        cout << "\t\tGround Speed:  " << sog << "\n";
        cout << "\t\tGround Course: " << cog << "\n";
    }
}

/* Proprietary Messages */
void NVS::ParseALVER(string payload) {
    vector<string> fields;
    boost::split(fields, payload, boost::is_any_of(","));

    manufacturer_ = fields[0];
    device_id_ = fields[1];
    firmware_version_ = fields[2];
    cout << manufacturer_ << "  " << device_id_ << "  (v " << firmware_version_ << ")\n";        
}

void NVS::ParsePORZD(string payload) {
    rmsError = atof(payload.substr(2,5).c_str());

    // Validity
    if (display_log_data_) {
        if (payload.substr(0,1) == "A")
            cout << "\t\tData Valid\n";
        else if (payload.substr(0,1) == "V")
            cout << "\t\tData Not Valid\n";
        else {
            cout << "\t\tData Validity is Unknown\n";
            return;
        }
        cout << "\t\tRMS Error: " << rmsError << "\n";
    }
}


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
        SendMessage(stopTransmissionMsg);
        cout << "Killing all incoming messages.\n";
    }
    // request proper messages
    if (cmd == "r") {
        cout << "Requesting NMEA Messages\n";
        RequestNMEAMsgs();
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

bool NVS::SendMessage(const vector< uint8_t > & msg) {
    return true;
}

void NVS::RequestNMEAMsgs() {
/*    // put together the desired payload
    stringstream request;
    const char* to_insert[] = {"GBS","GGA","RMC","PORZD"}; // list of the desired message codes
    vector<string> msgs (to_insert, to_insert+4);
    request << "PORZB";
    cout << "Requesting NMEA Messages @ " << updRate << " Hz:\n";
    for (vector<string>::const_iterator i = msgs.begin(); i != msgs.end(); i++) {
        request << "," << *i << "," << updRate;
        cout << "\t" << *i << "\n";
    }
    // cout << "desired payload: " << request.str() << "\n";
    
    // bridge the two bits of code
    const char* data_ = request.str().c_str();
    char data[sizeof(data_)];
    strcpy(data, data_);

    // bit of code that correctly computes a checksum
    // char data[] = "PORZB,GBS,10,GGA,10,RMC,10,PORZD,10";
    char *datapointer=&data[0];
    char checksum=0;
    while (*datapointer != '\0' && sizeof(checksum) < 1) {
        checksum ^= *datapointer;
        datapointer++;
    }
    const char* checksumpointer = &checksum;
    string cs = string(checksumpointer);
    ostringstream os;
    os << setw(2) << setfill('0') << hex << uppercase;
    copy(cs.begin(), cs.end(), ostream_iterator<unsigned int>(os, " "));
    string output = os.str();
    cout << "checksum: " << output << "\n";
    
    // Put it all together
    stringstream final_;
    final_ << "$" << request.str() << "*" << output << "\r\n";
    string final = final_.str();
    cout << "total message: " << final << "\n";
*/

    
    SendMessage(requestPrimaryMsg);
}