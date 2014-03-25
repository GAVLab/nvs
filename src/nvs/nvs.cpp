/*
 *  Author: Robert Cofield, for GAVLab
 */

 #include <nvs/nvs.h>

using namespace nvs; 
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

inline void printHex(uint8_t *data, int length) {
    for (int i = 0; i < length; ++i) {
        printf("0x%.2X ", (unsigned) (unsigned char) data[i]);
    }
    printf("\n");
}

inline void DefaultPortSettingsCallback(RspPort_Sts port_settings,
        double time_stamp) {
    std::cout << "RSP_SET:";
    cout << "Header" << port_settings.header.dle <<"\n"; 
    cout <<  "Message ID:" << port_settings.header.message_id <<"\n"; 
    cout << "Port No." << port_settings.port_no <<"\n"; 
    cout << "Port Settings" << port_settings.baud_rate <<"\n"; 
    cout << "Protocol" << port_settings.protocol <<"\n"; 
    cout << "Footer:" << port_settings.footer.dle ; 
    cout << port_settings.footer.etx <<"\n"; 
}

inline void DefaultRawDataCallback(RawData raw_data, 
        double time_stamp){
    //std::cout << "RAW_DATA:\n";
    // FOR TESTING
    cout << raw_data.time << "\n \n";
    for (unsigned int i = 0; i < 20; i++) {
        cout << (double)raw_data.raw_channel_measurements[i].sat_number << "\t";
        cout << raw_data.raw_channel_measurements[i].pseudo_range << "\n";
}
    cout << "\n";
    //cout << "Header:" << raw_data.header.dle << "\n"; 
    //cout << "Message ID:" << raw_data.header.message_id << "\n"; 
    // cout <<  "Data Time:" << raw_data.time << "   "; 
    // cout <<  "Data Week:" << raw_data.week_number << "  "; 
    // cout <<   "Time Shift:" << raw_data.gps_utc_time_shift; 
    // cout <<  "Time Correction:"  << raw_data.rec_time_correction << "\n"; 

    // TODO: if you want to see all data you'll need to loop through all reapeated fields,
    // I just have it showing the first channel's data for now
    // if (raw_data.raw_channel_measurements[0].signal_type== 0x01) {
    //     cout << "Signal Type:GLONASS\n";
    // }
    // else if (raw_data.raw_channel_measurements[0].signal_type= 0x02) {
    //     cout << "Signal Type: GPS \n";
    // }
    // else if (raw_data.raw_channel_measurements[0].signal_type= 0x04) {
    //     cout << "Signal Type: SBAS \n";
    // }
    //cout <<  "Signal Type:" << raw_data.raw_channel_measurements[0].signal_type << "\n"; 
    //cout <<  "Sat Number:" << (double)raw_data.raw_channel_measurements[0].sat_number << "\n"; 
    //cout <<  "Carrier Number (for GLONASS)" << raw_data.raw_channel_measurements[0].carrier_num << "\n";
    // cout <<  "Signal To Noise:"  << raw_data.raw_channel_measurements[0].sig_noise_ratio << "\n"; 
    // cout <<  "Carrier Phase:"  << raw_data.raw_channel_measurements[0].carrier_phase << "\n"; 
    //cout <<  "Pseudo Range:"  << raw_data.raw_channel_measurements[0].pseudo_range << "\n"; 
    // cout <<  "Doppler:" << raw_data.raw_channel_measurements[0].doppler_freq << "\n"; 
    // cout <<  "Raw Data Flags:" << raw_data.raw_channel_measurements[0].raw_data_flags << "\n"; 
    // cout << "Reserved" << raw_data.raw_channel_measurements[0].reserved << "\n"; 
    // cout <<  "Footer:" << hex << raw_data.footer.dle  << raw_data.footer.etx << "\n";  

}

inline void DefaultSoftwareCallback(RspSoftware software_version, 
        double time_stamp){
    
    std::cout << "RSP_SOFTW:\n";
    cout << "Header:" << hex << software_version.header.dle; 
    cout << "ID:" << hex << software_version.header.message_id << "\n"; 
    cout << "Number of Channels" << dec << software_version.num_channels << "\n"; 
    cout << "Version:" <<software_version.version_identifier << "\n"; 
    cout << "Serial Number:" << software_version.serial_num << "\n"; 
    // cout << software_version.reserved; 
    // cout << software_version.reserved2; 
    // cout << software_version.reserved3; 
    // cout << software_version.reserved4; 
    cout << "Footer:" << software_version.footer.dle; 
    cout << software_version.footer.etx << "\n";  

}

inline void DefaultChannelCallback(RspRec_Chan_Info channel_info, 
        double time_stamp) {
    std::cout << "RSP_CHANNEL_INFO:\n";
    cout << "Header:" << hex << channel_info.header.dle; 
    cout << "ID:" << hex << channel_info.header.message_id << "\n"; 
    cout << "Sat system" << dec << channel_info.sat_system << "\n"; 
    cout << "Sat Number:" <<channel_info.sat_number << "\n"; 
    cout << "Signal to noise ratio" << channel_info.sig_noise_ratio << "\n"; 
    cout << "Channel Status"  << channel_info.channel_sts << "\n"; 
    cout << "CHannel status:" <<channel_info.channel_sts2 << "\n"; 
    // cout << software_version.reserved; 
    // cout << software_version.reserved2; 
    // cout << software_version.reserved3; 
    // cout << software_version.reserved4; 
    cout << "Footer:" << channel_info.footer.dle; 
    cout << channel_info.footer.etx << "\n";  
}





bool stob(const string& s) {
    return s != "0";
}




/*
 *   Primary Receiver Class
 */
NVS::NVS() {
    time_handler_ = default_get_time;

    /* serial port */
    serial_port_usb = NULL;
    reading_status_ = false;
    is_connected_ = false;

    /* Reading */
    display_log_data_ = false;

    is_binr_ = false;
    buffer_index_ = 0;


    //CallBacks
    port_settings_callback_=DefaultPortSettingsCallback; 
    raw_data_callback_ = DefaultRawDataCallback;
    software_callback_=  DefaultSoftwareCallback;
    channel_callback_= DefaultChannelCallback; 

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
    serial::bytesize_t bytesize = serial::eightbits;
    serial::parity_t parity = serial::parity_odd;
    serial::stopbits_t stopbits = serial::stopbits_one;

try {
    //serial_port_usb = new serial::Serial(port, baudrate, timeout_);
    serial_port_usb = new serial::Serial(port, baudrate, timeout_,bytesize,parity,stopbits);
}
    
    catch (const exception & e) {
        cout << "Failed to create connection on port: " << port 
            << "\nErr: " << e.what() << "\n";
        serial_port_usb = NULL;
        is_connected_ = false;
        return false;
    }
    

    if (!serial_port_usb->isOpen()) {
        cout << "Failed create open port: " << port << "\n";
        delete serial_port_usb;
        serial_port_usb = NULL;
        is_connected_ = false;
        return false;
    } else {
        cout << "Successfully opened port: " << port << "\n";
        cout << "\tBaudrate: " << baudrate << "\n";
    }

    
    serial_port_usb->flush();

    if (false) { //!Ping()
        cout << "NVS not found on port: " << port << "\n";
        delete serial_port_usb;
       serial_port_usb = NULL;
        is_connected_ = false;
        Disconnect();
    } else {
        cout << "Successfully found NVS\n";
        is_connected_ = true;
    }
    
    
    StartReading();
    is_connected_=true;
    return true;
}







void NVS::Disconnect() {
    wait_for_command_ = false;
    if (reading_status_)
        StopReading();
    if (serial_port_usb != NULL) {
        if (serial_port_usb->isOpen())
            serial_port_usb->close();
        delete serial_port_usb;
        serial_port_usb = NULL;
    }
}

// bool NVS::Ping(int num_attempts) {
//     try {
//         while ((num_attempts--) > 0) {
//             sleep_msecs(1000);

//             unsigned char result[5000];
//             size_t bytes_read;
//             bytes_read = serial_port_usb->read(result, 5000);

//             if (bytes_read < 8) {
//                 cout << "Only read " << bytes_read << " bytes in response to ping\n";
//                 continue;
//             }

//             string result_;
//             result_.append((char*) result);

//             return true;
//         }
//     } catch (exception &e) {
//         cout << "\nError pinging receiver: " << e.what() << "\n";
//         return false;
//     }
//     return false;
// }



//SENDS MESSAGE TO SWITCH TO BINARY PROTOCOL
void NVS::StartReading() {
    reading_status_ = true;
    
    read_thread_ = boost::shared_ptr<boost::thread>
        (new boost::thread( boost::bind(&NVS::ReadSerialPort, this)));

    sleep_msecs(1000);

}
//SENDS BINARY MESSAGES

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
            len = serial_port_usb->read (new_data_buffer, MAX_NOUT_SIZE);
        } catch (exception &e) { // most likely unplugged
            cout << "Error reading serial port:\n\t" << e.what() << "\n";
            Disconnect();
            return;
        }
        // Timestamp the read
        read_timestamp_ = time_handler_();
        // cout << "Read Timestamp: " << read_timestamp_;
        // add data to the buffer to be parsed
        BufferIncomingData(new_data_buffer, len);
    }
}

// void NVS::WaitForCommand() {
//     wait_for_command_ = true;
//     string command;
//     while (wait_for_command_) {
//         cin >> command;
//         if (!command.empty())
//             ParseCommand(command);
//         sleep_msecs(10);
//     }
// }


bool NVS::Reboot(uint8_t reboot_type) {
        //Define Imputs in Main
        size_t length= 10; //Length of message 
    
        CntReboot message; 
        message.header.dle=NVS_DLE_BYTE; 
        message.header.message_id = NVS_CNT_RBT; 
        message.constant = 0x00; 
        message.constant2 = 0x01; 
        message.constant3 = 0x21; 
        message.constant4 = 0x01; 
        message.constant5 = 0x00; 
        message.reboot_type = reboot_type; // With erasing 0x00 without erasing 0x01
        message.footer.dle = NVS_DLE_BYTE;
        message.footer.etx = NVS_ETX_BYTE;

        unsigned char* msg_ptr = (unsigned char*) &message;
        return serial_port_usb->write(msg_ptr, sizeof(message)) == sizeof(message);
}
//FIX uint32_t?????
bool NVS::SetPortStatus(uint8_t port_no, uint32_t ex_speed, uint8_t protocol) {
    size_t length = 10; 
    CntPort_Status message; 
    message.header.dle=NVS_DLE_BYTE; 
    message.header.message_id = NVS_CNT_SET; 
    message.port_no= port_no; 
    message.ex_speed= ex_speed; 
    message.protocol= protocol; 
    message.footer.dle = NVS_DLE_BYTE;
    message.footer.etx = NVS_ETX_BYTE;
    

    unsigned char* msg_ptr = (unsigned char*) &message;
    return serial_port_usb->write(msg_ptr, sizeof(message)) == sizeof(message);


}

bool NVS::RequestCurrent() {
    Current_Status message; 
    message.header.dle=NVS_DLE_BYTE; 
    message.header.message_id = NVS_CNT_SET; 
    message.footer.dle = NVS_DLE_BYTE;
    message.footer.etx = NVS_ETX_BYTE;
    unsigned char* msg_ptr = (unsigned char*) &message;
    return serial_port_usb->write(msg_ptr, sizeof(message)) == sizeof(message);

}

bool NVS::RequestCourse(uint8_t output_rate){
    CntCourse_Ang_Speed message; 
    message.header.dle=NVS_DLE_BYTE; 
    message.header.message_id = NVS_CNT_CAAS;
    message.output_rate= output_rate; 
    message.footer.dle = NVS_DLE_BYTE;
    message.footer.etx = NVS_ETX_BYTE;
    unsigned char* msg_ptr = (unsigned char*) &message;
    return serial_port_usb->write(msg_ptr, sizeof(message)) == sizeof(message);
}
//FIX Request and loading should be different because this is only a function for 
//loading 1 sats ephem data, not all sats
bool NVS::RequestEphemeris(uint8_t sat_system, uint8_t sat_number){
    CntSV_Ephemeris message; 
    message.header.dle=NVS_DLE_BYTE; 
    message.header.message_id = NVS_CNT_SVEPHEM;
    message.sat_system= sat_system; 
    message.sat_number= sat_number; 
    message.footer.dle = NVS_DLE_BYTE;
    message.footer.etx = NVS_ETX_BYTE;
    unsigned char* msg_ptr = (unsigned char*) &message;
    return serial_port_usb->write(msg_ptr, sizeof(message)) == sizeof(message);
}

bool NVS::RequestAlmanac(uint8_t sat_system, uint8_t sat_number){
    CntAlmanac message; 
    message.header.dle=NVS_DLE_BYTE; 
    message.header.message_id = NVS_CNT_ALMAN;
    message.sat_system= sat_system; 
    message.sat_number= sat_number; 
    message.footer.dle = NVS_DLE_BYTE;
    message.footer.etx = NVS_ETX_BYTE;
    unsigned char* msg_ptr = (unsigned char*) &message;
    return serial_port_usb->write(msg_ptr, sizeof(message)) == sizeof(message);
}

bool NVS::RequestNumandDop(uint8_t output_rate) {
    CntNumber_Sat_Used message; 
    message.header.dle=NVS_DLE_BYTE; 
    message.header.message_id = NVS_CNT_NUMSATS;
    message.output_rate= output_rate;
    message.footer.dle = NVS_DLE_BYTE;
    message.footer.etx = NVS_ETX_BYTE;
    unsigned char* msg_ptr = (unsigned char*) &message;
    return serial_port_usb->write(msg_ptr, sizeof(message)) == sizeof(message);
}

bool NVS::RequestPVT(uint8_t output_rate) {
    CntPVT message; 
    message.header.dle=NVS_DLE_BYTE; 
    message.header.message_id = NVS_CNT_PVT;
    message.output_rate= output_rate;
    message.footer.dle = NVS_DLE_BYTE;
    message.footer.etx = NVS_ETX_BYTE;
    unsigned char* msg_ptr = (unsigned char*) &message;
    return serial_port_usb->write(msg_ptr, sizeof(message)) == sizeof(message);
}

bool NVS::RequestRaw(uint8_t meas_interval){
    CntRaw_Data message; 
    message.header.dle=NVS_DLE_BYTE; 
    message.header.message_id = 0xf4;
    message.meas_interval= meas_interval;
    message.footer.dle = NVS_DLE_BYTE;
    message.footer.etx = NVS_ETX_BYTE;
    unsigned char* msg_ptr = (unsigned char*) &message;
    return serial_port_usb->write(msg_ptr, sizeof(message)) == sizeof(message);
}


bool NVS::IsMessageId(uint8_t *id) {
    try {
        switch(*id) {
            case CNT_RBT:
                return true;
            case CNT_SET:
                return true;
            case RSP_SET:
                return true;
            case CNT_OPPARAM:
                return true;
            case RSP_OPPARAM:
                return true;
            case CNT_CANCEL:
                return true;
            case CNT_COOR:
                return true;
            case RSP_COOR:
                return true;
            case CNT_TEST:
                return true;
            case RSP_TEST:
                return true;
            case CNT_SATUSED:
                return true;
            case RSP_SATUSED:
                return true;
            case CNT_CAAS:
                return true;
            case RSP_CAAS:
                return true;
            case CNT_RECCHAN:
                return true;
            case RSP_RECCHAN:
                return true;
            case CNT_SVEPHEM:
                return true;
            case RSP_SVEPHEM:
                return true;
            case CNT_SOFTW:
                return true;
            case RSP_SOFTW:
                return true;
            case CNT_OPMOANT:
                return true;
            case RSP_OPMOANT:
                return true;
            case CNT_GPSTM:
                return true;
            case RSP_GPSTM:
                return true;
            case CNT_TMFREQ:
                return true;
            case RSP_TMFREQ:
                return true;
            case CNT_ALMAN:
                return true;
            case RSP_ALMAN:
                return true;
            case CNT_NUMSATS:
                return true;
            case RSP_NUMSATS:
                return true;
            case CNT_TMZONE:
                return true;
            case RSP_TMZONE:
                return true;
            case CNT_VISSAT:
                return true;
            case RSP_VISSAT:
                return true;
            case CNT_COMCHK:
                return true;
            case RSP_COMCHK:
                return true;
            case CNT_PVT:
                return true;
            case RSP_PVT:
                return true;
            case CNT_IONO:
                return true;
            case RSP_IONO:
                return true;
            case CNT_TMSCAL:
                return true;
            case RSP_TMSCAL:
                return true;
            case CNT_DOPRMS:
                return true;
            case RSP_DOPRMS:
                return true;
            case CNT_USEBLK:
                return true;
            case RSP_USEBLK:
                return true;
            case CNT_INFOCHN:
                return true;
            case RSP_INFOCHN:
                return true;
            case CNT_ATMOCOR:
                return true;
            case RSP_ATMOCOR:
                return true;
            case CNT_GDC:
                return true;
            case RSP_GDC:
                return true;
            case CNT_INSTCOOR:
                return true;
            case RSP_INSTCOOR:
                return true;
            case CNT_BINR:
                return true;
            case RSP_BINR:
                return true;
            case CNT_ADDPARAM:
                return true;
            case RSP_ADDPARAM:
                return true;
            case CNT_GLOTM:
                return true;
            case RSP_GLOTM:
                return true;
            case RAW_CNT:
                return true;
            case RAW_RSP:
            //cout << "Compared Correctly!!!!!\n";
                return true;
            case RAW_COOR:
                return true;
            case RAW_EPHEM:
                return true;
            case AID_CNT:
                return true;
            case AID_EPHEM:
                return true;
            case AID_ALMAN:
                return true;
            case AID_IONO:
                return true;
            case AID_TIME:
                return true;
            case AID_DIFF:
                return true;
            case AID_REF:
                return true;
            case AID_REFTM:
                return true;
        }
        return false;
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in NVS::IsMessageId(): " << e.what();
        // log_error_(output.str());
        return false;
    }
}


void NVS::BufferIncomingData(uint8_t *msg, size_t length) {
    //cout << length << endl;
    //cout << 0 << ": " << dec << (int)msg[0] << endl;
    // add incoming data to buffer 
    //std::cout << "Contents of msg: " << std::endl;
    //printHex(msg,length);
    uint8_t first_byte;
    uint8_t dle_byte; 
    uint8_t msgID;

    try {

        for (unsigned int i = 0; i < length; i++) {
            //cout << i << ": " << hex << (int)msg[i] << dec << endl;
            // make sure buffer_index_ is not larger than buffer
            if (buffer_index_ >= MAX_NOUT_SIZE) { // If
                buffer_index_ = 0;
                cout << "Reached Max Size Limit \n";
                
                //log_warning_("Overflowed receiver buffer. See nvs.cpp BufferIncomingData()");
            }
            //cout << "buffer_index_ = " << buffer_index_ << endl;

            if (buffer_index_ == 0) {   // looking for beginning of message
                if (msg[i] == NVS_DLE_BYTE) {  // DLE byte found
                    if (msg[i+1] != NVS_ETX_BYTE) { // DLE byte is not the end message one
                        if (IsMessageId(&msg[i+1])) { // DLE byte is the message start one
                            //cout << "Found Start of Message\n";
                            data_buffer_[buffer_index_++] = msg[i]; 
                            // cout << "Message ID BEGINNING="<< hex << msgID << "\t i=" << i <<"\n";       
                        }
                    }
                }   // end if (msg[i])
            } // end if (buffer_index_==0)

            else if (buffer_index_ == 1) {  // 2nd character of message is Message ID
                data_buffer_[buffer_index_++] = msg[i];
                msgID = msg[i];
                // cout << "Message ID-ID="<< hex << msgID << "\t i=" << i << "\n";

            }   // end else if (buffer_index_==1)

            else if ((msg[i] == NVS_DLE_BYTE) && (msg[i+1] == NVS_DLE_BYTE)) {
                // A double DLE byte is transmitted if the value 0x10h occurs in the data field.
                // This double DLE byte MUST be compressed into 1 byte before decoding!!
                // See BINR Protocol specs, Sect 3.1 BINR Message Struct
                //TODO: ABOVE
                //cout << "\nDOUBLE <DLE> BYTE PRESENT!!\n";
                // cout << "Message ID-DOUBLE DLE:" << msgID << "\t i=" << i << "\n"; 
                data_buffer_[buffer_index_++] = msg[i+1];
                i++;    

            }
            else if ((msg[i] == NVS_DLE_BYTE) && (msg[i+1] == NVS_ETX_BYTE)) {
                    data_buffer_[buffer_index_++] = msg[i];
                    data_buffer_[buffer_index_++] = msg[i+1];
                    // cout << "Found End of Message and Entering Parse Log\n";
                    // cout << "Message ID-END:" << msgID << "\t i=" << i <<  "\n"; 
                    ParseLog(data_buffer_, msgID,buffer_index_);
                    // reset counter
                    buffer_index_ = 0;
                    msgID = 0; 
                    i++;
                    //data_buffer_[buffer_index_++] = msg[i];
                   
                    //printHex(dle_byte,a);

            } // end if ((msg[i+1] == NVS_SYNC_BYTE)
            
            else {  // add data to buffer
                data_buffer_[buffer_index_++] = msg[i];
                // cout << "Message ID=-ALL DATA"<< hex << msgID <<"\t";
                // cout << msg[i] << "\t i=" << dec << i << hex <<  "\n";
            }
        }// end for
    } catch (std::exception &e) {
        std::stringstream output;
        output << "Error in NVS::BufferIncomingData(): " << e.what();
        //log_error_(output.str());
    }
}



void NVS::ParseLog(unsigned char* data_buffer_, uint8_t id, size_t buffer_index_){
    // cout << "Entered Parse Log \n";
    // cout << "Message ID:" << id << "\n"; 
    // printHex(data_buffer_, buffer_index_);
    switch(id){
        
        case RSP_SET:
        // SavePortSettings(data_buffer_)
        cout << "Reached Parse Log\n";
        RspPort_Sts cur_port_settings; 
        payload_length = buffer_index_; 
        memcpy(&cur_port_settings, data_buffer_,payload_length);
        if (port_settings_callback_)
            port_settings_callback_(cur_port_settings,read_timestamp_); 
        break; 

        case RAW_RSP:
        //cout << "\nCase Raw Rsp:\n";
        RawData raw_data; 
        payload_length=buffer_index_;
        //cout << "Payload Length:" << payload_length << "\n"; 
        // Copy Beginning of message and repeated fields
        memcpy(&raw_data, data_buffer_, payload_length-NVS_FOOTER_LENGTH);
        // Copy footer 
        memcpy(&raw_data.footer,data_buffer_+payload_length-NVS_FOOTER_LENGTH, NVS_FOOTER_LENGTH);


        //cout << "Copied to memory \n"; 
        if (raw_data_callback_)
            raw_data_callback_(raw_data, read_timestamp_); 
        break; 

        case RSP_SOFTW:
        RspSoftware software_version;
        payload_length= buffer_index_; 
        memcpy(&software_version, data_buffer_, payload_length);
        if (software_callback_)
            software_callback_(software_version, read_timestamp_);
            //cout << "Saved Software version\n";
        break; 

        case RSP_IONO:
        //cout << "\nIonosphere Strtucture Incomplete, Will Not Save\n";
        break; 

        case RSP_TMSCAL:
        //cout << "\nTime Scale Strtucture Incomplete, Will Not Save\n";
        break; 

        case RAW_COOR:
        //cout << "\nCoordniate Strtucture Incomplete, Will Not Save\n";
        break;

        case RAW_EPHEM:
        //cout << "\nEphemeris Strtucture Incomplete, Will Not Save\n";
        break;

        case RAW_CNT:
        //cout << "\nRAW_CNT case reached in ParseLog().";
        break;

        case RSP_RECCHAN:
        RspRec_Chan_Info channel_info; 
        payload_length=buffer_index_; 
        // memcpy(&channel_info, data_buffer_, payload_length);
        // if(channel_callback_)
        //     channel_callback_(channel_info, read_timestamp_);
        // break; 

        default:
        printHex( data_buffer_, buffer_index_);

    } 
}

// bool NVS::SavePortSettings(unsigned char* data_buffer_){

   
// }

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
// bool NVS::SendMessage(std::string msg) {
//     size_t len = msg.size();
//     size_t bytes_written = serial_port_usb->write(msg);
//     if (bytes_written == len)
//         return true;
//     else {
//         cout << "Full message was not sent over serial port\n";
//         return false;
//     }
// }

bool NVS::SendMessage(uint8_t* msg, size_t length) {
    cout << "SENDMESSAGE()\n";
    size_t bytes_written = serial_port_usb->write(msg, length);
    if (bytes_written == length)
        return true;
    else {
        cout << "Full message was not sent over serial port\n";
        return false;
    }
}



// bool NVS::SendMessage(uint8_t* msg) {
//     cout << "Fill me in\n";
//     return false;
// }


// bool NVS::SendMessage(const vector< uint8_t > & msg) {
//     cout << "Fill me in\n";
//     return true;
// }

