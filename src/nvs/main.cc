#include "nvs/nvs.h"
#include <iostream>

using namespace std;
using namespace nvs; 
int main (int argc, char *argv[]) {
    NVS *receiver = new NVS;

    // defaults";  
    string port = "/dev/ttyUSB1"; 
    string port1 = "/dev/ttyUSB1"; 
    int baudrate = 115200;
    int baudrate2= 115200;
    // CL parsing for port/baudrate
    // switch (argc) {
    //     case 4: // third argument
    //         receiver->is_binr_ = stob(argv[3]);
    //     case 3:
    //         istringstream(argv[2]) >> baudrate;
    //     case 2:
    //         port = argv[1];
    //         break;
    // }

    receiver->Connect(port, baudrate);
    // Because action is done in other threads, need to keep this one alive
    //receiver->WaitForCommand();


    // uint8_t chan[4];
    // chan[0]=0X10;
    // chan[1]=0x17;  
    // chan[2]=0x10;
    // chan[3]=0x03; 
    // bool result2 = receiver->SendMessage(&chan[0],4);
    // receiver->Reboot(0x00);
    




    uint8_t meas_interval=0x14;
    
    receiver->RequestRaw(meas_interval); 

    // receiver->RequestCurrent();
    



    while(1);

    return 0;
}


/*template <typename T, size_t N>
int get_index(const T (&array)[N], const T c) {
    const T* end = array + N;
    const T* match = find(array, array+N, c);
    return (end == match)? -1 : distance(array, match);
}



int main() {
    const size_t len = 9;
    const uint8_t bytes[len] = {0x04, 0x4D, 0x92, 0xD7, 0x03, 0x31};
    const uint8_t desired = 0x10;
    int positions = get_index(bytes, desired);
    cout << "bytes: " << bytes << "\n";
    cout << "positions: " << positions << "\n";
}*/
