#include <thread>
#include <iostream>
#include "DeskIlluminationData.hpp"
#include "../i2c_comm/rpi_slave.h"
#include "async_tcp_server.h"

int controllers_client_interface(){

    int sampling_rate = 2;
    DeskIlluminationData data(sampling_rate);

    thread t1 {i2c_slave_monitor(sampling_rate, data)};
    cout << "Created Child Thread 1 # " << t1.get_id() << endl;
    
    thread t2 {run_tcp_server(17000, data)};
    cout << "Created Child Thread 2 # " << t2.get_id() << endl;
    
    t1.join(); //wait until t1 finishes
    t2.join(); //wait until t2 finishes
    
    return 0;

}