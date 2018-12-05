#include <mutex>
#include <iostream>
#include <thread>
#include "DeskIlluminationData.hpp"
#include "../i2c_comm/rpi_slave.h"
#include "async_tcp_server.h"

using namespace std;

mutex mtx;

int main(){
	
	
    int sampling_frequency = 2;
    
    DeskIlluminationData data{sampling_frequency};

    thread t1 {i2c_slave_monitor, sampling_frequency, ref(data)};
    cout << "Created Child Thread 1 # " << t1.get_id() << endl;
    
    thread t2 {run_tcp_server, 17000, ref(data)};
    cout << "Created Child Thread 2 # " << t2.get_id() << endl;
    
    t1.join(); //wait until t1 finishes
    t2.join(); //wait until t2 finishes
    
    return 0;

}

/*int main(){
	controllers_client_interface();
	return 0;
}*/
