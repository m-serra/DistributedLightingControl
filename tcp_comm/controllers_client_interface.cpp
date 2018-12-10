// g++ -std=c++11 -lrt -lpigpio -pthread -lboost_system *.cpp ../i2c_comm/rpi_slave.cpp -o controllers_client_interface
#include <mutex>
#include <iostream>
#include <thread>
#include "DeskIlluminationData.hpp"
#include "../i2c_comm/rpi_slave.h"
#include "async_tcp_server.h"

using namespace std;

mutex mtx;

int main(int argc, char* argv[]){
	
	if(argc < 2){
		printf("Please provide the number of luminaires\n");
		return 1;
	}
	
    int sampling_frequency = 2;
    int n_luminaires = atoi(argv[1]);
    
    // Make array of data, one for each luminaire.
    // Ideally receive information about how many luminaires the system has
    
    //DeskIlluminationData data[n_luminaires];
    DeskIlluminationData data;
    n_luminaires = 1; // DELETE
    
    //initialize data objects
    for(int i = 0; i < n_luminaires;i++){
		data.set_sampling_frequency(sampling_frequency);
	}

    thread t1 {i2c_slave_monitor, sampling_frequency, ref(data)};
    cout << "Created Child Thread 1 # " << t1.get_id() << endl;
    
    thread t2 {run_tcp_server, 17000, ref(data)};
    cout << "Created Child Thread 2 # " << t2.get_id() << "\n\n";
    
    t1.join(); //wait until t1 finishes
    t2.join(); //wait until t2 finishes
    
    return 0;

}

/*int main(){
	controllers_client_interface();
	return 0;
}*/
