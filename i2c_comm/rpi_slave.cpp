// g++ -std=c++11 -lpigpio -lpthread -lrt rpi_slave.cpp -o rpi_slave
#include <stdio.h>
#include <pigpio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>
#include "../tcp_comm/DeskIlluminationData.hpp"
#include "rpi_slave.h"
#include <ctime>
#include <cstdlib>
#define SLAVE_ADDR 0x48

using namespace std;
using namespace std::chrono;

int init_slave(bsc_xfer_t &xfer, int addr) {
	
	gpioSetMode(18, PI_ALT3);
	gpioSetMode(19, PI_ALT3);
	
	xfer.control = (addr<<16) | /* Slave address */
				   (0x00<<13) | /* invert transmit status flags */
				   (0x00<<12) | /* enable host control */
				   (0x00<<11) | /* enable test fifo */
				   (0x00<<10) | /* invert receive status flags */
				   (0x01<<9) | /* enable receive */
				   (0x01<<8) | /* enable transmit */
				   (0x00<<7) | /* abort and clear FIFOs */
				   (0x00<<6) | /* send control reg as 1st I2C byte */
				   (0x00<<5) | /* send status regr as 1st I2C byte */
				   (0x00<<4) | /* set SPI polarity high */
				   (0x00<<3) | /* set SPI phase high */
				   (0x01<<2) | /* enable I2C mode */
				   (0x00<<1) | /* enable SPI mode */
					0x01 ; /* enable BSC peripheral */
	return bscXfer(&xfer);
}

int close_slave(bsc_xfer_t &xfer) {
	xfer.control = 0;
	return bscXfer(&xfer);
}

void i2c_slave_monitor(int sampling_frequency, DeskIlluminationData& data){

	int status, j;
	int desk;
	int time_stamp, i_meas, i_ref;
	int len = 8;
	char msg[len];

	if (gpioInitialise() < 0){
		cerr << "Error: " << strerror(errno);
		return;
	}

	bsc_xfer_t xfer;
	status = init_slave(xfer, SLAVE_ADDR);
	
	int block_size_in_seconds = 30;
	using clock = std::chrono::steady_clock;
	const auto times = sampling_frequency * block_size_in_seconds;
    const auto delay = std::chrono::microseconds{1000000 / sampling_frequency};
    auto next_sample = clock::now() + delay;
    
    std::srand(std::time(nullptr)); // use current time as seed for random generator
    int random_variable = std::rand();

    while(1){			
		xfer.txCnt = 0;
		status = bscXfer(&xfer);

		//if(xfer.rxCnt > 0){
		
			
		//	for(j=0;j<xfer.rxCnt;j++){
			//	msg[j] = xfer.rxBuf[j];
				
			//}
			i_meas = std::rand()/((RAND_MAX + 1u)/1024);
			i_ref = std::rand()/((RAND_MAX + 1u)/512);
            time_stamp = std::rand()/((RAND_MAX + 1u)/10);
			
			//sscanf (msg,"%d_%d_%d_%d", &desk,&time_stamp,&i_meas,&i_ref);
			
			data.new_sample(time_stamp, i_meas, i_ref);
		//}
				
		std::this_thread::sleep_until(next_sample);
		next_sample += delay;
	}

	status = close_slave(xfer);
	gpioTerminate();
	
	return;
}
