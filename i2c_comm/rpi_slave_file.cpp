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

int main(int argc, char *argv[]) {
	
	int status, j, key = 0;
	ofstream myfile; 
    myfile.open ("../tcp_comm/i1.txt", ofstream::out | ofstream::trunc);
    
    if (!myfile.is_open()) {
		cerr << "Error: " << strerror(errno);
		return 1;
	}
	
	if (gpioInitialise() < 0){
		cerr << "Error: " << strerror(errno);
		return 1;
	}
	
	bsc_xfer_t xfer;
	status = init_slave(xfer, SLAVE_ADDR);
		
	int rate = 2;
	unsigned int block_size_in_seconds = 15;
	using clock = std::chrono::steady_clock;
	const auto times = rate * block_size_in_seconds;
    const auto delay = std::chrono::microseconds{1000000 / rate};
    auto next_sample = clock::now() + delay;

	//for(int i = 0; i < times; i++){
	while(1){			
		xfer.txCnt = 0;
		status = bscXfer(&xfer);

		if(xfer.rxCnt > 0){
			printf("Received %d bytes\n", xfer.rxCnt);
			
			for(j=0;j<xfer.rxCnt;j++){
				myfile << xfer.rxBuf[j];
				printf("%c",xfer.rxBuf[j]);
			}
			
			myfile << "\n";
			printf("\n");
		}
		
		myfile.seekp (0, myfile.beg); // reset file pointer
		
		std::this_thread::sleep_until(next_sample);
		next_sample += delay;
	}
	
	status = close_slave(xfer);
	gpioTerminate();
	myfile.close();
	
	return 0;
}
