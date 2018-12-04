#ifndef RPI_SLAVE
#define RPI_SLAVE
#import "DeskIlluminationData.hpp"
void i2c_slave_monitor(int sampling_rate, DeskIlluminationData data);
#endif