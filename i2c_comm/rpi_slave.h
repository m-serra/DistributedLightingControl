#ifndef RPI_SLAVE
#define RPI_SLAVE
#include "../tcp_comm/DeskIlluminationData.hpp"
void i2c_slave_monitor(int sampling_rate, DeskIlluminationData& data);
#endif
