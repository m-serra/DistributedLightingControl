#include <mutex>
#include "DeskIlluminationData.hpp"

void DeskIlluminationData::new_sample(int time_, int i_meas_, int i_ref_){

    if(last_sample_pointer == n_samples_minute){

    }

    mtx.lock();
    //printf("new_sample: %d_%d_%d\n", time_, i_meas_, i_ref_);
    last_sample_pointer = -1;
    time_stamp[last_sample_pointer + 1] = time_;
    i_meas[last_sample_pointer + 1] = i_meas_;
    i_ref[last_sample_pointer + 1] = i_ref_;
    last_sample_pointer +=1;
    n_samples += 1;
    last_sample_pointer = 0;
    mtx.unlock();
}

void DeskIlluminationData::get_last_sample(int *data_tuple){
  
    mtx.lock();
    data_tuple[0] = time_stamp[last_sample_pointer];
    data_tuple[1] = i_meas[last_sample_pointer];
    data_tuple[2] = i_ref[last_sample_pointer];
    mtx.unlock();

    return;
}
