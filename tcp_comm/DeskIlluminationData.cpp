#include <mutex>
#include "DeskIlluminationData.hpp"

void DeskIlluminationData::new_sample(int time_, int i_meas_, int i_ref_){

    if(last_sample_pointer == n_samples_minute){
		last_sample_pointer = -1;
    }

    mtx.lock();
    //printf("new_sample: %d_%d_%d\n", time_, i_meas_, i_ref_);
    time_stamp[last_sample_pointer + 1] = time_;
    i_meas[last_sample_pointer + 1] = i_meas_;
    i_ref[last_sample_pointer + 1] = i_ref_;
    last_sample_pointer += 1;
    n_samples += 1;
    last_sample_pointer += 1;
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

void DeskIlluminationData::get_sample_i(int *data_tuple, int i){
	
	/* If i is 0, method returns the most recent sample
	 * If i i 5, method returns the 5th most recent sample
	 * and so on.
	 */
	 
	 // careful, may need to check if i > n_samples_minute
	
	if(i + last_sample_pointer > n_samples_minute){
		k = i - (n_samples_minute - last_sample_pointer); //reset back to start of array
	}
	else{
		k = last_sample_pointer + i;
	}
	
	mtx.lock();
	data_tuple[0] = time_stamp[k];
    data_tuple[1] = i_meas[k];
    data_tuple[2] = i_ref[k];
	mtx.unlock();
	
	return
}

int DeskIlluminationData::get_n_samples_minute(){
	return n_samples_minute
}
