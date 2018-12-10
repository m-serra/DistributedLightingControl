#include <mutex>
#include <string.h>
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
	int k = 0;
	
	if(i + last_sample_pointer > n_samples_minute){ //Compare with n_samples because a minute may still have not passed
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
	
	return;
}

void DeskIlluminationData::get_minute_history(int desk, char *buff){
	
	// ATENCAO: VOU IMPRIMIR DO MAIS RECENTE PARA O MAIS ANTIGO
	// TROCAR ORDEM NO FOR()
	
	int k = 0;
	int max_precision = 4;
	char d[2];
	char i_m[max_precision];
	char i_r[max_precision];
	char t[max_precision];
	sprintf(d, "%d_", desk);
	
	mtx.lock();
	for(int i = 0; i <  n_samples_minute; i++){
		
		if(i + last_sample_pointer > n_samples_minute){ //Compare with n_samples because a minute may still have not passed
			k = i - (n_samples_minute - last_sample_pointer); //reset back to start of array
		}
		else{
			k = last_sample_pointer + i;
		}
		
		strcat(buff, d);
		sprintf(t, "%d_", time_stamp[k]);
		strcat(buff, t);
		sprintf(i_m, "%d_", i_meas[k]);
		strcat(buff, i_m);
		
		//if(i == n_samples_minute - 1)
			//printf()
			//sprintf(i_r, "%d\n", i_ref[k]);
		//else
		sprintf(i_r, "%d | ", i_ref[k]);
		strcat(buff, i_r);		
	}
	
	strcat(buff, "\n");
	
	mtx.unlock();
}


int DeskIlluminationData::get_n_samples_minute(){
	return n_samples_minute;
}

void DeskIlluminationData::set_sampling_frequency(int sampling_frequency_){

		
	sampling_frequency = sampling_frequency_;
	n_samples_minute = 60 * sampling_frequency;
	
	try{
		time_stamp = new int [n_samples_minute];
		i_meas = new int [n_samples_minute];
		i_ref = new int [n_samples_minute];
	}
	catch (bad_alloc& ba){
		cerr << "bad_alloc caught: " << ba.what() << '\n';
	}

	for(int i = 0; i < n_samples_minute; i++){
		time_stamp[i] = -1;
		i_meas[i] = -1;
		i_ref[i] = -1;
	}
			
		
}
