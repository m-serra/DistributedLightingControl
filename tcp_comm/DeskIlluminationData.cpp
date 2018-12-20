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

void DeskIlluminationData::get_last_sample(char statistic, int *info){

    mtx.lock();
    if(statistic == 't') // apagar
		*info = time_stamp[last_sample_pointer];
	
	if(statistic == 'i')
		*info = i_meas[last_sample_pointer];
    
    if(statistic == 'r')
		*info = i_ref[last_sample_pointer];
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

void DeskIlluminationData::get_minute_history(char statistic, int *info){

	// ATENCAO: VOU IMPRIMIR DO MAIS RECENTE PARA O MAIS ANTIGO
	// TROCAR ORDEM NO FOR()

	int k = 0;
	

	mtx.lock();
	for(int i = 0; i <  n_samples_minute; i++){

		if(i + last_sample_pointer > n_samples_minute){ //Compare with n_samples because a minute may still have not passed
			k = i - (n_samples_minute - last_sample_pointer); //reset back to start of array
		}
		else{
			k = last_sample_pointer + i;
		}
		
		if(statistic == 'i')
			info[i] = i_meas[k];
	}

	mtx.unlock();
}

void DeskIlluminationData::info_to_string(char *msg, char request, int info, int *info_buffer){
	
	int max_precision = 4;
	char s[max_precision];
	

	if(request == 'g' || request == 's'){
		sprintf(msg, "%d\n", info);
	}
	else if(request == 'b'){
		for(int i = 0; i < n_samples_minute; i++){
			sprintf(s, "%d", info_buffer[i]);
			strcat(msg, s);

			if(i + 1 != n_samples_minute)
				strcat(msg, ",");
		}
		strcat(msg, "\n");
	}	
}

void DeskIlluminationData::get_request_info(int desk, char mode, char statistic, char *msg_out ){

	int max_precision = 4; //9999
	int info;
	int *info_buffer;
	
	if(mode == 'g' || mode =='s'){
		//*msg_out = new char [max_precision+1];
		get_last_sample(statistic, &info);
	}
	else if(mode == 'b'){ 
		
		//*msg_out = new char [(max_precision+1)*n_samples_minute+1];
		info_buffer = new int [n_samples_minute];
		
		for( int i = 0; i < n_samples_minute; i ++)
			info_buffer[i] = 0;
		
		get_minute_history(statistic, info_buffer);		
	}
	
	if(statistic == 'i'){
		info_to_string((msg_out), mode, info, info_buffer);
	}
	else if(statistic == 'd'){
		//duty cycle
		// compute using illuminance
	}
	else if(statistic == 's'){
		//occupancy state
	}
	else if(statistic == 'L'){
		//illuminance lower bound
	}
	else if(statistic == 'o'){
		//external illuminance
	}
	else if(statistic == 'r'){
		//illuminance control reference
	}
	else if(statistic == 'p'){
		//instantaneous power consuption (each desk and total)
	}
	else if(statistic == 't'){
		//elapsed time since last restart
		// se a mensagem lida for um restart iniciar um timer
	}
	else if(statistic == 'e'){
		//acumulated energy consuption since last restart (each desk ad total)
	}
	else if(statistic == 'c'){
		//acumulated comfort error since last restart (each desk and total)
	}
	else if(statistic == 'v'){
		//acumulated comfort flicker
	}
	

	if(mode == 'b')
		delete [] info_buffer;
	
	return;
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
