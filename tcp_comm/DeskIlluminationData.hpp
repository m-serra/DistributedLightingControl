#ifndef ILLUMINATION_DATA
#define ILLUMINATION_DATA
#include <iostream>
#include <mutex>

using namespace std;

class DeskIlluminationData{

    private:
        int n_samples_minute;
        int last_sample_pointer;
        
        mutex mtx;
    
    public:
        int n_samples;
        int *time_stamp;
        int *i_meas;
        int *i_ref;
        int sampling_frequency;
        
        DeskIlluminationData():
			sampling_frequency(0),
			n_samples(0),
			last_sample_pointer(-1),
			n_samples_minute(0)	
        {
			
		}

		
		void get_minute_history(char statistic, int *info);
        void new_sample(int time_, int i_meas_, int i_ref_);
        void get_last_sample(char statistic, int *info);
        void get_request_info(int desk, char request, char statistic, char *msg_out );
        void get_sample_i(int *data_tuple, int i);
        void set_sampling_frequency(int sampling_frequency_);
        int get_n_samples_minute();
        void info_to_string(char* msg, char request, int info, int *buffer);

};

#endif 
