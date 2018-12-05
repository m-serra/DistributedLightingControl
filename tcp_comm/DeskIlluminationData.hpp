#ifndef ILLUMINATION_DATA
#define ILLUMINATION_DATA
#include <iostream>
#include <mutex>

using namespace std;

class DeskIlluminationData{

    private:
        int n_samples_minute;
        int last_sample_pointer;
        int sampling_frequency;
        mutex mtx;
    
    public:
        int n_samples;
        int *time_stamp;
        int *i_meas;
        int *i_ref;

        DeskIlluminationData(int sampling_frequency_){

            sampling_frequency = sampling_frequency_;
            n_samples = 0;
            last_sample_pointer = 0;
            n_samples_minute = 60 * sampling_frequency_;
            
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

        void new_sample(int time_, int i_meas_, int i_ref_);
        void get_last_sample(int *data_tuple);

};

#endif 
