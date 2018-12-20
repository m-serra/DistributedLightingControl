//
// async_tcp_echo_server.cpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <boost/asio.hpp>
#include <fstream>
#include <chrono>
#include <thread>
#include <cstdint>
#include "DeskIlluminationData.hpp"
#include "async_tcp_server.h"
#include <curses.h>


using namespace std;
using namespace std::chrono;
using boost::asio::ip::tcp;

class session
    : public std::enable_shared_from_this<session>
{
	tcp::socket socket_;
    enum { max_length = 6 };
    std::size_t msg_out_length = 8;
    char msg_in[max_length];
    char request; //g: current value; s: start/stop stream; b: last minute buffer
    char statistic;
    int data_tuple[3];
    int desk = -1;
    char *msg_out;
    int streaming = 0;
    	
   
    DeskIlluminationData& data;
	
    public:
    
        session(tcp::socket socket, DeskIlluminationData& data_): 
            socket_(std::move(socket)),
            data(data_)
        {
			int n_samples_minute = data.get_n_samples_minute();
			int max_precision = 4;

			msg_out = new char [(max_precision+1)*n_samples_minute+1];
		}

        void start(){
            do_read();
        }
    
    private:
          
        void do_read(){
			
			//static std::chrono::system_clock::time_point next_sample;
        
            auto self(shared_from_this());
            socket_.async_read_some(boost::asio::buffer(msg_in, max_length),
                [this, self](boost::system::error_code ec, std::size_t length){
                
                if (!ec){
                    printw("Request: %s",msg_in);
                    refresh();
                    
                    if(strlen(msg_in) == 1 && strcmp(msg_in, "r") == 0){
						// Handle reset request
					}
					else{
						sscanf (msg_in, "%c %c %d", &request, &statistic, &desk);
						if(request == 's'){
							
							if(streaming == 0){
								streaming = 1;
								//next_sample = std::chrono::system_clock::now() + std::chrono::milliseconds(5000);
								do_write();
							}
							else{
								streaming = 0;
								do_write();
							}
						}
						else if(request == 'g' || request == 'b'){
							do_write();
						}
						else{
							printw("Unknown request\n");
							refresh();
							do_read(); // sure??
						}
					}
                }
            });
            
            
            if(streaming >= 1){
				streaming += 1;
				do_write();
			}
        }

        void do_write(){
			
			using clock = std::chrono::steady_clock;
			const auto delay = std::chrono::microseconds{1000000 / 1};
			auto next_sample = clock::now() + delay;
			
			if( request == 's' && streaming == 0){
				msg_out[0] = 'a'; msg_out[1] = 'c'; msg_out[2] = 'k'; msg_out[3] = '\n'; msg_out[4] = '\0';
			}
			else{ 
				data.get_request_info(desk, request, statistic, msg_out);
			}
			
            printw("Reply: %s", msg_out);
            refresh();
            
            int msg_len = strlen(msg_out);
            msg_out_length = msg_len;

            printw("Sent: %d  bytes.\n\n", msg_len);
            refresh();
            
            auto self(shared_from_this());
            boost::asio::async_write(socket_, boost::asio::buffer(msg_out, msg_out_length),
                [next_sample, this, self](boost::system::error_code ec, std::size_t /*length*/){
                    
                    if (!ec){
						msg_out[0] = '\0';
						
						if( request == 's' && streaming != 0){
							
							std::this_thread::sleep_until(next_sample);
						
							clear();
							
							if(streaming == 1){
								do_read();
							} 
							else if(streaming > 1){
								streaming += 1;
								do_write();
							}							
						}
							
						if(request == 'g' || request == 'b' || ( request == 's' && streaming == 0)){
							do_read();
						}
                    }
                    else{
						msg_out[0] = '\0';
						do_read();
					}
            });
        }      
};

class server{
	
	DeskIlluminationData& data;
	
    public:
      
        server(boost::asio::io_service& io_service, short port, DeskIlluminationData& data_)
            : acceptor_(io_service, tcp::endpoint(tcp::v4(), port)),
            socket_(io_service),
            data(data_)   
        {
			
            do_accept();
        }

    private:
      
      void do_accept(){

        acceptor_.async_accept(socket_,
            [this](boost::system::error_code ec){
                if (!ec){
                    std::make_shared<session>(std::move(socket_), data)->start();
                }

                do_accept();
            });
      }

      tcp::acceptor acceptor_;
      tcp::socket socket_;
};

void run_tcp_server(int port, DeskIlluminationData& data){

    try{
        boost::asio::io_service io_service;
        server s(io_service, port, data);
        io_service.run();
    }
    catch(std::exception& e){
        std::cerr << "Exception: " << e.what() << "\n";
    }
}
