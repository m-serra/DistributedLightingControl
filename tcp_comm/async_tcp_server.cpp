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
#include "DeskIlluminationData.hpp"
#include "async_tcp_server.h"

using namespace std;
using boost::asio::ip::tcp;

class session
    : public std::enable_shared_from_this<session>
{
	tcp::socket socket_;
    enum { max_length = 8 };
    char msg_in[max_length];
    char str[max_length];
    //char* msg_out;
    char request; //g: current value; s: start/stop stream; b: last minute buffer
    char statistic;
    int data_tuple[3];
    int desk = -1;
    char *buff;
    
    DeskIlluminationData& data;
	
    public:
    
        session(tcp::socket socket, DeskIlluminationData& data_): 
            socket_(std::move(socket)),
            data(data_)
        {
			int n_samples_minute = data.get_n_samples_minute();
			buff = new char [(max_length+2) * n_samples_minute + 1];
		}

        void start(){
            do_read();
        }

    private:
  
        void do_read(){
            
            auto self(shared_from_this());
            socket_.async_read_some(boost::asio::buffer(msg_in, max_length),
                [this, self](boost::system::error_code ec, std::size_t length){
                
                if (!ec){
                    printf("Request: %s",msg_in);
                    
                    if(strlen(msg_in) == 1 && strcmp(msg_in, "r") == 0){
						// Handle reset request
					}
					else{
						sscanf (msg_in, "%c %c %d", &request, &statistic, &desk);
						
						if(request == 'g'){
							do_write();
						}
						else if(request == 'b'){
							do_write();
						}
						else{
							printf("Unknown request\n");
							do_read(); // sure??
						}
					}
                }
            });
        }

		// instead of char mode I can use the class field request. For now this is less
		// prone to errors
        void do_write(){
			
			char *msg_out;
			
            if(request == 'g'){
				data.get_last_sample(data_tuple);
				sprintf (str, "%d_%d_%d_%d\n", desk, data_tuple[0], data_tuple[1], data_tuple[2]);
				msg_out = new char [strlen(str)];
				strcpy(msg_out, str);
			}
			else if(request == 'b'){ 
				data.get_minute_history(desk, buff);
				msg_out = new char [strlen(buff)];
				strcpy(msg_out,buff);
			}
			  
            printf("Reply: %s\n", msg_out);
            int len = strlen(msg_out);
            
            auto self(shared_from_this());
            boost::asio::async_write(socket_, boost::asio::buffer(msg_out,len),
                [msg_out, this, self](boost::system::error_code ec, std::size_t /*length*/){
                    
                    if (!ec){
						printf("Message sent: %s\n", msg_out);
						/*if(request == 'b' && sample < data.get_n_samples_minute()){
							do_write(sample + 1);
						}
						else if(request == 's'){
							//Handle streamm request
						}
						else{*/
							//if current value or last value of last minute buffer
							delete [] msg_out;
							do_read();
						/*}*/
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
