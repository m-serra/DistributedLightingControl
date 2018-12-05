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
    char data_[max_length];
    char msg_out[max_length];
    int data_tuple[3];
    int desk = 1;
    DeskIlluminationData& data;
	
    public:
    
        session(tcp::socket socket, DeskIlluminationData& data_): 
            socket_(std::move(socket)),
            data(data_)
        {}

        void start(){
            do_read();
        }

    private:
  
        void do_read(){
            
            auto self(shared_from_this());
            socket_.async_read_some(boost::asio::buffer(data_, max_length),
                [this, self](boost::system::error_code ec, std::size_t length){
                
                if (!ec){
                    printf("Request: %s\n", data_);
                    do_write(length);
                }
            });
        }

        void do_write(std::size_t length){
			printf("WRITEEEEEEE\n");
            std::string line;
            data.get_last_sample(data_tuple);
            sprintf (msg_out, "%d_%d_%d_%d", desk, data_tuple[0], data_tuple[1], data_tuple[2]);
            
            
            std::cout << "Reply: " << msg_out << "\n\n";
                
            auto self(shared_from_this());
            boost::asio::async_write(socket_, boost::asio::buffer(msg_out, max_length),
                [this, self](boost::system::error_code ec, std::size_t /*length*/){
                    
                    if (!ec){
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
