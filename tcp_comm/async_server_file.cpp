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


using namespace std;
using boost::asio::ip::tcp;

class session
    : public std::enable_shared_from_this<session>
{
    public:
    
        session(tcp::socket socket): 
            socket_(std::move(socket))
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
			
			ifstream myfile;
			myfile.open ("i1.txt");
			
            if(!myfile.is_open()) {
				cerr << "Error: " << strerror(errno);
				return;
			}
            std::string line;
            std::getline(myfile, line);
            myfile.close();
            
            std::cout << "Reply: " << line << "\n\n";
            /*if(fin.is_open()) {
				fin.seekg(-1,ios_base::end); // go to one spot before the EOF

				bool keepLooping = true;
				while(keepLooping) {
					char ch;
					fin.get(ch);            // Get current byte's data

					if((int)fin.tellg() <= 1) { // If the data was at or before the 0th byte
						fin.seekg(0);           // The first line is the last line
						keepLooping = false;    // So stop there
					}
					else if(ch == '\n') {       // If the data was a newline
						keepLooping = false;    // Stop at the current position.
					}
					else {                      // If the data was neither a newline nor at the 0 byte
						fin.seekg(-2,ios_base::cur);   // Move to the front of that data, then to the front of the data before it
					}
				}
			
				string lastLine;            
				getline(fin,lastLine);                      // Read the current line
				cout << "Result: " << lastLine << '\n';     // Display it

				fin.close();
			}*/
			
            auto self(shared_from_this());
            boost::asio::async_write(socket_, boost::asio::buffer(line.c_str(), 8),
                [this, self](boost::system::error_code ec, std::size_t /*length*/){
                    
                    if (!ec){
                        do_read();
                    }
            });
        }

    tcp::socket socket_;
    enum { max_length = 8 };
    char data_[max_length];
};

class server{

    public:
      
        server(boost::asio::io_service& io_service, short port)
            : acceptor_(io_service, tcp::endpoint(tcp::v4(), port)),
            socket_(io_service)
        {
            do_accept();
        }

    private:
      
      void do_accept(){

        acceptor_.async_accept(socket_,
            [this](boost::system::error_code ec){
                if (!ec){
                    std::make_shared<session>(std::move(socket_))->start();
                }

                do_accept();
            });
      }

      tcp::acceptor acceptor_;
      tcp::socket socket_;
};

int main(int argc, char* argv[]){

  try{

      if (argc != 2){
        std::cerr << "Usage: async_tcp_echo_server <port>\n";
        return 1;
      }

      boost::asio::io_service io_service;

      server s(io_service, std::atoi(argv[1]));

      io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
