#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>

using namespace boost::asio;
using ip::tcp;

int main(int argc, char** argv){

    io_service io;
    std::string msg_out;
    size_t len;

    boost::system::error_code err;
    tcp::resolver resolver(io); //finds endpoints from address
    //tcp::resolver::query query("192.168.27.149", "17000");
    tcp::resolver::query query("127.0.0.1", "17000");
    
    tcp::resolver::iterator endpoint = resolver.resolve(query);
    tcp::socket socket(io);

    socket.connect(*endpoint, err); //connect and wait
    
    /*if(argc > 1){
        std::string msg_out = argv[1];
        msg_out.assign(msg_out.c_str(), msg_out.size());

        write(socket, buffer(msg_out));
    }*/
   
    for(;;){

        if(std::getline(std::cin, msg_out)){        
            msg_out.assign(msg_out.c_str(), msg_out.size());
            write(socket, buffer(msg_out));
            msg_out.clear();    
        } 

        boost::array<char, 128> buf;
        len = socket.read_some(buffer(buf), err);

        if(err == error::eof)
            break;
        //else if(err)
        //    std::cout << "Unknown Error";
        
        std::cout << "Echo: ";
        std::cout.write(buf.data(), len);
        std::cout << "\n";
            
    }
}