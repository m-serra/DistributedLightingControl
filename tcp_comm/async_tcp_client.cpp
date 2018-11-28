#include <boost/shared_ptr>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp> 
#include <boost/asio.hpp>

using namespace boost;
using namespace boost::asio;
using asio::ip::tcp;

class conn : public enable_shared_from_this<conn> { 
    
    tcp::socket sock_;
    std::string msg_;
    
    conn(io_service& io) : sock_(io) {}
    
    void h_write() { //do something to preserve //connection or close it cleanly
    
    }
    
    public:
        static shared_ptr<conn> create(io_service& io) {
            return shared_ptr<conn>(new conn(io)); 
    }

    tcp::socket& socket() {
        return sock_;
    } 

    void start() {
        async_write(sock_,buffer(“Hello World”), 
            boost::bind(&conn::h_write, shared_from_this()));
    } 
};