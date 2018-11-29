#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio.hpp>

using namespace boost;
using namespace boost::asio;
using asio::ip::tcp;

// SEE: https://www.boost.org/doc/libs/1_58_0/doc/html/boost_asio/tutorial/tutdaytime3.html

class conn: public enable_shared_from_this<conn>{

    tcp::socket sock;
    std::string msg_out;
    std::string msg_in;

    conn(io_service& io): sock(io) {}

    void h_write(){
        // do somehting to preserve connection or close it cleanly
    }

    public:
        static shared_ptr<conn> create(io_service& io){
            return shared_ptr<conn>(new conn(io));
        }

        tcp::socket& socket(){
            return sock;
        }

        void start(){
            async_write(sock,
                        buffer(data),
                        boost::bind(&conn::h_write, shared_from_this())
                        );

            async_read(s, boost::asio::buffer(data, size), handler);     
        }
};

class tcp_server{

    private:
        tcp::acceptor acceptor;

    public:
        tcp_server(io_service& io)
        :acceptor(io, tcp::endpoint(tcp::v4(), 10000)){
            start_accept();
        }

    private:
        void start_accept(){
            shared_ptr<conn> new_conn = conn::create(acceptor.get_io_service());
            acceptor.async_accept(new_conn->socket(),
                                  boost::bind(&tcp_server::h_accept,
                                              this,
                                              new_conn)
                                  );
        }

        void h_accept(shared_ptr<conn> new_conn){
            new_conn->start();
            start_accept();
        }
};

int main(){
    io_service io;
    tcp_server server(io);
    io.run();
    return 0;
}
