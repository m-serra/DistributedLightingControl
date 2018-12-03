/*
g++ -std=c++11 -lboost_system async_tcp_server.cpp -o async_tcp_server
*/

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio.hpp>

using namespace boost;
using namespace boost::asio;
using asio::ip::tcp;


class Connection: public enable_shared_from_this<Connection>{

    tcp::socket socket_;
    tcp::acceptor acceptor_;
    std::string msg_out;
    std::string msg_in;

    void handle_write(){
        // do somehting to preserve connection or close it cleanly
    }

    void handle_read(){
        // do somehting to preserve connection or close it cleanly
        async_write(socket_,
                    buffer("Hello World!!!\n"),
                    boost::bind(&Connection::handle_write, shared_from_this())
                    ); 
    }

    public:

        Connection(tcp::acceptor &acceptor){
            acceptor_(acceptor);
            socket_(acceptor.get_io_service(), tcp::v4());
        }

        static shared_ptr<Connection> create(tcp::acceptor &acceptor){
            return shared_ptr<Connection>(new Connection(acceptor));
        }

        // getter for the field socket_()
        tcp::socket& socket(){
            return socket_;
        }

        void start(){

            //ask the io_service to execute the given handler
            async_read(socket_, 
                       boost::asio::buffer(msg_in),
                       boost::bind(&Connection::handle_read,
                       shared_from_this(),
                       boost::asio::placeholders::error,
                       boost::asio::placeholders::bytes_transferred));  
        }

    /*private:

        void start_accept(){
            acceptor_.async_accept(socket_, 
                boost::bind(&Connection::handle_accept, shared_from_this(),
                    boost::asio::placeholders::error));

        }

        void handle_accept(const boost::system::error_code& err){
       
                       
        }*/
};

class tcp_server{

    private:
        tcp::acceptor acceptor;

    public:
        tcp_server(io_service& io):
            //initialise an acceptor to listen on TCP port 17000
            acceptor(io, tcp::endpoint(tcp::v4(), 17000))
        {
            start_accept();
        }

    private:
        
        void start_accept(){
            // create a socket and initiate an asynchronous accept operation 
            // to wait for a new connection    
            shared_ptr<Connection> new_conn = Connection::create(acceptor);
            
            acceptor.async_accept(new_conn->socket(),
                                  boost::bind(&tcp_server::handle_accept,
                                              this,
                                              new_conn)
                                  );

        }

        void handle_accept(shared_ptr<Connection> new_conn){
            new_conn->start();
            start_accept();
        }
};

int main(){
	printf("started\n");
    io_service io;
    tcp_server server(io);
    io.run();
    return 0;
}
