#include "SocketObject.hpp"
#include <iostream>
#include <cmath>



#pragma mark - AbstractSocketObject

AbstractSocketObject::AbstractSocketObject(uint16_t port, std::string interfaceIP) : AbstractSocketObject()
{
    this->interfaceIP = interfaceIP;
    this->port = port;
    boost::asio::ip::address address = boost::asio::ip::address::from_string(interfaceIP);
    endpoint = boost::asio::ip::udp::endpoint(address, port);
}

AbstractSocketObject::~AbstractSocketObject()
{
    disconnect();
}

void AbstractSocketObject::connect()
{
    this->isConnected = true;
}

void AbstractSocketObject::disconnect()
{
    this->isConnected = false;
}

void AbstractSocketObject::reconnect()
{
    disconnect();
    connect();
}

void AbstractSocketObject::setInterface(std::string interfaceIP)
{
    this->interfaceIP = interfaceIP;
    reconnect();
}



#pragma mark - SocketSender

SocketSender::SocketSender(uint16_t port, std::string interface, bool multicast) : AbstractSocketObject(port, interface)
{
    this->isMulticast = multicast;
    connect();
}

void SocketSender::connect()
{
    AbstractSocketObject::connect();
    if( socket )
    {
        delete socket;
    }
    socket = new boost::asio::ip::udp::socket(io_service, endpoint.protocol());
}

void SocketSender::disconnect()
{
    io_service.stop();
    if( socket )
    {
        socket->close();
        delete socket;
        socket = nullptr;
    }
    AbstractSocketObject::disconnect();
}

bool SocketSender::sendData(void *data, size_t length)
{
    if( data && length > 0 )
    {
        socket->async_send_to(
                              boost::asio::buffer(data, length), endpoint,
                              boost::bind(&SocketSender::handle_send_to, this,
                                          boost::asio::placeholders::error));
        try
        {
            io_service.run();
            return true;
        }
        catch (std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << "\n";
        }
    }
    return false;
}

void SocketSender::handle_send_to(const boost::system::error_code& error)
{
    
}

void SocketSender::setIsMulticast(bool isMulticast)
{
    this->isMulticast = isMulticast;
    reconnect();
}



#pragma mark - SocketListener

SocketListener::SocketListener(size_t packetSize, uint16_t port, std::string interface) : AbstractSocketObject(port, interface)
{
    isRunning = false;
    this->packetSize = packetSize;
    data = new uint8_t[this->packetSize];
}

SocketListener::~SocketListener()
{
    stop();
    delete []data;
}

void SocketListener::connect()
{
    AbstractSocketObject::connect();
    
    if( socket )
    {
        delete socket;
    }
    
    socket = new boost::asio::ip::udp::socket(io_service);
    boost::asio::ip::address listen_address = boost::asio::ip::address::from_string("0.0.0.0");
    boost::asio::ip::udp::endpoint listen_endpoint(listen_address, port);
    socket->open(listen_endpoint.protocol());
    socket->set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket->bind(listen_endpoint);
    
    // Join the multicast group.
    socket->set_option(boost::asio::ip::multicast::join_group(endpoint.address()));
    
    socket->async_receive_from(boost::asio::buffer(data, packetSize), sender_endpoint,
                               boost::bind(&SocketListener::handle_receive_from, this,
                                           boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
}

void SocketListener::handle_receive_from(const boost::system::error_code& error, size_t bytes_recvd)
{
    if (!error)
    {
        // handle packet
        if( callback )
        {
            callback(data, packetSize, sender_endpoint.address().to_string());
        }
        socket->async_receive_from(boost::asio::buffer(data, packetSize), sender_endpoint,
                                   boost::bind(&SocketListener::handle_receive_from, this,
                                               boost::asio::placeholders::error,
                                               boost::asio::placeholders::bytes_transferred));
    }
}

void SocketListener::start(SocketListenerCallback callback)
{
    if( !getIsConnected() )
    {
        connect();
    }
    
    if( getIsConnected() )
    {
        this->callback = callback;
        listenThread = std::thread([this](){
            fprintf(stderr, "waiting for packets ...\n");
            isRunning = true;
            try
            {
                io_service.run();
            }
            catch (std::exception& e)
            {
                std::cerr << "Exception: " << e.what() << "\n";
            }
            isRunning = false;
            disconnect();
        });
    }
    else
    {
        std::cout<<"SocketListener::start : try to start but isn't connected"<<std::endl;
    }
}

void SocketListener::stop()
{
    io_service.stop();
    if( listenThread.joinable() )
    {
        try
        {
            listenThread.join();
        }
        catch (std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << "\n";
        }
        
    }
    isRunning = false;
}

void SocketListener::disconnect()
{
    if( socket )
    {
        socket->close();
        delete socket;
        socket = nullptr;
    }
    AbstractSocketObject::disconnect();
}

void SocketListener::reconnect()
{
    bool shouldRestart = getIsRunning();
    AbstractSocketObject::reconnect();
    if( shouldRestart )
    {
        start(this->callback);
    }
}
