#ifndef SocketObject_hpp
#define SocketObject_hpp

#include <stdio.h>
#include <string>
#include <set>
#include <functional>

#include <iostream>
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#if __APPLE__ ||  __linux__
#include <unistd.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#elif _WIN32
#include <ws2tcpip.h>
#include <winsock2.h>
#include <iphlpapi.h>
#include <stdlib.h>
#endif



class AbstractSocketObject
{
protected:
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket *socket = nullptr;
    boost::asio::ip::udp::endpoint endpoint;
    std::string localIP = "";
    std::string interfaceIP = "";
    uint16_t port = 0;
    bool isConnected = false;
    
    virtual void connect();
    virtual void disconnect();
    virtual void reconnect();
    
public:
    AbstractSocketObject() {}
    AbstractSocketObject(uint16_t port, std::string interfaceIP);
    virtual ~AbstractSocketObject();
    
    std::string getLocalIP() { return localIP; }
    std::string getInterface() { return this->interfaceIP; }
    virtual void setInterface(std::string interfaceIP);
    bool getIsConnected() { return this->isConnected; }
    
};



class SocketSender : public AbstractSocketObject
{
protected:
    bool isMulticast;
    
    virtual void connect() override;
    virtual void disconnect() override;
    void handle_send_to(const boost::system::error_code& error);
    
public:
    SocketSender(uint16_t port, std::string interfaceIP, bool multicast = true);
    
    bool sendData(void *data, size_t length);
    
    bool getIsMulticast() { return this->isMulticast; }
    void setIsMulticast(bool isMulticast);
};



typedef std::function<void(void *data, size_t size, std::string ip)> SocketListenerCallback;

class SocketListener : public AbstractSocketObject
{
protected:
    bool isRunning;
    std::thread listenThread;
    SocketListenerCallback callback;
    uint8_t *data;
    size_t packetSize;
    boost::asio::ip::udp::endpoint sender_endpoint;
    
    virtual void connect() override;
    virtual void disconnect() override;
    virtual void reconnect() override;
    void handle_receive_from(const boost::system::error_code& error, size_t bytes_recvd);
    
public:
    SocketListener(size_t packetSize, uint16_t port, std::string interfaceIP);
    ~SocketListener();
    
    void start(SocketListenerCallback callback = NULL);
    void stop();
    
    bool getIsRunning() { return isRunning; }
};

#endif /* SocketObject_hpp */
