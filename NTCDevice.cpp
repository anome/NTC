#include "NTCDevice.hpp"
#include <iostream>
#include <string.h>
#include <cmath>
#include <chrono>
extern "C"
{
#if _WIN32
#define NOMINMAX
#   include <windows.h>
#else
#   include <unistd.h>
#endif
}

#ifdef _WIN32
#define cross_platform_sleep(ms) Sleep(ms)
#else
#define cross_platform_sleep(ms) usleep(ms*1000)
#endif


#define MAX_UINT16 65535
#define MAX_UINT32 4294967295
#define MAX_UINT64 18446744073709551615



/* NTC Validation Errors Type */
typedef enum {
    NTC_ERR_NONE,
    NTC_ERR_NULLPTR,
    NTC_ERR_PID,
    NTC_ERR_SOURCE_NAME,
} ntc_error_t;



const uint8_t _NTC_PID[] = {'N', 'T', 'C', 'v', '0', '.', '1'};



int ntc_pkt_init(ntc_packet_t *packet, const std::string &source_name)
{
    if (packet == NULL) {
        errno = EINVAL;
        return -1;
    }
    
    // clear packet
    memset(packet, 0, sizeof *packet);
    memcpy(packet->pid, _NTC_PID, sizeof packet->pid);
    if( source_name.size()  > 0 )
    {
        memcpy(packet->source_name, source_name.c_str(), source_name.size());
    }
    packet->source_name_length = source_name.size();
    return 0;
}

ntc_error_t ntc_pkt_validate(const ntc_packet_t *packet) {
    if (packet == NULL)
        return NTC_ERR_NULLPTR;
    if (memcmp(packet->pid, _NTC_PID, sizeof packet->pid) != 0)
        return NTC_ERR_PID;
    return NTC_ERR_NONE;
}

const char *ntc_strerror(const ntc_error_t error) {
    switch (error) {
        case NTC_ERR_NONE:
            return "Success";
        case NTC_ERR_PID:
            return "Invalid ACN Packet Identifier";
        case NTC_ERR_SOURCE_NAME:
            return "Wrong Source Name";
        default:
            return "Unknown error";
    }
}



#pragma mark - NTCDeviceAbstract
// Constructor

NTCDeviceAbstract::NTCDeviceAbstract(std::string sourceName, std::string interfaceIP) : sourceName(sourceName), interfaceIP(interfaceIP)
{
}

NTCDeviceAbstract::~NTCDeviceAbstract()
{
    if( listener )
    {
        listener->stop();
        delete listener;
        listener = nullptr;
    }
}


// Utility

void NTCDeviceAbstract::updatePacket(ntc_packet_t &packet, double timecode, uint8_t order, bool updateIndex, double seekTime)
{
    unsigned long timeToSend = floor(timecode*1000);
    packet.ntc.milli = timeToSend % 1000;
    timeToSend /= 1000;
    packet.ntc.hour = timeToSend / 3600;
    timeToSend = timeToSend % 3600;
    packet.ntc.min = timeToSend / 60;
    timeToSend = timeToSend % 60;
    packet.ntc.sec = timeToSend;
    packet.ntc.order = order;
    if( order == NTC_ORDER_SEEK && seekTime >= 0 )
    {
        unsigned long seekToSend = floor(seekTime*1000);
        packet.ntc.seek_milli = seekToSend % 1000;
        seekToSend /= 1000;
        packet.ntc.seek_hour = seekToSend / 3600;
        seekToSend = seekToSend % 3600;
        packet.ntc.seek_min = seekToSend / 60;
        seekToSend = seekToSend % 60;
        packet.ntc.seek_sec = seekToSend;
    }
    
    if( updateIndex )
    {
        if( packet.ntc.index == MAX_UINT32 )
        {
            packet.ntc.index = 0;
        }
        else
        {
            packet.ntc.index++;
        }
    }
}

bool NTCDeviceAbstract::sendPacket(ntc_packet_t &packet, SocketSender *senderSocket)
{
    if( !senderSocket || !senderSocket->getIsConnected() )
    {
        return false;
    }
    
    std::string currentIp = listener->getInterface();
    memset(packet.ip, 0, 16);
    memcpy(packet.ip, currentIp.c_str(), currentIp.size());
    packet.ip_length = currentIp.size();
    
    memset(packet.source_name, 0, 64);
    memcpy(packet.source_name, sourceName.c_str(), sourceName.size());
    packet.source_name_length = sourceName.size();
    
    if( !senderSocket->sendData(&packet, sizeof(packet)) )
    {
        perror("error : sendData");
        return false;
    }
    return true;
}

double NTCDeviceAbstract::timeInSecForPacket(const ntc_packet_t &packet)
{
    NTCData ntc = packet.ntc;
    return (double)ntc.milli / 1000.f + ntc.sec + ntc.min * 60.f + ntc.hour * 3600;
}

double NTCDeviceAbstract::seekTimeInSecForPacket(const ntc_packet_t &packet)
{
    NTCData ntc = packet.ntc;
    return (double)ntc.seek_milli / 1000.f + ntc.seek_sec + ntc.seek_min * 60.f + ntc.seek_hour * 3600;
}


// Getter & Setter

bool NTCDeviceAbstract::getIsConnected()
{
    return listener && listener->getIsConnected();
}

std::string NTCDeviceAbstract::getSourceName()
{
    return sourceName;
}

void NTCDeviceAbstract::setSourceName(std::string sourceName)
{
    this->sourceName = sourceName;
}

double NTCDeviceAbstract::getInternalTime()
{
    double elapsedTime = (NTCTimeDurationMilli(NTCTimeResolution::now().time_since_epoch()).count() / 1000.f) - internalTimePoint;
    return internalTime + elapsedTime;
}

void NTCDeviceAbstract::setInternalTime(double time, double settingTimePoint)
{
    internalTime = time;
    internalTimePoint = settingTimePoint;
}

double NTCDeviceAbstract::timeSinceLastUpdate()
{
    NTCTimePoint currentTime = NTCTimeResolution::now();
    return (NTCTimeDurationMilli(currentTime.time_since_epoch()).count()/1000.) - internalTimePoint;
}

double NTCDeviceAbstract::getCurrentTime()
{
    if( isStopping() )
    {
        return 0;
    }
    else
    {
        if( isPlaying() )
        {
            nowTimePoint = getInternalTime();
        }
        else if( isPausing() )
        {
            nowTimePoint = pauseTime;
        }
        double durationSinceLastPoint = nowTimePoint-currentTimePoint;
        return currentTime + durationSinceLastPoint;
    }
    return 0;
}

void NTCDeviceAbstract::setCurrentTime(double time)
{
    setCurrentTimeAtPoint(time, getInternalTime());
}

void NTCDeviceAbstract::setCurrentTimeAtPoint(double time, double settingTimePoint)
{
    currentTimePoint = settingTimePoint;
    currentTime = time;
    nowTimePoint = getInternalTime();
}

bool NTCDeviceAbstract::isPlaying()
{
    double internalTime = getInternalTime();
    if( playTime >= 0  && internalTime >= playTime)
    {
        if( pauseTime >= 0 && pauseTime > playTime && internalTime >= pauseTime )
        {
            return false;
        }
        else if( stopTime >= 0 && stopTime > playTime && internalTime >= stopTime )
        {
            return false;
        }
        return true;
    }
    return false;
}

bool NTCDeviceAbstract::isPausing()
{
    double internalTime = getInternalTime();
    if( pauseTime >= 0  && internalTime >= pauseTime)
    {
        if( playTime >= 0 && playTime >= pauseTime && internalTime >= playTime )
        {
            return false;
        }
        else if( stopTime >= 0 && stopTime > pauseTime && internalTime >= stopTime )
        {
            return false;
        }
        return true;
    }
    return false;
}

bool NTCDeviceAbstract::isStopping()
{
    double internalTime = getInternalTime();
    if( stopTime >= 0  && internalTime >= stopTime)
    {
        if( playTime >= 0 && playTime >= stopTime && internalTime >= playTime )
        {
            return false;
        }
        else if( pauseTime >= 0 && pauseTime >= stopTime && internalTime >= pauseTime )
        {
            return false;
        }
        return true;
    }
    return false;
}



#pragma mark - NTCMaster

// Constructor

NTCMaster::NTCMaster(std::string sourceName, std::string interfaceIP) : NTCDeviceAbstract(sourceName, interfaceIP)
{
    internalTimePoint = NTCTimeDurationMilli(NTCTimeResolution::now().time_since_epoch()).count() / 1000.f;
    syncStepDuration = NTC_ASK_DURATION*1000.f / (double)NTC_ASK_COUNT;
    stop();
    listener = new SocketListener(sizeof(ntc_packet_t), NTC_ASK_PORT, this->interfaceIP);
    listener->start([this](void *data, size_t size, std::string ip) {
        ntc_error_t error;
        if( size != sizeof(ntc_packet_t) )
        {
            perror("askCallback : tried to decode non ntc_packet_t");
            return;
        }
        ntc_packet_t *packet = (ntc_packet_t*)data;
        
        // validate packet
        if( (error = ntc_pkt_validate(packet)) != NTC_ERR_NONE )
        {
            fprintf(stderr, "ntc_pkt_validate: %s\n", ntc_strerror(error));
            return;
        }
        
        if( packet->ntc.order == NTC_ORDER_SYNC )
        {
            askForTimecode(packet, ip);
        }
    });
}

NTCMaster::~NTCMaster()
{
    if( listener )
    {
        listener->stop();
        delete listener;
        listener = nullptr;
    }
    
    try
    {
        std::map<std::string,SlaveData*>::iterator it;
        for(it=slaveDataMap.begin(); it!=slaveDataMap.end();)
        {
            SlaveData *data = it->second;
            delete data;
            it = slaveDataMap.erase(it);
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }
}


// Timecode

void NTCMaster::askForTimecode(ntc_packet_t *packet, std::string fromIP)
{
    SlaveData *data = nullptr;
    if( slaveDataMap.find(fromIP) == slaveDataMap.end() )
    {
        data = new SlaveData();
        data->sender = new SocketSender(NTC_UPDATE_PORT, fromIP, false);
        ntc_pkt_init(&(data->currentPacket), sourceName);
        slaveDataMap[fromIP] = data;
    }
    else
    {
        data = slaveDataMap[fromIP];
    }
    
    data->syncPointMutex.lock();
    SocketSender *selectedSender = data->sender;
    if( !data->isSync && data->delayPoints.size() == 0 )
    {
        data->isSync = true;
        data->syncThread = std::thread([this, data, fromIP, selectedSender](){
            data->currentCount = NTC_ASK_COUNT;
            data->numberOfUpdatesPacket = 0;
            for(int i=0; i<NTC_ASK_COUNT; ++i)
            {
                NTCTimePoint startIterTime = NTCTimeResolution::now();
                updatePacket(data->currentPacket, getInternalTime(), NTC_ORDER_SYNC);
                ntc_packet_t currentPacket = data->currentPacket;
                sendPacket(currentPacket, selectedSender);
                data->delayPoints.push_back({startIterTime, NTCTimePoint()});
                data->packets.push_back(currentPacket);
                double elapsedTime = NTCTimeDurationMilli(NTCTimeResolution::now()-startIterTime).count();
                double sleepTime = syncStepDuration - elapsedTime;
                if( sleepTime > 0 )
                {
                    cross_platform_sleep(sleepTime);
                }
            }
            data->isSync = false;
        });
    }
    else
    {
        NTCTimePoint receivePoint = NTCTimeResolution::now();
        bool findPacket = false;
        for(int i=data->packets.size()-1; i>=0; --i)
        {
            ntc_packet_t currentPacket = data->packets.at(i);
            if( packet->ntc.index == currentPacket.ntc.index )
            {
                data->delayPoints.at(i).receivePoint = receivePoint;
                data->delayPoints.at(i).valid = true;
                ++data->numberOfUpdatesPacket;
                findPacket = true;
                break;
            }
        }
        
        if( !findPacket )
        {
            --data->currentCount;
            std::cout<<"missing index "<<packet->ntc.index<<std::endl;
        }
        
        if( data->numberOfUpdatesPacket >= data->currentCount )
        {
            double delay = 0;
            int numberOfValidPoint = 0;
            for(DelayPoint syncPoint : data->delayPoints)
            {
                if( syncPoint.valid )
                {
                    NTCTimePoint sendPoint = syncPoint.sendPoint;
                    NTCTimePoint receivePoint = syncPoint.receivePoint;
                    delay += NTCTimeDurationMilli(receivePoint-sendPoint).count() / 1000.;
                    ++numberOfValidPoint;
                }
            }
            data->delay = numberOfValidPoint > 0 ? delay / numberOfValidPoint : 0;
            if( isPlaying() )
            {
                sendPlayForData(data);
            }
            else if( isPausing() )
            {
                sendPauseForData(data);
            }
            else
            {
                sendStopForData(data);
            }
            
            sendSeekForData(data);
            
            data->delayPoints.clear();
            data->packets.clear();
            
            if( data->syncThread.joinable() )
                data->syncThread.join();
        }
    }
    data->syncPointMutex.unlock();
}

void NTCMaster::sendSeekForData(SlaveData *data)
{
    double currentTime = getCurrentTime();
    double timePoint;
    if( isPausing() )
    {
        timePoint = pauseTime;
    }
    else
    {
        timePoint = std::max(0., getInternalTime() - data->delay);
    }
    updatePacket(data->currentPacket, timePoint, NTC_ORDER_SEEK, true, currentTime);
    sendPacket(data->currentPacket, data->sender);
}

void NTCMaster::sendPlayForData(SlaveData *data)
{
    double playTimeToSend = std::max(0., playTime - data->delay);
    updatePacket(data->currentPacket, playTimeToSend, NTC_ORDER_PLAY);
    sendPacket(data->currentPacket, data->sender);
}

void NTCMaster::sendPauseForData(SlaveData *data)
{
    double pauseTimeToSend = std::max(0., pauseTime - data->delay);
    updatePacket(data->currentPacket, pauseTimeToSend, NTC_ORDER_PAUSE);
    sendPacket(data->currentPacket, data->sender);
}

void NTCMaster::sendStopForData(SlaveData *data)
{
    double stopTimeToSend = std::max(0., stopTime - data->delay);
    updatePacket(data->currentPacket, stopTimeToSend, NTC_ORDER_STOP);
    sendPacket(data->currentPacket, data->sender);
}

void NTCMaster::sendSeek()
{
    for(std::map<std::string,SlaveData*>::iterator it = slaveDataMap.begin(); it != slaveDataMap.end(); ++it)
    {
        sendSeekForData(it->second);
    }
}

void NTCMaster::sendPlay()
{
    for(std::map<std::string,SlaveData*>::iterator it = slaveDataMap.begin(); it != slaveDataMap.end(); ++it)
    {
        sendPlayForData(it->second);
    }
}

void NTCMaster::sendPause()
{
    for(std::map<std::string,SlaveData*>::iterator it = slaveDataMap.begin(); it != slaveDataMap.end(); ++it)
    {
        sendPauseForData(it->second);
    }
}

void NTCMaster::sendStop()
{
    for(std::map<std::string,SlaveData*>::iterator it = slaveDataMap.begin(); it != slaveDataMap.end(); ++it)
    {
        sendStopForData(it->second);
    }
}

void NTCMaster::play()
{
    double longestDelay = 0;
    for(std::map<std::string,SlaveData*>::iterator it = slaveDataMap.begin(); it != slaveDataMap.end(); ++it)
    {
        longestDelay = std::max(longestDelay, it->second->delay);
    }
    longestDelay *= 2.;
    
    double currentTime = getInternalTime();
    double timeToPlay = currentTime + longestDelay;
    double value = getCurrentTime();
    NTCDeviceAbstract::setCurrentTimeAtPoint(value, timeToPlay);
    playTime = timeToPlay;
    sendPlay();
}

void NTCMaster::pause()
{
    double longestDelay = 0;
    for(std::map<std::string,SlaveData*>::iterator it = slaveDataMap.begin(); it != slaveDataMap.end(); ++it)
    {
        longestDelay = std::max(longestDelay, it->second->delay);
    }
    longestDelay *= 2.;
    
    double currentTime = getInternalTime();
    pauseTime = currentTime + longestDelay;
    sendPause();
    sendSeek();
}

void NTCMaster::stop()
{
    if( !isStopping() )
    {
        double longestDelay = 0;
        for(std::map<std::string,SlaveData*>::iterator it = slaveDataMap.begin(); it != slaveDataMap.end(); ++it)
        {
            longestDelay = std::max(longestDelay, it->second->delay);
        }
        longestDelay *= 2.;
        
        double currentTime = getInternalTime();
        stopTime = currentTime + longestDelay;
    }
    sendStop();
}

void NTCMaster::setCurrentTimeAtPoint(double time, double settingTimePoint)
{
    if( isPausing() )
    {
        settingTimePoint = pauseTime;
    }
    NTCDeviceAbstract::setCurrentTimeAtPoint(time, settingTimePoint);
    sendSeek();
}



#pragma mark - NTCSlave

// Constructor

NTCSlave::NTCSlave(std::string sourceName, std::string interface, std::string masterIp) : NTCDeviceAbstract(sourceName, interface)
{
    sources.insert("Any Sources");
    listener = new SocketListener(sizeof(ntc_packet_t), NTC_UPDATE_PORT, this->interfaceIP);
    if( masterIp.size() )
    {
        multicastSender = new SocketSender(NTC_ASK_PORT, masterIp, false);
    }
    else
    {
        multicastSender = new SocketSender(NTC_ASK_PORT, this->interfaceIP);
    }
}

NTCSlave::~NTCSlave()
{
    stop();
    if( multicastSender )
    {
        delete multicastSender;
        multicastSender = nullptr;
    }
    if( unicastSender )
    {
        delete unicastSender;
        unicastSender = nullptr;
    }
}


// Handle

void NTCSlave::start(NTCSlaveDataCallback callback)
{
    dataCallback = callback;
    listener->start([this](void *data, size_t size, std::string ip) { decodeData(data, size, ip); });
    
    syncThreadMutex.lock();
    isRunning = true;
    synchronizingThread = std::thread([this](){
        syncDataMutex.lock();
        packetTimeIndexes.clear();
        syncPoints.clear();
        syncDataMutex.unlock();
        double duration = NTC_ASK_FREQ;
        NTCTimePoint lastCheck;
        ntc_packet_t packet;
        ntc_pkt_init(&packet, sourceName);
        do {
            if( duration >= NTC_ASK_FREQ )
            {
                lastCheck = NTCTimeResolution::now();
                isSynchronizing = true;
                updatePacket(packet, 0, NTC_ORDER_SYNC, false);
                sendPacket(packet, multicastSender);
                startTimePoint = NTCTimeResolution::now();
            }
            cross_platform_sleep(1);
            duration = NTCTimeDurationMilli(NTCTimeResolution::now()-lastCheck).count() / 1000.f;
        } while( isRunning );
    });
    syncThreadMutex.unlock();
}

void NTCSlave::stop()
{
    syncThreadMutex.lock();
    isRunning = false;
    if( synchronizingThread.joinable() )
    {
        try
        {
            synchronizingThread.join();
        }
        catch (std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << "\n";
        }
    }
    syncThreadMutex.unlock();
}

bool NTCSlave::getIsConnected()
{
    return NTCDeviceAbstract::getIsConnected() && multicastSender && multicastSender->getIsConnected();
}


// Listen

void NTCSlave::decodeData(void *data, size_t size, std::string fromString)
{
    ntc_error_t error;
    if( size != sizeof(ntc_packet_t) )
    {
        perror("decodeData : tried to decode non ntc_packet_t");
        return;
    }
    ntc_packet_t *receivedPacket = (ntc_packet_t*)data;
    
    // validate packet
    if( (error = ntc_pkt_validate(receivedPacket)) != NTC_ERR_NONE )
    {
        fprintf(stderr, "ntc_pkt_validate: %s\n", ntc_strerror(error));
        return;
    }
    
    std::string source((const char*)receivedPacket->source_name, receivedPacket->source_name_length);
    sources.insert(source);
    
    // check source name
    std::string sourceName = getSourceName();
    if( sourceName.size() && sourceName.compare("Any Sources") != 0 )
    {
        if( memcmp(receivedPacket->source_name, sourceName.c_str(), sourceName.size()) != 0 )
        {
            std::cerr<<"NTCSocketListener: skip packet from wrong source"<<std::endl;
            return;
        }
    }
    
    // get times
    NTCTimePoint currentPacketTime = NTCTimeResolution::now();
    double receivedTime = timeInSecForPacket(*receivedPacket);
    
    // check packet order
    uint8_t order = receivedPacket->ntc.order;
    if( order == NTC_ORDER_SYNC )
    {
        if( !unicastSender )
        {
            unicastSender = new SocketSender(NTC_ASK_PORT, fromString, false);
        }
        
        if( unicastSender->getInterface().compare(fromString) != 0 )
        {
            unicastSender->setInterface(fromString);
        }
        
        syncDataMutex.lock();
        syncPoints.push_back({currentPacketTime, receivedTime});
        ntc_packet_t packet = *receivedPacket;
        updatePacket(packet, receivedTime, NTC_ORDER_SYNC, false);
        sendPacket(packet, unicastSender);
        if( syncPoints.size() >= NTC_ASK_COUNT )
        {
            int nbOfPoints = syncPoints.size();
            double delay = 0;
            double averagePoint = 0;
            double averageTime = 0;
            for(int i=0; i<nbOfPoints; ++i)
            {
                SyncPoint point = syncPoints[i];
                double elapsetime = (NTCTimeDurationMilli(point.point-startTimePoint).count()/1000.) - (NTC_ASK_DURATION * ((double)i/(double)nbOfPoints));
                delay += elapsetime;
                averageTime += point.time;
                averagePoint += (NTCTimeDurationMilli(point.point.time_since_epoch()).count()/1000.);
            }
            
            currentDelay = delay / nbOfPoints;
            referencePoint = averagePoint / nbOfPoints;
            referenceTime = averageTime / nbOfPoints;
            setInternalTime(referenceTime, referencePoint);
            syncPoints.clear();
            if( dataCallback )
            {
                dataCallback(&(receivedPacket->ntc));
            }
        }
        syncDataMutex.unlock();
    }
    else if( order == NTC_ORDER_PLAY )
    {
        if( !(fabs(receivedTime-(playTime)) < 0.016) )
        {
            setCurrentTimeAtPoint(getCurrentTime(), receivedTime);
            playTime = receivedTime;
        }
        if( dataCallback )
        {
            dataCallback(&(receivedPacket->ntc));
        }
    }
    else if( order == NTC_ORDER_PAUSE )
    {
        pauseTime = receivedTime;
        if( dataCallback )
        {
            dataCallback(&(receivedPacket->ntc));
        }
    }
    else if( order == NTC_ORDER_STOP )
    {
        stopTime = receivedTime;
        if( dataCallback )
        {
            dataCallback(&(receivedPacket->ntc));
        }
    }
    else if( order == NTC_ORDER_SEEK )
    {
        double seekTime = seekTimeInSecForPacket(*receivedPacket);
        setCurrentTimeAtPoint(seekTime, receivedTime);
        if( dataCallback )
        {
            dataCallback(&(receivedPacket->ntc));
        }
    }
}


std::string NTCSlave::getInterface()
{
    return listener->getInterface();
}

void NTCSlave::setInterface(std::string interface)
{
    listener->setInterface(interface);
}


std::string NTCSlave::getMasterIP()
{
    if( unicastSender )
    {
        return unicastSender->getInterface();
    }
    
    return nullptr;
}

void NTCSlave::setMasterIP(std::string masterIP)
{
    multicastSender->setInterface(masterIP);
}
