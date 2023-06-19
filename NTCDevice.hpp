#ifndef NTCHandler_hpp
#define NTCHandler_hpp

#include "Socket/SocketObject.hpp"
#include <chrono>
#include <map>
#include <mutex>
#include <functional>



/* NTC Public Constants */
#define NTC_UPDATE_PORT 1312
#define NTC_ASK_PORT 1313
#define NTC_DEFAULT_MULTICAST_ADDR "224.0.0.123"

#define NTC_ASK_DURATION 1.
#define NTC_ASK_COUNT 60
#define NTC_ASK_FREQ 10.

#define NTC_ORDER_SYNC 't'
#define NTC_ORDER_PAUSE 'p'
#define NTC_ORDER_STOP 's'
#define NTC_ORDER_PLAY 'f'
#define NTC_ORDER_SEEK 'k'



typedef std::chrono::high_resolution_clock NTCTimeResolution;
typedef NTCTimeResolution::time_point NTCTimePoint;
typedef std::chrono::duration<double, std::milli> NTCTimeDurationMilli;
typedef std::chrono::duration<double, std::micro> NTCTimeDurationMicro;



typedef struct {
    uint8_t order;
    uint32_t index;
    uint16_t hour;
    uint8_t min;
    uint8_t sec;
    uint16_t milli;
    
    uint16_t seek_hour;
    uint8_t seek_min;
    uint8_t seek_sec;
    uint16_t seek_milli;
} NTCData;



/* NTC Packet Type */
/* All packet contents shall be transmitted in network byte order (big endian) */
typedef union {

#if _WIN32
#pragma pack(push, 1)
    struct {
        uint8_t ip_length;
        uint8_t ip[16];
        uint8_t pid[7];
        uint8_t source_name_length;
        uint8_t source_name[64];
        NTCData ntc; /* Network Timecode Protocol (NTCP) Layer */
    };
#pragma pack(pop)
#else
    struct {
        uint8_t __attribute__((packed)) ip_length;
        uint8_t __attribute__((packed)) ip[16];
        uint8_t __attribute__((packed)) pid[7];
        uint8_t __attribute__((packed)) source_name_length;
        uint8_t __attribute__((packed)) source_name[64];
        NTCData __attribute__((packed)) ntc; /* Network Timecode Protocol (NTCP) Layer */
    } __attribute__((packed));
#endif
    
    uint8_t raw[148]; /* raw buffer view */
} ntc_packet_t;



class NTCDeviceAbstract
{
protected:
    SocketListener *listener = nullptr;
    std::string interfaceIP = NTC_DEFAULT_MULTICAST_ADDR;
    std::string sourceName;
    double internalTimePoint = 0;
    double internalTime = 0;
    double nowTimePoint = 0;
    double currentTimePoint = 0;
    double currentTime = 0;
    bool timeIsRunning = true;
    
    double playTime = -1;
    double pauseTime = -1;
    double stopTime = -1;
    
    virtual void updatePacket(ntc_packet_t &packet, double timecode, uint8_t order, bool updateIndex = true, double seekTime = -1);
    virtual bool sendPacket(ntc_packet_t &packet, SocketSender *senderSocket);
    void setInternalTime(double time, double settingTimePoint);
    
public:
    NTCDeviceAbstract(std::string sourceName, std::string interfaceIP);
    ~NTCDeviceAbstract();
    
    virtual bool getIsConnected();
    
    virtual std::string getSourceName();
    virtual void setSourceName(std::string sourceName);
    
    double getInternalTime();
    double getCurrentTime();
    void setCurrentTime(double time);
    virtual void setCurrentTimeAtPoint(double time, double settingTimePoint);
    double timeSinceLastUpdate();
    
    bool isPlaying();
    bool isPausing();
    bool isStopping();
    
    static double timeInSecForPacket(const ntc_packet_t &packet);
    static double seekTimeInSecForPacket(const ntc_packet_t &packet);
};



class NTCMaster : public NTCDeviceAbstract
{
protected:
    struct DelayPoint
    {
        NTCTimePoint sendPoint;
        NTCTimePoint receivePoint;
        bool valid = false;
        DelayPoint(NTCTimePoint sendPoint, NTCTimePoint receivePoint) : sendPoint(sendPoint), receivePoint(receivePoint) {}
    };
    struct SlaveData
    {
        SocketSender *sender = nullptr;
        std::thread syncThread;
        std::mutex syncPointMutex;
        std::vector<DelayPoint> delayPoints;
        std::vector<ntc_packet_t> packets;
        ntc_packet_t currentPacket;
        bool isSync = false;
        double delay = 0;
        int currentCount = 0;
        int numberOfUpdatesPacket = 0;
        
        ~SlaveData()
        {
            if( syncThread.joinable() )
            {
                syncThread.join();
            }
            if( sender )
            {
                delete sender;
                sender = nullptr;
            }
        }
    };
    std::map<std::string, SlaveData*> slaveDataMap;
    double syncStepDuration = 0;
    
    void askForTimecode(ntc_packet_t *packet, std::string fromIP);
    
    void sendSeek();
    void sendSeekForData(SlaveData *data);
    void sendPlay();
    void sendPlayForData(SlaveData *data);
    void sendPause();
    void sendPauseForData(SlaveData *data);
    void sendStop();
    void sendStopForData(SlaveData *data);
    
public:
    NTCMaster(std::string sourceName = "", std::string interfaceIP = NTC_DEFAULT_MULTICAST_ADDR);
    ~NTCMaster();
    
    void play();
    void pause();
    void stop();
    
    virtual void setCurrentTimeAtPoint(double time, double settingTimePoint) override;
};



typedef std::function<void(NTCData*)> NTCSlaveDataCallback;

class NTCSlave : public NTCDeviceAbstract
{
protected:
    SocketSender *multicastSender = nullptr;
    SocketSender *unicastSender = nullptr;
    std::thread synchronizingThread;
    std::mutex syncThreadMutex;
    std::mutex syncDataMutex;
    bool isRunning = false;
    
    NTCTimePoint lastPacketTime;
    std::map<uint32_t,NTCTimePoint> packetTimeIndexes;
    NTCSlaveDataCallback dataCallback;
    double currentDelay = 0;
    double referencePoint = 0;
    double referenceTime = 0;
    
    bool isSynchronizing = false;
    int numberOfSyncPacket = 0;
    NTCTimePoint startTimePoint;
    struct SyncPoint
    {
        NTCTimePoint point;
        double time;
    };
    std::vector<SyncPoint> syncPoints;
    
    std::set<std::string> sources;
    
    void decodeData(void *data, size_t size, std::string fromString);
    
public:
    NTCSlave(std::string sourceName = "", std::string interfaceIp = NTC_DEFAULT_MULTICAST_ADDR, std::string masterIP = "");
    ~NTCSlave();
    
    void start(NTCSlaveDataCallback callback = NULL);
    void stop();
    
    bool getIsConnected() override;
    bool getIsRunning() { return ((SocketListener*)listener)->getIsRunning(); }
    
    std::string getInterface();
    void setInterface(std::string interfaceIP);
    std::string getMasterIP();
    void setMasterIP(std::string masterIP);
    
    std::set<std::string> getSources() { return this->sources; }
};

#endif /* NTCHandler_hpp */
