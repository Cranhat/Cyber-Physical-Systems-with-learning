#pragma once

#include <string>

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_mac.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_event.h"
#include "esp_system.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "nvs_flash.h"
#include "ping/ping_sock.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

/*
TCP sender, receiver
*/

class DataExchanger
{
public:
    DataExchanger(const std::string ip, const uint16_t port, const std::string ssid, const std::string password);
    ~DataExchanger();

    static void wifiEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
    void connectToWifi();
    int createSocketAndConnect();
    void startTcpClient();
    void appendToSending(std::function<int()> method, uint8_t *bufferPointer);
    void appendToReceiving(std::function<void()> method, uint8_t *bufferPointer, std::size_t bufferSize);
    void sendData(std::function<int()> method, uint8_t *bufferPointer);
    bool receiveData();

private:
    struct ReceivingEntry
    {
        std::function<void()> method;
        uint8_t *bufferPointer;
        std::size_t bufferSize;
        std::size_t bytesReceived;
    };

    std::vector<std::pair<std::function<int()>, uint8_t *>> sendingVector;
    std::vector<ReceivingEntry> receivingVector;
    const std::string ip;
    const uint16_t port;
    const std::string ssid;
    const std::string password;

    int sock;
};
