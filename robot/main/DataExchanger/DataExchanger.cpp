#include "DataExchanger.hpp"
#include "Uart/Uart.hpp"

#include <cerrno>

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_CONNECT_TIMEOUT_MS 15000

static const char *TAG = "DataExchanger";
static EventGroupHandle_t wifi_event_group;
static StaticEventGroup_t wifi_event_group_buffer;
static esp_netif_t *sta_netif = nullptr;
static bool wifi_handlers_registered = false;
static bool wifi_driver_initialized = false;

DataExchanger::DataExchanger(const std::string ip, const uint16_t port, const std::string ssid, const std::string password)
    : ip(ip), port(port), ssid(ssid), password(password)
{
    wifi_event_group = xEventGroupCreateStatic(&wifi_event_group_buffer);
    connectToWifi();
}

DataExchanger::~DataExchanger() = default;

void DataExchanger::connectToWifi()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK(err);
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK(err);
    }

    if (sta_netif == nullptr)
    {
        sta_netif = esp_netif_create_default_wifi_sta();
    }

    if (!wifi_driver_initialized)
    {
        wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
        err = esp_wifi_init(&wifi_initiation);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        {
            ESP_ERROR_CHECK(err);
        }
        wifi_driver_initialized = true;
    }
    if (!wifi_handlers_registered)
    {
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifiEventHandler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifiEventHandler, NULL));
        wifi_handlers_registered = true;
    }

    wifi_config_t wifi_configuration = {};
    strncpy((char *)wifi_configuration.sta.ssid,
            ssid.c_str(), sizeof(wifi_configuration.sta.ssid));
    strncpy((char *)wifi_configuration.sta.password,
            password.c_str(), sizeof(wifi_configuration.sta.password));

    wifi_configuration.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_configuration.sta.pmf_cfg.capable = true;
    wifi_configuration.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_configuration));
    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN)
    {
        ESP_ERROR_CHECK(err);
    }

    err = esp_wifi_connect();
    if (err != ESP_OK && err != ESP_ERR_WIFI_CONN)
    {
        ESP_ERROR_CHECK(err);
    }

    EventBits_t wifi_bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));

    if ((wifi_bits & WIFI_CONNECTED_BIT) == 0)
    {
        ESP_LOGW(TAG, "WiFi connect timeout. Ensure SSID is 2.4GHz and credentials are correct.");
    }
};

void DataExchanger::wifiEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    {
        printf("WiFi connected\n");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        printf("WiFi disconnected, retrying...\n");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        printf("Got IP, ready\n");
    }
}

int DataExchanger::createSocketAndConnect()
{
    struct sockaddr_in dest_addr{};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);

    if (inet_pton(AF_INET, ip.c_str(), &dest_addr.sin_addr) <= 0)
    {
        printf("Invalid Ip");
        return -1;
    }

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Socket creation failed: errno %d", errno);
        return -1;
    }

    int flag = 1;
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

    timeval timeout{};
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    printf("Connecting to %s:%d", ip.c_str(), port);

    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0)
    {
        ESP_LOGW(TAG, "Connect failed: errno %d", errno);
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "Connected!");
    return sock;
};

void DataExchanger::startTcpClient()
{
    while (true)
    {
        sock = createSocketAndConnect();

        if (sock < 0)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            printf("Couldn't connect to a socket\n");
            continue;
        }
        while (true)
        {
            printf("waiting\n");
            // vTaskDelay(pdMS_TO_TICKS(2000));
            for (std::size_t i = 0; i < sendingVector.size(); i++)
            {
                sendData(sendingVector[i].first, sendingVector[i].second);
            }
            if (!receiveData())
            {
                close(sock);
                sock = -1;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void DataExchanger::appendToSending(std::function<int()> method, uint8_t *bufferPointer)
{
    sendingVector.push_back({method, bufferPointer});
}

void DataExchanger::appendToReceiving(std::function<void()> method, uint8_t *bufferPointer, std::size_t bufferSize)
{
    receivingVector.push_back({method, bufferPointer, bufferSize, 0});
}

void DataExchanger::sendData(std::function<int()> method, uint8_t *bufferPointer)
{
    int length = method();
    if (length < 0)
    {
        printf("Length < 0\n");
        abort();
        // throw
    }
    else if (length == 0)
    {
        printf("No data to send\n");
        // no data to send
    }
    else
    {
        int sent = send(sock, bufferPointer, length, 0);
        if (sent < 0)
        {
            printf("error sent < 0");
            // throw
        }
        else
        {
            printf("%d bytes of data  succeesfully sent!", sent);
        }
    }
}

bool DataExchanger::receiveData()
{
    for (auto &entry : receivingVector)
    {
        const std::size_t remainingBytes = entry.bufferSize - entry.bytesReceived;
        const int received = recv(sock, entry.bufferPointer + entry.bytesReceived, remainingBytes, MSG_DONTWAIT);

        if (received > 0)
        {
            entry.bytesReceived += static_cast<std::size_t>(received);
            if (entry.bytesReceived == entry.bufferSize)
            {
                entry.method();
                entry.bytesReceived = 0;
            }
        }
        else if (received == 0)
        {
            ESP_LOGW(TAG, "Socket closed by peer");
            return false;
        }
        else if (errno != EAGAIN && errno != EWOULDBLOCK)
        {
            ESP_LOGW(TAG, "Receive failed: errno %d", errno);
            return false;
        }
    }

    return true;
}
